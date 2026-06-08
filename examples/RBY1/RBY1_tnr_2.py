"""
RBY1_tnr_2.py — Teach-and-Replay + AI skill execution for RB-Y1
================================================================

Self-contained: all dependencies live inside lilio_think.
No lilio_move required.

Architecture (simplified vs. OpenArm)
--------------------------------------
OA required a 50 Hz background thread for: CAN watchdog keep-alive,
real-time IK solving, and gravity-torque feedforward.

RBY1 needs none of that:
  - CartesianCommandBuilder with a long control_hold_time holds position
    indefinitely without repeated commands.
  - The robot resolves IK internally.
  - Gravity compensation during hand-guide is handled by the hardware button.

The only threads here are unavoidable ones:
  - pynput keyboard listener (library requirement)
  - rby1_sdk state callback (SDK requirement)

Recording happens directly in the state callback (fast: just appends
state.position).  FK is computed on demand in the main thread during
post-processing.

SDK calls (all verified against Rainbow Robotics examples):
  rby1_sdk v1.x — 01_hello_rby1.py, 03_robot_state.py, 21_record.py,
                  22_replay.py, 32_command_stream.py, 33_cartesian_command_stream.py

Recording
---------
The robot is always in a back-drivable state (hardware gravity compensation
is always on). Press [R] to start recording joint positions; press [S] to
stop and post-process into Cartesian waypoints.

Keyboard controls
-----------------
  r  — start recording
  s  — stop recording  (post-processes trajectory + freezes pose)
  c  — close gripper  (active arms only)
  v  — open  gripper  (active arms only)
  a  — toggle active arm: left <-> right
  o  — capture ROI   (register the target object for AI)
  d  — save skill demo
  e  — execute skill  (AI inference -> Cartesian replay)
  p  — replay last recorded trajectory (no AI)
  q  — quit

Usage (full AI mode)
--------------------
    python RBY1_tnr_2.py \\
        --address 192.168.30.1:50051 \\
        --config_vision ../config_vision_QM.json \\
        --config_robot  ../robot_config.json \\
        --skills_folder skills_lib/

Usage (simulation — no hardware required)
-----------------------------------------
    python RBY1_tnr_2.py --sim --address dummy:50051
"""

from __future__ import annotations

import os
import sys

# ── Simulation stub injection ─────────────────────────────────────────────────
# Must happen before rby1_sdk is imported anywhere in the module.
if "--sim" in sys.argv:
    # rby1_sdk_stub.py lives in the same directory as this script.
    import importlib.util as _ilu
    _stub_path = os.path.join(os.path.dirname(__file__), "rby1_sdk_stub.py")
    _spec  = _ilu.spec_from_file_location("rby1_sdk", _stub_path)
    _stub  = _ilu.module_from_spec(_spec)
    _spec.loader.exec_module(_stub)
    sys.modules["rby1_sdk"] = _stub
    print("[SIM] rby1_sdk stubbed.")

import time
import queue
import threading
from typing import Optional

import numpy as np
import rby1_sdk as rby
from pynput import keyboard as _kb

# Local helpers — no lilio_move dependency
sys.path.insert(0, os.path.dirname(__file__))
from lilio_think.example.utils import capture_roi, save_skill_demo, stream_action_plans


# ── Constants ─────────────────────────────────────────────────────────────────

# Link names — must match the robot URDF
_BASE = "base"
_EE_R = "ee_right"
_EE_L = "ee_left"

# CartesianCommandBuilder (confirmed from 33_cartesian_command_stream.py)
_CART_POS_TOL   = 0.3    # position tolerance (m)
_CART_STIFFNESS = 100.0
_CART_DAMPING   = 0.8

# Hold duration sent after exiting hand-guide (s).
# Long enough that the robot stays put; a new command overrides it.
_HOLD_TIME = 3600.0

# Convergence threshold for the initial move-to-first-waypoint (m)
_CONVERGE_THRESHOLD = 0.01   # 1 cm — matches 33_cartesian_command_stream.py

# Gripper step per keypress (normalised 0 = open, 1 = closed)
_GRIPPER_STEP = 0.1


# ── Helpers ───────────────────────────────────────────────────────────────────

def movej(robot, torso=None, right_arm=None, left_arm=None,
          minimum_time: float = 5.0) -> bool:
    """Blocking joint-space move (mirrors 00_helper.py from rby1-sdk examples).

    Use this before RBY1Controller.run() to bring the robot to a known safe
    configuration, since CartesianCommandBuilder works best from a well-defined
    starting pose.

    Args:
        robot        : connected rby1_sdk robot instance
        torso        : np.ndarray of torso joint angles (rad), or None to skip
        right_arm    : np.ndarray of right arm joint angles (rad), or None
        left_arm     : np.ndarray of left arm joint angles (rad), or None
        minimum_time : motion duration lower bound (s)

    Returns True on success, False on failure.
    """
    rc = rby.BodyComponentBasedCommandBuilder()
    if torso is not None:
        rc.set_torso_command(
            rby.JointPositionCommandBuilder()
            .set_minimum_time(minimum_time)
            .set_position(torso)
        )
    if right_arm is not None:
        rc.set_right_arm_command(
            rby.JointPositionCommandBuilder()
            .set_minimum_time(minimum_time)
            .set_position(right_arm)
        )
    if left_arm is not None:
        rc.set_left_arm_command(
            rby.JointPositionCommandBuilder()
            .set_minimum_time(minimum_time)
            .set_position(left_arm)
        )
    rv = robot.send_command(
        rby.RobotCommandBuilder().set_command(
            rby.ComponentBasedCommandBuilder().set_body_command(rc)
        ),
        1,
    ).get()
    if rv.finish_code != rby.RobotCommandFeedback.FinishCode.Ok:
        print("[movej] Failed.")
        return False
    return True


# ── Gripper ───────────────────────────────────────────────────────────────────

class Gripper:
    """Dual gripper via DynamixelBus.

    Confirmed from 35_leader_arm_teleop_with_monitor.py:
      bus    : rby.DynamixelBus(rby.upc.GripperDeviceName) at 2 Mbaud
      ID 0   : right gripper
      ID 1   : left gripper
      target : normalised 0.0 (open) -> 1.0 (closed)

    Unlike the leader-arm example we do NOT run a background loop — the
    Dynamixel holds its last commanded position in current-based position
    mode, so we only need to send a command when the target changes.
    """
    _RIGHT_ID = 0
    _LEFT_ID  = 1
    _BAUD     = 2_000_000

    def __init__(self):
        self.bus = rby.DynamixelBus(rby.upc.GripperDeviceName)
        self.bus.open_port()
        self.bus.set_baud_rate(self._BAUD)

        self.min_q   = np.zeros(2)        # encoder counts — fully closed
        self.max_q   = np.zeros(2)        # encoder counts — fully open
        self._target = np.zeros(2)        # [right, left] normalised
        self._ready  = False

    def initialize(self) -> bool:
        """Ping devices and enable torque. Returns True if both respond."""
        for dev_id in (self._RIGHT_ID, self._LEFT_ID):
            if not self.bus.ping(dev_id):
                print(f"[GRIPPER] Device {dev_id} not responding.")
                return False
        self.bus.torque_enable([self._RIGHT_ID, self._LEFT_ID], [1, 1])
        return True

    def home(self):
        """Find mechanical limits via opposing torques.

        Full procedure from 35_leader_arm_teleop_with_monitor.py:
          1. Switch to current-control mode
          2. Push closed -> record min_q over 30 static readings
          3. Push open   -> record max_q over 30 static readings
          4. Switch to current-based position-control (5 A limit)

        TODO: implement once bus current-control API is confirmed.
        """
        print("[GRIPPER] Homing — TODO: implement from example 35.")
        self.min_q  = np.array([0.0,    0.0])
        self.max_q  = np.array([4096.0, 4096.0])
        self._ready = True

    def set_target(self, right: float, left: float):
        """Set normalised target and immediately send the position command."""
        self._target = np.clip([right, left], 0.0, 1.0)
        if self._ready:
            q = self._target * (self.max_q - self.min_q) + self.min_q
            self.bus.group_sync_write_send_position(
                [(self._RIGHT_ID, q[0]), (self._LEFT_ID, q[1])]
            )

    def get_target(self) -> tuple:
        """Return current (right, left) normalised targets."""
        return float(self._target[0]), float(self._target[1])

    def close(self):
        """Disable torque on shutdown."""
        self.bus.torque_enable([self._RIGHT_ID, self._LEFT_ID], [0, 0])


# ── Controller ────────────────────────────────────────────────────────────────

class RBY1Controller:
    def __init__(self, arms: list, *,
                 address: str, model: str = "a",
                 camera=None, lip=None, stereo_depth=None):
        """
        Parameters
        ----------
        arms         : arms to teach/replay, e.g. ["left"], ["right"], ["left","right"]
        address      : robot gRPC address  e.g. "192.168.30.1:50051"
        model        : robot model string  "a" | "m" | "ub"  (default "a")
        camera       : ZEDMiniCamera or compatible (optional — required for AI)
        lip          : LIP instance for AI skill execution (optional)
        stereo_depth : S2M2ONNX instance for depth from stereo (optional — required for AI)
        """
        self.dt           = 0.02   # replay step period (50 Hz recording rate)
        self.arms         = arms
        self.lip          = lip
        self.camera       = camera
        self.stereo_depth = stereo_depth

        # ── Connect ───────────────────────────────────────────────────────────
        self.robot = rby.create_robot(address, model)
        if not self.robot.connect():
            print("Failed to connect to the robot.")
            sys.exit(1)

        if not self.robot.is_power_on(".*"):
            if not self.robot.power_on(".*"):
                print("Failed to power on."); sys.exit(1)

        if not self.robot.is_servo_on(".*"):
            if not self.robot.servo_on(".*"):
                print("Failed to enable servos."); sys.exit(1)

        cm = self.robot.get_control_manager_state()
        if cm.state == rby.ControlManagerState.FAULT:
            self.robot.reset_fault_control_manager()
        self.robot.enable_control_manager()
        print("Robot: connected and enabled.")

        # ── Model metadata ────────────────────────────────────────────────────
        self.model_info = self.robot.model()

        # ── Dynamics for FK ───────────────────────────────────────────────────
        self.dyn       = self.robot.get_dynamics()
        self.dyn_state = self.dyn.make_state(
            [_BASE, _EE_R, _EE_L],
            self.model_info.robot_joint_names,
        )
        self._BASE_IDX = 0
        self._EE_R_IDX = 1
        self._EE_L_IDX = 2
        self._dyn_lock = threading.Lock()

        # ── Asynchronous state (SDK callback thread) ──────────────────────────
        self._q_lock   = threading.Lock()
        self.q_state   = np.zeros(len(self.model_info.robot_joint_names))
        self.recording         = False
        self.raw_samples: list = []

        def _state_cb(state):
            with self._q_lock:
                self.q_state = np.asarray(state.position)
            if self.recording:
                gl, gr = self._gl_gr
                self.raw_samples.append((np.asarray(state.position).copy(), gl, gr))

        self.robot.start_state_update(_state_cb, 50)
        time.sleep(0.2)

        # ── Gripper ───────────────────────────────────────────────────────────
        self.gripper: Optional[Gripper] = None
        try:
            g = Gripper()
            if g.initialize():
                g.home()
                self.gripper = g
                print("Gripper: ready!")
            else:
                print("[GRIPPER] Not available — continuing without it.")
        except Exception as e:
            print(f"[GRIPPER] Init failed ({e}) — continuing without it.")

        # ── Command stream ────────────────────────────────────────────────────
        self.stream = self.robot.create_command_stream()

        self._gl, self._gr = 0.0, 0.0

        # ── Session state ─────────────────────────────────────────────────────
        self.traj:        list = []
        self._plan_queue: queue.Queue = queue.Queue()
        self.stop_program      = False
        self._replaying        = False

        # ── Deferred UI actions ───────────────────────────────────────────────
        self._roi_captured        = False
        self._demo_state          = None
        self._roi_requested       = False
        self._save_demo_requested = False
        self._execute_requested   = False
        self._inputting           = False
        self._pending_skill_name  = None
        self._pending_guideline   = None
        self._skill_chain:  list  = []

        # ── Keyboard listener ─────────────────────────────────────────────────
        self.listener = _kb.Listener(on_press=self.on_press)
        self.listener.start()
        print(f"Active arms: {self.arms}  —  [A] to cycle")
        print("Keys: [R] rec  [S] stop  [P] replay  "
              "[O] ROI  [D] save  [E] exec  [C/V] gripper  [Q] quit")
        if self.lip is None:
            print("[WARN] LIP not initialised — AI features (O/D/E) unavailable.")

    # ── Helpers ───────────────────────────────────────────────────────────────

    @property
    def _gl_gr(self) -> tuple:
        return self._gl, self._gr

    def _fk(self, q: Optional[np.ndarray] = None) -> tuple:
        """Return (T_left 4x4, T_right 4x4) in base frame."""
        with self._q_lock:
            q_snap = np.asarray(self.q_state if q is None else q)
        with self._dyn_lock:
            self.dyn_state.set_q(q_snap)
            self.dyn.compute_forward_kinematics(self.dyn_state)
            T_R = self.dyn.compute_transformation(
                self.dyn_state, self._BASE_IDX, self._EE_R_IDX)
            T_L = self.dyn.compute_transformation(
                self.dyn_state, self._BASE_IDX, self._EE_L_IDX)
        return T_L, T_R

    def _cartesian_cmd(self, TL: np.ndarray, TR: np.ndarray,
                       arms: list, minimum_time: float,
                       hold_time: float = 1.0) -> rby.RobotCommand:
        """Build a Cartesian command for the specified arms only."""
        def _arm_cmd(ee_link, T):
            return (
                rby.CartesianCommandBuilder()
                .set_command_header(
                    rby.CommandHeaderBuilder().set_control_hold_time(hold_time)
                )
                .add_target(_BASE, ee_link, T,
                            _CART_POS_TOL, _CART_STIFFNESS, _CART_DAMPING)
                .set_minimum_time(minimum_time)
            )

        body = rby.BodyComponentBasedCommandBuilder()
        if "right" in arms:
            body = body.set_right_arm_command(_arm_cmd(_EE_R, TR))
        if "left" in arms:
            body = body.set_left_arm_command(_arm_cmd(_EE_L, TL))
        return rby.RobotCommandBuilder().set_command(
            rby.ComponentBasedCommandBuilder().set_body_command(body)
        )

    def _send_hold(self):
        """Freeze the robot at its current pose for _HOLD_TIME seconds."""
        T_L, T_R = self._fk()
        self.stream.send_command(
            self._cartesian_cmd(T_L, T_R, ["left", "right"],
                                minimum_time=0.5, hold_time=_HOLD_TIME)
        )

    # ── Keyboard handler ──────────────────────────────────────────────────────

    def on_press(self, key):
        if self._inputting:
            return
        try:
            c = key.char
        except Exception:
            return

        if c == 'r':
            self.raw_samples = []
            self.recording   = True
            print("\n>> RECORDING...")

        elif c == 's':
            self.recording = False
            if self.raw_samples:
                print("Post-processing trajectory...")
                self.traj = []
                for q_snap, gl, gr in self.raw_samples:
                    TL, TR = self._fk(q_snap)
                    self.traj.append({
                        "arms": self.arms,
                        "TL": TL, "TR": TR,
                        "gl": gl, "gr": gr,
                    })
            print(f"\n>> STOPPED. {len(self.raw_samples)} samples -> "
                  f"{len(self.traj)} waypoints.")
            self.raw_samples = []

        elif c == 'c' and self.gripper is not None:
            self._gl = min(self._gl + _GRIPPER_STEP, 1.0) if "left"  in self.arms else self._gl
            self._gr = min(self._gr + _GRIPPER_STEP, 1.0) if "right" in self.arms else self._gr
            self.gripper.set_target(right=self._gr, left=self._gl)
        elif c == 'v' and self.gripper is not None:
            self._gl = max(self._gl - _GRIPPER_STEP, 0.0) if "left"  in self.arms else self._gl
            self._gr = max(self._gr - _GRIPPER_STEP, 0.0) if "right" in self.arms else self._gr
            self.gripper.set_target(right=self._gr, left=self._gl)

        elif c == 'a':
            self.arms = ["right"] if "left" in self.arms else ["left"]
            print(f"\n>> Active arms: {self.arms}")
        elif c == 'o':
            self._roi_requested = True
        elif c == 'd':
            self._save_demo_requested = True
        elif c == 'e':
            self._execute_requested = True
        elif c == 'p':
            if self.traj:
                self._plan_queue.put(self.traj)
        elif c == 'q':
            self.stop_program = True

    # ── Feedback / convergence ────────────────────────────────────────────────

    def _wait_converge(self, timeout: float = 10.0):
        """Block until bimanual position error < _CONVERGE_THRESHOLD or timeout."""
        t_end = time.time() + timeout
        while time.time() < t_end:
            fb = self.stream.request_feedback()
            try:
                body = (fb.component_based_command
                          .body_command
                          .body_component_based_command)
                err_r = (body.right_arm_command.cartesian_command
                             .se3_pose_tracking_errors[0].position_error)
                err_l = (body.left_arm_command.cartesian_command
                             .se3_pose_tracking_errors[0].position_error)
                if max(err_r, err_l) < _CONVERGE_THRESHOLD:
                    return
            except Exception:
                pass
            time.sleep(0.01)

    # ── Plan replay ───────────────────────────────────────────────────────────

    def _replay_plan(self, plan):
        """Replay a trajectory of Cartesian waypoints."""
        self._replaying = True
        traj = [{"arms": p["arms"],
                 "TL":   np.array(p["TL"]),
                 "TR":   np.array(p["TR"]),
                 "gl":   p["gl"],
                 "gr":   p["gr"]} for p in plan]
        t0   = traj[0]
        arms = t0["arms"]
        print(f"[REPLAY] {len(traj)} steps — arms: {arms}")

        init_L, init_R = self._fk()

        # Move active arm(s) to first waypoint; block until converged (up to 10 s)
        target_L = t0["TL"] if "left"  in arms else init_L
        target_R = t0["TR"] if "right" in arms else init_R
        self.stream.send_command(
            self._cartesian_cmd(target_L, target_R, ["left", "right"],
                                minimum_time=3.0, hold_time=1.0)
        )
        self._wait_converge(timeout=10.0)

        # Set gripper to first-waypoint state
        self._gl, self._gr = t0["gl"], t0["gr"]
        if self.gripper is not None:
            self.gripper.set_target(right=t0["gr"], left=t0["gl"])

        # Stream remaining waypoints at recording rate
        for step in traj[1:]:
            if self.stop_program:
                break
            step_L = step["TL"] if "left"  in step["arms"] else init_L
            step_R = step["TR"] if "right" in step["arms"] else init_R
            self.stream.send_command(
                self._cartesian_cmd(step_L, step_R, ["left", "right"],
                                    minimum_time=self.dt, hold_time=1.0)
            )
            if self.gripper is not None:
                self.gripper.set_target(right=step["gr"], left=step["gl"])
            self._gl, self._gr = step["gl"], step["gr"]
            time.sleep(self.dt)

        self._replaying = False
        print("\nReplay done.\n")

    # ── Main loop ─────────────────────────────────────────────────────────────

    def run(self):
        """Block until stop_program is set."""
        while not self.stop_program:

            if self._roi_requested:
                self._roi_requested = False
                bbox, img_left, img_right, depth = capture_roi(
                    self.stereo_depth, camera=self.camera
                )
                if bbox is not None and self.lip is not None:
                    self._demo_state = self.lip.get_demo_state(
                        img=img_left, depth=depth, normal=None,
                        user_roi=bbox, clip_distance=1.5, show_viz=False,
                    )
                    self._roi_captured = True

            if self._save_demo_requested:
                self._save_demo_requested = False
                self._inputting = True
                name  = self._pending_skill_name
                guide = self._pending_guideline
                self._pending_skill_name = self._pending_guideline = None
                save_skill_demo(self.traj, self.lip, demo_state=self._demo_state,
                                skill_name=name, guideline=guide)
                self._inputting = False

            if self._execute_requested:
                self._execute_requested = False
                self._inputting = True
                skill_name = (self._pending_skill_name
                              or input("Skill name to execute: ").strip())
                self._pending_skill_name = self._pending_guideline = None
                self._inputting = False
                if skill_name:
                    while not self._plan_queue.empty():
                        self._plan_queue.get_nowait()
                    image_l, image_r = self.camera.get_frames()
                    if image_l is not None:
                        print(f"[SKILL] Fetching plan for '{skill_name}'...")
                        stream_action_plans(
                            image_l, image_r, skill_name,
                            self._plan_queue,
                            lip=self.lip,
                            stereo_depth=self.stereo_depth,
                        )
                    else:
                        print("[SKILL] Could not grab camera frame.")

            if (self._skill_chain
                    and not self._execute_requested
                    and self._plan_queue.empty()):
                self._pending_skill_name = self._skill_chain.pop(0)
                self._execute_requested  = True

            if not self._plan_queue.empty() and not self._replaying:
                self._replay_plan(self._plan_queue.get_nowait())

            time.sleep(0.05)

    # ── Shutdown ──────────────────────────────────────────────────────────────

    def disconnect(self):
        self.stop_program = True
        if hasattr(self, 'listener'):
            self.listener.stop()
        if hasattr(self, 'stream'):
            self.stream.cancel()
        if hasattr(self, 'robot'):
            self.robot.disable_control_manager()
        if self.gripper is not None:
            self.gripper.close()
        if self.camera is not None:
            self.camera.close_cam()
        print("Robot disconnected.")


# ── Standalone entry point ────────────────────────────────────────────────────

if __name__ == "__main__":
    import argparse
    import json

    parser = argparse.ArgumentParser(description="RBY1 Teach-and-Replay")
    parser.add_argument("--address",       type=str, required=True,
                        help="Robot gRPC address, e.g. 192.168.30.1:50051")
    parser.add_argument("--model",         type=str, default="a",
                        help="Robot model: 'a' | 'm' | 'ub'  (default: 'a')")
    parser.add_argument("--arms",          type=str, default="left,right",
                        help="Active arms: 'left', 'right', or 'left,right'")
    parser.add_argument("--sim",           action="store_true",
                        help="Run without hardware (stub SDK + no camera)")
    parser.add_argument("--config_vision", type=str, default="../config_vision_QM.json",
                        help="Path to camera calibration JSON "
                             "(default: ../config_vision_QM.json)")
    parser.add_argument("--config_robot",  type=str, default="../robot_config.json",
                        help="Path to robot calibration JSON "
                             "(default: ../robot_config.json)")
    parser.add_argument("--skills_folder", type=str, default="skills_lib/",
                        help="Directory where skills are stored "
                             "(created automatically if absent)")
    args = parser.parse_args()

    # ── Camera ────────────────────────────────────────────────────────────────
    camera = None
    if not args.sim:
        from lilio_think.sensors.zed_camera import ZEDMiniCamera
        import pyzed.sl as sl
        camera = ZEDMiniCamera(resolution=sl.RESOLUTION.HD720, fps=30)
        if not camera.open_cam():
            sys.exit(1)
        camera.get_calibration()
        print("ZED camera: ready!")

    # ── Vision models (LIP + S2M2 stereo depth) ───────────────────────────────
    # AI features (O / D / E keys) require lilio_see and valid config files.
    # In --sim mode these are skipped.
    stereo_depth = None
    lip          = None
    if not args.sim:
        try:
            import lilio_see
            from lilio_think.src.Imitation_Pipeline import LIP
            from lilio_see.Utils.vision_2D import read_camera_calib_json
            from lilio_see.Model.S2M2.S2M2_onnx import S2M2ONNX

            with open(args.config_vision) as _f:
                _vis_cfg = json.load(_f)

            stereo_depth = S2M2ONNX(
                s2m2_onnx_path=lilio_see.get_model_paths()["s2m2_onnx_path"],
                camera_calibration=read_camera_calib_json(
                    _vis_cfg["camera_calibration"]
                ),
            )
            lip = LIP(
                vision_config_path=args.config_vision,
                robot_config_path=args.config_robot,
                skills_library_path=args.skills_folder,
            )
            print("LIP + S2M2: ready!")
        except Exception as _e:
            print(f"[WARN] Could not initialise AI pipeline: {_e}")
            print("[WARN] AI features (O/D/E) will be unavailable.")

    # ── Controller ────────────────────────────────────────────────────────────
    ctrl = RBY1Controller(
        arms=args.arms.split(","),
        address=args.address,
        model=args.model,
        camera=camera,
        lip=lip,
        stereo_depth=stereo_depth,
    )
    try:
        ctrl.run()
    finally:
        ctrl.disconnect()
