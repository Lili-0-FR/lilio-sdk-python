"""
OpenArm Teach-and-Replay — powered by the Lilio SDK
=====================================================

Keyboard controls
-----------------
  H   Hand-guide mode  (arm goes compliant)
  R   Start recording  (hand-guide mode only)
  S   Stop  recording  (arm goes stiff)
  P   Replay last recorded trajectory
  O   Capture ROI  (point camera at target object, then draw box)
  D   Save trajectory as a named skill
  E   Execute a skill from the server
  A   Switch active arm  (left ↔ right, stiff mode only)
  Q   Quit

Usage
-----
    python oa_tnr_sdk.py \\
        --arm right \\
        --api-key lilio_sk_... \\
        --host http://<robot-ip>:8000 \\
        --camera-config /path/to/config_vision_QM.json
"""
import json
import os
import sys
import time
import queue
import threading
import collections
import datetime
import argparse

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from loop_rate_limiters import RateLimiter
from pynput import keyboard

from lilio import LilioClient

from device_server import DeviceServer
from config import (
    PLACO_URDF, COLLISION_PATH, OA_CONFIG, VELOCITY_LIMIT, POSTURE_HOME,
    LEFT_JOINTS, RIGHT_JOINTS, GRIPPER_MIN, GRIPPER_MAX,
)
from OA_interface import OpenArmInterface
from OA_placo import OpenArmPlaco
from zed_camera import ZEDMiniCamera
from oa_utils import (
    capture_roi, save_skill_demo, stream_action_plans,
    move_to_cart_threaded, plot_tracking,
)

import pyzed.sl as sl


class OpenArmController:
    def __init__(self, arms: list, session):
        self.dt         = 0.02
        self._ctrl_rate = RateLimiter(frequency=1 / self.dt)
        self.arms       = arms
        self.session    = session

        # ── Hardware ──────────────────────────────────────────────────────────
        self.OAI = OpenArmInterface(read_timeout=1000)
        self.OAI.start()

        # ── IK pipeline ───────────────────────────────────────────────────────
        self.home_pose_joint = [0.0, 0, 0, 0, 0, 0, 1]
        self.OAP = OpenArmPlaco(urdf_path=PLACO_URDF,
                                collision_path=COLLISION_PATH,
                                config=OA_CONFIG,
                                posture_target=POSTURE_HOME,
                                velocity_limit=VELOCITY_LIMIT)
        self.OAP_check = OpenArmPlaco(urdf_path=PLACO_URDF,
                                      collision_path=COLLISION_PATH,
                                      config=OA_CONFIG,
                                      posture_target=POSTURE_HOME,
                                      velocity_limit=VELOCITY_LIMIT)
        for key, value in POSTURE_HOME.items():
            self.home_pose_joint.append(value)
            self.OAP.robot.set_joint(key, value)
        self.home_pose_joint = np.array(self.home_pose_joint)
        self.OAP.robot.update_kinematics()
        self.OAI.move_to_pose(self.home_pose_joint, 0.0, 0.0, 4,
                              robot_model=self.OAP, plot=True)
        print("Placo Inverse Kinematics: ready !")

        # ── Shared control state ───────────────────────────────────────────────
        obs = self.OAI.get_state()
        self.q          = obs["q_state"].copy()
        T_L, T_R        = self.OAP.get_ee_pose(self.q)
        self._cmd       = (T_L.copy(), T_R.copy(), obs["gl"], obs["gr"])
        self._cmd_lock  = threading.Lock()
        self._last_ee_L = T_L.copy()
        self._last_ee_R = T_R.copy()
        self._soft_mode = False
        self._prev_soft = False
        self._ctrl_tick = threading.Event()
        self.qot_scale  = 0
        self._inactive_hold_q = None

        # ── State flags ───────────────────────────────────────────────────────
        self.hand_guide           = False
        self.recording            = False
        self.raw_samples: list    = []
        self.traj:        list    = []
        self._plan_queue: queue.Queue = queue.Queue()
        self.stop_program         = False

        self._roi_captured        = False
        self._roi_crop_jpg        = None
        self._roi_requested       = False
        self._save_demo_requested = False
        self._execute_requested   = False
        self._inputting           = False
        self._pending_skill_name  = None
        self._pending_guideline   = None
        self._skill_chain:  list  = []
        self._replaying           = False

        # ── Joint logging ─────────────────────────────────────────────────────
        self._joint_buffer   = collections.deque(maxlen=400)
        self._plot_requested = False

        # ── Camera ────────────────────────────────────────────────────────────
        self.camera = ZEDMiniCamera(resolution=sl.RESOLUTION.HD720, fps=30)
        self.camera.open_cam()
        self.camera.get_calibration()
        print("ZED camera: ready !")

        # ── Background control thread ──────────────────────────────────────────
        self._ctrl_thread = threading.Thread(target=self._control_loop, daemon=True)
        self._ctrl_thread.start()

        # ── Keyboard listener ──────────────────────────────────────────────────
        self.listener = keyboard.Listener(on_press=self.on_press)
        self.listener.start()
        print("Controller: ready !")

    # ── Control loop (background thread) ───────────────────────────────────────

    def _control_loop(self):
        t_log, q_des_log, q_act_log = [], [], []
        t     = 0
        q_des = np.zeros(25)

        while not self.stop_program:
            obs      = self.OAI.get_state()
            tau_grav = self.OAP.get_gravity_torques(obs["q_state"])
            tau      = np.zeros(25)
            tau[7:14]  = tau_grav[6:13]
            tau[16:23] = tau_grav[15:22]

            if self._soft_mode:
                self._prev_soft = True
                with self._cmd_lock:
                    tl, tr, gl, gr = self._cmd

                self.OAI.send_action_soft(arms=self.arms, q_placo=obs["q_state"],
                                          gl=gl, gr=gr, tau=tau)
                # Keep inactive arm stiff to avoid CAN watchdog timeout
                inactive = [a for a in ["left", "right"] if a not in self.arms]
                with self._cmd_lock:
                    hold_q = self._inactive_hold_q.copy() if self._inactive_hold_q is not None else None
                if inactive and hold_q is not None:
                    self.OAI.send_action(arms=inactive, q_placo=hold_q,
                                         gl=gl, gr=gr, tau=tau)

                T_L, T_R = self.OAP.get_ee_pose(obs["q_state"])
                if self.recording:
                    self.raw_samples.append(obs)
                with self._cmd_lock:
                    self._cmd = (T_L, T_R, obs["gl"], obs["gr"])
                # Keep IK warm so transition to stiff is seamless
                warm = {name: obs["q_state"][7 + i]  for i, name in enumerate(LEFT_JOINTS)}
                warm.update({name: obs["q_state"][16 + i] for i, name in enumerate(RIGHT_JOINTS)})
                self.OAP.robot.q = warm
            else:
                with self._cmd_lock:
                    tL, tR, gl, gr = self._cmd
                if self._prev_soft and self.qot_scale < 1:
                    self.qot_scale += 0.2
                else:
                    self._prev_soft = False
                    self.qot_scale  = 0
                self.q = self.OAP.IK_solve(obs["q_state"], tL, tR, False)
                self.OAI.send_action(arms=["left", "right"], q_placo=self.q,
                                     gl=gl, gr=gr, tau=tau,
                                     kd_qot=self.qot_scale, kp_qot=self.qot_scale)
                T_L, T_R = self.OAP.get_ee_pose(self.q)
                q_des = self.q.copy()
                t_log.append(t); q_des_log.append(q_des); q_act_log.append(obs["q_state"].copy())

            self._joint_buffer.append((t, obs["q_state"].copy(), q_des, self._soft_mode))
            with self._cmd_lock:
                self._last_ee_L = T_L
                self._last_ee_R = T_R
            self._ctrl_rate.sleep()
            self._ctrl_tick.set()
            t += self.dt

        if t_log:
            plot_tracking(np.array(t_log), np.array(q_des_log),
                          np.array(q_act_log), "full_traceback")

    # ── Keyboard ───────────────────────────────────────────────────────────────

    def on_press(self, key):
        if self._inputting:
            return
        try:
            c = key.char
        except Exception:
            return

        if self._soft_mode:
            if c == 'r':
                self.raw_samples = []
                self.recording   = True
                print("\n>> RECORDING...")
            elif c == 's':
                self.recording = False
                if self.raw_samples:
                    print("Post-processing trajectory...")
                    self.traj = []
                    for obs in self.raw_samples:
                        TL, TR = self.OAP_check.get_ee_pose(obs["q_state"])
                        self.traj.append({"arms": self.arms, "TL": TL, "TR": TR,
                                          "gl": obs["gl"], "gr": obs["gr"]})
                self._prev_soft = True
                self.OAI.reset_integral(left=("left" in self.arms),
                                        right=("right" in self.arms))
                with self._cmd_lock:
                    self._inactive_hold_q = None
                self._soft_mode = False
                print(f"\n>> STOPPED. {len(self.raw_samples)} samples recorded.")
            elif c == 'c':
                if self._ctrl_tick.wait(timeout=0.5):
                    self._ctrl_tick.clear()
                    with self._cmd_lock:
                        cur_TL, cur_TR, cur_gl, cur_gr = self._cmd
                        new_gl = float(np.clip(cur_gl + 0.6, GRIPPER_MIN, GRIPPER_MAX)) if "left"  in self.arms else cur_gl
                        new_gr = float(np.clip(cur_gr + 0.6, GRIPPER_MIN, GRIPPER_MAX)) if "right" in self.arms else cur_gr
                        self._cmd = (cur_TL, cur_TR, new_gl, new_gr)
            elif c == 'v':
                if self._ctrl_tick.wait(timeout=0.5):
                    self._ctrl_tick.clear()
                    with self._cmd_lock:
                        cur_TL, cur_TR, cur_gl, cur_gr = self._cmd
                        new_gl = float(np.clip(cur_gl - 0.6, GRIPPER_MIN, GRIPPER_MAX)) if "left"  in self.arms else cur_gl
                        new_gr = float(np.clip(cur_gr - 0.6, GRIPPER_MIN, GRIPPER_MAX)) if "right" in self.arms else cur_gr
                        self._cmd = (cur_TL, cur_TR, new_gl, new_gr)

        if c == 'h':
            self.hand_guide = True
            self._soft_mode = True
            with self._cmd_lock:
                self._inactive_hold_q = self.OAI.get_state()["q_state"].copy()
            self.OAI.set_soft(arms=self.arms)
            print("\n>> HAND GUIDE mode. Press [R] to record, [S] to stop.")
        elif c == 'o':
            self._roi_requested = True
        elif c == 'd':
            self._save_demo_requested = True
        elif c == 'e':
            self._execute_requested = True
        elif c == 'a':
            if not self._soft_mode:
                self.arms = ["right"] if self.arms == ["left"] else ["left"]
                print(f"\n>> Active arms: {self.arms}")
        elif c == 'p':
            if self.traj:
                self._plan_queue.put(self.traj)
        elif c == 'q':
            self.stop_program = True

    # ── Plan replay ───────────────────────────────────────────────────────────

    def _replay_plan(self, plan):
        self._replaying = True
        traj = [{"arms": p["arms"],
                 "TL": np.array(p["TL"]),
                 "TR": np.array(p["TR"]),
                 "gl": p["gl"], "gr": p["gr"]} for p in plan]
        t0   = traj[0]
        arms = t0["arms"]
        print(f"[REPLAY] {len(traj)} steps — arms: {arms}")
        with self._cmd_lock:
            init_L = self._last_ee_L.copy()
            init_R = self._last_ee_R.copy()
            _, _, cur_gl, cur_gr = self._cmd
        target_L = t0["TL"] if "left"  in arms else init_L
        target_R = t0["TR"] if "right" in arms else init_R
        init_gl  = t0["gl"] if "left"  in arms else cur_gl
        init_gr  = t0["gr"] if "right" in arms else cur_gr

        def _set_cmd(cmd): self._cmd = cmd
        move_to_cart_threaded(target_L, target_R, init_L, init_R,
                              np.array([init_gl, init_gr]),
                              self._cmd_lock, _set_cmd, self.dt, self._ctrl_tick)
        for step in traj[1:]:
            if self.stop_program:
                return
            if not self._ctrl_tick.wait(timeout=0.5):
                continue
            self._ctrl_tick.clear()
            with self._cmd_lock:
                cur_TL, cur_TR, cur_gl, cur_gr = self._cmd
                new_TL = step["TL"] if "left"  in step["arms"] else cur_TL
                new_TR = step["TR"] if "right" in step["arms"] else cur_TR
                new_gl = step["gl"] if "left"  in step["arms"] else cur_gl
                new_gr = step["gr"] if "right" in step["arms"] else cur_gr
                self._cmd = (new_TL, new_TR, new_gl, new_gr)
        self._replaying = False
        print("\nReplay done.\n")

    # ── Main step ─────────────────────────────────────────────────────────────

    def step(self):
        """Single step of the TNR main loop — call in a while-not-stop_program loop."""
        while not self.stop_program and not self.hand_guide:
            if self._plot_requested:
                self._plot_requested = False
                self._save_joint_plot()

            if self._roi_requested:
                self._roi_requested = False
                self._roi_captured, self._roi_crop_jpg = capture_roi(
                    self.camera, self.session)

            if self._save_demo_requested:
                self._save_demo_requested = False
                self._inputting = True
                name  = self._pending_skill_name
                guide = self._pending_guideline
                self._pending_skill_name = self._pending_guideline = None
                save_skill_demo(self.traj, self.session,
                                skill_name=name, guideline=guide)
                self._inputting = False

            if self._execute_requested:
                self._execute_requested = False
                self._inputting = True
                skill_name = self._pending_skill_name
                self._pending_skill_name = self._pending_guideline = None
                self._inputting = False
                if skill_name:
                    while not self._plan_queue.empty():
                        self._plan_queue.get_nowait()
                    image_l, image_r = self.camera.get_frames()
                    print(f"[SKILL] Fetching plan for '{skill_name}'...")
                    stream_action_plans(image_l, image_r, skill_name,
                                        self._plan_queue, self.session,
                                        robot_model=self.OAP_check,
                                        q_current=self.OAI.get_state()["q_state"])

            # Auto-advance skill chain
            if (self._skill_chain
                    and not self._execute_requested
                    and self._plan_queue.empty()):
                self._pending_skill_name = self._skill_chain.pop(0)
                self._execute_requested  = True

            if not self._plan_queue.empty():
                self._replay_plan(self._plan_queue.get_nowait())

            time.sleep(0.05)

        if self.stop_program:
            return

        self.raw_samples = []
        self.traj        = []
        while not self.stop_program and self._soft_mode:
            time.sleep(self.dt)

        self.hand_guide = False

    # ── Joint transition plot ──────────────────────────────────────────────────

    def _save_joint_plot(self, out_dir: str = "plots") -> None:
        os.makedirs(out_dir, exist_ok=True)
        log   = list(self._joint_buffer)
        if not log:
            return
        times = np.array([e[0] for e in log]); times -= times[0]
        qs    = np.array([e[1] for e in log])
        qs_ik = np.array([e[2] for e in log])
        modes = [e[3] for e in log]
        t_switch = next((times[i] for i in range(1, len(modes))
                         if modes[i - 1] and not modes[i]), None)
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        for arm, q_off, label in [("left", 7, "Left"), ("right", 16, "Right")]:
            des = qs_ik[:, q_off:q_off + 7]
            act = qs[:,    q_off:q_off + 7]
            err = des - act
            fig, axes = plt.subplots(7, 1, figsize=(12, 18), sharex=True)
            fig.suptitle(f"Joint Tracking — soft→stiff transition — {label} arm", fontsize=13)
            for j, ax in enumerate(axes):
                ax.plot(times, des[:, j], "k--", linewidth=1.2, label="IK (desired)")
                ax.plot(times, act[:, j], color="tab:blue", linewidth=1.2, label="Measured")
                ax.fill_between(times, 0, err[:, j], color="tab:red", alpha=0.25, label="Error")
                if t_switch is not None:
                    ax.axvline(t_switch, color="k", linestyle=":", alpha=0.5, label="S pressed")
                ax.set_ylabel("rad", fontsize=8)
                ax.set_title(f"Joint {j + 1}", fontsize=9, loc="left")
                ax.legend(fontsize=7, loc="upper right", ncol=4)
                ax.grid(True, alpha=0.3)
            axes[-1].set_xlabel("Time (s)")
            plt.tight_layout()
            path = os.path.join(out_dir, f"joint_transition_{arm}_{ts}.png")
            plt.savefig(path, dpi=150)
            plt.close(fig)
            print(f"[PLOT] Saved → {path}")

    # ── Shutdown ───────────────────────────────────────────────────────────────

    def disconnect(self):
        self.stop_program = True
        self.listener.stop()
        self._ctrl_thread.join(timeout=2.0)
        self.OAI.set_torque(right=False, left=False)
        self.OAI.release_interface()
        if self.camera is not None:
            self.camera.close_cam()


# ── Entry point ────────────────────────────────────────────────────────────────

def _print_controls():
    print("\n" + "=" * 60)
    print("  OpenArm TNR — Teach aNd Replay  (Lilio SDK)")
    print("=" * 60)
    print("  H   — Hand-guide mode  (arm goes compliant)")
    print("  R   — Start recording  (hand-guide mode only)")
    print("  S   — Stop  recording  (arm goes stiff)")
    print("  C   — Close gripper    (hand-guide mode only)")
    print("  V   — Open  gripper    (hand-guide mode only)")
    print("  P   — Replay last recorded trajectory")
    print("  O   — Capture ROI  (draw box around target object)")
    print("  D   — Save trajectory as a named skill")
    print("  E   — Execute a skill from the server")
    print("  A   — Switch active arm  (stiff mode only)")
    print("  Q   — Quit")
    print("=" * 60 + "\n")


def main():
    parser = argparse.ArgumentParser(
        description="OpenArm Teach-and-Replay powered by the Lilio SDK")
    parser.add_argument("--arm", choices=["left", "right"], required=True,
                        help="Which arm to use as the active arm at start-up")
    parser.add_argument("--api-key", required=True,
                        help="Lilio API key  (e.g. lilio_sk_...)")
    parser.add_argument("--host", default="http://localhost:8000",
                        help="lilio_think server URL  (default: http://localhost:8000)")
    parser.add_argument("--camera-config", required=True,
                        help="Path to the camera calibration JSON produced by the ZED SDK")
    parser.add_argument("--dashboard-port", type=int, default=8765,
                        help="Port for the lilio_dashboard device server  (default: 8765)")
    args = parser.parse_args()

    with open(args.camera_config) as f:
        camera_calibration = json.load(f)["camera_calibration"]

    client = LilioClient(api_key=args.api_key, base_url=args.host)

    with client.session(camera_calibration) as session:
        controller = OpenArmController(arms=[args.arm], session=session)

        # Start the local device server so lilio_dashboard can connect.
        server = DeviceServer(controller, port=args.dashboard_port)
        server.start()

        _print_controls()
        try:
            while not controller.stop_program:
                controller.step()
        finally:
            controller.disconnect()


if __name__ == "__main__":
    main()
