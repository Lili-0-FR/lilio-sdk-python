import time
import threading

import numpy as np
import openarm_can as oa
import placo
from loop_rate_limiters import RateLimiter

from config import *
from oa_utils import plot_tracking


class OpenArmInterface:
    def __init__(self, read_timeout):
        # Hardware ports using CAN-FD
        self.left_hw  = oa.OpenArm("can4", True)
        self.right_hw = oa.OpenArm("can5", True)
        print("Connection: ready !")

        # PID gains
        self.kp = [300.0, 300.0, 150.0, 150.0, 40.0, 40.0, 30.0, 10.0]
        self.kd = [2.5,   2.5,   2.5,   2.5,   0.8,  0.8,  0.8,  0.9]
        self.ki = [kp * 0.9 for kp in self.kp]

        # Gripper limits
        self.gripper_dq         = 10.0
        self.gripper_torque_pu  = 0.25

        # Motor types
        self.arm_motor_types = [
            oa.MotorType.DM8009, oa.MotorType.DM8009,
            oa.MotorType.DM4340, oa.MotorType.DM4340,
            oa.MotorType.DM4310, oa.MotorType.DM4310, oa.MotorType.DM4310,
        ]
        self.arm_ids  = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07]
        self.arm_recv = [0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17]

        # Init arms and grippers
        self.left_hw.init_arm_motors(self.arm_motor_types, self.arm_ids, self.arm_recv)
        self.left_hw.init_gripper_motor(oa.MotorType.DM4310, 0x08, 0x18, oa.ControlMode.POS_FORCE)
        self.right_hw.init_arm_motors(self.arm_motor_types, self.arm_ids, self.arm_recv)
        self.right_hw.init_gripper_motor(oa.MotorType.DM4310, 0x08, 0x18, oa.ControlMode.POS_FORCE)
        print("Arm and gripper initialized !")

        self.left_hw.set_callback_mode_all(oa.CallbackMode.STATE)
        self.right_hw.set_callback_mode_all(oa.CallbackMode.STATE)
        print("Feedback mode: ready !")

        self.read_timeout = read_timeout

        # Threading
        self._lock       = threading.Lock()
        self._running    = True
        self._data_ready = False
        self.latest_state = {
            "q_state": np.zeros(25),
            "gl": 0.0, "gr": 0.0,
            "tl": 0.0, "tr": 0.0,
        }

        for _ in range(50):
            self.left_hw.recv_all(0)
            self.right_hw.recv_all(0)

        self.thread = threading.Thread(target=self._io_loop, daemon=True)
        self.thread.start()
        self.dt = OA_CONFIG["dt"]
        print("Communication thread: Running!")

        print("Waiting for initial hardware state...")
        while not self._data_ready:
            time.sleep(0.01)
        print("Communication thread: Ready and Synced!")

    def start(self):
        self.set_torque(left=True, right=True)
        time.sleep(0.2)
        start_states = self.get_state()
        start_obs    = start_states["q_state"]
        self.start_gl = start_states["gl"]
        self.start_gr = start_states["gr"]
        if np.all(start_obs[7:14] == 0):
            print("ERROR: Sensors reporting zero. Aborting to prevent jump!")
            self.set_torque(left=False, right=False)
            self.release_interface()
            exit()

    def move_to_pose(self, target_pose, gl, gr, duration=5.0,
                     robot_model=None, plot=True, plot_title="move_to_pose"):
        spline = placo.CubicSpline()
        spline.add_point(0.0, 0.0, 0.0)
        spline.add_point(duration, 1.0, 0.0)
        for _ in range(5):
            start_obs = self.get_state()["q_state"]
            start_gl  = self.get_state()["gl"]
            start_gr  = self.get_state()["gr"]

        t_log, q_des_log, q_act_log = [], [], []
        rate    = RateLimiter(frequency=1.0 / self.dt)
        t_start = time.monotonic()
        print("Reaching target pose...")
        while (t := time.monotonic() - t_start) < duration:
            alpha = spline.pos(t)
            q_cmd = np.zeros_like(target_pose)
            q_cmd[6] = 1
            for i in range(7, 25):
                q_cmd[i] = start_obs[i] + (target_pose[i] - start_obs[i]) * alpha
            gl_cmd = start_gl + (gl - start_gl) * alpha
            gr_cmd = start_gr + (gr - start_gr) * alpha
            tau = np.zeros(25)
            if robot_model is not None:
                tau_grav = robot_model.get_gravity_torques(q_cmd)
                tau[7:14]  = tau_grav[6:13]
                tau[16:23] = tau_grav[15:22]
            self.send_action(arms=["left", "right"], q_placo=q_cmd,
                             gl=gl_cmd, gr=gr_cmd, tau=tau)
            if plot:
                t_log.append(t)
                q_des_log.append(q_cmd[7:14].copy())
                q_act_log.append(self.get_state()["q_state"][7:14].copy())
            rate.sleep()
        print("Target pose reached !")
        if plot and len(t_log) > 0:
            plot_tracking(np.array(t_log), np.array(q_des_log),
                          np.array(q_act_log), plot_title)

    def _io_loop(self):
        while self._running:
            self.left_hw.refresh_all()
            self.right_hw.refresh_all()
            self.left_hw.recv_all(self.read_timeout)
            self.right_hw.recv_all(self.read_timeout)
            l_motors          = self.left_hw.get_arm().get_motors()
            r_motors          = self.right_hw.get_arm().get_motors()
            l_gripper_motor   = self.left_hw.get_gripper().get_motors()[0]
            r_gripper_motor   = self.right_hw.get_gripper().get_motors()[0]
            gl = l_gripper_motor.get_position()
            gr = r_gripper_motor.get_position()
            tl = l_gripper_motor.get_torque()
            tr = r_gripper_motor.get_torque()
            q = [0.0, 0, 0, 0, 0, 0, 1]
            for i in range(7):
                q.append(l_motors[i].get_position())
            q.append(0.0); q.append(0.0)  # left fingers
            for i in range(7):
                q.append(r_motors[i].get_position())
            q.append(0.0); q.append(0.0)  # right fingers
            with self._lock:
                self.latest_state["q_state"] = np.array(q)
                self.latest_state["gl"] = gl
                self.latest_state["gr"] = gr
                self.latest_state["tl"] = tl
                self.latest_state["tr"] = tr
                self._data_ready = True
            time.sleep(0.001)

    def get_state(self):
        with self._lock:
            return {k: (v.copy() if isinstance(v, np.ndarray) else v)
                    for k, v in self.latest_state.items()}

    def send_action_soft(self, arms, q_placo, gl, gr, tau):
        """MIT control (hand-guide mode): kp=0, kd=0.1, gravity feedforward."""
        kp_soft = [0.0] * 7
        kd_soft = [0.1] * 7
        tau_soft_scale = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.8])
        if "left" in arms:
            tau_L  = tau[7:14] * tau_soft_scale
            params = [oa.MITParam(kp_soft[i], kd_soft[i], q_placo[7:14][i], 0, tau_L[i])
                      for i in range(7)]
            self.left_hw.get_arm().mit_control_all(params)
            self.left_hw.get_gripper().posforce_control_one(
                0, oa.PosForceParam(gl, self.gripper_dq, self.gripper_torque_pu))
        if "right" in arms:
            tau_R  = tau[16:23] * tau_soft_scale
            params = [oa.MITParam(kp_soft[i], kd_soft[i], q_placo[16:23][i], 0, tau_R[i])
                      for i in range(7)]
            self.right_hw.get_arm().mit_control_all(params)
            self.right_hw.get_gripper().posforce_control_one(
                0, oa.PosForceParam(gr, self.gripper_dq, self.gripper_torque_pu))

    def send_action(self, arms, q_placo, gl, gr, tau, kp_qot=0, kd_qot=0):
        """PID control for the given arm(s)."""
        kp = self.kp.copy()
        kd = self.kd.copy()
        if kp_qot > 0:
            kp = [int(k * kp_qot) for k in kp]
            kd = [k * kd_qot       for k in kd]
        if "left" in arms:
            params = [oa.PIDParam(kp[i], kd[i], self.ki[i],
                                  q_placo[7:14][i], 0, tau[7:14][i])
                      for i in range(7)]
            self.left_hw.get_arm().pid_control_all(params)
            self.left_hw.get_gripper().posforce_control_one(
                0, oa.PosForceParam(gl, self.gripper_dq, self.gripper_torque_pu))
        if "right" in arms:
            params = [oa.PIDParam(kp[i], kd[i], self.ki[i],
                                  q_placo[16:23][i], 0, tau[16:23][i])
                      for i in range(7)]
            self.right_hw.get_arm().pid_control_all(params)
            self.right_hw.get_gripper().posforce_control_one(
                0, oa.PosForceParam(gr, self.gripper_dq, self.gripper_torque_pu))

    def set_torque(self, left=None, right=None):
        if left is True:
            self.left_hw.enable_all()
            self.left_hw.get_arm().reset_integral_all()
            print("Left arm torque enabled.")
        elif left is False:
            self.left_hw.disable_all()
            print("Left arm torque disabled.")
        if right is True:
            self.right_hw.enable_all()
            self.right_hw.get_arm().reset_integral_all()
            print("Right arm torque enabled.")
        elif right is False:
            self.right_hw.disable_all()
            print("Right arm torque disabled.")

    def set_soft(self, arms):
        state    = self.get_state()
        q_mes    = state["q_state"]
        kp_val   = [0.0] * 8
        kd_val   = [0.1] * 8
        if "left" in arms:
            self.left_hw.get_arm().reset_integral_all()
            params = [oa.MITParam(kp_val[i], kd_val[i], q_mes[7:14][i], 0, 0)
                      for i in range(7)]
            self.left_hw.get_arm().mit_control_all(params)
        if "right" in arms:
            self.right_hw.get_arm().reset_integral_all()
            params = [oa.MITParam(kp_val[i], kd_val[i], q_mes[16:23][i], 0, 0)
                      for i in range(7)]
            self.right_hw.get_arm().mit_control_all(params)

    def reset_integral(self, left=False, right=False):
        if left:
            self.left_hw.get_arm().reset_integral_all()
        if right:
            self.right_hw.get_arm().reset_integral_all()

    def release_interface(self):
        self._running = False
        self.thread.join()

    def grasp(self, side, gl):
        hw = self.left_hw if side == "left" else self.right_hw
        hw.get_gripper().posforce_control_one(
            0, oa.PosForceParam(gl, self.gripper_dq, self.gripper_torque_pu))
