"""
rby1_sdk_stub.py — drop-in stub for rby1_sdk when no robot is available.

Inject before importing RBY1_tnr_2:
    import sys
    import rby1_sdk_stub
    sys.modules['rby1_sdk'] = rby1_sdk_stub

Or run RBY1_tnr_2.py with --sim flag.

Behaviour:
  - All builder methods are chainable no-ops.
  - FK always returns identity matrices.
  - State callback fires at ~50 Hz with zero joint positions.
  - stream.request_feedback() returns zero position error → converges immediately.
  - Commands are printed to stdout.
"""

import threading
import time
import numpy as np


# ── Enums ─────────────────────────────────────────────────────────────────────

class ControlManagerState:
    class State:
        FAULT = "FAULT"
        ENABLED = "ENABLED"
    FAULT = "FAULT"


class RobotCommandFeedback:
    class FinishCode:
        Ok = "Ok"
        Failed = "Failed"
    class Result:
        finish_code = "Ok"


# ── Chainable builder stub ─────────────────────────────────────────────────────

class _Builder:
    """Any method call returns self, so chaining always works."""
    def __getattr__(self, name):
        def _noop(*args, **kwargs):
            return self
        return _noop


# ── Types (used in annotations) ───────────────────────────────────────────────

class RobotCommand:
    pass


# ── Specific builders (exposed as module-level names) ─────────────────────────

def RobotCommandBuilder():       return _Builder()
def ComponentBasedCommandBuilder(): return _Builder()
def BodyComponentBasedCommandBuilder(): return _Builder()
def CartesianCommandBuilder():   return _Builder()
def CommandHeaderBuilder():      return _Builder()
def JointPositionCommandBuilder(): return _Builder()


# ── Dynamics stub ─────────────────────────────────────────────────────────────

class _DynState:
    def set_q(self, q): pass

class _Dynamics:
    def make_state(self, links, joint_names):
        return _DynState()
    def compute_forward_kinematics(self, state): pass
    def compute_transformation(self, state, from_idx, to_idx):
        return np.eye(4)


# ── Command stream stub ───────────────────────────────────────────────────────

class _FeedbackArm:
    class _CartCmd:
        class _Error:
            position_error = 0.0
        se3_pose_tracking_errors = [_Error()]
    cartesian_command = _CartCmd()

class _FeedbackBody:
    right_arm_command = _FeedbackArm()
    left_arm_command  = _FeedbackArm()

class _FeedbackCBC:
    body_component_based_command = _FeedbackBody()

class _FeedbackBC:
    body_command = _FeedbackCBC()

class _Feedback:
    component_based_command = _FeedbackBC()

class _Stream:
    def send_command(self, cmd):
        print("[SIM] send_command")
    def request_feedback(self):
        return _Feedback()
    def cancel(self):
        print("[SIM] stream cancelled")


# ── Control manager state stub ────────────────────────────────────────────────

class _CMState:
    state = "ENABLED"   # not FAULT → no reset needed


# ── Robot model stub ──────────────────────────────────────────────────────────

_JOINT_NAMES = [f"joint_{i}" for i in range(32)]

class _Model:
    robot_joint_names = _JOINT_NAMES
    model_name = "A"


# ── Robot stub ────────────────────────────────────────────────────────────────

class _Robot:
    def __init__(self):
        self._cb_thread = None
        self._running   = False

    def connect(self):    print("[SIM] connect");    return True
    def is_power_on(self, pattern): return True
    def power_on(self, pattern):    return True
    def is_servo_on(self, pattern): return True
    def servo_on(self, pattern):    return True
    def get_control_manager_state(self): return _CMState()
    def reset_fault_control_manager(self): pass
    def enable_control_manager(self): print("[SIM] control manager enabled")
    def disable_control_manager(self): print("[SIM] control manager disabled")
    def model(self): return _Model()
    def get_dynamics(self): return _Dynamics()
    def create_command_stream(self): return _Stream()

    def get_state(self):
        class _S:
            position = np.zeros(len(_JOINT_NAMES))
        return _S()

    def start_state_update(self, cb, hz):
        """Fire cb at hz in a background daemon thread."""
        dt = 1.0 / hz
        self._running = True
        def _loop():
            while self._running:
                class _State:
                    position = np.zeros(len(_JOINT_NAMES))
                    class tool_flange_right:
                        switch_A = False
                    class tool_flange_left:
                        switch_A = False
                cb(_State())
                time.sleep(dt)
        self._cb_thread = threading.Thread(target=_loop, daemon=True)
        self._cb_thread.start()
        print(f"[SIM] state update started at {hz} Hz")

    def send_command(self, cmd, timeout=1):
        print("[SIM] send_command (blocking)")
        result = RobotCommandFeedback.Result()
        class _Future:
            def get(self_): return result
        return _Future()


# ── DynamixelBus stub ─────────────────────────────────────────────────────────

class DynamixelBus:
    def __init__(self, device): pass
    def open_port(self): pass
    def set_baud_rate(self, baud): pass
    def ping(self, dev_id): return True
    def torque_enable(self, ids, enables): pass
    def group_sync_write_send_position(self, targets):
        print(f"[SIM] gripper → {targets}")


# ── upc namespace ─────────────────────────────────────────────────────────────

class upc:
    GripperDeviceName = "/dev/ttyUSB_gripper_stub"


# ── Factory ───────────────────────────────────────────────────────────────────

def create_robot(address, model="a"):
    print(f"[SIM] create_robot({address!r}, {model!r})")
    return _Robot()
