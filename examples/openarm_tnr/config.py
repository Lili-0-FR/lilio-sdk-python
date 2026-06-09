"""
Robot configuration for the OpenArm TNR example.

Paths assume a standard LILIO installation at /home/master/LILIO/lilio_move.
Override with the LILIO_MOVE_ROOT environment variable if your setup differs:

    export LILIO_MOVE_ROOT=/path/to/lilio_move
"""
import os
import numpy as np

LILIO_MOVE_ROOT = os.environ.get("LILIO_MOVE_ROOT", "/home/master/LILIO/lilio_move")

PLACO_URDF     = os.path.join(LILIO_MOVE_ROOT, "robot_model", "openarm_description", "openarm_placo.urdf")
COLLISION_PATH = os.path.join(LILIO_MOVE_ROOT, "robot_model", "openarm_description", "collision.json")

LEFT_JOINTS  = [f"openarm_left_joint{i}"  for i in range(1, 8)]
RIGHT_JOINTS = [f"openarm_right_joint{i}" for i in range(1, 8)]

LEFT_FINGER   = "openarm_left_finger_joint1"
LEFT_FINGER2  = "openarm_left_finger_joint2"
RIGHT_FINGER  = "openarm_right_finger_joint1"
RIGHT_FINGER2 = "openarm_right_finger_joint2"

GRIPPER_MIN = 0.0
GRIPPER_MAX = 2.0

OA_CONFIG = {
    "dt":               0.02,
    "lambda_reg":       1e-4,
    "cost_ee":          np.array([50.0, 50.0, 50.0, 10.0, 10.0, 10.0]),
    "cost_posture":     1e-5,
    "collision_limit":  0.03,
    "collision_margin": 0.01,
}

VELOCITY_LIMIT = {
    **{f"openarm_left_joint{i}":  2.175 if i <= 4 else 2.61 for i in range(1, 8)},
    "openarm_left_finger_joint1": 5.0,
    "openarm_left_finger_joint2": 5.0,
    **{f"openarm_right_joint{i}": 2.175 if i <= 4 else 2.61 for i in range(1, 8)},
    "openarm_right_finger_joint1": 5.0,
    "openarm_right_finger_joint2": 5.0,
}

POSTURE_HOME = {
    "openarm_left_joint1":  0.0,   "openarm_left_joint2":  -0.52,
    "openarm_left_joint3":  0.0,   "openarm_left_joint4":   1.5708,
    "openarm_left_joint5": -0.52,  "openarm_left_joint6":   0.0,
    "openarm_left_joint7":  0.0,
    "openarm_left_finger_joint1": 0.0, "openarm_left_finger_joint2": 0.0,
    "openarm_right_joint1":  0.0,  "openarm_right_joint2":   0.52,
    "openarm_right_joint3":  0.0,  "openarm_right_joint4":   1.5708,
    "openarm_right_joint5":  0.52, "openarm_right_joint6":   0.0,
    "openarm_right_joint7":  0.0,
    "openarm_right_finger_joint1": 0.0, "openarm_right_finger_joint2": 0.0,
}
