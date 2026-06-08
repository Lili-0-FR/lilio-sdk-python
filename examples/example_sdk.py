"""
Static-image demonstration of the lilio_think pipeline via the SDK.

Usage
-----
    python example_sdk.py \\
        --demo_img_left   left_0000.png  \\
        --demo_img_right  right_0000.png \\
        --inf_img_left    left_0001.png  \\
        --inf_img_right   right_0001.png \\
        --demo_trajectory trajectory.npy \\
        --config_vision   config_vision_QM.json \\
        --api_key         lilio_sk_... \\
        --roi             640 150 1060 680 \\
        --host            http://localhost:8000
"""

import argparse
import json
import sys

import numpy as np
import cv2

from lilio import LilioClient, LilioError

SKILL_NAME = "open_coffee_machine"


def get_args_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('--demo_img_left',   type=str, required=True)
    parser.add_argument('--demo_img_right',  type=str, required=True)
    parser.add_argument('--inf_img_left',    type=str, required=True)
    parser.add_argument('--inf_img_right',   type=str, required=True)
    parser.add_argument('--demo_trajectory', type=str, required=True)
    parser.add_argument('--config_vision',   type=str, required=True)
    parser.add_argument('--api_key',         type=str, required=True)
    parser.add_argument('--host',            type=str, default="http://localhost:8000")
    parser.add_argument('--roi',             nargs=4, type=int, required=True,
                        metavar=('X1', 'Y1', 'X2', 'Y2'))
    return parser


def load_image(path: str) -> np.ndarray:
    bgr = cv2.imread(path, cv2.IMREAD_COLOR)
    if bgr is None:
        raise FileNotFoundError(f"Could not load image: {path}")
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)


args = get_args_parser().parse_args()

client = LilioClient(api_key=args.api_key, base_url=args.host)

with open(args.config_vision) as f:
    camera_calibration = json.load(f)["camera_calibration"]

# ── Demo phase ────────────────────────────────────────────────────────────────

img_demo_left  = load_image(args.demo_img_left)
img_demo_right = load_image(args.demo_img_right)
bbox = args.roi

traj = np.load(args.demo_trajectory, allow_pickle=True)

# ── Inference phase ───────────────────────────────────────────────────────────

img_inf_left  = load_image(args.inf_img_left)
img_inf_right = load_image(args.inf_img_right)

try:
    with client.session(camera_calibration) as session:
        session.set_roi(img_demo_left, img_demo_right, box=bbox)
        print(f"[ROI] set.")

        session.save_skill(SKILL_NAME, traj)
        print(f"[SKILL] '{SKILL_NAME}' saved.")

        plan = session.get_action_plan(SKILL_NAME, img_inf_left, img_inf_right)
        if plan:
            print("bottleneck end-effector pose:", plan[0])
        else:
            print("No plan generated — object not detected or pose estimation failed.")

except LilioError as e:
    print(f"[ERROR] {e}")
    sys.exit(1)
