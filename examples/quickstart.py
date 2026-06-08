import json
import cv2
import numpy as np
from lilio import LilioClient

client = LilioClient(api_key="lilio_sk_...", base_url="http://localhost:8000")

with open("assets/config_vision_QM.json") as f:
    camera_calibration = json.load(f)["camera_calibration"]

img_demo_left  = cv2.cvtColor(cv2.imread("assets/left_0000.png"),  cv2.COLOR_BGR2RGB)
img_demo_right = cv2.cvtColor(cv2.imread("assets/right_0000.png"), cv2.COLOR_BGR2RGB)
img_inf_left   = cv2.cvtColor(cv2.imread("assets/left_0001.png"),  cv2.COLOR_BGR2RGB)
img_inf_right  = cv2.cvtColor(cv2.imread("assets/right_0001.png"), cv2.COLOR_BGR2RGB)
traj = np.load("assets/trajectory.npy", allow_pickle=True)

with client.session(camera_calibration) as session:
    session.set_roi(img_demo_left, img_demo_right, box=[640, 150, 1060, 680])
    session.save_skill("open_coffee_machine", traj)

    plan = session.get_action_plan("open_coffee_machine", img_inf_left, img_inf_right)
    if plan:
        print("bottleneck pose:", plan[0])
    else:
        print("object not detected")
