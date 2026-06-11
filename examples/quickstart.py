import json
import cv2
import numpy as np
from lilio import LilioClient

client = LilioClient(api_key="lilio_sk_...")
with open("assets/config_vision_QM.json") as f:
    camera_calibration = json.load(f)["camera_calibration"]

img_demo_left  = cv2.cvtColor(cv2.imread("assets/left_0000.png"),  cv2.COLOR_BGR2RGB)
img_demo_right = cv2.cvtColor(cv2.imread("assets/right_0000.png"), cv2.COLOR_BGR2RGB)
img_inf_left   = cv2.cvtColor(cv2.imread("assets/left_0001.png"),  cv2.COLOR_BGR2RGB)
img_inf_right  = cv2.cvtColor(cv2.imread("assets/right_0001.png"), cv2.COLOR_BGR2RGB)
traj = np.load("assets/trajectory.npy", allow_pickle=True)

print("Draw a bounding box tightly around the coffee machine — include the full body but")
print("exclude as much background as possible. The tighter the box, the better the result.")
print("Press ENTER or SPACE to confirm, C to cancel.")
x, y, w, h = cv2.selectROI("Select ROI — coffee machine", cv2.cvtColor(img_demo_left, cv2.COLOR_RGB2BGR), fromCenter=False)
cv2.destroyWindow("Select ROI")
box = [x, y, x + w, y + h]

with client.session(camera_calibration) as session:
    print("[1/3] Setting ROI...")
    session.set_roi(img_demo_left, img_demo_right, box=box)
    print("[1/3] ROI set.")

    print("[2/3] Saving skill...")
    session.save_skill("open_coffee_machine", traj)
    print("[2/3] Skill saved.")

    print("[3/3] Running inference...")
    plan = session.get_action_plan("open_coffee_machine", img_inf_left, img_inf_right)
    print("[3/3] Done.")

    if plan:
        print("bottleneck pose:", plan[0])
    else:
        print("object not detected")
