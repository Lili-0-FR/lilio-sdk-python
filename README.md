# lili-o-sdk-python

Official Python SDK for the [Lili-O](https://lili-o.com) robot API.

## Installation

```bash
pip install lilio-think
```

## Authentication

Get your API key from the Lili-O dashboard, then pass it to the client:

```python
from lilio import LilioClient

client = LilioClient(api_key="lilio_sk_...")
```

By default the client points to `https://api.lili-o.com`. Use the `base_url` parameter to target a different server:

```python
client = LilioClient(api_key="lilio_sk_...", base_url="http://localhost:8000")
```

## Quick start

```python
import json
import numpy as np
import cv2
from lilio import LilioClient

client = LilioClient(api_key="lilio_sk_...")

with open("config_vision.json") as f:
    camera_calibration = json.load(f)["camera_calibration"]

img_demo_left  = cv2.cvtColor(cv2.imread("left_0000.png"),  cv2.COLOR_BGR2RGB)
img_demo_right = cv2.cvtColor(cv2.imread("right_0000.png"), cv2.COLOR_BGR2RGB)
img_inf_left   = cv2.cvtColor(cv2.imread("left_0001.png"),  cv2.COLOR_BGR2RGB)
img_inf_right  = cv2.cvtColor(cv2.imread("right_0001.png"), cv2.COLOR_BGR2RGB)
traj = np.load("trajectory.npy", allow_pickle=True)

with client.session(camera_calibration) as session:
    # Demo phase
    session.set_roi(img_demo_left, img_demo_right, box=[640, 150, 1060, 680])
    session.save_skill("open_coffee_machine", traj)

    # Inference phase
    plan = session.get_action_plan("open_coffee_machine", img_inf_left, img_inf_right)
    if plan:
        print("bottleneck pose:", plan[0])
```

## API reference

### `LilioClient`

| Method | Description |
|--------|-------------|
| `LilioClient(api_key, base_url)` | Create a client |
| `client.session(camera_calibration)` | Open a session (context manager) |
| `client.list_skills()` | List all saved skills |

### `Session`

| Method | Description |
|--------|-------------|
| `session.set_roi(left_img, right_img, box)` | Demo phase — set the object region of interest |
| `session.save_skill(skill_name, trajectory)` | Demo phase — save the skill |
| `session.get_action_plan(skill_name, left_img, right_img)` | Inference — get the action plan |

Images are passed as RGB `numpy` arrays. The SDK handles base64 encoding internally.

Sessions are closed automatically at the end of the `with` block, even if an error occurs.

## Error handling

All API errors raise `LilioError`:

```python
from lilio import LilioClient, LilioError

try:
    with client.session(camera_calibration) as session:
        plan = session.get_action_plan("unknown_skill", img_left, img_right)
except LilioError as e:
    print(e.status_code, e.detail)
```

## Examples

See the [`examples/`](examples/) folder for a full working script and robot-specific integrations.
