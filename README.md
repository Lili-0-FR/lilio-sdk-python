# lili-o-sdk-python

Official Python SDK for the [Lili-O](https://app.lili-o.com) robot API.

## Installation

```bash
git clone https://github.com/lili-o/lilio-sdk-python.git
cd lilio-sdk-python
pip install -e .
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

Edit the `api_key` in [`examples/quickstart.py`](examples/quickstart.py), then run:

```bash
cd examples
python quickstart.py
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

## Contributing

The best way to contribute is to open a PR and share what you're building with Lili-O — new robot integrations, example scripts, or improvements to the SDK are all welcome. If this project is useful to you, a star on GitHub goes a long way.
