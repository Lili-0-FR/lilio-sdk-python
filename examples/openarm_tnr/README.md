# OpenArm Teach-and-Replay — Lilio SDK example

A full teach-and-repeat controller for the OpenArm robot, powered by the
[Lilio SDK](../../README.md). The example is self-contained — every module
needed to run (`OA_interface`, `OA_placo`, `zed_camera`) is included in this
folder, no `lilio_move` import required at runtime.

---

## Architecture

```
                    this machine
┌────────────────────────────────────────────────────────┐
│                                                        │
│  oa_tnr_sdk.py                                         │
│  ├── OpenArmController                                 │
│  │     ├── OA_interface.py  ── CAN bus ──► OpenArm     │
│  │     ├── OA_placo.py      ── IK solver               │
│  │     └── zed_camera.py    ── ZED Mini camera         │
│  │                                                     │
│  ├── LilioClient (SDK) ─── HTTP ──► lilio_think :8000  │
│  │                                  (AI / skill server)│
│  │                                                     │
│  └── DeviceServer :8765 ◄── WebSocket/HTTP ──┐         │
│                                              │         │
└──────────────────────────────────────────────┼─────────┘
                                               │
                                   lilio_dashboard (browser)
```

---

## Step 1 — Install the hardware dependencies

These packages require native libraries and must be installed before anything else.

### openarm_can

The CAN bus library to communicate with OpenArm motors.
Follow the installation instructions in the OpenArm firmware repository:

> https://github.com/Lili-O-FR/openarm_can

```bash
# Typical install after cloning the repo:
pip install ./openarm_can
```

Make sure your CAN interfaces are configured and up before running the script.
The controller expects `can4` (left arm) and `can5` (right arm):

```bash
# Bring up CAN interfaces (adapt bitrate to your setup)
sudo ip link set can4 up type can bitrate 1000000
sudo ip link set can5 up type can bitrate 1000000
```

### ZED SDK + pyzed

Install the Stereolabs ZED SDK for your platform:

> https://www.stereolabs.com/docs/installation/

The Python bindings (`pyzed`) are included with the SDK installer.
Verify the install:

```bash
python -c "import pyzed.sl as sl; print(sl.Camera().get_sdk_version())"
```

### placo

The IK / kinematics solver:

```bash
pip install placo
```

---

## Step 2 — Set up the robot model

The controller needs the OpenArm URDF and collision files.
They live inside `lilio_move`:

```bash
export LILIO_MOVE_ROOT=/path/to/lilio_move   # default: /home/master/LILIO/lilio_move
```

If `lilio_move` is at the default path you don't need to set anything.

Expected files:
```
$LILIO_MOVE_ROOT/robot_model/openarm_description/openarm_placo.urdf
$LILIO_MOVE_ROOT/robot_model/openarm_description/collision.json
```

---

## Step 3 — Install the Python dependencies

```bash
# From this folder (examples/openarm_tnr/):
pip install -r requirements.txt
```

This installs the Lilio SDK (from the repo root), FastAPI, uvicorn, numpy,
opencv, matplotlib, pynput, and loop-rate-limiters.

---

## Step 4 — Get a Lilio API key

1. Go to the [Lilio dashboard](https://lili-o.com/dashboard)
2. Sign in or create an account
3. Open **API Keys** → **New key**
4. Copy the key — it starts with `lilio_sk_`

---

## Step 5 — Start `lilio_think`

`lilio_think` is the AI server that handles skill learning and inference.
It must be running before you start the controller.

```bash
# On the robot computer (or any machine reachable by the controller):
cd /path/to/lilio_think
python main.py
# → FastAPI server running on http://0.0.0.0:8000
```

---

## Step 6 — Run the controller

```bash
python oa_tnr_sdk.py \
    --arm right \
    --api-key lilio_sk_... \
    --host http://localhost:8000 \
    --camera-config /path/to/config_vision_QM.json
```

| Argument | Required | Description |
|----------|----------|-------------|
| `--arm` | yes | Starting active arm: `left` or `right` |
| `--api-key` | yes | Your Lilio API key (`lilio_sk_...`) |
| `--host` | no | `lilio_think` server URL (default: `http://localhost:8000`) |
| `--camera-config` | yes | Path to the ZED calibration JSON (`config_vision_QM.json`) |
| `--dashboard-port` | no | Port for the dashboard device server (default: `8765`) |

On start the script will:
1. Connect to the OpenArm motors via CAN
2. Move both arms to the home pose
3. Open the ZED camera
4. Open a session with `lilio_think` via the SDK
5. Start the dashboard device server on port `8765`
6. Start the keyboard listener

---

## Step 7 — Open the lilio_dashboard

The dashboard is a local web app in `lilio_dashboard/`.

```bash
cd /path/to/lilio_dashboard
npm install       # first time only
npm run dev
# → http://localhost:5173
```

Open `http://localhost:5173` in your browser.

### Add the robot as a device

1. Click **Add device** (or the `+` button in the device selector)
2. Set:
   - **Name**: anything (e.g. `OpenArm`)
   - **IP**: the IP of the machine running `oa_tnr_sdk.py`
     - Same machine → `localhost` or `127.0.0.1`
     - Different machine → e.g. `192.168.1.42`
   - **Port**: `8765`
3. Click **Save** — the device should show as **Connected**

### What you'll see

| Dashboard section | Data |
|---|---|
| **Joints** | Live position of all 14 arm joints (rad), updated at 20 Hz |
| **Cameras** | Left + right ZED feed at 15 fps |
| **End Effectors** | X/Y/Z position of both end-effectors |
| **App state** | `idle` / `hand_guide` / `recording` / `replaying` |

### Sending commands from the dashboard

The **App Manager** tab shows command buttons for the running app.
All keyboard shortcuts can be triggered from the UI.

For **Save skill** (`D`) and **Execute skill** (`E`), a `skill_name` must be
provided. These commands can also be sent directly via the device server API:

```bash
# Execute a skill
curl -X POST http://localhost:8765/apps/openarm_tnr/command \
     -H "Content-Type: application/json" \
     -d '{"cmd": "e", "skill_name": "open_coffee_machine"}'

# Save the recorded trajectory as a skill
curl -X POST http://localhost:8765/apps/openarm_tnr/command \
     -H "Content-Type: application/json" \
     -d '{"cmd": "d", "skill_name": "open_coffee_machine", "guideline": "opens the coffee machine"}'
```

---

## Keyboard controls

| Key | Action |
|-----|--------|
| `H` | Hand-guide mode — arm goes compliant |
| `R` | Start recording *(hand-guide mode only)* |
| `S` | Stop recording — arm goes stiff |
| `C` | Close gripper *(hand-guide mode only)* |
| `V` | Open gripper *(hand-guide mode only)* |
| `P` | Replay the last recorded trajectory |
| `O` | Capture ROI — point camera at target, draw bounding box |
| `D` | Save the recorded trajectory as a named skill |
| `E` | Execute a skill from the server |
| `A` | Switch active arm between left ↔ right *(stiff mode only)* |
| `Q` | Quit |

---

## Typical session walkthrough

```
1. Start lilio_think         →  python main.py
2. Start the controller      →  python oa_tnr_sdk.py --arm right ...
   Arms move to home pose.
3. Open the dashboard        →  npm run dev  →  add device localhost:8765
4. Press H                   →  arm goes compliant (hand-guide mode)
5. Press R                   →  start recording
   Move the arm through the task.
6. Press S                   →  stop recording, arm goes stiff
7. Press O                   →  point camera at object, draw ROI box
8. Press D                   →  name the skill (e.g. "pick_cup")
9. Press E                   →  enter skill name to execute
   Controller detects object, adapts trajectory, replays it.
```

---

## File overview

| File | Description |
|------|-------------|
| `oa_tnr_sdk.py` | Main controller class + CLI entry point |
| `device_server.py` | FastAPI device server for lilio_dashboard |
| `oa_utils.py` | Shared helpers — all lilio_think calls use the SDK here |
| `OA_interface.py` | Low-level CAN bus interface for the OpenArm motors |
| `OA_placo.py` | Placo-based IK solver and reachability checker |
| `zed_camera.py` | ZED Mini stereo camera wrapper |
| `config.py` | Robot constants (joints, gains, URDF paths) |
| `requirements.txt` | Python dependencies |
