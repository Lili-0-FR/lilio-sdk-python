# RBY1 Control — Teach and Replay

Teach-and-Replay + AI skill execution for the **Rainbow Robotics RB-Y1** robot.

> **lili-o light version — important constraints:**
> - Never move the target object while recording or during inference. Once a skill is saved, you may reposition the object before the next skill. Real-time tracking is not available.
> - Do not place objects between the robot arm and the target. Collision avoidance is not available.
> - One object at a time. Generalisation across objects is not available.
> - This pipeline is **not** meant for end-to-end tasks. Break every task into atomic subtasks (e.g. "pick can" is one skill, "place can in trash" is a separate skill).

---

## Installation

### 1. Install lilio_think and lilio_see

Everything is self-contained in this repository. From the root of `lilio_think`:

```bash
# Create and activate a virtual environment (Python 3.10 required)
python3.10 -m venv ~/lilio_venv
source ~/lilio_venv/bin/activate

# Install lilio_see (pre-built wheel, includes all ONNX vision models)
pip install dist/lilio_see-0.0.1-py3-none-any.whl

# Install lilio_think
pip install .
```

For **Jetson AGX Orin**, follow the dedicated Jetson section in the main `README.md` before running the above — onnxruntime-gpu and cuDNN need special handling on ARM64.

### 2. Set LD_LIBRARY_PATH (desktop Linux only)

Required for GPU inference via onnxruntime:

```bash
export LD_LIBRARY_PATH=~/lilio_venv/lib/python3.10/site-packages/nvidia/cudnn/lib/:$LD_LIBRARY_PATH
```

Add to `~/.bashrc` to make it permanent. On Jetson this is not needed (cuDNN is a system library).

### 3. Install RBY1-specific dependencies

```bash
pip install pynput numpy opencv-python scipy
```

You also need (not on PyPI — install from their respective distributions):

- **`rby1_sdk`** — Rainbow Robotics SDK
- **`pyzed`** — ZED SDK Python bindings (required for the camera; skip if using `--sim`)

### Hardware

| Component | Details |
|-----------|---------|
| Robot | Rainbow Robotics RB-Y1 |
| Camera | ZED Mini (stereo, HD720 @ 30 fps) |
| Gripper | Dynamixel bus, right ID=0, left ID=1, 2 Mbaud |
| Network | Robot reachable at its gRPC address (e.g. `192.168.30.1:50051`) |

---

## Running

All commands are run from the `RBY1/` directory:

```bash
cd lilio_think/example/RBY1
```

### Real hardware (full AI)

```bash
python RBY1_tnr_2.py \
    --address       192.168.30.1:50051 \
    --config_vision ../config_vision_QM.json \
    --config_robot  ../robot_config.json \
    --skills_folder skills_lib/
```

`skills_lib/` is created automatically if it does not exist.

### Simulation (no robot, no camera required)

Stubs the entire rby1_sdk — useful for testing keyboard / state-machine logic:

```bash
python RBY1_tnr_2.py --sim --address dummy:50051
```

AI features (O / D / E) are disabled in simulation mode.

### Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `--address` | required | Robot gRPC address |
| `--model` | `a` | Robot model: `a`, `m`, or `ub` |
| `--arms` | `left,right` | Active arms: `left`, `right`, or `left,right` |
| `--sim` | off | Stub the SDK — no hardware needed |
| `--config_vision` | `../config_vision_QM.json` | Camera calibration JSON |
| `--config_robot` | `../robot_config.json` | Robot calibration JSON (T_WC) |
| `--skills_folder` | `skills_lib/` | Directory where skills are stored |

---

## Configuration files

Both files live in `lilio_think/example/` and are shared with the static example.

**`config_vision_QM.json`** — ZED Mini HD720 camera intrinsics and stereo baseline. Replace with your own calibration values (use `lilio_think/sensors/zed_camera.py → get_calibration()` to export them from the ZED SDK).

**`robot_config.json`** — Robot name, sampling rate, and `T_WC` (4×4 hand-eye matrix, robot base → camera frame). Obtain via `calibration/hand_eye_calibration_V2.py`.

---

## Workflow

### Step 1 — Capture the object ROI

Point the ZED camera at the object, then press **`O`**.

This grabs a stereo frame, lets you draw a bounding box, computes depth with S2M2, and registers the object as the reference for AI inference. Do this **once per session**, before recording or executing.

> Do not move the object until the current skill is fully recorded and saved.

### Step 2 — Record a skill via hand-guiding

1. Physically move the robot arm to a comfortable starting position.
2. Press **`R`** to start recording. The robot is back-drivable (hardware gravity compensation is always on).
3. Guide the arm through the motion you want to teach.
4. Use **`C`** / **`V`** to close / open the gripper during the motion if needed.
5. Press **`S`** to stop. The trajectory is post-processed into Cartesian waypoints automatically.

### Step 3 — Save the skill

Press **`D`** and enter a name when prompted (e.g. `pick_can`).

The trajectory is saved and associated with the current ROI reference.

> One recording = one atomic subtask. To teach "pick a can and place it in the trash", create two separate skills: `pick_can` and `place_can`.

### Step 4 — Execute

Press **`E`** and enter the skill name when prompted.

The system grabs a fresh camera frame, runs AI inference (depth estimation + pose estimation) to adapt the recorded trajectory to the current object position, then replays it as a stream of Cartesian waypoints.

---

## Keyboard Controls

| Key | Action |
|-----|--------|
| `O` | Capture object ROI (do this first every session) |
| `R` | Start recording (hand-guide the robot) |
| `S` | Stop recording and post-process trajectory |
| `D` | Save recorded trajectory as a named skill |
| `E` | Execute a saved skill (AI inference + replay) |
| `P` | Replay the last recorded trajectory (no AI) |
| `C` | Close gripper (active arms only) |
| `V` | Open gripper (active arms only) |
| `A` | Toggle active arm: left / right |
| `Q` | Quit |

---

## Architecture

```
RBY1Controller
├── rby1_sdk robot          — gRPC connection, power/servo/control manager
├── Dynamics (FK)           — joint positions -> Cartesian poses (on demand)
├── State callback @ 50 Hz  — reads joint positions; appends raw samples if recording
├── Gripper (Dynamixel)     — open/close via DynamixelBus
├── Command stream          — CartesianCommandBuilder waypoints sent at 50 Hz replay rate
└── pynput keyboard         — non-blocking key events, deferred to main thread
```

Recording captures joint positions at 50 Hz. On stop, FK is run over every sample to convert to Cartesian waypoints. Replay streams those waypoints back at the same rate to preserve motion timing.

The RB-Y1 resolves IK internally — no background IK thread is needed.

---

## Simulation Mode

`rby1_sdk_stub.py` is a drop-in stub that replaces `rby1_sdk` when `--sim` is passed. All SDK calls become no-ops or return sensible defaults (identity FK, zero joint positions, immediate convergence). Useful for testing the keyboard / state-machine logic without hardware.

---

## Example: Teaching a Pick-and-Place Task

Bad approach (do not do this):

```
Record: reach -> grasp can -> move -> release   # too long, hard to generalise
```

Correct approach:

```
Skill 1 "pick_can":   reach -> grasp can -> lift
Skill 2 "place_can":  move to trash -> release -> retract
```

Execute them in sequence. Each skill is independently learnable and reusable.
