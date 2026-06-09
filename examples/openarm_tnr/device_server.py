"""
Local device server for the lilio_dashboard.

Exposes the OpenArmController state over HTTP + WebSocket so the dashboard
can connect to this machine as a device.

Endpoints
---------
  WS   /state                      — streams robot_state + app_status messages
  GET  /camera/{left|right}/stream — MJPEG camera feed
  GET  /apps                       — running app list
  GET  /apps/{name}/commands       — available commands for an app
  POST /apps/{name}/command        — trigger a command  { cmd, skill_name?, guideline? }
"""
import asyncio
import json
import threading

import cv2
import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import StreamingResponse

APP_NAME = "openarm_tnr"

COMMANDS = [
    {"cmd": "h", "label": "Hand guide",    "state": None},
    {"cmd": "o", "label": "Capture ROI",   "state": None},
    {"cmd": "d", "label": "Save skill",    "state": None},
    {"cmd": "e", "label": "Execute skill", "state": None},
    {"cmd": "p", "label": "Replay",        "state": None},
    {"cmd": "q", "label": "Quit",          "state": None},
]


class DeviceServer:
    """
    Background HTTP/WebSocket server that exposes an OpenArmController
    to the lilio_dashboard.

    Usage::

        server = DeviceServer(controller, port=8765)
        server.start()   # non-blocking, runs in a daemon thread
    """

    def __init__(self, controller, port: int = 8765):
        self._ctrl = controller
        self._port = port
        self._app  = self._build_app()
        self._thread = threading.Thread(target=self._run, daemon=True, name="device-server")

    def start(self) -> None:
        self._thread.start()
        print(f"[DeviceServer] Listening on http://0.0.0.0:{self._port}")

    # ── Internal ────────────────────────────────────────────────────────────

    def _run(self) -> None:
        uvicorn.run(self._app, host="0.0.0.0", port=self._port, log_level="warning")

    def _app_state(self) -> str:
        c = self._ctrl
        if c._soft_mode and c.recording:
            return "recording"
        if c._soft_mode:
            return "hand_guide"
        if c._replaying:
            return "replaying"
        return "idle"

    def _robot_state_msg(self) -> dict:
        """Build a robot_state WebSocket message from the current controller state."""
        c   = self._ctrl
        obs = c.OAI.get_state()
        q   = obs["q_state"]

        # Re-use the EE poses already computed by the control loop — no extra IK call.
        with c._cmd_lock:
            T_L = c._last_ee_L
            T_R = c._last_ee_R

        return {
            "type":     "robot_state",
            "joints":   {"left": q[7:14].tolist(), "right": q[16:23].tolist()},
            "torques":  {},
            "ee_poses": {"left": T_L.tolist(), "right": T_R.tolist()},
        }

    def _app_status_msg(self) -> dict:
        return {
            "type": "app_status",
            "apps": [{"name": APP_NAME,
                      "running":   not self._ctrl.stop_program,
                      "app_state": self._app_state()}],
        }

    # ── FastAPI app ──────────────────────────────────────────────────────────

    def _build_app(self) -> FastAPI:
        app = FastAPI(title="Lilio Device Server")
        app.add_middleware(
            CORSMiddleware,
            allow_origins=["*"],
            allow_methods=["*"],
            allow_headers=["*"],
        )

        # ── WebSocket /state ────────────────────────────────────────────────

        @app.websocket("/state")
        async def state_ws(ws: WebSocket):
            await ws.accept()
            try:
                while not self._ctrl.stop_program:
                    try:
                        await ws.send_text(json.dumps(self._robot_state_msg()))
                        await ws.send_text(json.dumps(self._app_status_msg()))
                    except Exception:
                        break
                    await asyncio.sleep(0.05)   # 20 Hz
            except WebSocketDisconnect:
                pass

        # ── MJPEG camera streams ────────────────────────────────────────────

        @app.get("/camera/{side}/stream")
        async def camera_stream(side: str):
            async def generate():
                while not self._ctrl.stop_program:
                    try:
                        frames     = self._ctrl.camera.get_frames()
                        frame      = frames[0] if side == "left" else frames[1]
                        if frame is not None:
                            _, buf = cv2.imencode(
                                ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
                            yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
                                   + buf.tobytes() + b"\r\n")
                    except Exception:
                        pass
                    await asyncio.sleep(1 / 15)   # 15 fps

            return StreamingResponse(
                generate(),
                media_type="multipart/x-mixed-replace; boundary=frame",
            )

        # ── App list ────────────────────────────────────────────────────────

        @app.get("/apps")
        async def list_apps():
            return [{"name": APP_NAME,
                     "running":   not self._ctrl.stop_program,
                     "app_state": self._app_state()}]

        # ── App commands (metadata) ─────────────────────────────────────────

        @app.get("/apps/{name}/commands")
        async def get_commands(name: str):
            return COMMANDS

        # ── Send a command ──────────────────────────────────────────────────

        @app.post("/apps/{name}/command")
        async def send_command(name: str, body: dict):
            """
            Trigger a controller action from the dashboard.

            Body fields
            -----------
            cmd         : str   — one of h / o / d / e / p / q
            skill_name  : str   — required for cmd=d or cmd=e
            guideline   : str   — optional description for cmd=d
            """
            cmd        = str(body.get("cmd", "")).lower()
            skill_name = body.get("skill_name")
            guideline  = body.get("guideline")
            c          = self._ctrl

            if cmd == "h":
                if not c._soft_mode:
                    c.hand_guide = True
                    c._soft_mode = True
                    with c._cmd_lock:
                        c._inactive_hold_q = c.OAI.get_state()["q_state"].copy()
                    c.OAI.set_soft(arms=c.arms)

            elif cmd == "o":
                c._roi_requested = True

            elif cmd == "d":
                if skill_name:
                    c._pending_skill_name = skill_name
                    c._pending_guideline  = guideline
                c._save_demo_requested = True

            elif cmd == "e":
                if skill_name:
                    c._pending_skill_name = skill_name
                c._execute_requested = True

            elif cmd == "p":
                if c.traj:
                    c._plan_queue.put(c.traj)

            elif cmd == "q":
                c.stop_program = True

            else:
                return {"error": f"Unknown command: {cmd!r}"}

            return {"ok": True, "cmd": cmd}

        return app
