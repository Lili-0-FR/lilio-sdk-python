import base64
import contextlib
from typing import Generator

import cv2
import numpy as np
import requests

from .exceptions import LilioError

DEFAULT_BASE_URL = "https://api.lili-o.com"


class Session:
    def __init__(self, client: "LilioClient", session_id: str):
        self._client = client
        self._session_id = session_id

    def set_roi(self, left_img: np.ndarray, right_img: np.ndarray, box: list[int]) -> dict:
        return self._client._post(
            "/robot/roi",
            params={"session_id": self._session_id},
            json={
                "left_image": _encode_image(left_img),
                "right_image": _encode_image(right_img),
                "box": box,
            },
        )

    def save_skill(self, skill_name: str, trajectory: np.ndarray) -> dict:
        return self._client._post(
            "/robot/save_skill",
            params={"session_id": self._session_id},
            json={
                "skill_name": skill_name,
                "trajectories": _serialize_trajectory(trajectory),
            },
        )

    def get_action_plan(
        self, skill_name: str, left_img: np.ndarray, right_img: np.ndarray
    ) -> list | None:
        result = self._client._post(
            "/robot/action_plans_stream",
            params={"session_id": self._session_id},
            json={
                "skill_name": skill_name,
                "left_image": _encode_image(left_img),
                "right_image": _encode_image(right_img),
            },
        )
        return result.get("plan")


class LilioClient:
    def __init__(self, api_key: str, base_url: str = DEFAULT_BASE_URL):
        self._base_url = base_url.rstrip("/")
        self._headers = {"Authorization": f"Bearer {api_key}"}

    @contextlib.contextmanager
    def session(self, camera_calibration: dict) -> Generator[Session, None, None]:
        resp = self._post("/session", json={"camera_calibration": camera_calibration})
        session_id = resp["session_id"]
        try:
            yield Session(self, session_id)
        finally:
            self._delete(f"/session/{session_id}")

    def list_skills(self) -> list[dict]:
        return self._get("/robot/action_plan")

    def _get(self, path: str, **kwargs) -> dict:
        resp = requests.get(f"{self._base_url}{path}", headers=self._headers, **kwargs)
        _check(resp)
        return resp.json()

    def _post(self, path: str, **kwargs) -> dict:
        resp = requests.post(f"{self._base_url}{path}", headers=self._headers, **kwargs)
        _check(resp)
        return resp.json()

    def _delete(self, path: str, **kwargs) -> None:
        resp = requests.delete(f"{self._base_url}{path}", headers=self._headers, **kwargs)
        _check(resp)


def _check(resp: requests.Response) -> None:
    if not resp.ok:
        try:
            detail = resp.json().get("detail", resp.text)
        except Exception:
            detail = resp.text
        raise LilioError(resp.status_code, detail)


def _encode_image(img: np.ndarray) -> str:
    bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    _, buf = cv2.imencode(".png", bgr)
    return base64.b64encode(buf).decode("utf-8")


def _serialize_trajectory(traj: np.ndarray) -> list:
    result = []
    for waypoint in traj:
        arms = waypoint["arms"]
        result.append({
            "arm": arms[0] if isinstance(arms, (list, np.ndarray)) else arms,
            "TL": waypoint["TL"].tolist() if isinstance(waypoint["TL"], np.ndarray) else waypoint["TL"],
            "TR": waypoint["TR"].tolist() if isinstance(waypoint["TR"], np.ndarray) else waypoint["TR"],
            "gl": float(waypoint["gl"]),
            "gr": float(waypoint["gr"]),
        })
    return result
