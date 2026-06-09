"""
Shared helpers for the OpenArm TNR example.

All lilio_think server calls go through the Lilio SDK Session object instead
of raw HTTP requests, making this a self-contained drop-in replacement for
lilio_move.OpenArm_control.utils.
"""
import sys
import queue
import termios

import cv2
import numpy as np
import placo

from lilio import LilioError


# ── Image ──────────────────────────────────────────────────────────────────────

def resize_image(img, desired_size):
    """Resize image, choosing interpolation to avoid aliasing."""
    w = img.shape[1]
    interp = cv2.INTER_AREA if desired_size[0] < w else cv2.INTER_LINEAR
    return cv2.resize(img, desired_size, interpolation=interp)


# ── Input ──────────────────────────────────────────────────────────────────────

def prompt_input(prompt: str) -> str:
    """Flush stdin buffer and prompt for input."""
    termios.tcflush(sys.stdin, termios.TCIFLUSH)
    sys.stdout.flush()
    return input(prompt)


# ── Vision / ROI ───────────────────────────────────────────────────────────────

def capture_roi(camera, session) -> tuple:
    """
    Let the user draw a bounding box on the live camera feed, then send the
    ROI to lilio_think via the SDK session.

    Returns (success: bool, crop_jpg: bytes | None).
    """
    if camera is None:
        print("[ROI] Camera not available.")
        return False, None

    bbox, i_L, i_R = camera.get_roi()

    crop_jpg = None
    try:
        x1, y1, x2, y2 = (int(v) for v in bbox)
        crop = i_L[y1:y2, x1:x2]
        if crop.size > 0:
            _, buf = cv2.imencode(".jpg", cv2.cvtColor(crop, cv2.COLOR_RGB2BGR),
                                  [cv2.IMWRITE_JPEG_QUALITY, 85])
            crop_jpg = buf.tobytes()
    except Exception as e:
        print(f"[ROI] Could not extract crop: {e}")

    session.set_roi(i_L, i_R, list(bbox))
    print("[ROI] Object reference captured.")
    return True, crop_jpg


# ── Skill / demo ───────────────────────────────────────────────────────────────

def save_skill_demo(traj: list, session,
                    skill_name: str = None,
                    guideline: str = None) -> None:
    """
    Prompt for a name and description, then upload the trajectory to
    lilio_think as a named skill via the SDK session.
    """
    if not traj:
        print("[SKILL] No trajectory to save.")
        return
    if skill_name is None:
        skill_name = prompt_input("Skill name: ")
    if guideline is None:
        guideline = prompt_input("What does this skill do? ")

    traj_array = np.array(traj, dtype=object)
    session.save_skill(skill_name, traj_array)
    print(f"[SKILL] '{skill_name}' saved.")


# ── Plan streaming ─────────────────────────────────────────────────────────────

def stream_action_plans(l_img, r_img, skill_name: str,
                        plan_queue: queue.Queue, session,
                        robot_model=None, q_current=None,
                        on_done=None) -> None:
    """
    Request an action plan from lilio_think and push it into plan_queue.
    Optionally runs a reachability check before accepting the plan.
    """
    try:
        plan = session.get_action_plan(skill_name, l_img, r_img)
        if not plan:
            print("[SKILL] No plan returned — object not detected or pose estimation failed.")
            return

        if robot_model is not None and q_current is not None:
            print("[REACHABILITY] Checking trajectory...")
            if not robot_model.check_traj_reachability(q_current, plan):
                print("[REACHABILITY] Trajectory NOT reachable — plan discarded.")
                return
            print("[REACHABILITY] Trajectory OK.")

        plan_queue.put(plan)

    except LilioError as e:
        print(f">>> [ERROR] Server error: {e}")
    except Exception as e:
        print(f">>> [ERROR] {e}")
    finally:
        if on_done:
            on_done()


# ── Cartesian interpolation ────────────────────────────────────────────────────

def move_to_cart_threaded(target_L, target_R, init_L, init_R,
                          gripper, cmd_lock, set_cmd,
                          dt: float, tick, duration: float = 3.0) -> None:
    """
    Cartesian-space spline interpolation synchronised with the control-loop
    tick event so every command frame is consumed exactly once.
    """
    spline = placo.CubicSpline()
    spline.add_point(0.0, 0.0, 0.0)
    spline.add_point(duration, 1.0, 0.0)
    t = 0.0
    while t < duration:
        if not tick.wait(timeout=1.0):
            break
        tick.clear()
        alpha = spline.pos(t)
        iL = placo.interpolate_frames(init_L, target_L, alpha)
        iR = placo.interpolate_frames(init_R, target_R, alpha)
        with cmd_lock:
            set_cmd((iL, iR, gripper[0], gripper[1]))
        t += dt


# ── Tracking plot ──────────────────────────────────────────────────────────────

def plot_tracking(t_arr, q_des_arr, q_act_arr, title: str) -> None:
    """Save a 7-joint tracking plot to plots/<title>.png."""
    import os
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    error_arr = q_des_arr - q_act_arr
    fig, axes = plt.subplots(7, 1, figsize=(12, 18), sharex=True)
    fig.suptitle(f"PID Joint Tracking — {title}", fontsize=13)
    for j, ax in enumerate(axes):
        ax.plot(t_arr, q_des_arr[:, j], "k--", linewidth=1.2, label="Desired")
        ax.plot(t_arr, q_act_arr[:, j], "tab:blue", linewidth=1.2, label="Actual")
        ax.fill_between(t_arr, 0, error_arr[:, j],
                        color="tab:red", alpha=0.25, label="Error")
        ax.set_ylabel("rad", fontsize=8)
        ax.set_title(f"Joint {j + 1}", fontsize=9, loc="left")
        ax.legend(fontsize=7, loc="upper right", ncol=3)
        ax.grid(True, alpha=0.3)
    axes[-1].set_xlabel("Time (s)")
    plt.tight_layout()
    os.makedirs("plots", exist_ok=True)
    path = f"plots/tracking_{title.replace(' ', '_')}.png"
    plt.savefig(path, dpi=150)
    plt.close(fig)
    print(f"[PLOT] Saved → {path}")
    for j in range(7):
        rms = np.sqrt(np.mean(error_arr[:, j] ** 2))
        print(f"  Joint {j + 1}: {rms * 1000:.2f} mrad")
