"""Flight recorder for a competition run — everything needed to debug it later.

One call at GO creates a timestamped run directory under ~/artemis/logs/ and
writes three correlated streams (all share t = seconds since GO, so a camera
event lines up with the control state at that instant):

  sensors.csv   one row per control tick (~30 Hz): the four ToF distances, IMU
                heading, commanded steering/speed, controller state + turn count,
                and the camera's latest line/heading as the controller saw them.
  camera.jsonl  one row per camera frame (~10 Hz) from the perception thread:
                line colour, vision heading, pillars, and the frame filename if
                one was snapshotted for this row.
  frames/*.jpg  the actual camera image, saved on every DETECTION CHANGE (so you
                can see *why* it called orange / a red pillar) plus a periodic
                keyframe, capped so a run can't fill the SD card.
  meta.json     mode, direction, width, code revision, and a final summary.

Writers are lock-free: the control thread owns sensors.csv, the camera thread
owns camera.jsonl + frames/. Both files are line-buffered, so a crash or power
cut keeps everything up to the last completed line. A whole 3-min run is a few
hundred KB of text plus a handful of MB of frames.

Degrades cleanly off-Pi (writes under ./logs, skips frames if Pillow is absent).
"""

import json
import os
import subprocess
import time
from datetime import datetime


def _code_revision(src_dir: str) -> str:
    """Short git rev if the tree is a repo (the Mac); else the deploy marker or
    'unknown' (the Pi's /home/pi/artemis is a plain copy, not a git repo)."""
    try:
        out = subprocess.run(
            ["git", "-C", src_dir, "rev-parse", "--short", "HEAD"],
            capture_output=True, text=True, timeout=2)
        if out.returncode == 0:
            return out.stdout.strip()
    except Exception:
        pass
    marker = os.path.join(src_dir, "DEPLOYED_REV")
    if os.path.exists(marker):
        try:
            return open(marker).read().strip()
        except Exception:
            pass
    return "unknown"


class RunLogger:
    SENSOR_HEADER = ("t,tof_front,tof_left,tof_right,tof_rear,imu_heading,"
                     "steering,speed,state,turns,cam_line,cam_heading\n")

    def __init__(self, challenge: str, mode: str, meta: dict = None,
                 root: str = None, save_frames: bool = True,
                 keyframe_period_s: float = 2.0, max_frames: int = 400):
        self._t0 = time.monotonic()
        root = root or os.path.expanduser("~/artemis/logs")
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.dir = os.path.join(root, f"{challenge}_{mode}_{stamp}")
        os.makedirs(os.path.join(self.dir, "frames"), exist_ok=True)

        self._sensors_f = open(os.path.join(self.dir, "sensors.csv"), "w", buffering=1)
        self._sensors_f.write(self.SENSOR_HEADER)
        self._camera_f = open(os.path.join(self.dir, "camera.jsonl"), "w", buffering=1)

        self.save_frames = save_frames
        self.keyframe_period_s = keyframe_period_s
        self.max_frames = max_frames
        self._frames_saved = 0
        self._last_keyframe_t = -1e9
        self._last_det = object()   # sentinel != any real detection
        self._Image = None

        src_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        self._meta = dict(meta or {})
        self._meta.update(challenge=challenge, mode=mode,
                          started=datetime.now().isoformat(timespec="seconds"),
                          code_revision=_code_revision(src_dir))
        self._write_meta()
        print(f"[logger] recording to {self.dir}")

    def t(self) -> float:
        return time.monotonic() - self._t0

    # -- control thread ----------------------------------------------------

    def log_sensors(self, sensors, steering, speed, state, turns,
                    cam_line=None, cam_heading=None) -> None:
        s = sensors
        ch = "" if cam_heading is None else f"{cam_heading:.2f}"
        self._sensors_f.write(
            f"{self.t():.3f},{s.tof_front:.0f},{s.tof_left:.0f},{s.tof_right:.0f},"
            f"{s.tof_rear:.0f},{s.imu_heading:.2f},{steering:.1f},{speed:.2f},"
            f"{state},{turns},{cam_line or ''},{ch}\n")

    # -- camera thread -----------------------------------------------------

    def log_camera(self, line_color, vision_heading, pillars=None, frame=None) -> None:
        t = self.t()
        pillars = pillars or []
        det = (line_color, tuple(sorted(p.get("color", "?") for p in pillars)))
        frame_name = None
        if frame is not None and self.save_frames and self._frames_saved < self.max_frames:
            changed = det != self._last_det
            keyframe = (t - self._last_keyframe_t) >= self.keyframe_period_s
            if changed or keyframe:
                frame_name = self._save_frame(t, frame, "det" if changed else "key")
                self._last_keyframe_t = t
        self._last_det = det
        rec = {"t": round(t, 3), "line": line_color,
               "vision_heading": None if vision_heading is None else round(vision_heading, 2),
               "pillars": pillars, "frame": frame_name}
        self._camera_f.write(json.dumps(rec) + "\n")

    def _save_frame(self, t: float, frame, tag: str):
        if self._Image is None:
            try:
                from PIL import Image
                self._Image = Image
            except ImportError:
                self.save_frames = False
                return None
        name = os.path.join("frames", f"{t:07.2f}_{tag}.jpg")
        try:
            self._Image.fromarray(frame).save(os.path.join(self.dir, name), quality=60)
            self._frames_saved += 1
            return name
        except Exception:
            return None

    # -- teardown ----------------------------------------------------------

    def close(self, summary: dict = None) -> None:
        if summary:
            self._meta["summary"] = summary
        self._meta["frames_saved"] = self._frames_saved
        self._meta["duration_s"] = round(self.t(), 1)
        self._write_meta()
        for f in (self._sensors_f, self._camera_f):
            try:
                f.close()
            except Exception:
                pass
        print(f"[logger] saved {self.dir} "
              f"({self._frames_saved} frames, {self._meta['duration_s']}s)")

    def _write_meta(self) -> None:
        try:
            with open(os.path.join(self.dir, "meta.json"), "w") as f:
                json.dump(self._meta, f, indent=2)
        except Exception:
            pass
