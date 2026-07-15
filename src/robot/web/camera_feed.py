"""Camera frames for the web MJPEG stream.

Real Pi camera (picamera2) if present, else a rendered placeholder so the UI works
on a dev machine. Both expose frames() -> generator of JPEG bytes.
"""

import io
import os
import threading
import time

from robot.hardware_config import Camera as CameraConfig

try:
    from picamera2 import Picamera2  # type: ignore
    _PI_CAM = True
except Exception:
    Picamera2 = None
    _PI_CAM = False

try:
    from PIL import Image, ImageDraw  # type: ignore
    _PIL = True
except Exception:
    _PIL = False


class PlaceholderSource:
    """Renders a dev frame (grid, crosshair, live teleop readout) at a steady rate."""

    def __init__(self, teleop=None, size=(640, 480), fps=12):
        self.teleop = teleop
        self.w, self.h = size
        self.period = 1.0 / fps

    def frames(self):
        while True:
            yield self._frame()
            time.sleep(self.period)

    def _frame(self) -> bytes:
        img = Image.new("RGB", (self.w, self.h), (18, 20, 24))
        d = ImageDraw.Draw(img)
        for x in range(0, self.w, 40):
            d.line([(x, 0), (x, self.h)], fill=(28, 30, 36))
        for y in range(0, self.h, 40):
            d.line([(0, y), (self.w, y)], fill=(28, 30, 36))
        cx, cy = self.w // 2, self.h // 2
        d.line([(cx - 22, cy), (cx + 22, cy)], fill=(70, 80, 92))
        d.line([(cx, cy - 22), (cx, cy + 22)], fill=(70, 80, 92))
        # ASCII only: PIL's default bitmap font is latin-1 and dies on em-dashes.
        d.text((12, 10), "ARTEMIS - DEV CAMERA (no hardware)", fill=(120, 200, 255))
        d.text((12, 28), time.strftime("%H:%M:%S"), fill=(150, 160, 170))
        st = self.teleop.state() if self.teleop else {}
        d.text((12, self.h - 42),
               f"steer {st.get('steering_deg', 0)} deg", fill=(180, 255, 190))
        d.text((12, self.h - 24),
               f"speed {st.get('speed_frac', 0)}", fill=(180, 255, 190))
        buf = io.BytesIO()
        img.save(buf, format="JPEG", quality=70)
        return buf.getvalue()


class PiCameraSource:
    """OV5647 via picamera2, JPEG-encoded by one shared capture thread.

    A single background thread captures + encodes into a latest-frame slot; every
    connected client streams that slot. This decouples capture rate from client
    count and — crucially — never runs two concurrent `capture_array()` calls on
    the one Picamera2 (which deadlocks and freezes the stream when a browser
    reconnects or a second tab opens).
    """

    def __init__(self, size=(640, 480), fps=20):
        from PIL import Image  # encode frames; Pillow is light on the Pi
        from libcamera import Transform  # ships with picamera2 on the Pi
        self._Image = Image
        self.cam = Picamera2()
        # Camera is mounted upside down — flip in-sensor (180°) so the stream
        # and any downstream detection see an upright frame at zero CPU cost.
        transform = Transform(hflip=int(CameraConfig.HFLIP),
                              vflip=int(CameraConfig.VFLIP))
        # picamera2 format names are byte-order and REVERSED from what they read:
        # "RGB888" delivers a BGR numpy array, "BGR888" delivers RGB. PIL/JPEG want
        # RGB, so request BGR888 — otherwise red<->blue swap (skin renders blue).
        self.cam.configure(self.cam.create_video_configuration(
            main={"size": size, "format": "BGR888"}, transform=transform))
        self.cam.start()
        self.period = 1.0 / fps

        self._latest = None
        self._seq = 0                     # bumped per frame so clients detect "new"
        self._cond = threading.Condition()
        self._running = True
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()

    def _capture_loop(self):
        while self._running:
            arr = self.cam.capture_array()
            buf = io.BytesIO()
            self._Image.fromarray(arr).save(buf, format="JPEG", quality=70)
            with self._cond:
                self._latest = buf.getvalue()
                self._seq += 1
                self._cond.notify_all()
            time.sleep(self.period)

    def frames(self):
        last_seq = -1
        while True:
            with self._cond:
                # Wait for a newer frame; time out periodically so a stalled
                # capture doesn't wedge the client connection silently.
                self._cond.wait_for(lambda: self._seq != last_seq, timeout=2.0)
                frame, last_seq = self._latest, self._seq
            if frame is not None:
                yield frame

    def close(self):
        self._running = False
        try:
            self.cam.stop()
        except Exception:
            pass


def make_camera_source(teleop=None):
    """Pi camera if available, else the dev placeholder. Needs Pillow for the latter.

    Set ARTEMIS_NO_CAMERA=1 to skip the Pi camera entirely and use the placeholder.
    Needed when the OV5647/libcamera stack wedges on init: a hung driver call sits
    in uninterruptible I/O and never raises, so the try/except below can't fall
    back — it just blocks the whole app before it binds the web port.
    """
    no_cam = os.environ.get("ARTEMIS_NO_CAMERA", "").lower() in ("1", "true", "yes")
    if _PI_CAM and not no_cam:
        try:
            return PiCameraSource()
        except Exception as exc:
            print(f"PiCameraSource failed ({exc!r}); falling back to placeholder")
    if _PIL:
        return PlaceholderSource(teleop=teleop)
    raise RuntimeError("No camera: install picamera2 (Pi) or Pillow (dev placeholder).")
