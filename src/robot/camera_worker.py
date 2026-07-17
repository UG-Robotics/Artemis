"""Background camera perception for competition runs.

Owns the one allowed Picamera2 instance (do NOT run the web dashboard's camera
stream at the same time — picamera2 deadlocks on concurrent owners; stop
artemis-web or export ARTEMIS_NO_CAMERA=1 for it) and publishes, at ~10 Hz:

    worker.vision_heading   float | None   wall-base heading (core/vision_heading)
    worker.line_color       'orange' | 'blue' | None   corner line (core/vision_lines)

Reads are lock-free attribute grabs — the control loop at 30 Hz just picks up
the latest values; None means "camera off / nothing seen / estimate invalid".

Off-Pi (no picamera2) construction succeeds and everything stays None, so the
same launcher code runs on a dev machine.
"""

import threading
import time

from core.vision_heading import estimate_heading
from core.vision_lines import detect_line_color
from robot.hardware_config import Camera as CameraConfig

try:
    from picamera2 import Picamera2  # type: ignore
    _PI_CAM = True
except (ImportError, NotImplementedError):
    Picamera2 = None
    _PI_CAM = False


class CameraWorker:
    """Latest-value camera perception at PERCEPTION_HZ, off the control path."""

    PERCEPTION_HZ = 10
    SIZE = (640, 480)

    def __init__(self):
        self.vision_heading = None
        self.line_color = None
        self.available = False
        self._running = False
        # Set to a RunLogger to record every perception frame (line/heading/
        # pillars + the frame that produced a detection change). Attached at GO
        # so the button-wait isn't logged; None = no logging.
        self.logger = None
        if not _PI_CAM:
            return
        from libcamera import Transform  # ships with picamera2 on the Pi
        self._cam = Picamera2()
        transform = Transform(hflip=int(CameraConfig.HFLIP),
                              vflip=int(CameraConfig.VFLIP))
        # "BGR888" delivers RGB numpy arrays (picamera2 names are reversed) —
        # same lesson as web/camera_feed.py.
        self._cam.configure(self._cam.create_video_configuration(
            main={"size": self.SIZE, "format": "BGR888"}, transform=transform))
        self._cam.start()
        self.available = True
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        period = 1.0 / self.PERCEPTION_HZ
        while self._running:
            t0 = time.monotonic()
            try:
                frame = self._cam.capture_array()
                self.vision_heading = estimate_heading(frame)
                self.line_color = detect_line_color(frame)
                logger = self.logger
                if logger is not None:
                    # pillars: [] until detect_pillars lands (obstacle challenge)
                    logger.log_camera(self.line_color, self.vision_heading,
                                      pillars=[], frame=frame)
            except Exception:
                self.vision_heading = None
                self.line_color = None
            dt = time.monotonic() - t0
            if dt < period:
                time.sleep(period - dt)

    def close(self):
        self._running = False
        if self.available:
            try:
                self._cam.stop()
                self._cam.close()  # release the device — stop() alone keeps it
                                   # acquired and the next owner gets "busy"
            except Exception:
                pass
