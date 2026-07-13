"""Forward camera (OV5647) — finds red/green pillars for the obstacle challenge.

Returns pillars in the shape the controller wants (SensorReading.pillars_visible):
dicts with 'color', 'angle' (deg, +=right) and 'distance' (mm), nearest first.
Not needed for the open challenge.
"""

from robot.hardware_config import Camera as CameraConfig

try:
    from picamera2 import Picamera2  # type: ignore
    import cv2  # type: ignore
    _CAMERA_AVAILABLE = True
except (ImportError, NotImplementedError):
    Picamera2 = cv2 = None
    _CAMERA_AVAILABLE = False


class Camera:
    """Pillar detector for the obstacle challenge."""

    def __init__(self, config=CameraConfig):
        self.config = config
        # TODO: start Picamera2 at config.RESOLUTION/FRAMERATE. Apply the same
        # upside-down flip as camera_feed.py:
        #   Transform(hflip=int(config.HFLIP), vflip=int(config.VFLIP))
        # or the pillar angles come out mirrored/inverted.

    def detect_pillars(self) -> list:
        """Visible pillars, nearest first ([] when none).

        TODO: HSV-threshold red/green, find contours, estimate angle from x in
        frame and distance from apparent height. Calibrate the HSV ranges on-site.
        """
        raise NotImplementedError("we'll detect pillars here on the Pi")

    def close(self) -> None:
        if _CAMERA_AVAILABLE:
            pass  # TODO: stop the camera
