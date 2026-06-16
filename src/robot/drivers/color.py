"""Downward color sensor (TCS34725) — reads the orange/blue section lines.

The controller counts sections and spots corner entry from this, so reliable
orange/blue/none on the real mat is what matters — purely a calibration job.
"""

from robot.hardware_config import Color as ColorConfig

try:
    import board  # type: ignore
    import adafruit_tcs34725  # type: ignore
    _I2C_AVAILABLE = True
except (ImportError, NotImplementedError):
    board = adafruit_tcs34725 = None
    _I2C_AVAILABLE = False


class ColorSensor:
    """Floor line detector: 'orange', 'blue', or None."""

    def __init__(self, config=ColorConfig):
        self.config = config
        # TODO: construct the sensor, set integration time + gain.

    def detect(self) -> "str | None":
        """Classify the floor as 'orange', 'blue', or None.

        TODO: read RGB, compare against thresholds we calibrate on the real mat.
        """
        raise NotImplementedError("we'll classify the line color here on the Pi")

    def close(self) -> None:
        if _I2C_AVAILABLE:
            pass
