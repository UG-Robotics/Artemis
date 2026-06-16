"""ToF distance sensors — the four wall distances (front/left/right/rear).

They share one I2C bus and the same default address, so bring-up uses each
sensor's XSHUT pin to enable them one at a time and give each its own address
(see hardware_config.Tof). VERIFY model/count first — our notes disagree
(3x VL53L0X vs 4x VL53L1X); this is written for the 4-sensor layout.
"""

from robot.hardware_config import Tof as TofConfig

# Adafruit CircuitPython stack; import lazily so this loads off-Pi.
try:
    import board  # type: ignore
    import busio  # type: ignore
    _I2C_AVAILABLE = True
except (ImportError, NotImplementedError):
    board = busio = None
    _I2C_AVAILABLE = False


class TofArray:
    """The wall-facing ToF sensors, keyed by position."""

    POSITIONS = ("front", "left", "right", "rear")

    def __init__(self, config=TofConfig):
        self.config = config
        self._sensors = {}
        # TODO: bring each sensor up via XSHUT and assign its address.

    def read_all(self) -> dict:
        """{'front','left','right','rear'} distances in mm (TOF_MAX_RANGE = no wall)."""
        raise NotImplementedError("we'll read the four sensors here on the Pi")

    def close(self) -> None:
        if _I2C_AVAILABLE:
            pass  # TODO: stop ranging
