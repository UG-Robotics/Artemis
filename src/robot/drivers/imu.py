"""IMU heading from an MPU6050.

The controller steers toward cardinals and measures turns from heading, so we
need a stable angle in degrees [0, 360) — simplest is integrating the gyro Z rate.
"""

from robot.hardware_config import Imu as ImuConfig

try:
    import smbus2  # type: ignore
    _I2C_AVAILABLE = True
except ImportError:
    smbus2 = None
    _I2C_AVAILABLE = False


class ImuDriver:
    """Heading source for the controller."""

    def __init__(self, config=ImuConfig):
        self.config = config
        self._heading = 0.0
        # TODO: wake the MPU6050, configure the gyro, measure the rest bias.

    def reset_heading(self, heading: float = 0.0) -> None:
        """Call this once at the start so 'forward' is a known value."""
        self._heading = heading % 360

    def heading(self) -> float:
        """Current heading in degrees [0, 360). TODO: integrate the gyro per tick."""
        raise NotImplementedError("we'll integrate the gyro here on the Pi")

    def close(self) -> None:
        if _I2C_AVAILABLE:
            pass
