"""Steering servo: maps a steering angle (deg, +=right) to a servo pulse.

The mapping depends on the built linkage, so the constants in
hardware_config.Servo have to be measured on the real car.
"""

from robot.hardware_config import Servo as ServoConfig
from core.config import MAX_STEERING_ANGLE

try:
    import RPi.GPIO as GPIO  # type: ignore
    _GPIO_AVAILABLE = True
except (ImportError, RuntimeError):
    GPIO = None
    _GPIO_AVAILABLE = False


class ServoDriver:
    """Front-wheel steering servo."""

    def __init__(self, config=ServoConfig):
        self.config = config
        # TODO: set up the PWM signal pin and centre the steering.

    def set_angle(self, angle_deg: float) -> None:
        """Steer to `angle_deg` (clamp to +/- MAX_STEERING_ANGLE, then map to a pulse)."""
        raise NotImplementedError("we'll output the servo pulse here on the Pi")

    def center(self) -> None:
        """Wheels straight ahead."""
        self.set_angle(0.0)

    def close(self) -> None:
        if _GPIO_AVAILABLE:
            pass  # TODO: stop the servo PWM
