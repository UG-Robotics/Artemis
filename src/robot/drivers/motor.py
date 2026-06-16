"""Drive motor: N20 gearmotor via a TB6612FNG H-bridge.

Turns a speed fraction (-1..1) into PWM + direction, and reads the encoder for
distance travelled (which the controller uses).
"""

from robot.hardware_config import Motor as MotorPins

# Pi-only GPIO lib; import lazily so this loads on a dev machine.
try:
    import RPi.GPIO as GPIO  # type: ignore
    _GPIO_AVAILABLE = True
except (ImportError, RuntimeError):
    GPIO = None
    _GPIO_AVAILABLE = False


class MotorDriver:
    """Drive motor with encoder odometry."""

    def __init__(self, config=MotorPins):
        self.config = config
        self._distance_mm = 0.0
        # TODO: set up PWM/direction/STBY pins + encoder edge callbacks.

    def set_speed_fraction(self, fraction: float) -> None:
        """Drive at `fraction` of full speed; negative reverses."""
        raise NotImplementedError("we'll drive PWM + direction here on the Pi")

    def stop(self) -> None:
        """Cut drive power."""
        raise NotImplementedError("we'll cut drive power here on the Pi")

    @property
    def distance_traveled(self) -> float:
        """Forward distance in mm from the encoder. TODO: convert counts to mm."""
        return self._distance_mm

    def close(self) -> None:
        if _GPIO_AVAILABLE:
            pass  # TODO: stop PWM, clean up pins
