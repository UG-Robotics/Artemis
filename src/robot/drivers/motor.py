"""Drive motor: 12V encoder gearmotor via a TB6612FNG H-bridge (B channel).

Turns a speed fraction (-1..1) into PWM + direction, and reads the encoder for
distance travelled (which the controller uses).

Uses pigpio (daemon enabled at boot) — same stack as the servo driver, and its
callback machinery gives clean encoder edge counting.
"""

from robot.hardware_config import Motor as MotorPins

try:
    import pigpio  # type: ignore
    _PIGPIO_AVAILABLE = True
except ImportError:
    pigpio = None
    _PIGPIO_AVAILABLE = False

_PWM_RANGE = 255


class MotorDriver:
    """Drive motor with encoder odometry."""

    def __init__(self, config=MotorPins):
        self.config = config
        self._distance_mm = 0.0
        self._edges = 0
        self._direction = 1
        self._pi = None
        self._encoder_cb = None
        if not _PIGPIO_AVAILABLE:
            return
        self._pi = pigpio.pi()
        if not self._pi.connected:
            raise RuntimeError("pigpiod is not running (sudo systemctl start pigpiod)")
        for pin in (config.PIN_IN1, config.PIN_IN2, config.PIN_STBY):
            self._pi.set_mode(pin, pigpio.OUTPUT)
            self._pi.write(pin, 0)
        self._pi.set_PWM_frequency(config.PIN_PWM, config.PWM_FREQUENCY_HZ)
        self._pi.set_PWM_range(config.PIN_PWM, _PWM_RANGE)
        self._pi.set_PWM_dutycycle(config.PIN_PWM, 0)
        self._pi.write(config.PIN_STBY, 1)
        if config.PIN_ENC_B is not None:
            self._pi.set_mode(config.PIN_ENC_B, pigpio.INPUT)
            self._encoder_cb = self._pi.callback(
                config.PIN_ENC_B, pigpio.RISING_EDGE, self._on_edge
            )

    def _on_edge(self, gpio, level, tick):
        # Single encoder channel: no direction sense, so trust the commanded one.
        self._edges += self._direction

    def set_speed_fraction(self, fraction: float) -> None:
        """Drive at `fraction` of full speed; negative reverses."""
        if self._pi is None:
            raise NotImplementedError("pigpio unavailable — running off-Pi?")
        fraction = max(-1.0, min(1.0, fraction)) * self.config.MAX_PWM_DUTY
        c = self.config
        if fraction >= 0:
            self._direction = 1
            self._pi.write(c.PIN_IN1, 1)
            self._pi.write(c.PIN_IN2, 0)
        else:
            self._direction = -1
            self._pi.write(c.PIN_IN1, 0)
            self._pi.write(c.PIN_IN2, 1)
        self._pi.set_PWM_dutycycle(c.PIN_PWM, round(abs(fraction) * _PWM_RANGE))

    def stop(self) -> None:
        """Cut drive power (coast; both inputs low)."""
        if self._pi is None:
            raise NotImplementedError("pigpio unavailable — running off-Pi?")
        c = self.config
        self._pi.set_PWM_dutycycle(c.PIN_PWM, 0)
        self._pi.write(c.PIN_IN1, 0)
        self._pi.write(c.PIN_IN2, 0)

    @property
    def distance_traveled(self) -> float:
        """Forward distance in mm from the encoder.

        Needs ENCODER_COUNTS_PER_REV + wheel circumference calibration; until
        then this returns 0 and the raw signed edge count is in `encoder_edges`.
        """
        if self.config.ENCODER_COUNTS_PER_REV:
            raise NotImplementedError("wire in counts->mm once calibrated")
        return self._distance_mm

    @property
    def encoder_edges(self) -> int:
        """Raw signed encoder edge count (direction from last command)."""
        return self._edges

    def close(self) -> None:
        if self._pi is not None:
            if self._encoder_cb is not None:
                self._encoder_cb.cancel()
            self.stop()
            self._pi.write(self.config.PIN_STBY, 0)
            self._pi.stop()
