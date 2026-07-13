"""Steering servo: maps a steering angle (deg, +=right) to a servo pulse.

The mapping depends on the built linkage, so the constants in
hardware_config.Servo have to be measured on the real car.

Uses pigpio (the pigpiod daemon must be running — enabled at boot on the Pi)
because its DMA-timed pulses are jitter-free, unlike RPi.GPIO software PWM.

`set_angle` is slew-rate limited: it sets a target and returns immediately,
while a background thread walks the servo toward it at SLEW_RATE_DPS so the
steering moves like a vehicle's, not a step function. Controllers can call it
at any rate; the thread owns the pulse output.
"""

import threading
import time

from core.config import MAX_STEERING_ANGLE
from robot.hardware_config import Servo as ServoConfig

try:
    import pigpio  # type: ignore
    _PIGPIO_AVAILABLE = True
except ImportError:
    pigpio = None
    _PIGPIO_AVAILABLE = False

_UPDATE_HZ = 50  # one step per servo PWM frame


class ServoDriver:
    """Front-wheel steering servo."""

    def __init__(self, config=ServoConfig):
        self.config = config
        self._pi = None
        self._target_deg = 0.0
        self._current_deg = 0.0
        self._lock = threading.Lock()
        self._running = False
        if _PIGPIO_AVAILABLE:
            self._pi = pigpio.pi()
            if not self._pi.connected:
                raise RuntimeError("pigpiod is not running (sudo systemctl start pigpiod)")
            self._pi.set_servo_pulsewidth(config.PIN_SIGNAL, self._angle_to_pulse(0.0))
            self._running = True
            self._thread = threading.Thread(target=self._slew_loop, daemon=True)
            self._thread.start()

    def _angle_to_pulse(self, angle_deg: float) -> int:
        """Clamp to the physical throw and interpolate between the measured pulses."""
        angle = max(-MAX_STEERING_ANGLE, min(MAX_STEERING_ANGLE, angle_deg))
        angle *= self.config.DIRECTION
        c = self.config
        if angle >= 0:
            span = c.PULSE_MAX_US - c.PULSE_CENTER_US
        else:
            span = c.PULSE_CENTER_US - c.PULSE_MIN_US
        return round(c.PULSE_CENTER_US + span * angle / MAX_STEERING_ANGLE)

    def _slew_loop(self) -> None:
        step = self.config.SLEW_RATE_DPS / _UPDATE_HZ
        period = 1.0 / _UPDATE_HZ
        while self._running:
            with self._lock:
                target = self._target_deg
            if self._current_deg != target:
                delta = target - self._current_deg
                self._current_deg += max(-step, min(step, delta))
                self._pi.set_servo_pulsewidth(
                    self.config.PIN_SIGNAL, self._angle_to_pulse(self._current_deg)
                )
            time.sleep(period)

    def set_angle(self, angle_deg: float) -> None:
        """Request `angle_deg`; the slew thread walks the servo there smoothly."""
        if self._pi is None:
            raise RuntimeError("pigpio unavailable — running off-Pi?")
        clamped = max(-MAX_STEERING_ANGLE, min(MAX_STEERING_ANGLE, angle_deg))
        with self._lock:
            self._target_deg = clamped

    @property
    def angle(self) -> float:
        """Where the steering actually is right now (deg), mid-slew included."""
        return self._current_deg

    def center(self) -> None:
        """Wheels straight ahead."""
        self.set_angle(0.0)

    def close(self) -> None:
        self._running = False
        if self._pi is not None:
            self._thread.join(timeout=0.5)
            self._pi.set_servo_pulsewidth(self.config.PIN_SIGNAL, 0)  # stop pulses
            self._pi.stop()
