"""Manual teleop: turn web button intents into actuator commands.

Holds a throttle/steer intent (each -1..1) and maps it onto the motor + servo.
Runs on a dev machine too: if the actuator drivers aren't implemented yet (no Pi)
their calls raise NotImplementedError, so we flag 'simulated' and just track intent.
A watchdog stops the robot if commands stop arriving (deadman for network teleop).
"""

import threading
import time

from core.config import MAX_STEERING_ANGLE
from robot.drivers.motor import MotorDriver
from robot.drivers.servo import ServoDriver


class Teleop:
    def __init__(self, speed_scale: float = 0.4):
        self._motor = MotorDriver()
        self._servo = ServoDriver()
        self._lock = threading.Lock()
        self.throttle = 0.0          # -1..1 (forward/reverse)
        self.steer = 0.0             # -1..1 (left/right)
        self.speed_scale = speed_scale
        self.simulated = False       # True once a driver call shows it's not wired up
        self.last_cmd = time.monotonic()
        self._apply()

    def _safe(self, fn, *args) -> bool:
        try:
            fn(*args)
            return True
        except NotImplementedError:
            self.simulated = True
        except Exception:
            self.simulated = True
        return False

    def _apply(self) -> None:
        self._safe(self._motor.set_speed_fraction, self.throttle * self.speed_scale)
        self._safe(self._servo.set_angle, self.steer * MAX_STEERING_ANGLE)

    def command(self, throttle=None, steer=None, speed_scale=None) -> None:
        with self._lock:
            if throttle is not None:
                self.throttle = max(-1.0, min(1.0, float(throttle)))
            if steer is not None:
                self.steer = max(-1.0, min(1.0, float(steer)))
            if speed_scale is not None:
                self.speed_scale = max(0.0, min(1.0, float(speed_scale)))
            self.last_cmd = time.monotonic()
            self._apply()

    def stop(self) -> None:
        with self._lock:
            self.throttle = 0.0
            self.steer = 0.0
            self._safe(self._motor.stop)
            self._safe(self._servo.center)
            self.last_cmd = time.monotonic()

    def watchdog(self, timeout: float) -> bool:
        """Deadman: stop if we're moving but no command arrived within `timeout`."""
        if self.throttle != 0.0 and (time.monotonic() - self.last_cmd) > timeout:
            self.stop()
            return True
        return False

    def state(self) -> dict:
        with self._lock:
            return {
                "throttle": round(self.throttle, 2),
                "steer": round(self.steer, 2),
                "speed_scale": round(self.speed_scale, 2),
                "steering_deg": round(self.steer * MAX_STEERING_ANGLE, 1),
                "speed_frac": round(self.throttle * self.speed_scale, 2),
                "simulated": self.simulated,
            }

    def close(self) -> None:
        self.stop()
        for dev in (self._motor, self._servo):
            try:
                dev.close()
            except Exception:
                pass
