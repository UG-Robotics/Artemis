"""ToF distance sensors — the four wall distances (front/left/right/rear).

All VL53L1X boot at the same default address (0x29), so bring-up holds every
sensor in reset via XSHUT, then wakes them one at a time and moves each to its
own address (see hardware_config.Tof).

Bench-verified quirks (Pi 3B+, adafruit-circuitpython-vl53l1x):
- The library's __init__ crashes with "Unknown distance mode" on a factory-fresh
  sensor: it sets timing_budget, whose setter reads distance_mode, and the
  phasecal register isn't one of the two values the getter recognizes yet.
  We construct the object manually and set the mode before the budget.
- Two sensors sharing 0x29 answer in lockstep (identical ACKs/IDs), so an I2C
  scan shows "one" device but ranging data collides to zeros. Never talk to
  0x29 until all but one sensor is held in reset.
"""

import time

from core.config import TOF_MAX_RANGE
from robot.hardware_config import Tof as TofConfig

# Adafruit CircuitPython stack; import lazily so this loads off-Pi.
try:
    import board  # type: ignore
    import busio  # type: ignore
    import adafruit_vl53l1x  # type: ignore
    from adafruit_bus_device import i2c_device  # type: ignore
    import RPi.GPIO as GPIO  # type: ignore
    _I2C_AVAILABLE = True
except (ImportError, NotImplementedError):
    board = busio = adafruit_vl53l1x = i2c_device = GPIO = None
    _I2C_AVAILABLE = False

_DEFAULT_ADDRESS = 0x29
_BOOT_DELAY_S = 0.05  # datasheet t_boot is 1.2ms; leave margin


def _make_sensor(i2c, address):
    """Construct a VL53L1X, dodging the library's fresh-sensor init crash."""
    sensor = adafruit_vl53l1x.VL53L1X.__new__(adafruit_vl53l1x.VL53L1X)
    sensor.i2c_device = i2c_device.I2CDevice(i2c, address)
    sensor._i2c = i2c
    model_id, module_type, _ = sensor.model_info
    if model_id != 0xEA or module_type != 0xCC:
        raise RuntimeError(
            f"device at 0x{address:02x} is not a VL53L1X "
            f"(model=0x{model_id:02x} type=0x{module_type:02x})"
        )
    sensor._sensor_init()
    sensor._timing_budget = TofConfig.TIMING_BUDGET_MS
    sensor.distance_mode = TofConfig.DISTANCE_MODE  # also applies the budget
    return sensor


class TofArray:
    """The wall-facing ToF sensors, keyed by position."""

    POSITIONS = ("front", "left", "right", "rear")

    def __init__(self, config=TofConfig, require_all=True):
        self.config = config
        self._sensors = {}
        self._last = {p: TOF_MAX_RANGE for p in self.POSITIONS}
        if not _I2C_AVAILABLE:
            return

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        pins = config.XSHUT_PINS
        # Hold everything in reset so nothing squats on 0x29.
        for pin in pins.values():
            GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)
        time.sleep(_BOOT_DELAY_S)

        i2c = busio.I2C(board.SCL, board.SDA)
        errors = {}
        for position in self.POSITIONS:
            # A marginal connection can fail one transaction; power-cycle
            # via XSHUT and retry before declaring the sensor dead.
            for attempt in range(3):
                GPIO.output(pins[position], GPIO.HIGH)
                time.sleep(_BOOT_DELAY_S)
                try:
                    sensor = _make_sensor(i2c, _DEFAULT_ADDRESS)
                    sensor.set_address(config.I2C_ADDRESSES[position])
                    sensor.start_ranging()
                    self._sensors[position] = sensor
                    break
                except (OSError, ValueError, RuntimeError) as exc:
                    errors[position] = exc
                    GPIO.output(pins[position], GPIO.LOW)
                    time.sleep(_BOOT_DELAY_S)
        self.missing = [p for p in self.POSITIONS if p not in self._sensors]
        self.errors = {p: errors[p] for p in self.missing}
        if self.missing and require_all:
            raise RuntimeError(
                f"ToF bring-up failed for {self.missing}: "
                + "; ".join(f"{p}: {errors.get(p)}" for p in self.missing)
            )

    def read_all(self) -> dict:
        """{'front','left','right','rear'} distances in mm.

        No target in range reads TOF_MAX_RANGE (the sim's convention). Between
        sensor updates (each sensor produces a reading every timing budget) the
        last known value is held, so callers can poll faster than the sensors.
        """
        for position, sensor in self._sensors.items():
            if not sensor.data_ready:
                continue
            distance_cm = sensor.distance
            sensor.clear_interrupt()
            self._last[position] = (
                TOF_MAX_RANGE if distance_cm is None else distance_cm * 10.0
            )
        return dict(self._last)

    def close(self) -> None:
        for sensor in self._sensors.values():
            sensor.stop_ranging()
        self._sensors.clear()
