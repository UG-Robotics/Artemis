"""Downward color sensor (TCS34725/27) — reads the orange/blue section lines.

Lives on its own software I2C bus (hardware_config.Color.BUS_ID) because its
fixed 0x29 address clashes with the VL53L1X ToF default on the main bus.

The controller counts sections and spots corner entry from this, so reliable
orange/blue/none on the real mat is what matters — mostly a calibration job.
Reads use normalised channels (r/g/b = R/G/B ÷ clear) so they're insensitive to
absolute brightness; classification thresholds get calibrated on the mat.
"""

import time

from robot.hardware_config import Color as ColorConfig

try:
    import smbus2  # type: ignore
    _I2C_AVAILABLE = True
except ImportError:
    smbus2 = None
    _I2C_AVAILABLE = False

_CMD = 0x80          # command bit — must be set to address a register
_ENABLE = 0x00       # PON=0x01, AEN=0x02
_ATIME = 0x01
_CONTROL = 0x0F
_ID = 0x12
_CDATA = 0x14        # C,R,G,B low/high pairs, 8 bytes from here
_GAIN = {1: 0x00, 4: 0x01, 16: 0x02, 60: 0x03}
_VALID_IDS = (0x44, 0x4D, 0x10)  # TCS34725 / TCS34727 / some clones


class ColorSensor:
    """Floor line detector: 'orange', 'blue', or None."""

    def __init__(self, config=ColorConfig):
        self.config = config
        self._bus = None
        self._addr = config.I2C_ADDRESS
        if not _I2C_AVAILABLE:
            return
        bus_id = getattr(config, "BUS_ID", 1)
        self._bus = smbus2.SMBus(bus_id)
        who = self._bus.read_byte_data(self._addr, _CMD | _ID)
        if who not in _VALID_IDS:
            raise RuntimeError(
                f"TCS3472x not found on i2c-{bus_id} at 0x{self._addr:02x} "
                f"(ID=0x{who:02x})"
            )
        # integration time: ATIME = 256 - t/2.4ms; longer = more light gathered
        atime = max(0, min(255, round(256 - config.INTEGRATION_TIME_MS / 2.4)))
        self._integration_s = (256 - atime) * 0.0024
        self._bus.write_byte_data(self._addr, _CMD | _ATIME, atime)
        self._bus.write_byte_data(self._addr, _CMD | _CONTROL, _GAIN.get(config.GAIN, 0x01))
        self._bus.write_byte_data(self._addr, _CMD | _ENABLE, 0x01)   # PON
        time.sleep(0.003)
        self._bus.write_byte_data(self._addr, _CMD | _ENABLE, 0x03)   # PON | AEN
        time.sleep(self._integration_s + 0.01)

    def raw(self) -> tuple:
        """(clear, red, green, blue), 16-bit counts each."""
        if self._bus is None:
            raise NotImplementedError("no I2C bus (running off-Pi?)")
        d = self._bus.read_i2c_block_data(self._addr, _CMD | _CDATA, 8)
        c = d[0] | (d[1] << 8)
        r = d[2] | (d[3] << 8)
        g = d[4] | (d[5] << 8)
        b = d[6] | (d[7] << 8)
        return c, r, g, b

    def normalized(self) -> tuple:
        """(r, g, b) each divided by clear — brightness-independent, 0..1."""
        c, r, g, b = self.raw()
        if c == 0:
            return 0.0, 0.0, 0.0
        return r / c, g / c, b / c

    def detect(self) -> "str | None":
        """Classify the floor as 'orange', 'blue', or None on the warm axis.

        rb = (R - B) / clear — brightness/gain-independent. Orange is warm
        (rb high), blue is cool (rb low), the white mat sits between. The
        thresholds sit permissively off the white value so a partial or motion-
        blurred line still reads as a line, while white stays None. One raw read
        (fast). Near-dark frames (clear < MIN_CLEAR) return None (noise floor)."""
        cfg = self.config
        c, r, g, b = self.raw()
        if c < getattr(cfg, "MIN_CLEAR", 12):
            return None
        rb = (r - b) / c
        if rb > cfg.ORANGE_RB_MIN:
            return 'orange'
        if rb < cfg.BLUE_RB_MAX:
            return 'blue'
        return None

    def close(self) -> None:
        if self._bus is not None:
            try:
                self._bus.write_byte_data(self._addr, _CMD | _ENABLE, 0x00)  # power off
            except Exception:
                pass
            self._bus.close()
