"""IMU heading — LSM6DSOX preferred, MPU6050 fallback.

The controller steers toward cardinals and counts turns off the heading, so it
needs a stable angle in degrees [0, 360). Both chips are 6-axis (no
magnetometer): yaw has no absolute reference and gyro-Z integration drifts, so
the driver attacks drift at the source (see docs/imu-accuracy.md):

  - finest gyro range (±250°/s) for best resolution at our slow turn rates;
  - startup bias calibration: average the stationary zero-rate offset;
  - ZUPT ("No Motion No Integration"): while still, freeze the heading and
    keep refining the bias so drift doesn't accumulate during stops.

`make_imu()` probes the bus and returns whichever chip is present — the
LSM6DSOX (WHO_AM_I 0x6C at 0x6A; bench-verified 2026-07-16, far better
zero-rate stability than the MPU6050) wins when both answer. Call `heading()`
once per control tick (integrates over wall-clock dt between calls).
"""

import math
import time

from robot.hardware_config import Imu as ImuConfig
from robot.hardware_config import ImuLsm as LsmConfig

try:
    import smbus2  # type: ignore
    _I2C_AVAILABLE = True
except ImportError:
    smbus2 = None
    _I2C_AVAILABLE = False


class _HeadingIntegrator:
    """Chip-agnostic heading logic: calibrated gyro-Z integration + ZUPT.

    Subclasses provide _init_device(), _gyro_z() (°/s, mounting-sign-corrected)
    and _accel_mag() (g).
    """

    def __init__(self, config, bus_id: int = 1):
        self.config = config
        self._heading = 0.0
        self._bias = config.GYRO_Z_BIAS
        self._last_t = None
        if not _I2C_AVAILABLE:
            self._bus = None
            return
        self._bus = smbus2.SMBus(bus_id)
        self._init_device()
        self.calibrate()

    def calibrate(self, samples: int = None) -> None:
        """Average the stationary zero-rate offset into the bias. Run after a
        warm-up (~60-120 s) with the robot held still; bias shifts as it heats."""
        if self._bus is None:
            return
        n = samples or self.config.CALIB_SAMPLES
        total = 0.0
        for _ in range(n):
            total += self._gyro_z()
            time.sleep(0.001)
        self._bias += total / n

    def reset_heading(self, heading: float = 0.0) -> None:
        """Call once at the start so 'forward' is a known value."""
        self._heading = heading % 360
        self._last_t = None

    def heading(self) -> float:
        """Current heading in degrees [0, 360). Call once per control tick."""
        if self._bus is None:
            raise RuntimeError("IMU not available (no I2C bus)")
        now = time.monotonic()
        if self._last_t is None:
            self._last_t = now
            return self._heading
        dt = now - self._last_t
        self._last_t = now

        rate = self._gyro_z() - self._bias
        # ZUPT: if not turning and not moving, don't integrate; refine the bias.
        if (abs(rate) < self.config.ZUPT_GYRO_THRESH
                and abs(self._accel_mag() - 1.0) < self.config.ZUPT_ACCEL_THRESH):
            self._bias += 0.05 * (self._gyro_z() - self._bias)  # slow bias tracking
            return self._heading

        self._heading = (self._heading + rate * dt) % 360
        return self._heading

    def close(self) -> None:
        if self._bus is not None:
            self._bus.close()


class Lsm6dsoxDriver(_HeadingIntegrator):
    """ST LSM6DSOX at 0x6A (0x6B if SA0 high). Little-endian output registers."""

    WHO_AM_I = 0x0F
    WHO_AM_I_VALUE = 0x6C
    CTRL1_XL = 0x10
    CTRL2_G = 0x11
    CTRL3_C = 0x12
    OUTX_L_G = 0x22
    OUTX_L_A = 0x28
    GYRO_SENS = 1000.0 / 8.75    # LSB per °/s at ±250 dps (8.75 mdps/LSB)
    ACCEL_SENS = 1000.0 / 0.061  # LSB per g at ±2 g (0.061 mg/LSB)

    def __init__(self, config=LsmConfig, bus_id: int = 1):
        super().__init__(config, bus_id)

    def _init_device(self) -> None:
        b, a = self._bus, self.config.I2C_ADDRESS
        who = b.read_byte_data(a, self.WHO_AM_I)
        if who != self.WHO_AM_I_VALUE:
            raise RuntimeError(
                f"IMU at 0x{a:02x}: WHO_AM_I=0x{who:02x}, expected 0x6C — "
                "not an LSM6DSOX."
            )
        # SW reset, then: block data update + auto-increment; accel 104 Hz ±2g;
        # gyro 104 Hz ±250 dps (finest resolution — the robot turns slowly).
        b.write_byte_data(a, self.CTRL3_C, 0x01)
        time.sleep(0.02)
        b.write_byte_data(a, self.CTRL3_C, 0x44)   # BDU | IF_INC
        b.write_byte_data(a, self.CTRL1_XL, 0x40)  # 104 Hz, ±2 g
        b.write_byte_data(a, self.CTRL2_G, 0x40)   # 104 Hz, ±250 dps
        time.sleep(0.05)

    def _read_word_le(self, reg: int) -> int:
        d = self._bus.read_i2c_block_data(self.config.I2C_ADDRESS, reg, 2)
        v = d[0] | (d[1] << 8)
        return v - 65536 if v >= 32768 else v

    def _gyro_z(self) -> float:
        return self.config.GYRO_Z_SIGN * self._read_word_le(self.OUTX_L_G + 4) / self.GYRO_SENS

    def accel(self) -> tuple:
        """Raw (ax, ay, az) in g, sensor frame (not gravity-compensated)."""
        return (self._read_word_le(self.OUTX_L_A) / self.ACCEL_SENS,
                self._read_word_le(self.OUTX_L_A + 2) / self.ACCEL_SENS,
                self._read_word_le(self.OUTX_L_A + 4) / self.ACCEL_SENS)

    def _accel_mag(self) -> float:
        ax, ay, az = self.accel()
        return math.sqrt(ax * ax + ay * ay + az * az)


class ImuDriver(_HeadingIntegrator):
    """MPU6050 at 0x68 (legacy fallback). Big-endian output registers."""

    WHO_AM_I = 0x75
    WHO_AM_I_MPU6050 = 0x68   # genuine part; counterfeits (relabelled ICM-20689) read 0x98
    PWR_MGMT_1 = 0x6B
    SLEEP_BIT = 0x40
    SMPLRT_DIV = 0x19
    CONFIG = 0x1A
    GYRO_CONFIG = 0x1B
    ACCEL_CONFIG = 0x1C
    ACCEL_XOUT_H = 0x3B
    GYRO_ZOUT_H = 0x47
    GYRO_SENS = 131.0      # LSB per °/s at ±250°/s
    ACCEL_SENS = 16384.0   # LSB per g at ±2g

    def __init__(self, config=ImuConfig, bus_id: int = 1):
        super().__init__(config, bus_id)

    def _init_device(self) -> None:
        b, a, cfg = self._bus, self.config.I2C_ADDRESS, self.config
        # Reject counterfeits/dead chips. A fake relabelled "MPU-6050A" (really an
        # ICM-20689) ACKs I2C and reports WHO_AM_I=0x98, but ignores every config
        # write and outputs all-zero data — so heading() would silently read a
        # constant 0 that looks healthy. Verify the genuine ID *and* that the chip
        # actually wakes; raise otherwise so the caller (telemetry) reports null.
        who = b.read_byte_data(a, self.WHO_AM_I)
        if who != self.WHO_AM_I_MPU6050:
            raise RuntimeError(
                f"IMU at 0x{a:02x}: WHO_AM_I=0x{who:02x}, expected 0x68 — not a "
                "genuine MPU6050 (0x98 = counterfeit ICM-20689). Replace the module."
            )
        b.write_byte_data(a, self.PWR_MGMT_1, 0x80)  # device reset
        time.sleep(0.1)
        b.write_byte_data(a, self.PWR_MGMT_1, 0x01)  # wake; clock = gyro PLL (stable)
        time.sleep(0.05)
        pwr = b.read_byte_data(a, self.PWR_MGMT_1)
        if pwr & self.SLEEP_BIT:
            raise RuntimeError(
                f"IMU at 0x{a:02x} did not wake (PWR_MGMT_1=0x{pwr:02x}, SLEEP set) — "
                "dead or counterfeit chip ignoring writes. Replace the module."
            )
        b.write_byte_data(a, self.SMPLRT_DIV, cfg.SAMPLE_RATE_DIV)
        b.write_byte_data(a, self.CONFIG, cfg.DLPF_CFG & 0x07)
        b.write_byte_data(a, self.GYRO_CONFIG, (cfg.GYRO_FS_SEL & 0x03) << 3)
        b.write_byte_data(a, self.ACCEL_CONFIG, 0x00)  # ±2g
        time.sleep(0.05)

    def _read_word(self, reg: int) -> int:
        hi = self._bus.read_byte_data(self.config.I2C_ADDRESS, reg)
        lo = self._bus.read_byte_data(self.config.I2C_ADDRESS, reg + 1)
        v = (hi << 8) | lo
        return v - 65536 if v >= 32768 else v

    def _gyro_z(self) -> float:
        """Yaw rate in °/s (sign-corrected for the mounting)."""
        return self.config.GYRO_Z_SIGN * self._read_word(self.GYRO_ZOUT_H) / self.GYRO_SENS

    def accel(self) -> tuple:
        """Raw (ax, ay, az) in g, sensor frame (not gravity-compensated)."""
        return (self._read_word(self.ACCEL_XOUT_H) / self.ACCEL_SENS,
                self._read_word(self.ACCEL_XOUT_H + 2) / self.ACCEL_SENS,
                self._read_word(self.ACCEL_XOUT_H + 4) / self.ACCEL_SENS)

    def _accel_mag(self) -> float:
        """Total acceleration in g (≈1.0 at rest)."""
        ax, ay, az = self.accel()
        return math.sqrt(ax * ax + ay * ay + az * az)


def make_imu(bus_id: int = 1):
    """Return a driver for whichever IMU is on the bus (LSM6DSOX preferred)."""
    if not _I2C_AVAILABLE:
        return Lsm6dsoxDriver(bus_id=bus_id)  # off-Pi no-op shell
    try:
        return Lsm6dsoxDriver(bus_id=bus_id)
    except (OSError, RuntimeError) as lsm_err:
        try:
            return ImuDriver(bus_id=bus_id)
        except (OSError, RuntimeError) as mpu_err:
            raise RuntimeError(
                f"no usable IMU: LSM6DSOX ({lsm_err}); MPU6050 ({mpu_err})"
            )
