"""IMU heading from an MPU6050.

The controller steers toward cardinals and counts turns off the heading, so it
needs a stable angle in degrees [0, 360). The MPU6050 is 6-axis (no magnetometer)
so yaw has no absolute reference and the gyro-Z integration drifts; this driver
attacks that at the source (see docs/imu-accuracy.md):

  - clean config: ±250°/s range (finest resolution), 42 Hz DLPF, 200 Hz rate;
  - startup bias calibration: average the stationary zero-rate offset and subtract;
  - ZUPT ("No Motion No Integration"): while the robot is still, freeze the heading
    and keep refining the bias, so drift doesn't accumulate during stops.

Call heading() once per control tick (it integrates over wall-clock dt between
calls). Higher accuracy still: drive the chip's on-board DMP (hardware quaternion
fusion) and read fused yaw from its FIFO — a drop-in upgrade behind this same
interface, via an i2cdevlib-derived library. Bench-test all of this on the Pi.
"""

import math
import time

from robot.hardware_config import Imu as ImuConfig

try:
    import smbus2  # type: ignore
    _I2C_AVAILABLE = True
except ImportError:
    smbus2 = None
    _I2C_AVAILABLE = False


class ImuDriver:
    """Heading source for the controller (calibrated gyro-Z integration + ZUPT)."""

    # MPU6050 registers
    PWR_MGMT_1 = 0x6B
    SMPLRT_DIV = 0x19
    CONFIG = 0x1A
    GYRO_CONFIG = 0x1B
    ACCEL_CONFIG = 0x1C
    ACCEL_XOUT_H = 0x3B
    GYRO_ZOUT_H = 0x47
    GYRO_SENS = 131.0      # LSB per °/s at ±250°/s
    ACCEL_SENS = 16384.0   # LSB per g at ±2g

    def __init__(self, config=ImuConfig, bus_id: int = 1):
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

    def _init_device(self) -> None:
        b, a, cfg = self._bus, self.config.I2C_ADDRESS, self.config
        b.write_byte_data(a, self.PWR_MGMT_1, 0x01)  # wake; clock = gyro PLL (stable)
        time.sleep(0.05)
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

    def _accel_mag(self) -> float:
        """Total acceleration in g (≈1.0 at rest)."""
        ax = self._read_word(self.ACCEL_XOUT_H) / self.ACCEL_SENS
        ay = self._read_word(self.ACCEL_XOUT_H + 2) / self.ACCEL_SENS
        az = self._read_word(self.ACCEL_XOUT_H + 4) / self.ACCEL_SENS
        return math.sqrt(ax * ax + ay * ay + az * az)

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
            raise RuntimeError("MPU6050 not available (no I2C bus)")
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
