"""Hardware config for this build: GPIO pins, I2C addresses, calibration.

Tuned control/physics constants live in core.config (shared with the sim) — keep
them out of here so wiring changes never touch them. Anything marked VERIFY is a
guess from the BOM/CAD for now — we'll verify it on the actual robot.
"""

# BCM pin numbers, not physical header pins.
GPIO_MODE = "BCM"


# Drive motor — N20 gearmotor via a TB6612FNG H-bridge.
class Motor:
    PIN_PWM = 12      # PWMA (hardware PWM pin). VERIFY against schemes/.
    PIN_IN1 = 23      # AIN1 (direction)
    PIN_IN2 = 24      # AIN2 (direction)
    PIN_STBY = 25     # STBY (high = enabled)
    PIN_ENC_A = 5     # encoder ch A. VERIFY the motor has an encoder + wiring.
    PIN_ENC_B = 6     # encoder ch B
    PWM_FREQUENCY_HZ = 1000  # VERIFY

    OUTPUT_RPM = None              # VERIFY measured RPM at the wheel
    ENCODER_COUNTS_PER_REV = None  # VERIFY counts per output rev
    MAX_PWM_DUTY = 1.0            # VERIFY: lower to cap top speed if needed


# Steering servo.
class Servo:
    PIN_SIGNAL = 13   # hardware PWM pin. VERIFY.
    MODEL = "SG90"    # VERIFY — BOM also lists MG996R
    # Pulse widths (us). Placeholders — jog the real linkage and record these.
    PULSE_CENTER_US = 1500
    PULSE_MIN_US = 1000
    PULSE_MAX_US = 2000
    DIRECTION = 1     # flip if positive steering turns the wrong way. VERIFY.
    # Steering limit is core.config.MAX_STEERING_ANGLE (77.48° from CAD). VERIFY.


# ToF distance sensors.
class Tof:
    MODEL = "VL53L1X"  # VERIFY — notes also say 3x VL53L0X
    COUNT = 4          # must match core.config.TOF_COUNT
    # Shared bus, same default address — enable each via XSHUT and reassign.
    XSHUT_PINS = {"front": 17, "left": 27, "right": 22, "rear": 4}  # VERIFY
    I2C_ADDRESSES = {"front": 0x30, "left": 0x31, "right": 0x32, "rear": 0x33}


# IMU — MPU6050.
class Imu:
    I2C_ADDRESS = 0x68   # 0x69 if AD0 high. VERIFY.
    # Register config — see docs/imu-accuracy.md. Lowest gyro range = finest
    # heading resolution (the robot turns slowly); DLPF tames vibration noise
    # without too much control-loop latency.
    GYRO_FS_SEL = 0      # 0 = ±250°/s
    DLPF_CFG = 3         # gyro 42 Hz bandwidth, 4.9 ms delay
    SAMPLE_RATE_DIV = 4  # 1 kHz / (1 + 4) = 200 Hz
    GYRO_Z_SIGN = 1      # flip to -1 if heading runs backwards once mounted. VERIFY.
    GYRO_Z_BIAS = 0.0    # deg/s at rest; overwritten by the startup calibration
    CALIB_SAMPLES = 600  # stationary samples averaged at startup (after warm-up)
    # ZUPT (zero-velocity update): when still, freeze heading and refine the bias.
    ZUPT_GYRO_THRESH = 1.5    # deg/s — |gyro| below this counts as "not turning"
    ZUPT_ACCEL_THRESH = 0.06  # g — ||accel|-1g| below this counts as "not moving"


# Downward color sensor — TCS34725.
class Color:
    I2C_ADDRESS = 0x29   # VERIFY — clashes with the default ToF address
    INTEGRATION_TIME_MS = 50   # VERIFY
    GAIN = 4                   # VERIFY
    THRESHOLDS = None          # VERIFY: calibrate orange/blue on the real mat


# Forward camera — OV5647. Obstacle challenge only.
class Camera:
    RESOLUTION = (640, 480)   # VERIFY the Pi 3B+ keeps up
    FRAMERATE = 30            # VERIFY
    HSV_RED = None            # VERIFY under competition lighting
    HSV_GREEN = None
