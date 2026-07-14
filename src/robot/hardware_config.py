"""Hardware config for this build: GPIO pins, I2C addresses, calibration.

Tuned control/physics constants live in core.config (shared with the sim) — keep
them out of here so wiring changes never touch them. Anything marked VERIFY is a
guess from the BOM/CAD for now — we'll verify it on the actual robot.
"""

# BCM pin numbers, not physical header pins.
GPIO_MODE = "BCM"


# Drive motor — 12V 300RPM gearmotor via a TB6612FNG H-bridge.
# Uses CHANNEL A (channel B confirmed dead at chip level, 2026-07): motor leads
# on A01/A02, driver inputs on PWMA/AIN1/AIN2. The Pi-side GPIOs are unchanged
# from the old channel-B wiring — only the TB6612 terminals moved. Level shifter
# removed 2026-07-13 — logic driven direct from 3.3V GPIO (TB6612 VIH>=2.0V).
class Motor:
    PIN_PWM = 13      # PWMA (hardware PWM pin) — pin 33
    PIN_IN1 = 12      # AIN1 (direction) — pin 32 (moved off GPIO5: dead breadboard jumper)
    PIN_IN2 = 6       # AIN2 (direction) — pin 31
    # STBY is hardwired to 3.3V (pin 17) on the bench build — the GPIO23 route had
    # a dead jumper. Chip is always enabled; the GPIO23 write below is a harmless
    # no-op. Rewire to a GPIO if you need software enable/disable.
    PIN_STBY = 23     # STBY (not connected; hardwired 3.3V) — see note above
    PIN_ENC_A = None  # VERIFY — schematic only shows encoder B wired
    PIN_ENC_B = 24    # encoder ch B
    PWM_FREQUENCY_HZ = 1000  # VERIFY

    OUTPUT_RPM = None              # VERIFY measured RPM at the wheel
    ENCODER_COUNTS_PER_REV = None  # VERIFY counts per output rev
    MAX_PWM_DUTY = 1.0            # VERIFY: lower to cap top speed if needed


# Steering servo.
class Servo:
    PIN_SIGNAL = 18   # via level shifter LV1->HV1 (hardware PWM pin)
    MODEL = "SG90"
    # Pulse widths (us): CENTER = wheels straight; MIN/MAX = pulse at full
    # LEFT/RIGHT throw (i.e. at -/+ MAX_STEERING_ANGLE = 35 deg).
    # Placeholders — jog the real linkage and record these.
    PULSE_CENTER_US = 1500
    PULSE_MIN_US = 1000
    PULSE_MAX_US = 2000
    DIRECTION = 1     # flip if positive steering turns the wrong way. VERIFY.
    # Steering slew rate (deg/s): how fast the wheels sweep toward a new
    # target. Full lock-to-lock (70 deg) takes 70/SLEW_RATE_DPS seconds.
    SLEW_RATE_DPS = 90.0
    # Steering limit is core.config.MAX_STEERING_ANGLE (77.48° from CAD). VERIFY.


# ToF distance sensors.
# NOTE: the Pi's I2C bus is set to 50 kHz (dtparam=i2c_arm_baudrate=50000 in
# /boot/firmware/config.txt) — at the default 100 kHz the long init writes
# fail on some sensors with this wiring.
class Tof:
    MODEL = "VL53L1X"  # VERIFY — notes also say 3x VL53L0X
    COUNT = 4          # must match core.config.TOF_COUNT
    # Shared bus, same default address — enable each via XSHUT and reassign.
    # XSHUT GPIOs verified electrically on 2026-07-03 (matches Cirkit
    # schematic after the 4th wire moved from GPIO4 to GPIO22).
    # Labels relabelled 2026-07-14 after sensors were physically repositioned
    # (was front=26 left=25 rear=22; each faced the wrong way). Re-verify with a
    # hand-wave test. Right was unchanged ("right is always right").
    XSHUT_PINS = {"front": 22, "left": 26, "right": 16, "rear": 25}
    I2C_ADDRESSES = {"front": 0x30, "left": 0x31, "right": 0x32, "rear": 0x33}
    DISTANCE_MODE = 2        # 1=short (<=1.36m), 2=long (<=3.6m) — WRO walls need long
    TIMING_BUDGET_MS = 50    # per-sensor; 4 sensors ~ 20Hz effective loop rate


# Start button — momentary, 10k pulldown to GND (pressed = HIGH).
class Button:
    PIN = 17


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
    # The camera is mounted upside down on the chassis, so flip both axes
    # (= 180° rotation) in-sensor. Every consumer (web stream + pillar
    # detector) then sees an upright frame. Set both False if it's remounted.
    HFLIP = True
    VFLIP = True
    HSV_RED = None            # VERIFY under competition lighting
    HSV_GREEN = None
