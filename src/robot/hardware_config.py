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
    # Top-speed cap, compresses the fraction->duty range (real-robot only; the sim
    # has no deadband). History 2026-07-18: 1.0 crashed corners (too fast) -> 0.62
    # fixed cornering but cruise (~49% duty) then STALLED on the straight (front
    # ToF froze mid-corridor, imufusion run held heading 90s but stopped moving).
    # 0.78 keeps corners controllable while lifting cruise clear of the static-
    # friction break.
    MAX_PWM_DUTY = 0.78
    # Deadband floor: below this PWM duty the gen-2 chassis doesn't move (it only
    # "crawled" at 0.35, and stalled from rest around ~0.49-0.53 on the mat). The
    # fraction remaps onto [MOTOR_MIN_DUTY, MAX_PWM_DUTY] so the slowest speed is
    # still comfortably above the break. TUNE on the mat: raise if it still stalls,
    # lower if the slowest speed is too fast to corner cleanly.
    MOTOR_MIN_DUTY = 0.48


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
    # Steering limit is core.config.MAX_STEERING_ANGLE (35°, measured throw).


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
    # Wired GPIO27 (header pin 13) -> button -> GND, internal pull-up.
    # Verified 2026-07-17 with robot.bench.button_test. Pressed = LOW.
    PIN = 27


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


class ImuLsm:
    """LSM6DSOX (bench-verified genuine 2026-07-16: WHO_AM_I 0x6C at 0x6A)."""
    I2C_ADDRESS = 0x6A   # 0x6B if SA0 pulled high
    GYRO_Z_SIGN = -1     # VERIFIED 2026-07-17 via robot.turn90: as mounted, a
                         # commanded right turn swept -90.9 raw, so Z is flipped
    GYRO_Z_BIAS = 0.0    # deg/s at rest; overwritten by the startup calibration
    CALIB_SAMPLES = 600  # stationary samples averaged at startup
    ZUPT_GYRO_THRESH = Imu.ZUPT_GYRO_THRESH
    ZUPT_ACCEL_THRESH = Imu.ZUPT_ACCEL_THRESH


# Downward color sensor — TCS34725/27 (this build reads ID 0x4d = TCS34727).
class Color:
    # 0x29 clashes with the VL53L1X ToF default, so the sensor lives on its own
    # software I2C bus (i2c-gpio overlay, SDA=GPIO20 SCL=GPIO21) added to the Pi's
    # /boot/firmware/config.txt on 2026-07-14. See [[project-open-challenge-tof]].
    BUS_ID = 3                 # /dev/i2c-3 (the dedicated color bus)
    I2C_ADDRESS = 0x29
    # Fast, short integration so a line flashing under the sensor at speed still
    # lands in a read (50ms blurred the line into the white mat -> missed);
    # higher gain keeps the light budget at 10ms.
    INTEGRATION_TIME_MS = 10    # 2.4ms min .. 700ms
    GAIN = 16                   # 1 / 4 / 16 / 60
    # Classify on the normalised warm axis (r-b) = (R-B)/clear — brightness- and
    # gain-independent. Calibrated on the mat 2026-07-18: orange r-b +0.35,
    # white +0.11, blue -0.09. Thresholds sit PERMISSIVELY off the white value
    # (so a partial/blurred line still reads as a line) while excluding white:
    #   r-b > ORANGE_RB_MIN -> orange;  r-b < BLUE_RB_MAX -> blue;  else none.
    # Symmetric margins off white (0.10): orange over-detected too little, blue
    # too much (mat run: 20 orange vs 175 blue reads, many off dark surfaces).
    ORANGE_RB_MIN = 0.15       # ~0.05 warm of white -> orange (easier now)
    BLUE_RB_MAX = 0.02         # ~0.08 cool of white -> blue (stricter now)
    MIN_CLEAR = 30             # ignore dark reads (walls/shadow read cool = false blue)
    THRESHOLDS = None          # legacy nearest-centroid refs (unused now)


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
