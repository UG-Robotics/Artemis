# Power & Sensor Architecture

Companion to the wiring diagram in [`../schemes/`](../schemes) (full schematic PDF in [`schematics/`](schematics)). This doc covers how power is distributed and budgeted, why each sensor is where it is, how everything is calibrated, and the failure modes we guard against.

## 1. Power architecture

```
2S LiPo pack (7.4 V nominal, 5000 mAh / 37 Wh, built-in dual-protection board)
 ├── TB6612FNG VM ──► drive motor (12 V-rated JGA25-370 gearmotor)
 └── XL4015 buck ──► 5 V rail
        ├── Raspberry Pi 3B+ (+ OV5647 camera via CSI)
        ├── SG90 steering servo
        └── sensor set (4× VL53L1X, LSM6DSOX, TCS34727) via the Pi's 3V3/5V pins
```

> **Battery history:** the build originally used a 3×18650 holder (11.1 V 3S). It was replaced with a single 7.4 V 2S LiPo brick ([label photo](images/battery-2s-lipo-label.jpeg): Xtreme 1, 5000 mAh / 37 Wh, 8.4 V max charge) — higher capacity, a built-in dual-protection circuit board, and a single rigid unit in the under-tub bay instead of a holder with spring contacts (a vibration liability). The 12 V-rated motor now runs below rated voltage, which is fine for us — the sim retune showed the vehicle is driven at ~35% throttle anyway — and the actual wheel RPM at pack voltage is exactly what the `OUTPUT_RPM` `VERIFY` measurement will pin down.

Two deliberate separations:

- **Motor current never crosses the logic rail.** The motor draws directly from the pack through the H-bridge; the buck isolates the 5 V rail from motor sag and inrush, so a stall can't brown out the Pi mid-run.
- **The buck output is set with a multimeter, capped at 5.2 V.** The Pi can detect under-voltage but not over-voltage, so the upper bound is enforced at calibration time, not in software (documented in [`../src/robot/bench/power_watch/power_watch.sh`](../src/robot/bench/power_watch/power_watch.sh)).

### Power budget (datasheet-typical; on-robot measurement is on the bench list)

| Load | Rail | Typical | Peak | Notes |
|---|---|---|---|---|
| Raspberry Pi 3B+ | 5 V | ~350–700 mA | ~1.2 A | Higher end with camera streaming + 4-sensor polling |
| OV5647 camera | 5 V (via Pi) | ~250 mA | — | Counted inside the Pi's loaded figure |
| SG90 servo | 5 V | ~10 mA idle, 100–250 mA moving | ~650 mA stall | Peak only at mechanical limit |
| 4× VL53L1X | 3V3 | ~80 mA (4 × ~20 mA ranging) | ~110 mA | Continuous long-distance ranging |
| LSM6DSOX | 3V3 | <1 mA | — | |
| TCS34727 | 3V3 | <1 mA | — | Own software I2C bus (see §2) |
| TB6612 logic | 3V3 | ~2 mA | — | |
| **5 V rail total** | | **~0.5–1.1 A** | **~2 A** | XL4015 is rated 5 A — >2× headroom at worst case |
| Drive motor | pack | load-dependent | ≤ TB6612 limit | TB6612 channel: 1.2 A continuous / 3.2 A peak; the JGA25-370 is driven at ~35% throttle in normal running, and a sustained stall is a fault state the controller never commands |

### Runtime power monitoring

A watchdog service ([`power_watch.sh`](../src/robot/bench/power_watch/power_watch.sh)) polls the Pi's `vcgencmd get_throttled` flags every 10 s and logs any under-voltage, frequency-capping, or thermal-throttle event — including the sticky "has occurred since boot" bits, so a transient brown-out during a run is caught even if we look hours later. This is how we verify the power budget in practice rather than trusting the arithmetic above.

## 2. Bus architecture (lessons included)

All I2C devices share the Pi's hardware bus **except** the color sensor. Three non-obvious choices, each bought with debugging time:

1. **ToF address assignment at boot.** All four VL53L1X ship at address 0x29. Each sensor's XSHUT line goes to its own GPIO; at startup we hold all four in reset, then enable them one at a time and reassign to 0x30–0x33 ([`../src/robot/hardware_config.py`](../src/robot/hardware_config.py)).
2. **The color sensor gets its own bus.** The TCS34727's address is fixed at 0x29 — the exact address the ToFs default to during their staggered boot. Rather than fight the race, the color sensor lives on a dedicated software I2C bus (`i2c-gpio` overlay, SDA=GPIO20 / SCL=GPIO21 → `/dev/i2c-3`).
3. **The hardware bus runs at 50 kHz, not the default 100 kHz.** The VL53L1X's long init sequences failed intermittently on our loom at 100 kHz; halving the clock (`dtparam=i2c_arm_baudrate=50000`) made init deterministic. Slower bus, but with a 50 ms ranging budget per sensor the bus is nowhere near the bottleneck.

## 3. Sensor selection & placement

| Sensor | Why this part | Why this position |
|---|---|---|
| 4× VL53L1X ToF (front/left/right/rear) | Long-distance mode ranges to 3.6 m — full 1000 mm track width plus diagonal views, where a VL53L0X (1.36 m) would clip | Body windows put the beams **below the 100 mm wall top**, so a ray always terminates on wall, not open air. Left/right opposing pair → centring error `right − left` in one subtraction; front → corner-wall detection and turn radius; rear → start-section signature for the finish stop |
| LSM6DSOX IMU | Replaced a counterfeit "MPU6050" (see §4). Bench-measured ≈0.07°/min raw gyro drift — ~0.2° over a 3-minute run before fusion | Inside the tub, screwed to the lid's stiffening ribs: rigid mount, away from the motor, because gyro noise is dominated by mechanical vibration coupling (analysis: [`imu-accuracy.md`](imu-accuracy.md)) |
| TCS34727 color | Detects the mat's orange/blue corner lines — the rules' ground-truth lap/corner reference | Facing straight down through a floor aperture at controlled ride height; ambient-light-proof compared to aiming a camera at the mat |
| OV5647 camera (160° wide-angle) | Pillar color+bearing for the obstacle challenge; heading fusion for the open challenge | High on the body nose: wide FOV covers both track sides; mounted inverted for cable routing, corrected in-sensor (180° flip) so every consumer sees an upright frame |

**ToF timing budget:** 50 ms per sensor → the 4-sensor array refreshes at ~20 Hz, matched to the control loop rate. Long-distance mode + 50 ms is the datasheet-recommended pairing for full range.

**ToF de-noising** ([`../src/core/tof_filter.py`](../src/core/tof_filter.py)): the VL53L1X occasionally drops a single frame to a spurious value (usually the 4 m no-target sentinel). We run a two-stage per-channel filter — an outlier gate that rejects physically-impossible frame-to-frame jumps unless two consecutive samples agree (a genuine wall edge passing), then a rolling median. A median was chosen over a low-pass deliberately: it never smears a spike into neighbouring samples and adds at most one frame of lag. The *same filter code* runs in the simulator (fed with bench-measured dropout rates), which is how the escape-valve tuning was validated: an early gate version held stale readings through full-lock turns and dropped the narrow-track sim suite from 10/10 to 0/10 — the sim caught it before the track did.

## 4. Calibration procedures

**IMU (every power-on, automatic):**
1. Chip-ID check first — the driver reads WHO_AM_I and refuses to start on a mismatch. This exists because our original "MPU6050" was a counterfeit that answered 0x98 and produced garbage; a fake or mis-wired part now fails loudly at boot instead of navigating badly.
2. Stationary bias calibration: 600 gyro samples averaged while the robot is still (the boot-to-button service does this during startup, which is why the robot must not be moved right after power-on).
3. Zero-velocity updates (ZUPT) during the run: whenever gyro and accelerometer both read "not moving", the heading is frozen and the bias estimate refined — bounding drift between corners.
4. Mounting sign verified empirically: a commanded 90° right turn swept −90.9° raw, so the Z-axis sign is inverted in config (`GYRO_Z_SIGN = -1`) — measured, not assumed.

**ToF array:** after any physical repositioning, each sensor is verified with a hand-wave test against its label. This rule exists because we once repositioned the sensors and three of the four labels ended up on the wrong axes — the array "worked" electrically while reporting geometry that was simply wrong.

**Servo:** the linkage is jogged over its range with [`bench/servo_test`](../src/robot/bench/servo_test) to record the real center/min/max pulse widths and the measured ±35° mechanical throw; software commands are clamped to that measured limit, and a slew-rate limit (90°/s) keeps steering transients from shocking the linkage.

**Color & camera (venue-dependent, calibrated at the competition table):** the TCS34727 orange/blue thresholds and the camera's red/green HSV ranges are lighting-dependent, so they are treated as *deployment* calibration, not constants — both are explicitly `VERIFY`-marked in `hardware_config.py` and re-measured on the actual mat.

**Discipline for everything above:** any physical constant that has not been measured on this build is marked `VERIFY` in [`hardware_config.py`](../src/robot/hardware_config.py). The marker is grep-able, so "what still runs on a guess" is a one-line query, not tribal knowledge.

## 5. Failure modes considered

| Risk | Mitigation |
|---|---|
| Motor stall browns out the Pi | Separate motor path from the 5 V rail via the buck; `power_watch` logs any under-voltage flag it misses |
| Counterfeit / mis-wired I2C part | WHO_AM_I verification at boot (policy adopted after the fake-IMU incident) |
| I2C address collision (ToF vs color) | Dedicated software bus for the color sensor; staggered XSHUT boot for the ToFs |
| Single-frame ToF glitches steering | Outlier gate + median filter, identical in sim and robot |
| Gyro drift over a 3-minute run | Warm-up-aware bias cal + ZUPT + drift bench-measured before trusting the sensor; ToF-only controller as a full fallback brain |
| Camera contention (dashboard vs run) | Picamera2 single-owner guard + `Conflicts=artemis-web` in the run service |
| Dead driver channel (it happened) | Bench probe scripts in [`bench/motor_test`](../src/robot/bench/motor_test) isolate chip vs wiring vs code in minutes; motor moved to the healthy channel |
