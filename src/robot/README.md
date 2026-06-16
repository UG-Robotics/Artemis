# Robot — physical vehicle control (Raspberry Pi)

Control code that runs on the real Artemis vehicle. It reuses the shared control
brain in [`../core/`](../core/) (the `Controller` and tuned `config`) and adds the
hardware drivers behind a single abstraction layer, so the same logic validated
in simulation drives the real robot.

> **Status: scaffold.** The structure, the hardware interface, and the control
> loop are in place. The device drivers are stubs and two control-loop
> dependencies are unfinished — see *Porting gap* below. It does not run yet.

## Layout

| File | Purpose |
|------|---------|
| `hal.py` | The `Hardware` contract the controller needs, plus `RealHardware` that composes the drivers and assembles a `SensorReading`. |
| `main.py` | Control-loop entry point; documents the porting gap. |
| `hardware_config.py` | Pins, I2C addresses, and calibration constants — every uncertain value is marked `VERIFY`. |
| `drivers/` | One module per device: `motor`, `servo`, `tof`, `imu`, `color`, `camera`. |

Physics and control constants stay in `core/config.py` (shared with the sim).
Only build-specific wiring/calibration lives in `hardware_config.py`.

## How it fits together

```
core.Controller.update(sensors, robot, track, dt)
        ▲              ▲        ▲      ▲
        │   SensorReading      │   OpenChallengeWorld (main.py)
        │   from RealHardware  │
        │                  RobotIO (main.py) → RealHardware actuators + encoder
        └── same call the simulation makes
```

`RealHardware` satisfies the same interface the sim's `Robot` does, so the
controller is unchanged between the two.

## Porting gap (what's left before it runs)

The controller assumes two sim-only things; both are stubbed in `main.py`:

1. **Pose** — it reads `robot.x/y/angle`. For the open challenge this is really
   just the line-dedup in `_detect_lines`, which encoder distance can cover
   (heading's already from the IMU). We'll add a small dead-reckoning estimator.
2. **Track map** — there isn't one. `OpenChallengeWorld` fakes the few values it
   asks for; returning `None` for the section query falls back to sensor/heading
   logic.

The obstacle challenge needs more (camera pillar geometry) — later.

## Bring-up order (suggested)

1. Check the wiring against `schemes/` and fill in the `VERIFY` values in
   `hardware_config.py` — measure the motor's real output RPM and encoder counts
   early, since odometry depends on them.
2. Implement and bench-test each driver in `drivers/` independently.
3. Implement the `PoseEstimator` / track model (porting gap) for the open
   challenge.
4. Tune on the real track; feed any control changes back into `core/` so the
   simulation stays in sync.

## Dependencies (Pi)

Driver imports are guarded, so this package imports on a dev machine without
these. On the Pi you will need (confirm exact packages during bring-up):
`RPi.GPIO` (or `pigpio`), `smbus2`, the Adafruit CircuitPython ToF/color
libraries, and `picamera2` + `opencv` for the obstacle-challenge camera.
