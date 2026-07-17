# Testing Workflow

Every control change follows the same three-stage pipeline: **simulate → bench → field**. Nothing drives the robot that hasn't passed the sim suites, and nothing joins the control loop that hasn't been bench-tested in isolation.

## Stage 1 — Simulation suites

The simulator ([`src/sim/open-challenge-sim/`](src/sim/open-challenge-sim)) shares the *actual* control code in [`src/core/`](src/core) — the tests exercise the same classes the robot runs, not a reimplementation.

Run from `src/sim/open-challenge-sim/` (needs Python 3.8+, `pygame`, `numpy` — see [`requirements.txt`](requirements.txt)):

| Suite | Command | What it verifies |
|---|---|---|
| PD stability | `python test_pd_tuning.py` | Three phases: straight-line PD convergence from an injected heading error; corner-exit stabilization; then the full 48-case placement/corner-entry matrix. **Pass bar: 48/48 finish 3 laps without wall contact.** |
| Rules scoring | `python test_open_score.py` | Scores complete `WallFollowController` runs exactly per the WRO rubric (24 section points + finish bonus). Ground truth comes from the sim's pose — the controller itself stays pose-free. **Pass bar: 30/30.** |
| Batch/headless | `python test_headless.py` | Graphics-free batch runner across randomized track configs — the harness the other suites build on; used in every tuning loop. |
| Sensor-noise regression | `python test_tof_noise.py` | Replays full open rounds under injected ToF dropout noise (bench-measured spike patterns) across filter configurations. This suite is how the ToF outlier-gate/median design was chosen — one candidate filter passed clean runs but scored 0/10 on the narrow track from filter lag, which only this suite exposed. |

The interactive viewer (`python sim_viewer.py`) is the debugging companion: pause, single-step, sensor-ray and path-trail overlays, and config switching across the randomized layouts.

**Regression rule:** a change to anything in `core/` requires re-running the PD and scoring suites before deployment. When a field failure is diagnosed, we first reproduce it in the sim (adding a config or noise case if needed), fix it in `core/`, and re-validate — so the sim's test matrix only ever grows.

## Stage 2 — Hardware bench tests

Each device has an isolated bring-up script in [`src/robot/bench/`](src/robot/bench), run before the device is trusted in the control loop and re-run after any rewiring:

| Script | Purpose |
|---|---|
| `i2c_scan` | Confirm every I2C device enumerates at its assigned address |
| `tof_test/` | Live per-sensor distance readout; hand-wave test verifies each sensor's label matches its axis (a mandatory step — mislabeled axes have burned us) |
| `imu_test`, `robot.turn90` | Drift measurement, bias calibration check, and mounted-sign verification via a commanded 90° turn |
| `servo_test/` | Jog the linkage to record real pulse-width limits and center |
| `motor_test/` | Graduated probes (passive GPIO probe → single-channel spin → full driver) that isolate chip vs wiring vs code faults |
| `button_test` | Start-button wiring and edge detection |
| `power_watch/` | Continuous under-voltage/throttling logger (runs as a service, not just at the bench) |

Verified values graduate from `VERIFY`-marked guesses to constants in [`src/robot/hardware_config.py`](src/robot/hardware_config.py); `grep VERIFY src/robot/hardware_config.py` lists exactly what still runs unmeasured.

## Stage 3 — Field runs with the flight recorder

Every real run is automatically logged by [`src/robot/run_logger.py`](src/robot/run_logger.py):

- `sensors.csv` — full sensor state at 30 Hz
- `camera.jsonl` — detections at 10 Hz, plus keyframe JPEGs on every detection change
- `meta.json` — run mode, parameters, and the **deployed git revision**, so any log is traceable to the exact code that produced it

Post-run analysis: `python3 -m robot.bench.log_timeline <run-dir>` renders the run as an event timeline with an ASCII plot of the driven path — how we do post-mortems on field behaviour without a camera crew. Logs are pulled off the Pi with `rsync` and kept per-run.
