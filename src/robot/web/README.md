# Web control panel

Manual teleop served by the Pi: a camera livestream with steering (◀ ▶, left) and
throttle (▲ ▼, right) pads, a speed slider, and an E‑STOP. Touch or keyboard
(WASD / arrows). **Dev/test tool only** — the WRO competition run is fully
autonomous and wireless-free; don't run this during a scored round.

## Run

```sh
cd src
python -m robot.web.app          # or: PYTHONPATH=src python -m robot.web.app
```

Open `http://<pi-ip>:8000`. On a dev machine (no robot) it starts in **SIMULATED**
mode: the camera shows a placeholder and the pads just log intent. On the Pi it
drives the real motor/servo once those drivers are implemented.

## Dependencies

- Server: Python stdlib only.
- Camera: `picamera2` on the Pi (real stream), or `Pillow` for the dev placeholder.

## Start on boot

```sh
sudo cp src/robot/web/artemis-web.service /etc/systemd/system/
# edit User= / WorkingDirectory= to match your install path
sudo systemctl enable --now artemis-web
```

## Safety

- **Deadman watchdog:** the robot stops if no control message arrives within
  ~0.6 s (tab hidden, page closed, Wi‑Fi drop). The browser also stops on blur /
  tab-hide and sends a heartbeat while a pad is held.
- Throttle is scaled by the speed slider (default 40%).
- Actuator calls are wrapped; if a driver isn't wired up the panel reports
  SIMULATED instead of crashing.
