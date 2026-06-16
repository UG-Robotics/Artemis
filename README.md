# Artemis — WRO 2026 Future Engineers

**Team:** UGhana Robotics

Autonomous driving software for the WRO 2026 Future Engineers competition. The
control logic is developed and validated in a 2D simulation, then shared with the
physical robot so the same brain runs in both.

## Approach

- **PD wall-following** for lateral centering between the track walls
- **Sensor fusion** — ToF distance, IMU heading, and a downward color sensor
- **State machine** covering wall following, corner turns, pillar avoidance,
  the three-point turn, and parallel parking
- **Adaptive corner navigation** for the varying open-challenge track widths
  (600–1000 mm)

## Directory structure

```
artemis/
├── src/                         # Control software
│   ├── core/                    # Shared control brain (config, controller, sensors)
│   ├── sim/                     # Simulations
│   │   ├── open-challenge-sim/  # Open challenge sim + test suite
│   │   └── obstacle-challenge-sim/  # Obstacle challenge sim (planned)
│   └── robot/                   # Physical robot control, Raspberry Pi (scaffold)
├── schemes/                     # Electromechanical / wiring diagrams
├── models/                      # CAD models for 3D printing
├── v-photos/                    # Vehicle photos
├── t-photos/                    # Team photos
└── video/                       # Competition video link
```

See [src/README.md](src/README.md) for how `core`, `sim`, and `robot` fit
together.

## Quick start

Requirements: Python 3.8+ and `pygame` (for the simulation viewer).

```bash
cd src/sim/open-challenge-sim
python sim_viewer.py        # real-time visualizer
python test_pd_tuning.py    # PD/navigation test suite
```

## Simulator controls

| Key | Action | Key | Action |
|-----|--------|-----|--------|
| SPACE | Pause / resume | T | Toggle sensor rays |
| R | Restart current config | P | Toggle path trail |
| ←/→ | Previous / next config | G | Toggle grid |
| A | Auto-play all configs | S | Screenshot |
| +/− | Simulation speed | Q / ESC | Quit |

## Status

- Open-challenge simulation: working; navigation validated across a test suite of
  placement and corner-entry cases.
- Robot code: scaffold in place (hardware-abstraction layer + driver stubs); see
  [src/robot/README.md](src/robot/README.md) for the remaining porting work.
- Obstacle challenge: simulation and on-robot logic in progress.
