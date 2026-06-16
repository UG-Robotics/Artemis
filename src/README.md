# Control Software

Control software for Team Artemis's WRO 2026 Future Engineers vehicle.

The control logic is shared between simulation and the real robot: a single
`core` package holds the "brain" (state machine, PD control, tuned constants),
and both the simulator and the physical robot supply their own sensors and
actuators behind the same interface.

## Structure

```
src/
├── core/                       # Shared, hardware-independent control brain
│   ├── config.py               # Physics, control, sensor and scoring constants
│   ├── controller.py           # State machine and control logic
│   └── sensors.py              # SensorReading — the perception→control contract
├── sim/
│   ├── open-challenge-sim/     # Open challenge simulation & test suite
│   └── obstacle-challenge-sim/ # Obstacle challenge simulation (planned)
└── robot/                      # Physical robot control (Raspberry Pi) — scaffold
```

`core` is imported by both `sim` and `robot`, so tuning validated in simulation
carries directly to the real vehicle.

## Open challenge simulation

A 2D Pygame model of the track, robot, and sensors used to develop and validate
the controller. See [sim/open-challenge-sim/README.md](sim/open-challenge-sim/README.md).

```bash
cd sim/open-challenge-sim
python sim_viewer.py        # real-time visualizer
python test_pd_tuning.py    # PD/navigation test suite
python test_headless.py     # headless integration test
```

The entry-point scripts add `src/` to the path so `import core...` resolves when
run directly from their folder.

## Robot

Physical-vehicle code that reuses `core` and adds hardware drivers behind a
hardware-abstraction layer. Currently a scaffold — see
[robot/README.md](robot/README.md) for the layout and the remaining porting work.
