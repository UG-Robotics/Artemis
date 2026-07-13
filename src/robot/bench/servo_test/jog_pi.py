#!/usr/bin/env python3
"""Interactive steering-servo jogger, run ON the Pi (uses pigpio directly).

    ssh pi@<pi> python3 -i /home/pi/artemis/src/robot/bench/servo_test/jog_pi.py

At the servo> prompt (shows the current pulse in us):
    10 / -10   relative jog in microseconds (any signed integer)
    u1500      go to an ABSOLUTE pulse width
    c          centre (1500 us)
    q          quit (stops pulses)

Calibration procedure:
  1. `c`, confirm wheels straight; if not, creep until straight -> PULSE_CENTER_US
  2. creep + until the wheels hit the right-hand 35 deg stop -> PULSE_MAX_US
  3. creep - past centre to the left-hand stop -> PULSE_MIN_US
Record all three in robot/hardware_config.py (Servo class).
"""

import sys
import pathlib

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[3]))  # src/

import pigpio
from robot.hardware_config import Servo as ServoConfig

LIMIT_LO, LIMIT_HI = 800, 2200  # don't command past typical mechanical range

pi = pigpio.pi()
if not pi.connected:
    sys.exit("pigpiod not running")

pulse = ServoConfig.PULSE_CENTER_US
pi.set_servo_pulsewidth(ServoConfig.PIN_SIGNAL, pulse)

print(__doc__)
while True:
    try:
        cmd = input(f"servo({pulse}us)> ").strip().lower()
    except (EOFError, KeyboardInterrupt):
        cmd = "q"
    if not cmd:
        continue
    if cmd == "q":
        pi.set_servo_pulsewidth(ServoConfig.PIN_SIGNAL, 0)
        pi.stop()
        print("stopped")
        break
    if cmd == "c":
        pulse = ServoConfig.PULSE_CENTER_US
    elif cmd.startswith("u"):
        try:
            pulse = int(cmd[1:])
        except ValueError:
            print("?"); continue
    else:
        try:
            pulse += int(cmd)
        except ValueError:
            print("?"); continue
    pulse = max(LIMIT_LO, min(LIMIT_HI, pulse))
    pi.set_servo_pulsewidth(ServoConfig.PIN_SIGNAL, pulse)
