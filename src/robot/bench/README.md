# Bench — hardware bring-up & debugging tools

Standalone throwaway tools for testing one component at a time during robot
bring-up. These are **not** part of the control loop — they run directly on the
Pi (or on an Arduino Uno used as a probe) to isolate wiring/driver problems
before wiring a device into `drivers/`.

| Tool | Runs on | What it does |
|------|---------|--------------|
| `i2c_scan/i2c_scan.ino` | Uno | Scan the I2C bus; TCS34725 colour sensor should show at `0x29`. |
| `color_test/color_test.ino` | Uno | Read raw RGB/clear from the TCS34725. |
| `tof_test/live_read.py` | Pi | Bring up all four VL53 ToFs via XSHUT re-addressing (`0x30`–`0x33`) and stream distances. |
| `servo_test/jog.py`, `jog_pi.py` | dev / Pi | Interactive steering-servo jogger (µs pulse) for finding centre and throw limits. `jog_pi.py` uses pigpio on the Pi. |
| `servo_test/servo_test.ino` | Uno | Sweep the servo from an Uno. |
| `motor_test/tb6612_uno_test*`, `chA_spin_test` | Uno | Drive the TB6612 motor driver directly to isolate the dead channel B. |
| `motor_test/uno_passive_probe` | Uno | Passive voltmeter: Pi drives the TB6612, Uno reads A01/A02 outputs. |
| `power_watch/power_watch.sh` (+ `.service`) | Pi | Log Pi power health (`vcgencmd get_throttled`) at boot then every 10 s. |

See each file's header comment for exact wiring and run instructions. Findings
from these tools are recorded in the bring-up notes (`docs/`, memory).
