#!/usr/bin/env python3
"""Interactive servo jogger for the Uno running servo_test.ino.

Usage:
    python jog.py [--port /dev/cu.usbmodem11201] [--baud 9600]

At the servo> prompt (the prompt shows the current pulse):
    1        creep +1 degree from the current position (relative)
    -1       creep -1 degree
    5 / -10  bigger relative steps, either direction
    a90      go to an ABSOLUTE angle (degrees)
    u1500    go to an ABSOLUTE pulse width (microseconds)
    c        centre (1500 us)
    ?        print current state
    q        quit

Relative steps are what you want for creeping up on a mechanical stop:
just hit "1" (or "-1") repeatedly and watch the wheel.
"""

import argparse
import sys
import time

import serial
from serial.tools import list_ports

US_PER_DEG = 1000.0 / 180.0   # sketch maps 0..180 deg -> 1000..2000 us
PULSE_MIN, PULSE_MAX = 500, 2500   # matches the sketch's attach() limits


def autodetect() -> str | None:
    for p in list_ports.comports():
        text = f"{p.device} {p.description} {p.manufacturer or ''}".lower()
        if "usbmodem" in p.device.lower() or "arduino" in text:
            return p.device
    return None


def converse(ser: serial.Serial, cmd: str):
    """Send a command, print the board's reply, return the reported pulse (us)."""
    ser.write((cmd + "\n").encode())
    time.sleep(0.05)
    pulse = None
    while ser.in_waiting:
        line = ser.readline().decode(errors="replace").rstrip()
        if not line:
            continue
        print(f"  < {line}")
        if "pulse=" in line:
            try:
                pulse = int(line.split("pulse=")[1].split("us")[0])
            except (IndexError, ValueError):
                pass
    return pulse


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=None, help="serial port (auto-detected if omitted)")
    ap.add_argument("--baud", type=int, default=9600)
    args = ap.parse_args()

    port = args.port or autodetect()
    if not port:
        print("No Arduino port found. Pass --port explicitly.", file=sys.stderr)
        return 1

    print(f"Opening {port} @ {args.baud}...")
    with serial.Serial(port, args.baud, timeout=1) as ser:
        time.sleep(2.0)  # Uno resets when the port opens; wait for the bootloader
        current = converse(ser, "?") or 1500
        print("Connected. Bare number = relative degrees (1, -1, 5). "
              "a<deg>/u<us> = absolute. c/?/q.")
        try:
            while True:
                raw = input(f"servo[{current}us]> ").strip()
                if raw.lower() in ("q", "quit", "exit"):
                    break
                if not raw:
                    continue
                low = raw.lower()
                pulse = None
                if low in ("c", "center", "centre"):
                    pulse = converse(ser, "c")
                elif low == "?":
                    pulse = converse(ser, "?")
                elif low.startswith("u"):
                    try:
                        pulse = converse(ser, f"u{int(low[1:])}")
                    except ValueError:
                        print("  ! usage: u<microseconds>, e.g. u1500")
                elif low.startswith("a"):
                    try:
                        deg = float(low[1:])
                    except ValueError:
                        print("  ! usage: a<degrees>, e.g. a90")
                        continue
                    target = int(round(1000 + deg * US_PER_DEG))
                    pulse = converse(ser, f"u{max(PULSE_MIN, min(PULSE_MAX, target))}")
                else:
                    try:
                        step_deg = float(raw)
                    except ValueError:
                        print("  ! number = relative degrees; or a<deg>/u<us>/c/?/q")
                        continue
                    target = int(round(current + step_deg * US_PER_DEG))
                    target = max(PULSE_MIN, min(PULSE_MAX, target))
                    pulse = converse(ser, f"u{target}")
                if pulse is not None:
                    current = pulse
        except (KeyboardInterrupt, EOFError):
            print()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
