"""Button bench test — watch one or more buttons wired: GPIO -> button -> GND.

Uses the Pi's internal pull-up, so no external resistor is needed:
    unpressed = HIGH (1)   pressed = LOW (0)

On start it reports each button's idle level, which catches the classic 4-pin
tactile-switch mistake: if you picked two pins from the *same* internal pair the
circuit is permanently closed, so the pin sits LOW forever (looks always
pressed). Use two DIAGONALLY opposite pins.

    python3 -m robot.bench.button_test          # default GPIO 27 (header pin 13)
    python3 -m robot.bench.button_test 27 4     # watch several at once

Uses RPi.GPIO, NOT pigpio: starting the pigpiod daemon resets the GPIO
subsystem, which drops the ToF XSHUT lines and knocks all four ToFs off the I2C
bus (they lose their reassigned addresses) — so the pigpio version killed the
live sensors when run alongside artemis-web. RPi.GPIO (the same library the ToF
driver uses) touches only the button pins. We never call GPIO.cleanup() on the
XSHUT-adjacent state — only the button pins we set up.

Ctrl+C to quit.
"""

import sys
import time

import RPi.GPIO as GPIO  # type: ignore

DEBOUNCE_S = 0.03


def main(pins):
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    for p in pins:
        GPIO.setup(p, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    time.sleep(0.1)

    print("idle check (want 1 = pull-up holding it high):")
    for p in pins:
        lvl = GPIO.input(p)
        note = ("OK" if lvl == 1 else
                "STUCK LOW -> wrong pins (same internal pair) or shorted to GND")
        print("  GPIO%-2d idle=%d  %s" % (p, lvl, note))

    print("\npress a button — Ctrl+C to quit")
    last = {p: GPIO.input(p) for p in pins}
    counts = {p: 0 for p in pins}
    try:
        while True:
            for p in pins:
                lvl = GPIO.input(p)
                if lvl == last[p]:
                    continue
                time.sleep(DEBOUNCE_S)          # settle, then re-read
                if GPIO.input(p) != lvl:
                    continue                    # bounce, ignore
                last[p] = lvl
                if lvl == 0:
                    counts[p] += 1
                    print("  GPIO%-2d PRESSED   (count %d)" % (p, counts[p]))
                else:
                    print("  GPIO%-2d released" % p)
            time.sleep(0.005)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up ONLY the button pins — GPIO.cleanup() with no args resets
        # every channel, which would drop the ToF XSHUT lines.
        GPIO.cleanup(pins)
        print("\ndone — presses: %s" % {("GPIO%d" % p): c for p, c in counts.items()})


if __name__ == "__main__":
    main([int(a) for a in sys.argv[1:]] or [27])
