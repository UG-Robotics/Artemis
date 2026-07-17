"""Shared start-button arming — the hot-swap point for any run script.

Every competition/bench script that should "boot, then wait for the physical
button" calls arm_start(). Keeping it here (not in one launcher) means swapping
which script the button drives is a one-line import, e.g. square_drive.py uses
the same button as open_challenge.py.

Uses RPi.GPIO — the SAME library the ToF driver uses for XSHUT — deliberately.
The old pigpio path needed the pigpiod daemon, and starting that daemon resets
the GPIO subsystem, dropping XSHUT LOW and knocking all four ToFs off the bus.
RPi.GPIO shares the driver's context, so no daemon and no pin reset. We never
call GPIO.cleanup() with no args (that would drop XSHUT mid-run).
"""

import time

from robot.hardware_config import Button

try:
    import RPi.GPIO as GPIO  # type: ignore
    _GPIO_AVAILABLE = True
except (ImportError, RuntimeError):
    GPIO = None
    _GPIO_AVAILABLE = False


def wait_for_start_button(timeout: float = None) -> bool:
    """Block until the start button is pressed (internal pull-up: pressed = LOW).

    Returns True on press, False on timeout. Falls back to an Enter prompt
    off-Pi. `timeout` (seconds) caps the wait; None waits forever.
    """
    if not _GPIO_AVAILABLE:
        input("RPi.GPIO unavailable — press Enter to start... ")
        return True
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(Button.PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    print(f"Waiting for start button (BCM {Button.PIN}, press to start)...")
    deadline = None if timeout is None else time.monotonic() + timeout
    while True:
        if GPIO.input(Button.PIN) == 0:         # HIGH = idle, LOW = pressed
            time.sleep(0.03)                    # debounce: must hold ~30ms
            if GPIO.input(Button.PIN) == 0:
                return True
        if deadline and time.monotonic() > deadline:
            print("Button wait timed out.")
            return False
        time.sleep(0.01)


def arm_start(wait_button: bool) -> bool:
    """Wait for the button, or run a 3-2-1 countdown when wait_button is False."""
    if wait_button:
        return wait_for_start_button()
    for n in (3, 2, 1):
        print(n)
        time.sleep(1.0)
    return True
