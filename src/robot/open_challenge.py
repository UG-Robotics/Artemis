"""Open-challenge run — real-robot entry point. On the Pi:

    python -m robot.open_challenge            # wait for the start button, then go
    python -m robot.open_challenge --now      # skip the button, 3-2-1 countdown
    python -m robot.open_challenge --width 600 # narrow track (default 1000mm)

Drives the ToF-only `WallFollowController` (no IMU, no colour, no pose) at
CONTROL_HZ until it finishes the 3 laps, the round time expires, or Ctrl-C.
This is the competition-style launcher for the open challenge; the obstacle
launcher will be a sibling once its perception (camera/pillars) is in.

Swap-in note: when the IMU + colour sensor are healthy, replace
`WallFollowController` below with `core.controller.Controller(challenge_type=
'open')` (plus a pose source) — the control loop here is identical.
"""

import argparse
import time

from core.config import CONTROL_HZ, TRACK_WIDTH_OPEN_WIDE
from core.wall_follow_controller import WallFollowController, WFState
from robot.hal import RealHardware
from robot.hardware_config import Button

try:
    import pigpio  # type: ignore
    _PIGPIO_AVAILABLE = True
except ImportError:
    pigpio = None
    _PIGPIO_AVAILABLE = False


class OpenChallengeWorld:
    """Minimal `track` stand-in — the wall-follower only asks for the width."""

    def __init__(self, track_width: float):
        self.challenge_type = "open"
        self.track_width = track_width

    def get_local_track_width(self, x: float, y: float) -> float:
        return self.track_width


def wait_for_start_button(timeout: float = None) -> bool:
    """Block until the start button is pressed (pressed = HIGH, 10k pulldown).

    Returns True once pressed. Falls back to a blocking prompt when pigpio isn't
    available (off-Pi) so the launcher still runs. `timeout` is a safety cap in
    seconds; None waits forever.
    """
    if not _PIGPIO_AVAILABLE:
        input("pigpio unavailable — press Enter to start... ")
        return True

    pi = pigpio.pi()
    if not pi.connected:
        input("pigpiod not running — press Enter to start... ")
        return True
    try:
        pi.set_mode(Button.PIN, pigpio.INPUT)
        pi.set_pull_up_down(Button.PIN, pigpio.PUD_DOWN)
        print(f"Waiting for start button (BCM {Button.PIN})...")
        deadline = None if timeout is None else time.monotonic() + timeout
        while pi.read(Button.PIN) == 0:
            if deadline and time.monotonic() > deadline:
                print("Button wait timed out.")
                return False
            time.sleep(0.01)
        return True
    finally:
        pi.stop()


def run(track_width: float = TRACK_WIDTH_OPEN_WIDE, wait_button: bool = True) -> None:
    """Run the open-challenge control loop until the run ends or Ctrl-C."""
    # ToF-only run: the IMU is untrusted and the colour sensor isn't wired.
    hw = RealHardware(use_camera=False, use_imu=False, use_color=False)
    world = OpenChallengeWorld(track_width)
    controller = WallFollowController(track_width=track_width)

    if wait_button:
        if not wait_for_start_button():
            hw.close()
            return
    else:
        for n in (3, 2, 1):
            print(n)
            time.sleep(1.0)
    print("GO")

    dt = 1.0 / CONTROL_HZ
    try:
        while controller.state != WFState.STOPPED:
            tick_start = time.monotonic()
            sensors = hw.read_sensors()
            controller.update(sensors, hw, world, dt)
            elapsed = time.monotonic() - tick_start
            if elapsed < dt:
                time.sleep(dt - elapsed)  # hold the control rate
    except KeyboardInterrupt:
        pass
    finally:
        hw.stop()
        hw.close()
        print(
            f"Done — state={controller.get_state_name()} "
            f"turns={controller.turns} t={controller.elapsed_time:.1f}s"
        )


def main() -> None:
    parser = argparse.ArgumentParser(description="WRO open-challenge run (ToF-only).")
    parser.add_argument("--now", action="store_true",
                        help="skip the start button; 3-2-1 countdown instead")
    parser.add_argument("--width", type=float, default=TRACK_WIDTH_OPEN_WIDE,
                        help="track width in mm (default 1000; use 600 for narrow)")
    args = parser.parse_args()
    run(track_width=args.width, wait_button=not args.now)


if __name__ == "__main__":
    main()
