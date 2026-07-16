"""Open-challenge run — real-robot entry point. On the Pi:

    python -m robot.open_challenge                     # ToF-only (default mode)
    python -m robot.open_challenge --mode fusion       # ToF + camera (heading fusion
                                                       #  + line colour for direction)
    python -m robot.open_challenge --mode imu --direction cw
                                                       # gyro heading-hold Controller
                                                       #  (requires a healthy IMU)
    common flags:
      --now          skip the start button, 3-2-1 countdown
      --width 600    narrow track (default 1000mm)

MODES
  tof     WallFollowController on the four ToF sensors alone. The proven
          baseline: 30/30 sim score, no camera, no IMU.
  fusion  Same controller + CameraWorker: the camera adds an absolute
          wall-base heading (blended 0.7 ToF / 0.3 vision) and orange/blue
          line detection, which locks the driving direction by rule (orange
          first = CW) instead of the geometric guess. Do NOT run the web
          dashboard's camera at the same time (one Picamera2 owner only —
          stop artemis-web or export ARTEMIS_NO_CAMERA=1 for it).
  imu     The primary gyro brain (core.controller.Controller): heading-hold
          PID, corner = target_heading += 90 on the corner line colour.
          Contingency for a trustworthy IMU (e.g. a 9-axis drift-corrected
          unit). Uses the camera for line detection (no colour sensor) and a
          dead-reckoned pose shim for the controller's line-dedup distance.
          --direction cw|ccw is required: the corner-entry colour depends on
          it (auto-detection from the first line is a TODO).
"""

import argparse
import math
import time

from core.config import CONTROL_HZ, MAX_SPEED, TRACK_WIDTH_OPEN_WIDE
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
    """Minimal `track` stand-in — controllers only ask for the width."""

    def __init__(self, track_width: float):
        self.challenge_type = "open"
        self.track_width = track_width

    def get_local_track_width(self, x: float, y: float) -> float:
        return self.track_width


class DeadReckonedPose:
    """Pose shim for the gyro Controller (imu mode).

    Controller.update() reads robot.x/y (only to measure distance between
    line detections) and robot.distance_traveled (parking, unused in open).
    Without an encoder, integrate the COMMANDED speed along the IMU heading —
    plenty for a 200mm line-dedup radius. Motor/servo calls pass through to
    the real hardware.
    """

    def __init__(self, hw):
        self._hw = hw
        self.x = 0.0
        self.y = 0.0
        self.distance_traveled = 0.0
        self._speed_cmd = 0.0

    def integrate(self, heading_deg: float, dt: float):
        v = self._speed_cmd * MAX_SPEED
        self.x += v * math.cos(math.radians(heading_deg)) * dt
        self.y += v * math.sin(math.radians(heading_deg)) * dt
        self.distance_traveled += abs(v) * dt

    def set_speed(self, fraction: float):
        self._speed_cmd = fraction
        self._hw.set_speed(fraction)

    def set_steering(self, angle: float):
        self._hw.set_steering(angle)

    def stop(self):
        self._speed_cmd = 0.0
        self._hw.stop()


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


def _countdown(wait_button: bool) -> bool:
    if wait_button:
        return wait_for_start_button()
    for n in (3, 2, 1):
        print(n)
        time.sleep(1.0)
    return True


def run_wall_follow(track_width: float, wait_button: bool, use_camera: bool) -> None:
    """tof / fusion modes: WallFollowController, optionally camera-augmented."""
    hw = RealHardware(use_camera=False, use_imu=False, use_color=False)
    camera = None
    if use_camera:
        from robot.camera_worker import CameraWorker
        camera = CameraWorker()
        print(f"CameraWorker: {'live' if camera.available else 'UNAVAILABLE (ToF-only fallback)'}")
    world = OpenChallengeWorld(track_width)
    controller = WallFollowController(track_width=track_width)

    if not _countdown(wait_button):
        hw.close()
        return
    print("GO")

    dt = 1.0 / CONTROL_HZ
    try:
        while controller.state != WFState.STOPPED:
            tick_start = time.monotonic()
            sensors = hw.read_sensors()
            if camera is not None and camera.available:
                sensors.vision_heading = camera.vision_heading
                sensors.color_detected = camera.line_color
            controller.update(sensors, hw, world, dt)
            elapsed = time.monotonic() - tick_start
            if elapsed < dt:
                time.sleep(dt - elapsed)  # hold the control rate
    except KeyboardInterrupt:
        pass
    finally:
        hw.stop()
        if camera is not None:
            camera.close()
        hw.close()
        print(f"Done — state={controller.get_state_name()} "
              f"turns={controller.turns} t={controller.elapsed_time:.1f}s")


def run_gyro(track_width: float, wait_button: bool, direction: int) -> None:
    """imu mode: the primary gyro Controller (needs a healthy IMU + camera lines)."""
    from core.controller import Controller, State
    from robot.camera_worker import CameraWorker

    hw = RealHardware(use_camera=False, use_imu=True, use_color=False)
    camera = CameraWorker()
    if not camera.available:
        print("CameraWorker UNAVAILABLE — corner lines will not trigger; "
              "IMU mode needs the camera for line detection")
    pose = DeadReckonedPose(hw)
    world = OpenChallengeWorld(track_width)
    controller = Controller(driving_direction=direction, challenge_type='open')

    if not _countdown(wait_button):
        hw.close()
        return
    print("GO")

    dt = 1.0 / CONTROL_HZ
    try:
        while controller.state not in (State.FINISHED, State.STOPPED):
            tick_start = time.monotonic()
            sensors = hw.read_sensors()
            if camera.available:
                sensors.color_detected = camera.line_color
            controller.update(sensors, pose, world, dt)
            pose.integrate(sensors.imu_heading, dt)
            elapsed = time.monotonic() - tick_start
            if elapsed < dt:
                time.sleep(dt - elapsed)
    except KeyboardInterrupt:
        pass
    finally:
        pose.stop()
        camera.close()
        hw.close()
        print(f"Done — state={controller.state.name} laps={controller.laps_completed} "
              f"sections={controller.sections_passed} t={controller.elapsed_time:.1f}s")


def main() -> None:
    parser = argparse.ArgumentParser(description="WRO open-challenge run.")
    parser.add_argument("--mode", choices=("tof", "fusion", "imu"), default="tof",
                        help="tof = ToF-only wall-follower (default); fusion = + camera "
                             "heading/lines; imu = gyro heading-hold Controller")
    parser.add_argument("--now", action="store_true",
                        help="skip the start button; 3-2-1 countdown instead")
    parser.add_argument("--width", type=float, default=TRACK_WIDTH_OPEN_WIDE,
                        help="track width in mm (default 1000; use 600 for narrow)")
    parser.add_argument("--direction", choices=("cw", "ccw"),
                        help="imu mode only: driving direction (sets corner-entry colour)")
    args = parser.parse_args()

    if args.mode == "imu":
        if not args.direction:
            parser.error("--mode imu requires --direction cw|ccw")
        run_gyro(args.width, not args.now, 1 if args.direction == "cw" else -1)
    else:
        run_wall_follow(args.width, not args.now, use_camera=(args.mode == "fusion"))


if __name__ == "__main__":
    main()
