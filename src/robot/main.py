"""Control loop for the real robot. Run on the Pi: python -m robot.main

Scaffold for now. The drivers are stubs, and two things still need doing before
this runs (both stubbed below):
  - pose: the controller reads robot.x/y; we'll dead-reckon it from the encoder
    distance + IMU heading. For the open challenge it's only used for line-dedup,
    so it's a small job.
  - track map: there's no map on the real robot. OpenChallengeWorld fakes the few
    things the controller asks for; returning None for the section query makes it
    fall back to sensor/heading logic, which is fine.
"""

import time

from core.config import CONTROL_HZ, TRACK_WIDTH_OPEN_WIDE
from core.controller import Controller
from robot.hal import RealHardware


class PoseEstimator:
    """Dead-reckoned pose from encoder + IMU. TODO: implement (see module docs)."""

    def __init__(self, hardware: RealHardware):
        self.hw = hardware

    def update(self) -> None:
        raise NotImplementedError("we'll dead-reckon pose from the encoder + IMU")

    @property
    def x(self) -> float:
        raise NotImplementedError("we'll dead-reckon pose from the encoder + IMU")

    @property
    def y(self) -> float:
        raise NotImplementedError("we'll dead-reckon pose from the encoder + IMU")


class RobotIO:
    """The `robot` object the controller expects. Actuators/odometry are real; pose isn't yet."""

    def __init__(self, hardware: RealHardware, pose: PoseEstimator):
        self.hw = hardware
        self.pose = pose

    def set_speed(self, fraction: float) -> None:
        self.hw.set_speed(fraction)

    def set_steering(self, angle_deg: float) -> None:
        self.hw.set_steering(angle_deg)

    def stop(self) -> None:
        self.hw.stop()

    @property
    def distance_traveled(self) -> float:
        return self.hw.distance_traveled

    @property
    def x(self) -> float:
        return self.pose.x

    @property
    def y(self) -> float:
        return self.pose.y

    @property
    def angle(self) -> float:
        return self.hw.imu.heading()


class OpenChallengeWorld:
    """Stand-in for `track` in the open challenge — no real map needed."""

    def __init__(self, track_width: float = TRACK_WIDTH_OPEN_WIDE):
        self.challenge_type = "open"
        self.track_width = track_width
        self.parking_lot = None
        self.sections = []
        self.pillars = []

    def get_local_track_width(self, x: float, y: float) -> float:
        return self.track_width  # VERIFY: the run's actual width (600 or 1000mm)

    def get_section_at_position(self, x: float, y: float):
        return None  # no map; controller falls back to sensor/heading logic


def run(track_width: float = TRACK_WIDTH_OPEN_WIDE) -> None:
    """Run the open-challenge control loop at CONTROL_HZ until stopped."""
    hw = RealHardware(use_camera=False)
    pose = PoseEstimator(hw)
    robot_io = RobotIO(hw, pose)
    world = OpenChallengeWorld(track_width)
    controller = Controller(challenge_type="open", start_in_parking=False)

    dt = 1.0 / CONTROL_HZ
    try:
        while True:
            tick_start = time.monotonic()
            pose.update()
            sensors = hw.read_sensors()
            controller.update(sensors, robot_io, world, dt)
            elapsed = time.monotonic() - tick_start
            if elapsed < dt:
                time.sleep(dt - elapsed)  # hold the control rate
    except KeyboardInterrupt:
        pass
    finally:
        hw.stop()
        hw.close()


if __name__ == "__main__":
    raise SystemExit(
        "Not runnable yet — once we implement the drivers and pose/track stubs, "
        "we'll call run() here."
    )
