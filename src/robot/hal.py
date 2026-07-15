"""Hardware abstraction layer.

`Hardware` is the contract the controller needs from "the robot" — the same shape
the sim's `Robot` already has, so `core.Controller` drives either one. `RealHardware`
is the on-Pi implementation that wires up the drivers.
"""

from typing import Protocol, runtime_checkable

from core.sensors import SensorReading
from robot.drivers.motor import MotorDriver
from robot.drivers.servo import ServoDriver
from robot.drivers.tof import TofArray
from robot.drivers.imu import ImuDriver
from robot.drivers.color import ColorSensor
from robot.drivers.camera import Camera


@runtime_checkable
class Hardware(Protocol):
    """What the control loop needs from a robot, real or simulated."""

    def read_sensors(self) -> SensorReading: ...
    def set_speed(self, fraction: float) -> None: ...
    def set_steering(self, angle_deg: float) -> None: ...
    def stop(self) -> None: ...

    @property
    def distance_traveled(self) -> float: ...


class RealHardware:
    """`Hardware` backed by the physical sensors and actuators.

    Constructs fine on a dev machine; the driver calls raise NotImplementedError
    until we implement them on the Pi.
    """

    def __init__(self, use_camera: bool = True, use_imu: bool = True,
                 use_color: bool = True):
        self.motor = MotorDriver()
        self.servo = ServoDriver()
        self.tof = TofArray()
        # IMU and colour are optional so a ToF-only run (open challenge on the
        # current build) doesn't construct or poll unhealthy hardware — the IMU
        # is untrustworthy and the colour sensor isn't wired yet (2026-07-14).
        self.imu = ImuDriver() if use_imu else None
        self.color = ColorSensor() if use_color else None
        self.camera = Camera() if use_camera else None

    def read_sensors(self) -> SensorReading:
        """Poll every enabled sensor into one reading. Disabled sensors report
        their neutral default (heading 0.0, no colour, no pillars)."""
        d = self.tof.read_all()
        return SensorReading(
            tof_front=d["front"],
            tof_rear=d["rear"],
            tof_left=d["left"],
            tof_right=d["right"],
            imu_heading=self.imu.heading() if self.imu else 0.0,
            color_detected=self.color.detect() if self.color else None,
            pillars_visible=self.camera.detect_pillars() if self.camera else [],
        )

    def set_speed(self, fraction: float) -> None:
        self.motor.set_speed_fraction(fraction)

    def set_steering(self, angle_deg: float) -> None:
        self.servo.set_angle(angle_deg)

    def stop(self) -> None:
        self.motor.stop()
        self.servo.center()

    @property
    def distance_traveled(self) -> float:
        return self.motor.distance_traveled

    def close(self) -> None:
        """Release every driver's resources."""
        for dev in (self.motor, self.servo, self.tof, self.imu, self.color, self.camera):
            if dev is not None:
                dev.close()
