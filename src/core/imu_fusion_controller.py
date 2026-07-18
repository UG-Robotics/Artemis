"""IMU-executed-turn fusion controller for the OPEN challenge.

Each sensor does the job it's best at — this is the answer to the two failure
modes we hit on the mat (colour-triggered corners fired early; ToF full-lock
turns hugged the inside wall):

  straights     : IMU heading-hold onto a fixed cardinal (KP/KD_HEADING) so the
                  robot tracks a true straight line, plus a gentle ToF lateral
                  trim to stay centred between the walls.
  corner DETECT : the FRONT ToF wall closing while a side reads open (the corner
                  mouth) — reliable geometry, not fooled by an early or repeated
                  line sighting.
  corner EXECUTE: bump the heading target by ±90° and PID onto it. The turn ends
                  exactly 90° later, aligned and re-centred by the trim — no
                  full-lock arc hugging the inside wall.
  direction     : the camera line colour locks the turn sense the first time it's
                  seen (orange = CW = right, blue = CCW = left). Until then the
                  wall-distance geometry is the fallback (the side reading farther
                  is the exit) — so it still works with the camera off.

Same `update(sensors, robot, track, dt)` contract as the other controllers, so
it runs in the sim (with the sim's noisy/drifting IMU) and on the Pi unchanged.
Needs a healthy IMU: the run wires RealHardware(use_imu=True) + the CameraWorker
for line colour, and resets the heading to 0 at GO.
"""

import math
from enum import Enum, auto

from core.config import (
    CONTROL_HZ,
    CORNER_OPEN_SIDE,
    CORNER_PERSIST_TICKS,
    CORNER_REARM_GATE,
    CORNER_TRIGGER_FRONT,
    FINISH_TOF_TOLERANCE,
    KD_HEADING,
    KP_HEADING,
    KP_TRIM,
    MAX_STUCK_RECOVERIES,
    ROUND_TIME,
    SPEED_OPEN_CORNER,
    SPEED_OPEN_CRUISE,
    STUCK_FRONT_MM,
    STUCK_REVERSE_TICKS,
    STUCK_TICKS,
    TURN_DEBOUNCE_TICKS,
    TURN_EXIT_HOLD,
    TURN_HEADING_GATE,
    TURNS_TO_FINISH,
    WALL_FOLLOW_MAX_STEER,
)


def _angle_diff(target: float, current: float) -> float:
    """Signed smallest target-current in degrees, in (-180, 180]."""
    d = (target - current + 180) % 360 - 180
    return d + 360 if d <= -180 else d


class IFState(Enum):
    STARTING = auto()
    DRIVING = auto()     # heading-hold straight, watching for a corner
    TURNING = auto()     # PID onto target_heading (± 90°)
    FINISHING = auto()   # laps done; roll to the start signature and stop
    STOPPED = auto()


class ImuFusionController:
    """Open-challenge driver: IMU heading-hold + ToF-detected, gyro-executed turns."""

    START_SETTLE_TICKS = 8

    def __init__(self, track_width: float = None):
        self.state = IFState.STARTING
        self.turns = 0
        self.turn_dir = 0            # +1 = right/CW, -1 = left/CCW; locked once
        self.target_heading = 0.0    # the cardinal we're holding
        self._prev_err = 0.0
        self._front_close_ticks = 0
        self._corner_cooldown = 0
        self._exit_ticks = 0
        self._settle = 0
        # collision guard (shared design with WallFollowController)
        self._stuck_ticks = 0
        self._stuck_count = 0
        self._recover_ticks = 0
        self.track_width = track_width
        self.start_tof_front = None
        self.start_tof_rear = None
        self.elapsed_time = 0.0

    # -- main loop ---------------------------------------------------------

    def update(self, sensors, robot, track, dt: float):
        self.elapsed_time += dt
        if self.track_width is None:
            self.track_width = track.get_local_track_width(0, 0)

        if self.elapsed_time >= ROUND_TIME:
            robot.stop()
            self.state = IFState.STOPPED
            return

        if self._handle_stuck(sensors, robot):
            return

        if self.state == IFState.STARTING:
            self._starting(sensors, robot)
        elif self.state == IFState.DRIVING:
            self._driving(sensors, robot)
        elif self.state == IFState.TURNING:
            self._turning(sensors, robot)
        elif self.state == IFState.FINISHING:
            self._finishing(sensors, robot)
        elif self.state == IFState.STOPPED:
            robot.stop()

    # -- states ------------------------------------------------------------

    def _starting(self, sensors, robot):
        """Let the ToF filter settle, snapshot the start signature, latch the
        heading reference, and begin driving."""
        self._settle += 1
        robot.set_steering(0)
        robot.set_speed(0.0)
        if self._settle < self.START_SETTLE_TICKS:
            return
        self.start_tof_front = sensors.tof_front
        self.start_tof_rear = sensors.tof_rear
        self.target_heading = sensors.imu_heading % 360
        self._prev_err = 0.0
        robot.set_speed(SPEED_OPEN_CRUISE)
        self.state = IFState.DRIVING

    def _driving(self, sensors, robot):
        self._lock_direction(sensors)
        if self._corner_cooldown > 0:
            self._corner_cooldown -= 1
        elif self._corner_ahead(sensors):
            self._begin_turn(sensors)
            return
        steer, _ = self._heading_steer(sensors, trim=True)
        robot.set_speed(SPEED_OPEN_CRUISE)
        robot.set_steering(steer)

    def _turning(self, sensors, robot):
        """PID onto target_heading; exit when we've held alignment briefly."""
        steer, aligned = self._heading_steer(sensors, trim=False)
        robot.set_speed(SPEED_OPEN_CORNER)
        robot.set_steering(steer)
        if aligned:
            self._exit_ticks += 1
        else:
            self._exit_ticks = 0
        if self._exit_ticks >= TURN_EXIT_HOLD:
            self.turns += 1
            self._corner_cooldown = TURN_DEBOUNCE_TICKS
            self._front_close_ticks = 0
            self.state = (IFState.FINISHING if self.turns >= TURNS_TO_FINISH
                          else IFState.DRIVING)

    def _finishing(self, sensors, robot):
        """Keep heading-holding (taking a corner if the finish sits past one)
        until the ToF signature matches the start section."""
        if self._corner_cooldown > 0:
            self._corner_cooldown -= 1
        elif self._corner_ahead(sensors):
            self._begin_turn(sensors)
            return
        steer, _ = self._heading_steer(sensors, trim=True)
        robot.set_speed(SPEED_OPEN_CORNER)
        robot.set_steering(steer)
        if self.start_tof_front is not None:
            front_ok = abs(sensors.tof_front - self.start_tof_front) < FINISH_TOF_TOLERANCE
            rear_ok = abs(sensors.tof_rear - self.start_tof_rear) < FINISH_TOF_TOLERANCE
            if front_ok and rear_ok:
                robot.stop()
                self.state = IFState.STOPPED

    # -- steering / detection ---------------------------------------------

    def _heading_steer(self, sensors, trim: bool):
        """PID the IMU heading onto target_heading; returns (steer, aligned).

        Positive steer = turn right = heading increases (verified on-robot,
        GYRO_Z_SIGN). When aligned and both side walls read real, add a gentle
        centring trim (steer right when there's more room on the right)."""
        err = _angle_diff(self.target_heading, sensors.imu_heading)
        d_err = err - self._prev_err
        self._prev_err = err
        steer = KP_HEADING * err + KD_HEADING * d_err
        aligned = abs(err) < TURN_HEADING_GATE
        if (trim and aligned
                and sensors.tof_left < self.track_width
                and sensors.tof_right < self.track_width):
            steer += (sensors.tof_right - sensors.tof_left) * KP_TRIM
        return max(-WALL_FOLLOW_MAX_STEER, min(WALL_FOLLOW_MAX_STEER, steer)), aligned

    def _corner_ahead(self, sensors) -> bool:
        """A corner = the front wall closing AND a side open (the corner mouth),
        held for CORNER_PERSIST_TICKS. The open-mouth test rejects a front wall
        with both sides present (a dead-end / mis-read), and the debounce rejects
        a single ToF spike."""
        mouth_open = max(sensors.tof_left, sensors.tof_right) > CORNER_OPEN_SIDE
        if sensors.tof_front < CORNER_TRIGGER_FRONT and mouth_open:
            self._front_close_ticks += 1
        else:
            self._front_close_ticks = 0
        return self._front_close_ticks >= CORNER_PERSIST_TICKS

    def _begin_turn(self, sensors):
        """Lock direction (geometry fallback), bump the heading target ±90°."""
        if self.turn_dir == 0:
            # the side that reads farther is the exit; turn toward it
            self.turn_dir = 1 if sensors.tof_right >= sensors.tof_left else -1
        self.target_heading = (self.target_heading + self.turn_dir * 90) % 360
        self._prev_err = _angle_diff(self.target_heading, sensors.imu_heading)
        self._exit_ticks = 0
        self._front_close_ticks = 0
        self.state = IFState.TURNING

    def _lock_direction(self, sensors):
        """Camera line colour locks the turn sense the first time it's seen:
        orange = CW = right (+1), blue = CCW = left (-1)."""
        if self.turn_dir == 0:
            color = getattr(sensors, "color_detected", None)
            if color == 'orange':
                self.turn_dir = 1
            elif color == 'blue':
                self.turn_dir = -1

    # -- collision guard ---------------------------------------------------

    def _handle_stuck(self, sensors, robot) -> bool:
        """Nose-first jam -> reverse out (steering back toward centre) and retry,
        up to MAX_STUCK_RECOVERIES, else stop. Returns True while it owns the tick."""
        if self._recover_ticks > 0:
            self._recover_ticks -= 1
            robot.set_speed(-SPEED_OPEN_CORNER)
            # steer to open the nose away from the closer side wall while backing
            steer_away = -1 if sensors.tof_left < sensors.tof_right else 1
            robot.set_steering(steer_away * WALL_FOLLOW_MAX_STEER)
            if self._recover_ticks == 0:
                self._prev_err = _angle_diff(self.target_heading, sensors.imu_heading)
                self._corner_cooldown = TURN_DEBOUNCE_TICKS
                if self.state not in (IFState.FINISHING, IFState.STOPPED):
                    self.state = IFState.DRIVING
            return True

        if sensors.tof_front < STUCK_FRONT_MM:
            self._stuck_ticks += 1
        else:
            self._stuck_ticks = 0

        if self._stuck_ticks >= STUCK_TICKS:
            self._stuck_ticks = 0
            self._stuck_count += 1
            if self._stuck_count > MAX_STUCK_RECOVERIES:
                robot.stop()
                self.state = IFState.STOPPED
                return True
            self._recover_ticks = STUCK_REVERSE_TICKS
            return True
        return False

    # -- introspection -----------------------------------------------------

    def get_state_name(self) -> str:
        return self.state.name
