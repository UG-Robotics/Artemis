"""Colour-triggered, IMU-executed open-challenge controller.

The logic the sim passed, driven by the downward COLOUR sensor (not the camera —
the camera is for obstacle-challenge pillars only):

  straights   : IMU heading-hold onto a cardinal + a gentle ToF centring trim.
  corner TRIG : the colour sensor reads a line — orange => turn RIGHT (+90),
                blue => turn LEFT (-90). Direction is thus auto-detected from
                whichever colour is crossed first (WRO puts both an orange entry
                and a blue exit line at every corner; going CW you hit orange
                first, CCW you hit blue first — and the "mid-turn" gate below
                makes the second line a no-op, so only the entry colour acts).
  corner CONF : the ToF geometry must agree it's a real corner (a side opened up
                or the front wall is close) — this rejects a stray colour read on
                a straight, which is why the ToFs are the confirmation.
  corner EXEC : bump the heading target by ±90° and PID onto it (IMU leads the
                turn), exiting exactly 90° later, aligned and re-centred.

Guards: a new turn only fires when the previous one has COMPLETED (heading within
CORNER_REARM_GATE of target) and a debounce cooldown has elapsed — so a corner
can never stack past 90° and the exit-colour line can't double-trigger.
"""

from enum import Enum, auto

from core.config import (
    CONTROL_HZ,
    CORNER_REARM_GATE,
    CORNER_TRIGGER_FRONT,
    FINISH_TOF_TOLERANCE,
    KD_HEADING,
    KP_HEADING,
    KP_TRIM,
    MAX_STUCK_RECOVERIES,
    ROUND_TIME,
    SIDE_WALL_VALID,
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
    d = (target - current + 180) % 360 - 180
    return d + 360 if d <= -180 else d


class CIState(Enum):
    STARTING = auto()
    DRIVING = auto()
    TURNING = auto()
    FINISHING = auto()
    STOPPED = auto()


class ColorImuController:
    """Open-challenge: colour triggers, ToF confirms, IMU executes the 90° turn."""

    START_SETTLE_TICKS = 8

    def __init__(self, track_width: float = None):
        self.state = CIState.STARTING
        self.turns = 0
        self.turn_dir = 0            # locked from the first colour: +1 right, -1 left
        self.entry_color = None      # the colour that means "corner" this run
        self.target_heading = 0.0
        self._prev_err = 0.0
        self._corner_cooldown = 0
        self._exit_ticks = 0
        self._settle = 0
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
            self.state = CIState.STOPPED
            return
        if self._handle_stuck(sensors, robot):
            return
        if self.state == CIState.STARTING:
            self._starting(sensors, robot)
        elif self.state == CIState.DRIVING:
            self._driving(sensors, robot)
        elif self.state == CIState.TURNING:
            self._turning(sensors, robot)
        elif self.state == CIState.FINISHING:
            self._finishing(sensors, robot)
        elif self.state == CIState.STOPPED:
            robot.stop()

    # -- states ------------------------------------------------------------

    def _starting(self, sensors, robot):
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
        self.state = CIState.DRIVING

    def _driving(self, sensors, robot):
        if self._corner_cooldown > 0:
            self._corner_cooldown -= 1
        elif self._corner_triggered(sensors):
            self._begin_turn()
            return
        steer, _ = self._heading_steer(sensors, trim=True)
        robot.set_speed(SPEED_OPEN_CRUISE)
        robot.set_steering(steer)

    def _turning(self, sensors, robot):
        steer, aligned = self._heading_steer(sensors, trim=False)
        robot.set_speed(SPEED_OPEN_CORNER)
        robot.set_steering(steer)
        self._exit_ticks = self._exit_ticks + 1 if aligned else 0
        if self._exit_ticks >= TURN_EXIT_HOLD:
            self.turns += 1
            self._corner_cooldown = TURN_DEBOUNCE_TICKS
            self.state = (CIState.FINISHING if self.turns >= TURNS_TO_FINISH
                          else CIState.DRIVING)

    def _finishing(self, sensors, robot):
        if self._corner_cooldown > 0:
            self._corner_cooldown -= 1
        elif self._corner_triggered(sensors):
            self._begin_turn()
            return
        steer, _ = self._heading_steer(sensors, trim=True)
        robot.set_speed(SPEED_OPEN_CORNER)
        robot.set_steering(steer)
        if self.start_tof_front is not None:
            front_ok = abs(sensors.tof_front - self.start_tof_front) < FINISH_TOF_TOLERANCE
            rear_ok = abs(sensors.tof_rear - self.start_tof_rear) < FINISH_TOF_TOLERANCE
            if front_ok and rear_ok:
                robot.stop()
                self.state = CIState.STOPPED

    # -- trigger / steering ------------------------------------------------

    def _corner_triggered(self, sensors) -> bool:
        """Colour reads a line, the ToF geometry confirms a real corner, and the
        heading is aligned (previous turn complete)."""
        color = getattr(sensors, "color_detected", None)
        if color not in ('orange', 'blue'):
            return False
        # lock the entry colour + direction on the first corner; ignore the other
        # (exit) colour forever after
        if self.turn_dir == 0:
            self._pending_dir = 1 if color == 'orange' else -1
        elif color != self.entry_color:
            return False
        else:
            self._pending_dir = self.turn_dir
        # aligned = the previous turn has finished
        if abs(_angle_diff(self.target_heading, sensors.imu_heading)) >= CORNER_REARM_GATE:
            return False
        # ToF confirmation: a side opened up, or the front wall is close
        tof_ok = (max(sensors.tof_left, sensors.tof_right) > SIDE_WALL_VALID
                  or sensors.tof_front < CORNER_TRIGGER_FRONT)
        return tof_ok

    def _begin_turn(self):
        if self.turn_dir == 0:
            self.turn_dir = self._pending_dir
            self.entry_color = 'orange' if self.turn_dir == 1 else 'blue'
        self.target_heading = (self.target_heading + self.turn_dir * 90) % 360
        self._exit_ticks = 0
        self.state = CIState.TURNING

    def _heading_steer(self, sensors, trim: bool):
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

    # -- collision guard ---------------------------------------------------

    def _handle_stuck(self, sensors, robot) -> bool:
        if self._recover_ticks > 0:
            self._recover_ticks -= 1
            robot.set_speed(-SPEED_OPEN_CORNER)
            steer_away = -1 if sensors.tof_left < sensors.tof_right else 1
            robot.set_steering(steer_away * WALL_FOLLOW_MAX_STEER)
            if self._recover_ticks == 0:
                self._prev_err = _angle_diff(self.target_heading, sensors.imu_heading)
                self._corner_cooldown = TURN_DEBOUNCE_TICKS
                if self.state not in (CIState.FINISHING, CIState.STOPPED):
                    self.state = CIState.DRIVING
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
                self.state = CIState.STOPPED
                return True
            self._recover_ticks = STUCK_REVERSE_TICKS
            return True
        return False

    def get_state_name(self) -> str:
        return self.state.name
