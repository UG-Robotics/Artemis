"""ToF-only wall-following controller for the OPEN challenge.

Why this exists (2026-07-14): the primary brain, `core.controller.Controller`,
is gyro heading-hold — every steering command is a PID on `imu_heading` and a
corner is `target_heading += 90°` triggered by the colour sensor. On the current
build the IMU is untrustworthy and the colour sensor isn't wired, so that
controller can't drive the real robot yet. This one drives the open challenge
from the four ToF distances alone — no IMU, no colour, no pose:

  - straights : PD centring between the left/right walls (KP_WALL/KD_WALL),
                falling back to single-wall following when one side opens up,
  - corners   : the front wall closes -> steer toward the open side until the
                front reopens; the turn direction locks on the first corner
                (a WRO loop only ever turns one way),
  - finish    : after TURNS_TO_FINISH corners (4 × 3 laps) roll on until the ToF
                signature matches the start section, then stop.

It exposes the SAME `update(sensors, robot, track, dt)` contract as
`Controller`, so it runs unchanged in the sim (tune the PID there first — see
`src/sim/open-challenge-sim/test_pd_tuning.py`) and on the Pi, and can be
swapped back for the gyro `Controller` once the IMU/colour are healthy.

Colour is used only as an OPTIONAL cross-check: when `sensors.color_detected`
is present it confirms a corner, but nothing here depends on it.
"""

from enum import Enum, auto

from core.config import (
    CONTROL_HZ,
    CORNER_CLEAR_FRONT,
    CORNER_PERSIST_TICKS,
    CORNER_TRIGGER_FRONT,
    FINISH_TOF_TOLERANCE,
    KD_WALL,
    KP_WALL,
    ROUND_TIME,
    SIDE_WALL_VALID,
    SPEED_OPEN_CORNER,
    SPEED_OPEN_CRUISE,
    TURN_DEBOUNCE_TICKS,
    TURN_EXIT_HOLD,
    TURN_MIN_TICKS,
    TURN_STEER,
    TURNS_TO_FINISH,
    WALL_FOLLOW_MAX_STEER,
    WALL_SETPOINT,
)


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


class WFState(Enum):
    """States for the ToF wall-follower."""
    STARTING = auto()
    DRIVING = auto()    # PD centring on the straight
    TURNING = auto()    # steering through a corner until the front reopens
    FINISHING = auto()  # laps done; roll to the start signature and stop
    STOPPED = auto()


class WallFollowController:
    """Open-challenge driver that needs only the four ToF sensors."""

    def __init__(self, track_width: float = None):
        self.state = WFState.STARTING
        self.turns = 0
        self.turn_dir = 0            # +1 = right, -1 = left; locked on first corner
        self._prev_center_err = 0.0
        self._turn_ticks = 0
        self._turn_clear_ticks = 0   # consecutive "realigned" ticks inside a turn
        self._corner_cooldown = 0    # ticks left before a new corner may trigger
        self._front_close_ticks = 0  # consecutive ticks the front has been < trigger
        self.track_width = track_width  # None -> ask the world each tick

        # Start-section ToF signature, snapshotted at launch, used to stop the run
        # roughly where it began.
        self.start_tof_front = None
        self.start_tof_rear = None

        self.elapsed_time = 0.0

    # -- main loop ---------------------------------------------------------

    def update(self, sensors, robot, track, dt: float):
        """Called at CONTROL_HZ. Same signature as Controller.update()."""
        self.elapsed_time += dt
        if self.track_width is None:
            # WallFollowController never reads pose, so x/y are unused here.
            self.track_width = track.get_local_track_width(0, 0)

        if self.elapsed_time >= ROUND_TIME:
            robot.stop()
            self.state = WFState.STOPPED
            return

        if self.state == WFState.STARTING:
            self._handle_starting(sensors, robot)
        elif self.state == WFState.DRIVING:
            self._handle_driving(sensors, robot)
        elif self.state == WFState.TURNING:
            self._handle_turning(sensors, robot)
        elif self.state == WFState.FINISHING:
            self._handle_finishing(sensors, robot)
        elif self.state == WFState.STOPPED:
            robot.stop()

    # -- states ------------------------------------------------------------

    def _handle_starting(self, sensors, robot):
        """Snapshot the start signature and begin driving."""
        self.start_tof_front = sensors.tof_front
        self.start_tof_rear = sensors.tof_rear
        self._prev_center_err = 0.0
        robot.set_steering(0)
        robot.set_speed(SPEED_OPEN_CRUISE)
        self.state = WFState.DRIVING

    def _handle_driving(self, sensors, robot):
        """PD-centre on the straight; hand off to TURNING when a corner is seen."""
        if self._corner_cooldown > 0:
            self._corner_cooldown -= 1
        elif self._corner_ahead(sensors):
            self._begin_turn(sensors)
            return
        robot.set_speed(SPEED_OPEN_CRUISE)
        robot.set_steering(self._wall_follow_steer(sensors))

    def _handle_turning(self, sensors, robot):
        """Hold a corner-ward steer until the corner is genuinely behind us.

        The corner is done when the front has reopened AND a side wall is back in
        range — i.e. we're realigned in the new corridor, not just glimpsing clear
        across the corner mid-sweep. Requiring TURN_EXIT_HOLD consecutive clear
        ticks adds hysteresis so a single noisy frame doesn't end the turn early.
        """
        self._turn_ticks += 1
        robot.set_speed(SPEED_OPEN_CORNER)
        robot.set_steering(self.turn_dir * TURN_STEER)

        # Both side walls back in range = parallel in the new corridor. Requiring
        # both (not either) rejects the mid-corner diagonal glimpse, where one
        # side is still the open corner mouth, so a 90° corner isn't cut into two.
        both_sides_back = (sensors.tof_left < SIDE_WALL_VALID
                           and sensors.tof_right < SIDE_WALL_VALID)
        realigned = sensors.tof_front > CORNER_CLEAR_FRONT and both_sides_back
        if self._turn_ticks >= TURN_MIN_TICKS and realigned:
            self._turn_clear_ticks += 1
        else:
            self._turn_clear_ticks = 0

        if self._turn_clear_ticks >= TURN_EXIT_HOLD:
            self.turns += 1
            self._prev_center_err = 0.0
            self._corner_cooldown = TURN_DEBOUNCE_TICKS  # one corner = one turn
            if self.turns >= TURNS_TO_FINISH:
                self.state = WFState.FINISHING
            else:
                self.state = WFState.DRIVING

    def _handle_finishing(self, sensors, robot):
        """Laps done: keep wall-following until the ToF signature matches start."""
        # Still take corners if the finish line sits past one.
        if self._corner_cooldown > 0:
            self._corner_cooldown -= 1
        elif self._corner_ahead(sensors):
            self._begin_turn(sensors)
            return
        robot.set_speed(SPEED_OPEN_CORNER)
        robot.set_steering(self._wall_follow_steer(sensors))

        if self.start_tof_front is not None:
            front_ok = abs(sensors.tof_front - self.start_tof_front) < FINISH_TOF_TOLERANCE
            rear_ok = abs(sensors.tof_rear - self.start_tof_rear) < FINISH_TOF_TOLERANCE
            if front_ok and rear_ok:
                robot.stop()
                self.state = WFState.STOPPED

    # -- steering / detection ---------------------------------------------

    def _wall_follow_steer(self, sensors):
        """PD steering to stay centred (or hold a setpoint against one wall).

        Sign convention matches the gyro controller's trim term: positive
        steering = turn right. tof_right is the gap to the right wall, so
        (tof_right - tof_left) > 0 means more room on the right => we're hugging
        the left => steer right to recover.
        """
        left, right = sensors.tof_left, sensors.tof_right
        left_valid = left < SIDE_WALL_VALID
        right_valid = right < SIDE_WALL_VALID

        if left_valid and right_valid:
            err = right - left
        elif left_valid:                      # right side is an opening: hold off left
            err = WALL_SETPOINT - left
        elif right_valid:                     # left side is an opening: hold off right
            err = right - WALL_SETPOINT
        else:
            err = 0.0                         # both open (mid-corner mouth): go straight

        d_err = err - self._prev_center_err
        self._prev_center_err = err
        steer = KP_WALL * err + KD_WALL * d_err
        return _clamp(steer, -WALL_FOLLOW_MAX_STEER, WALL_FOLLOW_MAX_STEER)

    def _corner_ahead(self, sensors):
        """A corner is the front wall closing in — and STAYING close.

        The front reading must sit below the trigger for CORNER_PERSIST_TICKS
        consecutive ticks before a corner fires. A real corner approach keeps the
        front closing for many ticks, so the debounce costs ~persist/CONTROL_HZ
        seconds of reaction; a single-frame ToF dropout (a stray near read) can't
        start a phantom turn. Colour, when present, confirms a corner but is
        never required (hybrid trigger — see module docstring).
        """
        if sensors.tof_front < CORNER_TRIGGER_FRONT:
            self._front_close_ticks += 1
        else:
            self._front_close_ticks = 0
        return self._front_close_ticks >= CORNER_PERSIST_TICKS

    def _begin_turn(self, sensors):
        """Enter TURNING, locking the loop's turn direction on the first corner."""
        if self.turn_dir == 0:
            # Turn toward whichever side has more room (the outside wall recedes
            # at a corner; the inside stays close).
            self.turn_dir = 1 if sensors.tof_right >= sensors.tof_left else -1
        self._turn_ticks = 0
        self._turn_clear_ticks = 0
        self._front_close_ticks = 0
        self.state = WFState.TURNING

    # -- introspection (telemetry / debugging) ----------------------------

    def get_state_name(self) -> str:
        return self.state.name
