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

import math
from enum import Enum, auto

from core.tof_heading import TofHeadingEstimator

from core.config import (
    CONTROL_HZ,
    KP_HEADING_WF,
    CORNER_CLEAR_FRONT,
    CORNER_OPEN_SIDE,
    CORNER_PERSIST_TICKS,
    CORNER_TRIGGER_FRONT,
    FINISH_TOF_TOLERANCE,
    KD_WALL,
    KP_WALL,
    MAX_SPEED,
    ROUND_TIME,
    SIDE_WALL_VALID,
    MAX_STUCK_RECOVERIES,
    SIDE_SUM_REALIGNED,
    STUCK_FRONT_MM,
    STUCK_REVERSE_TICKS,
    STUCK_TICKS,
    SILENT_CORNER_ARC,
    SILENT_CORNER_DECAY,
    SPEED_OPEN_CORNER,
    SPEED_OPEN_CRUISE,
    TURN_ARC_MAX,
    TURN_ARC_MIN,
    TURN_DEBOUNCE_TICKS,
    TURN_EXIT_HOLD,
    TURN_STEER,
    TURNS_TO_FINISH,
    VISION_FUSE_WEIGHT,
    WALL_FOLLOW_MAX_STEER,
    WALL_SETPOINT,
    WHEELBASE,
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
        self._turn_arc = 0.0         # dead-reckoned degrees swept in this turn
        self._stuck_ticks = 0        # consecutive ticks with the front pinned tiny
        self._stuck_count = 0        # jams recovered so far this run
        self._recover_ticks = 0      # ticks left in a reverse-recovery
        self._drive_arc = 0.0        # leaky steering integral while DRIVING
        self._hold_dist = None       # single-wall mode: distance being held
        self._front_close_ticks = 0  # consecutive ticks the front has been < trigger
        # Gyro-free heading from side-ToF rates; valid on straights only, so it
        # is fed in DRIVING/FINISHING and reset around every turn.
        self._heading = TofHeadingEstimator()
        self._speed_cmd = 0.0        # last commanded speed fraction (v estimate)
        self._dt = 1.0 / CONTROL_HZ
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
        self._dt = dt
        if self.track_width is None:
            # WallFollowController never reads pose, so x/y are unused here.
            self.track_width = track.get_local_track_width(0, 0)

        if self.elapsed_time >= ROUND_TIME:
            robot.stop()
            self.state = WFState.STOPPED
            return

        if self._handle_stuck(sensors, robot):
            return   # jammed / recovering — skip the normal state machine this tick

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

    # -- collision guard ---------------------------------------------------

    def _handle_stuck(self, sensors, robot) -> bool:
        """Detect a nose-first jam and reverse out of it. Returns True while the
        guard owns the tick (mid-recovery, or just detected a jam).

        A front ToF pinned below STUCK_FRONT_MM for STUCK_TICKS is the robot
        grinding a wall, not a corner — without this it keeps reading "corner"
        and the turn count runs away. On a jam it reverses (steering away from
        the locked turn side) for STUCK_REVERSE_TICKS, then resumes DRIVING;
        after MAX_STUCK_RECOVERIES it gives up and stops."""
        if self._recover_ticks > 0:
            self._recover_ticks -= 1
            robot.set_speed(-SPEED_OPEN_CORNER)
            robot.set_steering(-self.turn_dir * TURN_STEER if self.turn_dir else 0)
            if self._recover_ticks == 0:
                self._prev_center_err = 0.0
                self._heading.reset()
                self._corner_cooldown = TURN_DEBOUNCE_TICKS  # don't insta-recount
                if self.state not in (WFState.FINISHING, WFState.STOPPED):
                    self.state = WFState.DRIVING
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
                self.state = WFState.STOPPED
                return True
            self._recover_ticks = STUCK_REVERSE_TICKS
            robot.set_speed(-SPEED_OPEN_CORNER)
            robot.set_steering(-self.turn_dir * TURN_STEER if self.turn_dir else 0)
            return True
        return False

    # -- states ------------------------------------------------------------

    START_SETTLE_TICKS = 8

    def _handle_starting(self, sensors, robot):
        """Let the ToF filter warm up, then snapshot the start signature.

        The signature (front/rear at launch) is what stops the run in the right
        place; snapshotting it from the very first reading risks baking a
        dropout spike into the finish condition (premature stops seen in noisy
        sim runs). The robot starts rolling immediately — only the snapshot
        waits."""
        robot.set_steering(0)
        self._speed_cmd = SPEED_OPEN_CRUISE
        robot.set_speed(SPEED_OPEN_CRUISE)
        self._start_ticks = getattr(self, '_start_ticks', 0) + 1
        if self._start_ticks < self.START_SETTLE_TICKS:
            return
        self.start_tof_front = sensors.tof_front
        self.start_tof_rear = sensors.tof_rear
        self._prev_center_err = 0.0
        self.state = WFState.DRIVING

    def _handle_driving(self, sensors, robot):
        """PD-centre on the straight; hand off to TURNING when a corner is seen."""
        # Direction from the first corner line when a colour source exists
        # (camera line detection or the sim's mat): WRO layout — orange met
        # first = clockwise = right turns, blue first = counter-clockwise.
        # Rule-guaranteed, so it pre-empts the geometric more-open-side guess
        # at the first corner; the geometry remains the no-camera fallback.
        if self.turn_dir == 0 and sensors.color_detected in ('orange', 'blue'):
            self.turn_dir = 1 if sensors.color_detected == 'orange' else -1
        if self._corner_cooldown > 0:
            self._corner_cooldown -= 1
        elif self._corner_ahead(sensors):
            self._begin_turn(sensors)
            return
        self._speed_cmd = SPEED_OPEN_CRUISE
        robot.set_speed(SPEED_OPEN_CRUISE)
        steer = self._wall_follow_steer(sensors)
        robot.set_steering(steer)
        self._track_silent_corner(steer)

    def _track_silent_corner(self, steer):
        """Count a corner the wall-follower steered around without triggering.

        Hold-dist mode happily tracks the outer wall around a 90 deg bend (seen
        in traces: 22s between turn events = two straights + an uncounted
        corner, so FINISHING armed late and the stop signature matched in the
        wrong straight). A leaky integral of the commanded steering stays near
        zero on straights (corrections are symmetric) but accumulates the bend.
        """
        v = self._speed_cmd * MAX_SPEED
        self._drive_arc = (self._drive_arc * SILENT_CORNER_DECAY
                           + math.degrees(v * math.tan(math.radians(steer))
                                          / WHEELBASE) * self._dt)
        if self.turn_dir != 0 and self._drive_arc * self.turn_dir >= SILENT_CORNER_ARC:
            self.turns += 1
            self._drive_arc = 0.0
            self._prev_center_err = 0.0
            self._heading.reset()
            self._corner_cooldown = TURN_DEBOUNCE_TICKS
            if self.turns >= TURNS_TO_FINISH:
                self.state = WFState.FINISHING

    def _handle_turning(self, sensors, robot):
        """Hold a corner-ward steer until the corner is genuinely behind us.

        The corner is done when the front has reopened AND a side wall is back in
        range — i.e. we're realigned in the new corridor, not just glimpsing clear
        across the corner mid-sweep. Requiring TURN_EXIT_HOLD consecutive clear
        ticks adds hysteresis so a single noisy frame doesn't end the turn early.
        """
        self._turn_ticks += 1
        self._speed_cmd = SPEED_OPEN_CORNER
        robot.set_speed(SPEED_OPEN_CORNER)
        robot.set_steering(self.turn_dir * TURN_STEER)
        # Dead-reckoned arc: omega = v * tan(steer) / wheelbase (bicycle model,
        # commanded values — no gyro, no encoder). Good to ~±20%, which the
        # [TURN_ARC_MIN, TURN_ARC_MAX] window absorbs.
        v = self._speed_cmd * MAX_SPEED
        omega = math.degrees(v * math.tan(math.radians(TURN_STEER)) / WHEELBASE)
        self._turn_arc += omega * self._dt

        # Realigned = the side readings again SUM to a corridor width. The sum
        # is width-agnostic (600-1000mm sections all pass), unlike the old
        # per-side < SIDE_WALL_VALID test, which a centred robot in a 1000mm
        # corridor could never satisfy — it kept spinning past 90° and
        # ping-ponged down the same corridor (found via ground-truth score
        # tracing). The mid-corner diagonal glimpse still fails the sum: the
        # open corner mouth reads >1500 on one side.
        sides_sum_ok = (sensors.tof_left + sensors.tof_right) < SIDE_SUM_REALIGNED
        realigned = sensors.tof_front > CORNER_CLEAR_FRONT and sides_sum_ok
        if self._turn_arc >= TURN_ARC_MIN and realigned:
            self._turn_clear_ticks += 1
        else:
            self._turn_clear_ticks = 0

        # KNOWN LIMIT: an arc-cap force-exit with the front still blocked can
        # re-trigger the same physical corner after the cooldown and count it
        # twice (~15% of noisy runs finish one straight early, 22 sections
        # instead of 24). Requiring geometry-confirmed exits to count was tried
        # and is WORSE (wide corridors rarely confirm -> turns never reach 12).
        # The real fix is counting corners by the camera seeing the orange/blue
        # corner lines, not ToF geometry.
        if self._turn_clear_ticks >= TURN_EXIT_HOLD or self._turn_arc >= TURN_ARC_MAX:
            self.turns += 1
            self._prev_center_err = 0.0
            self._heading.reset()
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
        self._speed_cmd = SPEED_OPEN_CORNER
        robot.set_speed(SPEED_OPEN_CORNER)
        steer = self._wall_follow_steer(sensors)
        robot.set_steering(steer)
        self._track_silent_corner(steer)

        if self.start_tof_front is not None:
            front_ok = abs(sensors.tof_front - self.start_tof_front) < FINISH_TOF_TOLERANCE
            rear_ok = abs(sensors.tof_rear - self.start_tof_rear) < FINISH_TOF_TOLERANCE
            if front_ok and rear_ok:
                robot.stop()
                self.state = WFState.STOPPED

    # -- steering / detection ---------------------------------------------

    def _wall_follow_steer(self, sensors):
        """P steering on lateral offset + damping from estimated heading.

        Sign convention matches the gyro controller's trim term: positive
        steering = turn right. tof_right is the gap to the right wall, so
        (tof_right - tof_left) > 0 means more room on the right => we're hugging
        the left => steer right to recover.

        The damping term is the gyro-free heading estimate (side-ToF distance
        rates, core/tof_heading.py): a heading error toward the right wall gets
        a proportional left steer, which is what actually separates "off-centre
        but parallel" (P term only) from "centred but angled" (heading term
        only) — the ambiguity a single ToF per side cannot resolve. Until the
        estimator has a full window (start of a straight), the old KD on the
        raw differential stands in.
        """
        left, right = sensors.tof_left, sensors.tof_right
        left_valid = left < SIDE_WALL_VALID
        right_valid = right < SIDE_WALL_VALID

        # Single-wall mode holds the distance we HAD when the other side opened
        # (clamped to a sane band), not a fixed setpoint: pulling to
        # WALL_SETPOINT=250 from the centre of a 1000mm corridor is a violent
        # lateral manoeuvre right before every corner (openings in the open
        # challenge only ever mean a corner is near). In a 600mm corridor
        # centre and setpoint nearly coincide, which is why this only hurt on
        # wide tracks (ground-truth score traces).
        if left_valid and right_valid:
            err = right - left
            self._hold_dist = None
        elif left_valid:                      # right side is an opening: hold off left
            if self._hold_dist is None:
                self._hold_dist = _clamp(left, 200, 500)
            err = self._hold_dist - left
        elif right_valid:                     # left side is an opening: hold off right
            if self._hold_dist is None:
                self._hold_dist = _clamp(right, 200, 500)
            err = right - self._hold_dist
        else:
            err = 0.0                         # both open (mid-corner mouth): go straight

        d_err = err - self._prev_center_err
        self._prev_center_err = err

        tof_heading = self._heading.update(
            left, right, left_valid, right_valid,
            self._speed_cmd * MAX_SPEED, self._dt,
        )
        # Fuse the two gyro-free heading sources. The ToF-rate estimate is the
        # precise one, so it dominates; the camera wall-base estimate is
        # absolute and needs no motion, so it carries the ticks where the ToF
        # estimator is blind (window refilling after a turn, a side open).
        vision = getattr(sensors, "vision_heading", None)
        if tof_heading is not None and vision is not None:
            heading = (1 - VISION_FUSE_WEIGHT) * tof_heading + VISION_FUSE_WEIGHT * vision
        elif tof_heading is not None:
            heading = tof_heading
        else:
            heading = vision  # may be None

        if heading is not None:
            # heading > 0 = nose toward the right wall -> steer left.
            steer = KP_WALL * err - KP_HEADING_WF * heading
        else:
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
        # A real corner also shows its mouth: one side reads clear across the
        # track. Without this, a heading excursion that points the front ToF at
        # a side wall fires a phantom corner mid-straight (instrumented: real
        # triggers >= 2400 on the open side, phantoms <= 1360).
        mouth_open = max(sensors.tof_left, sensors.tof_right) > CORNER_OPEN_SIDE
        return self._front_close_ticks >= CORNER_PERSIST_TICKS and mouth_open

    def _begin_turn(self, sensors):
        """Enter TURNING, locking the loop's turn direction on the first corner."""
        if self.turn_dir == 0:
            # Turn toward whichever side has more room (the outside wall recedes
            # at a corner; the inside stays close).
            self.turn_dir = 1 if sensors.tof_right >= sensors.tof_left else -1
        self._turn_ticks = 0
        self._turn_arc = 0.0
        self._drive_arc = 0.0
        self._turn_clear_ticks = 0
        self._front_close_ticks = 0
        self._heading.reset()
        self.state = WFState.TURNING

    # -- introspection (telemetry / debugging) ----------------------------

    def get_state_name(self) -> str:
        return self.state.name
