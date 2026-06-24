"""Autonomous controller for the WRO 2026 Future Engineers vehicle.

Shared brain: sim and robot both drive it via update(). Primary driver is gyro
heading-hold — hold a goal heading with a PID on the IMU, a corner is just
target_heading += 90°. ToF trims lateral position when aligned; pillars are a
bounded heading-offset weave; parking and the three-point turn are separate.

Hardware-port note: update() still reads sim-only state — pose (robot.x/y) for
the line-dedup and track.* map queries. See robot/main.py.
"""

import math
from enum import Enum, auto
from typing import Optional

from core.config import *


def _angle_diff(a, b):
    """Shortest signed angle from b to a, in range [-180, 180]."""
    d = (a - b) % 360
    if d > 180:
        d -= 360
    return d


class State(Enum):
    """Robot state machine states."""
    STARTING = auto()
    DRIVING = auto()           # gyro heading-hold; corners are setpoint bumps
    WALL_FOLLOWING = auto()    # retained alias for tooling; routes to DRIVING
    PILLAR_AVOIDANCE = auto()
    CORNER_TURN = auto()       # retained for tooling; no longer entered
    THREE_POINT_TURN = auto()
    PARKING_APPROACH = auto()
    PARKING_EXECUTE = auto()
    STOP_SECTION = auto()
    FINISHED = auto()
    STOPPED = auto()


class ThreePointPhase(Enum):
    """Phases of the three-point turn maneuver."""
    TURN_RIGHT_FORWARD = auto()
    TURN_LEFT_REVERSE = auto()
    TURN_RIGHT_FORWARD_2 = auto()
    COMPLETE = auto()


class ParkingPhase(Enum):
    """Phases of the parallel parking maneuver."""
    APPROACH = auto()
    ALIGN = auto()
    STEER_IN = auto()
    STRAIGHTEN = auto()
    FINAL_ADJUST = auto()
    COMPLETE = auto()


class Controller:
    """Autonomous driving controller for WRO Future Engineers."""

    def __init__(self, driving_direction: int = 1, start_in_parking: bool = True,
                 challenge_type: str = 'obstacle'):
        self.state = State.STARTING
        self.driving_direction = driving_direction  # 1=CW, -1=CCW
        self.start_in_parking = start_in_parking
        self._is_open = (challenge_type == 'open')

        # Lap counting
        self.sections_passed = 0
        self.current_section = -1
        self.laps_completed = 0
        self.sections_in_current_lap = 0

        # Line detection
        self.last_line_color = None
        self.line_cooldown = 0  # Frames to ignore after detection
        self._last_line_pos = None  # (x, y) of last line detection

        # Heading-hold state (primary driver)
        self.target_heading = 0.0   # goal cardinal; bumped ±90° per corner
        self.prev_heading_err = 0.0
        self.turns = 0              # corners taken (bookkeeping/debug)

        # PD control state (still used by parking approach)
        self.prev_wall_error = 0
        self.prev_pillar_error = 0

        # Pillar avoidance
        self.avoiding_pillar = None
        self.avoidance_timer = 0
        self._pillar_lane_heading = 0.0  # lane cardinal snapshotted on entry

        # Corner turn
        self._corner_entry_heading = None
        self._corner_cooldown = 0
        self._corner_line_detected = False
        self._corner_entry_distance = 0
        self._corner_radius = CORNER_TURN_RADIUS  # Dynamic per-turn radius
        self._corner_approach_ticks = 0  # Pre-corner centering countdown

        # Three-point turn
        self.three_point_phase = ThreePointPhase.TURN_RIGHT_FORWARD
        self.three_point_start_heading = 0
        self.three_point_target_heading = 0
        self.needs_direction_change = False
        self.direction_changed = False

        # Parking
        self.parking_phase = ParkingPhase.APPROACH
        self.parking_start_heading = 0
        self.parking_timer = 0

        # Stop section verification (open challenge)
        self.start_tof_front = None
        self.start_tof_rear = None
        self.stop_section_tolerance = 200  # mm

        # Timing
        self.elapsed_time = 0
        self.control_timer = 0

        # Track info (set during first update)
        self.track_width = TRACK_WIDTH_OBSTACLE

    def update(self, sensors, robot, track, dt: float):
        """Main control loop, called at CONTROL_HZ rate.

        Args:
            sensors: SensorReading from the robot.
            robot: Robot instance (or hardware adapter).
            track: Track instance (or world-model adapter).
            dt: Time step in seconds.
        """
        self._corner_line_detected = False
        self.elapsed_time += dt
        self.control_timer += dt
        self.track_width = track.get_local_track_width(robot.x, robot.y)

        # Check time limit
        if self.elapsed_time >= ROUND_TIME:
            robot.stop()
            self.state = State.STOPPED
            return

        # Detect section changes via color lines
        self._detect_lines(sensors, robot)

        # State machine
        if self.state == State.STARTING:
            self._handle_starting(sensors, robot, track)
        elif self.state in (State.DRIVING, State.WALL_FOLLOWING, State.CORNER_TURN):
            self._handle_driving(sensors, robot, track)
        elif self.state == State.PILLAR_AVOIDANCE:
            self._handle_pillar_avoidance(sensors, robot, track)
        elif self.state == State.THREE_POINT_TURN:
            self._handle_three_point_turn(sensors, robot, track)
        elif self.state == State.PARKING_APPROACH:
            self._handle_parking_approach(sensors, robot, track)
        elif self.state == State.PARKING_EXECUTE:
            self._handle_parking_execute(sensors, robot, track)
        elif self.state == State.STOP_SECTION:
            self._handle_stop_section(sensors, robot, track)
        elif self.state == State.FINISHED:
            robot.stop()
        elif self.state == State.STOPPED:
            robot.stop()

    def _detect_lines(self, sensors, robot):
        """Detect orange/blue lines for section counting.

        Uses both a frame cooldown and a minimum distance requirement
        to prevent double-counting when the robot oscillates near a line.
        """
        if self.line_cooldown > 0:
            self.line_cooldown -= 1
            return

        if sensors.color_detected in ('orange', 'blue'):
            if self._last_line_pos is not None:
                dx = robot.x - self._last_line_pos[0]
                dy = robot.y - self._last_line_pos[1]
                if math.sqrt(dx * dx + dy * dy) < 200:
                    return

            self._last_line_pos = (robot.x, robot.y)
            self.last_line_color = sensors.color_detected
            self.sections_passed += 1
            self.sections_in_current_lap += 1
            self.line_cooldown = 15

            # Corner entry detection: only trigger on the "entry" color.
            # CW: orange = entering corner, CCW: blue = entering corner.
            corner_entry_color = 'orange' if self.driving_direction == 1 else 'blue'
            if sensors.color_detected == corner_entry_color:
                self._corner_line_detected = True

            if self.sections_in_current_lap >= 8:
                self.laps_completed += 1
                self.sections_in_current_lap = 0

                if self.laps_completed >= 3:
                    if self._is_open:
                        self.state = State.STOP_SECTION  # roll to the finish section
                    elif self.state != State.THREE_POINT_TURN:
                        self.state = State.PARKING_APPROACH

    def _handle_starting(self, sensors, robot, track):
        """Initial start - latch the lane heading and begin driving."""
        self.start_tof_front = sensors.tof_front
        self.start_tof_rear = sensors.tof_rear
        self.target_heading = (round(sensors.imu_heading / 90) * 90) % 360
        self.prev_heading_err = 0.0
        robot.set_speed(SPEED_CRUISE)
        robot.set_steering(0)
        self.state = State.DRIVING

    def _maybe_take_corner(self, sensors):
        """A corner line bumps the heading setpoint by 90°, debounced."""
        if self._corner_cooldown > 0:
            self._corner_cooldown -= 1
        if self._corner_line_detected and self._corner_cooldown == 0:
            self._corner_line_detected = False
            turn_sign = 1 if self.driving_direction == 1 else -1
            self.target_heading = (self.target_heading + turn_sign * 90) % 360
            self.turns += 1
            # reset derivative so the setpoint jump isn't seen as a spike
            self.prev_heading_err = _angle_diff(self.target_heading, sensors.imu_heading)
            self._corner_cooldown = CORNER_DEBOUNCE_TICKS

    def _heading_hold_steer(self, sensors, trim=True):
        """PID the IMU onto target_heading. Returns (steering, turning)."""
        heading_err = _angle_diff(self.target_heading, sensors.imu_heading)
        turning = abs(heading_err) > TURN_HEADING_GATE
        d_err = heading_err - self.prev_heading_err
        self.prev_heading_err = heading_err
        steering = heading_err * KP_HEADING + d_err * KD_HEADING

        # gentle centring only when aligned and both side walls read real
        if (trim and not turning
                and sensors.tof_left < self.track_width
                and sensors.tof_right < self.track_width):
            steering += (sensors.tof_right - sensors.tof_left) * KP_TRIM

        return max(-WALL_FOLLOW_MAX_STEER, min(WALL_FOLLOW_MAX_STEER, steering)), turning

    def _handle_driving(self, sensors, robot, track):
        """Hold the lane heading, take corners as setpoint bumps, hand off to pillars."""
        self._maybe_take_corner(sensors)
        steering, turning = self._heading_hold_steer(sensors)

        # dodge signs only when aligned and clear of a just-taken corner
        if (not turning and self._corner_cooldown == 0
                and track.challenge_type == 'obstacle'):
            target = self._pillar_to_avoid(sensors)
            if target is not None:
                self.avoiding_pillar = target
                self.avoidance_timer = 0
                self._pillar_lane_heading = self.target_heading
                self.state = State.PILLAR_AVOIDANCE
                return

        if turning:
            speed = SPEED_OPEN_CORNER if self._is_open else SPEED_CORNER
        else:
            speed = SPEED_OPEN_CRUISE if self._is_open else SPEED_CRUISE
        robot.set_speed(speed)
        robot.set_steering(steering)

    def _pillar_to_avoid(self, sensors):
        """Nearest sign (camera is nearest-first) inside the trigger range and
        forward cone, or None. A passed sign sits outside the cone, so it won't
        re-trigger."""
        for p in sensors.pillars_visible:
            if p['distance'] < PILLAR_TRIGGER_DIST and abs(p['angle']) < PILLAR_TRIGGER_FOV:
                return p
        return None

    def _handle_pillar_avoidance(self, sensors, robot, track):
        """Thread past the tracked sign: red -> keep right (sign on our left),
        green -> keep left. Closed-loop on the sign's camera bearing toward a
        capped heading offset, plus wall repulsion; back to DRIVING once passed."""
        self.avoidance_timer += 1

        # corner line takes priority: bump the heading setpoint and hand back
        if self._corner_line_detected:
            self._corner_line_detected = False
            turn_sign = 1 if self.driving_direction == 1 else -1
            self.target_heading = (self.target_heading + turn_sign * 90) % 360
            self.turns += 1
            self.prev_heading_err = _angle_diff(self.target_heading, sensors.imu_heading)
            self._corner_cooldown = CORNER_DEBOUNCE_TICKS
            self.avoiding_pillar = None
            self.state = State.DRIVING
            return

        # re-find the tracked sign (by identity); if gone, take the next one ahead
        cur = None
        ref = self.avoiding_pillar.get('pillar_ref') if self.avoiding_pillar else None
        if ref is not None:
            cur = next((p for p in sensors.pillars_visible
                        if p.get('pillar_ref') is ref), None)
        if cur is None:
            cur = self._pillar_to_avoid(sensors)
            self.avoiding_pillar = cur
        if cur is None:
            self.state = State.DRIVING
            return

        bearing = cur['angle']            # +right / -left
        distance = max(1.0, cur['distance'])

        # done once the sign is beside/behind us, or on timeout
        if abs(bearing) > PILLAR_PASSED_BEARING or self.avoidance_timer > PILLAR_AVOID_TIMEOUT:
            self.state = State.DRIVING
            self.avoiding_pillar = None
            return

        # one-sided hinge: how far the sign still needs to move to the mandated
        # side for PILLAR_CLEARANCE — never chase one already clear
        clear_angle = math.degrees(math.atan2(PILLAR_CLEARANCE, distance))
        if cur['color'] == 'red':
            hinge = max(0.0, bearing + clear_angle)
            offset = min(hinge, PILLAR_MAX_HEADING_OFFSET)   # angle right of lane
        else:
            hinge = max(0.0, clear_angle - bearing)
            offset = -min(hinge, PILLAR_MAX_HEADING_OFFSET)  # angle left of lane

        desired = self._pillar_lane_heading + offset
        steering = _angle_diff(desired, sensors.imu_heading) * KP_PILLAR_HEADING

        # wall repulsion: never thread ourselves into a wall
        if sensors.tof_left < PILLAR_WALL_MARGIN:
            steering += (PILLAR_WALL_MARGIN - sensors.tof_left) * KP_PILLAR_WALL
        if sensors.tof_right < PILLAR_WALL_MARGIN:
            steering -= (PILLAR_WALL_MARGIN - sensors.tof_right) * KP_PILLAR_WALL

        steering = max(-WALL_FOLLOW_MAX_STEER, min(WALL_FOLLOW_MAX_STEER, steering))
        robot.set_speed(SPEED_PILLAR)
        robot.set_steering(steering)

    def _handle_three_point_turn(self, sensors, robot, track):
        """Execute a three-point turn to reverse direction.

        Phase 1: Forward turn in current corner direction (~80°)
        Phase 2: Reverse turn in opposite direction (~80°)
        Phase 3: Forward turn to complete ~180° total
        """
        heading = sensors.imu_heading
        turn_sign = 1 if self.driving_direction == 1 else -1

        if self.three_point_phase == ThreePointPhase.TURN_RIGHT_FORWARD:
            robot.set_speed(SPEED_THREE_POINT)
            robot.set_steering(turn_sign * MAX_STEERING_ANGLE)

            heading_diff = abs(_angle_diff(heading, self.three_point_start_heading))
            if heading_diff > 80:
                self.three_point_phase = ThreePointPhase.TURN_LEFT_REVERSE
                self.three_point_target_heading = heading

        elif self.three_point_phase == ThreePointPhase.TURN_LEFT_REVERSE:
            robot.set_speed(-SPEED_THREE_POINT)
            robot.set_steering(-turn_sign * MAX_STEERING_ANGLE)

            heading_diff = abs(_angle_diff(heading, self.three_point_target_heading))
            if heading_diff > 80 or sensors.tof_rear < 150:
                self.three_point_phase = ThreePointPhase.TURN_RIGHT_FORWARD_2
                self.three_point_target_heading = heading

        elif self.three_point_phase == ThreePointPhase.TURN_RIGHT_FORWARD_2:
            robot.set_speed(SPEED_THREE_POINT)
            robot.set_steering(turn_sign * MAX_STEERING_ANGLE * 0.5)

            total_turn = abs(_angle_diff(heading, self.three_point_start_heading))
            if total_turn > 160:
                self.three_point_phase = ThreePointPhase.COMPLETE
                self.driving_direction *= -1
                self.needs_direction_change = False
                self.direction_changed = True
                # Latch the reversed lane heading for the heading-hold driver.
                self.target_heading = (round(sensors.imu_heading / 90) * 90) % 360
                self.prev_heading_err = 0.0
                self.state = State.DRIVING
                robot.set_steering(0)

    def _handle_parking_approach(self, sensors, robot, track):
        """Approach the parking lot after completing 3 laps."""
        if track.parking_lot is None:
            self.state = State.FINISHED
            return

        # Slow down and look for parking lot
        robot.set_speed(SPEED_PARKING)

        error = sensors.tof_right - sensors.tof_left
        derivative = error - self.prev_wall_error
        self.prev_wall_error = error

        steering = error * KP_WALL * 0.5 + derivative * KD_WALL * 0.5
        robot.set_steering(steering)

        # Check if we're in the parking section.
        # Use ToF sensors to detect the gap created by parking markers.
        pl = track.parking_lot

        dist_to_parking = math.sqrt((robot.x - (pl.x + pl.width/2))**2 +
                                     (robot.y - (pl.y + pl.length/2))**2)

        if dist_to_parking < 400:
            self.parking_start_heading = sensors.imu_heading
            self.state = State.PARKING_EXECUTE
            self.parking_phase = ParkingPhase.STEER_IN

    def _handle_parking_execute(self, sensors, robot, track):
        """Execute parallel parking maneuver."""
        if self.parking_phase == ParkingPhase.STEER_IN:
            # Steer toward the parking spot (toward outer wall)
            robot.set_speed(SPEED_PARKING * 0.5)

            # Determine which direction to steer based on parking position
            pl = track.parking_lot
            if pl.section_idx == 0:  # Top - park against top wall
                robot.set_steering(-15 * self.driving_direction)
            elif pl.section_idx == 1:  # Right - park against right wall
                robot.set_steering(-15 * self.driving_direction)
            elif pl.section_idx == 2:  # Bottom
                robot.set_steering(15 * self.driving_direction)
            else:  # Left
                robot.set_steering(15 * self.driving_direction)

            # Check if we're close to the outer wall
            if sensors.tof_front < 200 or sensors.tof_right < 100 or sensors.tof_left < 100:
                self.parking_phase = ParkingPhase.STRAIGHTEN

        elif self.parking_phase == ParkingPhase.STRAIGHTEN:
            heading_diff = _angle_diff(sensors.imu_heading, self.parking_start_heading)
            robot.set_speed(SPEED_PARKING * 0.3)

            if abs(heading_diff) > 5:
                robot.set_steering(-heading_diff * 0.3)
            else:
                robot.set_steering(0)
                self.parking_phase = ParkingPhase.FINAL_ADJUST

        elif self.parking_phase == ParkingPhase.FINAL_ADJUST:
            # Small adjustments
            self.parking_timer += 1
            if self.parking_timer > 30:  # ~1 second
                robot.stop()
                self.parking_phase = ParkingPhase.COMPLETE
                self.state = State.FINISHED

    def _handle_stop_section(self, sensors, robot, track):
        """Heading-hold at reduced speed (taking any remaining corner), then stop
        once the ToF readings match the starting section."""
        self._maybe_take_corner(sensors)
        steering, _ = self._heading_hold_steer(sensors)
        robot.set_speed(SPEED_PARKING)
        robot.set_steering(steering)

        if self.start_tof_front is not None and self.start_tof_rear is not None:
            front_match = abs(sensors.tof_front - self.start_tof_front) < self.stop_section_tolerance
            rear_match = abs(sensors.tof_rear - self.start_tof_rear) < self.stop_section_tolerance
            if front_match and rear_match:
                robot.stop()
                self.state = State.FINISHED

    def get_state_name(self) -> str:
        """Get human-readable state name."""
        return self.state.name

    def get_score(self, track, robot_x=None, robot_y=None, robot_angle=None) -> dict:
        """Calculate current score based on rules."""
        score = {
            'sections': min(self.sections_passed, MAX_SECTION_POINTS),
            'laps': min(self.laps_completed, MAX_LAP_POINTS),
            'finish_section': 0,
            'obstacle_bonus': 0,
            'parking_start': 0,
            'parking_result': 0,
            'total': 0,
        }

        if self.laps_completed >= 3 and self.state in (State.FINISHED, State.STOPPED):
            score['finish_section'] = POINTS_FINISH_SECTION

        if track.challenge_type == 'obstacle':
            signs_moved = any(p.is_outside_circle() for p in track.pillars)

            if self.laps_completed >= 3:
                if signs_moved:
                    score['obstacle_bonus'] = POINTS_3LAPS_MOVED
                else:
                    score['obstacle_bonus'] = POINTS_3LAPS_NOT_MOVED
            elif self.laps_completed >= 1:
                if signs_moved:
                    score['obstacle_bonus'] = POINTS_OBSTACLE_MOVED
                else:
                    score['obstacle_bonus'] = POINTS_OBSTACLE_NOT_MOVED

            if self.start_in_parking and self.laps_completed >= 1:
                score['parking_start'] = POINTS_START_IN_PARKING

            if self.state == State.FINISHED and track.parking_lot and robot_x is not None:
                parking_status = track.check_parking(
                    robot_x, robot_y, robot_angle, ROBOT_LENGTH, ROBOT_WIDTH
                )
                if parking_status == 'fully_parked':
                    score['parking_result'] = POINTS_PARKED_FULLY
                elif parking_status == 'partly_parked':
                    score['parking_result'] = POINTS_PARKED_PARTLY

        score['total'] = (score['sections'] + score['laps'] +
                         score['finish_section'] + score['obstacle_bonus'] +
                         score['parking_start'] + score['parking_result'])

        return score
