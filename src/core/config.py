"""Shared configuration for the WRO 2026 Future Engineers vehicle.

Physics, control, sensor and scoring constants used by BOTH the simulation
(``src/sim/``) and the real robot (``src/robot/``). All measurements are in
mm, angles in degrees, and time in seconds unless noted otherwise.

Based on the official WRO 2026 Future Engineers Self-Driving Cars rules.
"""

import math


# Track dimensions (rules section 13)
MAT_SIZE = 3200            # Full mat size (mm)
TRACK_INNER_SIZE = 3000    # Internal track size (mm)
WALL_HEIGHT = 100          # Wall height (mm) - for reference
WALL_OFFSET = (MAT_SIZE - TRACK_INNER_SIZE) / 2  # 100mm border

TRACK_WIDTH_OPEN_NARROW = 600    # Open challenge narrow
TRACK_WIDTH_OPEN_WIDE = 1000     # Open challenge wide
TRACK_WIDTH_OBSTACLE = 1000      # Obstacle challenge always 1000mm

LINE_THICKNESS = 20        # Orange/blue line thickness (mm)

# The track has 8 sections: 4 straight + 4 corners.
NUM_SECTIONS = 8
NUM_STRAIGHT_SECTIONS = 4
NUM_CORNER_SECTIONS = 4


# Traffic signs / pillars (rules sections 8, 9, 13)
PILLAR_DIAMETER = 50       # Pillar base ~50mm (fits in seat)
PILLAR_HEIGHT = 100        # Approximate pillar height
SIGN_SEAT_SIZE = 50        # Traffic sign seat 50x50mm
EVALUATION_CIRCLE_DIAMETER = 85  # Circle around seat for movement check

ZONES_PER_SECTION = 6      # 6 zones per straight section
SEATS_PER_SECTION = 6      # 4 T-intersections + 2 X-intersections


# Parking lot (rules and appendix A.6)
PARKING_WIDTH = 200         # Always 20cm
PARKING_LENGTH_FACTOR = 1.5 # 1.5 * robot length = 312mm with current dimensions
PARKING_MARKER_SIZE = (200, 20, 100)  # Magenta markers: 20cm x 2cm x 10cm
PARKING_PARALLEL_TOLERANCE = 20  # 2cm = 20mm wheel distance difference

# Parallel-park maneuver (gyro heading + encoder distance). The
# bay is on park_side; we pull in at a fixed heading offset, straighten back to
# the lane heading, then seat. Fallbacks stop cleanly in-section if it can't lock.
# The bay's perpendicular offset (what the camera/markers give) drives a heading
# crab that translates us into the bay, collapsing to parallel as the offset -> 0.
PARK_CRAB_ANGLE = 34        # deg — max heading crab toward the bay
KP_CRAB = 0.18              # crab deg per mm of perpendicular bay offset
PARK_LAT_TOL = 42           # mm — perpendicular offset that counts as seated
PARK_ALIGN_GATE = 8         # deg — |heading - lane| to count as parallel at stop
PARK_SEAT_LEAD = 20         # mm — enter the seat this far before bay centre
PARK_SPEED = 0.28           # fraction of max speed through the approach
PARK_SEAT_SPEED = 0.08      # fraction of max speed during the final seat
PARK_APPROACH_MAX_DIST = 2600  # mm — approach travel before giving up -> stop in section
PARK_OVERRUN = 85           # mm — bay-centre overshoot in seat before giving up


# Robot specifications (gen-2 chassis, from the CAD assembly — see
# models/README.md; v1 was 140x88mm with 30.4mm LEGO wheels)
ROBOT_LENGTH = 208         # mm over the wheels
ROBOT_WIDTH = 148          # mm over the wheels (incl. tires)
ROBOT_HEIGHT = 100         # mm — same as the track wall height
ROBOT_WEIGHT = None        # grams — gen-2 not yet weighed (rules limit 1500 g)
ROBOT_MAX_LENGTH = 300     # Max allowed by rules
ROBOT_MAX_WIDTH = 200      # Max allowed by rules
ROBOT_MAX_HEIGHT = 300     # Max allowed by rules
ROBOT_MAX_WEIGHT = 1500    # 1.5kg in grams

# Drive system (gen-2 rolling chassis)
WHEELBASE = 152            # Distance between front and rear axles (mm)
TRACK_WHEEL = 120          # Distance between left and right wheel centres (mm)
WHEEL_DIAMETER = 55        # mm — rubber wheels (was 30.4mm LEGO on v1)
MAX_STEERING_ANGLE = 35  # degrees (measured throw of reassembled servo linkage)

# Motor: 12V 300RPM gearmotor, direct rear-axle drive. 300 is the RATED output
# RPM at 12V; on the 11.1V 3S pack, loaded, it will run slower. Once the wheel
# RPM is bench-measured, record it as Motor.OUTPUT_RPM in
# src/robot/hardware_config.py and update this to match.
MOTOR_RPM = 300
WHEEL_CIRCUMFERENCE = math.pi * WHEEL_DIAMETER
MAX_SPEED = (MOTOR_RPM * WHEEL_CIRCUMFERENCE) / 60  # mm/s ≈ 863.9 mm/s
ACCELERATION = 500         # mm/s² (approximate)
DECELERATION = 800         # mm/s² (approximate, braking)

# Speed profiles, as a fraction of max speed. Cruise at 0.80 balances speed
# and stability; corner speed 0.55 keeps the bicycle-model turn controlled.
SPEED_CRUISE = 0.80
SPEED_CORNER = 0.55
SPEED_PILLAR = 0.65
SPEED_PARKING = 0.35
SPEED_THREE_POINT = 0.30

# Open challenge has no pillars to dodge, so it can push harder.
# Gen-2 retune (2026-07-17): fractions rebased to the 864 mm/s gen-2 top speed.
# 0.35 = ~302 mm/s cruise — 2026-07 sim sweep: 40/40 perfect scored runs with
# ~92s 3-lap time (vs 180s budget); 0.40 stayed 40/40 but tripled narrow-track
# wall contacts, so 0.35 is the validated ceiling for now.
SPEED_OPEN_CRUISE = 0.35
SPEED_OPEN_CORNER = 0.22

# Wall-following PD steering limit, separate from the robot's physical max.
WALL_FOLLOW_MAX_STEER = 30  # degrees — prevents PD oscillation with high max steering

# Corner turn geometry (bicycle model: R = wheelbase / tan(delta)).
# R = track_width/2 = 500mm keeps the robot centered through the corner arc.
CORNER_TURN_RADIUS = 500   # mm — turning radius matched to track geometry
CORNER_STEERING_ANGLE = math.degrees(math.atan(WHEELBASE / CORNER_TURN_RADIUS))  # ~16.9°
CORNER_ARC_LENGTH = CORNER_TURN_RADIUS * math.pi / 2  # ~785mm for a 90° arc
CORNER_MIN_EXIT_ANGLE = 87   # degrees — minimum heading change to exit corner
CORNER_MAX_EXIT_ANGLE = 100  # degrees — safety cap, force exit above this


# Sensor simulation
# ToF sensors (VL53L1X × 4, all mounted at 90° perpendicular to their face).
TOF_MAX_RANGE = 4000       # 4m max range
TOF_MIN_RANGE = 4          # 4mm min range
TOF_ACCURACY = 1           # ±1mm
TOF_BEAM_ANGLE = 4         # ~4 degree cone (approximate)
TOF_COUNT = 4              # front, left, right, rear
# VL53L1X occasionally drops a single frame to a spurious value (a no-target
# read jumps to TOF_MAX_RANGE, e.g. 233,233,4000,234). De-noising = the outlier
# gate below + an optional median (core/tof_filter.py). Sim regression sweep
# (test_tof_noise.py, paired tracks, 10 seeds): gate-only beats every median
# combination in every config — any median window adds lag that clips walls in
# tight 600mm corridors (window 3 alone: 9/10 -> 0/10 valid on narrow when
# stacked with the gate). So the median is DISABLED (window 1 = identity); the
# gate does the spike rejection and CORNER_PERSIST_TICKS debounces the corner
# trigger lag-free.
TOF_MEDIAN_WINDOW = 1
# Outlier gate: a sample further than TOF_MAX_JUMP from the current output is
# physically impossible frame-to-frame (robot moves ~25mm/tick) and is rejected —
# UNLESS it agrees with the immediately previous raw sample (a real level shift:
# a wall edge passing legitimately jumps 250 -> 4000 in one frame), or the gate
# has already rejected 2 in a row (escape valve for fast real ramps during
# full-lock turns, where consecutive frames confirm neither the output nor each
# other; without it the gate holds stale pre-turn values through the whole turn).
TOF_MAX_JUMP = 150
TOF_ANGLES = {             # pointing angle relative to robot heading (degrees)
    'front': 0,
    'left': -90,
    'right': 90,
    'rear': 180,
}

# IMU (MPU6050 / 10-DoF)
IMU_GYRO_DRIFT = 1.0       # degrees per minute drift
IMU_NOISE = 0.1            # degrees noise per reading

# Color sensor (TCS34725, mounted on underside)
COLOR_SENSOR_RANGE = 30    # mm from ground (effective detection range)

# Camera (OV5647 160° wide angle, mounted front-facing at 90° — no tilt)
CAMERA_FOV = 160           # degrees
CAMERA_DETECTION_RANGE = 1500  # mm max pillar detection distance
CAMERA_MIN_DETECTION = 100 # mm min detection distance


# Gyro heading-hold driving (primary controller). Hold a goal heading; a corner
# is target += 90°. ToF only trims laterally when aligned.
KP_HEADING = 1.0           # steering deg per deg of heading error. Gen-2 retune:
                           # the 152mm wheelbase halves yaw rate per steer deg, so
                           # gains roughly double vs v1 (0.45/0.25). Sweep at 0.35
                           # cruise: 0.9-1.2 all give 112/112 on test_pd_tuning
                           # phases 1-4; 1.0 = mid-plateau.
KD_HEADING = 0.5           # damping on heading error
KP_TRIM = 0.02             # centring: steering deg per mm of (tof_r - tof_l)
TURN_HEADING_GATE = 20     # deg — above this we're mid-corner: drop trim, slow
CORNER_DEBOUNCE_TICKS = 20 # ticks to ignore further corner lines after a turn

# PD control parameters (from team design doc)
KP_WALL = 0.05
KD_WALL = 0.02
KP_PILLAR = 0.08
KD_PILLAR = 0.03
KP_TURN = 0.06
KD_TURN = 0.02

# ToF-only wall-following open-challenge driver (core.wall_follow_controller).
# The IMU-free fallback: no heading, no colour, no pose — just the four ToF
# distances. Straights = PD centring between the side walls; corners = the front
# wall closing, steer toward the open side until it reopens. Tune KP_WALL/KD_WALL
# in the sim first (test_pd_tuning.py), then on the bench.
#
# A side ToF reading above SIDE_WALL_VALID counts as "no wall there" (a corner
# opening), so centring falls back to single-wall following at WALL_SETPOINT.
SIDE_WALL_VALID = 900          # mm — side reading above this = opening, not a wall
# Gyro-free heading estimation (core/tof_heading.py): heading error is recovered
# from the least-squares slope of the side-ToF distances over this many ticks
# (~0.33s at 30Hz — long enough that the ~1mm sensor noise averages below 1 deg
# at cruise, short enough to act as a damping term).
HEADING_WINDOW_TICKS = 10
KP_HEADING_WF = 1.2            # deg steering per deg of estimated heading error.
                               # Sim sweep (test_tof_noise.py): 0.6-1.2 = 10/10 valid,
                               # 0 collisions in EVERY config incl. narrow+noisy
                               # (0/10 without the heading term); >=2.0 over-damps.
                               # 1.2 = mid-plateau, tolerant of the ±30% speed-estimate
                               # error on the real robot (no encoder).

# Vision heading (core/vision_heading.py) — wall-base slope from the wide camera.
VISION_DARK_THRESHOLD = 90     # 8-bit grayscale below this = wall band (dark on
                               # light mat; verify against real frames per venue)
VISION_CHROMA_LIMIT = 25       # max-min channel spread for a "neutral" (wall) pixel;
                               # rejects the navy lane line, which stays blue-tinted
                               # even when dim enough to pass the darkness test
VISION_MIN_COLUMNS = 60        # min detected wall-base columns for a valid fit
# Corner-line detection (core/vision_lines.py) — camera replaces the colour sensor.
LINE_CHROMA_MARGIN = 30        # min R-B (orange) / B-R (blue) channel separation
LINE_MIN_PIXELS = 250          # min matching pixels in the lower-frame ROI to call
                               # a line (at 640x480 a line ~1m ahead spans far more;
                               # this floor rejects speckle)
VISION_FUSE_WEIGHT = 0.3       # weight of the camera heading when BOTH gyro-free
                               # heading sources are live (ToF-rate estimate keeps
                               # 0.7 — it's the precise one); vision alone carries
                               # the ticks where the ToF estimator is blind
VISION_HEADING_GAIN = 400.0    # deg heading per unit slope (rows/col) — PLACEHOLDER
                               # until bench calibration (tape-protractor yaw sweep);
                               # sign/zero are trustworthy, scale is not.
WALL_SETPOINT = 250            # mm — target distance when following a single wall
CORNER_TRIGGER_FRONT = 600     # mm — front wall this close = corner ahead, start turn.
                               # Gen-2 retune: the 152mm wheelbase turns a 217mm
                               # radius at full lock (vs 108mm on v1) and cruise is
                               # ~300mm/s, so the turn must start earlier than the
                               # v1-tuned 450. 10-seed scored sweep at 0.35 cruise:
                               # 600 = 80/80 perfect runs and the fewest wall
                               # contacts (650 and 550 both dropped noisy seeds).
CORNER_PERSIST_TICKS = 3       # front must stay below the trigger this many consecutive
                               # ticks before a corner fires — rejects single-frame ToF
                               # dropout spikes (a stray near read) that would otherwise
                               # false-trigger a turn. Sim-proven: without it, injected
                               # dropouts push a 12-turn round to ~21 turns (0/5 valid);
                               # with it, ~12 turns and 4-5/5 valid. Lag-free, unlike
                               # widening the median filter. See core/tof_filter.py.
CORNER_OPEN_SIDE = 1800        # mm — a REAL corner trigger also sees the corner
                               # mouth: one side reads clear across the track
                               # (>=2400 in every instrumented trigger, all widths,
                               # both directions). A phantom trigger — the front ToF
                               # catching a side wall diagonally after a heading
                               # excursion — has both walls present (max side
                               # <=1360 measured). Phantoms double-counted corners
                               # (premature finish) and caused the noisy-run
                               # collisions.
CORNER_CLEAR_FRONT = 700       # mm — front reopened past this = turn complete
TURN_STEER = MAX_STEERING_ANGLE  # deg — full lock through a corner (tighter arc so
                                 # a 90° corner completes before the front reopens)
# Turn progress is dead-reckoned kinematically (no gyro, no encoder): the arc
# swept at commanded steer/speed is omega = v * tan(steer) / wheelbase. The exit
# geometry check only ARMs inside [TURN_ARC_MIN, TURN_ARC_MAX]; at TURN_ARC_MAX
# the turn force-exits regardless (geometry alone under-constrains exit heading
# in wide corridors: ground-truth traces showed 150-330 deg spins).
TURN_ARC_MIN = 75              # deg — earliest the exit geometry may end a turn
TURN_ARC_MAX = 95              # deg — force exit; DRIVING recentres from here.
                               # The dead-reckoned arc overestimates true rotation
                               # (servo slew lag), so 115 produced real ~25 deg
                               # overshoots whose recovery lost the race to the
                               # inner wall in 600mm corridors (wall-grind stalls).
                               # Sweep: 95 -> 39/40 perfect noisy runs vs 31/40 at 115
TURN_MIN_TICKS = 24            # legacy tick floor (kept for the sim viewer HUD)
SILENT_CORNER_ARC = 60         # deg — dead-reckoned steering accumulated in DRIVING
                               # (leaky, ~7s time constant) beyond this in the turn
                               # direction = the wall-follower rounded a corner
                               # WITHOUT the front trigger firing (hold-dist mode
                               # tracks the outer wall around the bend). Count it,
                               # or the finish arms a corner late and the stop
                               # signature matches in the wrong straight.
SILENT_CORNER_DECAY = 0.995    # per-tick leak on the DRIVING arc accumulator, so
                               # symmetric straight-line PD corrections wash out
SIDE_SUM_REALIGNED = 1300      # mm — left+right at most this = back in a corridor
                               # (600-1000mm wide + sensor offsets); the mid-corner
                               # diagonal glimpse reads the open mouth (>1500) on one
                               # side. Replaces the old per-side < SIDE_WALL_VALID
                               # test, which a CENTRED robot in a 1000mm corridor
                               # could never satisfy -> endless spin past 90 deg
TURN_EXIT_HOLD = 6             # consecutive "realigned" ticks needed to end a turn.
                               # Gen-2 retune: 6 (was 4) — at ~190mm/s corner speed
                               # the wider 148mm body clips the inner wall if the
                               # turn ends on a 4-tick glimpse; sweep: 4->6 cut
                               # scored-run wall contacts ~5x (10.2 -> 2.2 sum/20)
TURN_DEBOUNCE_TICKS = 90       # ticks (~3s at 30Hz) to ignore new corners after a
                               # turn, so one physical corner isn't counted twice.
                               # 3s is still < half the shortest corner-to-corner
                               # time (~9s observed; >5s worst case), and covers the
                               # post-force-exit window where the heading is still
                               # settling and a side ToF can see back through the
                               # corner mouth just exited (phantom re-trigger seen
                               # 1.7s after exit with the old 1.5s debounce)
TURNS_TO_FINISH = 12           # 4 corners × 3 laps — run ends after this many turns
FINISH_TOF_TOLERANCE = 200     # mm — front/rear match to the start signature = stop

# Traffic-sign threading (obstacle). Red -> keep RIGHT (sign on our left),
# green -> keep LEFT. Thread by a capped heading offset off the lane heading.
PILLAR_TRIGGER_DIST = 650   # mm — start threading once a sign is this close
PILLAR_TRIGGER_FOV = 60     # deg — only act on signs within this forward cone
PILLAR_PASSED_BEARING = 80  # deg — sign is beside/behind us; back to DRIVING
PILLAR_AVOID_TIMEOUT = 120  # ticks (~4s at 30Hz) — safety exit from avoidance
PILLAR_CLEARANCE = 130      # mm — target gap between our centreline and the sign
PILLAR_MAX_HEADING_OFFSET = 28  # deg — most we'll angle off the lane to dodge
KP_PILLAR_HEADING = 0.8     # steering deg per deg of heading error
PILLAR_WALL_MARGIN = 150    # mm — repel from a wall closer than this
KP_PILLAR_WALL = 0.2        # steering deg per mm of wall-margin intrusion


# Scoring (rules section 10)
POINTS_PER_SECTION = 1       # 1.1: per section in correct direction
MAX_SECTION_POINTS = 24      # 24 sections total (3 laps × 8)
POINTS_PER_LAP = 1           # 1.2: per completed lap
MAX_LAP_POINTS = 3           # 3 laps
POINTS_FINISH_SECTION = 3    # 1.3: stopped in finish section
POINTS_OBSTACLE_MOVED = 2    # 1.4: signs moved, < 3 laps (need 1 lap)
POINTS_OBSTACLE_NOT_MOVED = 4  # 1.5: signs not moved, < 3 laps
POINTS_3LAPS_MOVED = 8       # 1.6: 3 laps, signs moved
POINTS_3LAPS_NOT_MOVED = 10  # 1.7: 3 laps, signs not moved
POINTS_START_IN_PARKING = 7  # 1.8.1: started in parking lot + 1 lap
POINTS_PARKED_FULLY = 15     # 1.8.2: fully parked and parallel
POINTS_PARKED_PARTLY = 7     # 1.8.3: partly parked or not parallel


# Simulation parameters
SIM_FPS = 60               # Simulation frame rate
CONTROL_HZ = 30            # Robot control loop rate (matches real robot)
ROUND_TIME = 180           # 3 minutes in seconds

# Square-drive IMU test (robot/square_drive.py) — 4x (drive side, turn 90).
SQUARE_SIDE_MM = 1000          # target side length
SQUARE_DRIVE_SPEED = 0.40      # fraction of max speed on the straights
SQUARE_TURN_SPEED = 0.55       # fraction through the full-lock corner
SQUARE_KP_HOLD = 2.0           # deg steering per deg heading error (straight hold)
SQUARE_SIDE_TIMEOUT_S = 12.0   # per-side safety cap
SQUARE_DIST_CAP_MM = 1350      # hard stop by TIME-distance even in accel mode, so a
                               # bad accel estimate can not drive the robot into a wall
PIXELS_PER_MM = 0.25       # Scale: 1mm = 0.25 pixels (800px for 3200mm)
WINDOW_WIDTH = int(MAT_SIZE * PIXELS_PER_MM) + 200  # Extra for UI panel
WINDOW_HEIGHT = int(MAT_SIZE * PIXELS_PER_MM)

# Colors (RGB), used only by the simulation viewer.
COLOR_WHITE = (255, 255, 255)
COLOR_BLACK = (0, 0, 0)
COLOR_RED = (220, 40, 40)
COLOR_GREEN = (40, 180, 40)
COLOR_ORANGE = (255, 140, 0)       # CMYK (0,60,100,0) ≈ RGB
COLOR_BLUE = (0, 70, 180)          # CMYK (100,80,0,0) ≈ RGB
COLOR_MAGENTA = (200, 0, 200)
COLOR_GRAY = (180, 180, 180)
COLOR_DARK_GRAY = (100, 100, 100)
COLOR_LIGHT_GRAY = (220, 220, 220)
COLOR_YELLOW = (255, 255, 0)
COLOR_ROBOT = (50, 120, 220)
COLOR_ROBOT_FRONT = (255, 200, 0)
COLOR_SENSOR_RAY = (0, 255, 255, 80)
