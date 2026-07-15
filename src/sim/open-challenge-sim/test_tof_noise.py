"""Headless regression harness for WallFollowController vs ToF dropout noise.

Runs the ToF-only wall-follower through a full OPEN round in the sim, across a
spread of track configs and RNG seeds, under several ToF conditions:

  clean           - sim default (+-1mm gaussian)               baseline
  clean+gate+med3 - no injected noise, gate + median w=3       isolates FILTER LAG cost
  noisy           - + 4000-spike dropouts, NO filter           does noise regress it?
  noisy+med3only  - + dropouts, median w=3, gate disabled      old approach
  noisy+gate1     - + dropouts, outlier gate only (w=1)        driver default: best
  noisy+gate+med3 - + dropouts, gate + median w=3              lag stack: worst on narrow

Injected noise reproduces the bench pattern (233,233,4000,234): with probability
P_SPIKE per channel per control tick, the reading is replaced by the no-target
sentinel (TOF_MAX_RANGE, i.e. 4000), plus a small chance of a stray near read.

A run is a VALID full round iff it stops in FINISHED with 12<=turns<=14 and
time>=90s (a real 3-lap open round is ~110-165s; <90s or turns>14 == false finish).
"""

import math
import os
import random
import sys

SRC = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SIMDIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SRC)
sys.path.insert(0, SIMDIR)   # must precede SRC so local robot.py/track.py win

from core.config import SIM_FPS, CONTROL_HZ, ROUND_TIME, TOF_MAX_RANGE, TOF_MIN_RANGE
from core.wall_follow_controller import WallFollowController, WFState
from core.tof_filter import TofMedianFilter
from track import Track
from robot import Robot

POSITIONS = ("front", "left", "right", "rear")


def inject_spikes(reading, rng, p_spike):
    for pos in POSITIONS:
        if rng.random() < p_spike:
            val = TOF_MAX_RANGE if rng.random() < 0.8 else rng.uniform(TOF_MIN_RANGE, 300)
            setattr(reading, f"tof_{pos}", val)


def start_pose(track):
    i = track.starting_section_idx
    s = track.straight_sections[i]
    cx, cy = (s.x1 + s.x2) / 2, (s.y1 + s.y2) / 2
    base = {0: 0, 1: 90, 2: 180, 3: 270}[i]
    angle = base if track.driving_direction == 1 else (base + 180) % 360
    return cx, cy, angle


def run(section_widths, direction, start_section, seed, noise, p_spike, window,
        max_jump='default'):
    rng = random.Random(seed)
    # Track() draws its randomized geometry from the GLOBAL rng — seed it so the
    # same seed produces the same track in every mode (paired comparison).
    random.seed(seed * 7919 + 13)
    track = Track(challenge_type='open', section_widths=section_widths,
                  driving_direction=direction, starting_section_idx=start_section)
    cx, cy, angle = start_pose(track)
    robot = Robot(cx, cy, angle)
    controller = WallFollowController()
    if window:
        kwargs = {} if max_jump == 'default' else {'max_jump': max_jump}
        filt = TofMedianFilter(POSITIONS, window=window, **kwargs)
    else:
        filt = None

    physics_dt, control_dt = 1.0 / SIM_FPS, 1.0 / CONTROL_HZ
    acc = elapsed = 0.0
    collisions = 0
    in_collision = False

    while elapsed < ROUND_TIME:
        acc += physics_dt
        while acc >= control_dt:
            acc -= control_dt
            reading = robot.get_sensors(track)
            if noise:
                inject_spikes(reading, rng, p_spike)
            if filt is not None:
                for pos in POSITIONS:
                    setattr(reading, f"tof_{pos}",
                            filt.update(pos, getattr(reading, f"tof_{pos}")))
            controller.update(reading, robot, track, control_dt)
        robot.update(physics_dt)
        if robot.check_wall_collision(track):
            if not in_collision:
                collisions += 1
                in_collision = True
            ocx = (track.outer_rect[0] + track.outer_rect[2]) / 2
            ocy = (track.outer_rect[1] + track.outer_rect[3]) / 2
            dx, dy = ocx - robot.x, ocy - robot.y
            d = max(1, math.hypot(dx, dy))
            robot.x += dx / d * 20
            robot.y += dy / d * 20
        else:
            in_collision = False
        elapsed += physics_dt
        if controller.state == WFState.STOPPED:
            break

    valid = (controller.state == WFState.STOPPED and 12 <= controller.turns <= 14
             and elapsed >= 90 and collisions == 0)
    return {'turns': controller.turns, 'collisions': collisions,
            'time': elapsed, 'valid': valid}


CONFIGS = [
    ("default-CW",  None,                     1, 0),
    ("default-CCW", None,                    -1, 0),
    ("narrow-CW",   [600, 600, 600, 600],     1, 0),
    ("wide-CW",     [1000, 1000, 1000, 1000], 1, 2),
]
MODES = [  # (label, noise, window, max_jump)
    ("clean",           False, None, None),
    ("clean+gate+med3", False, 3,    'default'),
    ("noisy",           True,  None, None),
    ("noisy+med3only",  True,  3,    None),        # old filter, gate disabled
    ("noisy+gate1",     True,  1,    'default'),   # gate alone, no median
    ("noisy+gate+med3", True,  3,    'default'),   # repo default (driver behavior)
]
SEEDS = list(range(1, 11))
P_SPIKE = 0.05


def main():
    print(f"P_SPIKE={P_SPIKE} per channel/tick | {len(SEEDS)} seeds | "
          f"validity: STOPPED, 12<=turns<=14, time>=90s, 0 collisions\n")
    hdr = f"{'config':<12} {'mode':<16} {'valid':<8} {'avg_turns':<10} {'avg_coll':<9} {'avg_time':<8}"
    print(hdr); print("-" * len(hdr))
    for name, widths, direction, start in CONFIGS:
        for label, noise, window, max_jump in MODES:
            rs = [run(widths, direction, start, s, noise, P_SPIKE, window, max_jump)
                  for s in SEEDS]
            valid = sum(r['valid'] for r in rs)
            at = sum(r['turns'] for r in rs) / len(rs)
            ac = sum(r['collisions'] for r in rs) / len(rs)
            atime = sum(r['time'] for r in rs) / len(rs)
            print(f"{name:<12} {label:<16} {valid}/{len(SEEDS)}     {at:<10.1f} {ac:<9.1f} {atime:<8.1f}")
        print()


if __name__ == "__main__":
    main()
