"""Score the WallFollowController open-challenge runs per WRO rules.

Ground-truth scoring from the sim (the controller itself is pose-free):
- sections: cumulative angle swept around the track centre, 45 deg per section
  (8 sections/lap), only counted in the driving direction, capped at 24
- laps: sections // 8, capped 3
- finish bonus (+3): controller STOPPED after >= 3 laps inside the starting
  straight section

Max = 24 + 3 + 3 = 30.
"""

import math
import os
import random
import sys

SRC = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, SRC)
sys.path.insert(0, os.path.join(SRC, "sim", "open-challenge-sim"))

import importlib.util
spec = importlib.util.spec_from_file_location(
    "tnoise", os.path.join(os.path.dirname(os.path.abspath(__file__)), "test_tof_noise.py"))
T = importlib.util.module_from_spec(spec)
spec.loader.exec_module(T)

from core.config import SIM_FPS, CONTROL_HZ, ROUND_TIME
from core.wall_follow_controller import WallFollowController, WFState
from core.tof_filter import TofMedianFilter
from track import Track
from robot import Robot

POSITIONS = ("front", "left", "right", "rear")


def run_scored(section_widths, direction, start_section, seed, noise, p_spike):
    rng = random.Random(seed)
    random.seed(seed * 7919 + 13)
    track = Track(challenge_type='open', section_widths=section_widths,
                  driving_direction=direction, starting_section_idx=start_section)
    cx, cy, angle = T.start_pose(track)
    robot = Robot(cx, cy, angle)
    controller = WallFollowController()
    filt = TofMedianFilter(POSITIONS)  # driver defaults (gate-only)

    ox1, oy1, ox2, oy2 = track.outer_rect
    ccx, ccy = (ox1 + ox2) / 2, (oy1 + oy2) / 2

    physics_dt, control_dt = 1.0 / SIM_FPS, 1.0 / CONTROL_HZ
    acc = elapsed = 0.0
    collisions = 0
    in_collision = False
    sweep = 0.0
    prev_ang = math.atan2(robot.y - ccy, robot.x - ccx)

    while elapsed < ROUND_TIME:
        acc += physics_dt
        while acc >= control_dt:
            acc -= control_dt
            reading = robot.get_sensors(track)
            if noise:
                T.inject_spikes(reading, rng, p_spike)
            for pos in POSITIONS:
                setattr(reading, f"tof_{pos}",
                        filt.update(pos, getattr(reading, f"tof_{pos}")))
            controller.update(reading, robot, track, control_dt)
        robot.update(physics_dt)

        ang = math.atan2(robot.y - ccy, robot.x - ccx)
        d = ang - prev_ang
        if d > math.pi: d -= 2 * math.pi
        elif d < -math.pi: d += 2 * math.pi
        prev_ang = ang
        # The controller picks its own loop direction at the first corner (the
        # scoring direction param only sets the start pose), so score |sweep|.
        sweep += d

        if robot.check_wall_collision(track):
            if not in_collision:
                collisions += 1
                in_collision = True
            dx, dy = ccx - robot.x, ccy - robot.y
            dd = max(1, math.hypot(dx, dy))
            robot.x += dx / dd * 20
            robot.y += dy / dd * 20
        else:
            in_collision = False
        elapsed += physics_dt
        if controller.state == WFState.STOPPED:
            break

    # Boundary-crossing count: start is mid-section, so the first section
    # boundary sits ~22.5 deg of sweep away, then one every 45 deg.
    deg = abs(math.degrees(sweep))
    sections = max(0, int((deg + 22.5) // 45))
    laps = min(3, sections // 8)
    sec_pts = min(24, sections)
    finish = 0
    if controller.state == WFState.STOPPED and laps >= 3 and elapsed < ROUND_TIME:
        s = track.straight_sections[track.starting_section_idx]
        # generous membership: within the section's bounding rect padded by width
        pad = 60
        if (min(s.x1, s.x2) - pad <= robot.x <= max(s.x1, s.x2) + pad and
                min(s.y1, s.y2) - pad <= robot.y <= max(s.y1, s.y2) + pad):
            finish = 3
    total = sec_pts + laps + finish
    return {'total': total, 'sections': sec_pts, 'laps': laps, 'finish': finish,
            'collisions': collisions, 'time': elapsed}


def main():
    print("WRO open-challenge score, driver defaults (gate filter, heading fusion)")
    print("scoring: sections(<=24) + laps(<=3) + finish-in-start-section(3) = max 30\n")
    seeds = list(range(1, 11))
    grand = []
    for name, widths, d, st in T.CONFIGS:
        for label, noise in [('clean', False), ('noisy', True)]:
            rs = [run_scored(widths, d, st, s, noise, 0.05) for s in seeds]
            tots = [r['total'] for r in rs]
            full = sum(1 for r in rs if r['total'] == 30)
            avg = sum(tots) / len(tots)
            worst = min(tots)
            fin = sum(1 for r in rs if r['finish'])
            coll = sum(r['collisions'] for r in rs) / len(rs)
            grand += tots
            print(f"  {name:<12} {label:<6} avg={avg:5.1f}/30  perfect={full}/10  "
                  f"worst={worst:2d}  finish_bonus={fin}/10  coll={coll:.1f}")
    print(f"\n  OVERALL: avg {sum(grand)/len(grand):.1f}/30 across {len(grand)} runs, "
          f"{sum(1 for t in grand if t==30)}/{len(grand)} perfect")


if __name__ == "__main__":
    main()
