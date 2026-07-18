"""Score the ImuFusionController in sim (uses the sim's noisy/drifting IMU +
ToF, and can inject ToF dropout noise). Same WRO scoring as test_open_score:
sections(<=24) + laps(<=3) + finish-in-start-section(3) = max 30.
"""

import math
import os
import random
import sys

SRC = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, SRC)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import importlib.util
_spec = importlib.util.spec_from_file_location(
    "tnoise", os.path.join(os.path.dirname(os.path.abspath(__file__)), "test_tof_noise.py"))
T = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(T)

from core.config import SIM_FPS, CONTROL_HZ, ROUND_TIME
from core.imu_fusion_controller import ImuFusionController, IFState
from core.tof_filter import TofMedianFilter
from track import Track
from robot import Robot

POS = ("front", "left", "right", "rear")


def run_scored(widths, direction, start, seed, noise, p_spike=0.05):
    rng = random.Random(seed)
    random.seed(seed * 7919 + 13)
    track = Track(challenge_type='open', section_widths=widths,
                  driving_direction=direction, starting_section_idx=start)
    cx, cy, angle = T.start_pose(track)
    robot = Robot(cx, cy, angle)
    controller = ImuFusionController()
    filt = TofMedianFilter(POS)

    ox1, oy1, ox2, oy2 = track.outer_rect
    ccx, ccy = (ox1 + ox2) / 2, (oy1 + oy2) / 2
    pdt, cdt = 1.0 / SIM_FPS, 1.0 / CONTROL_HZ
    acc = elapsed = 0.0
    collisions = 0
    in_coll = False
    sweep = 0.0
    prev_ang = math.atan2(robot.y - ccy, robot.x - ccx)

    while elapsed < ROUND_TIME:
        acc += pdt
        while acc >= cdt:
            acc -= cdt
            reading = robot.get_sensors(track)
            if noise:
                T.inject_spikes(reading, rng, p_spike)
            for p in POS:
                setattr(reading, f"tof_{p}", filt.update(p, getattr(reading, f"tof_{p}")))
            controller.update(reading, robot, track, cdt)
        robot.update(pdt)
        ang = math.atan2(robot.y - ccy, robot.x - ccx)
        d = ang - prev_ang
        if d > math.pi: d -= 2 * math.pi
        elif d < -math.pi: d += 2 * math.pi
        prev_ang = ang
        sweep += d
        if robot.check_wall_collision(track):
            if not in_coll:
                collisions += 1
                in_coll = True
            irx1, iry1, irx2, iry2 = track.inner_rect
            if irx1 <= robot.x <= irx2 and iry1 <= robot.y <= iry2:
                dx, dy = robot.x - ccx, robot.y - ccy
            else:
                dx, dy = ccx - robot.x, ccy - robot.y
            dd = max(1, math.hypot(dx, dy))
            robot.x += dx / dd * 20
            robot.y += dy / dd * 20
        else:
            in_coll = False
        elapsed += pdt
        if controller.state == IFState.STOPPED:
            break

    deg = abs(math.degrees(sweep))
    sections = max(0, int((deg + 22.5) // 45))
    laps = min(3, sections // 8)
    sec_pts = min(24, sections)
    finish = 0
    if controller.state == IFState.STOPPED and laps >= 3 and elapsed < ROUND_TIME:
        s = track.straight_sections[track.starting_section_idx]
        pad = 60
        if (min(s.x1, s.x2) - pad <= robot.x <= max(s.x1, s.x2) + pad and
                min(s.y1, s.y2) - pad <= robot.y <= max(s.y1, s.y2) + pad):
            finish = 3
    return {'total': sec_pts + laps + finish, 'sections': sec_pts, 'laps': laps,
            'finish': finish, 'collisions': collisions, 'time': elapsed}


def main():
    print("ImuFusionController — WRO open score (sim IMU+ToF, gyro-executed turns)\n")
    seeds = list(range(1, 11))
    grand = []
    for name, widths, d, st in T.CONFIGS:
        for label, noise in [('clean', False), ('noisy', True)]:
            rs = [run_scored(widths, d, st, s, noise) for s in seeds]
            tots = [r['total'] for r in rs]
            grand += tots
            print(f"  {name:<12} {label:<6} avg={sum(tots)/len(tots):5.1f}/30  "
                  f"perfect={sum(1 for t in tots if t == 30)}/10  "
                  f"worst={min(tots):2d}  coll={sum(r['collisions'] for r in rs)/len(rs):.1f}")
        print()
    print(f"OVERALL: avg {sum(grand)/len(grand):.2f}/30, "
          f"{sum(1 for t in grand if t == 30)}/{len(grand)} perfect")


if __name__ == "__main__":
    main()
