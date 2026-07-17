"""Summarize a run log: the detection timeline + control milestones.

    python3 -m robot.bench.log_timeline                 # newest run
    python3 -m robot.bench.log_timeline <run-dir>       # a specific run
    python3 -m robot.bench.log_timeline --list          # list runs

Prints, in time order: every camera detection CHANGE (line colour / pillars
appearing or clearing) and every controller state / turn-count change — so you
can read "t=12.4s camera saw orange" against "t=12.5s controller entered a
turn". Points at the saved frame for each detection change.
"""

import json
import math
import os
import sys

LOGS = os.path.expanduser("~/artemis/logs")


def _runs():
    if not os.path.isdir(LOGS):
        return []
    return sorted((os.path.join(LOGS, d) for d in os.listdir(LOGS)
                   if os.path.isdir(os.path.join(LOGS, d))))


def timeline(run: str) -> None:
    meta = {}
    mp = os.path.join(run, "meta.json")
    if os.path.exists(mp):
        meta = json.load(open(mp))
    print(f"=== {os.path.basename(run.rstrip('/'))} ===")
    print(f"mode={meta.get('mode')} dir={meta.get('direction','-')} "
          f"width={meta.get('track_width','-')} rev={meta.get('code_revision','-')} "
          f"dur={meta.get('duration_s','?')}s")
    if "summary" in meta:
        print(f"result: {meta['summary']}")

    events = []

    # camera: emit a row when the detection (line + pillar set) changes
    cam = os.path.join(run, "camera.jsonl")
    if os.path.exists(cam):
        last = object()
        for line in open(cam):
            try:
                r = json.loads(line)
            except ValueError:
                continue
            det = (r.get("line"), tuple(sorted(p.get("color", "?")
                                              for p in r.get("pillars", []))))
            if det != last:
                pills = ",".join(det[1]) or "-"
                frame = r.get("frame") or ""
                events.append((r["t"], "CAM",
                               f"line={det[0] or '-':<6} pillars={pills:<10} {frame}"))
                last = det

    # control: emit a row when state or turn count changes
    sc = os.path.join(run, "sensors.csv")
    if os.path.exists(sc):
        last = None
        with open(sc) as f:
            header = f.readline().strip().split(",")
            ix = {name: i for i, name in enumerate(header)}
            for line in f:
                c = line.rstrip("\n").split(",")
                if len(c) < len(header):
                    continue
                key = (c[ix["state"]], c[ix["turns"]])
                if key != last:
                    events.append((float(c[ix["t"]]), "CTRL",
                                   f"state={key[0]:<12} turns={key[1]}"))
                    last = key

    for t, kind, msg in sorted(events, key=lambda e: e[0]):
        print(f"  t={t:7.2f}s  {kind:<4} {msg}")
    print(f"({len(events)} events)")

    # square-drive runs carry an estimated x/y path — draw it
    sqf = os.path.join(run, "square.csv")
    if os.path.exists(sqf):
        _plot_path(sqf)


def _plot_path(square_csv: str, w: int = 61, h: int = 25) -> None:
    """ASCII scatter of the estimated x/y path (works over SSH, no GUI/deps).

    S = start (0,0), E = end, digits = the phase-boundary corner index, '.' the
    path. Y is drawn up = +y (north). The closing gap between S and E is the
    dead-reckoning error you also see physically."""
    pts, corners = [], []
    with open(square_csv) as f:
        header = f.readline().strip().split(",")
        ix = {n: i for i, n in enumerate(header)}
        last_side = None
        for line in f:
            c = line.rstrip("\n").split(",")
            if len(c) <= ix.get("y_mm", 0):
                continue
            try:
                x, y, side = float(c[ix["x_mm"]]), float(c[ix["y_mm"]]), c[ix["side"]]
            except ValueError:
                continue
            pts.append((x, y))
            if side != last_side and side.isdigit():
                corners.append((x, y, side))
                last_side = side
    if len(pts) < 2:
        print("  (no path points to plot)")
        return

    xs = [p[0] for p in pts] + [0.0]
    ys = [p[1] for p in pts] + [0.0]
    minx, maxx, miny, maxy = min(xs), max(xs), min(ys), max(ys)
    spanx = max(maxx - minx, 1.0)
    spany = max(maxy - miny, 1.0)

    def cell(x, y):
        cx = int((x - minx) / spanx * (w - 1))
        cy = int((y - miny) / spany * (h - 1))
        return cx, (h - 1 - cy)      # invert so +y is up

    grid = [[" "] * w for _ in range(h)]
    for x, y in pts:
        cx, cy = cell(x, y)
        if grid[cy][cx] == " ":
            grid[cy][cx] = "."
    for x, y, side in corners:
        cx, cy = cell(x, y)
        grid[cy][cx] = side
    sx, sy = cell(0.0, 0.0)
    ex, ey = cell(pts[-1][0], pts[-1][1])
    grid[sy][sx] = "S"
    grid[ey][ex] = "E"

    print(f"\n  estimated path  ({minx:.0f}..{maxx:.0f} x, {miny:.0f}..{maxy:.0f} y mm)"
          f"   S=start E=end  gap={math.hypot(pts[-1][0], pts[-1][1]):.0f}mm")
    for row in grid:
        print("  |" + "".join(row) + "|")


def main(argv):
    if "--list" in argv:
        for r in _runs():
            print(r)
        return
    args = [a for a in argv if not a.startswith("-")]
    if args:
        run = args[0]
    else:
        runs = _runs()
        if not runs:
            print(f"no runs under {LOGS}")
            return
        run = runs[-1]
    timeline(run)


if __name__ == "__main__":
    main(sys.argv[1:])
