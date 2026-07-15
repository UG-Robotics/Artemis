"""Gyro-free heading estimation from the side ToF sensors.

The wall-follower's core weakness (see wall_follow_controller.py) is that a
single ToF per side can't separate lateral offset from heading: (right - left)
is the same for "off-centre but parallel" and "centred but angled". This module
recovers the heading half of that ambiguity from the RATE of change of the side
distances: driving at speed v with heading error theta (relative to the
corridor axis, positive = nose toward the right wall),

    d_left_dot  = +v * sin(theta)
    d_right_dot = -v * sin(theta)

A single tick of difference is useless — at cruise (~205 mm/s, 30 Hz) a 5 deg
error moves a side reading ~0.6 mm/tick, below the sensor's ~1 mm noise — so
the rate comes from a least-squares slope over a short rolling window
(HEADING_WINDOW_TICKS, ~0.3 s): slope noise there is ~3 mm/s, i.e. under 1 deg
of heading at cruise. The price is ~half a window of latency, which is fine for
a damping term.

No gyro anywhere: this is one of the two heading sources for the sensor suite
(4 ToF + wide camera) — the other is the camera wall-angle estimator, which is
absolute but slower; this one is fast but only valid alongside a straight wall.

Usage per control tick, straights only (reset() on every turn entry/exit —
corner geometry breaks the wall-parallel model):

    theta = est.update(d_left, d_right, left_valid, right_valid, speed_mms, dt)
    # theta in degrees, positive = nose toward the right wall; None until
    # enough consecutive valid samples accumulate.
"""

import math
from collections import deque

from core.config import HEADING_WINDOW_TICKS


def _slope(points):
    """Least-squares slope of (t, d) pairs, in mm/s."""
    n = len(points)
    mt = sum(p[0] for p in points) / n
    md = sum(p[1] for p in points) / n
    num = sum((t - mt) * (d - md) for t, d in points)
    den = sum((t - mt) ** 2 for t, _ in points)
    return num / den if den > 0 else 0.0


class TofHeadingEstimator:
    """Heading error (deg) from side-ToF distance rates. Positive = veering right."""

    def __init__(self, window: int = HEADING_WINDOW_TICKS):
        self.window = max(3, int(window))
        self._left = deque(maxlen=self.window)   # (t, distance) while left valid
        self._right = deque(maxlen=self.window)
        self._t = 0.0

    def reset(self):
        """Drop history — call at turn entry/exit, where the model is invalid."""
        self._left.clear()
        self._right.clear()

    def update(self, d_left, d_right, left_valid, right_valid,
               speed_mms, dt) -> float | None:
        """Feed one tick; return heading error in degrees, or None if unknown."""
        self._t += dt
        # A side going invalid mid-window means the geometry changed (opening),
        # not the heading — restart that side's history.
        for hist, dist, valid in ((self._left, d_left, left_valid),
                                  (self._right, d_right, right_valid)):
            if not valid:
                hist.clear()
                continue
            # A jump a wall can't make in one tick (leaked spike or a wall
            # edge) would poison the slope fit — restart the window instead.
            if hist and abs(dist - hist[-1][1]) > 300:
                hist.clear()
            hist.append((self._t, dist))

        if speed_mms is None or speed_mms < 1e-6:
            return None

        estimates = []
        if len(self._left) == self.window:
            estimates.append(_slope(self._left) / speed_mms)      # +v*sin(theta)
        if len(self._right) == self.window:
            estimates.append(-_slope(self._right) / speed_mms)    # -v*sin(theta)
        if not estimates:
            return None

        sin_theta = sum(estimates) / len(estimates)
        sin_theta = max(-1.0, min(1.0, sin_theta))
        return math.degrees(math.asin(sin_theta))
