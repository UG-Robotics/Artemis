"""Per-channel de-noising for the ToF sensors: outlier gate + rolling median.

Shared by the real driver (`robot.drivers.tof.TofArray`) and the simulator so the
exact same de-noising runs in both places. The VL53L1X occasionally drops a
single frame to a spurious value — most often the no-target sentinel jumping to
TOF_MAX_RANGE (e.g. 233, 233, 4000, 234) but also stray near reads.

Two stages, both per channel:

1. Outlier gate (zero lag on plausible samples): a sample further than
   TOF_MAX_JUMP from the current output can't be real frame-to-frame motion, so
   it is rejected and the previous output held — UNLESS it agrees (within
   TOF_MAX_JUMP) with the immediately previous raw sample. Two matching
   "impossible" samples in a row mean a genuine level shift (a wall edge
   passing: 250 -> 4000 legitimately in one frame); the shift is accepted and
   the median window reset so the new level lands immediately, not after the
   window refills.

   Escape valve: the gate never rejects more than MAX_CONSECUTIVE_REJECTS
   samples in a row. During a full-lock turn the true distance can ramp faster
   than TOF_MAX_JUMP every frame (the ray sweeps along a wall), so consecutive
   samples confirm neither the output nor each other; without the valve the
   gate would hold a stale pre-turn reading for the whole ramp (sim: 10/10 ->
   0/10 on the narrow track from late turn exits). With it, staleness is
   bounded at 2 frames while an isolated single-frame spike is still rejected.

2. Rolling median over the last N accepted samples: mops up small in-range
   glitches the gate lets through. Median beats an averaging/low-pass filter
   here: it never smears a spike into its neighbours, and it adds at most
   floor(N/2) samples of lag instead of an exponential tail.
"""

from collections import deque
from statistics import median

from core.config import TOF_MAX_JUMP, TOF_MEDIAN_WINDOW


class TofMedianFilter:
    """Outlier gate + rolling median per named channel (front/left/right/rear)."""

    MAX_CONSECUTIVE_REJECTS = 2

    def __init__(self, positions, window: int = TOF_MEDIAN_WINDOW,
                 max_jump: float = TOF_MAX_JUMP):
        self.window = max(1, int(window))
        self.max_jump = max_jump  # None disables the gate
        self._history = {p: deque(maxlen=self.window) for p in positions}
        self._prev_raw = {p: None for p in positions}
        self._out = {p: None for p in positions}
        self._rejects = {p: 0 for p in positions}

    def update(self, position: str, sample: float) -> float:
        """Push one raw sample for a channel, return the current filtered value."""
        prev_raw = self._prev_raw[position]
        out = self._out[position]
        self._prev_raw[position] = sample
        hist = self._history[position]

        if self.max_jump is not None and out is not None:
            plausible = abs(sample - out) <= self.max_jump
            confirmed_shift = (not plausible and prev_raw is not None
                               and abs(sample - prev_raw) <= self.max_jump)
            forced = (not plausible and not confirmed_shift
                      and self._rejects[position] >= self.MAX_CONSECUTIVE_REJECTS)
            if confirmed_shift or forced:
                hist.clear()  # level shift/ramp: don't let the old level outvote it
            elif not plausible:
                self._rejects[position] += 1
                return out    # isolated outlier: hold the previous output

        self._rejects[position] = 0
        hist.append(sample)
        self._out[position] = median(hist)
        return self._out[position]

    def filter(self, readings: dict) -> dict:
        """Filter a {position: raw_distance} dict, returning filtered distances."""
        return {p: self.update(p, v) for p, v in readings.items()}
