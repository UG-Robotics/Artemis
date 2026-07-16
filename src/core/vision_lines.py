"""Detect the orange/blue corner lines on the mat from the wide camera.

Replaces the dead colour sensor. The WRO mat has an orange and a blue line
across the corridor before each corner; which colour appears FIRST fixes the
driving direction (rules: orange first = clockwise, blue first = counter-
clockwise), and every sighting is a corner cue. The camera sees them well even
in low light (verified on the 2026-07-14 run recordings).

Pure numpy, same reasons as vision_heading. Looks only at the lower part of
the frame (the mat directly ahead); classification is per-pixel chromatic:

  orange: red clearly above blue, and the pixel is warm (R >= G >= B-ish)
  blue  : blue clearly above red

measured RELATIVE to the frame's own mat tint: the mat dominates the lower
ROI, so the per-frame median of each chroma channel IS the ambient white
balance, and a line is a chroma OUTLIER against it. An absolute threshold
fails in real halls — on the 2026-07-14 recordings the cool lighting tinted
40-93% of the mat past a fixed blue threshold; median-relative classification
collapses that to the line pixels alone.

Returns 'orange' / 'blue' / None per frame — the same contract as the old
colour sensor's SensorReading.color_detected.
"""

import numpy as np

from core.config import (
    LINE_MIN_PIXELS,
    LINE_CHROMA_MARGIN,
)


def detect_line_color(image) -> str | None:
    """'orange' | 'blue' | None for an RGB (HxWx3 uint8) frame."""
    h = image.shape[0]
    roi = image[int(h * 0.55):].astype(int)   # mat ahead of the nose
    r, g, b = roi[..., 0], roi[..., 1], roi[..., 2]

    rb = r - b                                # warm chroma axis
    br = -rb
    rb_off = rb - int(np.median(rb))          # de-tint: mat median = 0
    br_off = br - int(np.median(br))

    orange = (rb_off > LINE_CHROMA_MARGIN) & (r - g > 0)
    blue = br_off > LINE_CHROMA_MARGIN

    n_orange = int(orange.sum())
    n_blue = int(blue.sum())
    if max(n_orange, n_blue) < LINE_MIN_PIXELS:
        return None
    return 'orange' if n_orange >= n_blue else 'blue'
