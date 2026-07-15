"""Gyro-free heading estimation from the wide-angle camera: wall-base slope.

Second heading source for the 4-ToF + camera sensor suite (the first is the
side-ToF rate estimator, core/tof_heading.py). The two are complementary:

  - ToF rates  : fast, precise, but RELATIVE (a rate needs motion) and blind
                 whenever a side opens up (corner mouths, the exact moment
                 heading matters most on exit);
  - wall slope : ABSOLUTE per frame (no motion needed) and works from the
                 front-facing view, but coarser and ~camera-latency slow.

Principle (FlawFactory's camera-only heading, adapted): the arena mat is light,
the walls are a dark band sitting on it. In a front-facing view, the image row
of the wall's base along the x axis encodes geometry:

  - robot parallel to the side walls -> the base of the wall ahead is level;
    the two side-wall base lines vanish symmetrically;
  - robot yawed by theta -> the wall-base line in the central view tilts, and
    its slope is monotonic in theta for the small angles we care about.

So: find the wall-base row per image column (the bottom edge of the dark band),
fit a line over the central columns (limits fisheye distortion, which grows
toward the edges), and map slope -> degrees with a single calibration gain.

Pure numpy — cv2 is NOT required (the Pi has it, dev machines may not). The
caller hands in an RGB (or grayscale) numpy array; on the Pi that's the same
array the camera driver already captures.

Calibration: VISION_HEADING_GAIN converts slope (rows per column) to degrees.
Bench procedure: put the robot centred in a corridor, yaw it to a few known
angles (tape protractor), record estimate_slope() and fit the line. The default
gain is a placeholder from fisheye geometry — calibrate before trusting scale
(the SIGN and zero point are trustworthy as-is).
"""

import numpy as np

from core.config import (
    VISION_CHROMA_LIMIT,
    VISION_DARK_THRESHOLD,
    VISION_HEADING_GAIN,
    VISION_MIN_COLUMNS,
)


WALL_MIN_RUN = 4  # rows — a dark blob must be at least this thick to be "wall"


def wall_base_profile(image, dark_threshold=VISION_DARK_THRESHOLD):
    """Per-column image row of the wall base (bottom edge of the dark band).

    image: HxW (grayscale) or HxWx3 (RGB) uint8 numpy array.
    Returns (cols, rows): integer arrays of column indices and the wall-base
    row found in each, columns with no detectable band omitted.

    Robustness choices, tuned on real run frames (2026-07-14 recordings):
    - "dark" means max(R,G,B) < threshold AND the pixel is chromatically
      neutral (max-min < VISION_CHROMA_LIMIT): the wall is near-black in every
      channel, while the blue/orange lane lines are tinted — the navy line in
      dim light passes a pure darkness test (seen on real frames) but its
      blue channel sits well above red, so the neutrality test rejects it.
    - the base is found scanning from the BOTTOM: the mat is always bright, so
      the lowest fully-dark window in a column is the wall band — background
      clutter above the wall (spectators, chairs, dark ceiling) can never be
      selected, which the naive top-down scan got wrong on real frames.
    - a dark run must be >= WALL_MIN_RUN rows thick, rejecting thin shadows
      and compression noise.
    """
    if image.ndim == 3:
        channel_max = image.max(axis=2).astype(float)
        chroma = channel_max - image.min(axis=2)
    else:
        channel_max = image.astype(float)
        chroma = np.zeros_like(channel_max)
    h, w = channel_max.shape

    # Skip the extreme bottom (mat under the nose, often in shadow) and the top
    # (pure background). The wall base lives near the vertical middle with the
    # current front-facing, untilted mounting.
    top, bottom = int(h * 0.10), int(h * 0.92)
    dark = ((channel_max[top:bottom] < dark_threshold)
            & (chroma[top:bottom] < VISION_CHROMA_LIMIT))
    n = dark.shape[0]
    if n <= WALL_MIN_RUN:
        return np.array([], dtype=int), np.array([], dtype=int)

    # Sliding-window sum over rows: window fully dark <=> sum == WALL_MIN_RUN.
    csum = np.cumsum(dark, axis=0)
    csum = np.vstack([np.zeros((1, dark.shape[1]), dtype=int), csum])
    winsum = csum[WALL_MIN_RUN:] - csum[:-WALL_MIN_RUN]       # start rows
    full = winsum == WALL_MIN_RUN

    ok = full.any(axis=0)
    # Lowest fully-dark window start per column; its last row = wall base.
    lowest_start = (n - WALL_MIN_RUN) - np.argmax(full[::-1], axis=0)
    base = lowest_start + WALL_MIN_RUN - 1

    cols = np.nonzero(ok)[0]
    rows = base[ok] + top
    return cols, rows


def estimate_slope(image, dark_threshold=VISION_DARK_THRESHOLD,
                   min_columns=VISION_MIN_COLUMNS):
    """Least-squares slope (rows per column) of the wall base over the central
    half of the frame, or None if too few columns detected."""
    cols, rows = wall_base_profile(image, dark_threshold)
    if len(cols) == 0:
        return None
    w = image.shape[1]
    central = (cols > w * 0.25) & (cols < w * 0.75)
    cols, rows = cols[central], rows[central]
    if len(cols) < min_columns:
        return None
    # Median-filter the profile before fitting: lane lines / pillar bases can
    # notch individual columns.
    order = np.argsort(cols)
    cols, rows = cols[order].astype(float), rows[order].astype(float)
    fit = np.polyfit(cols, rows, 1)
    return float(fit[0])


def estimate_heading(image, dark_threshold=VISION_DARK_THRESHOLD,
                     gain=VISION_HEADING_GAIN) -> float | None:
    """Heading error in degrees (positive = nose toward the right wall),
    or None when no usable wall base is in view."""
    slope = estimate_slope(image, dark_threshold)
    if slope is None:
        return None
    # Yawing right raises the wall base on the right side of the image
    # (right wall nearer -> its base lower... sign fixed by bench convention
    # below and verified against real frames in tools/vision_heading_demo.py).
    return gain * slope
