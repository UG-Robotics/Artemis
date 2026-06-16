"""The sensor reading shared between perception and control.

Both the sim's `Robot` and the real drivers produce one of these; the
`Controller` consumes it. That shared shape is what lets one controller run
against either source.
"""

from dataclasses import dataclass, field
from typing import List, Optional

from core.config import TOF_MAX_RANGE


@dataclass
class SensorReading:
    """All sensors at one control tick.

    Distances in mm; imu_heading in degrees [0, 360); color_detected is
    'orange'/'blue'/None; pillars_visible is dicts of {color, angle, distance},
    nearest first.
    """
    tof_front: float = TOF_MAX_RANGE
    tof_rear: float = TOF_MAX_RANGE
    tof_left: float = TOF_MAX_RANGE
    tof_right: float = TOF_MAX_RANGE
    imu_heading: float = 0.0
    color_detected: Optional[str] = None
    pillars_visible: List[dict] = field(default_factory=list)
