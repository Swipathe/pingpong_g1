from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional

import numpy as np


@dataclass(frozen=True)
class MocapFrame:
    frame_number: int
    source_time_s: float
    body_position_mm: Optional[np.ndarray]
    body_quaternion_xyzw: Optional[np.ndarray]
    body_markers_mm: Dict[int, np.ndarray]
    unlabeled_markers_mm: np.ndarray
    body_pose_source_time_s: Optional[float] = None
