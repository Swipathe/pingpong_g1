from typing import Tuple


ARM_JOINT_NAMES: Tuple[str, ...] = (
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
)
ROBOT29_ARM_INDICES: Tuple[int, ...] = tuple(range(15, 29))
MOTION_NPZ_ARM_INDICES: Tuple[int, ...] = (
    11,
    15,
    19,
    21,
    23,
    25,
    27,
    12,
    16,
    20,
    22,
    24,
    26,
    28,
)
SOFT_LIMIT_FACTOR = 0.9
POSITION_TOLERANCE_M = 0.0005
ORIENTATION_TOLERANCE_RAD = 0.0017453292519943296
MAX_REQUEST_BODY_BYTES = 262144
MAX_MESH_BYTES = 67108864
MAX_STL_TRIANGLES = 1000000
POSE_SCHEMA = "hitter_ready_arm_pose/v1"


def centered_soft_limit(
    lower: float,
    upper: float,
    factor: float = SOFT_LIMIT_FACTOR,
) -> Tuple[float, float]:
    if not lower < upper:
        raise ValueError("lower must be smaller than upper")
    if not 0.0 < factor <= 1.0:
        raise ValueError("factor must be in (0, 1]")
    midpoint = (lower + upper) / 2.0
    half_range = (upper - lower) * factor / 2.0
    return midpoint - half_range, midpoint + half_range
