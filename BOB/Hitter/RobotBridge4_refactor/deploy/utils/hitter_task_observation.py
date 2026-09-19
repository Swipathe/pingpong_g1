from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation as sRot


@dataclass(frozen=True)
class TaskObservationResult:
    pre_clip: np.ndarray
    post_clip: np.ndarray
    clip_mask: np.ndarray
    clip_count: int
    errors: tuple[str, ...]


class TaskObservationAssemblyError(ValueError):
    reason_code: str

    def __init__(self, message: str, *, reason_code: str):
        super().__init__(message)
        self.reason_code = reason_code


def _copied_float32_vector(
    value: np.ndarray,
    *,
    name: str,
    shape: tuple[int, ...],
) -> np.ndarray:
    copied = np.array(value, dtype=np.float32, copy=True)
    if copied.shape != shape:
        raise TaskObservationAssemblyError(
            f"{name} must have shape {shape}, received {copied.shape}.",
            reason_code="OBS_WRONG_SHAPE",
        )
    return copied


def _normalized_anchor_quaternion(
    quaternion_xyzw: np.ndarray,
) -> np.ndarray:
    norm = np.linalg.norm(quaternion_xyzw)
    if not np.isfinite(norm):
        return np.full(4, np.nan, dtype=np.float32)
    if norm < 1.0e-6:
        return np.asarray([0.0, 0.0, 0.0, 1.0], dtype=np.float32)
    return (quaternion_xyzw / norm).astype(np.float32)


def _yaw_inverse_apply(
    quaternion_xyzw: np.ndarray,
    vector_w: np.ndarray,
) -> np.ndarray:
    yaw = float(sRot.from_quat(np.asarray(quaternion_xyzw, dtype=np.float64)).as_euler("xyz", degrees=False)[2])
    return (
        sRot.from_euler(
            "z",
            -yaw,
            degrees=False,
        )
        .apply(vector_w)
        .astype(np.float32)
    )


def assemble_active_hitter_task_observation(
    *,
    robot_anchor_position_w: np.ndarray,
    robot_anchor_quaternion_xyzw: np.ndarray,
    base_target_xy_w: np.ndarray,
    racket_target_position_w: np.ndarray,
    racket_target_velocity_w: np.ndarray,
    policy_time_to_strike_s: float,
    maximum_policy_time_to_strike_s: float,
    obs_clip_value: float | None,
) -> TaskObservationResult:
    """Return active HITTER task fields in full-observation indices 6:17."""
    robot_anchor_position_w = _copied_float32_vector(
        robot_anchor_position_w,
        name="robot_anchor_position_w",
        shape=(3,),
    )
    robot_anchor_quaternion_xyzw = _copied_float32_vector(
        robot_anchor_quaternion_xyzw,
        name="robot_anchor_quaternion_xyzw",
        shape=(4,),
    )
    base_target_xy_w = _copied_float32_vector(
        base_target_xy_w,
        name="base_target_xy_w",
        shape=(2,),
    )
    racket_target_position_w = _copied_float32_vector(
        racket_target_position_w,
        name="racket_target_position_w",
        shape=(3,),
    )
    racket_target_velocity_w = _copied_float32_vector(
        racket_target_velocity_w,
        name="racket_target_velocity_w",
        shape=(3,),
    )

    anchor_quaternion = _normalized_anchor_quaternion(robot_anchor_quaternion_xyzw)
    if np.all(np.isfinite(anchor_quaternion)):
        base_forward_xy_w = sRot.from_quat(anchor_quaternion).as_matrix()[:, 0][:2].astype(np.float32)
        base_target_delta_w = np.asarray(
            [
                base_target_xy_w[0] - robot_anchor_position_w[0],
                base_target_xy_w[1] - robot_anchor_position_w[1],
                0.0,
            ],
            dtype=np.float32,
        )
        base_target_xy_b = _yaw_inverse_apply(
            anchor_quaternion,
            base_target_delta_w,
        )[:2]
        racket_target_pos_b = _yaw_inverse_apply(
            anchor_quaternion,
            racket_target_position_w - robot_anchor_position_w,
        )
    else:
        base_forward_xy_w = np.full(2, np.nan, dtype=np.float32)
        base_target_xy_b = np.full(2, np.nan, dtype=np.float32)
        racket_target_pos_b = np.full(3, np.nan, dtype=np.float32)

    policy_tts = float(policy_time_to_strike_s)
    maximum_policy_tts = float(maximum_policy_time_to_strike_s)
    pre_clip = np.concatenate(
        [
            base_forward_xy_w,
            base_target_xy_b,
            racket_target_pos_b,
            racket_target_velocity_w,
            np.asarray([policy_tts], dtype=np.float32),
        ],
        axis=0,
    ).astype(np.float32)

    clip_value = None if obs_clip_value is None else float(obs_clip_value)
    if clip_value is not None and clip_value > 0.0:
        post_clip = np.clip(
            pre_clip,
            -clip_value,
            clip_value,
        ).astype(np.float32)
    else:
        post_clip = pre_clip.copy()

    finite_mask = np.isfinite(pre_clip)
    clip_mask = finite_mask & (post_clip != pre_clip)
    errors = []
    if not np.all(finite_mask):
        errors.append("OBS_NONFINITE")
    if np.any(clip_mask):
        errors.append("OBS_CLIPPED")
    if np.isfinite(policy_tts) and not (0.0 < policy_tts <= maximum_policy_tts):
        errors.append("OBS_TTS_OUT_OF_RANGE")

    pre_clip.setflags(write=False)
    post_clip.setflags(write=False)
    clip_mask.setflags(write=False)
    return TaskObservationResult(
        pre_clip=pre_clip,
        post_clip=post_clip,
        clip_mask=clip_mask,
        clip_count=int(np.count_nonzero(clip_mask)),
        errors=tuple(errors),
    )


__all__ = [
    "TaskObservationAssemblyError",
    "TaskObservationResult",
    "assemble_active_hitter_task_observation",
]
