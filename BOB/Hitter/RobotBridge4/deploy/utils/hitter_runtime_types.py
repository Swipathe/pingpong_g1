from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class PlannerFailureReason(str, Enum):
    TRACK_ENDED = "TRACK_ENDED"
    ESTIMATOR_NOT_READY = "ESTIMATOR_NOT_READY"
    BASE_POSE_INVALID = "BASE_POSE_INVALID"
    BALL_NOT_INCOMING = "BALL_NOT_INCOMING"
    NO_FUTURE_CROSSING = "NO_FUTURE_CROSSING"
    HIT_HEIGHT_OUT_OF_RANGE = "HIT_HEIGHT_OUT_OF_RANGE"
    NONFINITE_INPUT_OR_OUTPUT = "NONFINITE_INPUT_OR_OUTPUT"
    INTERNAL_ERROR = "INTERNAL_ERROR"


class PlannerRejected(RuntimeError):
    def __init__(self, reason: PlannerFailureReason, detail: str):
        self.reason = PlannerFailureReason(reason)
        self.detail = str(detail)
        super().__init__(f"{self.reason.value}: {self.detail}")


@dataclass(frozen=True, order=True)
class SnapshotKey:
    track_id: int
    generation: int

    def __post_init__(self) -> None:
        if type(self.track_id) is not int or self.track_id <= 0:
            raise ValueError("track_id must be positive")
        if type(self.generation) is not int or self.generation < 0:
            raise ValueError("generation must be a non-negative integer")

    def to_json_dict(self) -> dict[str, int]:
        return {
            "schema_version": 2,
            "track_id": self.track_id,
            "generation": self.generation,
        }
