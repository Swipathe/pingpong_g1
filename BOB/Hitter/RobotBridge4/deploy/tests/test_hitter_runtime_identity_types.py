from __future__ import annotations

import numpy as np
import pytest

from diagnostics.hitter_task_models import SnapshotKey as DiagnosticSnapshotKey
from utils.hitter_realtime import BallEstimateSnapshot, PlannerResultSnapshot
from utils.hitter_runtime_types import PlannerFailureReason, SnapshotKey


@pytest.fixture
def snapshot() -> BallEstimateSnapshot:
    return BallEstimateSnapshot(
        track_id=9,
        generation=1,
        source_frame=1,
        source_time_s=1.0,
        received_monotonic_s=2.0,
        position_w=np.array([1.5, 0.0, 1.0]),
        velocity_w=np.array([-1.0, 0.0, 0.0]),
        base_position_w=np.array([0.0, 0.0, 0.8]),
        base_quaternion_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
        base_valid=True,
        visible=True,
        ready=True,
        consumed=False,
        new_track=True,
    )


def test_snapshot_key_requires_positive_track_id():
    assert SnapshotKey(track_id=9, generation=0).to_json_dict() == {
        "schema_version": 2,
        "track_id": 9,
        "generation": 0,
    }
    with pytest.raises(ValueError, match="track_id must be positive"):
        SnapshotKey(track_id=0, generation=0)


def test_diagnostics_reexports_the_canonical_snapshot_key():
    assert DiagnosticSnapshotKey is SnapshotKey


@pytest.mark.parametrize(
    ("kwargs", "message"),
    [
        ({"track_id": True, "generation": 0}, "track_id must be positive"),
        ({"track_id": 9, "generation": -1}, "generation must be a non-negative integer"),
        ({"track_id": 9, "generation": True}, "generation must be a non-negative integer"),
    ],
)
def test_snapshot_key_rejects_noncanonical_identity_values(kwargs, message):
    with pytest.raises(ValueError, match=message):
        SnapshotKey(**kwargs)


def test_runtime_snapshots_expose_no_epoch_attribute(snapshot):
    assert snapshot.track_id == 9
    assert snapshot.consumed is False
    assert snapshot.new_track is True
    assert not hasattr(snapshot, "track_epoch")

    result = PlannerResultSnapshot(
        track_id=9,
        source_generation=1,
        source_frame=1,
        strike_deadline_monotonic_s=3.0,
        completed_monotonic_s=2.1,
        command=None,
        failure_reason=PlannerFailureReason.INTERNAL_ERROR,
    )
    assert result.track_id == 9
    assert not hasattr(result, "track_epoch")
