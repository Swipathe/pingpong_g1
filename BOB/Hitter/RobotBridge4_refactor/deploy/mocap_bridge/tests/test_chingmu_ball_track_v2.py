import json
import subprocess
from types import SimpleNamespace

import numpy as np
import pytest

from deploy.mocap_bridge import chingmu_table_lcm_bridge as bridge_module
from deploy.mocap_bridge.chingmu_table_lcm_bridge import (
    BallTracker,
    BridgeConfig,
    ChingMuTableLcmBridge,
    PelvisOrientationCalibration,
    TableFrame,
    _operation_mode,
    _validate_numeric_arguments,
    build_arg_parser,
)

INT64_MAX = 2**63 - 1


@pytest.fixture
def identity_table():
    return TableFrame(
        center_raw_mm=np.zeros(3, dtype=np.float64),
        x_axis_raw=np.array([1.0, 0.0, 0.0], dtype=np.float64),
        y_axis_raw=np.array([0.0, 1.0, 0.0], dtype=np.float64),
        z_axis_raw=np.array([0.0, 0.0, 1.0], dtype=np.float64),
        corners_raw_mm=np.zeros((4, 3), dtype=np.float64),
        rectangle_score=0.0,
    )


def raw(position_world, config=None):
    config = BridgeConfig() if config is None else config
    position_world = np.asarray(position_world, dtype=np.float64).reshape(3)
    return (
        position_world
        - np.array(
            [0.5 * config.table_length_m, 0.0, config.table_height_m],
            dtype=np.float64,
        )
    ) * 1000.0


def fixture_row(update):
    return {
        "phase": update.phase.value,
        "track_id": update.track_id,
        "publish_valid": update.publish_valid,
        "publish_end": update.publish_end,
        "position_world": [round(float(component), 6) for component in update.position_world],
    }


def python_contract_fixture():
    table = TableFrame(
        center_raw_mm=np.zeros(3, dtype=np.float64),
        x_axis_raw=np.array([1.0, 0.0, 0.0], dtype=np.float64),
        y_axis_raw=np.array([0.0, 1.0, 0.0], dtype=np.float64),
        z_axis_raw=np.array([0.0, 0.0, 1.0], dtype=np.float64),
        corners_raw_mm=np.zeros((4, 3), dtype=np.float64),
        rectangle_score=0.0,
    )
    config = BridgeConfig()
    tracker = BallTracker()
    cases = [
        ([raw([0.8, -0.1, 1.0], config)], 100, 1.00, 1_000_000),
        ([raw([0.5, -0.1, 1.0], config)], 101, 1.01, 1_000_001),
        ([], 102, 1.02, 1_000_002),
        ([raw([-0.02, -0.1, 1.0], config)], 103, 1.03, 1_000_003),
        ([], 128, 1.28, 1_000_004),
        ([raw([0.9, 0.2, 1.1], config)], 129, 1.29, 1_000_100),
    ]
    return [
        fixture_row(
            tracker.update(
                markers,
                frame_number=frame,
                source_time_s=source_time,
                table=table,
                config=config,
                allocation_time_us=allocation_time,
            )
        )
        for markers, frame, source_time, allocation_time in cases
    ]


def test_short_miss_and_return_keep_the_same_id(identity_table):
    tracker = BallTracker()
    first = tracker.update(
        [raw([0.8, -0.1, 1.0])],
        frame_number=100,
        source_time_s=1.0,
        table=identity_table,
        config=BridgeConfig(),
        allocation_time_us=1_000_000,
    )
    approach = tracker.update(
        [raw([0.5, -0.1, 1.0])],
        frame_number=101,
        source_time_s=1.01,
        table=identity_table,
        config=BridgeConfig(),
        allocation_time_us=1_000_001,
    )
    miss = tracker.update(
        [],
        frame_number=102,
        source_time_s=1.02,
        table=identity_table,
        config=BridgeConfig(),
    )
    returned = tracker.update(
        [raw([-0.02, -0.1, 1.0])],
        frame_number=103,
        source_time_s=1.03,
        table=identity_table,
        config=BridgeConfig(),
    )
    assert first.track_id == approach.track_id == returned.track_id == 1_000_000
    assert miss.phase is bridge_module.BallTrackPhase.MISSING_GRACE
    assert not miss.publish_end
    assert returned.publish_valid


def test_cpp_and_python_contract_fixtures_match():
    cpp = json.loads(
        subprocess.check_output(
            [
                "deploy/mocap_bridge/.build-v2/test_vicon_ball_track_v2",
                "--emit-track-fixture",
            ],
            text=True,
        )
    )
    assert python_contract_fixture() == cpp


def test_candidate_selection_is_independent_of_marker_order(identity_table):
    config = BridgeConfig()
    inactive_candidates = [raw([0.8, 0.1, 1.0]), raw([0.4, 0.2, 1.0])]
    inactive_results = []
    for candidates in (inactive_candidates, list(reversed(inactive_candidates))):
        inactive_results.append(
            BallTracker().update(
                candidates,
                frame_number=100,
                source_time_s=1.0,
                table=identity_table,
                config=config,
                allocation_time_us=1_350_000,
            )
        )
    np.testing.assert_allclose(inactive_results[0].position_world, [0.4, 0.2, 1.0])
    np.testing.assert_allclose(inactive_results[0].position_world, inactive_results[1].position_world)

    tied_candidates = [raw([1.25, 0.0, 1.0]), raw([0.75, 0.0, 1.0])]
    active_results = []
    for candidates in (tied_candidates, list(reversed(tied_candidates))):
        tracker = BallTracker()
        tracker.update(
            [raw([1.0, 0.0, 1.0])],
            frame_number=100,
            source_time_s=1.0,
            table=identity_table,
            config=config,
            allocation_time_us=1_360_000,
        )
        active_results.append(
            tracker.update(
                candidates,
                frame_number=101,
                source_time_s=1.01,
                table=identity_table,
                config=config,
            )
        )
    np.testing.assert_allclose(active_results[0].position_world, [0.75, 0.0, 1.0])
    np.testing.assert_allclose(active_results[0].position_world, active_results[1].position_world)


def test_association_gate_is_strict_and_timeout_emits_end_once(identity_table):
    config = BridgeConfig()
    tracker = BallTracker()
    started = tracker.update(
        [raw([0.8, 0.0, 1.0])],
        frame_number=100,
        source_time_s=1.0,
        table=identity_table,
        config=config,
        allocation_time_us=1_300_000,
    )
    exactly = tracker.update(
        [raw([1.15, 0.0, 1.0])],
        frame_number=101,
        source_time_s=1.01,
        table=identity_table,
        config=config,
    )
    outside_tracker = BallTracker()
    outside_started = outside_tracker.update(
        [raw([0.8, 0.0, 1.0])],
        frame_number=100,
        source_time_s=1.0,
        table=identity_table,
        config=config,
        allocation_time_us=1_300_100,
    )
    outside = outside_tracker.update(
        [raw([1.150001, 0.0, 1.0])],
        frame_number=101,
        source_time_s=1.01,
        table=identity_table,
        config=config,
    )
    ended = outside_tracker.update(
        [],
        frame_number=125,
        source_time_s=1.25,
        table=identity_table,
        config=config,
    )
    repeated = outside_tracker.update(
        [],
        frame_number=126,
        source_time_s=1.26,
        table=identity_table,
        config=config,
    )
    assert exactly.publish_valid and exactly.track_id == started.track_id
    assert outside.phase is bridge_module.BallTrackPhase.MISSING_GRACE
    assert ended.publish_end and ended.track_id == outside_started.track_id
    np.testing.assert_allclose(ended.position_world, outside_started.position_world)
    assert repeated.phase is bridge_module.BallTrackPhase.INACTIVE
    assert repeated.track_id == 0
    assert not repeated.publish_end


def test_source_time_fallback_uses_configured_frame_rate(identity_table):
    config = BridgeConfig(source_rate_hz=100.0)
    tracker = BallTracker()
    tracker.update(
        [raw([0.8, 0.0, 1.0], config)],
        frame_number=100,
        source_time_s=1.0,
        table=identity_table,
        config=config,
        allocation_time_us=1_400_000,
    )
    non_increasing = tracker.update(
        [raw([1.1, 0.0, 1.0], config)],
        frame_number=103,
        source_time_s=1.0,
        table=identity_table,
        config=config,
    )
    assert non_increasing.publish_valid
    np.testing.assert_allclose(tracker.velocity_world, [10.0, 0.0, 0.0])
    ended = tracker.update(
        [],
        frame_number=128,
        source_time_s=float("nan"),
        table=identity_table,
        config=config,
    )
    assert ended.publish_end


def test_allocator_is_positive_monotonic_and_fails_closed(identity_table):
    config = BridgeConfig()
    tracker = BallTracker()
    first = tracker.update(
        [raw([0.8, 0.0, 1.0])],
        frame_number=100,
        source_time_s=1.0,
        table=identity_table,
        config=config,
        allocation_time_us=0,
    )
    tracker.update(
        [],
        frame_number=125,
        source_time_s=1.25,
        table=identity_table,
        config=config,
    )
    second = tracker.update(
        [raw([0.9, 0.0, 1.0])],
        frame_number=126,
        source_time_s=1.26,
        table=identity_table,
        config=config,
        allocation_time_us=-100,
    )
    assert first.track_id == 1
    assert second.track_id == 2

    exhausted = BallTracker()
    exhausted.last_allocated_track_id = INT64_MAX
    failed = exhausted.update(
        [raw([0.8, 0.0, 1.0])],
        frame_number=100,
        source_time_s=1.0,
        table=identity_table,
        config=config,
        allocation_time_us=1,
    )
    failed_again = exhausted.update(
        [raw([0.9, 0.0, 1.0])],
        frame_number=101,
        source_time_s=1.01,
        table=identity_table,
        config=config,
        allocation_time_us=INT64_MAX,
    )
    assert failed.phase is bridge_module.BallTrackPhase.INACTIVE
    assert failed.track_id == 0
    assert not failed.publish_valid and not failed.publish_end
    assert failed_again.track_id == 0
    assert exhausted.last_allocated_track_id == INT64_MAX


def test_v2_defaults_and_track_arguments_are_strict():
    config = BridgeConfig()
    args = build_arg_parser().parse_args(["--pelvis-orientation-calib", "pelvis.json"])
    assert config.channel == "vicon_state_data_v2"
    assert config.source_rate_hz == 300.0
    assert config.ball_track_association_radius_m == 0.35
    assert config.ball_track_end_timeout_s == 0.25
    assert args.channel == "vicon_state_data_v2"
    assert args.print_hz == 1.0
    assert args.source_rate_hz == 300.0

    wrong_channel = build_arg_parser().parse_args(
        [
            "--pelvis-orientation-calib",
            "pelvis.json",
            "--channel",
            "vicon_state_data",
        ]
    )
    with pytest.raises(ValueError, match="--channel"):
        _validate_numeric_arguments(wrong_channel, _operation_mode(wrong_channel))

    for option, value in (
        ("--ball-track-association-radius-m", "nan"),
        ("--ball-track-end-timeout-s", "0"),
        ("--source-rate-hz", "inf"),
    ):
        invalid = build_arg_parser().parse_args(
            [
                "--pelvis-orientation-calib",
                "pelvis.json",
                option,
                value,
            ]
        )
        with pytest.raises(ValueError, match=option):
            _validate_numeric_arguments(invalid, _operation_mode(invalid))


def test_messages_assign_ids_and_ball_end_keeps_last_finite_position(identity_table):
    direct = bridge_module.make_message(
        "ball",
        [0.8, 0.0, 1.0],
        [0.0, 0.0, 0.0, 1.0],
        track_id=77,
        frame_number=100,
        source_time_s=1.0,
        valid=True,
        publish_time_us=123,
    )
    assert direct.track_id == 77

    calibration = PelvisOrientationCalibration(
        rotation_rigid_from_pelvis=np.eye(3),
        sample_count=30,
        source_duration_s=1.0,
        position_rms_m=0.0,
        angular_rms_deg=0.0,
    )
    bridge = ChingMuTableLcmBridge(
        config=BridgeConfig(),
        table_frame=identity_table,
        pelvis_orientation_calibration=calibration,
    )
    valid_messages = bridge.process_frame(
        SimpleNamespace(
            body_position_mm=[0.0, 0.0, 0.0],
            body_quaternion_xyzw=[0.0, 0.0, 0.0, 1.0],
            unlabeled_markers_mm=[raw([0.8, 0.0, 1.0])],
            frame_number=100,
            source_time_s=1.0,
        ),
        publish_time_us=1_000_000,
    )
    end_messages = bridge.process_frame(
        SimpleNamespace(
            body_position_mm=[0.0, 0.0, 0.0],
            body_quaternion_xyzw=[0.0, 0.0, 0.0, 1.0],
            unlabeled_markers_mm=[],
            frame_number=125,
            source_time_s=1.25,
        ),
        publish_time_us=1_000_001,
    )
    valid_by_name = {message.name: message for message in valid_messages}
    end_by_name = {message.name: message for message in end_messages}
    assert valid_by_name["table"].track_id == 0
    assert valid_by_name["ball"].track_id == 1_000_000
    assert end_by_name["ball"].track_id == 1_000_000
    assert end_by_name["ball"].valid == 0
    np.testing.assert_allclose(end_by_name["ball"].pos_vicon, valid_by_name["ball"].pos_vicon)


def test_base_invalid_frames_freeze_active_track_until_valid_resume(identity_table):
    calibration = PelvisOrientationCalibration(
        rotation_rigid_from_pelvis=np.eye(3),
        sample_count=30,
        source_duration_s=1.0,
        position_rms_m=0.0,
        angular_rms_deg=0.0,
    )
    bridge = ChingMuTableLcmBridge(
        config=BridgeConfig(),
        table_frame=identity_table,
        pelvis_orientation_calibration=calibration,
    )

    def frame(number, source_time_s, *, base_valid, markers):
        return SimpleNamespace(
            body_position_mm=[0.0, 0.0, 0.0] if base_valid else None,
            body_quaternion_xyzw=([0.0, 0.0, 0.0, 1.0] if base_valid else None),
            unlabeled_markers_mm=markers,
            frame_number=number,
            source_time_s=source_time_s,
        )

    started = bridge.process_frame(
        frame(100, 1.0, base_valid=True, markers=[raw([0.8, 0.0, 1.0])]),
        publish_time_us=1_600_000,
    )
    started_ball = {msg.name: msg for msg in started}["ball"]
    frozen_frame = bridge.ball_tracker.frame_number
    frozen_source_time = bridge.ball_tracker.source_time_s

    for number, source_time_s in ((200, 2.0), (500, 5.0), (1_000, 10.0)):
        invalid_messages = bridge.process_frame(
            frame(number, source_time_s, base_valid=False, markers=[]),
            publish_time_us=1_600_000 + number,
        )
        assert all(msg.name != "ball" for msg in invalid_messages)
        assert bridge.ball_tracker.phase is bridge_module.BallTrackPhase.ACTIVE
        assert bridge.ball_tracker.track_id == started_ball.track_id
        assert bridge.ball_tracker.frame_number == frozen_frame
        assert bridge.ball_tracker.source_time_s == frozen_source_time

    resumed = bridge.process_frame(
        frame(
            1_001,
            10.01,
            base_valid=True,
            markers=[raw([0.8, 0.0, 1.0])],
        ),
        publish_time_us=1_700_000,
    )
    resumed_ball = {msg.name: msg for msg in resumed}["ball"]
    assert resumed_ball.track_id == started_ball.track_id
    assert resumed_ball.valid == 1


def test_base_invalid_frames_freeze_missing_grace_until_valid_resume(identity_table):
    calibration = PelvisOrientationCalibration(
        rotation_rigid_from_pelvis=np.eye(3),
        sample_count=30,
        source_duration_s=1.0,
        position_rms_m=0.0,
        angular_rms_deg=0.0,
    )
    bridge = ChingMuTableLcmBridge(
        config=BridgeConfig(),
        table_frame=identity_table,
        pelvis_orientation_calibration=calibration,
    )

    def frame(number, source_time_s, *, base_valid, markers):
        return SimpleNamespace(
            body_position_mm=[0.0, 0.0, 0.0] if base_valid else None,
            body_quaternion_xyzw=([0.0, 0.0, 0.0, 1.0] if base_valid else None),
            unlabeled_markers_mm=markers,
            frame_number=number,
            source_time_s=source_time_s,
        )

    started = bridge.process_frame(
        frame(100, 1.0, base_valid=True, markers=[raw([0.8, 0.0, 1.0])]),
        publish_time_us=1_800_000,
    )
    started_ball = {msg.name: msg for msg in started}["ball"]
    bridge.process_frame(
        frame(102, 1.02, base_valid=True, markers=[]),
        publish_time_us=1_800_002,
    )
    assert bridge.ball_tracker.phase is bridge_module.BallTrackPhase.MISSING_GRACE
    frozen_frame = bridge.ball_tracker.frame_number
    frozen_source_time = bridge.ball_tracker.source_time_s

    for number, source_time_s in ((300, 3.0), (800, 8.0)):
        invalid_messages = bridge.process_frame(
            frame(number, source_time_s, base_valid=False, markers=[]),
            publish_time_us=1_800_000 + number,
        )
        assert all(msg.name != "ball" for msg in invalid_messages)
        assert bridge.ball_tracker.phase is bridge_module.BallTrackPhase.MISSING_GRACE
        assert bridge.ball_tracker.track_id == started_ball.track_id
        assert bridge.ball_tracker.frame_number == frozen_frame
        assert bridge.ball_tracker.source_time_s == frozen_source_time

    resumed = bridge.process_frame(
        frame(
            801,
            8.01,
            base_valid=True,
            markers=[raw([0.8, 0.0, 1.0])],
        ),
        publish_time_us=1_900_000,
    )
    resumed_ball = {msg.name: msg for msg in resumed}["ball"]
    assert resumed_ball.track_id == started_ball.track_id
    assert resumed_ball.valid == 1
