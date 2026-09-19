from __future__ import annotations

import numpy as np
from deploy.mocap_bridge.chingmu_sdk_client import MocapFrame
from deploy.mocap_bridge.chingmu_table_lcm_bridge import (
    BridgeConfig,
    PelvisOrientationCalibration,
    TableFrame,
    _config_from_args,
)
from deploy.mocap_bridge.vicon_table_lcm_bridge import (
    ViconTableLcmBridge,
    build_arg_parser,
)


def _identity_table() -> TableFrame:
    return TableFrame(
        center_raw_mm=np.array([1365.369, 0.0, 760.0]),
        x_axis_raw=np.array([1.0, 0.0, 0.0]),
        y_axis_raw=np.array([0.0, 1.0, 0.0]),
        z_axis_raw=np.array([0.0, 0.0, 1.0]),
        corners_raw_mm=np.array(
            [
                [0.0, -756.2255, 760.0],
                [0.0, 756.2255, 760.0],
                [2730.738, -756.2255, 760.0],
                [2730.738, 756.2255, 760.0],
            ],
            dtype=np.float64,
        ),
        rectangle_score=0.0,
    )


def _identity_pelvis_calibration() -> PelvisOrientationCalibration:
    return PelvisOrientationCalibration(
        rotation_rigid_from_pelvis=np.eye(3),
        sample_count=10,
        source_duration_s=1.0,
        position_rms_m=0.0,
        angular_rms_deg=0.0,
        translation_rigid_origin_to_pelvis_origin_pelvis_m=np.zeros(3),
    )


def test_parser_defaults_match_robotbridge_contract():
    args = build_arg_parser().parse_args(
        [
            "--table-calib",
            "table.json",
            "--pelvis-orientation-calib",
            "pelvis.json",
        ]
    )

    assert args.base_subject == "G2Pelvis"
    assert args.channel == "vicon_state_data_v2"
    assert args.host == "localhost:801"


def test_parser_exposes_v2_ball_track_defaults_to_shared_config():
    args = build_arg_parser().parse_args(
        [
            "--table-calib",
            "table.json",
            "--pelvis-orientation-calib",
            "pelvis.json",
        ]
    )

    config = _config_from_args(args)

    assert config.channel == "vicon_state_data_v2"
    assert config.ball_track_association_radius_m == 0.35
    assert config.ball_track_end_timeout_s == 0.25


def test_parser_accepts_tracker_name_alias():
    args = build_arg_parser().parse_args(
        [
            "--tracker-name",
            "G1Pelvis",
            "--table-calib",
            "table.json",
            "--pelvis-orientation-calib",
            "pelvis.json",
        ]
    )

    assert args.tracker_name == "G1Pelvis"
    assert args.base_subject == "G2Pelvis"


def test_vicon_bridge_processes_unlabeled_marker_like_chingmu():
    config = BridgeConfig(base_subject="G2Pelvis")
    bridge = ViconTableLcmBridge(
        config=config,
        table_frame=_identity_table(),
        pelvis_orientation_calibration=_identity_pelvis_calibration(),
    )
    frame = MocapFrame(
        frame_number=12,
        source_time_s=0.04,
        body_position_mm=np.array([1365.369, 0.0, 790.0]),
        body_quaternion_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
        body_markers_mm={},
        unlabeled_markers_mm=np.array([[100.0, 0.0, 900.0]]),
        body_pose_source_time_s=0.04,
    )

    messages = bridge.process_frame(frame)
    by_name = {message.name: message for message in messages}

    assert "G2Pelvis" in by_name
    assert "ball" in by_name
    assert "table" in by_name
    assert by_name["ball"].valid == 1
    assert np.allclose(by_name["ball"].pos_vicon, [0.1, 0.0, 0.9])
    assert np.allclose(by_name["G2Pelvis"].pos_vicon, [1.365369, 0.0, 0.79])
