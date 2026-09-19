import math
import csv
import io

import pytest

from deploy.mocap_bridge import monitor_vicon_lcm as monitor
from unitree_sdk2.lcm_types.transformation_t import transformation_t


def ball_message(*, track_id=1):
    return message("ball", track_id)


def message(name, track_id):
    result = transformation_t()
    result.name = name
    result.track_id = track_id
    result.valid = 1
    result.occluded = 0
    result.vicon_frame_number = 123
    result.vicon_time_s = 4.5
    result.publish_time_us = 6_000_000
    result.pos_vicon = [0.8, -0.1, 1.0]
    result.quat_vicon = [0.0, 0.0, 0.0, 1.0]
    return result


def test_monitor_defaults_to_v2_and_exports_track_id():
    args = monitor.build_arg_parser().parse_args([])
    assert args.channel == "vicon_state_data_v2"
    assert args.print_hz == 1.0
    row = monitor.message_csv_row(ball_message(track_id=77), received_monotonic_s=3.0)
    assert row["track_id"] == 77
    assert list(row) == [
        "track_id",
        "valid",
        "occluded",
        "source_frame",
        "source_time_s",
        "publish_time_us",
        "x",
        "y",
        "z",
        "received_monotonic_s",
    ]
    assert row == {
        "track_id": 77,
        "valid": 1,
        "occluded": 0,
        "source_frame": 123,
        "source_time_s": 4.5,
        "publish_time_us": 6_000_000,
        "x": 0.8,
        "y": -0.1,
        "z": 1.0,
        "received_monotonic_s": 3.0,
    }


@pytest.mark.parametrize("name,track_id", [("ball", 0), ("table", 7), ("G2Pelvis", 7)])
def test_monitor_rejects_invalid_subject_id_contract(name, track_id):
    assert monitor.message_contract_error(message(name, track_id)) is not None


@pytest.mark.parametrize("field", ["pos_vicon", "quat_vicon"])
def test_monitor_rejects_non_finite_pose(field):
    malformed = ball_message()
    values = list(getattr(malformed, field))
    values[0] = math.nan
    setattr(malformed, field, values)
    assert monitor.message_contract_error(malformed) == "pose_not_finite"


def test_monitor_accepts_valid_ball_and_non_ball_ids():
    assert monitor.message_contract_error(ball_message(track_id=9)) is None
    assert monitor.message_contract_error(message("table", 0)) is None
    assert monitor.message_contract_error(message("G2Pelvis", 0)) is None


def monitor_callback_with_csv():
    output = io.StringIO()
    writer = csv.DictWriter(output, fieldnames=monitor.CSV_FIELDS)
    writer.writeheader()
    return monitor.MonitorCallback(csv_writer=writer), output


def test_callback_survives_decode_error_and_accepts_following_packet(capsys):
    callback, output = monitor_callback_with_csv()
    callback("vicon_state_data_v2", b"not-a-transformation")
    accepted = ball_message(track_id=19)
    callback("vicon_state_data_v2", accepted.encode())

    assert callback.received_count == 2
    assert callback.accepted_count == 1
    assert callback.contract_error_count == 1
    assert callback.last_by_name["ball"].track_id == 19
    rows = list(csv.DictReader(io.StringIO(output.getvalue())))
    assert len(rows) == 1
    assert rows[0]["track_id"] == "19"
    assert capsys.readouterr().out.splitlines() == [
        "contract_error=decode_error name=<decode>",
        "contract_error=none name=<decode>",
    ]


def test_contract_invalid_packet_does_not_update_last_or_csv():
    callback, output = monitor_callback_with_csv()
    invalid = ball_message(track_id=0)
    callback("vicon_state_data_v2", invalid.encode())

    assert callback.accepted_count == 0
    assert "ball" not in callback.last_by_name
    assert list(csv.DictReader(io.StringIO(output.getvalue()))) == []

    accepted = ball_message(track_id=23)
    callback("vicon_state_data_v2", accepted.encode())
    assert callback.accepted_count == 1
    assert callback.last_by_name["ball"].track_id == 23
    rows = list(csv.DictReader(io.StringIO(output.getvalue())))
    assert [row["track_id"] for row in rows] == ["23"]


def test_subject_error_transitions_ignore_interleaved_healthy_subjects(capsys):
    callback, _output = monitor_callback_with_csv()
    bad_id = ball_message(track_id=0)
    healthy_table = message("table", 0)
    bad_pose = ball_message(track_id=11)
    bad_pose.pos_vicon = [math.nan, -0.1, 1.0]
    recovered = ball_message(track_id=12)

    for packet in (
        bad_id,
        healthy_table,
        bad_id,
        healthy_table,
        bad_pose,
        healthy_table,
        bad_pose,
        recovered,
    ):
        callback("vicon_state_data_v2", packet.encode())

    assert capsys.readouterr().out.splitlines() == [
        "contract_error=ball_track_id_not_positive name=ball",
        "contract_error=pose_not_finite name=ball",
        "contract_error=none name=ball",
    ]
    assert callback.last_by_name["ball"].track_id == 12
    assert callback.last_by_name["table"].track_id == 0


def test_callback_field_extraction_error_is_fail_closed_and_recovers(capsys):
    malformed = ball_message(track_id=31)
    malformed.valid = "not-an-integer"
    decoded = iter([malformed, ball_message(track_id=32)])
    callback, output = monitor_callback_with_csv()
    callback.decoder = lambda _data: next(decoded)

    callback("vicon_state_data_v2", b"first")
    assert callback.accepted_count == 0
    assert "ball" not in callback.last_by_name
    assert list(csv.DictReader(io.StringIO(output.getvalue()))) == []

    callback("vicon_state_data_v2", b"second")
    assert callback.accepted_count == 1
    assert callback.last_by_name["ball"].track_id == 32
    rows = list(csv.DictReader(io.StringIO(output.getvalue())))
    assert [row["track_id"] for row in rows] == ["32"]
    assert capsys.readouterr().out.splitlines() == [
        "contract_error=field_error name=ball",
        "contract_error=none name=ball",
    ]


def test_callback_track_id_overflow_is_fail_closed_and_recovers(capsys):
    malformed = ball_message(track_id=41)
    malformed.track_id = float("inf")
    decoded = iter([malformed, ball_message(track_id=42)])
    callback, output = monitor_callback_with_csv()
    callback.decoder = lambda _data: next(decoded)

    callback("vicon_state_data_v2", b"first")
    callback("vicon_state_data_v2", b"second")

    assert callback.accepted_count == 1
    assert callback.contract_error_count == 1
    assert callback.last_by_name["ball"].track_id == 42
    rows = list(csv.DictReader(io.StringIO(output.getvalue())))
    assert [row["track_id"] for row in rows] == ["42"]
    assert capsys.readouterr().out.splitlines() == [
        "contract_error=track_id_not_integer name=ball",
        "contract_error=none name=ball",
    ]
