import subprocess
from pathlib import Path

import pytest

from unitree_sdk2.lcm_types.transformation_t import transformation_t


def test_new_messages_default_track_id_to_zero():
    """A missing v2 track field would make the generated binding unusable."""
    assert transformation_t().track_id == 0


def test_python_round_trip_preserves_track_id():
    """Dropping or misordering track_id on the v2 wire must be observable."""
    msg = transformation_t()
    msg.name = "ball"
    msg.vicon_frame_number = 41
    msg.vicon_time_s = 0.125
    msg.publish_time_us = 1_700_000_000_000_000
    msg.track_id = 9001
    msg.valid, msg.occluded = 1, 0
    msg.pos_vicon = [0.7, -0.2, 1.0]
    msg.quat_vicon = [0.0, 0.0, 0.0, 1.0]

    decoded = transformation_t.decode(msg.encode())

    assert (
        decoded.name,
        decoded.vicon_frame_number,
        decoded.vicon_time_s,
        decoded.publish_time_us,
        decoded.track_id,
        decoded.valid,
        decoded.occluded,
        decoded.pos_vicon,
        decoded.quat_vicon,
    ) == (
        "ball",
        41,
        0.125,
        1_700_000_000_000_000,
        9001,
        1,
        0,
        (0.7, -0.2, 1.0),
        (0.0, 0.0, 0.0, 1.0),
    )


def test_v1_payload_is_rejected_by_the_v2_fingerprint():
    """A v1 message must fail on its known fingerprint, not a truncated body."""
    v1_payload_bytes = bytes.fromhex("71f936e3b20f1df5") + (b"\0" * 96)

    assert transformation_t._get_packed_fingerprint() != v1_payload_bytes[:8]
    with pytest.raises(ValueError, match="Decode error"):
        transformation_t.decode(v1_payload_bytes)


def test_python_decodes_fixed_cpp_v2_wire_fixture():
    """The C++ producer's fixed v2 payload must decode in the Python binding."""
    binary = Path(__file__).parents[1] / ".build-v2" / "test_transformation_t_v2"
    if not binary.is_file():
        pytest.skip("C++ v2 fixture is built by build_v2_mocap.sh")

    completed = subprocess.run([str(binary), "--emit-hex"], check=True, capture_output=True, text=True)
    decoded = transformation_t.decode(bytes.fromhex(completed.stdout.strip()))

    assert decoded.track_id == 9001
