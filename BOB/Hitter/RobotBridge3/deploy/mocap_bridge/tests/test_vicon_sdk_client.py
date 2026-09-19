from __future__ import annotations

import json
import sys
import textwrap
import time
from pathlib import Path

import numpy as np

from deploy.mocap_bridge.vicon_sdk_client import ViconSdkClient


def _write_fake_helper(
    tmp_path: Path,
    frames: list[dict],
    sleep_s: float = 0.0,
    tail_sleep_s: float = 0.0,
) -> Path:
    helper = tmp_path / "fake_vicon_helper.py"
    helper.write_text(
        textwrap.dedent(
            f"""
            import json
            import time

            frames = json.loads({json.dumps(json.dumps(frames))})
            for frame in frames:
                print(json.dumps(frame), flush=True)
                time.sleep({sleep_s!r})
            time.sleep({tail_sleep_s!r})
            """
        ),
        encoding="utf-8",
    )
    return helper


def test_client_reads_vicon_helper_as_mocap_frame(tmp_path):
    helper = _write_fake_helper(
        tmp_path,
        [
            {
                "frame_number": 17,
                "source_time_s": 0.056,
                "body_position_mm": [10.0, 20.0, 30.0],
                "body_quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
                "body_markers_mm": {},
                "unlabeled_markers_mm": [[100.0, 200.0, 300.0]],
            }
        ],
    )
    client = ViconSdkClient(
        sys.executable,
        helper_args=[str(helper)],
    )
    try:
        client.start()
        frame = client.next_frame(timeout_s=1.0)
    finally:
        client.close()

    assert frame is not None
    assert frame.frame_number == 17
    assert frame.source_time_s == 0.056
    assert np.allclose(frame.body_position_mm, [10.0, 20.0, 30.0])
    assert np.allclose(frame.body_quaternion_xyzw, [0.0, 0.0, 0.0, 1.0])
    assert np.allclose(frame.unlabeled_markers_mm, [[100.0, 200.0, 300.0]])


def test_client_keeps_latest_frame_only(tmp_path):
    helper = _write_fake_helper(
        tmp_path,
        [
            {
                "frame_number": i,
                "source_time_s": i / 300.0,
                "body_position_mm": None,
                "body_quaternion_xyzw": None,
                "body_markers_mm": {},
                "unlabeled_markers_mm": [[float(i), 0.0, 1000.0]],
            }
            for i in range(5)
        ],
        tail_sleep_s=0.5,
    )
    client = ViconSdkClient(sys.executable, helper_args=[str(helper)])
    try:
        client.start()
        time.sleep(0.3)
        frame = client.next_frame(timeout_s=1.0)
    finally:
        client.close()

    assert frame is not None
    assert frame.frame_number == 4
    assert client.dropped_frame_count >= 1
