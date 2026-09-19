from __future__ import annotations

import os
from pathlib import Path
import subprocess
import sys
import textwrap

ROOT = Path(__file__).resolve().parents[2]


def _pythonpath() -> str:
    paths = [str(ROOT / "deploy"), str(ROOT)]
    inherited = os.environ.get("PYTHONPATH")
    if inherited:
        paths.append(inherited)
    return os.pathsep.join(paths)


def _run_isolation_check(source: str) -> subprocess.CompletedProcess[str]:
    env = os.environ.copy()
    env["PYTHONPATH"] = _pythonpath()
    return subprocess.run(
        [sys.executable, "-c", textwrap.dedent(source)],
        cwd=ROOT,
        env=env,
        capture_output=True,
        text=True,
        check=False,
    )


def test_core_serialization_and_realtime_import_without_diagnostics() -> None:
    result = _run_isolation_check("""
        import sys

        from utils.hitter_serialization import freeze_json_value, to_builtin_json
        import utils.hitter_realtime

        assert freeze_json_value({"value": [1]})["value"] == (1,)
        assert to_builtin_json({"value": (1,)}) == {"value": [1]}
        assert not any(name == "diagnostics" or name.startswith("diagnostics.") for name in sys.modules)
        """)

    assert result.returncode == 0, result.stderr


def test_shared_frame_and_vicon_import_without_chingmu_client() -> None:
    result = _run_isolation_check("""
        import sys
        from dataclasses import FrozenInstanceError

        import numpy as np

        from deploy.mocap_bridge.mocap_types import MocapFrame
        from deploy.mocap_bridge.vicon_sdk_client import ViconSdkClient

        assert ViconSdkClient is not None
        assert "deploy.mocap_bridge.chingmu_sdk_client" not in sys.modules
        frame = MocapFrame(
            frame_number=1,
            source_time_s=0.01,
            body_position_mm=None,
            body_quaternion_xyzw=None,
            body_markers_mm={},
            unlabeled_markers_mm=np.empty((0, 3)),
        )
        assert frame.body_pose_source_time_s is None
        try:
            frame.frame_number = 2
        except FrozenInstanceError:
            pass
        else:
            raise AssertionError("MocapFrame must remain frozen")
        """)

    assert result.returncode == 0, result.stderr
