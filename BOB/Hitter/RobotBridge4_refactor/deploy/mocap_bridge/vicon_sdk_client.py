from __future__ import annotations

import json
import os
import subprocess
import threading
import time
from collections import deque
from pathlib import Path
from typing import Iterable, Optional, Sequence

import numpy as np

from deploy.mocap_bridge.mocap_types import MocapFrame

ROOT = Path(__file__).resolve().parents[2]


class ViconSdkClient:
    """Read Vicon frames from the C++ SDK helper as ChingMu-shaped MocapFrame.

    The helper is deliberately thin: it only exposes raw Vicon subject pose and
    unlabeled markers. Table/world calibration, ball filtering, LCM publishing,
    and RobotBridge2 naming stay in the shared Python bridge path.
    """

    def __init__(
        self,
        helper_path: os.PathLike[str] | str = (ROOT / "deploy" / "mocap_bridge" / "bin" / "vicon_frame_stream"),
        *,
        server_ip: str = "localhost:801",
        body_name: str = "G2Pelvis",
        source_rate_hz: float = 300.0,
        helper_args: Optional[Sequence[str]] = None,
        env: Optional[dict[str, str]] = None,
    ):
        self.helper_path = Path(helper_path)
        self.server_ip = server_ip
        self.body_name = body_name
        self.source_rate_hz = float(source_rate_hz)
        self.helper_args = list(helper_args) if helper_args is not None else None
        self.env = dict(env) if env is not None else None

        self._process: Optional[subprocess.Popen[str]] = None
        self._reader_thread: Optional[threading.Thread] = None
        self._stderr_thread: Optional[threading.Thread] = None
        self._condition = threading.Condition()
        self._latest_frames: deque[MocapFrame] = deque(maxlen=1)
        self._closed = False
        self._last_stderr = ""
        self._last_error: Optional[BaseException] = None
        self._dropped_frame_count = 0

    @property
    def dropped_frame_count(self) -> int:
        with self._condition:
            return self._dropped_frame_count

    @property
    def last_stderr(self) -> str:
        with self._condition:
            return self._last_stderr

    def start(self) -> None:
        if self._process is not None:
            raise RuntimeError("ViconSdkClient is already started")
        command = self._build_command()
        self._process = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
            env=self._build_env(),
        )
        self._reader_thread = threading.Thread(
            target=self._reader_loop,
            name="vicon-sdk-client-stdout",
            daemon=True,
        )
        self._stderr_thread = threading.Thread(
            target=self._stderr_loop,
            name="vicon-sdk-client-stderr",
            daemon=True,
        )
        self._reader_thread.start()
        self._stderr_thread.start()

    def next_frame(self, timeout_s: float = 0.1) -> Optional[MocapFrame]:
        deadline = time.monotonic() + timeout_s
        with self._condition:
            while not self._latest_frames and not self._closed:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return None
                self._condition.wait(remaining)
            if self._latest_frames:
                return self._latest_frames.popleft()
            if self._last_error is not None:
                raise RuntimeError(f"Vicon frame helper stopped: {self._last_error}") from self._last_error
            return None

    def close(self) -> None:
        process = self._process
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=2.0)
        with self._condition:
            self._closed = True
            self._condition.notify_all()
        for thread in (self._reader_thread, self._stderr_thread):
            if thread is not None:
                thread.join(timeout=1.0)

    def _build_command(self) -> list[str]:
        if self.helper_args is not None:
            return [str(self.helper_path), *self.helper_args]
        return [
            str(self.helper_path),
            "--host",
            self.server_ip,
            "--base-subject",
            self.body_name,
            "--source-rate-hz",
            str(self.source_rate_hz),
        ]

    def _build_env(self) -> dict[str, str]:
        env = os.environ.copy()
        if self.env:
            env.update(self.env)
        sdk_lib_dir = ROOT / "vicon_datastream_sdk" / "linux64" / "Linux64"
        if sdk_lib_dir.exists():
            old = env.get("LD_LIBRARY_PATH", "")
            env["LD_LIBRARY_PATH"] = f"{sdk_lib_dir}:{old}" if old else str(sdk_lib_dir)
        return env

    def _reader_loop(self) -> None:
        assert self._process is not None
        assert self._process.stdout is not None
        try:
            for raw_line in self._process.stdout:
                line = raw_line.strip()
                if not line:
                    continue
                payload = json.loads(line)
                frame = self._frame_from_payload(payload)
                with self._condition:
                    if len(self._latest_frames) == self._latest_frames.maxlen:
                        self._dropped_frame_count += 1
                    self._latest_frames.append(frame)
                    self._condition.notify_all()
        except BaseException as exc:  # keep reader failure visible to caller
            with self._condition:
                self._last_error = exc
                self._closed = True
                self._condition.notify_all()
            return
        with self._condition:
            self._closed = True
            self._condition.notify_all()

    def _stderr_loop(self) -> None:
        assert self._process is not None
        assert self._process.stderr is not None
        for raw_line in self._process.stderr:
            line = raw_line.rstrip()
            if not line:
                continue
            with self._condition:
                self._last_stderr = line

    def _frame_from_payload(self, payload: dict) -> MocapFrame:
        body_position = _optional_vector(payload.get("body_position_mm"), 3)
        body_quaternion = _optional_vector(
            payload.get("body_quaternion_xyzw"),
            4,
        )
        body_markers = {
            int(marker_id): _required_vector(position, 3)
            for marker_id, position in payload.get("body_markers_mm", {}).items()
        }
        unlabeled = _marker_array(payload.get("unlabeled_markers_mm", []))
        frame_number = int(payload["frame_number"])
        source_time_s = float(payload["source_time_s"])
        return MocapFrame(
            frame_number=frame_number,
            source_time_s=source_time_s,
            body_position_mm=body_position,
            body_quaternion_xyzw=body_quaternion,
            body_markers_mm=body_markers,
            unlabeled_markers_mm=unlabeled,
            body_pose_source_time_s=(
                source_time_s if body_position is not None and body_quaternion is not None else None
            ),
        )


def _optional_vector(value: object, size: int) -> Optional[np.ndarray]:
    if value is None:
        return None
    return _required_vector(value, size)


def _required_vector(value: object, size: int) -> np.ndarray:
    array = np.asarray(value, dtype=np.float64)
    if array.shape != (size,):
        raise ValueError(f"expected vector shape {(size,)}, got {array.shape}")
    if not np.all(np.isfinite(array)):
        raise ValueError("vector contains non-finite values")
    return array


def _marker_array(value: Iterable[object]) -> np.ndarray:
    array = np.asarray(list(value), dtype=np.float64)
    if array.size == 0:
        return np.empty((0, 3), dtype=np.float64)
    if array.ndim != 2 or array.shape[1] != 3:
        raise ValueError(f"expected unlabeled marker array with shape (N, 3), got {array.shape}")
    if not np.all(np.isfinite(array)):
        raise ValueError("unlabeled marker array contains non-finite values")
    return array


__all__ = ["ViconSdkClient"]
