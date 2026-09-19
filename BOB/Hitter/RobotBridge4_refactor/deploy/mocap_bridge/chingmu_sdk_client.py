from __future__ import annotations

import ctypes
import threading
import time
from collections import deque
from typing import Callable, Deque, Dict, List, Optional, Tuple

import numpy as np

from deploy.mocap_bridge.mocap_types import MocapFrame


class Timeval(ctypes.Structure):
    _fields_ = [
        ("tv_sec", ctypes.c_long),
        ("tv_usec", ctypes.c_long),
    ]


class TrackerReport(ctypes.Structure):
    _fields_ = [
        ("msg_time", Timeval),
        ("sensor", ctypes.c_int),
        ("frameCounter", ctypes.c_uint),
        ("pos", ctypes.c_double * 3),
        ("quat", ctypes.c_double * 4),
    ]


class HierarchyReport(ctypes.Structure):
    _fields_ = [
        ("msg_time", Timeval),
        ("sensor", ctypes.c_int),
        ("parent", ctypes.c_int),
        ("name", ctypes.c_char * 127),
    ]


class EndHierarchyReport(ctypes.Structure):
    _fields_ = [
        ("msg_time", Timeval),
        ("retarget_flag", ctypes.c_int),
    ]


TrackerCallback = ctypes.CFUNCTYPE(None, ctypes.c_void_p, TrackerReport)
HierarchyCallback = ctypes.CFUNCTYPE(None, ctypes.c_void_p, HierarchyReport)
EndHierarchyCallback = ctypes.CFUNCTYPE(None, ctypes.c_void_p, EndHierarchyReport)


class ChingMuSdkClient:
    """Receive G2Pelvis markers and unlabeled markers from one MCAvatar stream."""

    BODY_MARKER_SENSOR_BASE = 7800
    BODY_MARKER_SENSOR_STRIDE = 50

    def __init__(
        self,
        sdk_library,
        *,
        server_ip: str,
        body_name: str,
        body_pose_id: Optional[int] = None,
        poll_body_pose: bool = True,
        body_pose_poll_hz: float = 60.0,
        library=None,
        sleep_fn: Callable[[float], None] = time.sleep,
        monotonic_fn: Callable[[], float] = time.monotonic,
        discovery_delay_s: float = 1.0,
    ):
        self.server_ip = str(server_ip)
        self.body_name = str(body_name)
        self.server = f"MCAvatar@{self.server_ip}".encode("ascii")
        self.body_pose_server = self._server_for_subject(self.body_name)
        self.sleep_fn = sleep_fn
        self.monotonic_fn = monotonic_fn
        self.discovery_delay_s = float(discovery_delay_s)
        self.lib = library if library is not None else ctypes.CDLL(str(sdk_library))
        self._configured_body_pose_id = None if body_pose_id is None else int(body_pose_id)
        self.poll_body_pose = bool(poll_body_pose)
        self.body_pose_poll_hz = float(body_pose_poll_hz)
        if not np.isfinite(self.body_pose_poll_hz) or self.body_pose_poll_hz <= 0.0:
            raise ValueError("body_pose_poll_hz must be finite and positive")

        self.body_id: Optional[int] = None
        self.body_pose_id: Optional[int] = None
        self._body_pose_servers: Tuple[bytes, ...] = ()
        self._body_pose_lock = threading.Lock()
        self._latest_polled_body_pose: Optional[Tuple[np.ndarray, np.ndarray, float]] = None
        self._body_pose_poll_stop_event = threading.Event()
        self._body_pose_poll_thread: Optional[threading.Thread] = None
        self.body_marker_sensor_range = range(0, 0)
        self._hierarchy_sensor_ids = set()
        self._rigid_body_ids = set()
        self._rigid_body_names: Dict[str, int] = {}
        self._hierarchy_complete = False
        self._condition = threading.Condition()
        # A mocap bridge must stay real-time: if Python briefly falls behind
        # the 360 Hz callback, discard stale completed frames and keep latest.
        self._ready: Deque[MocapFrame] = deque(maxlen=1)
        self.dropped_frame_count = 0
        self._current_frame: Optional[int] = None
        self._current_time_s = 0.0
        self._current_body_position: Optional[np.ndarray] = None
        self._current_body_quaternion: Optional[np.ndarray] = None
        self._current_body_pose_source_time_s: Optional[float] = None
        self._current_body_markers: Dict[int, np.ndarray] = {}
        self._current_pending_unlabeled: List[Tuple[int, np.ndarray]] = []
        self._started = False
        self._hierarchy_registered = False
        self._end_hierarchy_registered = False
        self._tracker_registered = False

        self._hierarchy_callback = HierarchyCallback(self._on_hierarchy)
        self._end_hierarchy_callback = EndHierarchyCallback(self._on_end_hierarchy)
        self._tracker_callback = TrackerCallback(self._on_tracker)
        self._bind()

    def _bind(self) -> None:
        self.lib.CMVrpnStartExtern.argtypes = []
        self.lib.CMVrpnStartExtern.restype = None
        self.lib.CMVrpnQuitExtern.argtypes = []
        self.lib.CMVrpnQuitExtern.restype = None
        self.lib.CMPluginConnectServer.argtypes = [ctypes.c_char_p]
        self.lib.CMPluginConnectServer.restype = ctypes.c_bool
        self.lib.CMPluginRegisterUpdateHierarchy.argtypes = [
            ctypes.c_char_p,
            ctypes.c_void_p,
            HierarchyCallback,
        ]
        self.lib.CMPluginRegisterUpdateHierarchy.restype = ctypes.c_bool
        self.lib.CMPluginRegisterEndHierarchy.argtypes = [
            ctypes.c_char_p,
            ctypes.c_void_p,
            EndHierarchyCallback,
        ]
        self.lib.CMPluginRegisterEndHierarchy.restype = ctypes.c_bool
        self.lib.CMPluginUnRegisterUpdateHierarchy.argtypes = [
            ctypes.c_char_p,
            ctypes.c_void_p,
            HierarchyCallback,
        ]
        self.lib.CMPluginUnRegisterUpdateHierarchy.restype = ctypes.c_bool
        self.lib.CMPluginRegisterTrackerData.argtypes = [
            ctypes.c_char_p,
            ctypes.c_void_p,
            TrackerCallback,
        ]
        self.lib.CMPluginRegisterTrackerData.restype = ctypes.c_bool
        self.lib.CMPluginUnRegisterTrackerData.argtypes = [
            ctypes.c_char_p,
            ctypes.c_void_p,
            TrackerCallback,
        ]
        self.lib.CMPluginUnRegisterTrackerData.restype = ctypes.c_bool
        self.lib.CMTrackerExternTC.argtypes = [
            ctypes.c_char_p,
            ctypes.c_int,
            ctypes.POINTER(ctypes.c_int),
            ctypes.POINTER(ctypes.c_double),
            ctypes.POINTER(ctypes.c_double),
            ctypes.POINTER(Timeval),
        ]
        self.lib.CMTrackerExternTC.restype = ctypes.c_bool

    @staticmethod
    def _source_time(report: TrackerReport) -> float:
        return ChingMuSdkClient._timeval_source_time(report.msg_time)

    @staticmethod
    def _timeval_source_time(timestamp: Timeval) -> float:
        return float(timestamp.tv_sec) + 1.0e-6 * float(timestamp.tv_usec)

    def _server_for_subject(self, subject_name: str) -> bytes:
        if "@" in subject_name:
            return subject_name.encode("ascii")
        return f"{subject_name}@{self.server_ip}".encode("ascii")

    def _on_hierarchy(self, _userdata, report: HierarchyReport) -> None:
        name = bytes(report.name).split(b"\0", 1)[0].decode("utf-8", errors="replace")
        sensor = int(report.sensor)
        with self._condition:
            self._hierarchy_sensor_ids.add(sensor)
            if int(report.parent) < 0 and not name.startswith("Skeleton_"):
                self._rigid_body_ids.add(sensor)
                self._rigid_body_names[name] = sensor
            self._condition.notify_all()

    def _on_end_hierarchy(self, _userdata, _report: EndHierarchyReport) -> None:
        with self._condition:
            self._hierarchy_complete = True
            self._condition.notify_all()

    def _is_labelled_rigid_body_marker_locked(self, sensor: int) -> bool:
        for body_id in self._rigid_body_ids:
            marker_start = self.BODY_MARKER_SENSOR_BASE + self.BODY_MARKER_SENSOR_STRIDE * body_id
            if marker_start <= sensor < marker_start + self.BODY_MARKER_SENSOR_STRIDE:
                return True
        return False

    def _on_tracker(self, _userdata, report: TrackerReport) -> None:
        frame_number = int(report.frameCounter)
        sensor = int(report.sensor)
        with self._condition:
            if self._current_frame is None:
                self._begin_frame_locked(frame_number, self._source_time(report))
            elif frame_number < self._current_frame:
                return
            elif frame_number != self._current_frame:
                self._finish_frame_locked()
                self._begin_frame_locked(frame_number, self._source_time(report))

            position = np.asarray(tuple(report.pos), dtype=np.float64)
            if self.body_id is not None and sensor == self.body_id:
                self._current_body_position = position.copy()
                self._current_body_quaternion = np.asarray(tuple(report.quat), dtype=np.float64)
                self._current_body_pose_source_time_s = self._source_time(report)
            elif sensor in self.body_marker_sensor_range:
                self._current_body_markers[sensor] = position
            elif sensor not in self._hierarchy_sensor_ids and not self._is_labelled_rigid_body_marker_locked(sensor):
                self._current_pending_unlabeled.append((sensor, position))

    def _begin_frame_locked(self, frame_number: int, source_time_s: float) -> None:
        self._current_frame = frame_number
        self._current_time_s = source_time_s
        self._current_body_position = None
        self._current_body_quaternion = None
        self._current_body_pose_source_time_s = None
        self._current_body_markers = {}
        self._current_pending_unlabeled = []

    def _finish_frame_locked(self) -> None:
        if self._current_frame is None:
            return
        if self._ready.maxlen is not None and len(self._ready) >= self._ready.maxlen:
            self.dropped_frame_count += 1
        self._ready.append(
            MocapFrame(
                frame_number=self._current_frame,
                source_time_s=self._current_time_s,
                body_position_mm=(None if self._current_body_position is None else self._current_body_position.copy()),
                body_quaternion_xyzw=(
                    None if self._current_body_quaternion is None else self._current_body_quaternion.copy()
                ),
                body_markers_mm={sensor: position.copy() for sensor, position in self._current_body_markers.items()},
                unlabeled_markers_mm=np.asarray(
                    [
                        position
                        for sensor, position in self._current_pending_unlabeled
                        if sensor not in self._hierarchy_sensor_ids
                        and not self._is_labelled_rigid_body_marker_locked(sensor)
                    ],
                    dtype=np.float64,
                ).reshape(-1, 3),
                body_pose_source_time_s=(self._current_body_pose_source_time_s),
            )
        )
        self._condition.notify_all()

    def _try_connect(self, server: bytes, deadline: float, retry_interval_s: float) -> bool:
        while not self.lib.CMPluginConnectServer(server):
            if self.monotonic_fn() >= deadline:
                return False
            self.sleep_fn(retry_interval_s)
        return True

    def _connect(self, server: bytes, deadline: float, retry_interval_s: float) -> None:
        if not self._try_connect(server, deadline, retry_interval_s):
            raise TimeoutError(f"could not connect to ChingMu server {server.decode('ascii')}")

    def _reset_stream_state_locked(self) -> None:
        self._hierarchy_sensor_ids.clear()
        self._rigid_body_ids.clear()
        self._rigid_body_names.clear()
        self._hierarchy_complete = False
        self.body_id = None
        self.body_pose_id = None
        self._body_pose_servers = ()
        with self._body_pose_lock:
            self._latest_polled_body_pose = None
        self.body_marker_sensor_range = range(0, 0)
        self._current_frame = None
        self._current_time_s = 0.0
        self._current_body_position = None
        self._current_body_quaternion = None
        self._current_body_pose_source_time_s = None
        self._current_body_markers = {}
        self._current_pending_unlabeled = []
        self._ready.clear()

    def start(
        self,
        *,
        connect_timeout_s: float = 5.0,
        resolve_timeout_s: float = 5.0,
        retry_interval_s: float = 0.02,
    ) -> None:
        if self._started:
            return
        with self._condition:
            self._reset_stream_state_locked()
        self.lib.CMVrpnStartExtern()
        self._started = True
        try:
            # The vendor library builds its server map asynchronously and can
            # crash if ConnectServer is called before discovery has started.
            self.sleep_fn(self.discovery_delay_s)
            self._connect(
                self.server,
                self.monotonic_fn() + float(connect_timeout_s),
                retry_interval_s,
            )
            if not self.lib.CMPluginRegisterUpdateHierarchy(self.server, None, self._hierarchy_callback):
                raise RuntimeError("could not register ChingMu hierarchy callback")
            self._hierarchy_registered = True
            if not self.lib.CMPluginRegisterEndHierarchy(self.server, None, self._end_hierarchy_callback):
                raise RuntimeError("could not register ChingMu end-hierarchy callback")
            self._end_hierarchy_registered = True

            deadline = self.monotonic_fn() + float(resolve_timeout_s)
            while self.body_name not in self._rigid_body_names or not self._hierarchy_complete:
                if self.monotonic_fn() >= deadline:
                    known = ", ".join(sorted(self._rigid_body_names))
                    raise TimeoutError(
                        f"body {self.body_name!r} not resolved before ChingMu "
                        f"hierarchy completed; known={known}, "
                        f"hierarchy_complete={self._hierarchy_complete}"
                    )
                self.sleep_fn(retry_interval_s)

            self.body_id = self._rigid_body_names[self.body_name]
            self.body_pose_id = self.body_id if self._configured_body_pose_id is None else self._configured_body_pose_id
            body_pose_servers = [self.body_pose_server]
            if self.server not in body_pose_servers:
                body_pose_servers.append(self.server)
            self._body_pose_servers = tuple(body_pose_servers)
            marker_start = self.BODY_MARKER_SENSOR_BASE + self.BODY_MARKER_SENSOR_STRIDE * self.body_id
            self.body_marker_sensor_range = range(marker_start, marker_start + self.BODY_MARKER_SENSOR_STRIDE)
            if not self.lib.CMPluginRegisterTrackerData(self.server, None, self._tracker_callback):
                raise RuntimeError("could not register ChingMu tracker callback")
            self._tracker_registered = True
            self._start_body_pose_poll_thread()
        except Exception:
            self.close()
            raise

    def _start_body_pose_poll_thread(self) -> None:
        if not self.poll_body_pose or self.body_pose_id is None:
            return
        if self._body_pose_poll_thread is not None and self._body_pose_poll_thread.is_alive():
            return
        self._body_pose_poll_stop_event.clear()
        self._body_pose_poll_thread = threading.Thread(
            target=self._body_pose_poll_loop,
            name="ChingMuBodyPosePoll",
            daemon=True,
        )
        self._body_pose_poll_thread.start()

    def _stop_body_pose_poll_thread(self) -> None:
        thread = self._body_pose_poll_thread
        if thread is None:
            return
        self._body_pose_poll_stop_event.set()
        if thread is not threading.current_thread():
            thread.join(timeout=1.0)
        if not thread.is_alive():
            self._body_pose_poll_thread = None

    def _body_pose_poll_loop(self) -> None:
        interval_s = 1.0 / self.body_pose_poll_hz
        while not self._body_pose_poll_stop_event.is_set():
            self._poll_and_cache_body_pose_once()
            self._body_pose_poll_stop_event.wait(interval_s)

    def _poll_body_pose_from_server(self, server: bytes) -> Optional[Tuple[np.ndarray, np.ndarray, float]]:
        if self.body_pose_id is None:
            return None
        timecode_data = (ctypes.c_int * 1)()
        body_position = (ctypes.c_double * 3)()
        body_quaternion = (ctypes.c_double * 4)()
        timestamp = Timeval()
        if not self.lib.CMTrackerExternTC(
            server,
            int(self.body_pose_id),
            timecode_data,
            body_position,
            body_quaternion,
            ctypes.byref(timestamp),
        ):
            return None
        return (
            np.asarray(tuple(body_position), dtype=np.float64),
            np.asarray(tuple(body_quaternion), dtype=np.float64),
            self._timeval_source_time(timestamp),
        )

    def _poll_body_pose(
        self,
    ) -> Optional[Tuple[np.ndarray, np.ndarray, float]]:
        for server in self._body_pose_servers:
            polled = self._poll_body_pose_from_server(server)
            if polled is not None:
                return polled
        return None

    def _poll_and_cache_body_pose_once(self) -> bool:
        if not self.poll_body_pose:
            return False
        polled = self._poll_body_pose()
        if polled is None:
            return False
        body_position, body_quaternion, source_time_s = polled
        with self._body_pose_lock:
            self._latest_polled_body_pose = (
                body_position.copy(),
                body_quaternion.copy(),
                source_time_s,
            )
        return True

    def _get_latest_polled_body_pose(
        self,
    ) -> Optional[Tuple[np.ndarray, np.ndarray, float]]:
        with self._body_pose_lock:
            if self._latest_polled_body_pose is None:
                return None
            (
                body_position,
                body_quaternion,
                source_time_s,
            ) = self._latest_polled_body_pose
            return (
                body_position.copy(),
                body_quaternion.copy(),
                source_time_s,
            )

    def _frame_with_polled_body_pose(self, frame: MocapFrame) -> MocapFrame:
        if not self.poll_body_pose:
            return frame
        if frame.body_position_mm is not None and frame.body_quaternion_xyzw is not None:
            return frame
        polled = self._get_latest_polled_body_pose()
        if polled is None:
            if not self._poll_and_cache_body_pose_once():
                return frame
            polled = self._get_latest_polled_body_pose()
        if polled is None:
            return frame
        body_position, body_quaternion, source_time_s = polled
        return MocapFrame(
            frame_number=frame.frame_number,
            source_time_s=frame.source_time_s,
            body_position_mm=body_position.copy(),
            body_quaternion_xyzw=body_quaternion.copy(),
            body_markers_mm={sensor: position.copy() for sensor, position in frame.body_markers_mm.items()},
            unlabeled_markers_mm=frame.unlabeled_markers_mm.copy(),
            body_pose_source_time_s=source_time_s,
        )

    def next_frame(self, *, timeout_s: float = 0.1) -> Optional[MocapFrame]:
        deadline = self.monotonic_fn() + float(timeout_s)
        with self._condition:
            while not self._ready:
                remaining = deadline - self.monotonic_fn()
                if remaining <= 0.0:
                    return None
                self._condition.wait(remaining)
            frame = self._ready.popleft()
        return self._frame_with_polled_body_pose(frame)

    def close(self) -> None:
        self._stop_body_pose_poll_thread()
        if not self._started:
            return
        # The Linux vendor library races with its callback thread when the
        # CMPluginUnRegister* functions are called under a live 360 Hz stream,
        # and it exports no end-hierarchy unregister symbol. CMVrpnQuitExtern
        # owns the thread shutdown; retain all callback objects until it returns.
        self.lib.CMVrpnQuitExtern()
        self._tracker_registered = False
        self._end_hierarchy_registered = False
        self._hierarchy_registered = False
        self._hierarchy_complete = False
        self._started = False
