import os

if "MUJOCO_GL" not in os.environ and not os.environ.get("DISPLAY"):
    os.environ["MUJOCO_GL"] = "egl"

import select
import time
from contextlib import nullcontext
import lcm
import threading
import copy
import numpy as np
import torch
from simulator.base_sim import BaseSim
import mujoco.viewer
import mujoco
import math
from scipy.spatial.transform import Rotation as R
from utils.helpers import get_gravity, get_rpy, quaternion_to_euler_array
from utils.motion_lib.rotations import calc_heading_quat_inv, my_quat_rotate
from utils.hitter_runtime_capabilities import (
    HitterRuntimeCapabilities,
    LoopPacing,
    MUJOCO_HITTER_RUNTIME_CAPABILITIES,
    PlannerFeed,
)
from utils.hitter_ball_pipeline import BasePoseW, RealtimeViconBallPipeline
from utils.hitter_chingmu_ghost import (
    ChingMuGhostBallSource,
    ChingMuGhostSourceSettings,
)
from utils.hitter_hil_trace import HitterHilTraceConfig
from utils.read_only_lcm import PublicationAudit, PublishDenyLcm
from loguru import logger
from scipy.spatial.transform import Rotation as sRot
# from pynput import keyboard

DESIRED_BODY_INDICES = [0, 2, 4, 6, 8, 10, 12, 15, 17, 19, 22, 24, 26, 29]
CHINGMU_GHOST_HITTER_RUNTIME_CAPABILITIES = HitterRuntimeCapabilities(
    planner_feed=PlannerFeed.SNAPSHOT_STREAM,
    loop_pacing=LoopPacing.SIMULATOR,
)


def _cfg_get(cfg, key: str, default=None):
    if cfg is None:
        return default
    if isinstance(cfg, dict):
        return cfg.get(key, default)
    return getattr(cfg, key, default)


def _cfg_bool(cfg, key: str, default: bool) -> bool:
    return bool(_cfg_get(cfg, key, default))


def _cfg_float(cfg, key: str, default: float) -> float:
    return float(_cfg_get(cfg, key, default))


def _cfg_int(cfg, key: str, default: int) -> int:
    return int(_cfg_get(cfg, key, default))


def validate_hitter_ball_input_config(config) -> None:
    input_cfg = _cfg_get(config, "hitter_ball_input", None)
    mode = str(_cfg_get(input_cfg, "mode", "mujoco_direct")).strip().lower()
    if mode not in {"mujoco_direct", "chingmu_ghost"}:
        raise ValueError(
            "hitter_ball_input.mode must be 'mujoco_direct' or "
            "'chingmu_ghost', got {!r}".format(mode)
        )
    if mode != "chingmu_ghost":
        return

    errors = []
    if str(_cfg_get(input_cfg, "robot_backend", "")).strip().lower() != "mujoco":
        errors.append("robot_backend must be mujoco")
    if str(_cfg_get(input_cfg, "planner_feed", "")).strip().lower() != "snapshot_stream":
        errors.append("planner_feed must be snapshot_stream")
    if str(_cfg_get(input_cfg, "loop_pacing", "")).strip().lower() != "simulator":
        errors.append("loop_pacing must be simulator")
    if not _cfg_bool(input_cfg, "read_only", False):
        errors.append("read_only must be true")
    if not _cfg_bool(input_cfg, "use_mujoco_base_pose", False):
        errors.append("use_mujoco_base_pose must be true")
    if _cfg_bool(input_cfg, "ghost_collision", True):
        errors.append("ghost_collision must be false")
    if not str(_cfg_get(input_cfg, "channel", "")).strip():
        errors.append("channel must be nonempty")

    control_cfg = _cfg_get(config, "control", None)
    if not _cfg_bool(control_cfg, "real_time", False):
        errors.append("control.real_time must be true")

    positive_float_fields = (
        "stale_timeout_s",
        "table_position_tolerance_m",
        "table_angle_tolerance_deg",
    )
    for field in positive_float_fields:
        value = _cfg_float(input_cfg, field, float("nan"))
        if not np.isfinite(value) or value <= 0.0:
            errors.append(f"{field} must be positive")
    ghost_max_extrapolation = _cfg_float(
        input_cfg,
        "ghost_max_extrapolation_s",
        float("nan"),
    )
    if not np.isfinite(ghost_max_extrapolation) or ghost_max_extrapolation < 0.0:
        errors.append("ghost_max_extrapolation_s must be nonnegative")
    if _cfg_int(input_cfg, "required_table_confirmations", 0) < 1:
        errors.append("required_table_confirmations must be positive")
    if _cfg_float(input_cfg, "ball_xy_margin_m", float("nan")) < 0.0:
        errors.append("ball_xy_margin_m must be nonnegative")
    min_height = _cfg_float(input_cfg, "ball_min_height_offset_m", float("nan"))
    max_height = _cfg_float(input_cfg, "ball_max_height_offset_m", float("nan"))
    if not (
        np.isfinite(min_height)
        and np.isfinite(max_height)
        and min_height < max_height
    ):
        errors.append("ball height offsets must be finite with min < max")

    HitterHilTraceConfig.from_mapping(input_cfg)

    if errors:
        raise ValueError(
            "Invalid chingmu_ghost hitter_ball_input: "
            + "; ".join(errors)
        )


def _publication_attempt_count(audit) -> int:
    snapshot = getattr(audit, "snapshot", None)
    if not callable(snapshot):
        return 0
    return int(snapshot()[0])


class Mujoco(BaseSim):
    is_real = False
    hitter_runtime_capabilities = MUJOCO_HITTER_RUNTIME_CAPABILITIES

    def __init__(self, config):
        super().__init__(config)
        self.marker = self.cfg.get('marker', False)
        self.real_start_time = None
        self.sim_start_time = 0
        self.viewer = None
        self.viewer_enabled = bool(getattr(self.cfg.control, "viewer", False))
        if self.viewer_enabled and not os.environ.get("DISPLAY"):
            logger.warning("[Mujoco] viewer=True but no DISPLAY is available; running headless.")
            self.viewer_enabled = False

        logger.info(f'Visualization Marker: {self.marker}')
        self.target_dof_pos = None
        if self.viewer_enabled:
            self._load_viewer()

        self._init_communication()
        self._init_hitter_ball_input()
        self.spin()
        if getattr(self.cfg.control, "use_teleop", False):
            while True:
                if self.connected:
                    print("============================== init sim done ==============================")
                    break
        else:
            self.firstReceiveAlarm = True

        # Keyboard pause support is currently disabled.
        self.paused = False
        # self.listener = keyboard.Listener(on_press=self._on_press_fallback)
        # self.listener.start()

    def _setup(self):
        super()._setup()
        validate_hitter_ball_input_config(self.cfg)
        self._mujoco_poll_stop_event = threading.Event()
        self._mujoco_communication_closed = False
        self._mujoco_owned_subscribers = []
        self._hitter_lcm_publication_audit = PublicationAudit()
        self.hitter_hil_trace_config = None
        if self._hitter_ball_input_mode() == "chingmu_ghost":
            self.hitter_hil_trace_config = HitterHilTraceConfig.from_mapping(
                _cfg_get(self.cfg, "hitter_ball_input", None)
            )
        lcm_url = self._hitter_ball_input_config_get(
            "lcm_url",
            "udpm://239.255.76.67:7667?ttl=255",
        )
        raw_lcm = lcm.LCM(lcm_url)
        if self._hitter_ball_input_mode() == "chingmu_ghost":
            self.lc = PublishDenyLcm(
                raw_lcm,
                self._hitter_lcm_publication_audit,
            )
        else:
            self.lc = raw_lcm

    def _hitter_ball_input_config_get(self, key: str, default=None):
        cfg = getattr(self.cfg, "hitter_ball_input", None)
        return _cfg_get(cfg, key, default)

    def _hitter_ball_input_mode(self) -> str:
        mode = str(
            self._hitter_ball_input_config_get("mode", "mujoco_direct")
        ).strip().lower()
        if mode not in {"mujoco_direct", "chingmu_ghost"}:
            raise ValueError(
                "hitter_ball_input.mode must be 'mujoco_direct' or "
                "'chingmu_ghost', got {!r}".format(mode)
            )
        return mode

    def _load_asset(self):
        super()._load_asset()
        xml_path = os.path.join(self.cfg.asset.asset_root, self.cfg.asset.asset_file)
        
        self.mujoco_model = mujoco.MjModel.from_xml_path(xml_path)
        self.mujoco_data = mujoco.MjData(self.mujoco_model)
        self.mujoco_model.opt.timestep = self.low_dt
        print("mujoco time step : ", self.mujoco_model.opt.timestep)
        
        self.default_qpos = self.mujoco_data.qpos.copy()
        default_root_pos = getattr(self.cfg.asset, "default_root_pos", None)
        if default_root_pos is not None:
            default_root_pos = np.asarray(default_root_pos, dtype=np.float64).reshape(-1)
            if default_root_pos.shape != (3,):
                raise ValueError(f"asset.default_root_pos must contain 3 values, got {default_root_pos!r}")
            self.default_qpos[:3] = default_root_pos
        self.default_qvel = self.mujoco_data.qvel.copy()
        self._init_table_tennis_state()
        
        # Compute frozen DOF indices as the complement of active DOFs.
        self.frozen_dof_idx = np.array([i for i in range(self.num_dof) if i not in self.active_dof_idx], dtype=np.int32)
        foot_body_names = getattr(self.cfg.asset, "foot_body_names", None)
        if not foot_body_names:
            foot_body_names = ["left_ankle_roll_link", "right_ankle_roll_link"]
        self.foot_body_names = list(foot_body_names)
        self.foot_body_ids = []
        for name in self.foot_body_names:
            body_id = mujoco.mj_name2id(self.mujoco_model, mujoco.mjtObj.mjOBJ_BODY, name)
            if body_id < 0:
                logger.warning(f"[Mujoco] foot body not found: {name}")
                continue
            self.foot_body_ids.append(body_id)
        self.foot_body_ids = np.asarray(self.foot_body_ids, dtype=np.int32)
        self.foot_contact_forces_w = np.zeros((len(self.foot_body_ids), 3), dtype=np.float32)

    def _init_table_tennis_state(self):
        self.table_tennis_cfg = getattr(self.cfg, "table_tennis", None)
        self.table_tennis_enabled = bool(
            self.table_tennis_cfg and self.table_tennis_cfg.get("enabled", False)
        )
        self.hitter_ball_body_id = -1
        self.hitter_ball_qposadr = None
        self.hitter_ball_qveladr = None
        self.ball_pos_world = np.zeros(3, dtype=np.float32)
        self.ball_quat_world = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32)
        self.ball_vel_world = np.zeros(3, dtype=np.float32)
        self.ball_ang_vel_world = np.zeros(3, dtype=np.float32)
        self.ball_visible = False
        self.ball_state_estimator_ready = False
        self.ball_state_estimator_sample_count = 0
        self.ball_state_estimator_min_samples = 31
        self.randomize_hitter_ball = False
        self.base_pose_valid = True
        self.hitter_ghost_mode_enabled = False
        self.hitter_ball_source = self
        self._hitter_ghost_lock = threading.RLock()
        self._hitter_pending_ghost_snapshot = None
        self.hitter_ball_ghost_body_id = -1
        self.hitter_ball_ghost_geom_id = -1
        self.hitter_ball_ghost_mocap_id = -1
        self.hitter_ghost_max_extrapolation_s = 0.05
        self._hitter_base_pose_lock = threading.RLock()
        self._hitter_base_pose_position_w = np.zeros(3, dtype=np.float32)
        self._hitter_base_pose_quaternion_xyzw = np.array(
            [0.0, 0.0, 0.0, 1.0],
            dtype=np.float32,
        )
        self._hitter_base_pose_simulation_time_s = 0.0
        self._hitter_base_pose_captured_monotonic_s = time.monotonic()
        self._hitter_ghost_unregister_listener = None
        self.table_tennis_rng = np.random.default_rng()
        self.table_tennis_ball_trajectory_index = 0
        self._hitter_ball_launch_qpos = None
        self._hitter_ball_launch_qvel = None
        self._init_hitter_ghost_visual_state()
        if not self.table_tennis_enabled:
            return
        self.randomize_hitter_ball = bool(
            self.table_tennis_cfg.get("randomize_ball_on_reset", False)
        )
        self.table_tennis_rng = np.random.default_rng(
            self.table_tennis_cfg.get("ball_random_seed", None)
        )

        ball_body_name = self.table_tennis_cfg.get("ball_body_name", "hitter_ball")
        ball_joint_name = self.table_tennis_cfg.get("ball_joint_name", "hitter_ball_freejoint")
        self.hitter_ball_body_id = mujoco.mj_name2id(self.mujoco_model, mujoco.mjtObj.mjOBJ_BODY, ball_body_name)
        ball_joint_id = mujoco.mj_name2id(self.mujoco_model, mujoco.mjtObj.mjOBJ_JOINT, ball_joint_name)
        if self.hitter_ball_body_id < 0 or ball_joint_id < 0:
            logger.warning(
                f"[Mujoco] table_tennis.enabled=True but ball body/joint not found: {ball_body_name}/{ball_joint_name}"
            )
            self.table_tennis_enabled = False
            return

        self.hitter_ball_qposadr = int(self.mujoco_model.jnt_qposadr[ball_joint_id])
        self.hitter_ball_qveladr = int(self.mujoco_model.jnt_dofadr[ball_joint_id])
        self.reset_hitter_ball(update_default=True)

    def _init_hitter_ghost_visual_state(self):
        body_id = mujoco.mj_name2id(
            self.mujoco_model,
            mujoco.mjtObj.mjOBJ_BODY,
            "hitter_ball_ghost",
        )
        geom_id = mujoco.mj_name2id(
            self.mujoco_model,
            mujoco.mjtObj.mjOBJ_GEOM,
            "hitter_ball_ghost_geom",
        )
        self.hitter_ball_ghost_body_id = int(body_id)
        self.hitter_ball_ghost_geom_id = int(geom_id)
        self.hitter_ball_ghost_mocap_id = -1
        if body_id >= 0:
            mocap_id = int(self.mujoco_model.body_mocapid[body_id])
            self.hitter_ball_ghost_mocap_id = mocap_id
            if mocap_id >= 0:
                self.mujoco_data.mocap_pos[mocap_id] = [0.0, 0.0, -10.0]
                self.mujoco_data.mocap_quat[mocap_id] = [1.0, 0.0, 0.0, 0.0]
        if geom_id >= 0:
            self.mujoco_model.geom_rgba[geom_id, 3] = 0.0

    def _sample_table_tennis_vector(self, key: str, default, size: int, *, randomize: bool) -> np.ndarray:
        if randomize:
            candidates = self.table_tennis_cfg.get(f"{key}_candidates", None)
            if candidates is not None:
                arr = np.asarray(candidates, dtype=np.float64)
                if arr.ndim != 2 or arr.shape[1] != size:
                    raise ValueError(f"table_tennis.{key}_candidates must have shape (N, {size}), got {arr.shape}.")
                return arr[int(self.table_tennis_rng.integers(arr.shape[0]))].copy()

            value_range = self.table_tennis_cfg.get(f"{key}_range", None)
            if value_range is not None:
                bounds = np.asarray(value_range, dtype=np.float64)
                if bounds.shape != (size, 2):
                    raise ValueError(f"table_tennis.{key}_range must have shape ({size}, 2), got {bounds.shape}.")
                low = bounds[:, 0]
                high = bounds[:, 1]
                if np.any(high < low):
                    raise ValueError(f"table_tennis.{key}_range contains high < low: {value_range!r}.")
                return self.table_tennis_rng.uniform(low, high)

        value = self.table_tennis_cfg.get(key, default)
        arr = np.asarray(value, dtype=np.float64).reshape(-1)
        if arr.shape != (size,):
            raise ValueError(f"table_tennis.{key} must contain {size} values, got {value!r}.")
        return arr

    def _sample_table_tennis_trajectory(self):
        candidates = self.table_tennis_cfg.get("ball_trajectory_candidates", None)
        if not candidates:
            return None

        count = len(candidates)
        selection = str(self.table_tennis_cfg.get("ball_trajectory_selection", "sequence")).strip().lower()
        if selection == "random":
            index = int(self.table_tennis_rng.integers(count))
        elif selection in {"sequence", "sequential", "cycle"}:
            index = self.table_tennis_ball_trajectory_index % count
            self.table_tennis_ball_trajectory_index += 1
        else:
            raise ValueError(f"table_tennis.ball_trajectory_selection must be 'sequence' or 'random', got {selection!r}.")

        candidate = candidates[index]
        name = str(candidate.get("name", f"trajectory_{index}"))
        pos = np.asarray(candidate.get("pos"), dtype=np.float64).reshape(-1)
        lin_vel = np.asarray(candidate.get("lin_vel"), dtype=np.float64).reshape(-1)
        ang_vel = np.asarray(candidate.get("ang_vel", self.table_tennis_cfg.get("ball_initial_ang_vel", [0.0, 0.0, 0.0])), dtype=np.float64).reshape(-1)
        if pos.shape != (3,) or lin_vel.shape != (3,) or ang_vel.shape != (3,):
            raise ValueError(
                f"table_tennis.ball_trajectory_candidates[{index}] must provide pos/lin_vel/ang_vel with 3 values."
            )
        return name, pos, lin_vel, ang_vel

    def reset_hitter_ball(self, pos=None, lin_vel=None, update_default: bool = False):
        if getattr(self, "hitter_ghost_mode_enabled", False):
            self._park_physical_hitter_ball(update_default=True)
            return
        if not self.table_tennis_enabled or self.hitter_ball_qposadr is None or self.hitter_ball_qveladr is None:
            return

        randomize = self.randomize_hitter_ball and not update_default
        trajectory_name = None
        trajectory = self._sample_table_tennis_trajectory() if randomize and pos is None and lin_vel is None else None
        if trajectory is not None:
            trajectory_name, pos, lin_vel, ang_vel = trajectory
        elif pos is None:
            pos = self._sample_table_tennis_vector("ball_initial_pos", [2.74, 0.0, 1.5], 3, randomize=randomize)
        quat = self.table_tennis_cfg.get("ball_initial_quat_wxyz", [1.0, 0.0, 0.0, 0.0])
        if lin_vel is None:
            lin_vel = self._sample_table_tennis_vector("ball_initial_lin_vel", [-3.0, 0.0, -1], 3, randomize=randomize)
        if trajectory is None:
            ang_vel = self._sample_table_tennis_vector("ball_initial_ang_vel", [0.0, 0.0, 0.0], 3, randomize=randomize)

        qposadr = self.hitter_ball_qposadr
        qveladr = self.hitter_ball_qveladr
        with self._viewer_lock():
            self.mujoco_data.qpos[qposadr:qposadr + 3] = np.asarray(pos, dtype=np.float64)
            self.mujoco_data.qpos[qposadr + 3:qposadr + 7] = np.asarray(quat, dtype=np.float64)
            self.mujoco_data.qvel[qveladr:qveladr + 3] = np.asarray(lin_vel, dtype=np.float64)
            self.mujoco_data.qvel[qveladr + 3:qveladr + 6] = np.asarray(ang_vel, dtype=np.float64)
            mujoco.mj_forward(self.mujoco_model, self.mujoco_data)

            if update_default:
                self.default_qpos[qposadr:qposadr + 7] = self.mujoco_data.qpos[qposadr:qposadr + 7]
                self.default_qvel[qveladr:qveladr + 6] = self.mujoco_data.qvel[qveladr:qveladr + 6]
        self._update_hitter_ball_state()
        if randomize:
            logger.debug(
                "[Mujoco] hitter ball reset{}: pos={}, lin_vel={}, ang_vel={}",
                f" ({trajectory_name})" if trajectory_name else "",
                pos,
                lin_vel,
                ang_vel,
            )

    def capture_hitter_ball_launch_state(self) -> bool:
        if getattr(self, "hitter_ghost_mode_enabled", False):
            return False
        if (
            not self.table_tennis_enabled
            or self.hitter_ball_qposadr is None
            or self.hitter_ball_qveladr is None
        ):
            return False
        qposadr = self.hitter_ball_qposadr
        qveladr = self.hitter_ball_qveladr
        with self._viewer_lock():
            self._hitter_ball_launch_qpos = self.mujoco_data.qpos[
                qposadr:qposadr + 7
            ].copy()
            self._hitter_ball_launch_qvel = self.mujoco_data.qvel[
                qveladr:qveladr + 6
            ].copy()
        return True

    def restore_hitter_ball_launch_state(self) -> bool:
        if getattr(self, "hitter_ghost_mode_enabled", False):
            return False
        if (
            not self.table_tennis_enabled
            or self.hitter_ball_qposadr is None
            or self.hitter_ball_qveladr is None
            or self._hitter_ball_launch_qpos is None
            or self._hitter_ball_launch_qvel is None
        ):
            return False
        qposadr = self.hitter_ball_qposadr
        qveladr = self.hitter_ball_qveladr
        with self._viewer_lock():
            self.mujoco_data.qpos[qposadr:qposadr + 7] = self._hitter_ball_launch_qpos
            self.mujoco_data.qvel[qveladr:qveladr + 6] = self._hitter_ball_launch_qvel
            mujoco.mj_forward(self.mujoco_model, self.mujoco_data)
        self._update_hitter_ball_state()
        return True

    def _update_hitter_ball_state(self):
        if getattr(self, "hitter_ghost_mode_enabled", False):
            source = getattr(self, "hitter_ball_source", None)
            if source is not None and source is not self and hasattr(source, "state"):
                state = source.state()
                self.ball_pos_world = np.asarray(
                    state.position_w,
                    dtype=np.float32,
                ).copy()
                self.ball_vel_world = np.asarray(
                    state.velocity_w,
                    dtype=np.float32,
                ).copy()
                self.ball_quat_world = np.array(
                    [0.0, 0.0, 0.0, 1.0],
                    dtype=np.float32,
                )
                self.ball_ang_vel_world = np.zeros(3, dtype=np.float32)
                self.ball_visible = bool(state.visible)
                self.ball_state_estimator_ready = bool(state.ready)
                self.ball_state_estimator_sample_count = int(
                    state.sample_count
                )
            else:
                self.ball_pos_world = np.zeros(3, dtype=np.float32)
                self.ball_vel_world = np.zeros(3, dtype=np.float32)
                self.ball_quat_world = np.array(
                    [0.0, 0.0, 0.0, 1.0],
                    dtype=np.float32,
                )
                self.ball_ang_vel_world = np.zeros(3, dtype=np.float32)
                self.ball_visible = False
                self.ball_state_estimator_ready = False
                self.ball_state_estimator_sample_count = 0
            return
        if not self.table_tennis_enabled or self.hitter_ball_body_id < 0:
            self.ball_visible = False
            self.ball_state_estimator_ready = False
            self.ball_state_estimator_sample_count = 0
            return
        self.ball_pos_world = self.mujoco_data.xpos[self.hitter_ball_body_id].astype(np.float32).copy()
        self.ball_quat_world = self.mujoco_data.xquat[self.hitter_ball_body_id][[1, 2, 3, 0]].astype(np.float32).copy()
        qveladr = self.hitter_ball_qveladr
        self.ball_vel_world = self.mujoco_data.qvel[qveladr:qveladr + 3].astype(np.float32).copy()
        self.ball_ang_vel_world = self.mujoco_data.qvel[qveladr + 3:qveladr + 6].astype(np.float32).copy()
        self.ball_visible = True
        self.ball_state_estimator_ready = True
        self.ball_state_estimator_sample_count = self.ball_state_estimator_min_samples

    def _park_physical_hitter_ball(self, *, update_default: bool) -> None:
        if (
            not self.table_tennis_enabled
            or self.hitter_ball_qposadr is None
            or self.hitter_ball_qveladr is None
        ):
            return
        qposadr = self.hitter_ball_qposadr
        qveladr = self.hitter_ball_qveladr
        parked_qpos = np.array(
            [0.0, 0.0, -10.0, 1.0, 0.0, 0.0, 0.0],
            dtype=np.float64,
        )
        with self._viewer_lock():
            self.mujoco_data.qpos[qposadr:qposadr + 7] = parked_qpos
            self.mujoco_data.qvel[qveladr:qveladr + 6] = 0.0
            if update_default:
                self.default_qpos[qposadr:qposadr + 7] = parked_qpos
                self.default_qvel[qveladr:qveladr + 6] = 0.0
            mujoco.mj_forward(self.mujoco_model, self.mujoco_data)
        self.ball_visible = False
        self.ball_state_estimator_ready = False
        self.ball_state_estimator_sample_count = 0

    def _init_communication(self):
        self.firstReceiveAlarm = False
        self.firstReceiveOdometer = False
        self._init_time = time.time()
        
        self.teleop_state_subscriber = self.lc.subscribe('camera_reference_data', self._teleop_state_handler)
        self._mujoco_owned_subscribers = [self.teleop_state_subscriber]

    def _motion_ball_planner_config(self):
        motion_cfg = getattr(self.cfg, "motion", None)
        if motion_cfg is None:
            return {}
        if isinstance(motion_cfg, dict):
            return motion_cfg.get("ball_planner", {}) or {}
        return getattr(motion_cfg, "ball_planner", {}) or {}

    def copy_hitter_base_pose_w(self) -> BasePoseW:
        with self._hitter_base_pose_lock:
            position = np.asarray(
                getattr(
                    self,
                    "root_trans_world",
                    self._hitter_base_pose_position_w,
                ),
                dtype=np.float32,
            ).reshape(3).copy()
            quaternion = np.asarray(
                getattr(
                    self,
                    "root_quat_world",
                    self._hitter_base_pose_quaternion_xyzw,
                ),
                dtype=np.float32,
            ).reshape(4).copy()
            valid = bool(getattr(self, "base_pose_valid", True))
            simulation_time_s = float(self._hitter_base_pose_simulation_time_s)
            captured_monotonic_s = float(
                self._hitter_base_pose_captured_monotonic_s
            )
        norm = float(np.linalg.norm(quaternion.astype(np.float64)))
        if not np.isfinite(norm) or norm <= 1.0e-12:
            quaternion = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32)
            valid = False
        else:
            quaternion = (quaternion.astype(np.float64) / norm).astype(np.float32)
        return BasePoseW(
            position_w=position,
            quaternion_xyzw=quaternion,
            valid=valid,
            simulation_time_s=simulation_time_s,
            captured_monotonic_s=captured_monotonic_s,
        )

    def _update_hitter_base_pose_cache(self) -> None:
        with self._hitter_base_pose_lock:
            self._hitter_base_pose_position_w = np.asarray(
                self.root_trans_world,
                dtype=np.float32,
            ).reshape(3).copy()
            quat = np.asarray(self.root_quat_world, dtype=np.float32).reshape(4)
            norm = float(np.linalg.norm(quat.astype(np.float64)))
            if np.isfinite(norm) and norm > 1.0e-12:
                quat = (quat.astype(np.float64) / norm).astype(np.float32)
            else:
                quat = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32)
            self._hitter_base_pose_quaternion_xyzw = quat.copy()
            self._hitter_base_pose_simulation_time_s = float(
                getattr(self.mujoco_data, "time", 0.0)
            )
            self._hitter_base_pose_captured_monotonic_s = time.monotonic()
            self.base_pose_valid = True

    def queue_hitter_ghost_snapshot(self, snapshot) -> None:
        queued = {
            "position_w": np.asarray(
                snapshot.position_w,
                dtype=np.float32,
            ).reshape(3).copy(),
            "velocity_w": np.asarray(
                snapshot.velocity_w,
                dtype=np.float32,
            ).reshape(3).copy(),
            "visible": bool(snapshot.visible),
            "ready": bool(snapshot.ready),
            "received_monotonic_s": float(snapshot.received_monotonic_s),
        }
        with self._hitter_ghost_lock:
            self._hitter_pending_ghost_snapshot = queued

    def apply_pending_hitter_ghost_snapshot(
        self,
        *,
        now_monotonic_s: float,
    ) -> None:
        with self._viewer_lock():
            self._apply_pending_hitter_ghost_snapshot_locked(
                now_monotonic_s=now_monotonic_s,
                forward=True,
            )

    def _apply_pending_hitter_ghost_snapshot_locked(
        self,
        *,
        now_monotonic_s: float,
        forward: bool = False,
    ) -> None:
        if (
            not getattr(self, "hitter_ghost_mode_enabled", False)
            or self.hitter_ball_ghost_mocap_id < 0
            or self.hitter_ball_ghost_geom_id < 0
        ):
            return
        with self._hitter_ghost_lock:
            snapshot = self._hitter_pending_ghost_snapshot
            self._hitter_pending_ghost_snapshot = None
        if snapshot is None:
            return
        if not (snapshot["visible"] and snapshot["ready"]):
            self._hide_hitter_ghost_locked()
            if forward:
                mujoco.mj_forward(self.mujoco_model, self.mujoco_data)
            return
        dt = float(now_monotonic_s) - float(snapshot["received_monotonic_s"])
        if not np.isfinite(dt):
            dt = 0.0
        dt = float(np.clip(dt, 0.0, self.hitter_ghost_max_extrapolation_s))
        position = snapshot["position_w"] + snapshot["velocity_w"] * dt
        mocap_id = self.hitter_ball_ghost_mocap_id
        self.mujoco_data.mocap_pos[mocap_id] = position.astype(np.float64)
        self.mujoco_data.mocap_quat[mocap_id] = [1.0, 0.0, 0.0, 0.0]
        self.mujoco_model.geom_rgba[self.hitter_ball_ghost_geom_id, 3] = 1.0
        if forward:
            mujoco.mj_forward(self.mujoco_model, self.mujoco_data)

    def _hide_hitter_ghost_locked(self) -> None:
        if self.hitter_ball_ghost_mocap_id >= 0:
            self.mujoco_data.mocap_pos[self.hitter_ball_ghost_mocap_id] = [
                0.0,
                0.0,
                -10.0,
            ]
            self.mujoco_data.mocap_quat[self.hitter_ball_ghost_mocap_id] = [
                1.0,
                0.0,
                0.0,
                0.0,
            ]
        if self.hitter_ball_ghost_geom_id >= 0:
            self.mujoco_model.geom_rgba[self.hitter_ball_ghost_geom_id, 3] = 0.0

    def set_hitter_ghost_mode(self, enabled: bool) -> None:
        self.hitter_ghost_mode_enabled = bool(enabled)
        if self.hitter_ghost_mode_enabled:
            self.preserve_hitter_ball_state_on_calibrate = False
            self.randomize_hitter_ball = False
            self._hide_hitter_ghost_locked()
            self._park_physical_hitter_ball(update_default=True)
        else:
            self._hide_hitter_ghost_locked()

    def _init_hitter_ball_input(self) -> None:
        mode = self._hitter_ball_input_mode()
        if mode == "mujoco_direct":
            self.hitter_ball_source = self
            self.hitter_runtime_capabilities = MUJOCO_HITTER_RUNTIME_CAPABILITIES
            self.set_hitter_ghost_mode(False)
            return

        self.set_hitter_ghost_mode(True)
        planner_cfg = dict(self._motion_ball_planner_config())
        stale_timeout_s = float(
            self._hitter_ball_input_config_get("stale_timeout_s", 0.15)
        )
        pipeline = RealtimeViconBallPipeline(
            planner_cfg,
            self.copy_hitter_base_pose_w,
            stale_timeout_s=stale_timeout_s,
        )
        table_center_xy = planner_cfg.get(
            "table_center_xy_w",
            [1.365369, 0.0],
        )
        settings = ChingMuGhostSourceSettings(
            lcm_url=self._hitter_ball_input_config_get(
                "lcm_url",
                "udpm://239.255.76.67:7667?ttl=255",
            ),
            channel=self._hitter_ball_input_config_get(
                "channel",
                "vicon_state_data",
            ),
            stale_timeout_s=stale_timeout_s,
            expected_table_center_w=(
                float(table_center_xy[0]),
                float(table_center_xy[1]),
                float(planner_cfg.get("table_height", 0.76)),
            ),
            expected_table_quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
            table_position_tolerance_m=float(
                self._hitter_ball_input_config_get(
                    "table_position_tolerance_m",
                    0.03,
                )
            ),
            table_angle_tolerance_deg=float(
                self._hitter_ball_input_config_get(
                    "table_angle_tolerance_deg",
                    3.0,
                )
            ),
            required_table_confirmations=int(
                self._hitter_ball_input_config_get(
                    "required_table_confirmations",
                    3,
                )
            ),
            table_length_m=float(planner_cfg.get("table_length", 2.730738)),
            table_width_m=float(planner_cfg.get("table_width", 1.512451)),
            ball_xy_margin_m=float(
                self._hitter_ball_input_config_get("ball_xy_margin_m", 0.50)
            ),
            ball_min_height_offset_m=float(
                self._hitter_ball_input_config_get(
                    "ball_min_height_offset_m",
                    -0.30,
                )
            ),
            ball_max_height_offset_m=float(
                self._hitter_ball_input_config_get(
                    "ball_max_height_offset_m",
                    2.50,
                )
            ),
        )
        source = ChingMuGhostBallSource(
            settings,
            pipeline=pipeline,
            lcm_factory=lcm.LCM,
            publication_audit=self._hitter_lcm_publication_audit,
        )
        self._hitter_ghost_unregister_listener = (
            source.register_hitter_ball_listener(
                self.queue_hitter_ghost_snapshot
            )
        )
        self.hitter_ball_source = source
        self.hitter_runtime_capabilities = (
            CHINGMU_GHOST_HITTER_RUNTIME_CAPABILITIES
        )
        publish_count = _publication_attempt_count(
            getattr(self, "_hitter_lcm_publication_audit", None)
        )
        logger.info(
            "HITTER HIL safety banner: simulator=Mujoco robot_backend=mujoco "
            "ball_pipeline=chingmu_ghost channel={} read_only=true "
            "ghost_collision=false control_publication_count={}",
            settings.channel,
            publish_count,
        )
        source.start()

    def poll(self, cb=None):
        try:
            while not self._mujoco_poll_stop_event.is_set():
                timeout = 0.01
                rfds, _wfds, _efds = select.select(
                    [self.lc.fileno()],
                    [],
                    [],
                    timeout,
                )
                if rfds:
                    self.lc.handle()
        except KeyboardInterrupt:
            pass

    def spin(self):
        self.run_thread = threading.Thread(target=self.poll, daemon=True)
        self.run_thread.start()

    def close(self) -> bool:
        if getattr(self, "_mujoco_communication_closed", False):
            return True
        ok = True
        source = getattr(self, "hitter_ball_source", None)
        if source is not None and source is not self and hasattr(source, "close"):
            ok = bool(source.close()) and ok
        unregister = getattr(self, "_hitter_ghost_unregister_listener", None)
        if callable(unregister):
            try:
                unregister()
            finally:
                self._hitter_ghost_unregister_listener = None
        stop_event = getattr(self, "_mujoco_poll_stop_event", None)
        if stop_event is not None:
            stop_event.set()
        run_thread = getattr(self, "run_thread", None)
        if (
            run_thread is not None
            and run_thread is not threading.current_thread()
            and hasattr(run_thread, "join")
        ):
            run_thread.join(timeout=1.0)
            if hasattr(run_thread, "is_alive") and run_thread.is_alive():
                ok = False
        subscribers = list(getattr(self, "_mujoco_owned_subscribers", []))
        if not subscribers and hasattr(self, "teleop_state_subscriber"):
            subscribers = [self.teleop_state_subscriber]
        for subscriber in subscribers:
            try:
                self.lc.unsubscribe(subscriber)
            except Exception:
                logger.exception("[Mujoco] failed to unsubscribe LCM subscriber")
                ok = False
        self._mujoco_owned_subscribers = []
        viewer = getattr(self, "viewer", None)
        if viewer is not None and hasattr(viewer, "close"):
            try:
                viewer.close()
            except Exception:
                logger.exception("[Mujoco] failed to close viewer")
                ok = False
        if ok:
            self._mujoco_communication_closed = True
        publish_count = _publication_attempt_count(
            getattr(self, "_hitter_lcm_publication_audit", None)
        )
        logger.info(
            "HITTER HIL shutdown: control_publication_count={}",
            publish_count,
        )
        return ok

    def _on_press_fallback(self, key):
        try:
            if key == keyboard.Key.space:
                self.paused = not self.paused
                print(f"\n[GLOBAL PAUSE] Status: {self.paused}", flush=True)

            if hasattr(key, 'char') and key.char == 'c':
                cam = self.viewer.cam
                print("\n" + "="*30)
                print("Current Camera Parameters:")
                print(f"self.viewer.cam.lookat[:] = np.array([{cam.lookat[0]:.8f}, {cam.lookat[1]:.8f}, {cam.lookat[2]:.8f}])")
                print(f"self.viewer.cam.distance = {cam.distance:.4f}")
                print(f"self.viewer.cam.azimuth = {cam.azimuth:.4f}")
                print(f"self.viewer.cam.elevation = {cam.elevation:.4f}")
                print("="*30 + "\n")
        except Exception as e:
            print(f"Error in keyboard listener: {e}")

    def _viewer_lock(self):
        viewer = getattr(self, "viewer", None)
        if viewer is None:
            return nullcontext()
        try:
            return viewer.lock()
        except Exception:
            return nullcontext()

    def _load_viewer(self):
        self.viewer = mujoco.viewer.launch_passive(self.mujoco_model, self.mujoco_data)
        
        # Fixed static camera view.
        with self._viewer_lock():
            self.viewer.cam.lookat[:] = np.array([0.0,0.0,0.8])
            self.viewer.cam.distance = 2.5
            self.viewer.cam.azimuth = 180
            self.viewer.cam.elevation = -10
        
        # Tracking camera mode can be enabled by uncommenting the block below.
        # self.viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        
        # body_name = "torso_link"
        # body_id = mujoco.mj_name2id(self.mujoco_model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        # self.viewer.cam.trackbodyid = body_id
        
        # self.viewer.cam.lookat[:] = np.array([0.0, 0.0, 0.0]) 
        # self.viewer.cam.distance = 3       
        # self.viewer.cam.azimuth = 180
        # self.viewer.cam.elevation = 0
        
        self.marker_pos = None

    def render(self):
        if self.viewer is None:
            return
        if not self.viewer.is_running():
            return
        with self._viewer_lock():
            if self.marker_pos is not None:
                self.viewer.user_scn.ngeom = 0
                max_marker_count = min(self.marker_pos.shape[0], len(self.viewer.user_scn.geoms))
                for i in range(max_marker_count):
                    pos = self.marker_pos[i].flatten()[:3]
                    mujoco.mjv_initGeom(
                        self.viewer.user_scn.geoms[i],
                        type=mujoco.mjtGeom.mjGEOM_SPHERE,
                        size=np.array([0.03, 0, 0], dtype=np.float64),
                        pos=pos.astype(np.float64),
                        mat=np.eye(3).flatten().astype(np.float64),
                        rgba=np.array([1,0,0,1], dtype=np.float32)
                    )
                self.viewer.user_scn.ngeom = max_marker_count
        self.viewer.sync()

    def get_state(self):
        data = self.mujoco_data
        self.root_quat = data.qpos.astype(np.double)[3:7][[1,2,3,0]]
        self.root_quat_world = data.qpos.astype(np.double)[3:7][[1,2,3,0]]
        r = R.from_quat(self.root_quat)
        self.root_rpy = quaternion_to_euler_array(self.root_quat)
        self.root_rpy[self.root_rpy > math.pi] -= 2 * math.pi
        
        self.root_trans = data.qpos[:3].astype(np.double)
        self.root_trans_world = data.qpos[:3].astype(np.double)

        self.projected_gravity = r.apply(np.array([0., 0., -1.]), inverse=True).astype(np.double)
        lin_vel_world = data.qvel.astype(np.double)[0:3]
        self.base_lin_vel = r.apply(lin_vel_world, inverse=True).astype(np.float32)
        self.base_ang_vel = data.qvel.astype(np.double)[3:6]
        
        # Read positions and velocities for active DOFs only.
        all_dof_pos = data.qpos[7:].astype(np.double)
        all_dof_vel = data.qvel[6:].astype(np.double)
        self.dof_pos = all_dof_pos[self.active_dof_idx]
        self.dof_vel = all_dof_vel[self.active_dof_idx]

        if self.cfg.control.update_with_fk:
            fk_info, fk_info_tensor = self.fk()
            self.torso_quat = fk_info[self.torso_name]['quat']
            self.torso_trans = fk_info[self.torso_name]['pos']
            self.robot_fk_info = fk_info_tensor

        if hasattr(self.cfg.control, 'use_teleop'):
            if self.cfg.control.use_teleop:
                idx_r2p = [
                    0, 6, 12, 1, 7, 13, 2, 8, 14, 3, 9, 15, 22, 4, 10, 16, 23, 5, 11, 17, 24, 18, 25, 19, 26, 20, 27, 21, 28
                ]
                self.teleop_dof_pos = self.teleop_dof_pos_tmp.copy()[idx_r2p]
                self.teleop_quat = self.align_quat(self.teleop_quat_tmp.copy())
                if self.cfg.control.update_with_fk:
                    fk_info, fk_info_tensor = self.fk_teleop()
                    self.teleop_quat = fk_info[self.torso_name]['quat']
                    teleop_body_pos_w = fk_info_tensor[DESIRED_BODY_INDICES, :3]
                    self.teleop_body_pos_w_aligned = self.align_pos_batch(teleop_body_pos_w)

        if self.foot_body_ids.size > 0:
            forces_body = data.cfrc_ext[self.foot_body_ids, :3].astype(np.float32)
            xmat = data.xmat[self.foot_body_ids].reshape(-1, 3, 3)
            self.foot_contact_forces_w = np.einsum("nij,nj->ni", xmat, forces_body)
        else:
            self.foot_contact_forces_w = np.zeros((0, 3), dtype=np.float32)
        self._update_hitter_base_pose_cache()
        self._update_hitter_ball_state()

    def apply_action(self, action):
        # Record the first simulator timestamp.
        if self.real_start_time is None:
            self.real_start_time = time.perf_counter()
            self.sim_start_time = self.mujoco_data.time

        self.act = action.copy()
        is_mosaic = getattr(self.cfg.control, 'is_mosaic', False)
        
        # Compute target joint positions.
        if is_mosaic:
            tgt_dof_pos = action
        else:
            current_default_angles = self.default_angles[self.active_dof_idx].copy()
            use_residual = getattr(self.cfg.control, 'use_residual', False)
            if use_residual and self.ref_dof_pos is not None:
                if hasattr(self.cfg.control, 'residual_joint_indices') and self.cfg.control.residual_joint_indices is not None:
                    for idx in self.cfg.control.residual_joint_indices:
                        current_default_angles[idx] = self.ref_dof_pos[idx]
            self.dof_tracking_init_pos = current_default_angles
            tgt_dof_pos = current_default_angles + action * self.cfg.control.action_scale

        # Physics stepping.
        torque_limit = np.array(self.cfg.control.torque_clip_value, dtype=np.float32)
        
        for _ in range(self.decimation):
            while self.paused:
                if not self.viewer.is_running():
                    break
                self.render()
                time.sleep(0.01)

            with self._viewer_lock():
                all_dof_pos = self.mujoco_data.qpos[7:].astype(np.float32)
                all_dof_vel = self.mujoco_data.qvel[6:].astype(np.float32)

                torque_active = (tgt_dof_pos - all_dof_pos[self.active_dof_idx]) * self.kps[self.active_dof_idx] \
                                - all_dof_vel[self.active_dof_idx] * self.kds[self.active_dof_idx]
                torque_active = np.clip(torque_active, -torque_limit, torque_limit)

                if len(self.frozen_dof_idx) > 0:
                    torque_all = np.zeros(self.num_dof, dtype=np.float32)
                    torque_all[self.active_dof_idx] = torque_active
                    torque_frozen = (self.default_angles[self.frozen_dof_idx] - all_dof_pos[self.frozen_dof_idx]) * self.kps[self.frozen_dof_idx] \
                                    - all_dof_vel[self.frozen_dof_idx] * self.kds[self.frozen_dof_idx]
                    torque_all[self.frozen_dof_idx] = torque_frozen
                    self.mujoco_data.ctrl[:] = torque_all
                else:
                    self.mujoco_data.ctrl[:] = torque_active

                self._apply_pending_hitter_ghost_snapshot_locked(
                    now_monotonic_s=time.monotonic(),
                )
                mujoco.mj_step(self.mujoco_model, self.mujoco_data)

        # Rendering.
        if self.viewer_enabled:
            self.render()

        # Optional real-time synchronization.
        is_real_time = getattr(self.cfg.control, 'real_time', False)
        if is_real_time:
            sim_elapsed = self.mujoco_data.time - self.sim_start_time
            real_elapsed = time.perf_counter() - self.real_start_time
            
            # Synchronize with wall time.
            if sim_elapsed > real_elapsed:
                time_to_wait = sim_elapsed - real_elapsed
                if time_to_wait > 0.0005: 
                    time.sleep(time_to_wait)

    def calibrate(self, refresh, init_ref_dof_pos=None):
        if refresh:
            default_qpos = self.default_qpos.copy()
            default_qvel = self.default_qvel

            # Use init_ref_dof_pos when provided; otherwise use default_angles.
            # default_qpos[7:] expects all joints, not only active joints.
            if init_ref_dof_pos is not None:
                # Start from complete default joint angles.
                current_default_angles = self.default_angles.copy()
                use_residual = getattr(self.cfg.control, 'use_residual', False)    
                if use_residual and init_ref_dof_pos is not None:
                    # Read residual joint indices from config.
                    if hasattr(self.cfg.control, 'residual_joint_indices') and self.cfg.control.residual_joint_indices is not None:
                        residual_joint_indices = self.cfg.control.residual_joint_indices
                        # Flatten init_ref_dof_pos if it is two-dimensional.
                        ref_dof_pos_flat = init_ref_dof_pos.flatten() if init_ref_dof_pos.ndim > 1 else init_ref_dof_pos
                        for idx in residual_joint_indices:
                            if idx < len(ref_dof_pos_flat) and idx < len(current_default_angles):
                                current_default_angles[idx] = ref_dof_pos_flat[idx]
                default_qpos[7:7 + self.num_dof] = current_default_angles
            else:
                # Use complete default_angles for all joints.
                default_qpos[7:7 + self.num_dof] = self.default_angles

            logger.info(f"Resetting envs with ref_dof_pos={[f'{x:.3f}' for x in default_qpos[7:7 + self.num_dof].tolist()]}")
            with self._viewer_lock():
                preserve_ball_state = bool(
                    getattr(
                        self,
                        "preserve_hitter_ball_state_on_calibrate",
                        False,
                    )
                    and self.table_tennis_enabled
                    and self.hitter_ball_qposadr is not None
                    and self.hitter_ball_qveladr is not None
                )
                if preserve_ball_state:
                    qposadr = self.hitter_ball_qposadr
                    qveladr = self.hitter_ball_qveladr
                    ball_qpos = self.mujoco_data.qpos[
                        qposadr:qposadr + 7
                    ].copy()
                    ball_qvel = self.mujoco_data.qvel[
                        qveladr:qveladr + 6
                    ].copy()
                self.mujoco_data.qpos[:] = default_qpos.copy()
                self.mujoco_data.qvel[:] = default_qvel
                if preserve_ball_state:
                    self.mujoco_data.qpos[qposadr:qposadr + 7] = ball_qpos
                    self.mujoco_data.qvel[qveladr:qveladr + 6] = ball_qvel
                self.mujoco_data.ctrl[:] = 0
                mujoco.mj_forward(self.mujoco_model, self.mujoco_data)
            self._update_hitter_ball_state()
        else:
            self.get_state()
            cur_dof_pos = self.dof_pos
            final_goal = np.zeros_like(self.default_angles[self.active_dof_idx])
            default_pos = self.default_angles[self.active_dof_idx]
            target = cur_dof_pos - default_pos
            target_seq=[]
            while np.max(np.abs(target-final_goal)) > 0.01:
                target-=np.clip((target-final_goal), -0.05, 0.05)
                target_seq += [copy.deepcopy(target)]
            for tgt in target_seq:
                next_tgt = tgt/self.cfg.control.action_scale
                self.apply_action(next_tgt[None,])
                self.get_state()
            logger.info(f'Simulation Done!')
            while True:
                self.apply_action(np.zeros((1, self.num_action)))
        self.reset_teleop()
    
    def check_termination(self):
        return abs(self.root_rpy[0]) > 1.2 or abs(self.root_rpy[1]) > 1.2
        # return False
