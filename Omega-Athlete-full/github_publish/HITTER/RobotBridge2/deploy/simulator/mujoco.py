import os

if "MUJOCO_GL" not in os.environ and not os.environ.get("DISPLAY"):
    os.environ["MUJOCO_GL"] = "egl"

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
from loguru import logger
from scipy.spatial.transform import Rotation as sRot
# from pynput import keyboard

DESIRED_BODY_INDICES = [0, 2, 4, 6, 8, 10, 12, 15, 17, 19, 22, 24, 26, 29]

class Mujoco(BaseSim):
    is_real = False
    def __init__(self, config):
        super().__init__(config)
        self.marker = self.cfg.get('marker', False)
        self.real_start_time = None
        self.sim_start_time = 0
        self.viewer = None
        self.viewer_enabled = bool(getattr(self.cfg.control, "viewer", False))
        self.playback_slowdown = max(1.0, float(getattr(self.cfg.control, "playback_slowdown", 1.0)))
        if self.viewer_enabled and not os.environ.get("DISPLAY"):
            logger.warning("[Mujoco] viewer=True but no DISPLAY is available; running headless.")
            self.viewer_enabled = False

        logger.info(f'Visualization Marker: {self.marker}')
        self.target_dof_pos = None
        if self.viewer_enabled:
            self._load_viewer()

        self._init_communication()
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
        self.lc = lcm.LCM('udpm://239.255.76.67:7667?ttl=255')

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
        self.table_tennis_enabled = bool(self.table_tennis_cfg and self.table_tennis_cfg.get("enabled", False))
        self.hitter_ball_body_id = -1
        self.hitter_ball_qposadr = None
        self.hitter_ball_qveladr = None
        self.ball_pos_world = np.zeros(3, dtype=np.float32)
        self.ball_quat_world = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32)
        self.ball_vel_world = np.zeros(3, dtype=np.float32)
        self.ball_ang_vel_world = np.zeros(3, dtype=np.float32)
        self.randomize_hitter_ball = False
        self.table_tennis_rng = np.random.default_rng()
        self.table_tennis_ball_trajectory_index = 0
        self.use_analytic_table_bounce = False
        self.table_tennis_table_center_xy = np.array([1.37, 0.0], dtype=np.float64)
        self.table_tennis_table_length = 2.74
        self.table_tennis_table_width = 1.525
        self.table_tennis_table_height = 0.76
        self.table_tennis_ball_radius = 0.02
        self.table_tennis_vertical_restitution = 0.8
        self.table_tennis_horizontal_restitution = 1.0
        self.use_analytic_racket_hit = False
        self.hitter_ball_geom_id = -1
        self.hitter_racket_face_geom_id = -1
        self.analytic_racket_hit_armed = False
        self.analytic_racket_hit_applied = False
        self.analytic_racket_hit_target_pos = np.zeros(3, dtype=np.float64)
        self.analytic_racket_hit_outgoing_vel = np.zeros(3, dtype=np.float64)
        self.analytic_racket_hit_target_radius = 0.08
        self.analytic_racket_hit_racket_radius = 0.20
        self.analytic_racket_hit_require_racket_near = True
        self.analytic_racket_hit_min_incoming_speed_x = 0.05
        self.analytic_racket_hit_snap_to_target = True
        self.analytic_racket_hit_separation = 0.03
        if not self.table_tennis_enabled:
            return
        self.randomize_hitter_ball = bool(self.table_tennis_cfg.get("randomize_ball_on_reset", False))
        self.table_tennis_rng = np.random.default_rng(self.table_tennis_cfg.get("ball_random_seed", None))
        self.use_analytic_table_bounce = bool(self.table_tennis_cfg.get("use_analytic_table_bounce", False))
        self.use_analytic_racket_hit = bool(self.table_tennis_cfg.get("use_analytic_racket_hit", False))
        table_center_xy = self.table_tennis_cfg.get(
            "table_center_xy_w", self.table_tennis_cfg.get("table_center_xy", [1.37, 0.0])
        )
        self.table_tennis_table_center_xy = np.asarray(table_center_xy, dtype=np.float64).reshape(2)
        self.table_tennis_table_length = float(self.table_tennis_cfg.get("table_length", 2.74))
        self.table_tennis_table_width = float(self.table_tennis_cfg.get("table_width", 1.525))
        self.table_tennis_table_height = float(self.table_tennis_cfg.get("table_height", 0.76))
        self.table_tennis_ball_radius = float(self.table_tennis_cfg.get("ball_radius", 0.02))
        self.table_tennis_vertical_restitution = float(self.table_tennis_cfg.get("vertical_restitution", 0.8))
        self.table_tennis_horizontal_restitution = float(self.table_tennis_cfg.get("horizontal_restitution", 1.0))
        self.analytic_racket_hit_target_radius = float(self.table_tennis_cfg.get("analytic_racket_hit_target_radius", 0.08))
        self.analytic_racket_hit_racket_radius = float(self.table_tennis_cfg.get("analytic_racket_hit_racket_radius", 0.20))
        self.analytic_racket_hit_require_racket_near = bool(
            self.table_tennis_cfg.get("analytic_racket_hit_require_racket_near", True)
        )
        self.analytic_racket_hit_min_incoming_speed_x = float(
            self.table_tennis_cfg.get("analytic_racket_hit_min_incoming_speed_x", 0.05)
        )
        self.analytic_racket_hit_snap_to_target = bool(self.table_tennis_cfg.get("analytic_racket_hit_snap_to_target", True))
        self.analytic_racket_hit_separation = float(self.table_tennis_cfg.get("analytic_racket_hit_separation", 0.03))

        ball_body_name = self.table_tennis_cfg.get("ball_body_name", "hitter_ball")
        ball_joint_name = self.table_tennis_cfg.get("ball_joint_name", "hitter_ball_freejoint")
        ball_geom_name = self.table_tennis_cfg.get("ball_geom_name", "hitter_ball_geom")
        racket_face_geom_name = self.table_tennis_cfg.get("racket_face_geom_name", "right_racket_face_collision")
        self.hitter_ball_body_id = mujoco.mj_name2id(self.mujoco_model, mujoco.mjtObj.mjOBJ_BODY, ball_body_name)
        ball_joint_id = mujoco.mj_name2id(self.mujoco_model, mujoco.mjtObj.mjOBJ_JOINT, ball_joint_name)
        self.hitter_ball_geom_id = mujoco.mj_name2id(self.mujoco_model, mujoco.mjtObj.mjOBJ_GEOM, ball_geom_name)
        self.hitter_racket_face_geom_id = mujoco.mj_name2id(
            self.mujoco_model,
            mujoco.mjtObj.mjOBJ_GEOM,
            racket_face_geom_name,
        )
        if self.hitter_ball_body_id < 0 or ball_joint_id < 0:
            logger.warning(
                f"[Mujoco] table_tennis.enabled=True but ball body/joint not found: {ball_body_name}/{ball_joint_name}"
            )
            self.table_tennis_enabled = False
            return
        if self.use_analytic_racket_hit and (self.hitter_ball_geom_id < 0 or self.hitter_racket_face_geom_id < 0):
            logger.warning(
                "[Mujoco] use_analytic_racket_hit=True but ball/racket geom not found: {}/{}",
                ball_geom_name,
                racket_face_geom_name,
            )
            self.use_analytic_racket_hit = False

        self.hitter_ball_qposadr = int(self.mujoco_model.jnt_qposadr[ball_joint_id])
        self.hitter_ball_qveladr = int(self.mujoco_model.jnt_dofadr[ball_joint_id])
        self.reset_hitter_ball(update_default=True)

    def _hitter_ball_inside_table(self, xy: np.ndarray) -> bool:
        half = np.array([0.5 * self.table_tennis_table_length, 0.5 * self.table_tennis_table_width], dtype=np.float64)
        return bool(np.all(np.abs(xy - self.table_tennis_table_center_xy) <= half))

    def _apply_hitter_ball_analytic_table_bounce(self, prev_ball_vel: np.ndarray) -> bool:
        if (
            not self.table_tennis_enabled
            or not self.use_analytic_table_bounce
            or self.hitter_ball_qposadr is None
            or self.hitter_ball_qveladr is None
        ):
            return False
        if prev_ball_vel[2] >= 0.0:
            return False

        qposadr = self.hitter_ball_qposadr
        qveladr = self.hitter_ball_qveladr
        pos = self.mujoco_data.qpos[qposadr:qposadr + 3]
        vel = self.mujoco_data.qvel[qveladr:qveladr + 3]
        contact_z = self.table_tennis_table_height + self.table_tennis_ball_radius
        if pos[2] > contact_z or vel[2] >= 0.0 or not self._hitter_ball_inside_table(pos[:2]):
            return False

        pos[2] = contact_z
        vel[:2] *= self.table_tennis_horizontal_restitution
        vel[2] = -vel[2] * self.table_tennis_vertical_restitution
        mujoco.mj_forward(self.mujoco_model, self.mujoco_data)
        return True

    def clear_hitter_analytic_racket_hit(self) -> None:
        self.analytic_racket_hit_armed = False
        self.analytic_racket_hit_applied = False
        self.analytic_racket_hit_target_pos[:] = 0.0
        self.analytic_racket_hit_outgoing_vel[:] = 0.0

    def set_hitter_analytic_racket_hit(self, target_pos, outgoing_vel) -> None:
        if not self.table_tennis_enabled or not self.use_analytic_racket_hit:
            return
        if self.analytic_racket_hit_applied:
            return
        target_pos = np.asarray(target_pos, dtype=np.float64).reshape(-1)
        outgoing_vel = np.asarray(outgoing_vel, dtype=np.float64).reshape(-1)
        if target_pos.shape != (3,) or outgoing_vel.shape != (3,):
            raise ValueError("analytic racket hit target_pos/outgoing_vel must both contain 3 values.")
        if not np.isfinite(target_pos).all() or not np.isfinite(outgoing_vel).all():
            raise ValueError("analytic racket hit target_pos/outgoing_vel must be finite.")
        self.analytic_racket_hit_target_pos = target_pos.copy()
        self.analytic_racket_hit_outgoing_vel = outgoing_vel.copy()
        self.analytic_racket_hit_armed = True

    def _hitter_ball_racket_contact_active(self) -> bool:
        if self.hitter_ball_geom_id < 0 or self.hitter_racket_face_geom_id < 0:
            return False
        for contact_index in range(self.mujoco_data.ncon):
            contact = self.mujoco_data.contact[contact_index]
            if {
                int(contact.geom1),
                int(contact.geom2),
            } == {
                self.hitter_ball_geom_id,
                self.hitter_racket_face_geom_id,
            }:
                return True
        return False

    @staticmethod
    def _point_segment_distance(point: np.ndarray, start: np.ndarray, end: np.ndarray) -> float:
        segment = end - start
        length_sq = float(np.dot(segment, segment))
        if length_sq < 1.0e-12:
            return float(np.linalg.norm(point - end))
        alpha = float(np.clip(np.dot(point - start, segment) / length_sq, 0.0, 1.0))
        closest = start + alpha * segment
        return float(np.linalg.norm(point - closest))

    def _hitter_racket_near_ball(self, ball_pos: np.ndarray) -> bool:
        if self._hitter_ball_racket_contact_active():
            return True
        if self.hitter_racket_face_geom_id < 0:
            return False
        racket_pos = self.mujoco_data.geom_xpos[self.hitter_racket_face_geom_id]
        return bool(np.linalg.norm(ball_pos - racket_pos) <= self.analytic_racket_hit_racket_radius)

    def _apply_hitter_ball_analytic_racket_hit(self, prev_ball_pos: np.ndarray, prev_ball_vel: np.ndarray) -> bool:
        if (
            not self.table_tennis_enabled
            or not self.use_analytic_racket_hit
            or not self.analytic_racket_hit_armed
            or self.analytic_racket_hit_applied
            or self.hitter_ball_qposadr is None
            or self.hitter_ball_qveladr is None
        ):
            return False
        if prev_ball_vel[0] >= -self.analytic_racket_hit_min_incoming_speed_x:
            return False

        qposadr = self.hitter_ball_qposadr
        qveladr = self.hitter_ball_qveladr
        ball_pos = self.mujoco_data.qpos[qposadr:qposadr + 3]
        target_pos = self.analytic_racket_hit_target_pos
        distance_to_target = self._point_segment_distance(target_pos, prev_ball_pos, ball_pos.copy())
        if distance_to_target > self.analytic_racket_hit_target_radius:
            return False
        if self.analytic_racket_hit_require_racket_near and not self._hitter_racket_near_ball(ball_pos):
            return False

        outgoing_vel = self.analytic_racket_hit_outgoing_vel.copy()
        if self.analytic_racket_hit_snap_to_target:
            ball_pos[:] = target_pos
            speed = float(np.linalg.norm(outgoing_vel))
            if speed > 1.0e-9 and self.analytic_racket_hit_separation > 0.0:
                ball_pos[:] = ball_pos + outgoing_vel / speed * self.analytic_racket_hit_separation
        self.mujoco_data.qvel[qveladr:qveladr + 3] = outgoing_vel
        self.mujoco_data.qvel[qveladr + 3:qveladr + 6] = 0.0
        self.analytic_racket_hit_applied = True
        self.analytic_racket_hit_armed = False
        mujoco.mj_forward(self.mujoco_model, self.mujoco_data)
        logger.info(
            "[Mujoco] Applied analytic racket hit: target_pos={}, outgoing_vel={}, target_dist={:.4f}",
            target_pos.astype(np.float32),
            outgoing_vel.astype(np.float32),
            distance_to_target,
        )
        return True

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
        if not self.table_tennis_enabled or self.hitter_ball_qposadr is None or self.hitter_ball_qveladr is None:
            return

        randomize = self.randomize_hitter_ball and not update_default
        trajectory_name = None
        trajectory = self._sample_table_tennis_trajectory() if randomize and pos is None and lin_vel is None else None
        if trajectory is not None:
            trajectory_name, pos, lin_vel, ang_vel = trajectory
        elif pos is None:
            pos = self._sample_table_tennis_vector("ball_initial_pos", [1.80, 0.0, 1.05], 3, randomize=randomize)
        quat = self.table_tennis_cfg.get("ball_initial_quat_wxyz", [1.0, 0.0, 0.0, 0.0])
        if lin_vel is None:
            lin_vel = self._sample_table_tennis_vector("ball_initial_lin_vel", [-2.2, 0.0, 0.2], 3, randomize=randomize)
        if trajectory is None:
            ang_vel = self._sample_table_tennis_vector("ball_initial_ang_vel", [0.0, 0.0, 0.0], 3, randomize=randomize)

        qposadr = self.hitter_ball_qposadr
        qveladr = self.hitter_ball_qveladr
        with self._viewer_lock():
            self.clear_hitter_analytic_racket_hit()
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

    def _update_hitter_ball_state(self):
        if not self.table_tennis_enabled or self.hitter_ball_body_id < 0:
            return
        self.ball_pos_world = self.mujoco_data.xpos[self.hitter_ball_body_id].astype(np.float32).copy()
        self.ball_quat_world = self.mujoco_data.xquat[self.hitter_ball_body_id][[1, 2, 3, 0]].astype(np.float32).copy()
        qveladr = self.hitter_ball_qveladr
        self.ball_vel_world = self.mujoco_data.qvel[qveladr:qveladr + 3].astype(np.float32).copy()
        self.ball_ang_vel_world = self.mujoco_data.qvel[qveladr + 3:qveladr + 6].astype(np.float32).copy()

    def _init_communication(self):
        self.firstReceiveAlarm = False
        self.firstReceiveOdometer = False
        self._init_time = time.time()
        
        self.teleop_state_subscriber = self.lc.subscribe('camera_reference_data', self._teleop_state_handler)

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

                prev_ball_pos = None
                prev_ball_vel = None
                if (
                    self.table_tennis_enabled
                    and (self.use_analytic_table_bounce or self.use_analytic_racket_hit)
                    and self.hitter_ball_qposadr is not None
                    and self.hitter_ball_qveladr is not None
                ):
                    qposadr = self.hitter_ball_qposadr
                    qveladr = self.hitter_ball_qveladr
                    prev_ball_pos = self.mujoco_data.qpos[qposadr:qposadr + 3].copy()
                    prev_ball_vel = self.mujoco_data.qvel[qveladr:qveladr + 3].copy()

                mujoco.mj_step(self.mujoco_model, self.mujoco_data)

                if prev_ball_pos is not None and prev_ball_vel is not None:
                    hit_applied = self._apply_hitter_ball_analytic_racket_hit(prev_ball_pos, prev_ball_vel)
                    if not hit_applied:
                        self._apply_hitter_ball_analytic_table_bounce(prev_ball_vel)

        # Rendering.
        if self.viewer_enabled:
            self.render()

        # Optional real-time synchronization.
        is_real_time = getattr(self.cfg.control, 'real_time', False)
        if is_real_time:
            sim_elapsed = (self.mujoco_data.time - self.sim_start_time) * self.playback_slowdown
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
                self.mujoco_data.qpos[:] = default_qpos.copy()
                self.mujoco_data.qvel[:] = default_qvel
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
