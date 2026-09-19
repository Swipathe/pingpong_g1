from __future__ import annotations

import time
import lcm
import copy
import select
import threading
from typing import Callable
import numpy as np
from simulator.base_sim import BaseSim
from loguru import logger
from scipy.spatial.transform import Rotation as sRot

import sys
sys.path.append('../')
from unitree_sdk2.lcm_types.body_control_data_lcmt import body_control_data_lcmt
from unitree_sdk2.lcm_types.rc_command_lcmt import rc_command_lcmt
from unitree_sdk2.lcm_types.state_estimator_lcmt import state_estimator_lcmt
from unitree_sdk2.lcm_types.pd_tau_targets_lcmt import pd_tau_targets_lcmt
from unitree_sdk2.lcm_types.transformation_t import transformation_t

from utils.helpers import get_gravity, get_rpy
from utils.hitter_ball_pipeline import (
    BasePoseW,
    RealtimeViconBallPipeline,
)
from utils.hitter_realtime import BallEstimateSnapshot
from utils.hitter_runtime_capabilities import (
    REAL_WORLD_HITTER_RUNTIME_CAPABILITIES,
)

class RealWorld(BaseSim):
    is_real = True
    hitter_runtime_capabilities = REAL_WORLD_HITTER_RUNTIME_CAPABILITIES

    def __init__(self, config):
        super().__init__(config)
        self._init_communication()
        self.spin()
        self.sync = True
        while True:
            if self.connected():
                break
            time.sleep(0.001)
    
    def _setup(self):
        super()._setup()
        self.lc = lcm.LCM('udpm://239.255.76.67:7667?ttl=255')
    
    def _load_asset(self):
        super()._load_asset()
        self.joint_serial_num = self.cfg.asset.joint_order
        self.policy_joint_order = list(self.joint_serial_num.keys())
        # index for policy to robot
        self.idx_r2p=[self.joint_serial_num[k] for k in self.joint_serial_num]
        # index for robot to policy
        self.idx_p2r=[self.idx_r2p.index(i) for i in range(self.num_dof)]

    def _init_low_state(self):
        super()._init_low_state()

        # Initialize ref_dof_pos for residual control and target_dof_pos
        self.ref_dof_pos = None
        self.target_dof_pos = None

        # remote controller part
        self.mode = 0
        self.ctrlmode_left = 0
        self.ctrlmode_right = 0
        self.left_stick = [0, 0]
        self.right_stick = [0, 0]
        self.left_upper_switch = 0
        self.left_lower_left_switch = 0
        self.left_lower_right_switch = 0
        self.right_upper_switch = 0
        self.right_lower_left_switch = 0
        self.right_lower_right_switch = 0
        self.left_upper_switch_pressed = 0
        self.left_lower_left_switch_pressed = 0
        self.left_lower_right_switch_pressed = 0
        self.right_upper_switch_pressed = 0
        self.right_lower_left_switch_pressed = 0
        self.right_lower_right_switch_pressed = 0

        # LCM callbacks are asynchronous; keep safe defaults so startup can
        # call get_state() before every stream has delivered its first packet.
        self.root_trans_tmp = np.zeros(3, dtype=np.float32)
        self.root_rpy_tmp = np.zeros(3, dtype=np.float32)
        self.root_quat_tmp = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32)
        self.base_lin_vel_tmp = np.zeros(3, dtype=np.float32)
        self.base_ang_vel_tmp = np.zeros(3, dtype=np.float32)
        self.root_trans_world_tmp = np.zeros(3, dtype=np.float32)
        self.root_quat_world_tmp = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32)
        self.dof_pos_tmp = np.zeros(self.num_dof, dtype=np.float32)
        self.dof_vel_tmp = np.zeros(self.num_dof, dtype=np.float32)

        # Heading reset (yaw-only) to avoid large initial yaw correction when IMU world frame
        # (initialized at boot) is different from robot orientation at policy start.
        self._heading_inv_rot = None  # scipy Rotation
        self._reset_heading_on_start = bool(getattr(self.cfg.control, "reset_heading_on_start", True))

        self._init_ball_state()
        self.terminate_on_r2 = bool(getattr(self.cfg.control, "terminate_on_r2", True))
        logger.info(f"[RealWorld] terminate_on_r2={self.terminate_on_r2}")

    def _init_ball_state(self) -> None:
        self._hitter_ball_state_lock = threading.RLock()
        if not hasattr(self, "root_trans_world_tmp"):
            self.root_trans_world_tmp = np.zeros(3, dtype=np.float32)
        if not hasattr(self, "root_quat_world_tmp"):
            self.root_quat_world_tmp = np.array(
                [0.0, 0.0, 0.0, 1.0],
                dtype=np.float32,
            )
        self.ball_pos_world = np.zeros(3, dtype=np.float32)
        self.ball_vel_world = np.zeros(3, dtype=np.float32)
        self.ball_visible = False
        self.base_pose_valid = False
        self.base_pose_valid_tmp = False
        self.firstReceiveBall = False
        self._last_base_pose_wait_log_monotonic_s = None
        planner_cfg = {}
        motion_cfg = getattr(self.cfg, "motion", None)
        if motion_cfg is not None:
            planner_cfg = motion_cfg.get("ball_planner", {}) or {}
        self._hitter_ball_pipeline = RealtimeViconBallPipeline(
            planner_cfg,
            self._copy_base_pose_for_ball_pipeline,
            stale_timeout_s=None,
        )
        self.ball_state_estimator_ready = False
        self.ball_state_estimator_sample_count = 0
        self.ball_state_estimator_min_samples = self.ball_state_estimator.min_samples

    @property
    def ball_pos_world_tmp(self) -> np.ndarray:
        return self._hitter_ball_pipeline.state().position_w

    @property
    def ball_vel_world_tmp(self) -> np.ndarray:
        return self._hitter_ball_pipeline.state().velocity_w

    @property
    def ball_visible_tmp(self) -> bool:
        return bool(self._hitter_ball_pipeline.state().visible)

    @property
    def ball_state_estimator_ready_tmp(self) -> bool:
        return bool(self._hitter_ball_pipeline.state().ready)

    @property
    def ball_state_estimator_sample_count_tmp(self) -> int:
        return int(self._hitter_ball_pipeline.state().sample_count)

    @property
    def ball_track_epoch(self) -> int:
        return int(self._hitter_ball_pipeline.state().track_epoch)

    @property
    def ball_snapshot_generation(self) -> int:
        return int(self._hitter_ball_pipeline.state().generation)

    @property
    def latest_ball_snapshot(self) -> BallEstimateSnapshot | None:
        return self._hitter_ball_pipeline.state().latest_snapshot

    @property
    def ball_state_estimator(self):
        return self._hitter_ball_pipeline.ball_state_estimator

    @property
    def ball_state_estimator_sample_rate_hz(self) -> float:
        return float(self._hitter_ball_pipeline.estimator_sample_rate_hz)

    @property
    def ball_state_estimator_time(self) -> float | None:
        return self._hitter_ball_pipeline.last_estimator_timestamp_s

    @property
    def ball_state_estimator_last_host_time(self) -> float | None:
        return self._hitter_ball_pipeline.state().last_received_monotonic_s

    def _copy_base_pose_for_ball_pipeline(self) -> BasePoseW:
        with self._hitter_ball_state_lock:
            return BasePoseW(
                position_w=self.root_trans_world_tmp,
                quaternion_xyzw=self.root_quat_world_tmp,
                valid=bool(self.base_pose_valid_tmp),
                simulation_time_s=0.0,
                captured_monotonic_s=time.monotonic(),
            )

    def register_hitter_ball_listener(
        self,
        listener: Callable[[BallEstimateSnapshot], None],
    ) -> Callable[[], None]:
        return self._hitter_ball_pipeline.register_listener(listener)

    def reset_ball_state_estimator(self) -> None:
        self._hitter_ball_pipeline.reset_after_strike()
        self.ball_state_estimator_ready = False
        self.ball_state_estimator_sample_count = 0
        self.ball_visible = False
        logger.info("[RealWorld] ball state estimator reset.")

    def _clear_ball_tracking(
        self,
        *,
        emit_snapshot: bool = False,
        source_message=None,
        received_monotonic_s: float | None = None,
        advance_epoch: bool = True,
    ) -> None:
        if not advance_epoch:
            raise ValueError(
                "advance_epoch=False is not supported by the ordered "
                "RealWorld ball reset"
            )
        self.reset_ball_state_estimator()
        if emit_snapshot:
            if source_message is None:
                raise ValueError(
                    "source_message is required when emitting a snapshot"
                )
            self._hitter_ball_pipeline.ingest_transformation(
                source_message,
                received_monotonic_s=received_monotonic_s,
            )

    def reset_heading(self):
        """Reset heading (yaw only) so that current root_quat becomes heading-zero for policy."""
        quat_xyzw = np.asarray(self.root_quat_tmp, dtype=np.float64).reshape(-1)[:4]
        try:
            yaw = float(sRot.from_quat(quat_xyzw).as_euler("xyz", degrees=False)[2])
        except Exception:
            yaw = 0.0
        self._heading_inv_rot = sRot.from_euler("z", -yaw, degrees=False)
        logger.info(f"[RealWorld] heading reset enabled, yaw0={yaw:.3f} rad")
    
    def _init_communication(self):
        self._poll_stop_event = threading.Event()
        self._communication_close_lock = threading.Lock()
        self._communication_closed = False
        self.firstReceiveAlarm = False
        self.firstReceiveOdometer = False
        self.firstReceiveVicon = False
        self._init_time = time.time()
        
        self.root_state_subscriber = self.lc.subscribe('state_estimator_data', self._root_state_handler)
        self.joint_state_subscriber = self.lc.subscribe('body_control_data', self._joint_state_handler)
        self.vicon_state_subscriber = self.lc.subscribe('vicon_state_data', self._vicon_state_handler)
        self.remote_controller_subscriber = self.lc.subscribe('rc_command_data', self._remote_controller_handler)
        self.teleop_state_subscriber = self.lc.subscribe('camera_reference_data', self._teleop_state_handler)

    def _root_state_handler(self, channel, data):
        msg = state_estimator_lcmt.decode(data)
        if not self.firstReceiveOdometer:
            self.firstReceiveOdometer = True
            logger.info('State Estimator Information Received!')
            logger.info(f'Root Translation: {np.array(msg.p)}, Root Linear Velocity: {np.array(msg.vBody)}')
        self.root_trans_tmp = np.array(msg.p)
        self.root_rpy_tmp = np.array(msg.rpy)
        self.root_quat_tmp = np.roll(np.array(msg.quat), -1) # (w,x,y,z) -> (x,y,z,w)
        self.base_lin_vel_tmp = np.array(msg.vBody)
        self.base_ang_vel_tmp = np.array(msg.omegaBody)

    def _vicon_state_handler(self, channel, data):
        msg = transformation_t.decode(data)
        name = str(getattr(msg, "name", "") or "").strip().lower()
        pos_raw = np.asarray(msg.pos_vicon, dtype=np.float64).reshape(-1)[:3]
        quat = np.asarray(msg.quat_vicon, dtype=np.float32).reshape(-1)[:4]
        vicon_time_s = float(getattr(msg, "vicon_time_s", 0.0) or 0.0)
        vicon_frame_number = int(getattr(msg, "vicon_frame_number", 0) or 0)
        if name == "ball":
            print(f"[VICON] name={name}, vicon_time_s={vicon_time_s:.2f}, vicon_frame_number={vicon_frame_number}, pos={pos_raw}, quat={quat}, valid={getattr(msg, 'valid', 1)}, occluded={getattr(msg, 'occluded', 0)}")
        if name == "ball":
            self._update_ball_state_from_vicon(msg, pos_raw)
            return

        if name == "table":
            return

        if name != "g2pelvis":
            return

        pos = pos_raw.astype(np.float32)
        valid = bool(getattr(msg, "valid", 1)) and not bool(
            getattr(msg, "occluded", 0)
        )
        quaternion_norm = float(
            np.linalg.norm(quat.astype(np.float64))
        )
        first_valid_pose = False
        with self._hitter_ball_state_lock:
            if (
                not valid
                or pos.shape != (3,)
                or quat.shape != (4,)
                or not np.isfinite(pos).all()
                or not np.isfinite(quat).all()
                or not np.isfinite(quaternion_norm)
                or quaternion_norm <= 1.0e-12
            ):
                self.base_pose_valid_tmp = False
                return

            if not self.firstReceiveVicon:
                self.firstReceiveVicon = True
                first_valid_pose = True
            self.root_trans_world_tmp = pos.copy()
            self.root_quat_world_tmp = (
                quat.astype(np.float64) / quaternion_norm
            ).astype(np.float32)
            self.base_pose_valid_tmp = True
        if first_valid_pose:
            logger.info('Vicon root information received!')
            logger.info(f'Root Translation World: {pos}')

    def _update_ball_state_from_vicon(self, msg, pos: np.ndarray) -> None:
        received_monotonic_s = time.monotonic()
        first_ball_position = None
        try:
            update = (
                self._hitter_ball_pipeline.ingest_transformation_update(
                    msg,
                    received_monotonic_s=received_monotonic_s,
                )
            )
        except ValueError as exc:
            logger.warning(
                "[RealWorld] rejected invalid Vicon ball sample: {}",
                exc,
            )
            return
        snapshot = update.snapshot
        if snapshot is not None and snapshot.visible and not self.firstReceiveBall:
            self.firstReceiveBall = True
            first_ball_position = snapshot.position_w.copy()
        if update.bounce_detected:
            logger.info(
                "[RealWorld] Vicon ball table bounce detected; cleared polynomial fit buffer."
            )
        if first_ball_position is not None:
            logger.info('Vicon ball information received!')
            logger.info(f'Ball Position World: {first_ball_position}')

    def _joint_state_handler(self, channel, data):
        msg = body_control_data_lcmt.decode(data)
        if not self.firstReceiveAlarm:
            self.time_delay = time.time() - self._init_time
            self.firstReceiveAlarm = True
            logger.info("Communication build successfully between the policy and the transition layer!")
            logger.info(f'First signal arrives after {self.time_delay}s!')
        self.dof_pos_tmp = np.array(msg.q)[self.idx_r2p]
        self.dof_vel_tmp = np.array(msg.qd)[self.idx_r2p]
    
    def _remote_controller_handler(self, channel, data):
        msg = rc_command_lcmt.decode(data)
        
        self.left_upper_switch_pressed = ((msg.left_upper_switch and not self.left_upper_switch) or self.left_upper_switch_pressed)
        self.left_lower_left_switch_pressed = ((msg.left_lower_left_switch and not self.left_lower_left_switch) or self.left_lower_left_switch_pressed)
        self.left_lower_right_switch_pressed = ((msg.left_lower_right_switch and not self.left_lower_right_switch) or self.left_lower_right_switch_pressed)
        self.right_upper_switch_pressed = ((msg.right_upper_switch and not self.right_upper_switch) or self.right_upper_switch_pressed)
        self.right_lower_left_switch_pressed = ((msg.right_lower_left_switch and not self.right_lower_left_switch) or self.right_lower_left_switch_pressed)
        self.right_lower_right_switch_pressed = ((msg.right_lower_right_switch) and not self.right_lower_right_switch) or self.right_lower_right_switch_pressed

        self.mode = msg.mode
        self.right_stick = msg.right_stick
        self.left_stick = msg.left_stick
        self.left_upper_switch = msg.left_upper_switch
        self.left_lower_left_switch = msg.left_lower_left_switch
        self.left_lower_right_switch = msg.left_lower_right_switch
        self.right_upper_switch = msg.right_upper_switch
        self.right_lower_left_switch = msg.right_lower_left_switch
        self.right_lower_right_switch = msg.right_lower_right_switch

    def check_teleop_sync(self):
        if self.right_upper_switch_pressed:
            self.sync = not self.sync
            self.right_upper_switch_pressed = False
            print(f"==================== right_upper_switch_pressed ===================")
            print(f" {self.sync} ")

    def get_state(self):
        self.root_trans = self.root_trans_tmp.copy()
        self.base_lin_vel = self.base_lin_vel_tmp.copy()
        self.root_quat = self.root_quat_tmp.copy()
        ball_pipeline_state = self._hitter_ball_pipeline.state()
        with self._hitter_ball_state_lock:
            self.root_trans_world = self.root_trans_world_tmp.copy()
            self.root_quat_world = self.root_quat_world_tmp.copy()
            self.ball_pos_world = ball_pipeline_state.position_w.copy()
            self.ball_vel_world = ball_pipeline_state.velocity_w.copy()
            self.ball_visible = bool(ball_pipeline_state.visible)
            self.ball_state_estimator_ready = bool(
                ball_pipeline_state.ready
            )
            self.ball_state_estimator_sample_count = int(
                ball_pipeline_state.sample_count
            )
            self.base_pose_valid = bool(self.base_pose_valid_tmp)
        self.root_rpy = self.root_rpy_tmp.copy()
        self.base_ang_vel = self.base_ang_vel_tmp.copy()

        # Apply yaw-only heading reset for policy, if enabled.
        if self._heading_inv_rot is not None:
            try:
                q = self._heading_inv_rot * sRot.from_quat(np.asarray(self.root_quat, dtype=np.float64).reshape(-1)[:4])
                self.root_quat = q.as_quat().astype(np.float32)
                # Keep rpy consistent with the adjusted quaternion
                self.root_rpy = sRot.from_quat(self.root_quat).as_euler("xyz", degrees=False).astype(np.float32)
            except Exception as exc:
                logger.warning(f"[RealWorld] failed to apply heading reset: {exc}")

        self.projected_gravity = get_gravity(self.root_quat, w_last=True)
        self.dof_pos = self.dof_pos_tmp.copy()[self.active_dof_idx]
        self.dof_vel = self.dof_vel_tmp.copy()[self.active_dof_idx]

        if self.cfg.control.update_with_fk:
            fk_info, fk_info_tensor = self.fk()
            self.torso_quat = fk_info[self.torso_name]['quat']
            self.torso_trans = fk_info[self.torso_name]['pos']
            self.robot_fk_info = fk_info_tensor

        if hasattr(self.cfg.control, 'use_teleop'):
            if self._init_teleop:
                if self.cfg.control.use_teleop:
                    self.check_teleop_sync()
                    if self.sync:
                        idx_r2p = [
                            0, 6, 12, 1, 7, 13, 2, 8, 14, 3, 9, 15, 22, 4, 10, 16, 23, 5, 11, 17, 24, 18, 25, 19, 26, 20, 27, 21, 28
                        ]
                        self.teleop_dof_pos = self.teleop_dof_pos_tmp.copy()[idx_r2p]
                        self.teleop_quat = self.align_quat(self.teleop_quat_tmp.copy())
                        if self.cfg.control.update_with_fk:
                            fk_info, _ = self.fk_teleop()
                            self.teleop_quat = fk_info[self.torso_name]['quat']

    def apply_action(self, action):
        self.act = action.copy()
        action = action.squeeze(0)
        is_mosaic = getattr(self.cfg.control, 'is_mosaic', False)
        if is_mosaic:
            dof_target_pos = action.copy()
        else:
            # Update default joint angles from reference motion if enabled and available
            current_default_angles = self.default_angles[self.active_dof_idx].copy()
            
            # Check if residual control is enabled in config
            use_residual = getattr(self.cfg.control, 'use_residual', False)
            
            if use_residual and self.ref_dof_pos is not None:
                # Get residual joint indices from config
                if hasattr(self.cfg.control, 'residual_joint_indices') and self.cfg.control.residual_joint_indices is not None:
                    residual_joint_indices = self.cfg.control.residual_joint_indices
                    # Flatten ref_dof_pos if it's 2D
                    ref_dof_pos_flat = self.ref_dof_pos.flatten() if self.ref_dof_pos.ndim > 1 else self.ref_dof_pos
                    for idx in residual_joint_indices:
                        if idx < len(ref_dof_pos_flat) and idx < len(current_default_angles):
                            current_default_angles[idx] = ref_dof_pos_flat[idx]
            
            tgt_dof_pos = current_default_angles + action*self.cfg.control.action_scale
            self.target_dof_pos = tgt_dof_pos  # Store target positions for monitoring
            
            dof_target_pos = self.default_angles.copy()
            dof_target_pos[self.active_dof_idx] = tgt_dof_pos
            dof_target_pos = dof_target_pos[self.idx_p2r]

        cmd = pd_tau_targets_lcmt()
        cmd.q_des = dof_target_pos.copy()
        cmd.qd_des = np.zeros_like(dof_target_pos)
        cmd.kp = self.kps.copy()[self.idx_p2r]
        cmd.kd = self.kds.copy()[self.idx_p2r]
        cmd.tau_ff = np.zeros_like(dof_target_pos)
        cmd.se_contactState = np.zeros(2)
        cmd.timestamp_us = int(time.time()*10**6)
        
        self.lc.publish("pd_plustau_targets", cmd.encode())

    def _calibration_action_from_residual(self, residual, default_pos):
        if getattr(self.cfg.control, 'is_mosaic', False):
            return default_pos + residual
        return residual / self.cfg.control.action_scale

    def _policy_start_base_pose_ready(self, *, now: float | None = None) -> bool:
        if bool(getattr(self, "base_pose_valid_tmp", False)):
            return True
        if now is None:
            now = time.monotonic()
        last_log = getattr(
            self,
            "_last_base_pose_wait_log_monotonic_s",
            None,
        )
        if last_log is None or float(now) - last_log >= 1.0:
            logger.warning(
                "[RealWorld] waiting for a valid G2Pelvis world pose before "
                "entering the policy loop"
            )
            self._last_base_pose_wait_log_monotonic_s = float(now)
        return False

    def poll(self, cb=None):
        try:
            while not self._poll_stop_event.is_set():
                timeout = 0.01
                rfds, wfds, efds = select.select([self.lc.fileno()], [], [], timeout)
                if rfds:
                    self.lc.handle()
                else:
                    continue
        except KeyboardInterrupt:
            pass

    def spin(self):
        self._poll_stop_event.clear()
        self.run_thread = threading.Thread(target=self.poll, daemon=True)
        self.run_thread.start()

    def close(self) -> bool:
        with self._communication_close_lock:
            if self._communication_closed:
                return True
            self._poll_stop_event.set()
            run_thread = getattr(self, "run_thread", None)
            if (
                run_thread is not None
                and run_thread is not threading.current_thread()
                and run_thread.is_alive()
            ):
                run_thread.join(timeout=1.0)
                if run_thread.is_alive():
                    logger.warning(
                        "[RealWorld] LCM poll thread did not stop within 1.0 s"
                    )
                    return False
            for subscription in (
                self.root_state_subscriber,
                self.joint_state_subscriber,
                self.vicon_state_subscriber,
                self.remote_controller_subscriber,
                self.teleop_state_subscriber,
            ):
                self.lc.unsubscribe(subscription)
            ball_pipeline = getattr(
                self,
                "_hitter_ball_pipeline",
                None,
            )
            if ball_pipeline is not None:
                ball_pipeline.close()
            self._communication_closed = True
            return True

    def connected(self):
        return self.firstReceiveAlarm
    
    def calibrate(self, refresh, init_ref_dof_pos=None):
        self.get_state()
        if refresh:
            # Handle init_ref_dof_pos if provided
            current_default_angles = self.default_angles.copy()[self.active_dof_idx]
            if init_ref_dof_pos is not None:
                use_residual = getattr(self.cfg.control, 'use_residual', False)    
                if use_residual and init_ref_dof_pos is not None:
                    # Get residual joint indices from config
                    if hasattr(self.cfg.control, 'residual_joint_indices') and self.cfg.control.residual_joint_indices is not None:
                        residual_joint_indices = self.cfg.control.residual_joint_indices
                        # Flatten init_ref_dof_pos if it's 2D
                        ref_dof_pos_flat = init_ref_dof_pos.flatten() if init_ref_dof_pos.ndim > 1 else init_ref_dof_pos
                        for idx in residual_joint_indices:
                            if idx < len(ref_dof_pos_flat) and idx < len(current_default_angles):
                                current_default_angles[idx] = ref_dof_pos_flat[idx]
            
            logger.info('Calibraiting..., Press R2 to continue')
            while True:
                if self.right_lower_right_switch_pressed:
                    logger.info('R2 button pressed, Start Calibrating...')
                    self.right_lower_right_switch_pressed = False
                    break

            cur_dof_pos = self.dof_pos
            final_goal = np.zeros_like(current_default_angles)
            default_pos = current_default_angles.copy()
            target = cur_dof_pos - default_pos
            target_seq = []
            while np.max(np.abs(target-final_goal))>0.01:
                target-=np.clip((target-final_goal), -0.05, 0.05)
                target_seq += [copy.deepcopy(target)]
            for tgt in target_seq:
                step_start=time.time()
                next_tgt = self._calibration_action_from_residual(tgt, default_pos)
                self.apply_action(next_tgt[None,])
                self.get_state()
                time_till_next_step = self.high_dt - (time.time()-step_start)
                if time_till_next_step>0:
                    time.sleep(time_till_next_step)
            logger.info('Calibration Done. Press R2 to continue')
            while True:
                if self.right_lower_right_switch_pressed:
                    if not self._policy_start_base_pose_ready():
                        continue
                    logger.info('R2 pressed again, Communication built between policy layer and transition layer!')
                    self.right_lower_right_switch_pressed = False
                    # Reset heading at policy start so "current facing" becomes zero yaw.
                    if self._reset_heading_on_start:
                        # Use latest state before resetting heading
                        self.get_state()
                        self.reset_heading()
                        self.reset_teleop()
                    break
                
        else:
            raise NotImplementedError
            


    def check_termination(self):
        if not bool(getattr(self, "terminate_on_r2", getattr(self.cfg.control, "terminate_on_r2", True))):
            return False
        return self.right_lower_right_switch_pressed
        # return abs(self.root_rpy_tmp[0])>0.8 or abs(self.root_rpy_tmp[1])>0.8 or self.right_lower_right_switch_pressed
