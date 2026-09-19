#!/usr/bin/env python3
import argparse
import select
import sys
import time
from pathlib import Path

import lcm
import numpy as np
import onnxruntime as ort
from omegaconf import OmegaConf
from scipy.spatial.transform import Rotation as sRot

BRIDGE_DIR = Path(__file__).resolve().parents[2]
DEPLOY_DIR = BRIDGE_DIR / "deploy"
for path in (BRIDGE_DIR, DEPLOY_DIR):
    path_str = str(path)
    if path_str not in sys.path:
        sys.path.insert(0, path_str)

from unitree_sdk2.lcm_types.transformation_t import transformation_t
from utils.hitter_planner import (
    BallStateEstimator,
    BallTrajectoryPredictor,
    BaseTargetPlanner,
    HitterSystemPlanner,
    StrikePlanner,
)


class MocapState:
    def __init__(self, ball_estimator, *, sample_rate_hz=300.0, max_sample_gap_s=0.25):
        self.base_pos = None
        self.base_quat = None
        self.ball_pos = None
        self.ball_vel = np.zeros(3, dtype=np.float64)
        self.ball_estimator = ball_estimator
        self.ball_state_ready = False
        self.ball_state_sample_count = 0
        self.last_msg_time = None
        self.ball_stream_time = None
        self.last_ball_host_time = None
        self.sample_rate_hz = float(sample_rate_hz)
        self.max_sample_gap_s = None if max_sample_gap_s is None else float(max_sample_gap_s)

    def _next_ball_timestamp(self, host_time):
        if self.ball_stream_time is None or self.last_ball_host_time is None:
            self.ball_stream_time = 0.0
        else:
            wall_dt = max(0.0, float(host_time - self.last_ball_host_time))
            nominal_dt = 1.0 / self.sample_rate_hz if self.sample_rate_hz > 0.0 else wall_dt
            if self.max_sample_gap_s is not None and wall_dt > self.max_sample_gap_s:
                self.ball_stream_time += wall_dt
            else:
                self.ball_stream_time += nominal_dt
        self.last_ball_host_time = float(host_time)
        return self.ball_stream_time

    def update(self, msg):
        name = str(msg.name or "").strip().lower()
        pos = np.asarray(msg.pos_vicon, dtype=np.float64).reshape(-1)[:3]
        quat = np.asarray(msg.quat_vicon, dtype=np.float64).reshape(-1)[:4]
        self.last_msg_time = time.time()

        if "ball" in name:
            estimate = self.ball_estimator.add_sample(pos, timestamp=self._next_ball_timestamp(self.last_msg_time))
            self.ball_pos = estimate.position
            self.ball_vel = estimate.velocity
            self.ball_state_ready = bool(estimate.valid)
            self.ball_state_sample_count = int(estimate.sample_count)
            return

        if "table" in name:
            return

        self.base_pos = pos
        norm = np.linalg.norm(quat)
        self.base_quat = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64) if norm < 1.0e-9 else quat / norm

    @property
    def ready(self):
        return (
            self.base_pos is not None
            and self.base_quat is not None
            and self.ball_pos is not None
            and self.ball_state_ready
        )


def _range2(value, default):
    if value is None:
        return default
    return (float(value[0]), float(value[1]))


def build_planner(planner_cfg, motion_cfg):
    predictor = BallTrajectoryPredictor(
        gravity=planner_cfg.get("gravity", [0.0, 0.0, -9.81]),
        drag_coefficient=float(planner_cfg.get("drag_coefficient", 0.0)),
        vertical_restitution=float(planner_cfg.get("vertical_restitution", 0.8)),
        horizontal_restitution=float(planner_cfg.get("horizontal_restitution", 0.9)),
        dt=float(planner_cfg.get("prediction_dt", 0.005)),
        table_height=float(planner_cfg.get("table_height", 0.76)),
        table_center_xy=planner_cfg.get("table_center_xy_w", [1.37, 0.0]),
        table_length=float(planner_cfg.get("table_length", 2.74)),
        table_width=float(planner_cfg.get("table_width", motion_cfg.get("physical_table_width", 1.525))),
        ball_radius=float(planner_cfg.get("ball_radius", 0.02)),
    )
    strike_planner = StrikePlanner(
        predictor=predictor,
        virtual_hit_plane_x=float(planner_cfg.get("virtual_hit_plane_x", motion_cfg.get("virtual_hit_plane_x", 0.0))),
        desired_landing_point=planner_cfg.get("desired_landing_point_w", [2.05, 0.0, 0.78]),
        post_hit_flight_time=float(planner_cfg.get("post_hit_flight_time", 0.55)),
        racket_restitution=float(planner_cfg.get("racket_restitution", 0.85)),
        prediction_horizon_s=float(planner_cfg.get("prediction_horizon_s", 2.0)),
        require_future_hit_plane_crossing=bool(planner_cfg.get("require_future_hit_plane_crossing", False)),
    )
    base_planner = BaseTargetPlanner(
        racket_x_offset_b=float(planner_cfg.get("racket_x_offset_b", planner_cfg.get("strike_plane_x", 0.40))),
        forehand_nominal_racket_y_b=float(planner_cfg.get("forehand_nominal_racket_y_b", -0.5)),
        backhand_nominal_racket_y_b=float(planner_cfg.get("backhand_nominal_racket_y_b", 0.22)),
        default_base_z=float(planner_cfg.get("target_base_height_w", motion_cfg.get("target_base_height_w", 0.793))),
    )
    return HitterSystemPlanner(strike_planner=strike_planner, base_planner=base_planner)


def build_ball_state_estimator(planner_cfg, motion_cfg):
    max_gap = planner_cfg.get("state_estimator_max_sample_gap_s", 0.25)
    return BallStateEstimator(
        window_size=int(planner_cfg.get("state_estimator_window_size", 31)),
        min_samples=int(planner_cfg.get("state_estimator_min_samples", planner_cfg.get("state_estimator_window_size", 31))),
        table_height=float(planner_cfg.get("table_height", 0.76)),
        table_center_xy=planner_cfg.get("table_center_xy_w", [1.37, 0.0]),
        table_length=float(planner_cfg.get("table_length", 2.74)),
        table_width=float(planner_cfg.get("table_width", motion_cfg.get("physical_table_width", 1.525))),
        ball_radius=float(planner_cfg.get("ball_radius", 0.02)),
        bounce_height_tolerance=float(planner_cfg.get("state_estimator_bounce_height_tolerance", 0.03)),
        bounce_velocity_threshold=float(planner_cfg.get("state_estimator_bounce_velocity_threshold", 0.10)),
        max_sample_gap_s=None if max_gap is None else float(max_gap),
    )


def clip_racket_velocity(velocity, strike_type, planner_cfg):
    if not bool(planner_cfg.get("clip_racket_velocity_to_training_range", True)):
        return np.asarray(velocity, dtype=np.float64)
    velocity = np.asarray(velocity, dtype=np.float64).copy()
    ranges = (planner_cfg.get("racket_velocity_clip_ranges", {}) or {}).get(strike_type, {}) or {}
    defaults = {
        "forehand": {"x": [2.6, 3.4], "y": [-0.4, 1.6], "z": [-0.3, 2.7]},
        "backhand": {"x": [2.6, 3.4], "y": [-1.6, 0.4], "z": [-0.3, 2.7]},
    }
    for idx, axis in enumerate(("x", "y", "z")):
        low, high = ranges.get(axis, defaults[strike_type][axis])
        velocity[idx] = np.clip(velocity[idx], float(low), float(high))
    return velocity


def yaw_inverse_apply(quat_xyzw, vector_w):
    yaw = float(sRot.from_quat(np.asarray(quat_xyzw, dtype=np.float64)).as_euler("xyz", degrees=False)[2])
    return sRot.from_euler("z", -yaw, degrees=False).apply(vector_w).astype(np.float32)


def make_obs(state, command, planner_cfg):
    base_pos = state.base_pos.astype(np.float32)
    base_quat = state.base_quat.astype(np.float32)
    base_rot = sRot.from_quat(base_quat).as_matrix()
    base_forward_xy = base_rot[:, 0][:2].astype(np.float32)

    target_base_z = float(planner_cfg.get("target_base_height_w", 0.793))
    delta_base_w = np.array(
        [
            command.p_base_target_xy[0] - base_pos[0],
            command.p_base_target_xy[1] - base_pos[1],
            target_base_z - base_pos[2],
        ],
        dtype=np.float32,
    )
    obs_base_target_pos = yaw_inverse_apply(base_quat, delta_base_w)
    obs_racket_target_pos = yaw_inverse_apply(base_quat, command.strike_plan.p_racket_target - base_pos)
    obs_racket_target_vel = clip_racket_velocity(command.v_racket_target_w, command.strike_type, planner_cfg).astype(np.float32)
    obs_time_to_strike = np.asarray([max(float(command.time_to_strike), 0.0)], dtype=np.float32)

    obs = np.concatenate(
        [
            np.zeros(3, dtype=np.float32),
            np.asarray([0.0, 0.0, -1.0], dtype=np.float32),
            base_forward_xy,
            obs_base_target_pos,
            obs_racket_target_pos,
            obs_racket_target_vel,
            obs_time_to_strike,
            np.zeros(29, dtype=np.float32),
            np.zeros(29, dtype=np.float32),
            np.zeros(29, dtype=np.float32),
        ],
        axis=0,
    )
    if obs.size != 105:
        raise RuntimeError(f"assembled obs has {obs.size} values, expected 105")
    return obs.reshape(1, -1).astype(np.float32)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default=str(DEPLOY_DIR / "config/mimic/hitter.yaml"))
    parser.add_argument("--checkpoint", default=None)
    parser.add_argument("--lcm-url", default="udpm://239.255.76.67:7667?ttl=255")
    parser.add_argument("--channel", default="vicon_state_data")
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--print-hz", type=float, default=5.0)
    parser.add_argument("--allow-nearest-hit-plane", action="store_true")
    args = parser.parse_args()

    cfg = OmegaConf.to_container(OmegaConf.load(args.config), resolve=True)
    motion_cfg = cfg.get("motion", {}) or {}
    planner_cfg = motion_cfg.get("ball_planner", {}) or {}
    if args.allow_nearest_hit_plane:
        planner_cfg["require_future_hit_plane_crossing"] = False
    checkpoint = args.checkpoint or cfg.get("policy", {}).get("checkpoint")
    if not checkpoint:
        raise RuntimeError("checkpoint is not set")

    session = ort.InferenceSession(str(checkpoint))
    input_name = session.get_inputs()[0].name
    planner = build_planner(planner_cfg, motion_cfg)
    state_max_gap = planner_cfg.get("state_estimator_max_sample_gap_s", 0.25)
    state = MocapState(
        build_ball_state_estimator(planner_cfg, motion_cfg),
        sample_rate_hz=float(planner_cfg.get("state_estimator_sample_rate_hz", 300.0)),
        max_sample_gap_s=None if state_max_gap is None else float(state_max_gap),
    )
    lc = lcm.LCM(args.lcm_url)

    def handler(channel, data):
        state.update(transformation_t.decode(data))

    lc.subscribe(args.channel, handler)
    start = time.time()
    last_print = 0.0
    failures = 0
    successes = 0
    print(f"Dry-run listening on {args.channel}; checkpoint={checkpoint}", flush=True)

    while True:
        now = time.time()
        if args.duration > 0.0 and now - start >= args.duration:
            break
        rfds, _, _ = select.select([lc.fileno()], [], [], 0.01)
        if rfds:
            lc.handle()

        if not state.ready:
            if now - last_print >= 1.0:
                print(
                    "waiting for Vicon base+ball messages "
                    f"and ball estimator samples ({state.ball_state_sample_count}/"
                    f"{state.ball_estimator.min_samples})...",
                    flush=True,
                )
                last_print = now
            continue

        try:
            base_forward_xy_w = sRot.from_quat(state.base_quat).as_matrix()[:, 0][:2]
            command = planner.plan_command(
                state.ball_pos,
                state.ball_vel,
                current_base_xy_w=state.base_pos,
                base_forward_xy_w=base_forward_xy_w,
            )
            obs = make_obs(state, command, planner_cfg)
            action = session.run(None, {input_name: obs})[0].reshape(-1)
            successes += 1
            if now - last_print >= 1.0 / max(args.print_hz, 1.0e-6):
                print(
                    "ok "
                    f"base=[{state.base_pos[0]: .3f},{state.base_pos[1]: .3f},{state.base_pos[2]: .3f}] "
                    f"ball=[{state.ball_pos[0]: .3f},{state.ball_pos[1]: .3f},{state.ball_pos[2]: .3f}] "
                    f"ball_vel=[{state.ball_vel[0]: .3f},{state.ball_vel[1]: .3f},{state.ball_vel[2]: .3f}] "
                    f"type={command.strike_type} tts={command.time_to_strike:.3f} "
                    f"base_target=[{command.p_base_target_xy[0]: .3f},{command.p_base_target_xy[1]: .3f}] "
                    f"racket=[{command.strike_plan.p_racket_target[0]: .3f},{command.strike_plan.p_racket_target[1]: .3f},{command.strike_plan.p_racket_target[2]: .3f}] "
                    f"action[min,max,mean,norm]=[{action.min(): .3f},{action.max(): .3f},{action.mean(): .3f},{np.linalg.norm(action): .3f}]",
                    flush=True,
                )
                last_print = now
        except Exception as exc:
            failures += 1
            if now - last_print >= 0.5:
                print(
                    f"planner/model dry-run not ready: {exc}; "
                    f"ball=[{state.ball_pos[0]:.3f},{state.ball_pos[1]:.3f},{state.ball_pos[2]:.3f}] "
                    f"vel=[{state.ball_vel[0]:.3f},{state.ball_vel[1]:.3f},{state.ball_vel[2]:.3f}]",
                    flush=True,
                )
                last_print = now

    print(f"successes={successes} failures={failures}", flush=True)
    return 0 if successes > 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
