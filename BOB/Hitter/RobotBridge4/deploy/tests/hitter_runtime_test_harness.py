from __future__ import annotations

from collections import deque
from dataclasses import replace
from types import SimpleNamespace
from typing import Callable
import threading

import numpy as np

from agents.hitter_agent import HitterAgent
from envs.hitter import HitterEnv
from utils.hitter_planner import HitterWbcCommand, StrikePlan
from utils.hitter_realtime import (
    BallEstimateSnapshot,
    CommandPhase,
    CompletedResultBatch,
    PlannerResultSnapshot,
)
from utils.hitter_runtime_factory import (
    HitterRuntimeSettings,
    resolve_hitter_runtime_settings,
)


def default_runtime_settings() -> HitterRuntimeSettings:
    return resolve_hitter_runtime_settings(
        policy_config={"hitter_seed": 7},
        motion_config={},
        control_config={"low_dt": 0.005, "decimation": 4},
    )


def make_snapshot(
    track_id: int = 7,
    generation: int = 1,
    *,
    received: float = 10.0,
    new_track: bool = True,
    consumed: bool = False,
) -> BallEstimateSnapshot:
    return BallEstimateSnapshot(
        track_id=track_id,
        generation=generation,
        source_frame=generation,
        source_time_s=received,
        received_monotonic_s=received,
        position_w=np.asarray([1.0, 0.0, 0.9], dtype=np.float64),
        velocity_w=np.asarray([-2.0, 0.0, 0.2], dtype=np.float64),
        base_position_w=np.asarray([0.0, 0.0, 0.78], dtype=np.float64),
        base_quaternion_xyzw=np.asarray(
            [0.0, 0.0, 0.0, 1.0], dtype=np.float64
        ),
        base_valid=True,
        visible=True,
        ready=True,
        consumed=consumed,
        new_track=new_track,
    )


def make_command(*, marker: float = 0.0, time_to_strike: float = 0.90):
    position = np.asarray([0.35, -0.10 + marker, 1.0], dtype=np.float64)
    velocity = np.asarray([1.0, 0.1 + marker, 0.2], dtype=np.float64)
    plan = StrikePlan(
        t_strike=time_to_strike,
        p_racket_target=position,
        v_racket_target=velocity,
        v_ball_in=np.asarray([-2.0, 0.0, -0.2], dtype=np.float64),
        v_ball_out=np.asarray([2.0, 0.0, 0.5], dtype=np.float64),
    )
    return HitterWbcCommand(
        strike_type="forehand",
        p_base_target_xy=np.asarray([-0.4, 0.0], dtype=np.float64),
        v_racket_target_w=velocity,
        time_to_strike=time_to_strike,
        strike_plan=plan,
        strike_table_y_w=float(position[1]),
        strike_side_source="table_y",
    )


def success_result(
    track_id: int = 7,
    generation: int = 1,
    *,
    now: float = 10.0,
    deadline: float = 10.90,
    marker: float = 0.0,
) -> PlannerResultSnapshot:
    return PlannerResultSnapshot(
        track_id=track_id,
        source_generation=generation,
        source_frame=generation,
        strike_deadline_monotonic_s=deadline,
        completed_monotonic_s=now,
        command=make_command(
            marker=marker,
            time_to_strike=max(deadline - now, 0.0),
        ),
    )


def failure_result(
    reason,
    track_id: int = 7,
    generation: int = 1,
    *,
    now: float = 10.0,
) -> PlannerResultSnapshot:
    return PlannerResultSnapshot(
        track_id=track_id,
        source_generation=generation,
        source_frame=generation,
        strike_deadline_monotonic_s=float("nan"),
        completed_monotonic_s=now,
        command=None,
        failure_reason=reason,
        error_text=f"diagnostic text for {reason.value}",
    )


def empty_batch() -> CompletedResultBatch:
    return CompletedResultBatch(
        results=(),
        frozen_results=(),
        overflowed=False,
        overflow_count=0,
        overflowed_track_ids=(),
    )


def result_batch(
    *results: PlannerResultSnapshot,
    overflowed_track_ids: tuple[int, ...] = (),
) -> CompletedResultBatch:
    overflow_count = len(overflowed_track_ids)
    return CompletedResultBatch(
        results=tuple(results),
        frozen_results=(None,) * len(results),
        overflowed=overflow_count > 0,
        overflow_count=overflow_count,
        overflowed_track_ids=overflowed_track_ids,
    )


class FakeKinematic:
    def forward(self, *, joint_pos, base_pos, base_quat):
        del joint_pos, base_quat
        return {
            "right_racket_link": {
                "pos": np.asarray(base_pos, dtype=np.float64)
                + np.asarray([0.45, -0.25, 0.30], dtype=np.float64)
            }
        }, None


class FakeSimulator:
    high_dt = 0.02

    def __init__(self, *, is_real: bool):
        self.is_real = is_real
        self.base_pose_valid = True
        self.root_trans_world = np.asarray([1.2, -0.3, 0.78], dtype=np.float32)
        self.root_quat_world = np.asarray(
            [0.0, 0.0, 0.0, 1.0], dtype=np.float32
        )
        self.dof_pos = np.zeros(29, dtype=np.float32)
        self.dof_vel = np.zeros(29, dtype=np.float32)
        self.kinematic = FakeKinematic()
        self.applied_actions: list[np.ndarray] = []
        self.get_state_calls = 0
        self.reset_estimator_calls = 0
        self.listener: Callable[[BallEstimateSnapshot], None] | None = None
        self.session_open = is_real
        self.runtime_events = deque()
        self.consumed_track_ids: set[int] = set()
        self.consumption_reasons: dict[int, str] = {}
        self.admitted_track_ids: set[int] = set()
        self.latest_snapshot: BallEstimateSnapshot | None = None
        self.active_track_id: int | None = None
        self.stream_fresh = True
        self.ball_fresh = True
        self.latched_fault = None
        self.no_ball_since = 0.0
        self.call_order: list[str] = []
        self.status_calls = 0
        self.preserve_hitter_ball_state_on_calibrate = False
        self.ball_track_id = 0
        self.ball_snapshot_generation = 0
        self.ball_pos_world = np.asarray([1.0, 0.0, 0.9], dtype=np.float32)
        self.ball_vel_world = np.asarray([-2.0, 0.0, 0.2], dtype=np.float32)
        self.ball_visible = False
        self.ball_state_estimator_ready = False
        self.mujoco_data = SimpleNamespace(time=0.0)

    def register_hitter_ball_listener(self, listener):
        self.listener = listener

        def unregister():
            if self.listener is listener:
                self.listener = None

        return unregister

    def present(self, snapshot: BallEstimateSnapshot, *, admitted: bool = True):
        self.latest_snapshot = snapshot
        self.active_track_id = snapshot.track_id
        if admitted:
            self.admitted_track_ids.add(snapshot.track_id)

    def hitter_snapshot_planning_eligible(self, snapshot):
        self.call_order.append("eligible")
        latest = self.latest_snapshot
        return bool(
            self.session_open
            and self.latched_fault is None
            and snapshot.track_id in self.admitted_track_ids
            and snapshot.track_id not in self.consumed_track_ids
            and self.active_track_id == snapshot.track_id
            and latest is not None
            and latest.track_id == snapshot.track_id
            and latest.generation == snapshot.generation
            and latest.source_frame == snapshot.source_frame
        )

    def consume_hitter_track(self, track_id: int, *, reason: str):
        already = track_id in self.consumed_track_ids
        self.consumed_track_ids.add(track_id)
        self.consumption_reasons[track_id] = reason
        if self.latest_snapshot is not None and self.latest_snapshot.track_id == track_id:
            self.latest_snapshot = replace(self.latest_snapshot, consumed=True)
        return not already

    def end_hitter_policy_session(self, *, reason: str):
        self.session_open = False
        ids = () if self.active_track_id is None else (self.active_track_id,)
        for track_id in ids:
            self.consume_hitter_track(track_id, reason=reason)
        return ids

    def begin_hitter_policy_session(self, *, now_monotonic_s: float):
        del now_monotonic_s
        if not self.stream_fresh or not self.base_pose_valid:
            return False
        self.session_open = True
        self.no_ball_since = 0.0
        return True

    def hitter_vicon_status(self, *, now_monotonic_s: float):
        del now_monotonic_s
        self.status_calls += 1
        self.call_order.append("status")
        return SimpleNamespace(
            stream_fresh=self.stream_fresh,
            ball_fresh=self.ball_fresh,
            base_pose_valid=self.base_pose_valid,
            active_track_id=self.active_track_id,
            last_track_id=self.active_track_id,
            visible=self.active_track_id is not None,
            ready_for_new_serve=True,
            latched_fault=self.latched_fault,
        )

    def drain_hitter_vicon_events(self):
        self.call_order.append("events")
        events = tuple(self.runtime_events)
        self.runtime_events.clear()
        return events

    def reset_ball_state_estimator(self):
        self.reset_estimator_calls += 1

    def get_state(self):
        self.get_state_calls += 1

    def calibrate(self, refresh, ref_dof_pos):
        del refresh, ref_dof_pos

    def update_obs(self):
        return {
            "dof_pos": self.dof_pos.copy(),
            "dof_vel": self.dof_vel.copy(),
            "base_ang_vel": np.zeros(3, dtype=np.float32),
            "projected_gravity": np.asarray(
                [0.0, 0.0, -1.0], dtype=np.float32
            ),
        }

    def apply_action(self, action):
        self.applied_actions.append(np.asarray(action, dtype=np.float32).copy())

    def check_termination(self):
        return False

    def reset_hitter_ball(self):
        return None

    def capture_hitter_ball_launch_state(self):
        return True

    def restore_hitter_ball_launch_state(self):
        return True


class FakePlannerWorker:
    def __init__(self):
        self.batches = deque()
        self.submitted: list[BallEstimateSnapshot] = []
        self.closed = False
        self.submit_hook = None
        self.call_order: list[str] | None = None
        self.stats = SimpleNamespace(
            submitted=0,
            completed=0,
            failed=0,
            dropped_pending=0,
            completed_result_queue_overflow_total=0,
        )

    def submit(self, snapshot):
        if self.submit_hook is not None:
            self.submit_hook(snapshot)
        self.submitted.append(snapshot)
        self.stats.submitted += 1

    def queue(self, batch: CompletedResultBatch):
        self.batches.append(batch)

    def drain_completed_results(self):
        if self.call_order is not None:
            self.call_order.append("batch")
        return self.batches.popleft() if self.batches else empty_batch()

    def close(self, timeout_s=None):
        del timeout_s
        self.closed = True
        return True


def make_hitter_env_for_test(
    is_real: bool,
    settings: HitterRuntimeSettings | None = None,
) -> HitterEnv:
    settings = default_runtime_settings() if settings is None else settings
    env = HitterEnv.__new__(HitterEnv)
    env.simulator = FakeSimulator(is_real=is_real)
    env.cfg = SimpleNamespace(
        control=SimpleNamespace(
            obs_clip_value=1000.0,
            action_clip_value=1000.0,
        ),
        collect_dataset=False,
    )
    env.policy_cfg = {}
    env.motion_cfg = {
        "waiting_base_target_xy_w": [-0.4, 0.0],
        "ball_planner": {"target_base_height_w": 0.78},
    }
    env.hitter_runtime_settings = settings
    env.hitter_rng = np.random.default_rng(settings.hitter_seed)
    env.waiting_time_to_strike_s = settings.waiting_tts_s
    env.waiting_racket_body_name = "right_racket_link"
    env.reset_ball_on_command = False
    env.playback_speed = 1.0
    env.real_world_first_frame_transition_s = 0.0
    env._real_world_first_frame_transition_pending = False
    env._last_waiting_racket_target_error_log_s = 0.0
    env._last_ball_estimator_not_ready_log_s = 0.0
    env._last_ball_planner_error_log_s = 0.0
    env._policy_dim = 29
    env.policy_model_meta = object()
    env.policy_default_joint_pos = np.zeros(29, dtype=np.float32)
    env.policy_default_joint_pos_sim = np.zeros(29, dtype=np.float32)
    env.policy_action_scales = np.ones(29, dtype=np.float32)
    env.policy_action_scales_sim = np.ones(29, dtype=np.float32)
    env.prev_policy_action = np.zeros(29, dtype=np.float32)
    env.action_beta = 1.0
    env._policy_to_sim_adapter = None
    env._sim_to_policy_adapter = None
    env._policy_to_sim = np.arange(29, dtype=np.int32)
    env._sim_to_policy = np.arange(29, dtype=np.int32)
    env.dof_pos = np.zeros((1, 29), dtype=np.float32)
    env.dof_vel = np.zeros((1, 29), dtype=np.float32)
    env.base_ang_vel = np.zeros((1, 3), dtype=np.float32)
    env.projected_gravity = np.asarray([[0.0, 0.0, -1.0]], dtype=np.float32)
    env.action = np.zeros((1, 29), dtype=np.float32)
    env.episode_length_buf = np.zeros(1, dtype=np.float64)
    env.first_obs_received = True
    env.time_step = 0.0
    env.save_video_enabled = False
    env.hard_reset = False
    env.history_handler = SimpleNamespace(reset=lambda: None)
    env.init_ref_dof_pos = None
    env.collected_traj = {}
    env._init_hitter_command_state()
    env._init_hitter_lifecycle_state()
    env._hitter_lifecycle_lock = threading.RLock()
    env._hitter_submitted_track_ids = set()
    env._hitter_runtime_accepting = bool(is_real)
    env._waiting_anchor_fault = None
    env.waiting_base_anchor_xy_w = (
        np.asarray(env.simulator.root_trans_world[:2], dtype=np.float32).copy()
        if is_real
        else None
    )
    if is_real:
        env.hitter_planner_worker = FakePlannerWorker()
        env._unregister_ball_listener = env.simulator.register_hitter_ball_listener(
            env._submit_hitter_planner_snapshot
        )
        env.simulator.session_open = True
    env.compute_observation()
    return env


class FakeOnnxSession:
    def __init__(self, action_dim: int):
        self.action_dim = action_dim
        self.calls = 0
        self.last_inputs = None

    def run(self, _outputs, inputs):
        self.calls += 1
        self.last_inputs = inputs
        return [np.zeros((1, self.action_dim), dtype=np.float32)]


class AgentHarness:
    def __init__(self, env: HitterEnv):
        self.env = env
        self.agent = HitterAgent.__new__(HitterAgent)
        self.agent.env = env
        self.agent.policy = FakeOnnxSession(env._policy_dim)
        self.last_observation = None

    @property
    def onnx_calls(self):
        return self.agent.policy.calls

    @property
    def apply_action_calls(self):
        return len(self.env.simulator.applied_actions)

    def run_ticks(self, count: int, *, phase: CommandPhase):
        self.env.hitter_command_lifecycle.phase = phase
        obs = self.env.obs_buf_dict
        for _ in range(count):
            obs = self.agent._run_iteration(obs)
            self.last_observation = self.agent.policy.last_inputs["obs"]
        return obs


def agent_harness(*, is_real: bool = True) -> AgentHarness:
    return AgentHarness(make_hitter_env_for_test(is_real=is_real))
