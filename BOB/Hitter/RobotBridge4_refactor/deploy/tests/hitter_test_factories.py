from __future__ import annotations

import numpy as np

from utils.hitter_planner import HitterWbcCommand, StrikePlan
from utils.hitter_realtime import PlannerResultSnapshot
from utils.hitter_runtime_types import PlannerFailureReason


def command(
    side: str = "forehand",
    base=(-0.4, -0.2),
    position=(0.0, -0.2, 1.0),
    velocity=(1.0, 0.0, 0.5),
    *,
    time_to_strike: float = 0.92,
    t_strike: float | None = None,
    ball_in=(-2.0, 0.1, -0.3),
    ball_out=(3.0, -0.2, 1.0),
    side_source: str = "table_y",
) -> HitterWbcCommand:
    position_array = np.asarray(position, dtype=np.float64)
    velocity_array = np.asarray(velocity, dtype=np.float64)
    return HitterWbcCommand(
        strike_type=side,
        p_base_target_xy=np.asarray(base, dtype=np.float64),
        v_racket_target_w=velocity_array.copy(),
        time_to_strike=float(time_to_strike),
        strike_plan=StrikePlan(
            t_strike=(float(time_to_strike) if t_strike is None else float(t_strike)),
            p_racket_target=position_array.copy(),
            v_racket_target=velocity_array.copy(),
            v_ball_in=np.asarray(ball_in, dtype=np.float64),
            v_ball_out=np.asarray(ball_out, dtype=np.float64),
        ),
        strike_table_y_w=float(position_array[1]),
        strike_side_source=side_source,
    )


def success(
    *,
    track_id: int = 7,
    generation: int = 1,
    deadline: float = 2.0,
    planned_command: HitterWbcCommand | None = None,
    strike_type: str = "forehand",
    base=(-0.4, -0.2),
    position=(0.0, -0.2, 1.0),
    velocity=(1.0, 0.0, 0.5),
    completed: float = 1.0,
) -> PlannerResultSnapshot:
    if planned_command is None:
        planned_command = command(
            strike_type,
            base,
            position,
            velocity,
            time_to_strike=max(float(deadline) - float(completed), 0.0),
        )
    return PlannerResultSnapshot(
        track_id=track_id,
        source_generation=generation,
        source_frame=generation,
        strike_deadline_monotonic_s=float(deadline),
        completed_monotonic_s=float(completed),
        command=planned_command,
    )


def failure(
    *,
    track_id: int = 7,
    generation: int = 1,
    reason: PlannerFailureReason = PlannerFailureReason.ESTIMATOR_NOT_READY,
    completed: float = 1.0,
    error_text: str = "planner rejected test fixture",
) -> PlannerResultSnapshot:
    return PlannerResultSnapshot(
        track_id=track_id,
        source_generation=generation,
        source_frame=generation,
        strike_deadline_monotonic_s=float("nan"),
        completed_monotonic_s=float(completed),
        command=None,
        failure_reason=reason,
        error_text=error_text,
    )
