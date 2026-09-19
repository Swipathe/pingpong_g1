from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class PlannerFeed(str, Enum):
    INLINE_STATE = "inline_state"
    SNAPSHOT_STREAM = "snapshot_stream"


class LoopPacing(str, Enum):
    SIMULATOR = "simulator"
    AGENT = "agent"


@dataclass(frozen=True)
class HitterRuntimeCapabilities:
    planner_feed: PlannerFeed
    loop_pacing: LoopPacing

    @property
    def uses_snapshot_stream(self) -> bool:
        return self.planner_feed == PlannerFeed.SNAPSHOT_STREAM

    @property
    def uses_agent_pacing(self) -> bool:
        return self.loop_pacing == LoopPacing.AGENT


MUJOCO_HITTER_RUNTIME_CAPABILITIES = HitterRuntimeCapabilities(
    planner_feed=PlannerFeed.INLINE_STATE,
    loop_pacing=LoopPacing.SIMULATOR,
)

REAL_WORLD_HITTER_RUNTIME_CAPABILITIES = HitterRuntimeCapabilities(
    planner_feed=PlannerFeed.SNAPSHOT_STREAM,
    loop_pacing=LoopPacing.AGENT,
)


def hitter_runtime_capabilities_for(simulator) -> HitterRuntimeCapabilities:
    capabilities = getattr(simulator, "hitter_runtime_capabilities", None)
    if capabilities is not None:
        if not isinstance(capabilities, HitterRuntimeCapabilities):
            raise TypeError(
                "simulator.hitter_runtime_capabilities must be "
                "HitterRuntimeCapabilities."
            )
        return capabilities

    # Compatibility for existing lightweight simulator doubles. Production
    # backends declare an explicit immutable capability profile.
    if bool(getattr(simulator, "is_real", False)):
        return REAL_WORLD_HITTER_RUNTIME_CAPABILITIES
    return MUJOCO_HITTER_RUNTIME_CAPABILITIES
