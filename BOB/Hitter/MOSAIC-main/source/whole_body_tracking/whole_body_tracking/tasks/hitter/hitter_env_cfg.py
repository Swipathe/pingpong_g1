from __future__ import annotations

from dataclasses import MISSING
import os
from pathlib import Path

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import ContactSensorCfg
from isaaclab.terrains import TerrainImporterCfg
from isaaclab.terrains.terrain_importer import TerrainImporter
from isaaclab.utils import configclass
from isaaclab.utils.noise import AdditiveUniformNoiseCfg as Unoise

import whole_body_tracking.tasks.hitter.mdp as mdp
from whole_body_tracking.tasks.hitter.mdp.commands import HitterStrikingCommandCfg


VELOCITY_RANGE = {
    "x": (-0.5, 0.5),
    "y": (-0.5, 0.5),
    "z": (-0.2, 0.2),
    "roll": (-0.52, 0.52),
    "pitch": (-0.52, 0.52),
    "yaw": (-0.78, 0.78),
}

HITTER_READY_ARM_NAMES = [
    "left_elbow_link",
    "left_wrist_roll_link",
    "left_wrist_pitch_link",
    "left_wrist_yaw_link",
    "right_elbow_link",
    "right_wrist_roll_link",
    "right_wrist_pitch_link",
    "right_wrist_yaw_link",
]


_MOSAIC_ROOT = Path(__file__).resolve().parents[5]
HITTER_LOCAL_GROUND_PLANE_USD = os.environ.get(
    "HITTER_LOCAL_GROUND_PLANE_USD",
    str(_MOSAIC_ROOT / "assets/isaac/Environments/Grid/default_environment.usd"),
)


class HitterLocalGroundPlaneTerrainImporter(TerrainImporter):
    """使用本地 USD 缓存里的 Isaac 网格地面。"""

    def import_ground_plane(self, name: str, size: tuple[float, float] = (2.0e6, 2.0e6)):
        prim_path = self.cfg.prim_path + f"/{name}"
        if prim_path in self.terrain_prim_paths:
            raise ValueError(f"A terrain with the name '{name}' already exists. Existing terrains: {', '.join(self.terrain_names)}.")
        self.terrain_prim_paths.append(prim_path)
        ground_plane_cfg = sim_utils.GroundPlaneCfg(
            usd_path=HITTER_LOCAL_GROUND_PLANE_USD,
            physics_material=self.cfg.physics_material,
            size=size,
            color=(0.0, 0.0, 0.0),
        )
        ground_plane_cfg.func(prim_path, ground_plane_cfg)


@configclass
class HitterFlatSceneCfg(InteractiveSceneCfg):
    """用于 HITTER WBC 训练的平地场景。"""

    terrain = TerrainImporterCfg(
        class_type=TerrainImporter,
        prim_path="/World/ground",
        terrain_type="plane",
        collision_group=-1,
        physics_material=sim_utils.RigidBodyMaterialCfg(
            friction_combine_mode="multiply",
            restitution_combine_mode="multiply",
            static_friction=1.0,
            dynamic_friction=1.0,
            restitution=0.0,
        ),
    )
    robot: ArticulationCfg = MISSING
    light = AssetBaseCfg(
        prim_path="/World/light",
        spawn=sim_utils.DistantLightCfg(color=(0.75, 0.75, 0.75), intensity=3000.0),
    )
    sky_light = AssetBaseCfg(
        prim_path="/World/skyLight",
        spawn=sim_utils.DomeLightCfg(color=(0.9, 0.9, 0.9), intensity=1000.0),
    )
    contact_forces = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/Robot/.*",
        history_length=3,
        force_threshold=10.0,
        debug_vis=False,
    )


@configclass
class HitterStrikingCommandsCfg:
    """HITTER 风格击球任务的指令配置。"""

    motion: HitterStrikingCommandCfg = HitterStrikingCommandCfg(
        asset_name="robot",
        resampling_time_range=(1.0e9, 1.0e9),
        debug_vis=True,
        pose_range={
            "x": (-0.02, 0.02),
            "y": (-0.02, 0.02),
            "z": (-0.01, 0.01),
            "roll": (-0.05, 0.05),
            "pitch": (-0.05, 0.05),
            "yaw": (-0.10, 0.10),
        },
        velocity_range=VELOCITY_RANGE,
        joint_position_range=(-0.05, 0.05),
    )


@configclass
class HitterStrikingActionsCfg:
    """HITTER WBC 的动作配置。"""

    joint_pos = mdp.JointPositionActionCfg(asset_name="robot", joint_names=[".*"], use_default_offset=True)


@configclass
class HitterStrikingObservationsCfg:
    """HITTER 非对称 actor-critic 训练的观测组。"""

    @configclass
    class PolicyCfg(ObsGroup):
        base_ang_vel = ObsTerm(func=mdp.base_ang_vel, noise=Unoise(n_min=-0.2, n_max=0.2))
        projected_gravity = ObsTerm(func=mdp.projected_gravity, noise=Unoise(n_min=-0.05, n_max=0.05))
        base_forward_xy = ObsTerm(func=mdp.hitter_base_forward_xy, params={"command_name": "motion"})
        # 第三阶段任务指令：暴露球拍位置、速度目标和击球时序。
        base_target_xy = ObsTerm(func=mdp.hitter_base_target_xy, params={"command_name": "motion"})
        racket_target_pos = ObsTerm(func=mdp.hitter_racket_target_pos, params={"command_name": "motion"})
        racket_target_vel = ObsTerm(func=mdp.hitter_racket_target_vel, params={"command_name": "motion"})
        time_to_strike = ObsTerm(func=mdp.hitter_time_to_strike, params={"command_name": "motion"})
        joint_pos = ObsTerm(func=mdp.joint_pos_rel, noise=Unoise(n_min=-0.01, n_max=0.01))
        joint_vel = ObsTerm(func=mdp.joint_vel_rel, noise=Unoise(n_min=-0.5, n_max=0.5))
        actions = ObsTerm(func=mdp.last_action)

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True

    @configclass
    class PrivilegedCfg(ObsGroup):
        base_ang_vel = ObsTerm(func=mdp.base_ang_vel)
        projected_gravity = ObsTerm(func=mdp.projected_gravity)
        base_forward_xy = ObsTerm(func=mdp.hitter_base_forward_xy, params={"command_name": "motion"})
        motion_anchor_pos_b = ObsTerm(func=mdp.motion_anchor_pos_b, params={"command_name": "motion"})
        motion_anchor_ori_b = ObsTerm(func=mdp.motion_anchor_ori_b, params={"command_name": "motion"})
        # 第三阶段任务指令：暴露球拍位置、速度目标和击球时序。
        base_target_xy = ObsTerm(func=mdp.hitter_base_target_xy, params={"command_name": "motion"})
        racket_target_pos = ObsTerm(func=mdp.hitter_racket_target_pos, params={"command_name": "motion"})
        racket_target_vel = ObsTerm(func=mdp.hitter_racket_target_vel, params={"command_name": "motion"})
        time_to_strike = ObsTerm(func=mdp.hitter_time_to_strike, params={"command_name": "motion"})
        joint_pos = ObsTerm(func=mdp.joint_pos_rel)
        joint_vel = ObsTerm(func=mdp.joint_vel_rel)
        actions = ObsTerm(func=mdp.last_action)
        base_lin_vel = ObsTerm(func=mdp.base_lin_vel)
        body_pose = ObsTerm(func=mdp.hitter_robot_body_pose, params={"command_name": "motion"})
        time_left = ObsTerm(func=mdp.hitter_time_left)
        reference_joint_state = ObsTerm(func=mdp.hitter_reference_joint_state, params={"command_name": "motion"})

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()
    critic: PrivilegedCfg = PrivilegedCfg()


@configclass
class HitterEmptyRewardsCfg:
    """基础占位配置；具体 HITTER 任务会提供自己的奖励配置。"""

    pass


@configclass
class HitterStrikingTerminationsCfg:
    """HITTER WBC 训练的终止条件。"""

    time_out = DoneTerm(func=mdp.time_out, time_out=True)
    anchor_pos = DoneTerm(
        func=mdp.hitter_bad_anchor_height,
        params={"command_name": "motion", "target_height": 0.793, "threshold": 0.35},
    )
    anchor_ori = DoneTerm(
        func=mdp.bad_anchor_ori,
        params={"asset_cfg": SceneEntityCfg("robot"), "command_name": "motion", "threshold": 0.9},
    )
    base_bad_orientation = None
    base_height = None


@configclass
class HitterStrikingEventCfg:
    """复用 MOSAIC tracking 的启动和间隔随机化配置。"""

    physics_material = EventTerm(
        func=mdp.randomize_rigid_body_material,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=".*"),
            "static_friction_range": (1.0, 1.0),
            "dynamic_friction_range": (1.0, 1.0),
            "restitution_range": (0.0, 0.0),
            "num_buckets": 64,
        },
    )
    add_joint_default_pos = EventTerm(
        func=mdp.randomize_joint_default_pos,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=[".*"]),
            "pos_distribution_params": (-0.01, 0.01),
            "operation": "add",
        },
    )
    base_com = EventTerm(
        func=mdp.randomize_rigid_body_com,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names="torso_link"),
            "com_range": {"x": (-0.025, 0.025), "y": (-0.05, 0.05), "z": (-0.05, 0.05)},
        },
    )
    push_robot = EventTerm(
        func=mdp.push_by_setting_velocity,
        mode="interval",
        interval_range_s=(1.0, 3.0),
        params={"velocity_range": VELOCITY_RANGE},
    )


@configclass
class HitterStrikingEnvCfg(ManagerBasedRLEnvCfg):
    """稳定精简版 HITTER 风格目标条件 WBC 任务。"""

    scene: HitterFlatSceneCfg = HitterFlatSceneCfg(num_envs=4096, env_spacing=2.5)
    observations: HitterStrikingObservationsCfg = HitterStrikingObservationsCfg()
    actions: HitterStrikingActionsCfg = HitterStrikingActionsCfg()
    commands: HitterStrikingCommandsCfg = HitterStrikingCommandsCfg()
    rewards: HitterEmptyRewardsCfg = HitterEmptyRewardsCfg()
    terminations: HitterStrikingTerminationsCfg = HitterStrikingTerminationsCfg()
    events: HitterStrikingEventCfg = HitterStrikingEventCfg()

    def __post_init__(self):
        self.decimation = 4
        self.episode_length_s = 10.0
        self.sim.dt = 0.005
        self.sim.render_interval = self.decimation
        self.sim.physics_material = self.scene.terrain.physics_material
        self.sim.physx.gpu_max_rigid_patch_count = 15 * 2**17
        self.viewer.eye = (3.5, 3.5, 3.0)
        self.viewer.origin_type = "env"
        self.viewer.asset_name = "robot"
