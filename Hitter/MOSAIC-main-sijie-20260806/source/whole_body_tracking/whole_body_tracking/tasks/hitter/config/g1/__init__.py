import gymnasium as gym

from . import agents, flat_env_cfg


gym.register(
    id="Hitter-Striking-PlannerDomain-Flat-G1-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": flat_env_cfg.G1HitterStrikingPlannerDomainFlatEnvCfg,
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:G1HitterStrikingPlannerDomainPPORunnerCfg",
    },
)
