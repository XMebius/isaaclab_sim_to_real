# register the environments and agent configurations for Aliengo

import gymnasium as gym
from . import agents, rough_env_cfg



##
# Register Gym environments.
##
gym.register(
    id="Rough-Aliengo",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": rough_env_cfg.AliengoRoughEnvCfg,
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:UnitreeAliengoRoughPPORunnerCfg",
    },
)