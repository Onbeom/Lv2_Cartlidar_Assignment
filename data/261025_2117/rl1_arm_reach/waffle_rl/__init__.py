"""Waffle 4WD 강화학습 과제 등록. 이 패키지를 import하면 gym에 과제가 등록된다."""
import gymnasium as gym

from . import agents

gym.register(
    id="Waffle-RL1-ArmReach",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.reach_env_cfg:RL1ArmReachEnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:RL1ArmReachPPORunnerCfg",
        "default_agent": "rsl_rl",
    },
)

gym.register(
    id="Waffle-RL1-ArmReachV2",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.reach_env_cfg:RL1ArmReachV2EnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:RL1ArmReachV2PPORunnerCfg",
        "default_agent": "rsl_rl",
    },
)

gym.register(
    id="Waffle-RL1-ArmReachV3S1",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.reach_env_cfg:RL1ArmReachV3S1EnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:RL1ArmReachV2PPORunnerCfg",
        "default_agent": "rsl_rl",
    },
)

gym.register(
    id="Waffle-RL1-ArmReachV3S2",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.reach_env_cfg:RL1ArmReachV3S1EnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:RL1ArmReachV3S2PPORunnerCfg",
        "default_agent": "rsl_rl",
    },
)

gym.register(
    id="Waffle-RL1-ArmReachV3S3",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.reach_env_cfg:RL1ArmReachV3S3EnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:RL1ArmReachV2PPORunnerCfg",
        "default_agent": "rsl_rl",
    },
)

gym.register(
    id="Waffle-RL1-ArmReachV4",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.reach_env_cfg:RL1ArmReachV4EnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:RL1ArmReachV2PPORunnerCfg",
        "default_agent": "rsl_rl",
    },
)

gym.register(
    id="Waffle-RL1-ArmReachV5",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.reach_env_cfg:RL1ArmReachV5EnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:RL1ArmReachV2PPORunnerCfg",
        "default_agent": "rsl_rl",
    },
)

gym.register(
    id="Waffle-RL1-ArmReachV6",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.reach_env_cfg:RL1ArmReachV6EnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:RL1ArmReachV2PPORunnerCfg",
        "default_agent": "rsl_rl",
    },
)

gym.register(
    id="Waffle-RL1-ArmReachV7",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.reach_env_cfg:RL1ArmReachV7EnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:RL1ArmReachV2PPORunnerCfg",
        "default_agent": "rsl_rl",
    },
)

gym.register(
    id="Waffle-RL1-ArmReachV8",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.reach_env_cfg:RL1ArmReachV8EnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:RL1ArmReachV8PPORunnerCfg",
        "default_agent": "rsl_rl",
    },
)

gym.register(
    id="Waffle-RL1-ArmReachV9",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.reach_env_cfg:RL1ArmReachV9EnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:RL1ArmReachV8PPORunnerCfg",
        "default_agent": "rsl_rl",
    },
)

gym.register(
    id="Waffle-RL1-ArmReachV10",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.reach_env_cfg:RL1ArmReachV10EnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:RL1ArmReachV8PPORunnerCfg",
        "default_agent": "rsl_rl",
    },
)

gym.register(
    id="Waffle-RL1-ArmReachV11",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.reach_env_cfg:RL1ArmReachV11EnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:RL1ArmReachV8PPORunnerCfg",
        "default_agent": "rsl_rl",
    },
)

gym.register(
    id="Waffle-RL1-ArmReachV12",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.reach_env_cfg:RL1ArmReachV12EnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:RL1ArmReachV8PPORunnerCfg",
        "default_agent": "rsl_rl",
    },
)

gym.register(
    id="Waffle-RL1-ArmReachV13",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.reach_env_cfg:RL1ArmReachV13EnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:RL1ArmReachV8PPORunnerCfg",
        "default_agent": "rsl_rl",
    },
)

gym.register(
    id="Waffle-RL1-ArmReachV14",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.reach_env_cfg:RL1ArmReachV14EnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:RL1ArmReachV8PPORunnerCfg",
        "default_agent": "rsl_rl",
    },
)

gym.register(
    id="Waffle-RL1-ArmReachV15",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.reach_env_cfg:RL1ArmReachV15EnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:RL1ArmReachV8PPORunnerCfg",
        "default_agent": "rsl_rl",
    },
)

gym.register(
    id="Waffle-RL1-ArmReachV16",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.reach_env_cfg:RL1ArmReachV16EnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:RL1ArmReachV8PPORunnerCfg",
        "default_agent": "rsl_rl",
    },
)
