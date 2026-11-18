"""RL1 PPO 설정. Isaac Lab Franka reach(FrankaReachPPORunnerCfg) 값을 그대로 쓴다."""
from isaaclab.utils import configclass

from isaaclab_rl.rsl_rl import RslRlMLPModelCfg, RslRlOnPolicyRunnerCfg, RslRlPpoAlgorithmCfg


@configclass
class RL1ArmReachPPORunnerCfg(RslRlOnPolicyRunnerCfg):
    num_steps_per_env = 24
    max_iterations = 1000
    save_interval = 50
    experiment_name = "rl1_arm_reach"
    run_name = ""
    actor = RslRlMLPModelCfg(
        hidden_dims=[64, 64],
        activation="elu",
        obs_normalization=False,
        distribution_cfg=RslRlMLPModelCfg.GaussianDistributionCfg(init_std=1.0),
    )
    critic = RslRlMLPModelCfg(hidden_dims=[64, 64], activation="elu", obs_normalization=False)
    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=1.0,
        use_clipped_value_loss=True,
        clip_param=0.2,
        entropy_coef=0.001,
        num_learning_epochs=8,
        num_mini_batches=4,
        learning_rate=1.0e-3,
        schedule="adaptive",
        gamma=0.99,
        lam=0.95,
        desired_kl=0.01,
        max_grad_norm=1.0,
    )


@configclass
class RL1ArmReachV2PPORunnerCfg(RL1ArmReachPPORunnerCfg):
    # 설계 v2: 환경에 들어가는 행동을 ±1로 자른다(v1은 자르지 않아 행동 값이 커지며 발산)
    clip_actions = 1.0


@configclass
class RL1ArmReachV3S2PPORunnerCfg(RL1ArmReachV2PPORunnerCfg):
    # 설계 v3 S2: 정책 초기 표준편차 1.0 → 0.3(덜 거칠게 탐색)
    actor = RslRlMLPModelCfg(
        hidden_dims=[64, 64],
        activation="elu",
        obs_normalization=False,
        distribution_cfg=RslRlMLPModelCfg.GaussianDistributionCfg(init_std=0.3),
    )


@configclass
class RL1ArmReachV8PPORunnerCfg(RL1ArmReachV2PPORunnerCfg):
    # 설계 v8: 관찰 정규화(관찰 단위가 m·rad·rad/s로 섞이고 남은 거리는 mm 크기라 그대로는 입력 차이가 작음)
    actor = RslRlMLPModelCfg(
        hidden_dims=[64, 64],
        activation="elu",
        obs_normalization=True,
        distribution_cfg=RslRlMLPModelCfg.GaussianDistributionCfg(init_std=1.0),
    )
    critic = RslRlMLPModelCfg(hidden_dims=[64, 64], activation="elu", obs_normalization=True)
