"""RL1 MDP 항목 설정. 실행 클래스(mdp.py)는 문자열로 가리켜 Kit가 뜬 뒤에 불러온다.

mdp.py는 Isaac Lab 실행 모듈(pxr 포함)을 import하므로 Kit보다 먼저 불러오면 확장 로딩이 깨진다.
"""
from dataclasses import MISSING

from isaaclab.envs.mdp.actions.actions_cfg import JointPositionActionCfg
from isaaclab.managers import CommandTermCfg
from isaaclab.utils import configclass

ARM_BODIES = ["arm_link_1", "arm_link_2", "arm_link_3", "arm_link_4", "arm_link_5",
              "gripper_left_link", "gripper_right_link"]


@configclass
class RateLimitedJointPositionActionCfg(JointPositionActionCfg):
    class_type: type | str = "waffle_rl.mdp:RateLimitedJointPositionAction"
    max_rate: float = MISSING          # rad/s
    use_default_offset: bool = False


@configclass
class TableTargetCommandCfg(CommandTermCfg):
    class_type: type | str = "waffle_rl.mdp:TableTargetCommand"
    asset_name: str = "robot"
    body_name: str = "arm_link_5"
    table_path: str = MISSING
    tcp_offset: tuple = (0.0, 0.0, -0.08295)
    base_link_z: float = 0.0067499816754000011   # URDF base_footprint_to_base_link z (m)


@configclass
class VelocityCommandJointActionCfg(JointPositionActionCfg):
    """설계 v2: 행동 = 관절 속도 명령(행동 [-1, 1] × max_rate). 위치 목표에 적분하고 속도 목표로도 넣는다."""

    class_type: type | str = "waffle_rl.mdp:VelocityCommandJointAction"
    max_rate: float = MISSING          # rad/s
    use_default_offset: bool = False


@configclass
class RateLimitedFeedforwardJointPositionActionCfg(JointPositionActionCfg):
    """설계 v3 S3: 행동 [-1, 1] → 관절 범위 목표(절대 위치). 목표 이동은 ≤ max_rate, 그 이동 속도를 속도 목표로 넣는다."""

    class_type: type | str = "waffle_rl.mdp:RateLimitedFeedforwardJointPositionAction"
    max_rate: float = MISSING          # rad/s
    use_default_offset: bool = False


@configclass
class FilteredFeedforwardJointPositionActionCfg(JointPositionActionCfg):
    """설계 v4: S3 행동 + 원하는 목표 EMA 필터(alpha)."""

    class_type: type | str = "waffle_rl.mdp:FilteredFeedforwardJointPositionAction"
    max_rate: float = MISSING          # rad/s
    alpha: float = 0.2                 # 정책 스텝(30 Hz)당 반영 비율. 시간 상수 약 0.15 s
    use_default_offset: bool = False


@configclass
class AccelLimitedJointPositionActionCfg(FilteredFeedforwardJointPositionActionCfg):
    """설계 v6: v4 행동 + 물리 스텝마다 가속도·속도 제한 궤적(속도 목표가 고르게 바뀜)."""

    class_type: type | str = "waffle_rl.mdp:AccelLimitedJointPositionAction"
    max_acc: float = MISSING           # rad/s²
    hold_current: bool = False         # True면 위치 목표를 한 물리 스텝 늦춘 궤적 위치로 준다(mdp.py 설명)


@configclass
class DeltaAccelLimitedJointPositionActionCfg(AccelLimitedJointPositionActionCfg):
    """설계 v7: 행동 = 원하는 목표 변화량(정책 스텝마다 ±delta_max rad) + v6 궤적."""

    class_type: type | str = "waffle_rl.mdp:DeltaAccelLimitedJointPositionAction"
    delta_max: float = 0.1             # rad/정책 스텝(30 Hz) = 3 rad/s
    window: float = 0.5                # 원하는 목표 − 실제 관절 최대(rad)


@configclass
class RelativeAccelLimitedJointPositionActionCfg(AccelLimitedJointPositionActionCfg):
    """설계 v8: 행동 = 지금 관절 + a × scale_rad 상대 목표 + v6 궤적."""

    class_type: type | str = "waffle_rl.mdp:RelativeAccelLimitedJointPositionAction"
    scale_rad: float = 0.3


@configclass
class RelativeCmdAccelLimitedJointPositionActionCfg(RelativeAccelLimitedJointPositionActionCfg):
    """설계 v9: 상대 목표 기준 = 명령 중인 궤적 위치 목표."""

    class_type: type | str = "waffle_rl.mdp:RelativeCmdAccelLimitedJointPositionAction"
    window: float = 0.5                # 원하는 목표 − 실제 관절 최대(rad)


@configclass
class DeadbandRelativeCmdJointPositionActionCfg(RelativeCmdAccelLimitedJointPositionActionCfg):
    """설계 v11: v9 행동 + 불감대."""

    class_type: type | str = "waffle_rl.mdp:DeadbandRelativeCmdJointPositionAction"
    deadband: float = 0.05


@configclass
class HoldGateRelativeJointPositionActionCfg(RelativeAccelLimitedJointPositionActionCfg):
    """설계 v14: v8 상대 목표 + 유지 게이트(TCP 오차 hold_r_in 안이면 고정, hold_r_out 밖이면 풀림)."""

    class_type: type | str = "waffle_rl.mdp:HoldGateRelativeJointPositionAction"
    hold_r_in: float = 0.004
    hold_r_out: float = 0.008
    command_name: str = "tcp_target"
    tcp_body: str = "arm_link_5"
    tcp_offset: tuple = (0.0, 0.0, -0.08295)
