"""RL1 팔 목표 위치 도달 환경(차체 바닥 고정, 상태 기반, 위치만).

구조는 Isaac Lab core/reach(Franka reach)를 따른다: dt 1/120, decimation 4(정책 30 Hz), 에피소드 12 s, 목표 4 s마다 바꿈.
바꾼 점: 행동 = 팔 4관절 위치 목표(관절 범위로 옮기고 v12 제어기 속도 한계), 목표 = 충돌 없는 자세 표의 TCP 위치,
리셋 자세 = 같은 표의 무작위 자세, 성공 종료 없음(목표 유지까지 학습), 팔 자기충돌 접촉 벌점 추가.
"""
from pathlib import Path

from isaaclab_physx.physics import PhysxCfg

import isaaclab.envs.mdp as mdp
import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import CurriculumTermCfg as CurrTerm
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import ContactSensorCfg
from isaaclab.utils import configclass
from isaaclab.utils.noise import UniformNoiseCfg as Unoise

from . import mdp_cfg as rl1
from .robot_cfg import ARM_JOINTS, WAFFLE_ARM_FIXED_CFG

RESULTS = Path(__file__).resolve().parents[1] / "results"
TRAIN_TABLE = str(RESULTS / "targets" / "targets_train_seed1.npz")
# TCP = 두 손가락 충돌 메시의 가운데(arm_link_5 좌표, m). 집게 링크 원점(z −0.0577, 손가락 뿌리)에서
# 접근 방향(집게 링크 x = arm_link_5 −z)으로 손가락 메시 x 범위 −0.0145~0.065의 가운데 0.02525 m 앞(사용자 2026-10-25)
TCP_OFFSET = (0.0, 0.0, -0.08295)
ARM_MAX_RATE = 43 * 2 * 3.141592653589793 / 60   # 4.5029 rad/s, XM430-W350 무부하 속도(v12와 같음)

ROOT = "{ENV_REGEX_NS}/Robot/Geometry/base_footprint/base_link"
BODY_PATHS = {
    "base_link": ROOT,
    "wheel_front_left": ROOT + "/wheel_front_left", "wheel_front_right": ROOT + "/wheel_front_right",
    "wheel_rear_left": ROOT + "/wheel_rear_left", "wheel_rear_right": ROOT + "/wheel_rear_right",
    "lidar_front": ROOT + "/lidar_front", "lidar_rear": ROOT + "/lidar_rear",
    "arm_link_1": ROOT + "/arm_link_1",
    "arm_link_2": ROOT + "/arm_link_1/arm_link_2",
    "arm_link_3": ROOT + "/arm_link_1/arm_link_2/arm_link_3",
    "arm_link_4": ROOT + "/arm_link_1/arm_link_2/arm_link_3/arm_link_4",
    "arm_link_5": ROOT + "/arm_link_1/arm_link_2/arm_link_3/arm_link_4/arm_link_5",
}
BODY_PATHS["gripper_left_link"] = BODY_PATHS["arm_link_5"] + "/gripper_left_link"
BODY_PATHS["gripper_right_link"] = BODY_PATHS["arm_link_5"] + "/gripper_right_link"
CONTACT_SENSORS = [f"contact_{b}" for b in rl1.ARM_BODIES]


def _contact_sensor(body):
    # 팔 몸체 하나 × 로봇의 다른 몸체 13개 사이 힘(force_matrix_w). 바닥은 필터에 넣지 않는다.
    others = [p for name, p in BODY_PATHS.items() if name != body]
    return ContactSensorCfg(prim_path=BODY_PATHS[body], filter_prim_paths_expr=others, history_length=0)


@configclass
class RL1SceneCfg(InteractiveSceneCfg):
    ground = AssetBaseCfg(prim_path="/World/ground", spawn=sim_utils.GroundPlaneCfg())
    robot = WAFFLE_ARM_FIXED_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
    light = AssetBaseCfg(prim_path="/World/light",
                         spawn=sim_utils.DomeLightCfg(color=(0.75, 0.75, 0.75), intensity=2500.0))

    def __post_init__(self):
        for body, name in zip(rl1.ARM_BODIES, CONTACT_SENSORS):
            setattr(self, name, _contact_sensor(body))


@configclass
class CommandsCfg:
    tcp_target = rl1.TableTargetCommandCfg(table_path=TRAIN_TABLE, tcp_offset=TCP_OFFSET,
                                           resampling_time_range=(4.0, 4.0), debug_vis=False)


@configclass
class ActionsCfg:
    arm = rl1.RateLimitedJointPositionActionCfg(asset_name="robot", joint_names=ARM_JOINTS, preserve_order=True,
                                                max_rate=ARM_MAX_RATE)


_ARM = SceneEntityCfg("robot", joint_names=ARM_JOINTS, preserve_order=True)
_TCP = SceneEntityCfg("robot", body_names=["arm_link_5"])


@configclass
class ObservationsCfg:
    @configclass
    class PolicyCfg(ObsGroup):
        joint_pos = ObsTerm(func=mdp.joint_pos_rel, params={"asset_cfg": _ARM}, noise=Unoise(n_min=-0.01, n_max=0.01))
        joint_vel = ObsTerm(func=mdp.joint_vel_rel, params={"asset_cfg": _ARM}, noise=Unoise(n_min=-0.01, n_max=0.01))
        target = ObsTerm(func=mdp.generated_commands, params={"command_name": "tcp_target"})
        actions = ObsTerm(func=mdp.last_action)

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()


@configclass
class EventCfg:
    reset_arm = EventTerm(func="waffle_rl.mdp:reset_joints_from_table", mode="reset",
                          params={"command_name": "tcp_target", "asset_cfg": _ARM})


@configclass
class RewardsCfg:
    tcp_error = RewTerm(func="waffle_rl.mdp:tcp_position_error", weight=-0.2,
                        params={"command_name": "tcp_target", "asset_cfg": _TCP, "tcp_offset": TCP_OFFSET})
    tcp_fine = RewTerm(func="waffle_rl.mdp:tcp_position_error_tanh", weight=0.5,
                       params={"std": 0.02, "command_name": "tcp_target", "asset_cfg": _TCP, "tcp_offset": TCP_OFFSET})
    self_contact = RewTerm(func="waffle_rl.mdp:arm_self_contact_count", weight=-1.0,
                           params={"sensor_names": CONTACT_SENSORS, "threshold": 0.1})
    action_rate = RewTerm(func=mdp.action_rate_l2, weight=-0.0001)
    joint_vel = RewTerm(func=mdp.joint_vel_l2, weight=-0.0001, params={"asset_cfg": _ARM})


@configclass
class TerminationsCfg:
    time_out = DoneTerm(func=mdp.time_out, time_out=True)


@configclass
class CurriculumCfg:
    action_rate = CurrTerm(func=mdp.modify_reward_weight,
                           params={"term_name": "action_rate", "weight": -0.005, "num_steps": 4500})
    joint_vel = CurrTerm(func=mdp.modify_reward_weight,
                         params={"term_name": "joint_vel", "weight": -0.001, "num_steps": 4500})


@configclass
class RL1ArmReachEnvCfg(ManagerBasedRLEnvCfg):
    scene: RL1SceneCfg = RL1SceneCfg(num_envs=512, env_spacing=1.5)
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    commands: CommandsCfg = CommandsCfg()
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    events: EventCfg = EventCfg()
    curriculum: CurriculumCfg = CurriculumCfg()

    def __post_init__(self):
        self.decimation = 4
        self.sim.render_interval = self.decimation
        self.episode_length_s = 12.0
        self.sim.dt = 1.0 / 120.0          # v12 물리 dt와 같음
        self.sim.physics = PhysxCfg()      # Isaac Sim PhysX(TGS, v12와 같은 엔진)


# ---------- 설계 v2 (2026-10-25, v1 학습 실패 뒤 사용자 승인) ----------
# v1 실패: 행동을 [-1, 1]로 자른 뒤 관절 범위로 옮겨서 정책 출력이 커져도 동작이 같고 벌점만 커져 발산(seed 1),
# 속도 목표 0이라 0.11 rad/s만 넘어도 토크 포화(seed 2 포화 62 %). 판정 기준 J1~J6은 그대로 둔다.
_ARM5 = SceneEntityCfg("robot", joint_names=["RMX52_.*"])


@configclass
class ActionsV2Cfg:
    arm = rl1.VelocityCommandJointActionCfg(asset_name="robot", joint_names=ARM_JOINTS, preserve_order=True,
                                            max_rate=ARM_MAX_RATE)


@configclass
class RewardsV2Cfg(RewardsCfg):
    # Franka reach에 있던 행동 크기 벌점(v1에서 빠뜨림)
    action_magnitude = RewTerm(func=mdp.action_l2, weight=-0.005)
    # 판정 J4(토크 포화 시간 ≤ 5 %)를 겨냥: 팔 5관절 중 포화 비율
    torque_saturation = RewTerm(func="waffle_rl.mdp:torque_saturation_ratio", weight=-0.5,
                                params={"asset_cfg": _ARM5, "effort": 3.8})


@configclass
class RL1ArmReachV2EnvCfg(RL1ArmReachEnvCfg):
    actions: ActionsV2Cfg = ActionsV2Cfg()
    rewards: RewardsV2Cfg = RewardsV2Cfg()


# ---------- 설계 v3 (2026-10-26, 사용자 승인) ----------
# v1·v2 결과는 처음 USD 복사본의 물리 오류 위에서 나온 것이라 원인 판단에 쓰지 않는다(계획 문서 10장).
# 고친 물리로 다시 돌린 v2: 탐색 초기 포화·접촉 벌점이 추적 보상보다 몇 배 커서 "가만히 있기"로 수렴(10-4).
# v3: 추적 보상을 키우고, 포화·접촉 벌점은 처음 0.1배로 두었다가 4500 정책 스텝 뒤 v2 값으로 올린다.
# 후보 S1 = 속도 명령 행동, S2 = S1 + 초기 표준편차 0.3(agents), S3 = 절대 위치 행동(속도 피드포워드).
@configclass
class RewardsV3Cfg(RewardsV2Cfg):
    tcp_error = RewTerm(func="waffle_rl.mdp:tcp_position_error", weight=-1.0,
                        params={"command_name": "tcp_target", "asset_cfg": _TCP, "tcp_offset": TCP_OFFSET})
    tcp_fine = RewTerm(func="waffle_rl.mdp:tcp_position_error_tanh", weight=1.0,
                       params={"std": 0.05, "command_name": "tcp_target", "asset_cfg": _TCP, "tcp_offset": TCP_OFFSET})
    self_contact = RewTerm(func="waffle_rl.mdp:arm_self_contact_count", weight=-0.1,
                           params={"sensor_names": CONTACT_SENSORS, "threshold": 0.1})
    torque_saturation = RewTerm(func="waffle_rl.mdp:torque_saturation_ratio", weight=-0.05,
                                params={"asset_cfg": _ARM5, "effort": 3.8})


@configclass
class CurriculumV3Cfg(CurriculumCfg):
    self_contact = CurrTerm(func=mdp.modify_reward_weight,
                            params={"term_name": "self_contact", "weight": -1.0, "num_steps": 4500})
    torque_saturation = CurrTerm(func=mdp.modify_reward_weight,
                                 params={"term_name": "torque_saturation", "weight": -0.5, "num_steps": 4500})


@configclass
class ActionsV3S3Cfg:
    arm = rl1.RateLimitedFeedforwardJointPositionActionCfg(asset_name="robot", joint_names=ARM_JOINTS,
                                                           preserve_order=True, max_rate=ARM_MAX_RATE)


@configclass
class RL1ArmReachV3S1EnvCfg(RL1ArmReachV2EnvCfg):
    rewards: RewardsV3Cfg = RewardsV3Cfg()
    curriculum: CurriculumV3Cfg = CurriculumV3Cfg()


@configclass
class RL1ArmReachV3S3EnvCfg(RL1ArmReachV3S1EnvCfg):
    actions: ActionsV3S3Cfg = ActionsV3S3Cfg()


# ---------- 설계 v4 (2026-10-26, 사용자 지시 "학습 내용 수정해서 다시 시작") ----------
# v3 S3: 위치 오차는 크게 줄었으나(중앙값 24 mm) 정책 출력이 떨려 포화 79 %, 정지 RMS 1.1 rad/s, 속도 19 rad/s,
# 접촉 71구간(계획 문서 12장). v4 = S3 + 행동 EMA 필터 + 남은 거리 관찰 + 매끄러움 벌점 강화.
@configclass
class ActionsV4Cfg:
    arm = rl1.FilteredFeedforwardJointPositionActionCfg(asset_name="robot", joint_names=ARM_JOINTS,
                                                        preserve_order=True, max_rate=ARM_MAX_RATE, alpha=0.2)


@configclass
class ObservationsV4Cfg:
    @configclass
    class PolicyCfg(ObservationsCfg.PolicyCfg):
        tcp_error = ObsTerm(func="waffle_rl.mdp:tcp_error_vec_b",
                            params={"command_name": "tcp_target", "asset_cfg": _TCP, "tcp_offset": TCP_OFFSET},
                            noise=Unoise(n_min=-0.002, n_max=0.002))

    policy: PolicyCfg = PolicyCfg()


@configclass
class RewardsV4Cfg(RewardsV3Cfg):
    action_rate = RewTerm(func=mdp.action_rate_l2, weight=-0.01)
    joint_vel = RewTerm(func=mdp.joint_vel_l2, weight=-0.001, params={"asset_cfg": _ARM})


@configclass
class CurriculumV4Cfg(CurriculumV3Cfg):
    action_rate = CurrTerm(func=mdp.modify_reward_weight,
                           params={"term_name": "action_rate", "weight": -0.05, "num_steps": 4500})
    joint_vel = CurrTerm(func=mdp.modify_reward_weight,
                         params={"term_name": "joint_vel", "weight": -0.005, "num_steps": 4500})


@configclass
class RL1ArmReachV4EnvCfg(RL1ArmReachV3S1EnvCfg):
    actions: ActionsV4Cfg = ActionsV4Cfg()
    observations: ObservationsV4Cfg = ObservationsV4Cfg()
    rewards: RewardsV4Cfg = RewardsV4Cfg()
    curriculum: CurriculumV4Cfg = CurriculumV4Cfg()


# ---------- 설계 v5 (2026-10-26, 사용자 승인) ----------
# v4: J2만 통과. 학습 오차가 300 it부터 16 mm에서 평평(정밀 보상 부족), 움직이기 시작할 때 속도 4.5 초과(953/1000구간),
# 차체 근처 목표에서 접촉(103구간). v5 = 정밀 보상(std 0.01) 추가, 목표 이동 상한 3.0 rad/s, 접촉 벌점 최종 −5.
ARM_MAX_RATE_V5 = 3.0   # rad/s, 제어기 상한(모터 무부하 4.5029보다 낮춤)


@configclass
class ActionsV5Cfg:
    arm = rl1.FilteredFeedforwardJointPositionActionCfg(asset_name="robot", joint_names=ARM_JOINTS,
                                                        preserve_order=True, max_rate=ARM_MAX_RATE_V5, alpha=0.2)


@configclass
class RewardsV5Cfg(RewardsV4Cfg):
    tcp_precise = RewTerm(func="waffle_rl.mdp:tcp_position_error_tanh", weight=1.0,
                          params={"std": 0.01, "command_name": "tcp_target", "asset_cfg": _TCP,
                                  "tcp_offset": TCP_OFFSET})


@configclass
class CurriculumV5Cfg(CurriculumV4Cfg):
    self_contact = CurrTerm(func=mdp.modify_reward_weight,
                            params={"term_name": "self_contact", "weight": -5.0, "num_steps": 4500})


@configclass
class RL1ArmReachV5EnvCfg(RL1ArmReachV4EnvCfg):
    actions: ActionsV5Cfg = ActionsV5Cfg()
    rewards: RewardsV5Cfg = RewardsV5Cfg()
    curriculum: CurriculumV5Cfg = CurriculumV5Cfg()


# ---------- 설계 v6 (2026-10-26, 사용자 지시: 실패하면 다음 설계로 바로) ----------
# v5 진단(계획 문서 14장): 속도 목표 펄스(정책 스텝 첫 물리 스텝에만 목표가 움직임)와, 목표 속도가 즉시 3 rad/s가 되어
# 위치 목표가 실제 관절보다 최대 0.34 rad 앞서 나가는 것이 포화·속도 넘침의 원인. v6 = v5 + 물리 스텝마다 가속도 제한 궤적.
ARM_MAX_ACC_V6 = 10.0   # rad/s², 도구 확인 T6(계단 명령)으로 고른 값


@configclass
class ActionsV6Cfg:
    arm = rl1.AccelLimitedJointPositionActionCfg(asset_name="robot", joint_names=ARM_JOINTS, preserve_order=True,
                                                 max_rate=ARM_MAX_RATE_V5, alpha=0.2, max_acc=ARM_MAX_ACC_V6)


@configclass
class RL1ArmReachV6EnvCfg(RL1ArmReachV5EnvCfg):
    actions: ActionsV6Cfg = ActionsV6Cfg()


# ---------- 설계 v7 (2026-10-26, 사용자 지시: 실패하면 다음 설계로 바로) ----------
# v6: 실제 속도 넘침은 고쳤으나(접촉 없는 구간 최대 3.05 rad/s) 정지 구간에서 정책 출력이 흔들려 J5 1000구간 모두 실패,
# 학습 오차는 16 mm 근처에서 평평(절대 위치 행동의 분해능, 계획 문서 15장). v7 = 변화량 행동 + 원하는 목표 − 관절 관찰.
@configclass
class ActionsV7Cfg:
    arm = rl1.DeltaAccelLimitedJointPositionActionCfg(asset_name="robot", joint_names=ARM_JOINTS, preserve_order=True,
                                                      max_rate=ARM_MAX_RATE_V5, max_acc=ARM_MAX_ACC_V6,
                                                      delta_max=0.1, window=0.5)


@configclass
class ObservationsV7Cfg:
    @configclass
    class PolicyCfg(ObservationsV4Cfg.PolicyCfg):
        desired_gap = ObsTerm(func="waffle_rl.mdp:arm_desired_minus_pos",
                              params={"action_name": "arm", "asset_cfg": _ARM})

    policy: PolicyCfg = PolicyCfg()


@configclass
class RL1ArmReachV7EnvCfg(RL1ArmReachV6EnvCfg):
    actions: ActionsV7Cfg = ActionsV7Cfg()
    observations: ObservationsV7Cfg = ObservationsV7Cfg()


# ---------- 설계 v8 (2026-10-26, 사용자 지시: 실패하면 다음 설계로 바로) ----------
# v7: 변화량 행동으로 정지 떨림은 없앴으나(J5 중앙값 0.0001) 학습 오차가 105 mm에서 멈춤(계획 문서 16장).
# v8 = 상대 목표 행동(지금 관절 + 0.3 rad × a) + v6 궤적 + 궤적 목표 − 관절 관찰 + 관찰 정규화(agents).
@configclass
class ActionsV8Cfg:
    arm = rl1.RelativeAccelLimitedJointPositionActionCfg(asset_name="robot", joint_names=ARM_JOINTS,
                                                         preserve_order=True, max_rate=ARM_MAX_RATE_V5,
                                                         max_acc=ARM_MAX_ACC_V6, scale_rad=0.3)


@configclass
class RL1ArmReachV8EnvCfg(RL1ArmReachV7EnvCfg):
    actions: ActionsV8Cfg = ActionsV8Cfg()


# ---------- 설계 v9 (2026-10-27, 사용자 지시: 실패하면 다음 설계로 바로) ----------
# v8: J1 92.9 %·J2 1.03 s·속도 2.9 rad/s로 좋아졌으나 정지 구간에서 중력 처짐을 버티느라 출력이 흔들려 J5 998구간 실패,
# 접촉 37구간(손목·집게가 차체를 침, 계획 문서 17장). v9 = 상대 목표 기준을 명령 목표로 + 커리큘럼 뒤 접촉 종료.
@configclass
class ActionsV9Cfg:
    arm = rl1.RelativeCmdAccelLimitedJointPositionActionCfg(asset_name="robot", joint_names=ARM_JOINTS,
                                                            preserve_order=True, max_rate=ARM_MAX_RATE_V5,
                                                            max_acc=ARM_MAX_ACC_V6, scale_rad=0.3, window=0.5)


@configclass
class TerminationsV9Cfg(TerminationsCfg):
    arm_contact = DoneTerm(func="waffle_rl.mdp:arm_contact_after",
                           params={"sensor_names": CONTACT_SENSORS, "threshold": 0.1, "start_step": 4500})


@configclass
class RL1ArmReachV9EnvCfg(RL1ArmReachV8EnvCfg):
    actions: ActionsV9Cfg = ActionsV9Cfg()
    terminations: TerminationsV9Cfg = TerminationsV9Cfg()


# ---------- 설계 v10 (2026-10-27, 사용자 지시: 실패하면 다음 설계로 바로) ----------
# v9: 기준 변경은 정지 떨림을 못 고쳤고(J5 중앙값 0.204) 접촉 종료는 접촉·J1을 나쁘게 함(계획 문서 18장).
# 정지 떨림은 정밀 보상(std 0.01)을 넣은 v5부터 생김. v10 = v8 + 목표 20 mm 안 관절 속도 벌점.
@configclass
class RewardsV10Cfg(RewardsV5Cfg):
    settle = RewTerm(func="waffle_rl.mdp:joint_vel_l2_near_target", weight=-1.0,
                     params={"radius": 0.02, "command_name": "tcp_target", "asset_cfg": _ARM, "tcp_cfg": _TCP,
                             "tcp_offset": TCP_OFFSET})


@configclass
class RL1ArmReachV10EnvCfg(RL1ArmReachV8EnvCfg):
    rewards: RewardsV10Cfg = RewardsV10Cfg()


# ---------- 설계 v11 (2026-10-27, 사용자 지시: 실패하면 다음 설계로 바로) ----------
# v10: 정착 벌점(−1.0)도 떨림을 못 없애고 접근만 느려짐(J1 59 %, J2 2.07 s). 궤적 생성기만으로는 멈춤(계획 문서 19장)
# → 떨림은 상대 목표(적분기) 고리에서 생김. v11 = 명령 목표 기준 상대 목표 + 불감대 0.05 + 정착 벌점 −0.3, 접촉 종료 없음.
@configclass
class ActionsV11Cfg:
    arm = rl1.DeadbandRelativeCmdJointPositionActionCfg(asset_name="robot", joint_names=ARM_JOINTS,
                                                        preserve_order=True, max_rate=ARM_MAX_RATE_V5,
                                                        max_acc=ARM_MAX_ACC_V6, scale_rad=0.3, window=0.5,
                                                        deadband=0.05)


@configclass
class RewardsV11Cfg(RewardsV10Cfg):
    settle = RewTerm(func="waffle_rl.mdp:joint_vel_l2_near_target", weight=-0.3,
                     params={"radius": 0.02, "command_name": "tcp_target", "asset_cfg": _ARM, "tcp_cfg": _TCP,
                             "tcp_offset": TCP_OFFSET})


@configclass
class RL1ArmReachV11EnvCfg(RL1ArmReachV8EnvCfg):
    actions: ActionsV11Cfg = ActionsV11Cfg()
    rewards: RewardsV11Cfg = RewardsV11Cfg()


# ---------- 설계 v12 (2026-10-27, 사용자 지시: 실패하면 다음 설계로 바로) ----------
# v11: 불감대·정착 벌점도 유지 떨림을 못 없앰(J5 중앙 0.163). 정책 없이 목표를 고정하면 0.001 rad/s로 멈추고,
# v8 정책의 유지 구간 출력은 부호가 정책 스텝의 70 %에서 바뀌는 15 Hz 떨림(|a| 약 0.2). 행동 변화 벌점(최종 −0.05)이
# 떨림 비용으로 스텝당 약 0.03에 그쳐 추적 보상(스텝당 최대 약 2)에 비해 작다. v12 = v8 + 행동 변화 벌점 최종 −0.5(10배).
@configclass
class CurriculumV12Cfg(CurriculumV5Cfg):
    action_rate = CurrTerm(func=mdp.modify_reward_weight,
                           params={"term_name": "action_rate", "weight": -0.5, "num_steps": 4500})


@configclass
class RL1ArmReachV12EnvCfg(RL1ArmReachV8EnvCfg):
    curriculum: CurriculumV12Cfg = CurriculumV12Cfg()


# ---------- 설계 v13 (2026-10-27, 사용자 지시: 실패하면 다음 설계로 바로) ----------
# v12: 행동 변화 벌점 10배로도 유지 떨림 그대로(J5 중앙 0.196, 행동 std 0.03). 정밀 보상(std 0.01)이 없던 v4는 J5 중앙
# 0.0001, 넣은 v5부터 0.2~0.34. 1 − tanh(d/σ)는 d=0에서 꺾이고 기울기가 가장 커서 서브 mm 오차를 쫓는 떨림을 만든다고 본다.
# v13 = v8 + 정밀 보상을 1 − tanh((d/σ)²)(σ 0.01)로. 5~15 mm 기울기는 비슷하게 남기고 0 근처 기울기만 0으로.
@configclass
class RewardsV13Cfg(RewardsV5Cfg):
    tcp_precise = RewTerm(func="waffle_rl.mdp:tcp_position_error_tanh_sq", weight=1.0,
                          params={"std": 0.01, "command_name": "tcp_target", "asset_cfg": _TCP,
                                  "tcp_offset": TCP_OFFSET})


@configclass
class RL1ArmReachV13EnvCfg(RL1ArmReachV8EnvCfg):
    rewards: RewardsV13Cfg = RewardsV13Cfg()


# ---------- 설계 v14 (2026-10-27, 사용자 지시: 실패하면 다음 설계로 바로) ----------
# v13: 정밀 보상 꼴을 바꿔도 떨림 그대로(J5 중앙 0.215). 진단(판정 아님): v8 정책에 출력 EMA는 효과 없음(β 0.5 → 0.210),
# 유지 게이트(6 mm 안 고정, 9 mm 밖 풀림)는 J5 중앙 0.0001·J1 94.1 %. v14 = v8 + 유지 게이트를 넣고 학습.
@configclass
class ActionsV14Cfg:
    arm = rl1.HoldGateRelativeJointPositionActionCfg(asset_name="robot", joint_names=ARM_JOINTS,
                                                     preserve_order=True, max_rate=ARM_MAX_RATE_V5,
                                                     max_acc=ARM_MAX_ACC_V6, scale_rad=0.3,
                                                     hold_r_in=0.006, hold_r_out=0.009, tcp_offset=TCP_OFFSET)


@configclass
class RL1ArmReachV14EnvCfg(RL1ArmReachV8EnvCfg):
    actions: ActionsV14Cfg = ActionsV14Cfg()


# ---------- 설계 v15 (2026-10-27, 사용자 결정: J4 토크 포화는 PhysX 실제 관절 힘으로 잼) ----------
# v14: 학습 중 유지 게이트는 J1을 떨어뜨림(9 mm 바로 밖에 멈춤). v8 정책 + 실행 때 게이트(진단)는 J1 94.1 %·J5 중앙 0.0001.
# 남은 접촉 42구간 중 35는 목표가 아니라 가는 길(팔을 뒤로 넘길 때 차체 뒤 위 구조, 앞 아래 라이다·상판)에서 남(계획 문서 25장).
# v15 = v8 학습 + 가는 길 차체 거리 벌점(팔 충돌 형상 점 ↔ 차체 충돌 형상, 여유 30 mm, TCP 오차 30 mm 넘을 때만),
#       실행(평가) 때는 유지 게이트 6/9 mm(RL1ArmReachV15PlayEnvCfg).
CLEARANCE_BODIES = ["arm_link_3", "arm_link_4", "arm_link_5", "gripper_left_link", "gripper_right_link"]
_CLR = SceneEntityCfg("robot", body_names=CLEARANCE_BODIES, preserve_order=True)


@configclass
class RewardsV15Cfg(RewardsV5Cfg):
    chassis_clearance = RewTerm(func="waffle_rl.mdp:arm_chassis_clearance_penalty", weight=-0.2,
                                params={"margin": 0.03, "gate_err": 0.03, "command_name": "tcp_target",
                                        "asset_cfg": _CLR, "tcp_cfg": _TCP, "tcp_offset": TCP_OFFSET})


@configclass
class CurriculumV15Cfg(CurriculumV5Cfg):
    chassis_clearance = CurrTerm(func=mdp.modify_reward_weight,
                                 params={"term_name": "chassis_clearance", "weight": -2.0, "num_steps": 4500})


@configclass
class RL1ArmReachV15EnvCfg(RL1ArmReachV8EnvCfg):
    rewards: RewardsV15Cfg = RewardsV15Cfg()
    curriculum: CurriculumV15Cfg = CurriculumV15Cfg()


@configclass
class ActionsV15PlayCfg:
    arm = rl1.HoldGateRelativeJointPositionActionCfg(asset_name="robot", joint_names=ARM_JOINTS,
                                                     preserve_order=True, max_rate=ARM_MAX_RATE_V5,
                                                     max_acc=ARM_MAX_ACC_V6, scale_rad=0.3,
                                                     hold_r_in=0.006, hold_r_out=0.009, tcp_offset=TCP_OFFSET)


@configclass
class RL1ArmReachV15PlayEnvCfg(RL1ArmReachV15EnvCfg):
    """v15 실행(평가·화면 재생)용: 학습한 v15 정책 + 유지 게이트. 관찰·보상은 학습과 같다."""
    actions: ActionsV15PlayCfg = ActionsV15PlayCfg()


# ---------- 설계 v16 (2026-10-27, 사용자 지시: 다른 일을 끝낸 뒤 묻지 말고 RL 진행) ----------
# v15: 가는 길 차체 거리 벌점(최종 −2)으로 접촉이 줄지 않음(58구간). 학습 끝 거리 벌점 에피소드당 −0.025, 접촉 벌점(−5)도
# 보상 배율(정책 스텝 dt 1/30)을 거치면 접촉 구간 하나에 약 −1.7로, 에피소드 보상(약 8)과 빠른 도달 이득에 비해 작다고 본다.
# v16 = v15 + 접촉 벌점 최종 −5 → −50, 차체 거리 벌점 최종 −2 → −20(여유 30 mm·가는 길만 그대로). 실행 때 유지 게이트 6/9 mm.
@configclass
class CurriculumV16Cfg(CurriculumV15Cfg):
    self_contact = CurrTerm(func=mdp.modify_reward_weight,
                            params={"term_name": "self_contact", "weight": -50.0, "num_steps": 4500})
    chassis_clearance = CurrTerm(func=mdp.modify_reward_weight,
                                 params={"term_name": "chassis_clearance", "weight": -20.0, "num_steps": 4500})


@configclass
class RL1ArmReachV16EnvCfg(RL1ArmReachV15EnvCfg):
    curriculum: CurriculumV16Cfg = CurriculumV16Cfg()


@configclass
class RL1ArmReachV16PlayEnvCfg(RL1ArmReachV16EnvCfg):
    """v16 실행(평가·화면 재생)용: 학습한 v16 정책 + 유지 게이트."""
    actions: ActionsV15PlayCfg = ActionsV15PlayCfg()
