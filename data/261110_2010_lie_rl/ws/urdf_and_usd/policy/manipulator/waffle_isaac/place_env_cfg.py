"""주사위 눈 놓기 (2026-11-07): 스토리지에서 60° 로 쥐고 꺼낸 퍽을 상자 위 3 x 3 격자 (간격 60 mm) 의 지정 칸에 세워 놓기.

미션: 장소마다 주사위 눈 1 ~ 6 개 (오차 10 mm, 잘못 놓은 퍽 다시 집기 없음). 차체는 상자 앞면 5 ~ 6 cm 에 멈춤 (사용자 확정).
기구 (팀 URDF FK, 정지 55 mm, 퍽 바닥 = 상자 위 + 3 mm): 집게 45° 면 앞줄 3 칸이 관절3 77 ~ 80° (상한 74.5° 넘음),
**60° 면 9 칸 모두 관절3 60° 이하** (앞줄 56 ~ 60, 가운데 32 ~ 37, 먼 줄 0 ~ 4°). 스토리지 12 칸 60° 꺼내기도 됨 -> 놓기는 60° 쥐기.

과제: 시작 = 퍽을 쥔 채 상자 위 (잡는 점 관절1 축 0.20 m, 퍽 바닥이 상자 위 6 cm), 손 안 퍽은 쥘 때 어긋남 (집게 방향 · 위아래 ±2.5 mm).
먼저 놓은 퍽 (같은 주사위 눈의 앞 순서 칸, 먼 줄부터 놓음) 은 장애물. 정책 관측의 퍽 위치 = 놓기 가정 (잡는 점 15 mm 아래), 칸 위치 = 인식 (±2 mm 치우침).
행동 = 45° 집기와 같음 (잡는 점 직교 이동 + 집게), 집게 피치만 60°. 놓기 = 집게 벌림 (조금만, 옆 퍽 안 치게).
"""

from __future__ import annotations

import math

import isaaclab.sim as sim_utils
from isaaclab.assets import RigidObjectCfg
from isaaclab.sensors import ContactSensorCfg
from isaaclab.utils import configclass

from .dice2_cfg import puck_cfg
from .robot_cfg import BODY_LINK, FINGER_HOLD_30, FINGER_RELEASED
from .sidepick_env_cfg import SidePickSceneCfg, WaffleSidePickEnvCfg

N_OBS = 5  # 먼저 놓은 퍽 (장애물) 최대 수 (주사위 6 의 마지막 칸 앞에 5 개)
PLACE_START_POSE = {"joint1": 0.0, "joint2": -0.2454, "joint3": 0.2999, "joint4": 0.9927, "joint5": 0.0}
"""퍽 쥔 시작 자세: 잡는 점 base (0.296, 0, 0.1682) = 관절1 축 0.20 m, 퍽 바닥이 상자 위 6 cm, 집게 60° 아래 (관절 2 · 3 · 4 = −14 · 17 · 57°)."""
PLACE_START_GRASP_B = (0.296, 0.0, 0.1682)
L2_SIZE = 0.82  # [m] L2 받침 = 410 mm 상자 4 개 (0.82 x 0.82 x 0.07, dice_mission.sdf place_L2_pedestal)
L2_GRID_FROM_FACE = 0.21  # [m] 받침 앞면 (y 3.91) -> 격자 가운데 (L2 좌표 y 3.70)
GRID = 0.06  # [m] 격자 간격
DICE = {  # (u: +먼 줄 / −앞줄, v: +왼 / −오른) 칸, 돌린 모양 포함
    1: [[(0, 0)]],
    2: [[(1, 1), (-1, -1)], [(1, -1), (-1, 1)]],
    3: [[(1, 1), (0, 0), (-1, -1)], [(1, -1), (0, 0), (-1, 1)]],
    4: [[(1, 1), (1, -1), (-1, 1), (-1, -1)]],
    5: [[(1, 1), (1, -1), (0, 0), (-1, 1), (-1, -1)]],
    6: [[(1, 1), (0, 1), (-1, 1), (1, -1), (0, -1), (-1, -1)], [(1, 1), (1, 0), (1, -1), (-1, 1), (-1, 0), (-1, -1)]],
}


def _obs_pucks():
    out = {}
    for j in range(N_OBS):
        p = puck_cfg(f"Obs{j}", "cyl", (0.2, 0.4, 0.9), (-0.5, 0.40 + 0.07 * j))
        out[f"obs{j}"] = p
    return out


@configclass
class PlaceSceneCfg(SidePickSceneCfg):
    obs0 = _obs_pucks()["obs0"]
    obs1 = _obs_pucks()["obs1"]
    obs2 = _obs_pucks()["obs2"]
    obs3 = _obs_pucks()["obs3"]
    obs4 = _obs_pucks()["obs4"]
    platformL2 = RigidObjectCfg(  # L2 받침 (쓰지 않을 때는 땅 아래로 치움)
        prim_path="{ENV_REGEX_NS}/PlatformL2",
        spawn=sim_utils.CuboidCfg(size=(0.82, 0.82, 0.07), rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
                                  collision_props=sim_utils.CollisionPropertiesCfg(), visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.8, 0.68, 0.48))),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(0.0, 0.0, -0.5)))
    arm_plat_contact = ContactSensorCfg(  # 팔 · 손가락이 L2 받침에 닿음 (상자 접촉과 같은 벌점)
        prim_path="{ENV_REGEX_NS}/Robot/" + "(link[2-5]|gripper_roll_link|gripper_left_link|gripper_right_link)",
        filter_prim_paths_expr=["{ENV_REGEX_NS}/PlatformL2"], update_period=0.0, history_length=1,
    )
    arm_obs_contact = ContactSensorCfg(  # 팔 · 손가락이 먼저 놓은 퍽에 닿음
        prim_path="{ENV_REGEX_NS}/Robot/" + "(link[2-5]|gripper_roll_link|gripper_left_link|gripper_right_link)",
        filter_prim_paths_expr=[f"{{ENV_REGEX_NS}}/PuckObs{j}" for j in range(N_OBS)], update_period=0.0, history_length=1,
    )
    arm_body_contact = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/Robot/" + "(link[2-5]|gripper_roll_link|gripper_left_link|gripper_right_link)",
        filter_prim_paths_expr=["{ENV_REGEX_NS}/Robot/" + BODY_LINK], update_period=0.0, history_length=1,
    )


@configclass
class WafflePlace60EnvCfg(WaffleSidePickEnvCfg):
    scene: PlaceSceneCfg = PlaceSceneCfg(num_envs=256, env_spacing=1.5, replicate_physics=True, clone_in_fabric=True)
    observation_space = 46
    action_space = 4
    episode_length_s = 10.0  # 2026-11-10 2 초 안정 판정이 들어가게 (이전 8)

    # 배치 (정지 거리 5 ~ 6 cm)
    surface = "box"  # box = L1 · L3 (205 상자, 격자 가운데 = 상자 가운데), l2 = L2 받침 (0.82 판, 격자 가운데 = 앞면에서 0.21 m)
    start_grasp_b = PLACE_START_GRASP_B  # 시작 자세 잡는 점 (base)
    start_table = "place_starts60.json"  # 2026-11-09 꺼낸 뒤 위로 가져온 자세 표 (격자 ±7.5 cm · 좌우 ±10 cm · 퍽 바닥 윗면 위 3 ~ 10 cm), "" = 고정 자세
    start_table_prob = 1.0
    place_gap = (0.05, 0.06)
    place_lat = (-0.04, 0.04)
    place_yaw = 0.087
    inhand_err = 0.0025  # [m] 손 안 퍽 어긋남 (집게 방향 수평 · 위아래 각각 ±)
    cell_perc_bias = 0.002  # [m] 칸 인식 치우침
    perc_delay_steps = (3, 9)  # 2026-11-10 #191 팔 카메라 실측: 지연 105 ~ 278 ms
    perc_period_steps = (3, 10)  # 3.2 ~ 9 Hz
    perc_drop_p = (0.1, 0.4)  # 팔 움직일 때 끊김
    box_puck_prob = 0.5

    # 판정
    release_thresh = 0.5  # 집게 행동이 이보다 커야 벌림 (−1 ~ 1)
    place_tol = 0.010  # [m] 미션 오차
    release_xy_max = 0.012  # 이보다 어긋나게 놓으면 실패 · 종료
    release_h_max = 0.012  # 퍽 바닥이 상자 위 이보다 높을 때 놓으면 실패
    retract_dist = 0.04  # [m] 놓은 뒤 잡는 점이 퍽 중심에서 이만큼 멀어지면 끝난 것
    success_hold_steps = 60  # 2026-11-10 발제 v5: 중심 10 mm 안 + 2 초 안정 (30 Hz x 2 s, 이전 15 = 0.5 s)
    obs_move_max = 0.003  # 먼저 놓은 퍽이 이만큼 밀리면 실패

    # 보상
    # 2026-11-08 첫 학습: 칸 위 5.6 mm 까지 가져가지만 놓지 않고 시간 끝 (58/64) - 들고 있기만 해도 스텝마다 최대 +5 (8 s 면 1000 넘음) > 놓기 성공 +40.
    # -> 들고 있는 동안 보상은 '가까워진 만큼' (퍼텐셜 차이, 머물면 0) + 시간 벌점, 놓기 · 성공 보상을 크게
    rew_xy = 0.0
    rew_xy_fine = 0.0
    rew_lower = 0.0
    rew_phi = 20.0  # 퍼텐셜 φ = −(칸 수평 오차 / 1 cm + 맞을수록 무게 둔 높이 / 1 cm) 의 스텝 차이 x 이것
    rew_time = -0.05  # 스텝마다 (놓고 끝낼 때까지)
    rew_low_far = -1.0  # 칸 위가 아닌데 퍽 바닥이 상자 위 2 cm 아래 (옆 퍽 · 상자 칠 위험)
    rew_held_tilt_p = -2.0  # 든 퍽 기울기 / 5° (최대 3 배)
    rew_release_good = 30.0
    rew_release_bad = -30.0  # + 종료
    rew_drop = -20.0  # 놓기 전에 손에서 빠짐 + 종료
    rew_retract = 1.0
    rew_after_move = -10.0  # 놓은 뒤 퍽이 밀림 [/cm]
    rew_success = 100.0
    rew_topple_p = -30.0  # 놓은 퍽 쓰러짐 · 상자 밖 + 종료
    rew_obs_hit = -30.0  # 먼저 놓은 퍽에 닿음 · 밀림 + 종료

    def __post_init__(self):
        super().__post_init__()
        self.grasp_pitch = math.radians(60.0)
        # 2026-11-08 환경 2048 대 (상자 · 퍽 8 개) 에서 PhysX 'Patch buffer overflow ... at least 254277' -> 접촉이 빠져 물리가 틀어짐. 기본 163840 -> 2^19
        self.sim = self.sim.replace(physics=self.sim.physics.replace(gpu_max_rigid_patch_count=2**19))
        self.gripper_open = FINGER_RELEASED  # 놓을 때 조금만 벌림 (옆 칸 퍽 30 mm 틈)
        self.pick_j3_max = 1.3
        self.near_start_prob = 0.0
        self.ready_noise = 0.0
        self.rew_self_end = -100.0
        self.rew_j3_bend = -5.0
        self.rew_arm_body_end = -50.0
        mat = sim_utils.RigidBodyMaterialCfg(static_friction=0.5, dynamic_friction=0.5, friction_combine_mode="min")
        for k in ["puckT_cyl", "puckT_box", "puckN"] + [f"obs{j}" for j in range(N_OBS)]:
            o = getattr(self.scene, k)
            setattr(self.scene, k, o.replace(spawn=o.spawn.replace(physics_material=mat)))
        self.scene.robot = self.scene.robot.replace(init_state=self.scene.robot.init_state.replace(
            joint_pos={**self.scene.robot.init_state.joint_pos, **PLACE_START_POSE,
                       "gripper_left_joint": FINGER_HOLD_30, "gripper_right_joint": FINGER_HOLD_30}))


PLACE45_START_POSE = {"joint1": 0.0, "joint2": -0.0204, "joint3": 0.2807, "joint4": 0.5251, "joint5": 0.0}
"""L2 퍽 쥔 시작 자세: 잡는 점 base (0.356, 0, 0.1682) = 관절1 축 0.26 m, 퍽 바닥이 판 위 6 cm, 집게 45° 아래 (관절 2 · 3 · 4 = −1 · 16 · 30°)."""


@configclass
class WafflePlace45L2EnvCfg(WafflePlace60EnvCfg):
    """L2 받침 (0.82 x 0.82 x 0.07, 격자 가운데가 앞면에서 0.21 m) 에 주사위 눈 놓기 (2026-11-08). 정지 5 ~ 6 cm 면 격자 칸 관절1 축 0.25 ~ 0.37 m
    -> 60° 는 먼 줄 3 칸 못 닿음, **45° 면 9 칸 모두 (관절3 앞줄 35 ~ 38 · 가운데 0 ~ 4 · 먼 줄 −55 ~ −69°)** (팀 URDF FK). 스토리지에서 45° 로 꺼냄 (팀 넣기 경로와 같은 쥐기)."""

    def __post_init__(self):
        super().__post_init__()
        self.surface = "l2"
        self.grasp_pitch = math.radians(45.0)
        self.start_grasp_b = (0.356, 0.0, 0.1682)
        self.start_table = "place_starts45l2.json"
        self.scene.robot = self.scene.robot.replace(init_state=self.scene.robot.init_state.replace(
            joint_pos={**self.scene.robot.init_state.joint_pos, **PLACE45_START_POSE}))
