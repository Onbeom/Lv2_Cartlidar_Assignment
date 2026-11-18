"""스토리지에서 퍽 꺼내기 강화학습 설정 (2026-11-10, 환경 unload_env.py).

미션 (발제 v5): 놓기는 L1 · L3 205 상자 = 집게 60°, L2 820 받침대 북쪽 = 45° -> 꺼낼 때부터 그 피치로 쥠 (손 안에서 바꾸지 않음).
관측 · 행동은 45° 집기 (SidePick45Chain, 관측 33 · 행동 4) 와 같아 그 체크포인트에서 이어 학습.
"""

from __future__ import annotations

import math

import isaaclab.sim as sim_utils
from isaaclab.sensors import ContactSensorCfg
from isaaclab.utils import configclass

from .place_env_cfg import PlaceSceneCfg
from .robot_cfg import BODY_LINK, FINGER_OPEN_TASK
from .sidepick_env_cfg import WaffleSidePick45ChainEnvCfg


@configclass
class WaffleUnload60EnvCfg(WaffleSidePick45ChainEnvCfg):
    scene: PlaceSceneCfg = PlaceSceneCfg(num_envs=2048, env_spacing=1.5, replicate_physics=True, clone_in_fabric=True)
    episode_length_s = 8.0
    unload_pitch = "60"
    unload_nb_prob = 0.9  # 옆 칸에 퍽 있음
    unload_fill_prob = 0.7  # 다른 칸 (최대 5 개) 에 퍽 있음
    rew_obs_hit = -30.0  # 다른 칸 퍽에 닿음 + 종료

    def __post_init__(self):
        super().__post_init__()
        self.grasp_pitch = math.radians(float(self.unload_pitch))
        self.gripper_open = FINGER_OPEN_TASK  # 49.6 mm: 퍽 양옆 약 10 mm, 옆 칸 퍽 (중심 54 mm) 안 닿음
        self.perc_bias = 0.003  # 칸 설계 좌표 vs 실물 (보정 오차) — 실측 뒤 줄임
        self.perc_jitter = 0.0005
        self.ready_noise = 0.05
        self.success_lift = 0.06  # 칸 벽 (바닥 + 30 mm) 위로 퍽 바닥이 3 cm
        self.lift_goal = 0.08
        self.pick_j3_max = None
        self.dock_layout = False
        self.dock_rand_prob = 0.0
        self.near_start_prob = 0.0
        self.sim = self.sim.replace(physics=self.sim.physics.replace(gpu_max_rigid_patch_count=2**19))
        # 손가락은 칸 벽 · 홈에 닿아도 됨 (팔레트 = base_link) -> 팔 링크 · 집게 몸만 몸체 접촉 종료
        self.scene.arm_body_contact = ContactSensorCfg(
            prim_path="{ENV_REGEX_NS}/Robot/(link[2-5]|gripper_roll_link)",
            filter_prim_paths_expr=["{ENV_REGEX_NS}/Robot/" + BODY_LINK], update_period=0.0, history_length=1)
        mat = sim_utils.RigidBodyMaterialCfg(static_friction=0.5, dynamic_friction=0.5, friction_combine_mode="min")
        for k in [f"obs{j}" for j in range(5)]:
            o = getattr(self.scene, k)
            setattr(self.scene, k, o.replace(spawn=o.spawn.replace(physics_material=mat)))


@configclass
class WaffleUnload45EnvCfg(WaffleUnload60EnvCfg):
    """L2 (발제 v5: 820 받침대 북쪽) 놓기용 45° 꺼내기 (꺼낼 수 있는 칸 8 개, 1열 4 칸은 자기 충돌 여유 부족)."""

    unload_pitch = "45"
