"""B안 팔레트 놓기 고정 궤적 확인 (Isaac PhysX, 학습 없음): pallet_place_traj.json 을 그대로 따라 쥔 퍽을 칸에 놓아 봄.

대마다 칸 하나 x 퍽 모양 하나 (칸 12 x 원기둥 · 상자 = 24 대, 다른 칸은 비어 있음).
  1. 앞 운반 자세 (q_front) 에서 집게를 벌리고 퍽을 손가락 사이 (TCP = 퍽 중심 + 15 mm, 집게 아래) 에 둔 뒤 쥠
  2. 궤적 (rotate -> carry -> place) 을 관절 속도 한도 (--speed) 로 따라감, 놓는 높이에서 잠깐 멈춤
  3. 집게 벌림 (FINGER_RELEASED, 37.6 mm = 팔레트 설계 grip_open 38) -> retract -> return -> rotate_back -> 1 s 기다림
  4. 결과: 퍽 바닥 가운데 - 칸 축 (수평), 퍽 바닥 - 칸 바닥 (높이), 기울기, 상자 퍽 면 방향, 옮기는 중 퍽이 손가락에서 벗어난 최대 거리,
     팔 링크 · 손가락 · 퍽이 몸체 (base_link = 차체 · 팔레트 · 기둥) 에 닿은 최대 힘 (단계별), 처음 닿은 구간 · 자세
  퍽 마찰 (--puck_mu, 기본 0.5): 퍽-팔레트 마찰. 팀 접촉 재질 1.0 그대로면 칸에서 끼임.
  기둥 옆 4칸 (궤적 tilt_deg 45): 퍽을 세운 채 45° 기울인 집게에 쥐어 줌 (앞 운반 자세 = 칸의 q_front), 그 칸만 --tilt_speed.
  쥐기 (--grip_close, --settle_ki): 아래 처짐 보정을 먼저 자리 잡힌 뒤 쥠, 손가락 목표는 두 손가락 힘이 같은 값.
  처짐 보정 (--ki): 팀 팔 구동은 펌웨어 위치 P 800 · I 0 (강성 18.88 N·m/rad) 이라 퍽을 쥔 팔이 6 ~ 13 mm 처진다 (ki 0 으로 확인).
     호스트 (ROS 노드) 가 측정 관절값으로 목표를 고쳐 주는 적분 보정을 흉내 냄: 목표 = 계획 + bias, bias += ki · dt · (계획 − 측정).

  빼기 (--unload): 칸에 앉힌 퍽을 놓기 궤적 거꾸로 꺼냄 = 빈 집게로 다가가기 (rotate_back -> return -> retract 거꾸로, 손가락 FINGER_RELEASED)
     -> 놓는 자세에서 쥠 -> place -> carry -> rotate 거꾸로 (쥔 채) -> 앞 운반 자세. 결과: 쥔 퍽 - 손가락 가운데, 퍽 기울기 (집게 기준), 칸 축에서 벗어난 거리.
  속도: 쥔 구간 = --row_speed (그 열) > --tilt_speed (기울인 칸) > --speed, 빈 집게 구간 = --empty_speed (없으면 쥔 구간과 같음).

    ./run_rl.sh pallet_check            # -> pallet_place_check.json
    ./run_rl.sh pallet_unload           # -> pallet_unload_check.json
    ./run_rl.sh pallet_unload --neighbors pairs   # 이웃 칸 퍽: 같은 열 바로 옆 칸에 퍽을 앉혀 두고 빼기 -> 결과 nb_max_speed (이웃이 밀린 최대 속도) · nb_slot
                                        #   slot = 같은 열 · 뒤 열 모두, behind = 뒤 열만, opposite = 같은 곳 모양 반대, none = 비움 (기본)
                                        #   막는 쌍 (2026-11-13): 1 열 1_0 ↔ 1_1 · 1_2 ← 1_3 -> mission_planner slots.BLOCKERS. 디버그 NB_DEBUG=1 · NB_TRACE=1
    ./run_rl.sh pallet_unload --place --num_envs 180 --seed 1   # 꺼낸 퍽을 장소 상자 칸에 놓기 (ROS placeAt 경로) -> dice_place_eval.json
                                        #   대 = (칸 10 x 모양 2) x 상자 칸 9. 상자 = 차체 앞 끝 5 ~ 6 cm (--box_gap), 좌우 ±2 cm, 요 ±2°
                                        #   --box_neighbors far (먼 칸에 퍽, 계획기 place_order 순서) · --box_err 2 --box_err_deg 2 (상자 측정 오차) · --no-place_comp
    ./run_rl.sh pallet_check_play       # 창으로 (대 수를 줄여 보기: --num_envs 4)
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--traj", default=None, help="궤적 파일 (기본 pallet_place_traj.json)")
parser.add_argument("--speed", type=float, default=0.8, help="[rad/s] 관절 목표 속도 한도")
parser.add_argument("--num_envs", type=int, default=24, help="대 수 (칸 12 x 원기둥 · 상자 순서로 앞에서부터)")
parser.add_argument("--ki", type=float, default=4.0, help="[1/s] 호스트 쪽 처짐 보정: 관절 목표 = 계획 + 적분 (계획 − 측정) x ki. 0 = 끔 (펌웨어 I 0 그대로)")
parser.add_argument("--bias_max", type=float, default=0.15, help="[rad] 처짐 보정 한도")
parser.add_argument("--gui", action="store_true")
parser.add_argument("--tilt_speed", type=float, default=0.8, help="[rad/s] 집게를 45° 기울여 놓는 칸 (궤적 tilt_deg) 의 쥔 구간 속도. 2026-11-03 12칸 45° 부터 0.8 + --row_speed 3:0.4 "
                    "(0.8 로 모두 돌리면 3열 상자 퍽 4 대가 칸 벽에 걸림, 4칸만 기울이던 때 기본 0.4)")
parser.add_argument("--settle_ki", action=argparse.BooleanOptionalAction, default=True,
                    help="퍽을 쥐기 전에 처짐 보정 (ki) 을 1.5 s 먼저 돌려 집게 자세를 맞춤 (실제로도 집을 때 이미 보정 중). 끄면 쥔 뒤 보정이 집게를 2 ~ 3° 돌려 기울인 칸 퍽이 기욺")
parser.add_argument("--grip_close", type=float, default=-0.0075, help="쥐는 손가락 목표. 팀 값 FINGER_CLOSE −0.011 이면 한쪽 손가락만 힘 한계 (13 N) 에 걸려 "
                    "퍽이 닫는 축으로 3 ~ 4 mm 치우침 -> −0.0075 (퍽에서 2 mm 더, 약 12 N) 면 두 손가락 힘이 같아 가운데 (2026-11-02)")
parser.add_argument("--debug", action="store_true", help="0 번 대의 쥐기 전후 값을 자세히 출력하고 끝냄")
parser.add_argument("--box_mm", type=float, default=None, help="[mm] 상자 퍽 한 변 바꿔 시험 (기본 30)")
parser.add_argument("--roll_pi", default="", help="손목 롤 (joint5) 180° 로 놓을 칸 '열:칸,...' (예 2:0,2:3,3:0,3:3). joint5 한계를 ±3.3 rad 로 늘려 시험 (팀 한계 ±2.8)")
parser.add_argument("--puck_mu", type=float, default=0.5, help="퍽 마찰 (정지 = 미끄럼, 결합 min = 퍽-팔레트 마찰). 기본 0.5 = PLA 끼리 어림 (실측 전). "
                    "팀 값 1.0 이면 틈 1 mm 칸에서 퍽이 조금 기울며 끼임 (떨어뜨리기 8/24 -> 0.5 에서 23/24, 2026-11-02)")
parser.add_argument("--contact_offset", type=float, default=0.002, help="[m] 로봇 · 퍽 접촉 거리")
parser.add_argument("--drop", action="store_true", help="팔 없이 퍽만: 칸 축 · 칸 방향에 맞춰 퍽 바닥 = 칸 벽 윗면 + 5 mm 에서 놓아 앉는지 (충돌 형상 확인)")
parser.add_argument("--row_speed", default="3:0.4", help="[rad/s] 열별 쥔 구간 속도 '열:속도,...' (예 3:0.4). 그 열은 --speed · --tilt_speed 대신 (2026-11-03: 12칸 45° 에서 "
                    "0.8 이면 3열만 실패 20/24)")
parser.add_argument("--empty_speed", type=float, default=0.8, help="[rad/s] 퍽을 쥐지 않은 구간 (놓은 뒤 retract · return · rotate_back, 빼기 전 다가가기) 속도. 0 이면 쥔 구간과 같음")
parser.add_argument("--unload", action="store_true", help="빼기 확인: 칸에 앉힌 퍽을 궤적 거꾸로 꺼내 봄 (기본 결과 pallet_unload_check.json)")
parser.add_argument("--out", default=None, help="결과 JSON (기본 pallet_place_check.json, --unload 면 pallet_unload_check.json)")
parser.add_argument("--hand_offset", default="", help="[mm] 쥔 퍽을 기대 자리에서 일부러 옮겨 쥐어 줌 '집게축:옆,...' (집게 x · z, 예 0:0,3:0,-3:0,0:3). "
                    "묶음마다 칸 12 x 원기둥 · 상자 = 24 대 (--num_envs 무시). 손 안 어긋남 허용치 측정 (2026-11-05, 이어 평가 넘겨받을 때 중앙값 5 ~ 6 mm)")
AppLauncher.add_app_launcher_args(parser)
parser.add_argument("--neighbors", default="none", choices=["none", "slot", "opposite", "pairs", "behind"],
                    help="이웃 칸 퍽 (막는 쌍 시험, 2026-11-13 · 계획기 slots.BLOCKERS 근거): slot = 같은 열 · 뒤 열 모두 (칸 모양대로), opposite = 같은 곳 (모양 반대), pairs = 같은 열 바로 옆 칸 하나씩 (칸 모양대로), behind = 뒤 열만, none = 비움")
parser.add_argument("--place", action="store_true",
                    help="꺼낸 뒤 장소 상자 칸에 놓기 (ROS placeAt 과 같은 경로, 2026-11-16) -> dice_place_eval.json. --unload 를 켬. "
                         "대 i = (칸 · 모양 20 가지) x 상자 칸 9 곳 순서 (180 대면 한 번씩)")
parser.add_argument("--box_gap", default="0.05,0.06", help="[m] 차체 앞 끝 -> 상자 앞면 거리 범위 (균일, #225 가까이 붙기 5 ~ 6 cm)")
parser.add_argument("--box_lat", type=float, default=0.02, help="[m] 상자 중심 좌우 어긋남 ± (균일)")
parser.add_argument("--box_yaw", type=float, default=2.0, help="[deg] 상자 요 ± (균일, #225 다가간 뒤 방향 ±2°)")
parser.add_argument("--box_err", type=float, default=0.0, help="[mm] 상자 측정 오차 σ (정규, 중심 x · y 각각). 칸 좌표는 잰 상자로 계산")
parser.add_argument("--box_err_deg", type=float, default=0.0, help="[deg] 상자 요 측정 오차 σ (정규)")
parser.add_argument("--place_comp", action=argparse.BooleanOptionalAction, default=True,
                    help="ROS place 보정 (radial_offset · slope · lateral · source_offset_mm, control manipulator.yaml) 적용. 끄면 칸 중심 그대로")
parser.add_argument("--box_neighbors", default="none", choices=["none", "far", "all"],
                    help="상자 다른 칸에 원기둥 퍽을 미리 세워 둠: far = J1 에서 목표보다 먼 칸 모두 (계획기 place_order 가 먼 칸부터 놓으므로 실제 최악),"
                         " all = 다른 8 칸 모두 (순서를 안 지킬 때), none = 비움")
parser.add_argument("--seed", type=int, default=0, help="상자 자리 · 측정 오차 난수")
args_cli = parser.parse_args()
if args_cli.place:
    args_cli.unload = True
args_cli.headless = not args_cli.gui
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import torch  # noqa: E402
from isaaclab_physx.physics import PhysxCfg  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.assets import Articulation, ArticulationCfg, AssetBaseCfg, RigidObjectCfg  # noqa: E402
from isaaclab.envs import DirectRLEnv, DirectRLEnvCfg, ViewerCfg  # noqa: E402
from isaaclab.scene import InteractiveSceneCfg  # noqa: E402
from isaaclab.sensors import ContactSensorCfg  # noqa: E402
from isaaclab.sim import SimulationCfg  # noqa: E402
from isaaclab.utils import configclass  # noqa: E402
from isaaclab.utils.math import quat_apply, quat_apply_inverse, quat_from_matrix  # noqa: E402

from pallet_timeline import PHASE_OF, PHASES, SEGS, Speeds, parse_row_speed  # noqa: E402  (chain_pallet_eval.py 와 같은 궤적)
from pallet_timeline import seg_speed as _seg_speed, timeline as _timeline  # noqa: E402
from waffle_isaac.dice2_cfg import BOX_H, BOX_SIZE, PUCK_H, box_cfg, puck_cfg  # noqa: E402
from waffle_isaac.robot_cfg import (  # noqa: E402
    ARM_JOINTS, ARM_LINKS_REGEX, BODY_LINK, EE_BODY, FINGER_CLOSE, FINGER_LINKS_REGEX, FINGER_RELEASED,
    FLOOR_MATERIAL, GRIPPER_JOINT, GRIPPER_MIMIC, WAFFLE_CFG, WHEEL_REGEX,
)

TRAJ = json.load(open(args_cli.traj or os.path.join(HERE, "pallet_place_traj.json"), encoding="utf-8"))
Q_FRONT = TRAJ["transit"]["q_front"]
BODY = "{ENV_REGEX_NS}/Robot/" + BODY_LINK
CENTER_EE = TRAJ["tcp_offset_ee"][0] + TRAJ["grasp_dz"]  # 쥔 퍽 중심 = gripper_roll_link x 0.155 (TCP 아래 15 mm)
OPEN_PLACE = 0.010  # 자리 잡을 때 벌림 61.6 mm
FINGER_AT_PUCK = -0.0056  # 퍽을 둘 때 손가락 관절을 바로 이 값 (30.4 mm, 퍽 30 mm 와 0.2 mm 틈) 으로 써 넣음: 닫는 데 0.4 s 걸려 그동안 퍽이 떨어짐
if args_cli.grip_close is not None:
    FINGER_CLOSE = args_cli.grip_close

_robot = WAFFLE_CFG.replace(
    prim_path="{ENV_REGEX_NS}/Robot",
    spawn=WAFFLE_CFG.spawn.replace(
        activate_contact_sensors=True,
        collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=args_cli.contact_offset, rest_offset=0.0),
        articulation_props=WAFFLE_CFG.spawn.articulation_props.replace(enabled_self_collisions=True),
    ),
    init_state=WAFFLE_CFG.init_state.replace(joint_pos={**WAFFLE_CFG.init_state.joint_pos, **dict(zip(ARM_JOINTS, Q_FRONT)),
                                                        GRIPPER_JOINT: OPEN_PLACE, GRIPPER_MIMIC: OPEN_PLACE}),
)


def _puck(name, shape, rgb, xy):
    p = puck_cfg(name, shape, rgb, xy)
    sp = p.spawn.replace(activate_contact_sensors=True,
                         collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=args_cli.contact_offset, rest_offset=0.0))
    if args_cli.puck_mu is not None:
        sp = sp.replace(physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=args_cli.puck_mu, dynamic_friction=args_cli.puck_mu,
                                                                        friction_combine_mode="min"))
    if shape == "box" and args_cli.box_mm:
        sp = sp.replace(size=(args_cli.box_mm / 1000, args_cli.box_mm / 1000, sp.size[2]))
    return p.replace(spawn=sp)


def _touch(prim):
    return ContactSensorCfg(prim_path=prim, filter_prim_paths_expr=[BODY], update_period=0.0, history_length=1)


@configclass
class SceneCfg(InteractiveSceneCfg):
    ground = AssetBaseCfg(prim_path="/World/ground", spawn=sim_utils.GroundPlaneCfg(physics_material=FLOOR_MATERIAL))
    robot: ArticulationCfg = _robot
    puckC: RigidObjectCfg = _puck("C", "cyl", (0.9, 0.1, 0.1), (0.0, 0.6))
    puckB: RigidObjectCfg = _puck("B", "box", (0.1, 0.3, 0.9), (0.0, 0.7))
    arm_body = _touch("{ENV_REGEX_NS}/Robot/" + ARM_LINKS_REGEX)
    finger_body = _touch("{ENV_REGEX_NS}/Robot/" + FINGER_LINKS_REGEX)
    puckC_body = _touch("{ENV_REGEX_NS}/PuckC")
    puckB_body = _touch("{ENV_REGEX_NS}/PuckB")
    # 이웃 칸 퍽 (--neighbors): 칸 k 마다 하나 (모양 = --neighbors), 쓰지 않는 대는 멀리 둠
    for _k, _s in enumerate(TRAJ["slots"]):
        _sh = "cyl" if _s["shape"] == "cylinder" else "box"
        if args_cli.neighbors == "opposite": _sh = "box" if _sh == "cyl" else "cyl"   # pairs · behind 는 칸 모양대로
        locals()[f"nb{_k}"] = _puck(f"N{_k}", _sh, (0.6, 0.6, 0.6), (-0.6 + 0.12 * _k, 0.95))
    del _k, _s, _sh
    if args_cli.place:   # 장소 상자 (자리는 실행 중 정함) · 다른 칸 퍽 (--box_neighbors)
        boxL = box_cfg("L", (0.0, -0.8))
        if args_cli.box_neighbors != "none":
            for _k in range(8):
                locals()[f"bn{_k}"] = _puck(f"BN{_k}", "cyl", (0.2, 0.7, 0.2), (0.6 + 0.07 * _k, -0.95))
            del _k
    light = AssetBaseCfg(prim_path="/World/Light", spawn=sim_utils.DomeLightCfg(intensity=2000.0, color=(0.85, 0.85, 0.85)))


@configclass
class PalletCheckEnvCfg(DirectRLEnvCfg):
    decimation = 1
    episode_length_s = 1000.0
    action_space = 1
    observation_space = 1
    state_space = 0
    viewer: ViewerCfg = ViewerCfg(eye=(0.25, -0.55, 0.55), lookat=(-0.02, 0.0, 0.15), origin_type="env", env_index=0)
    sim: SimulationCfg = SimulationCfg(dt=1 / 120, render_interval=4, physics=PhysxCfg(), physics_material=FLOOR_MATERIAL)
    scene: SceneCfg = SceneCfg(num_envs=24, env_spacing=1.5, replicate_physics=True, clone_in_fabric=True)


class PalletCheckEnv(DirectRLEnv):
    """관절 목표 (arm_target) · 손가락 목표 (grip) 를 밖에서 정해 주는 껍데기 환경 (관측 · 보상 없음)."""

    cfg: PalletCheckEnvCfg

    def __init__(self, cfg, render_mode=None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        r = self.robot
        self._arm, _ = r.find_joints(ARM_JOINTS, preserve_order=True)
        self._gl, _ = r.find_joints([GRIPPER_JOINT])
        self._gr, _ = r.find_joints([GRIPPER_MIMIC])
        self._wheels, _ = r.find_joints([WHEEL_REGEX])
        self._ee = r.find_bodies(EE_BODY)[0][0]
        self.arm_target = r.data.default_joint_pos.torch[:, self._arm].clone()
        self.grip = torch.full((self.num_envs, 1), OPEN_PLACE, device=self.device)

    @property
    def robot(self) -> Articulation:
        return self.scene["robot"]

    def _pre_physics_step(self, actions):
        pass

    def _apply_action(self):
        cmd = self.robot.actuators.target_command
        cmd.set_position_index(value=self.arm_target.contiguous(), joint_ids=self._arm)
        cmd.set_position_index(value=self.grip.expand(-1, 2).contiguous(), joint_ids=self._gl + self._gr)
        cmd.set_velocity_index(value=torch.zeros(self.num_envs, len(self._wheels), device=self.device), joint_ids=self._wheels)

    def _get_observations(self):
        return {"policy": torch.zeros(self.num_envs, 1, device=self.device)}

    def _get_rewards(self):
        return torch.zeros(self.num_envs, device=self.device)

    def _get_dones(self):
        z = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        return z, z

    def force(self, k):
        f = self.scene[k].data.normal_force_matrix_w.torch  # (N, 몸, 상대, 3)
        return torch.linalg.norm(f, dim=-1).amax(dim=(1, 2))

    def force_body(self, k):
        """가장 세게 닿은 몸 이름 (대마다)."""
        f = torch.linalg.norm(self.scene[k].data.normal_force_matrix_w.torch, dim=-1).amax(dim=2)  # (N, 몸)
        names = self.scene[k].body_names
        return [names[i] for i in f.argmax(dim=1).tolist()]


ROLL_PI = {tuple(int(v) for v in x.split(":")) for x in args_cli.roll_pi.split(",") if x}
ROW_SPEED = parse_row_speed(args_cli.row_speed)
HAND_OFFSETS = [tuple(float(v) for v in x.split(":")) for x in args_cli.hand_offset.split(",") if x]
SPEEDS = Speeds(args_cli.speed, args_cli.tilt_speed, ROW_SPEED, args_cli.empty_speed)
OUT = args_cli.out or os.path.join(HERE, "dice_place_eval.json" if args_cli.place else "pallet_unload_check.json" if args_cli.unload else "pallet_place_check.json")


def seg_speed(slot, held):
    """[rad/s] 쥔 구간 = --row_speed (그 열) > --tilt_speed (기울인 칸) > --speed, 빈 집게 구간 = --empty_speed (있으면). 식은 pallet_timeline."""
    return _seg_speed(slot, held, SPEEDS)


def timeline(slot, dt):
    """칸 궤적 -> 물리 스텝마다 (팔 목표 5, 손가락 hold/open, 단계, 구간) 목록 (pallet_timeline.timeline, --roll_pi 칸은 손목 롤 180°)."""
    return _timeline(slot, dt, SPEEDS, Q_FRONT, roll_pi=(slot["row"], slot["index"]) in ROLL_PI)


UNLOAD_PHASES = ["-", "approach", "grip", "lift", "carry"]


def timeline_unload(slot, dt):
    """빼기: 놓기 궤적 거꾸로 -> 물리 스텝마다 (팔 목표 5, 손가락 0 쥠 · 1 벌림, 단계 UNLOAD_PHASES, 놓기 구간 번호 SEGS).
    빈 집게로 rotate_back · return · retract 를 거꾸로 따라 놓는 자세 (release) 까지 -> 0.3 s 멈춤 -> 쥠 0.6 s
    -> place · carry · rotate 거꾸로 (쥔 채) -> 앞 운반 자세 -> 1 s 기다림."""
    S = {sg["name"]: [list(q) for q in sg["q"]] for sg in slot["segments"]}
    q_front = list(slot.get("q_front", Q_FRONT))
    out, prev = [], q_front

    def go(name, qs, g, ph):
        nonlocal prev
        speed = seg_speed(slot, g == 0)
        for q in qs:
            n = max(1, int(math.ceil(max(abs(a - b) for a, b in zip(q, prev)) / (speed * dt))))
            for k in range(1, n + 1):
                out.append(([p + (a - p) * k / n for a, p in zip(q, prev)], g, ph, SEGS.index(name)))
            prev = q

    def wait(sec, g, ph, name):
        out.extend([(list(prev), g, ph, SEGS.index(name))] * int(sec / dt))

    for name in ("rotate_back", "return", "retract"):
        go(name, S[name][::-1], 1, 1)
    go("release", S["release"], 1, 1)
    wait(0.3, 1, 1, "release")
    wait(0.6, 0, 2, "release")
    go("place", S["place"][::-1], 0, 3)
    go("carry", S["carry"][::-1], 0, 4)
    go("rotate", S["rotate"][::-1] + [q_front], 0, 4)
    wait(1.0, 0, 4, "rotate")
    return out


NB_OF = None


def place_neighbors(u, case, ids):
    """--neighbors: 대마다 목표 칸과 같은 열 · 뒤 열 칸에 이웃 퍽을 앉힘 (칸 축 · 칸 방향, 바닥 + 0.5 mm)."""
    if args_cli.neighbors == "none":
        return
    dev, n = u.device, u.num_envs
    rp, rq = u.robot.data.root_link_pos_w.torch, u.robot.data.root_link_quat_w.torch
    for k, s in enumerate(TRAJ["slots"]):
        o = u.scene[f"nb{k}"]
        if args_cli.neighbors == "pairs":
            use = torch.tensor([s is NB_OF[e] for e in range(n)], device=dev)
        elif args_cli.neighbors == "behind":
            use = torch.tensor([s["row"] > c[0]["row"] for c in case], device=dev)
        else:
            use = torch.tensor([c[0] is not s and s["row"] >= c[0]["row"] for c in case], device=dev)   # 열 규칙: 목표 칸과 같은 열 · 뒤 열만 (앞 열은 비어 있어야 함)
        loc = torch.tensor([[s["x"], s["y"], s["floor_z"] + 0.0005 + PUCK_H / 2]], device=dev).expand(n, 3)
        th = math.radians(s["theta_deg"])
        q = torch.tensor([[0, 0, math.sin(th / 2), math.cos(th / 2)]], device=dev).expand(n, 4)
        pose = torch.cat((rp + quat_apply(rq, loc), q), dim=1)
        cur = torch.cat((o.data.root_pos_w.torch, o.data.root_quat_w.torch), dim=1)
        o.write_root_pose_to_sim_index(root_pose=torch.where(use[:, None], pose, cur), env_ids=ids)
        o.write_root_velocity_to_sim_index(root_velocity=torch.zeros(n, 6, device=dev), env_ids=ids)


def unload_test(u, case, is_box, ids, q_start, step, puck_pose, ee_frame, exp_center, puck_up):
    """빼기 확인: 칸에 퍽을 앉힘 (칸 축 · 칸 방향, 바닥 + 0.5 mm) -> 앞 운반 자세에서 자리 잡기 -> timeline_unload.
    성공 = 끝에 쥔 퍽 중심 - 손가락 가운데 < 5 mm, 퍽 축 - 기대 축 < 5°, 퍽 바닥 가운데가 칸 축에서 수평 40 mm 넘게 벗어남 (칸 밖)."""
    dev, n, dt = u.device, u.num_envs, u.physics_dt
    rp, rq = u.robot.data.root_link_pos_w.torch, u.robot.data.root_link_quat_w.torch
    loc = torch.tensor([[s["x"], s["y"], s["floor_z"] + 0.0005 + PUCK_H / 2] for s, _ in case], device=dev)
    th = torch.tensor([math.radians(s["theta_deg"]) for s, _ in case], device=dev)
    q_loc = torch.stack((torch.zeros_like(th), torch.zeros_like(th), torch.sin(th / 2), torch.cos(th / 2)), dim=1)  # (x,y,z,w), 루트 요 0 가정
    pose = torch.cat((rp + quat_apply(rq, loc), q_loc), dim=1)
    for k, m in (("puckC", ~is_box), ("puckB", is_box)):
        o = u.scene[k]
        cur = torch.cat((o.data.root_pos_w.torch, o.data.root_quat_w.torch), dim=1)
        o.write_root_pose_to_sim_index(root_pose=torch.where(m[:, None], pose, cur), env_ids=ids)
        o.write_root_velocity_to_sim_index(root_velocity=torch.zeros(n, 6, device=dev), env_ids=ids)
    place_neighbors(u, case, ids)
    box = setup_box(u, case, ids) if args_cli.place else None
    u.grip[:] = FINGER_RELEASED
    bias = torch.zeros(n, 5, device=dev)
    for _ in range(int(1.5 / dt)):  # 앞 운반 자세 자리 잡기 (처짐 보정) + 퍽이 칸에 앉음
        if args_cli.ki > 0:
            bias = torch.clamp(bias + args_cli.ki * dt * (q_start - u.robot.data.joint_pos.torch[:, u._arm]), -args_cli.bias_max, args_cli.bias_max)
            u.arm_target[:] = q_start + bias
        step()
    if os.environ.get("NB_DEBUG"):
        for k, s in enumerate(TRAJ["slots"]):
            pk = quat_apply_inverse(rq, u.scene[f"nb{k}"].data.root_pos_w.torch - rp)
            print(f"[NbDbg] 칸 {s['row']}:{s['index']} 기대 ({s['x']:.4f},{s['y']:.4f},{s['floor_z'] + PUCK_H / 2:.4f}) 대0 {[round(v, 4) for v in pk[0].tolist()]} 대1 {[round(v, 4) for v in pk[1].tolist()]}", flush=True)
        pt = quat_apply_inverse(rq, torch.where(is_box[:, None], u.scene["puckB"].data.root_pos_w.torch, u.scene["puckC"].data.root_pos_w.torch) - rp)
        print(f"[NbDbg] 목표 퍽 대0 {[round(v, 4) for v in pt[0].tolist()]} 칸 {case[0][0]['row']}:{case[0][0]['index']}", flush=True)
    tl = [timeline_unload(c[0], dt) for c in case]
    T = max(len(x) for x in tl)
    tgt = torch.tensor([[x[min(t, len(x) - 1)][0] for t in range(T)] for x in tl], device=dev)
    grp = torch.tensor([[x[min(t, len(x) - 1)][1] for t in range(T)] for x in tl], device=dev)
    phs = torch.tensor([[x[min(t, len(x) - 1)][2] for t in range(T)] for x in tl], device=dev)
    fmax = {k: torch.zeros(n, len(UNLOAD_PHASES), device=dev) for k in ("arm", "finger", "puck")}
    slip = torch.zeros(n, device=dev)
    print(f"[PalletUnload] {n} 대, 궤적 {T} 스텝 ({T * dt:.1f} s)", flush=True)
    NBV = torch.zeros(n, device=dev)
    for t in range(T):
        if args_cli.ki > 0:
            bias = torch.clamp(bias + args_cli.ki * dt * (tgt[:, t] - u.robot.data.joint_pos.torch[:, u._arm]), -args_cli.bias_max, args_cli.bias_max)
        u.arm_target[:] = tgt[:, t] + bias
        u.grip[:, 0] = torch.where(grp[:, t] > 0, torch.tensor(FINGER_RELEASED, device=dev), torch.tensor(FINGER_CLOSE, device=dev))
        step()
        ph = phs[:, t]
        fp = torch.where(is_box, u.force("puckB_body"), u.force("puckC_body"))
        for k, f in (("arm", u.force("arm_body")), ("finger", u.force("finger_body")), ("puck", fp)):
            fmax[k].scatter_reduce_(1, ph[:, None], f[:, None], reduce="amax")
        held = ph >= 3
        if held.any():
            pe, ex, _, _ = ee_frame()
            pc, _ = puck_pose()
            slip = torch.where(held, torch.maximum(slip, torch.linalg.norm(pc - exp_center(pe, ex), dim=1)), slip)
        if args_cli.neighbors != "none":
            nbv = torch.stack([torch.linalg.norm(u.scene[f"nb{k}"].data.root_lin_vel_w.torch, dim=1) for k in range(len(TRAJ["slots"]))], 1).amax(1)
            NBV[:] = torch.maximum(NBV, nbv)
        if os.environ.get("NB_TRACE") and t % 30 == 0:
            pc_, _ = puck_pose(); pe_, ex_, _, _ = ee_frame(); ec_ = exp_center(pe_, ex_)
            for e in (2, 4):
                lp = quat_apply_inverse(rq[e:e+1], pc_[e:e+1] - rp[e:e+1])[0]
                nbf = max(float(torch.linalg.norm(u.scene[f"nb{k}"].data.root_lin_vel_w.torch[e])) for k in range(len(TRAJ["slots"])))
                print(f"[NbTr] t={t * dt:5.2f} 대{e} 단계 {UNLOAD_PHASES[int(ph[e])]} 퍽 {[round(v, 4) for v in lp.tolist()]} 손 어긋남 {float(torch.linalg.norm(pc_[e] - ec_[e])) * 1000:6.1f} mm 이웃 최대 속도 {nbf:.3f}", flush=True)
        if t % 240 == 0:
            print(f"[PalletUnload] t={t * dt:5.1f} s 단계 {[UNLOAD_PHASES[int(v)] for v in ph[:min(n, 12)].tolist()]}", flush=True)
    pe, ex, _, _ = ee_frame()
    pc, qc = puck_pose()
    off = torch.linalg.norm(pc - exp_center(pe, ex), dim=1)
    ax = quat_apply(qc, torch.tensor([[0, 0, 1.0]], device=dev).expand(n, 3))
    ang = torch.rad2deg(torch.acos(torch.clamp((ax * puck_up(ex)).sum(1).abs(), max=1.0)))
    bot = quat_apply_inverse(rq, pc - ax * (PUCK_H / 2) - rp)
    res = []
    for i, (s, shape) in enumerate(case):
        away = math.hypot(float(bot[i, 0]) - s["x"], float(bot[i, 1]) - s["y"])
        ok = float(off[i]) < 0.005 and float(ang[i]) < 5.0 and away > 0.040
        r = dict(slot=f"{s['row']}열 {s['index']}", puck_shape=shape, held=ok, off_mm=round(float(off[i]) * 1000, 1), puck_tilt_vs_grip_deg=round(float(ang[i]), 1),
                 away_from_slot_mm=round(away * 1000, 1), max_slip_mm=round(float(slip[i]) * 1000, 1), duration_s=round(len(tl[i]) * dt, 1), nb_max_speed=round(float(NBV[i]), 4), nb_slot=(f"{NB_OF[i]['row']}:{NB_OF[i]['index']}" if NB_OF else None),
                 max_force_N={k: {UNLOAD_PHASES[j]: round(float(fmax[k][i, j]), 1) for j in range(1, len(UNLOAD_PHASES))} for k in fmax})
        res.append(r)
        f = r["max_force_N"]
        print(f"[PalletUnload] {r['slot']} {shape:3s} {'쥠' if ok else '실패'} | 퍽-손가락 가운데 {r['off_mm']:5.1f} mm 기울기 {r['puck_tilt_vs_grip_deg']:5.1f}° 칸 축에서 {r['away_from_slot_mm']:6.1f} mm"
              f" | 미끄럼 {r['max_slip_mm']:4.1f} mm | {r['duration_s']} s | 손가락-몸체 approach {f['finger']['approach']} grip {f['finger']['grip']} lift {f['finger']['lift']}"
              f" | 퍽-몸체 lift {f['puck']['lift']}", flush=True)
    summary = dict(held=sum(r["held"] for r in res), total=n, speed_rad_s=args_cli.speed, tilt_speed=args_cli.tilt_speed, row_speed=ROW_SPEED, empty_speed=args_cli.empty_speed,
                   ki=args_cli.ki, puck_mu=args_cli.puck_mu, grip_close=FINGER_CLOSE, tilt_slots=sorted({(c[0]["row"], c[0]["index"]) for c in case if c[0].get("tilt_deg")}),
                   duration_s_per_slot={r["slot"]: r["duration_s"] for r in res[:len(TRAJ["slots"])]},
                   arm_body_max_N=round(float(fmax["arm"].max()), 1), finger_body_max_N=round(float(fmax["finger"].max()), 1))
    print(f"[PalletUnload] 요약 {summary}", flush=True)
    if args_cli.place:
        place_test(u, case, is_box, ids, step, puck_pose, bias, tgt[:, -1].clone(), res, box)
        return
    json.dump(dict(summary=summary, results=res), open(OUT, "w", encoding="utf-8"), ensure_ascii=False, indent=1)


# ---------------------------------------------------------------- 장소 상자에 놓기 (--place, 2026-11-16)
# ROS manipulator_node placeAt + runPlaceStream 과 같은 경로 · 시간표:
#   칸 (잰 상자 칸 중심, 윗면 z) -> 보정 (J1 바깥 a + b (r − 0.2665)/0.06 mm · 꺼낸 칸별 · 옆) -> tip = 칸 + 퍽 반 높이 + grasp_z_offset + clearance
#   집게 45° (place.pitch), pre = back = tip − 집게 축 x 0.04 -> [꺼내기 끝 자세 + bias 이어 씀] -> pre (0.8 rad/s, 처음 최소 1 s)
#   -> tip 직선 (1 cm 마다 max(0.2 s, dq / 0.8)) -> 벌림 1.0 s (release 0.0798 rad = 36.4 mm) -> 머묾 0.3 s -> back 직선 -> 이동 자세 q_front_tilt
#   처짐 보정 = 꺼내기와 같은 적분 (ki · bias_max). 결과: 퍽 바닥 가운데 - 참 칸 중심 (수평), 서 있음, 상자 위, 옆 퍽 밀림
FRONT_X = 0.1328          # 차체 앞 끝 (mission_planner poses.FRONT)
TOOL_X = 0.139            # gripper_roll_link -> tool_link (ROS 집게 끝)
GRASP_Z, CLEAR, APPROACH = 0.015, 0.001, 0.04   # pick.grasp_z_offset · place.clearance · place.approach_distance = retreat_distance
PLACE_PITCH = math.radians(45.0)
RELEASE_PLACE = (36.4 - 41.6) / 2000.0   # place.release_position 0.0798 rad = 36.4 mm -> 손가락 m (sim_skill_server 와 같은 식)
CELL = 0.06


def _place_params():
    """control manipulator.yaml place 보정값 (radial a · b, lateral, 꺼낸 칸별)."""
    import yaml
    f = os.path.join(HERE, "..", "..", "..", "ros2_ws", "src", "control", "config", "manipulator.yaml")
    d = yaml.safe_load(open(f, encoding="utf-8"))
    pl = next(iter(d.values()))["ros__parameters"]["place"]
    so = {}
    for e in pl.get("source_offset_mm", []):
        k, a, b = e.split(":")
        so[k] = (float(a), float(b))
    return dict(a=pl["radial_offset_mm"], b=pl["radial_offset_slope_mm"], lat=pl["lateral_offset_mm"], src=so, file=os.path.normpath(f))


def setup_box(u, case, ids):
    """대마다 상자 참 자리 (차체 앞 끝 -> 앞면 box_gap, 좌우 box_lat, 요 box_yaw) · 잰 자리 (+ 오차) · 놓을 칸. 상자 · 옆 퍽을 장면에 둠."""
    dev, n = u.device, u.num_envs
    g = torch.Generator(device="cpu").manual_seed(args_cli.seed)
    U = lambda lo, hi: lo + (hi - lo) * torch.rand(n, generator=g)
    glo, ghi = (float(v) for v in args_cli.box_gap.split(","))
    yaw = torch.deg2rad(U(-args_cli.box_yaw, args_cli.box_yaw))
    cx, cy = FRONT_X + U(glo, ghi) + BOX_SIZE / 2, U(-args_cli.box_lat, args_cli.box_lat)
    mx = cx + torch.randn(n, generator=g) * args_cli.box_err / 1000.0
    my = cy + torch.randn(n, generator=g) * args_cli.box_err / 1000.0
    myaw = yaw + torch.deg2rad(torch.randn(n, generator=g) * args_cli.box_err_deg)
    combos = 2 * len(TRAJ["slots"])
    cells = [((i // combos) % 9) // 3 - 1 for i in range(n)], [((i // combos) % 9) % 3 - 1 for i in range(n)]
    dxs, dys = torch.tensor(cells[0], dtype=torch.float32), torch.tensor(cells[1], dtype=torch.float32)

    def cell_xy(x, y, th, dx, dy):
        return x + CELL * (dx * torch.cos(th) - dy * torch.sin(th)), y + CELL * (dx * torch.sin(th) + dy * torch.cos(th))

    tx, ty = cell_xy(cx, cy, yaw, dxs, dys)       # 참 칸 중심 (base_link)
    px, py = cell_xy(mx, my, myaw, dxs, dys)      # 잰 칸 중심 -> 놓을 점
    rp, rq = u.robot.data.root_link_pos_w.torch, u.robot.data.root_link_quat_w.torch
    to_w = lambda x, y: rp + quat_apply(rq, torch.stack((x, y, torch.zeros_like(x)), 1).to(dev))
    bw = to_w(cx, cy)
    bw[:, 2] = BOX_H / 2
    ryaw = 2 * torch.atan2(rq[:, 2], rq[:, 3]).cpu()   # 차체 요 (평지)
    hq = (yaw + ryaw) / 2
    bq = torch.stack((torch.zeros(n), torch.zeros(n), torch.sin(hq), torch.cos(hq)), 1).to(dev)
    o = u.scene["boxL"]
    o.write_root_pose_to_sim_index(root_pose=torch.cat((bw, bq), 1), env_ids=ids)
    o.write_root_velocity_to_sim_index(root_velocity=torch.zeros(n, 6, device=dev), env_ids=ids)
    top_b = BOX_H - rp[:, 2]   # 상자 윗면 높이 (base_link, 평지)
    nb0 = None
    if args_cli.box_neighbors != "none":   # 다른 칸에 원기둥 (far = 목표보다 J1 에서 먼 칸만, 쓰지 않는 퍽은 상자 밖 바닥)
        nb0 = []
        J1 = (0.096, 0.0)

        def jd(i, a, b):
            x_, y_ = cell_xy(cx[i:i + 1], cy[i:i + 1], yaw[i:i + 1], torch.tensor([float(a)]), torch.tensor([float(b)]))
            return math.hypot(float(x_) - J1[0], float(y_) - J1[1])
        use = []
        for k in range(8):
            ox, oy, us = [], [], []
            for i in range(n):
                others = [(a, b) for a in (-1, 0, 1) for b in (-1, 0, 1) if (a, b) != (cells[0][i], cells[1][i])]
                a, b = others[k]
                ox.append(a); oy.append(b)
                us.append(args_cli.box_neighbors == "all" or jd(i, a, b) > jd(i, cells[0][i], cells[1][i]) + 1e-6)
            qx, qy = cell_xy(cx, cy, yaw, torch.tensor(ox, dtype=torch.float32), torch.tensor(oy, dtype=torch.float32))
            w = to_w(qx, qy)
            w[:, 2] = BOX_H + PUCK_H / 2 + 0.0005
            um = torch.tensor(us, device=dev)
            away = to_w(torch.full((n,), -0.6), torch.full((n,), -0.5 + 0.07 * k))
            away[:, 2] = PUCK_H / 2 + 0.0005
            w = torch.where(um[:, None], w, away)
            use.append(um)
            ob = u.scene[f"bn{k}"]
            ob.write_root_pose_to_sim_index(root_pose=torch.cat((w, torch.tensor([[0, 0, 0, 1.0]], device=dev).expand(n, 4)), 1), env_ids=ids)
            ob.write_root_velocity_to_sim_index(root_velocity=torch.zeros(n, 6, device=dev), env_ids=ids)
            nb0.append(w.clone())
        box_nb_count = torch.stack(use, 1).sum(1).cpu()
    return dict(nb_count=(box_nb_count if args_cli.box_neighbors != "none" else torch.zeros(n)), true=torch.stack((tx, ty), 1), aim=torch.stack((px, py), 1), top=top_b.cpu(), dx=cells[0], dy=cells[1], nb0=nb0,
                gap=(cx - BOX_SIZE / 2 - FRONT_X), lat=cy, yaw_deg=torch.rad2deg(yaw))


_ARM = None


def ik45(p, yaw, q0):
    """ROS solveArm (pitch 45° 아래 · 바깥, 손목 롤 0, 집게 끝 tool_link) -> (q 5, 위치 오차 m). 팀 URDF (pallet_traj.Arm)."""
    global _ARM
    import numpy as np
    from scipy.optimize import least_squares
    if _ARM is None:
        import pallet_traj
        _ARM = pallet_traj.Arm(pallet_traj.URDF)
    A = _ARM
    a_dir = np.array([math.cos(PLACE_PITCH) * math.cos(yaw), math.cos(PLACE_PITCH) * math.sin(yaw), -math.sin(PLACE_PITCH)])

    def res(x):
        t, R = A.tcp([yaw, x[0], x[1], x[2], 0.0], TOOL_X)
        return np.r_[(t - p) * 1000.0, (R[:, 0] - a_dir) * 100.0]
    starts = [q0] if q0 is not None else [[a, b, c] for a in (-1.2, -0.8, -0.4, 0.0, 0.4) for b in (-1.2, -0.8, 0.0, 0.8) for c in (0.7, 1.2, 1.6, 1.95)]
    best = None
    for x0 in starts:
        x0 = np.clip(x0, A.lo[1:4] + 1e-4, A.hi[1:4] - 1e-4)
        s_ = least_squares(res, x0, bounds=(A.lo[1:4], A.hi[1:4]), xtol=1e-12, ftol=1e-12)
        r = res(s_.x)
        e = float(np.linalg.norm(r[:3])) / 1000.0 + float(np.linalg.norm(r[3:])) / 100.0 * 0.05
        if best is None or e < best[1]:
            best = ([yaw, *s_.x, 0.0], e)
        if q0 is not None and e < 2e-4:
            break
    return best


def place_plan(slot, q_end, aim_xy, top_z, comp, j1):
    """한 대의 놓기 시간표 [(t, q5, 손가락 m)] (ROS JointTimeline 규칙) · tip · IK 최대 오차."""
    import numpy as np
    x, y = float(aim_xy[0]), float(aim_xy[1])
    if comp:
        r = math.hypot(x - j1[0], y - j1[1])
        c = (comp["a"] + comp["b"] * (r - 0.2665) / 0.06) / 1000.0
        x += (x - j1[0]) / r * c
        y += (y - j1[1]) / r * c
        so = comp["src"].get(f"{slot['row']}_{slot['index']}")
        if so:
            x += so[0] / 1000.0
            y += so[1] / 1000.0
        y += comp["lat"] / 1000.0
    tip = np.array([x, y, top_z + PUCK_H / 2 + GRASP_Z + CLEAR])
    yaw = math.atan2(tip[1] - j1[1], tip[0] - j1[0])
    d = np.array([math.cos(PLACE_PITCH) * math.cos(yaw), math.cos(PLACE_PITCH) * math.sin(yaw), -math.sin(PLACE_PITCH)])
    pre = back = tip - d * APPROACH
    q_pre, emax = ik45(pre, yaw, None)
    pts = [(0.0, list(q_end), FINGER_CLOSE)]

    def move_through(q, speed, grip, first_min):
        dq = max(abs(a - b) for a, b in zip(q, pts[-1][1]))
        pts.append((pts[-1][0] + max(first_min, dq / speed), list(q), grip))

    def hold(sec, grip):
        pts.append((pts[-1][0] + sec, pts[-1][1], grip))
    move_through(q_pre, 0.8, FINGER_CLOSE, 1.0)
    seed = q_pre
    for a_, b_, grip in ((pre, tip, FINGER_CLOSE), (tip, back, RELEASE_PLACE)):
        nn = max(1, math.ceil(np.linalg.norm(b_ - a_) / 0.01 - 1e-9))
        for i in range(1, nn + 1):
            q, e = ik45(a_ + (b_ - a_) * i / nn, yaw, seed[1:4])
            emax = max(emax, e)
            dq = max(abs(u_ - v_) for u_, v_ in zip(q, seed))
            pts.append((pts[-1][0] + max((float(np.linalg.norm(b_ - a_)) / nn) / 0.05, dq / 0.8), q, grip))
            seed = q
        if grip == FINGER_CLOSE:
            hold(1.0, RELEASE_PLACE)   # gripper.move_time
            hold(0.3, RELEASE_PLACE)   # pallet.place_dwell
    move_through(TRAJ["transit"]["q_front_tilt"], 0.8, RELEASE_PLACE, 1.0)
    hold(1.5, RELEASE_PLACE)           # 끝난 뒤 퍽이 자리 잡을 시간 (판정용)
    return pts, tip, emax


def place_test(u, case, is_box, ids, step, puck_pose, bias, q_end, unload_res, box):
    dev, n, dt = u.device, u.num_envs, u.physics_dt
    comp = _place_params() if args_cli.place_comp else None
    import pallet_traj
    j1 = pallet_traj.Arm(pallet_traj.URDF).j1
    plans, tips, ik_err = [], [], []
    for i, (s, _) in enumerate(case):
        pts, tip, e = place_plan(s, q_end[i].tolist(), box["aim"][i], float(box["top"][i]), comp, j1)
        plans.append(pts); tips.append(tip); ik_err.append(e)
    T = int(max(p[-1][0] for p in plans) / dt) + 1
    print(f"[DicePlace] {n} 대, 놓기 {T * dt:.1f} s, 보정 {'켬 ' + comp['file'] if comp else '끔'}, IK 최대 오차 {max(ik_err) * 1000:.2f} mm", flush=True)

    def at(pts, t):
        for k in range(1, len(pts)):
            if t <= pts[k][0]:
                a, b = pts[k - 1], pts[k]
                f = (t - a[0]) / max(b[0] - a[0], 1e-9)
                return [x + (y - x) * f for x, y in zip(a[1], b[1])], a[2] + (b[2] - a[2]) * f
        return pts[-1][1], pts[-1][2]
    for t in range(T):
        qs, gs = zip(*(at(p, t * dt) for p in plans))
        plan = torch.tensor(qs, device=dev, dtype=torch.float32)
        if args_cli.ki > 0:
            bias = torch.clamp(bias + args_cli.ki * dt * (plan - u.robot.data.joint_pos.torch[:, u._arm]), -args_cli.bias_max, args_cli.bias_max)
        u.arm_target[:] = plan + bias
        u.grip[:, 0] = torch.tensor(gs, device=dev, dtype=torch.float32)
        step()
        if t % 240 == 0:
            print(f"[DicePlace] t={t * dt:5.1f} s", flush=True)
    rp, rq = u.robot.data.root_link_pos_w.torch, u.robot.data.root_link_quat_w.torch
    pc, qc = puck_pose()
    ax = quat_apply(qc, torch.tensor([[0, 0, 1.0]], device=dev).expand(n, 3))
    ax = ax * torch.where(ax[:, 2:3] < 0, -1.0, 1.0)
    bot = quat_apply_inverse(rq, pc - ax * (PUCK_H / 2) - rp).cpu()
    tilt = torch.rad2deg(torch.acos(torch.clamp(ax[:, 2], max=1.0))).cpu()
    nbm = torch.zeros(n)
    if box["nb0"] is not None:
        for k in range(8):
            d = torch.linalg.norm((u.scene[f"bn{k}"].data.root_pos_w.torch - box["nb0"][k])[:, :2], dim=1).cpu()
            nbm = torch.maximum(nbm, d)
    res = []
    for i, (s, shape) in enumerate(case):
        err = math.hypot(float(bot[i, 0] - box["true"][i, 0]), float(bot[i, 1] - box["true"][i, 1]))
        dz = float(bot[i, 2] - box["top"][i])
        stand = float(tilt[i]) < 10.0 and abs(dz) < 0.005
        ok_unload = bool(unload_res[i]["held"])
        ok = ok_unload and stand and err < 0.010 and float(nbm[i]) < 0.004
        res.append(dict(slot=f"{s['row']}_{s['index']}", puck_shape=shape, cell=[box["dx"][i], box["dy"][i]], unload_ok=ok_unload, placed=ok,
                        err_mm=round(err * 1000, 1), err_xy_mm=[round(float(bot[i, 0] - box["true"][i, 0]) * 1000, 1), round(float(bot[i, 1] - box["true"][i, 1]) * 1000, 1)],
                        standing=stand, tilt_deg=round(float(tilt[i]), 1), dz_mm=round(dz * 1000, 1),
                        nb_moved_mm=round(float(nbm[i]) * 1000, 1), nb_count=int(box["nb_count"][i]), box_gap_mm=round(float(box["gap"][i]) * 1000, 1),
                        box_lat_mm=round(float(box["lat"][i]) * 1000, 1), box_yaw_deg=round(float(box["yaw_deg"][i]), 2),
                        j1_dist_m=round(math.hypot(tips[i][0] - j1[0], tips[i][1] - j1[1]), 4), ik_err_mm=round(ik_err[i] * 1000, 2)))
    good = [r for r in res if r["unload_ok"]]
    errs = sorted(r["err_mm"] for r in good if r["standing"])
    pct = lambda q: errs[min(len(errs) - 1, int(q * len(errs)))] if errs else None
    by = lambda key: {k: f"{sum(r['placed'] for r in res if key(r) == k)}/{sum(1 for r in res if key(r) == k)}" for k in sorted({key(r) for r in res})}
    summary = dict(placed=sum(r["placed"] for r in res), total=n, unload_ok=len(good), placed_given_unload=sum(r["placed"] for r in good),
                   standing=sum(r["standing"] for r in good), within_10mm=sum(r["standing"] and r["err_mm"] < 10 for r in good),
                   within_4mm=sum(r["standing"] and r["err_mm"] < 4 for r in good), nb_pushed=sum(r["nb_moved_mm"] >= 4 for r in good),
                   err_mm_p50=pct(0.5), err_mm_p95=pct(0.95), by_slot=by(lambda r: r["slot"]), by_cell=by(lambda r: f"{r['cell'][0]},{r['cell'][1]}"),
                   by_shape=by(lambda r: r["puck_shape"]),
                   config=dict(box_gap=args_cli.box_gap, box_lat=args_cli.box_lat, box_yaw=args_cli.box_yaw, box_err_mm=args_cli.box_err,
                               box_err_deg=args_cli.box_err_deg, place_comp=args_cli.place_comp, box_neighbors=args_cli.box_neighbors,
                               seed=args_cli.seed, ki=args_cli.ki, puck_mu=args_cli.puck_mu, grip_close=FINGER_CLOSE,
                               comp=({k: v for k, v in comp.items() if k != "file"} if comp else None)))
    for r in res[:min(n, 40)]:
        print(f"[DicePlace] {r['slot']} {r['puck_shape']:3s} 칸 {r['cell']} {'놓음' if r['placed'] else '실패'} | 어긋남 {r['err_mm']:5.1f} mm 기울기 {r['tilt_deg']:5.1f}° "
              f"높이 {r['dz_mm']:+5.1f} mm 옆퍽 {r['nb_moved_mm']:4.1f} mm | 꺼냄 {r['unload_ok']} | J1 {r['j1_dist_m']:.3f} m", flush=True)
    print(f"[DicePlace] 요약 {json.dumps(summary, ensure_ascii=False)}", flush=True)
    json.dump(dict(summary=summary, results=res), open(OUT, "w", encoding="utf-8"), ensure_ascii=False, indent=1)



def drop_test(u, case, is_box, ids):
    """팔 없이: 칸마다 퍽을 칸 축 · 칸 방향 (상자 면 = 반경 · 접선) 에 맞춰 퍽 바닥 = 칸 벽 윗면 + 5 mm 에서 놓음 -> 1.5 s 뒤."""
    dev, n, dt = u.device, u.num_envs, u.physics_dt
    rp, rq = u.robot.data.root_link_pos_w.torch, u.robot.data.root_link_quat_w.torch
    loc = torch.tensor([[s["x"], s["y"], s["wall_top_z"] + 0.005 + PUCK_H / 2] for s, _ in case], device=dev)
    th = torch.tensor([math.radians(s["theta_deg"]) for s, _ in case], device=dev)
    q_loc = torch.stack((torch.zeros_like(th), torch.zeros_like(th), torch.sin(th / 2), torch.cos(th / 2)), dim=1)  # (x,y,z,w), 루트 요 0 가정
    pose = torch.cat((rp + quat_apply(rq, loc), q_loc), dim=1)
    for k, m in (("puckC", ~is_box), ("puckB", is_box)):
        o = u.scene[k]
        cur = torch.cat((o.data.root_pos_w.torch, o.data.root_quat_w.torch), dim=1)
        o.write_root_pose_to_sim_index(root_pose=torch.where(m[:, None], pose, cur), env_ids=ids)
        o.write_root_velocity_to_sim_index(root_velocity=torch.zeros(n, 6, device=dev), env_ids=ids)
    def yawdeg():
        qq = torch.where(is_box[:, None], u.scene["puckB"].data.root_quat_w.torch, u.scene["puckC"].data.root_quat_w.torch)
        xb = quat_apply_inverse(rq, quat_apply(qq, torch.tensor([[1.0, 0, 0]], device=dev).expand(n, 3)))
        return [round(((math.degrees(math.atan2(float(xb[i, 1]), float(xb[i, 0]))) - s["theta_deg"] + 45) % 90) - 45, 1) for i, (s, _) in enumerate(case)]
    u.scene.write_data_to_sim(); u.sim.step(render=False); u.scene.update(dt)
    print(f"[PalletDrop] 놓은 직후 면 방향 오차 (상자) {[v for v, (_, sh) in zip(yawdeg(), case) if sh == 'box']}", flush=True)
    for _ in range(int(1.5 / dt)):
        u._apply_action()
        u.scene.write_data_to_sim()
        u.sim.step(render=args_cli.gui)
        u.scene.update(dt)
    pc = torch.where(is_box[:, None], u.scene["puckB"].data.root_pos_w.torch, u.scene["puckC"].data.root_pos_w.torch)
    qc = torch.where(is_box[:, None], u.scene["puckB"].data.root_quat_w.torch, u.scene["puckC"].data.root_quat_w.torch)
    ax = quat_apply(qc, torch.tensor([[0, 0, 1.0]], device=dev).expand(n, 3))
    bot = quat_apply_inverse(rq, pc - ax * (PUCK_H / 2) - rp)
    ok = 0
    for i, (s, shape) in enumerate(case):
        dz = float(bot[i, 2]) - s["floor_z"]
        tilt = math.degrees(math.acos(min(1.0, abs(float(ax[i, 2])))))
        ok += abs(dz) < 0.003 and tilt < 3
        print(f"[PalletDrop] {s['row']}열 {s['index']} {shape:3s} 칸 바닥 위 {dz * 1000:+6.1f} mm 기울기 {tilt:4.1f}°", flush=True)
    pcb = quat_apply_inverse(rq, pc - rp)
    print(f"[PalletDrop] 끝 면 방향 오차 (상자) {[v for v, (_, sh) in zip(yawdeg(), case) if sh == 'box']}", flush=True)
    print(f"[PalletDrop] 끝 퍽 중심 - 칸 축 xy mm (상자) {[(round((float(pcb[i, 0]) - s['x']) * 1000, 1), round((float(pcb[i, 1]) - s['y']) * 1000, 1)) for i, (s, sh) in enumerate(case) if sh == 'box']}", flush=True)
    print(f"[PalletDrop] 요약 앉음 {ok}/{n}", flush=True)


def main():
    cfg = PalletCheckEnvCfg()
    cfg.scene.num_envs = 2 * len(TRAJ["slots"]) * len(HAND_OFFSETS) if HAND_OFFSETS else args_cli.num_envs
    env = PalletCheckEnv(cfg)
    env.reset()
    u, dev, n, dt = env, env.device, env.num_envs, env.physics_dt
    ids = torch.arange(n, device=dev)
    if ROLL_PI:
        j5 = u.robot.find_joints(["joint5"])[0]
        u.robot.write_joint_position_limit_to_sim_index(limits=torch.tensor([[[-3.3, 3.3]]], device=dev).repeat(n, 1, 1), joint_ids=j5)
        print(f"[PalletCheck] 손목 롤 180° 칸 {sorted(ROLL_PI)} (joint5 한계 ±3.3 rad 로 늘림)", flush=True)
    slots = TRAJ["slots"]
    case = [(slots[i % len(slots)], "box" if (i // len(slots)) % 2 else "cyl") for i in range(n)]
    global NB_OF
    if args_cli.neighbors == "pairs":   # (목표 칸, 같은 열 바로 옆 칸 = 번호 ±1) 순서쌍 x 목표 퍽 모양 2
        adj = [(s, o) for s in slots for o in slots if o["row"] == s["row"] and abs(o["index"] - s["index"]) == 1]
        base = [(s, sh, o) for s, o in adj for sh in ("cyl", "box")]
        case = [(base[i % len(base)][0], base[i % len(base)][1]) for i in range(n)]
        NB_OF = [base[i % len(base)][2] for i in range(n)]
        print(f"[NbPairs] 쌍 {len(adj)} x 모양 2 = {len(base)}, 대 {n}", flush=True)
    is_box = torch.tensor([c[1] == "box" for c in case], device=dev)
    blk = 2 * len(slots)  # --hand_offset 묶음 = 칸 x 모양
    hoff = torch.tensor([HAND_OFFSETS[i // blk] if HAND_OFFSETS else (0.0, 0.0) for i in range(n)], device=dev) / 1000.0  # (n, 2) 집게 x · z [m]

    def step(k_force=None, acc=None):
        u._apply_action()
        u.scene.write_data_to_sim()
        u.sim.step(render=args_cli.gui)
        u.scene.update(dt)

    def puck_pose():
        pc, qc = u.scene["puckC"].data.root_pos_w.torch, u.scene["puckC"].data.root_quat_w.torch
        pb, qb = u.scene["puckB"].data.root_pos_w.torch, u.scene["puckB"].data.root_quat_w.torch
        return torch.where(is_box[:, None], pb, pc), torch.where(is_box[:, None], qb, qc)

    def ee_frame():
        d = u.robot.data
        p, q = d.body_link_pos_w.torch[:, u._ee], d.body_link_quat_w.torch[:, u._ee]
        ex = quat_apply(q, torch.tensor([[1.0, 0, 0]], device=dev).expand(n, 3))
        ey = quat_apply(q, torch.tensor([[0, 1.0, 0]], device=dev).expand(n, 3))
        ez = quat_apply(q, torch.tensor([[0, 0, 1.0]], device=dev).expand(n, 3))
        return p, ex, ey, ez

    if args_cli.drop:
        drop_test(u, case, is_box, ids)
        env.close()
        return

    # 칸마다 앞 운반 자세 (기울인 칸은 집게를 기울인 자세 q_front)
    tilted = torch.tensor([bool(c[0].get("tilt_deg")) for c in case], device=dev)
    q_start = torch.tensor([c[0].get("q_front", Q_FRONT) for c in case], device=dev)
    qj = u.robot.data.joint_pos.torch.clone()
    qj[:, u._arm] = q_start
    u.robot.write_joint_position_to_sim_index(position=qj, env_ids=ids)
    u.robot.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(qj), env_ids=ids)
    u.arm_target[:] = q_start
    if tilted.any():
        print(f"[PalletCheck] 집게 기울여 놓는 칸 {sorted({(c[0]['row'], c[0]['index'], c[0]['tilt_deg']) for c in case if c[0].get('tilt_deg')})}", flush=True)

    def puck_up(ex):
        """쥔 퍽 축 (위): 수직 칸 = 집게 반대, 기울인 칸 = 차체 위 (퍽은 세운 채)."""
        up_b = quat_apply(u.robot.data.root_link_quat_w.torch, torch.tensor([[0, 0, 1.0]], device=dev).expand(n, 3))
        return torch.where(tilted[:, None], up_b, -ex)

    def exp_center(pe, ex):
        """쥔 퍽 중심 기대값 = TCP − 퍽 축 x GRASP_DZ (수직이면 gripper x 0.155 와 같음)."""
        return pe + ex * TRAJ["tcp_offset_ee"][0] - puck_up(ex) * TRAJ["grasp_dz"]

    if args_cli.unload:
        assert not ROLL_PI, "--unload 는 --roll_pi 없이"
        unload_test(u, case, is_box, ids, q_start, step, puck_pose, ee_frame, exp_center, puck_up)
        env.close()
        return

    place_neighbors(u, case, ids)
    # 1. 앞 운반 자세에서 자리 잡기 (집게 벌림, 처짐 보정이 자리 잡을 때까지) -> 퍽을 손가락 사이에 -> 쥠
    bias = torch.zeros(n, 5, device=dev)
    for _ in range(int((1.5 if args_cli.settle_ki else 0.5) / dt)):
        if args_cli.settle_ki and args_cli.ki > 0:
            bias = torch.clamp(bias + args_cli.ki * dt * (q_start - u.robot.data.joint_pos.torch[:, u._arm]), -args_cli.bias_max, args_cli.bias_max)
            u.arm_target[:] = q_start + bias
        step()
    p, ex, ey, ez = ee_frame()
    center = exp_center(p, ex) + ex * hoff[:, :1] + ez * hoff[:, 1:]  # --hand_offset (없으면 0)
    up = puck_up(ex)
    R = torch.stack((torch.cross(ey, up, dim=1), ey, up), dim=2)  # 퍽 y = 닫는 축, 퍽 z = 위, 퍽 x = y x z (수직이면 집게 z = 반경 방향)
    pose = torch.cat((center, quat_from_matrix(R)), dim=1)
    qj = u.robot.data.joint_pos.torch.clone()
    qj[:, u._gl[0]] = FINGER_AT_PUCK
    qj[:, u._gr[0]] = FINGER_AT_PUCK
    u.robot.write_joint_position_to_sim_index(position=qj, env_ids=ids)
    u.robot.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(qj), env_ids=ids)
    u.grip[:] = FINGER_CLOSE
    for k, m in (("puckC", ~is_box), ("puckB", is_box)):
        o = u.scene[k]
        cur = torch.cat((o.data.root_pos_w.torch, o.data.root_quat_w.torch), dim=1)
        o.write_root_pose_to_sim_index(root_pose=torch.where(m[:, None], pose, cur), env_ids=ids)
        o.write_root_velocity_to_sim_index(root_velocity=torch.zeros(n, 6, device=dev), env_ids=ids)
    if args_cli.debug:
        rp, rq = u.robot.data.root_link_pos_w.torch, u.robot.data.root_link_quat_w.torch
        b = lambda w: [round(v, 4) for v in quat_apply_inverse(rq[:1], w[:1] - rp[:1])[0].tolist()]
        print(f"[PalletDbg] 팔 관절 {[round(v, 3) for v in u.robot.data.joint_pos.torch[0, u._arm].tolist()]} (목표 {[round(v, 3) for v in Q_FRONT]})"
              f" 손가락 {u.robot.data.joint_pos.torch[0, u._gl[0]].item():.4f}", flush=True)
        print(f"[PalletDbg] 집게 원점 {b(p)} 집게 x {[round(v, 3) for v in ex[0].tolist()]} 기대 퍽 중심 {b(center)} 쿼터니언 {[round(v, 3) for v in pose[0, 3:].tolist()]}", flush=True)
        for k in range(61):
            pc_, qc_ = puck_pose()
            if k in (0, 1, 2, 5, 10, 30, 60):
                print(f"[PalletDbg] {k} 스텝 퍽 {b(pc_)} 자세 {[round(v, 3) for v in qc_[0].tolist()]} 퍽-몸체 {u.force('puckC_body')[0].item():.1f} N", flush=True)
            step()
        env.close()
        return
    u.grip[:] = FINGER_CLOSE
    for _ in range(int(0.6 / dt)):
        step()
    fing0 = u.robot.data.joint_pos.torch[:, u._gl[0]].clone()
    pe, ex, _, _ = ee_frame()
    pc0, _ = puck_pose()
    e0 = pc0 - exp_center(pe, ex)  # 쥔 직후 손 안 어긋남 (미끄럼은 이것 대비 변화)
    off0 = torch.linalg.norm(e0, dim=1)
    print(f"[PalletCheck] 쥔 직후: 손가락 왼 {[round(v, 4) for v in fing0.tolist()]} 오른 {[round(v, 4) for v in u.robot.data.joint_pos.torch[:, u._gr[0]].tolist()]} 퍽-손가락 가운데 mm {[round(v * 1000, 1) for v in off0.tolist()]}", flush=True)

    # 2 ~ 3. 궤적
    tl = [timeline(c[0], dt) for c in case]
    T = max(len(x) for x in tl)
    tgt = torch.tensor([[x[min(t, len(x) - 1)][0] for t in range(T)] for x in tl], device=dev)  # (n, T, 5)
    grp = torch.tensor([[x[min(t, len(x) - 1)][1] for t in range(T)] for x in tl], device=dev)
    phs = torch.tensor([[x[min(t, len(x) - 1)][2] for t in range(T)] for x in tl], device=dev)
    sgs = torch.tensor([[x[min(t, len(x) - 1)][3] for t in range(T)] for x in tl], device=dev)
    first = [dict() for _ in range(n)]  # 처음 닿은 때 (종류별): 구간, 시각, 관절, 퍽 바닥 (base)
    fmax = {k: torch.zeros(n, len(PHASES), device=dev) for k in ("arm", "finger", "puck")}
    slip = torch.zeros(n, device=dev)
    print(f"[PalletCheck] {n} 대, 궤적 {T} 스텝 ({T * dt:.1f} s), 쥔 손가락 {fing0.mean().item():.4f}", flush=True)
    if not args_cli.settle_ki:
        bias = torch.zeros(n, 5, device=dev)
    for t in range(T):
        if args_cli.ki > 0:
            bias = torch.clamp(bias + args_cli.ki * dt * (tgt[:, t] - u.robot.data.joint_pos.torch[:, u._arm]), -args_cli.bias_max, args_cli.bias_max)
        u.arm_target[:] = tgt[:, t] + bias
        u.grip[:, 0] = torch.where(grp[:, t] > 0, torch.tensor(FINGER_RELEASED, device=dev), torch.tensor(FINGER_CLOSE, device=dev))
        step()
        ph = phs[:, t]
        fp = torch.where(is_box, u.force("puckB_body"), u.force("puckC_body"))
        for k, f in (("arm", u.force("arm_body")), ("finger", u.force("finger_body")), ("puck", fp)):
            fmax[k].scatter_reduce_(1, ph[:, None], f[:, None], reduce="amax")
            hit = (f > 2.0).nonzero().flatten().tolist()
            if hit:
                rp_, rq_ = u.robot.data.root_link_pos_w.torch, u.robot.data.root_link_quat_w.torch
                pc_, qc_ = puck_pose()
                az = quat_apply(qc_, torch.tensor([[0, 0, 1.0]], device=dev).expand(n, 3))
                pbb = quat_apply_inverse(rq_, pc_ - az * (PUCK_H / 2) - rp_)
                who = u.force_body({"arm": "arm_body", "finger": "finger_body"}[k]) if k != "puck" else None
                for i in hit:
                    if k not in first[i]:
                        first[i][k] = dict(link=who[i] if who else None, seg=SEGS[int(sgs[i, t])], t_s=round(t * dt, 2), q=[round(v, 3) for v in u.robot.data.joint_pos.torch[i, u._arm].tolist()],
                                           puck_bottom_b=[round(v, 4) for v in pbb[i].tolist()], N=round(float(f[i]), 1))
        held = ph <= 2
        if held.any():
            pe, ex, _, _ = ee_frame()
            pc, _ = puck_pose()
            slip = torch.where(held, torch.maximum(slip, torch.linalg.norm(pc - exp_center(pe, ex) - e0, dim=1)), slip)
        if t % 240 == 0:
            print(f"[PalletCheck] t={t * dt:5.1f} s 단계 {[PHASES[int(v)] for v in ph[:min(n, 12)].tolist()]}", flush=True)

    # 4. 결과 (base_link 좌표)
    d = u.robot.data
    rp, rq = d.root_link_pos_w.torch, d.root_link_quat_w.torch
    pc, qc = puck_pose()
    ax = quat_apply(qc, torch.tensor([[0, 0, 1.0]], device=dev).expand(n, 3))
    ax = ax * torch.where(ax[:, 2:3] < 0, -1.0, 1.0)
    bot = quat_apply_inverse(rq, pc - ax * (PUCK_H / 2) - rp)
    pxb = quat_apply_inverse(rq, quat_apply(qc, torch.tensor([[1.0, 0, 0]], device=dev).expand(n, 3)))
    res = []
    for i, (s, shape) in enumerate(case):
        dxy = math.hypot(float(bot[i, 0]) - s["x"], float(bot[i, 1]) - s["y"])
        dz = float(bot[i, 2]) - s["floor_z"]
        tilt = math.degrees(math.acos(min(1.0, float(ax[i, 2]))))
        yaw = math.degrees(math.atan2(float(pxb[i, 1]), float(pxb[i, 0])))
        face = ((yaw - s["theta_deg"] + 45.0) % 90.0) - 45.0
        ok = dxy < 0.003 and abs(dz) < 0.003 and tilt < 5.0 and float(slip[i]) < 0.015
        r = dict(slot=f"{s['row']}열 {s['index']}", puck_shape=shape, seated=ok, hand_offset_mm=[round(float(v) * 1000, 1) for v in hoff[i]],
                 hand_after_grip_mm=round(float(off0[i]) * 1000, 1), xy_err_mm=round(dxy * 1000, 1), bottom_above_floor_mm=round(dz * 1000, 1),
                 tilt_deg=round(tilt, 1), box_face_err_deg=round(face, 1) if shape == "box" else None, max_slip_mm=round(float(slip[i]) * 1000, 1),
                 duration_s=round(len(tl[i]) * dt, 1),
                 max_force_N={k: {PHASES[j]: round(float(fmax[k][i, j]), 1) for j in range(1, len(PHASES))} for k in fmax}, first_contact=first[i])
        res.append(r)
        f = r["max_force_N"]
        print(f"[PalletCheck] {r['slot']} {shape:3s} {'앉음' if ok else '실패'} | 수평 {r['xy_err_mm']:5.1f} mm 높이 {r['bottom_above_floor_mm']:+6.1f} mm 기울기 {r['tilt_deg']:5.1f}°"
              f"{'' if shape != 'box' else f' 면 {face:+5.1f}°'} | 미끄럼 {r['max_slip_mm']:4.1f} mm | 팔-몸체 N carry {f['arm']['carry']} place {f['arm']['place']} after {f['arm']['after']}"
              f" | 손가락-몸체 place {f['finger']['place']} release {f['finger']['release']} after {f['finger']['after']} | 퍽-몸체 carry {f['puck']['carry']}", flush=True)
        for k, v in first[i].items():
            print(f"[PalletCheck]     처음 닿음 {k}: {v}", flush=True)
    summary = dict(seated=sum(r["seated"] for r in res), total=n, speed_rad_s=args_cli.speed, ki=args_cli.ki, puck_mu=args_cli.puck_mu, roll_pi=sorted(ROLL_PI), tilt_slots=sorted({(c[0]["row"], c[0]["index"]) for c in case if c[0].get("tilt_deg")}), tilt_speed=args_cli.tilt_speed,
                   row_speed=ROW_SPEED, empty_speed=args_cli.empty_speed, duration_s_per_slot={r["slot"]: r["duration_s"] for r in res[:len(TRAJ["slots"])]},
                   grip_close=FINGER_CLOSE, settle_ki=args_cli.settle_ki,
                   arm_body_max_N=round(float(fmax["arm"].max()), 1), finger_body_max_N=round(float(fmax["finger"].max()), 1))
    if HAND_OFFSETS:  # 어긋남별 앉음 (원기둥 · 상자)
        summary["grasp_dz"] = TRAJ["grasp_dz"]
        summary["by_hand_offset"] = {}
        for k, ho in enumerate(HAND_OFFSETS):
            rr = res[k * blk:(k + 1) * blk]
            c = sum(x["seated"] for x in rr if x["puck_shape"] == "cyl")
            b = sum(x["seated"] for x in rr if x["puck_shape"] == "box")
            summary["by_hand_offset"][f"{ho[0]:g}:{ho[1]:g}"] = dict(seated=c + b, total=len(rr), cyl=c, box=b,
                                                                      finger_body_max_N=round(max(max(x["max_force_N"]["finger"].values()) for x in rr), 1))
            print(f"[PalletCheck] 손 안 어긋남 집게축 {ho[0]:+g} 옆 {ho[1]:+g} mm: 앉음 {c + b}/{len(rr)} (원기둥 {c} · 상자 {b})", flush=True)
    print(f"[PalletCheck] 요약 {summary}", flush=True)
    json.dump(dict(summary=summary, traj_max_tilt_deg=TRAJ.get("max_tilt_deg"), results=res), open(OUT, "w", encoding="utf-8"),
              ensure_ascii=False, indent=1)
    env.close()


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
