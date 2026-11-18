"""매니퓰레이터 모델 이어 붙여 실행 (ROS 상태 머신의 시뮬레이션 판, 정책_학습기록 4.11 절):

  상자 위 퍽 --[1 집기 SidePick SP3]--> 쥐고 3 cm 듦 (0.5 s) --[2 옮기기·맞추기 Store ST3]--> 통 축 2.5 mm·입구 위 12 mm·직립
             --[3 넣기 Insert]--> 퍽 통 안 (입구 아래 60 mm) + 집게 위로 뺌 -> 끝 (팔 멈춤)

모델 파일은 그대로 (합치지 않음). 이 스크립트가 세 모델을 불러 단계마다 관측을 그 모델이 학습한 방식대로 만들어 넣고,
행동도 그 모델 방식대로 해석 (1·2 = 관절 += 0.04 rad, 3 = 잡는 점 1 mm + IK).
장면 = 정렬·집기 장면 (상자 + 퍽, 원기둥만) + 스토리지 접촉 거리 2 mm + 팔·손가락-몸체 접촉 센서. 목표 통 = P·N 번갈아.

    ./run_isaac.sh chain_play [환경 수]   창으로 (기본 4 대)
    ./run_isaac.sh chain_eval [환경 수]   창 없이 단계별 성공률 (기본 64 대)
"""

import argparse
import os
import sys

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--num_envs", type=int, default=4)
parser.add_argument("--gui", action="store_true")
parser.add_argument("--seconds", type=float, default=25.0)
parser.add_argument("--sidepick", default=None)
parser.add_argument("--store", default=None)
parser.add_argument("--insert", default=None)
parser.add_argument("--insert_ik", action="store_true", help="3 단계 넣기를 강화 학습 모델 대신 손 IK 로: 퍽 바닥을 통 축에 맞추며 1 mm / 스텝 내려 10 mm -> 열기 -> 위로 빼기 (store_hybrid 와 같음)")
parser.add_argument("--ik_depth", type=float, default=0.010)  # 2026-10-31: 14 -> 10 mm (턱 위 5 mm 에서 놓음, 팀 시험 16 mm 위에서 놓아도 12/12 앉음). 14 mm 는 손목 한계로 손가락이 벽을 눌러 못 놓는 대가 생김
parser.add_argument("--ik_gate_clear", type=float, default=0.04, help="손 IK 넘김: 퍽 바닥이 입구 위 이 높이 [m] 안. 2026-10-31: ST-D5 는 입구 위 15 mm 로 옴 -> 0.04 (C1 시절 0.25 는 높은 곳에서 IK 로 내려 팔이 몸체에 부딪힘)")
parser.add_argument("--ik_gate_up", type=float, default=0.95, help="손 IK 넘김: 퍽 직립 (바깥 컵은 퍽 6 ~ 7° 기울어야 닿음)")
parser.add_argument("--ik_rot_w", type=float, default=0.05, help="손 IK 넣기의 집게 자세 유지 가중치 (store_hybrid 와 같은 0.05). 2026-10-30: 넣기 정책 값 0.3 을 쓰면 컵 축까지 접선 방향 20 ~ 37 mm 를 못 움직임 (관절 1 을 돌리면 집게 요가 바뀌어 자세 유지와 다툼)")
parser.add_argument("--no_tight_collision", action="store_true", help="로봇·퍽 접촉 거리 0.002/0 덮어쓰기를 끔 (집기 학습 환경과 같은 기본 충돌). 2026-10-31: 체인 안 목표 퍽 넘어짐 17/64 (단독 1/64) 원인 확인용")
parser.add_argument("--ik_hold", default="radial", choices=("radial", "entry"), help="손 IK 넣기의 집게 자세 목표. radial (2026-10-31 기본): 매 스텝 '집게 방향 = joint1 축 -> 잡는 점 수평, 카메라 위' (관절 1 을 돌리면 목표도 같이 돌아감). entry: 넣기 단계에 들어온 순간 자세를 고정 (이전, 접선 방향 2 ~ 4 mm 오차가 자세 유지와 비겨 못 줄임)")
parser.add_argument("--no_return", action="store_true", help="넣기 끝난 뒤 초기 자세 복귀를 안 함 (2026-10-31 이전 동작)")
parser.add_argument("--ret_s", type=float, default=1.5, help="복귀 구간 하나 (접기 / 관절 1 돌리기 / 펴기) 시간 [s]")
parser.add_argument("--no_safety", action="store_true", help="집기 안전 계층 끔 (2026-10-31 추가: 집기 중 관절 1 을 퍽 방향 ±0.6 rad 로 제한, 목표 퍽이 넘어지거나 pick_timeout 안에 못 집으면 팔을 그 자리에 멈춤)")
parser.add_argument("--pick_timeout", type=float, default=8.0)
parser.add_argument("--lie", action="store_true", help="2026-11-01: 누운 직육면체 퍽 경로 (LiePick 정책으로 위에서 가운데 잡고 들기 -> 관절 보간 + 경유 자세로 세우기 -> 옮기기 -> 컵 입구 위에서 놓기 -> 복귀)")
parser.add_argument("--ik_depth_box", type=float, default=-0.002, help="누운 퍽 (가운데 쥠) 놓는 깊이 [m]: 손가락 밑이 퍽 바닥 위 약 1 mm 라 컵 안으로 못 넣음 -> 퍽 바닥이 입구 위 2 mm 에서 놓음 (팀 시험: 16 ~ 20 mm 떨어뜨려도 12/12 앉음)")
parser.add_argument("--stand_s", type=float, default=1.5, help="누운 퍽 세우기 구간 (들기 -> 경유 / 경유 -> 옆 쥐기) 하나 시간 [s]")
parser.add_argument("--debug_ik", action="store_true", help="넣기 단계 (손 IK) 첫 2 대의 축·깊이·관절·IK 명령을 5 스텝마다 출력 (2026-10-30 추적용)")
parser.add_argument("--rl_gate_mm", type=float, default=4.0, help="넣기 정책 넘김: 퍽 바닥-컵 축 [mm] (이전 2.5)")
parser.add_argument("--rl_gate_clear", type=float, default=0.025, help="넣기 정책 넘김: 퍽 바닥이 입구 위 이 안 [m]. 2026-10-30 ST-D 는 입구 + 15 mm 에 맞춤 (이전 12 mm)")
parser.add_argument("--occ", type=float, default=0.0, help="2026-11-01 연속 채우기: 목표가 아닌 바깥 컵 3 개에 퍽이 미리 꽂혀 있을 확률 (1.0 = 다 참). 0 = 빈 판")
parser.add_argument("--overhead", type=float, default=0.0, help="손 IK 넣기를 위에서 들어가기로: 쥔 퍽이 컵 축 이 [m] 안이면 넘겨받아 제자리에서 퍽 바닥을 입구 + oh_clear 로 올리고, 그 높이로 축 위까지 옮긴 뒤 곧게 내림. 0 = 끔 (예전 옆에서 낮게 들어가기)")
parser.add_argument("--oh_clear", type=float, default=0.06, help="[m] 위에서 들어가기 높이 (퍽 바닥 - 입구). 꽂힌 퍽은 입구 위 45 mm")
parser.add_argument("--oh_step", type=float, default=0.003, help="[m / 스텝] 위에서 들어가기 이동 속도")
parser.add_argument("--ik_lift", type=float, default=0.04, help="[m] 놓은 뒤 위로 빼는 높이 (2 mm / 스텝). 옆 컵이 차 있으면 0.08 (손가락 밑이 꽂힌 퍽 위로)")
parser.add_argument("--release_open", type=float, default=None, help="[m] 넣기·복귀 단계에서 놓을 때 손가락 목표 (집기는 그대로 0.010 = 61.6 mm). 옆 컵이 차 있으면 −0.003 (≈ 35.6 mm, 손가락 바깥 33 mm < 옆 퍽 면 39 mm)")
parser.add_argument("--ik_gate_mm", type=float, default=50.0, help="손 IK 넣기 때 옮기기 -> 넣기 넘어가는 통 축 거리 [mm] (옮기기 정책은 통 근처 4 cm 까지 확실, 마지막 맞추기는 손 IK)")
parser.add_argument("--collect_handover", default=None, help="집기가 끝난 (쥐고 3 cm 들어 0.5 s) 실제 상태를 이 json 에 모음 -> store_starts.json 의 H (이어 받기) 시작 상태. 집기 모델만 씀")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.headless = not args_cli.gui
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import glob  # noqa: E402
import json  # noqa: E402
import math  # noqa: E402

import gymnasium as gym  # noqa: E402
import torch  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
import waffle_isaac  # noqa: E402,F401
from isaaclab.envs import ViewerCfg  # noqa: E402
from isaaclab.sensors import ContactSensorCfg  # noqa: E402
from isaaclab.utils.math import quat_apply, quat_apply_inverse, quat_box_minus, quat_from_matrix, quat_mul  # noqa: E402
from waffle_isaac.dice2_cfg import PUCK_H, puck_cfg  # noqa: E402
from waffle_isaac.store_env import tube_code_from  # noqa: E402
from waffle_isaac.insert_env_cfg import INSERT_POSE, WaffleInsertEnvCfg  # noqa: E402
from waffle_isaac.robot_cfg import ARM_JOINTS, ARM_LINKS_REGEX, FINGER_LINKS_REGEX, ROOT_TO_BASE_LINK  # noqa: E402
from waffle_isaac.sidepick_env_cfg import WaffleSidePickEnvCfg  # noqa: E402
from waffle_isaac.liepick_env_cfg import WaffleLiePickEnvCfg  # noqa: E402
from waffle_isaac.store_env_cfg import BODY, START_POSE, WaffleStoreEnvCfg  # noqa: E402

PHASES = ("1 집기", "2 옮기기·맞추기", "3 넣기", "4 넣기 끝·복귀")
TUCK = (-0.1, 0.7, -1.6)  # 복귀 때 접은 자세 관절 2·3·4 (2026-10-31 MuJoCo return_path: 관절 1 을 −3 ~ 3 rad 돌려도 몸체에 안 닿음, 잡는 점 base z 0.39 = 컵 속 퍽 위로)


def pick_policy(arg, fixed, pattern):
    if arg:
        return arg
    if os.path.exists(fixed):
        return fixed
    return sorted(glob.glob(os.path.join(HERE, pattern)))[-1]


def main():
    p_sp = pick_policy(args_cli.sidepick, os.path.join(HERE, "policies/liepick_LP8.pt" if args_cli.lie else "policies/sidepick_latest.pt"),
                       "logs/rsl_rl/waffle_liepick/*/exported/policy.pt" if args_cli.lie else "logs/rsl_rl/waffle_sidepick/*/exported/policy.pt")
    p_st = None if args_cli.collect_handover else pick_policy(args_cli.store, os.path.join(HERE, "policies/store_latest.pt"), "logs/rsl_rl/waffle_store/*/exported/policy.pt")
    p_in = None if (args_cli.collect_handover or args_cli.insert_ik) else pick_policy(args_cli.insert, os.path.join(HERE, "policies/insert_latest.pt"), "logs/rsl_rl/waffle_insert/*/exported/policy.pt")

    cfg = WaffleLiePickEnvCfg() if args_cli.lie else WaffleSidePickEnvCfg()
    cfg.scene.num_envs = args_cli.num_envs
    cfg.box_shape_prob = 0.0  # 상자 모양 퍽은 통 (지름 약 32) 에 안 들어감
    cfg.near_start_prob = 0.0  # 실제 과제 배치 (학습용 역방향 커리큘럼 끔)
    cfg.episode_length_s = args_cli.seconds + 5.0
    cfg.viewer = ViewerCfg(eye=(0.85, -0.85, 0.75), lookat=(0.12, 0.0, 0.25), origin_type="env", env_index=0)
    col = sim_utils.CollisionPropertiesCfg(contact_offset=0.002, rest_offset=0.0)  # 넣기 학습과 같게 (통 틈 1 mm)
    if not args_cli.no_tight_collision:
        cfg.scene.robot = cfg.scene.robot.replace(spawn=cfg.scene.robot.spawn.replace(collision_props=col))
        cfg.scene.puckT_cyl = cfg.scene.puckT_cyl.replace(spawn=cfg.scene.puckT_cyl.spawn.replace(collision_props=col))
    cfg.scene.arm_body_contact = ContactSensorCfg(prim_path="{ENV_REGEX_NS}/Robot/" + ARM_LINKS_REGEX, filter_prim_paths_expr=[BODY],
                                                  update_period=0.0, history_length=1)
    cfg.scene.finger_body_contact = ContactSensorCfg(prim_path="{ENV_REGEX_NS}/Robot/" + FINGER_LINKS_REGEX,
                                                     filter_prim_paths_expr=[BODY], update_period=0.0, history_length=1)
    if args_cli.occ > 0:  # 2026-11-01: 컵에 미리 꽂힌 퍽 3 개 (치워 두는 곳 = 네 환경 사이 모서리)
        occ_paths = [f"{{ENV_REGEX_NS}}/PuckOcc{j}" for j in range(3)]
        for j in range(3):
            oc_ = puck_cfg(f"Occ{j}", "cyl", (0.55, 0.55, 0.6), (0.75, 0.65 + 0.1 * j))
            setattr(cfg.scene, f"occ{j}", oc_.replace(spawn=oc_.spawn.replace(activate_contact_sensors=True, collision_props=col)))  # 컵 틈 1 mm: 넣는 퍽과 같은 접촉 여유 (기본 여유면 생성 때 튀어나감)
        cfg.scene.robot_occ_contact = ContactSensorCfg(prim_path="{ENV_REGEX_NS}/Robot/" + "(link[2-5]|gripper_roll_link|gripper_left_link|gripper_right_link)",
                                                       filter_prim_paths_expr=occ_paths, update_period=0.0, history_length=1)
        cfg.scene.puck_occ_contact = ContactSensorCfg(prim_path="{ENV_REGEX_NS}/PuckOcc.*", filter_prim_paths_expr=["{ENV_REGEX_NS}/PuckT_" + ("box" if args_cli.lie else "cyl")],
                                                      update_period=0.0, history_length=1)
    env = gym.make("Waffle-LiePick-Direct" if args_cli.lie else "Waffle-SidePick-Direct", cfg=cfg)
    u = env.unwrapped
    r, n, dev = u.robot, u.num_envs, u.device
    collect = args_cli.collect_handover
    pols = [torch.jit.load(p, map_location=dev).eval() for p in ((p_sp, p_sp, p_sp) if collect else (p_sp, p_st, p_in or p_st))]
    rows = []
    print(f"[Chain] 모델 1 집기 {p_sp}\n[Chain] 모델 2 옮기기·맞추기 {p_st}\n[Chain] 모델 3 넣기 {'손 IK (' + str(args_cli.ik_depth * 1000) + ' mm 넣고 놓기)' if args_cli.insert_ik else p_in}", flush=True)
    obs, _ = env.reset()

    S = json.load(open(os.path.join(HERE, "store_starts.json"), encoding="utf-8"))
    tubes = torch.tensor([[t["axis"][0], t["axis"][1], t["rim_z"]] for t in S["tubes"]], device=dev)
    tube = torch.tensor([i % len(S["tubes"]) for i in range(n)], device=dev)  # 2026-10-30: 퍽 플레이트 바깥 줄 컵 4 개 번갈아
    tube_names = [t["name"] for t in S["tubes"]]
    occ_on = torch.zeros(n, 3, dtype=torch.bool, device=dev)
    if args_cli.occ > 0:
        seats = torch.tensor([t.get("seat_z", t["rim_z"] - 0.015) for t in S["tubes"]], device=dev)
        nt = len(S["tubes"])
        oth = torch.stack([torch.tensor([c for c in range(nt) if c != int(tb)][:3], device=dev) for tb in tube.tolist()])
        occ_on = torch.rand(n, 3, device=dev) < args_cli.occ
        occ_p0 = None  # 첫 스텝 뒤 (자리 잡은 뒤) 위치 기록
    occ_first = torch.full((n,), -1, dtype=torch.long, device=dev)  # 처음 1 N 넘게 닿은 단계 (0 ~ 3, 넣기는 4 + IK 단계)
    occ_fph = torch.zeros(n, 4, device=dev)
    occ_src = torch.full((n,), -1, dtype=torch.long, device=dev)
    oh_over = torch.zeros(n, dtype=torch.bool, device=dev)
    st_cfg, in_cfg = WaffleStoreEnvCfg(), WaffleInsertEnvCfg()
    q_start = torch.tensor([[START_POSE[j] for j in ARM_JOINTS]], device=dev)
    q_ins = torch.tensor([[INSERT_POSE[j] for j in ARM_JOINTS]], device=dev)
    bl = torch.tensor([[0.0, 0.0, ROOT_TO_BASE_LINK]], device=dev)
    ex = torch.tensor([[1.0, 0.0, 0.0]], device=dev).repeat(n, 1)
    ez = torch.tensor([[0.0, 0.0, 1.0]], device=dev).repeat(n, 1)
    cols = torch.tensor([getattr(r, "num_base_dofs", 6) + j for j in u._arm], device=dev)
    off = u._off

    def to_bl(p_w):
        return quat_apply_inverse(r.data.root_link_quat_w.torch, p_w - r.data.root_link_pos_w.torch) - bl

    def vec_bl(v_w):
        return quat_apply_inverse(r.data.root_link_quat_w.torch, v_w)

    def state():
        pc = to_bl(u.puck_pos_w())
        uu = vec_bl(quat_apply(u.puck_quat_w(), ez))
        uu = uu * torch.where(uu[:, 2:3] < 0, -1.0, 1.0)
        pb = pc - uu * (PUCK_H / 2)
        t = tubes[tube]
        gb = to_bl(u.grasp_w())
        return dict(pc=pc, u=uu, pb=pb, t=t, gb=gb, dxy=torch.linalg.norm(pb[:, :2] - t[:, :2], dim=1), depth=t[:, 2] - pb[:, 2],
                    held=torch.linalg.norm(pc - gb, dim=1) < 0.03, fing=r.data.joint_pos.torch[:, u._gl[0]])

    def force(k):
        f = u.scene[k].data.normal_force_matrix_w.torch
        return torch.linalg.norm(f, dim=-1).amax(dim=(1, 2))

    phase = torch.zeros(n, dtype=torch.long, device=dev)
    cnt = torch.zeros(n, dtype=torch.long, device=dev)
    t_enter = torch.zeros(n, 4, device=dev)
    prev6 = torch.zeros(n, 6, device=dev)
    prev6[:, 5] = -1.0  # 옮기기 모델은 쥔 채로 시작하게 학습됨
    ph_steps = torch.zeros(n, dtype=torch.long, device=dev)
    ik_stage = torch.zeros(n, dtype=torch.long, device=dev)  # 손 IK 넣기: 0 내림, 1 열기, 2 빼기
    ik_cnt = torch.zeros(n, dtype=torch.long, device=dev)
    prev4 = torch.zeros(n, 4, device=dev)
    prev4[:, 3] = -1.0
    q5_hold = torch.zeros(n, 4, device=dev)
    f_arm_max = torch.zeros(n, device=dev)
    f_fing_max = torch.zeros(n, device=dev)
    body_steps = torch.zeros(n, device=dev)
    # 2026-10-31 대별 실패 분류용 (정책_학습기록 6 절): 옆 퍽 닿음·목표 퍽 넘어짐 (집기 중), 쥔 적 있음, 처음 놓친 단계, 팔-몸체 처음 닿은 단계, 단계별 최대 힘
    nb_touch = torch.zeros(n, dtype=torch.bool, device=dev)
    toppled = torch.zeros(n, dtype=torch.bool, device=dev)
    ever_held = torch.zeros(n, dtype=torch.bool, device=dev)
    lost_phase = torch.full((n,), -1, dtype=torch.long, device=dev)
    hit_phase = torch.full((n,), -1, dtype=torch.long, device=dev)
    f_phase = torch.zeros(n, 3, device=dev)
    # 2026-10-31 넣기 (손 IK) 세부 기록: 맞춤 단계 결과 (조건 / 시간 끝), 맞춤 중 최소 축 오차, 맞춤 끝·내리기 끝의 축·깊이, 내리기 결과, 내리기 중 손가락-몸체 최대 힘, 퍽 직립
    ret_k = torch.full((n,), -1, dtype=torch.long, device=dev)  # 복귀 진행 스텝 (−1 = 시작 전)
    ret_q0 = torch.zeros(n, 5, device=dev)
    ret_hit = torch.zeros(n, device=dev)  # 복귀 중 팔-몸체 최대 힘
    q_init = r.data.joint_pos.torch[:, u._arm].clone()  # 초기 자세 (정렬·집기 시작 자세)
    dn_q = torch.zeros(n, 5, device=dev)  # 내리기 끝 관절
    pk_min_d = torch.full((n,), 9.0, device=dev)  # 집기 중 잡는 점-목표 최소 거리
    pk_close = torch.zeros(n, dtype=torch.bool, device=dev)  # 집기 중 집게 닫는 행동을 한 적
    pk_top_t = torch.full((n,), -1.0, device=dev)  # 목표 퍽이 넘어진 시각
    tb_init = to_bl(u.puck_pos_w())[:, :2].clone()  # 집을 퍽 처음 위치 (base)
    nb_init = to_bl(u._obj("puckN").data.root_pos_w.torch)[:, :2].clone()
    ret_hit_seg = torch.full((n,), -1, dtype=torch.long, device=dev)
    th_init = torch.atan2(tb_init[:, 1], tb_init[:, 0] - st_cfg.joint1_x)  # 집을 퍽 방향 (관절 1)
    aborted = torch.zeros(n, dtype=torch.bool, device=dev)  # 집기 실패로 멈춘 대
    abort_q = torch.zeros(n, 5, device=dev)
    hit_t = torch.full((n,), -1.0, device=dev)  # 팔-몸체 처음 닿은 시각·관절·링크
    hit_q = torch.zeros(n, 5, device=dev)
    hit_body = torch.full((n,), -1, dtype=torch.long, device=dev)
    hit_fmax = torch.zeros(n, device=dev)
    arm_sensor_names = list(u.scene["arm_body_contact"].body_names)
    lie_k = torch.full((n,), -1, dtype=torch.long, device=dev)  # 누운 퍽 세우기 진행 스텝 (−1 = 시작 전)
    lie_cnt = torch.zeros(n, dtype=torch.long, device=dev)
    lie_q0 = torch.zeros(n, 5, device=dev)
    lie_via = torch.zeros(n, 5, device=dev)
    lie_to = torch.zeros(n, 5, device=dev)
    stood = torch.zeros(n, dtype=torch.bool, device=dev)
    al_ok = torch.zeros(n, dtype=torch.bool, device=dev)
    al_tmo = torch.zeros(n, dtype=torch.bool, device=dev)
    al_min = torch.full((n,), 9.0, device=dev)
    al_end = torch.zeros(n, 2, device=dev)  # (축, 깊이)
    dn_end = torch.zeros(n, 3, device=dev)  # (축, 깊이, 직립)
    dn_tmo = torch.zeros(n, dtype=torch.bool, device=dev)
    dn_ff = torch.zeros(n, device=dev)
    al_xy = torch.zeros(n, 2, device=dev)  # 맞춤 끝 (컵 축 − 퍽 바닥) xy, base
    steps = int(args_cli.seconds / u.step_dt)
    arm_prev = u.arm_target.clone()
    go0 = float(cfg.gripper_open)
    fing_open_thr = -0.002 if args_cli.release_open is None else min(-0.002, args_cli.release_open - 0.001)  # 놓음 판정 (쥔 30 mm = −0.0058)
    def place_occ():  # 리셋 직후엔 로봇 위치 버퍼가 옛 값일 수 있어 몇 스텝 뒤에 놓음
        allid = torch.arange(n, device=dev)
        for j in range(3):
            ob = u.scene[f"occ{j}"]
            cj = oth[:, j]
            pb_ = torch.stack((tubes[cj, 0], tubes[cj, 1], seats[cj] + PUCK_H / 2 + 0.001), dim=1) + torch.tensor([[0.0, 0.0, ROOT_TO_BASE_LINK]], device=dev)
            pw = r.data.root_link_pos_w.torch + quat_apply(r.data.root_link_quat_w.torch, pb_)
            pos = torch.where(occ_on[:, j:j + 1], pw, ob.data.root_pos_w.torch)
            ob.write_root_pose_to_sim_index(root_pose=torch.cat((pos, ob.data.default_root_pose.torch[:, 3:7]), dim=1), env_ids=allid)
            ob.write_root_velocity_to_sim_index(root_velocity=torch.zeros(n, 6, device=dev), env_ids=allid)

    for k in range(steps):
        s = state()
        ax, _, az = u.a5_axes_w()
        d = r.data
        # 관측 (모델마다 학습 때와 같게)
        o_sp = u._get_observations()["policy"]
        tcode = tube_code_from(s["t"], st_cfg.joint1_x)  # 옮기기 환경과 같은 함수
        o_st = torch.cat((d.joint_pos.torch[:, u._arm] - q_start, d.joint_vel.torch[:, u._arm], d.joint_pos.torch[:, u._gl], s["gb"], s["pc"] - s["gb"],
                          s["u"], s["t"], s["pb"] - s["t"], vec_bl(-az), vec_bl(ax), tcode, prev6), dim=-1)
        est_bottom = s["gb"] - torch.tensor([[0.0, 0.0, in_cfg.grasp_dz + PUCK_H / 2]], device=dev)
        o_in = torch.cat((d.joint_pos.torch[:, u._arm] - q_ins, d.joint_vel.torch[:, u._arm], d.applied_torque.torch[:, u._arm], d.joint_pos.torch[:, u._gl],
                          s["gb"], s["t"], est_bottom - s["t"], vec_bl(-az), tcode, prev4), dim=-1)
        with torch.inference_mode():
            a_sp = pols[0](o_sp)
            a_st = pols[1](o_st) if not collect else torch.zeros(n, 6, device=dev)
            a_in = pols[2](o_in) if not (collect or args_cli.insert_ik) else torch.zeros(n, 4, device=dev)
        # 단계마다 행동 해석 (학습 때와 같게): 1 집기 = 잡는 점 5 mm 직교 + IK (집게 수평·바깥 방향), 2 옮기기 = 관절 += 0.04 rad,
        # 3 넣기 = 잡는 점 1 mm 직교 + IK (들어온 순간 자세 유지), 끝 = 멈춤·집게 벌림. 마지막 성분 = 집게 (> 0 벌림)
        q = d.joint_pos.torch[:, u._arm]
        lead = u.cfg.target_lead_max
        a_sp, a_st, a_in = a_sp.clamp(-1.0, 1.0), a_st.clamp(-1.0, 1.0), a_in.clamp(-1.0, 1.0)
        m0, m1, m3, m4 = phase == 0, phase == 1, phase == 2, phase == 3
        tgt = arm_prev.clone()
        grip = torch.ones(n, device=dev)
        if m0.any() and not args_cli.lie:
            dq0 = u._ik_step(a_sp[:, :3] * u.cfg.cart_scale)
            tgt[m0] = (arm_prev + dq0)[m0]
            grip[m0] = a_sp[m0, -1]
            u.prev_actions[m0] = u.actions[m0]
            u.actions[m0] = a_sp[m0]
        if m0.any() and args_cli.lie:  # 누운 퍽: LiePick 행동 해석 (학습 때와 같게: 집게 요 ψ·숙임 φ + 자세 목표 IK) -> 들면 관절 보간으로 세우기
            mp = m0 & (lie_k < 0)
            u.psi = torch.where(mp, u.psi + a_sp[:, 3] * u.cfg.psi_scale, u.psi)
            u.phi = torch.where(mp, torch.clamp(u.phi + a_sp[:, 4] * u.cfg.phi_scale, 0.0, math.pi / 2), u.phi)
            dq0 = u._ik_pose(a_sp[:, :3] * u.cfg.cart_scale)
            tgt[mp] = (arm_prev + dq0)[mp]
            grip[mp] = a_sp[mp, -1]
            u.prev_actions[mp] = u.actions[mp]
            u.actions[mp] = a_sp[mp]
            ms = m0 & (lie_k >= 0)
            if ms.any():
                Ns = max(1, int(args_cli.stand_s / u.step_dt))
                kk = lie_k.float().unsqueeze(1)
                def sm2(x):
                    x = x.clamp(0.0, 1.0)
                    return 0.5 - 0.5 * torch.cos(math.pi * x)
                qs = torch.where(kk < Ns, lie_q0 * (1 - sm2(kk / Ns)) + lie_via * sm2(kk / Ns), lie_via * (1 - sm2((kk - Ns) / Ns)) + lie_to * sm2((kk - Ns) / Ns))
                tgt[ms] = qs[ms]
                grip[ms] = -1.0
                lie_k[ms] += 1
                stood |= ms & (lie_k >= 2 * Ns)
        if m1.any():  # 옮기기 환경 _pre_physics_step 과 똑같이: 통 근처 정밀 모드 (x 0.25) + 손목 롤 0 고정
            fine = (s["dxy"] < st_cfg.fine_xy) & (-s["depth"] < st_cfg.zone_clear)
            sc = st_cfg.action_scale * torch.where(fine, st_cfg.fine_scale, 1.0).unsqueeze(1)
            t1 = arm_prev + sc * a_st[:, :5]
            if st_cfg.lock_roll:
                t1[:, 4] = 0.0
            tgt[m1] = t1[m1]
            grip[m1] = a_st[m1, 5]
            prev6[m1] = a_st[m1]
        if m3.any():
            q5 = d.body_link_quat_w.torch[:, u._a5]
            fresh = m3 & (ph_steps < 2)  # 넣기 단계 들어온 직후 자세를 유지 목표로
            q5_hold[fresh] = q5[fresh]
            J = d.body_link_jacobian_w.torch[:, u._a5][:, :, cols]
            rr = quat_apply(q5, off)
            Jl = J[:, 0:3] - torch.cross(rr.unsqueeze(-1).expand(-1, -1, 5), J[:, 3:6], dim=1)
            w = args_cli.ik_rot_w if args_cli.insert_ik else in_cfg.rot_hold_weight
            Jf = torch.cat((Jl, J[:, 3:6] * w), dim=1)
            if args_cli.insert_ik:  # 손 IK: 0 입구 위 8 mm 에서 축 맞춤 -> 1 1 mm/스텝 내림 (축 보정) -> 2 열기 0.5 s -> 3 위로 4 cm
                dpb = torch.zeros(n, 3, device=dev)
                xy = torch.clamp((s["t"][:, :2] - s["pb"][:, :2]) * 0.5, -0.004, 0.004)
                al, dn = m3 & (ik_stage == 0), m3 & (ik_stage == 1)
                dpb[:, :2] = torch.where((al | dn)[:, None], xy, dpb[:, :2])
                hz = torch.clamp(((s["t"][:, 2] + 0.008) - s["pb"][:, 2]) * 0.5, -0.003, 0.003)
                dpb[:, 2] = torch.where(al, hz, torch.where(dn, -0.001, torch.where(m3 & (ik_stage == 3), 0.002, 0.0)))
                if args_cli.overhead > 0:  # 2026-11-01 위에서 들어가기: 낮으면 제자리 올림 -> 그 높이로 축 위 -> (축 2 mm 안 고정) 입구 위 8 mm 까지 곧게 내림
                    clr = -s["depth"]
                    oh_over |= al & (s["dxy"] < 0.002) & (clr > args_cli.oh_clear - 0.008)
                    low = al & ~oh_over & (clr < args_cli.oh_clear - 0.005)
                    mv = al & ~oh_over & ~low
                    v = s["t"][:, :2] - s["pb"][:, :2]
                    v = v * torch.clamp(args_cli.oh_step / v.norm(dim=1, keepdim=True).clamp(min=1e-6), max=0.5)
                    dpb[:, :2] = torch.where(low[:, None], torch.zeros_like(v), torch.where(mv[:, None], v, dpb[:, :2]))
                    hz_oh = torch.clamp(args_cli.oh_clear - clr, -args_cli.oh_step, args_cli.oh_step)
                    dpb[:, 2] = torch.where(low | mv, hz_oh, dpb[:, 2])
                a_in = torch.cat((dpb / in_cfg.cart_scale, torch.where(m3 & (ik_stage >= 2), 1.0, -1.0).unsqueeze(1)), dim=1)
            if args_cli.ik_hold == "radial":  # 집게 방향 = joint1 축 -> 잡는 점 (수평), 카메라 위 (집기 환경 _ik_step 과 같은 목표)
                rq = d.root_link_quat_w.torch
                gb_ = s["gb"]
                ra = torch.stack((gb_[:, 0] - st_cfg.joint1_x, gb_[:, 1], torch.zeros_like(gb_[:, 0])), dim=1)
                ra = ra / ra.norm(dim=1, keepdim=True).clamp(min=1e-6)
                upv = torch.tensor([[0.0, 0.0, 1.0]], device=dev).expand(n, 3)
                Rt = torch.stack((ra, torch.cross(upv, ra, dim=1), upv), dim=2)
                q_tgt = quat_mul(rq, quat_from_matrix(Rt))
            else:
                q_tgt = q5_hold
            err = torch.cat((quat_apply(d.root_link_quat_w.torch, a_in[:, :3] * in_cfg.cart_scale), quat_box_minus(q_tgt, q5) * w), dim=1)
            dq = (Jf.transpose(1, 2) @ torch.linalg.solve(Jf @ Jf.transpose(1, 2) + in_cfg.ik_damping * torch.eye(6, device=dev), err.unsqueeze(-1))).squeeze(-1)
            if args_cli.insert_ik:  # 2026-10-31: 관절 한계 반영 - 한계 0.02 rad 안에서 바깥으로 미는 관절은 빼고 다시 풂 (팔꿈치 관절 3 이 1.6 에 걸려 내리기가 멈추던 것)
                qa_ = arm_prev
                blk = ((qa_ + dq > u._hi - 0.02) & (dq > 0)) | ((qa_ + dq < u._lo + 0.02) & (dq < 0))
                if blk.any():
                    Jb = Jf * (~blk).float().unsqueeze(1)
                    dq2 = (Jb.transpose(1, 2) @ torch.linalg.solve(Jb @ Jb.transpose(1, 2) + in_cfg.ik_damping * torch.eye(6, device=dev), err.unsqueeze(-1))).squeeze(-1)
                    dq = torch.where(blk.any(dim=1, keepdim=True), dq2, dq)
            tgt[m3] = (arm_prev + dq)[m3]
            grip[m3] = a_in[m3, 3]
            prev4[m3] = a_in[m3]
        if args_cli.release_open is not None:  # 2026-11-01: 넣기 단계부터 좁게 벌림 (집기 단계는 넓게)
            u.cfg.gripper_open = torch.where((phase >= 2)[:, None], torch.full((n, 1), args_cli.release_open, device=dev), torch.full((n, 1), go0, device=dev))
        if args_cli.debug_ik and m3.any() and k % 5 == 0:
            for i in [j for j in torch.nonzero(m3).flatten().tolist() if int(tube[j]) in (0, 1) and int(ik_stage[j]) == 0][:2]:
                dqi = (tgt[i] - q[i]).tolist()
                print(f"[IKdbg] t={k * u.step_dt:5.2f} env{i} 단계 {int(ik_stage[i])} 축 {s['dxy'][i].item() * 1000:5.1f} mm 깊이 {s['depth'][i].item() * 1000:+5.1f} mm "
                      f"축-퍽 xy mm {[round(v * 1000, 1) for v in (s['t'][i, :2] - s['pb'][i, :2]).tolist()]} 명령 dp mm {[round(v, 2) for v in a_in[i, :3].tolist()]} "
                      f"q {[round(v, 3) for v in q[i].tolist()]} 목표-q {[round(v, 4) for v in dqi]} 한계 j1 [{u._lo[0].item():.2f},{u._hi[0].item():.2f}]", flush=True)
        u.arm_target[:] = torch.clamp(torch.clamp(tgt, q - lead, q + lead), u._lo, u._hi)
        if not args_cli.no_safety:  # 안전 계층 (2026-10-31): 집기 실패 뒤 정책이 팔을 옆으로 크게 돌려 link4 가 몸체에 닿던 것 (70 ~ 454 N)
            j1c = th_init.clamp(min=-math.pi, max=math.pi)
            lo1, hi1 = j1c - 0.6, j1c + 0.6
            u.arm_target[:, 0] = torch.where(m0, torch.maximum(torch.minimum(u.arm_target[:, 0], hi1), lo1), u.arm_target[:, 0])
            u.arm_target[aborted] = abort_q[aborted]
            u.actions[aborted, -1] = 1.0
        u.arm_target[m4] = arm_prev[m4]
        if not args_cli.no_return:  # 복귀: 넣기 끝 (집게 위로 뺀 뒤) -> 접기 -> 관절 1 을 초기 각으로 -> 초기 자세
            go_ret = m4 & (ret_k < 0)
            ret_q0[go_ret] = q[go_ret]
            ret_k[go_ret] = 0
            mr = m4 & (ret_k >= 0)
            if mr.any():
                Nseg = max(1, int(args_cli.ret_s / u.step_dt))
                tuck_a = ret_q0.clone(); tuck_a[:, 1:4] = torch.tensor([TUCK], device=dev); tuck_a[:, 4] = 0.0
                tuck_b = tuck_a.clone(); tuck_b[:, 0] = q_init[:, 0]
                kk = ret_k.float().unsqueeze(1)
                def sm(x):
                    x = x.clamp(0.0, 1.0)
                    return 0.5 - 0.5 * torch.cos(math.pi * x)
                s1, s2, s3 = sm(kk / Nseg), sm((kk - Nseg) / Nseg), sm((kk - 2 * Nseg) / Nseg)
                qr = torch.where(kk < Nseg, ret_q0 * (1 - s1) + tuck_a * s1, torch.where(kk < 2 * Nseg, tuck_a * (1 - s2) + tuck_b * s2, tuck_b * (1 - s3) + q_init * s3))
                u.arm_target[mr] = qr[mr]
                ret_k[mr] += 1
        u.actions[:, -1] = torch.where(m4, torch.ones_like(grip), grip)
        for _ in range(cfg.decimation):
            u._apply_action()
            u.scene.write_data_to_sim()
            u.sim.step(render=args_cli.gui)
            u.scene.update(u.physics_dt)
        u.episode_length_buf[:] = 0  # 정렬·집기 환경의 시간 끝 리셋 막기 (이 스크립트가 직접 돌림)
        arm_prev = u.arm_target.clone()
        fa, ff = force("arm_body_contact"), force("finger_body_contact")
        f_arm_max = torch.maximum(f_arm_max, fa)
        f_fing_max = torch.maximum(f_fing_max, ff)
        body_steps += (fa > 1.0).float()
        ret_hit = torch.where(ret_k >= 0, torch.maximum(ret_hit, fa), ret_hit)
        _Ns = max(1, int(args_cli.ret_s / u.step_dt))
        ret_hit_seg = torch.where((ret_hit_seg < 0) & (ret_k >= 0) & (fa > 1.0), (ret_k // _Ns).clamp(max=2), ret_hit_seg)
        pi = phase.clamp(max=2)
        f_phase.scatter_reduce_(1, pi.unsqueeze(1), fa.unsqueeze(1), reduce="amax")
        newhit = (hit_phase < 0) & (fa > 1.0) & (phase < 3)
        if newhit.any():
            fb = torch.linalg.norm(u.scene["arm_body_contact"].data.normal_force_matrix_w.torch, dim=-1).amax(dim=2)  # (N, 몸)
            hit_body = torch.where(newhit, fb.argmax(dim=1), hit_body)
            hit_t = torch.where(newhit, torch.full_like(hit_t, (k + 1) * u.step_dt), hit_t)
            hit_q = torch.where(newhit[:, None], r.data.joint_pos.torch[:, u._arm], hit_q)
        hit_fmax = torch.where(phase < 3, torch.maximum(hit_fmax, fa), hit_fmax)
        hit_phase = torch.where((hit_phase < 0) & (fa > 1.0) & (phase < 3), phase, hit_phase)
        nb_touch |= (phase == 0) & u._contact("arm_nb_contact")
        if args_cli.occ > 0 and k == 2:
            place_occ()
        if args_cli.occ > 0:
            if occ_p0 is None and k >= 10:  # 0.3 s 자리 잡은 뒤
                occ_p0 = [u.scene[f"occ{j}"].data.root_pos_w.torch.clone() for j in range(3)]
            fro, fpo = force("robot_occ_contact"), force("puck_occ_contact")
            fro, fpo = (fro, fpo) if k >= 10 else (fro * 0, fpo * 0)
            fo = torch.maximum(fro, fpo)
            occ_fph.scatter_reduce_(1, phase.clamp(max=3).unsqueeze(1), fo.unsqueeze(1), reduce="amax")
            newo = (occ_first < 0) & (fo > 1.0)
            occ_first = torch.where(newo, torch.where(phase == 2, 4 + ik_stage, phase), occ_first)
            occ_src = torch.where(newo, (fpo > 1.0).long(), occ_src)
        if not args_cli.lie:
            toppled |= (phase == 0) & (u.puck_up() < 0.7)
        # 단계 전환
        s = state()
        if args_cli.insert_ik:
            m3n = phase == 2
            ik_cnt += 1
            okc = (s["dxy"] < 0.0015) & ((s["depth"] + 0.008).abs() < 0.003)
            to1 = m3n & (ik_stage == 0) & ((okc & (oh_over | (args_cli.overhead <= 0))) | (ik_cnt > (300 if args_cli.overhead > 0 else 150)))
            qn = r.data.joint_pos.torch[:, u._arm]
            wrist_lim = ((qn - u._lo).abs() < 0.05).any(dim=1) | ((u._hi - qn).abs() < 0.05).any(dim=1)
            # 2026-10-31: 손목 (관절 4) 이 한계 근처면 집게가 기울어 손가락이 컵 벽·판을 눌러 (24 ~ 62 N) 못 벌리고 못 뺌 -> 거기서 멈추고 놓음. 손가락이 10 N 넘게 눌려도
            ikd = args_cli.ik_depth_box if args_cli.lie else args_cli.ik_depth
            to2 = m3n & (ik_stage == 1) & ((s["depth"] > ikd) | (ik_cnt > 40) | ((s["depth"] > 0.0) & (wrist_lim | (ff > 10.0))))
            al_min = torch.where(m3n & (ik_stage == 0), torch.minimum(al_min, s["dxy"]), al_min)
            al_ok |= to1 & okc
            al_tmo |= to1 & ~okc
            al_end = torch.where(to1[:, None], torch.stack((s["dxy"], s["depth"]), dim=1), al_end)
            al_xy = torch.where(to1[:, None], s["t"][:, :2] - s["pb"][:, :2], al_xy)
            dn_ff = torch.where(m3n & (ik_stage == 1), torch.maximum(dn_ff, ff), dn_ff)
            dn_tmo |= to2 & ~(s["depth"] > ikd)
            dn_end = torch.where(to2[:, None], torch.stack((s["dxy"], s["depth"], s["u"][:, 2]), dim=1), dn_end)
            dn_q = torch.where(to2[:, None], r.data.joint_pos.torch[:, u._arm], dn_q)
            to3 = m3n & (ik_stage == 2) & (ik_cnt > 15)
            ik_stage[to1], ik_cnt[to1] = 1, 0
            ik_stage[to2], ik_cnt[to2] = 2, 0
            ik_stage[to3], ik_cnt[to3] = 3, 0
        lift = u.lift()
        ever_held |= s["held"] & (phase == 0)
        pk_min_d = torch.where(phase == 0, torch.minimum(pk_min_d, torch.linalg.norm(u.grasp_target_w() - u.grasp_w(), dim=1)), pk_min_d)
        pk_close |= (phase == 0) & (u.actions[:, -1] <= 0.0)
        if not args_cli.lie:
            pk_top_t = torch.where((pk_top_t < 0) & (phase == 0) & (u.puck_up() < 0.7), (k + 1) * u.step_dt, pk_top_t)
        if not args_cli.no_safety:
            if args_cli.lie:  # 누운 퍽: 원래 누워 있음 -> 넘어짐 대신 '들었다가 떨어뜨림' (세우기 전·후 쥠 잃음) 과 시간 끝
                ab = (phase == 0) & ~aborted & ((ever_held & ~s["held"]) | ((k + 1) * u.step_dt > args_cli.pick_timeout + 4.0))
            else:
                ab = (phase == 0) & ~aborted & ((u.puck_up() < 0.7) | ((k + 1) * u.step_dt > args_cli.pick_timeout))
            abort_q[ab] = r.data.joint_pos.torch[ab][:, u._arm]
            aborted |= ab
        released = (phase == 2) & (ik_stage >= 2) if args_cli.insert_ik else torch.zeros_like(ever_held)
        lost_now = (phase >= 1) & (phase <= 2) & ~s["held"] & ~released & (lost_phase < 0)
        lost_phase = torch.where(lost_now, phase, lost_phase)
        c0 = (phase == 0) & (lift > 0.03) & s["held"] & (u.puck_up() > 0.9)
        if args_cli.lie:
            fing_l = r.data.joint_pos.torch[:, u._gl[0]]
            hold_l = (fing_l > u.cfg.grasp_finger_range[0]) & (fing_l < u.cfg.grasp_finger_range[1]) & s["held"]
            lie_cnt = torch.where((phase == 0) & (lie_k < 0) & hold_l & (lift > u.cfg.lift_ok), lie_cnt + 1, torch.zeros_like(lie_cnt))
            go_s = (phase == 0) & (lie_k < 0) & (lie_cnt >= 9)
            if go_s.any():
                qn_ = r.data.joint_pos.torch[:, u._arm]
                th_a = torch.atan2(s["gb"][:, 1], s["gb"][:, 0] - st_cfg.joint1_x)
                lie_q0[go_s] = qn_[go_s]
                v_ = qn_.clone(); v_[:, 0] = th_a; v_[:, 1:4] = torch.tensor([[-0.7, 0.25, 0.2]], device=dev)
                t_ = v_.clone(); t_[:, 1:4] = torch.tensor([[0.30, 1.42, -1.72]], device=dev); t_[:, 4] = 0.0
                lie_via[go_s], lie_to[go_s] = v_[go_s], t_[go_s]
                lie_k[go_s] = 0
            c0 = (phase == 0) & stood & s["held"] & (u.puck_up() > 0.95)
        if args_cli.insert_ik:  # 통 근처면 손 IK 가 맞춤까지
            c1 = (phase == 1) & s["held"] & (s["dxy"] < args_cli.ik_gate_mm / 1000) & (s["depth"] > -args_cli.ik_gate_clear) & (s["depth"] < 0.005) & (s["u"][:, 2] > args_cli.ik_gate_up)
            if args_cli.overhead > 0:
                c1 = (phase == 1) & s["held"] & (s["dxy"] < args_cli.overhead) & (s["depth"] < 0.005) & (s["u"][:, 2] > args_cli.ik_gate_up)
        else:
            c1 = (phase == 1) & s["held"] & (s["dxy"] < args_cli.rl_gate_mm / 1000) & (s["depth"] > -args_cli.rl_gate_clear) & (s["depth"] < 0.002) & (s["u"][:, 2] > 0.985)
        # 끝 = 퍽이 통 축 6 mm 안·입구 아래 60 mm 넘게 + 집게 벌림 (집게를 6 cm 위로 빼는 조건은 새 팔 높이 한계 (통 위 약 4 cm) 로 못 채워 뺌)
        c2 = (phase == 2) & (s["depth"] > st_cfg.stored_depth) & (s["dxy"] < st_cfg.inside_xy + 0.002) & (s["fing"] > fing_open_thr)
        if args_cli.insert_ik:  # 2026-10-31: 집게를 열고 4 cm 위로 뺀 뒤에 '넣기 끝' (그다음 바로 복귀)
            c2 = c2 & (ik_stage >= 3) & (ik_cnt >= int(round(args_cli.ik_lift / 0.002)))
        go = torch.where(phase == 0, c0, torch.where(phase == 1, c1, c2))
        cnt = torch.where(go, cnt + 1, torch.zeros_like(cnt))
        need = torch.where(phase == 0, 15, torch.where(phase == 1, 3, 5))
        adv = go & (cnt >= need) & (phase < 3)
        ph_steps += 1
        if adv.any():
            phase[adv] += 1
            cnt[adv] = 0
            ph_steps[adv] = 0
            ik_stage[adv], ik_cnt[adv] = 0, 0
            oh_over[adv] = False
            t_enter[adv, phase[adv]] = (k + 1) * u.step_dt
            if collect:  # 집기 -> 옮기기 넘어가는 순간의 실제 상태를 모으고 그 대는 멈춤 (끝 단계)
                jp = r.data.joint_pos.torch
                pq = u.puck_quat_w()
                for i in torch.nonzero(adv & (phase == 1)).flatten().tolist():
                    rows.append({"cat": "H", "tube": -1, "q": [round(float(v), 5) for v in jp[i, u._arm].tolist()],
                                 "finger": [float(jp[i, u._gl[0]]), float(jp[i, u._gr[0]])],
                                 "puck_b": [round(float(v), 5) for v in s["pc"][i].tolist()], "puck_quat_b": [float(v) for v in pq[i].tolist()]})
                phase[adv & (phase == 1)] = 3
            for i in torch.nonzero(adv).flatten().tolist():
                print(f"[Chain] t={(k + 1) * u.step_dt:5.1f}s env{i} (통 {tube_names[int(tube[i])]}) -> {PHASES[int(phase[i])]}", flush=True)
        if k % 60 == 59:
            print(f"[Chain] t={(k + 1) * u.step_dt:5.1f}s 단계 {[int(v) + 1 for v in phase.tolist()][:16]} 퍽 들기 mm {[round(v * 1000) for v in lift.tolist()][:8]}"
                  f" 통 축 mm {[round(v * 1000, 1) for v in s['dxy'].tolist()][:8]} 입구 아래 mm {[round(v * 1000) for v in s['depth'].tolist()][:8]}", flush=True)
    if collect:
        json.dump({"note": "집기 (SidePick) 가 끝난 실제 상태 (Isaac): 팔 관절, 손가락, 퍽 중심 (base_link), 퍽 자세 (월드 = 루트 요 0, x,y,z,w)", "samples": rows},
                  open(collect, "w"))
        print(f"[Chain] 이어 받기 상태 {len(rows)} 개 -> {collect}", flush=True)
    s = state()
    done = phase == 3
    print(f"[Chain] ===== {args_cli.seconds:.0f} s 결과 ({n} 대, 목표 컵 {tube_names} 번갈아) =====", flush=True)
    print(f"[Chain] 1 집기 성공 {int((phase >= 1).sum())}/{n}, 2 통 위 맞춤 {int((phase >= 2).sum())}/{n}, 3 통 안 넣고 놓음 {int(done.sum())}/{n}", flush=True)
    stored = (s["depth"] > st_cfg.stored_depth) & (s["dxy"] < st_cfg.inside_xy + 0.002)
    for ti, nm in enumerate(tube_names):
        mt = tube == ti
        print(f"[Chain]   {nm} 통 목표 {int(mt.sum())} 대: 집기 {int(((phase >= 1) & mt).sum())}, 맞춤 단계 {int(((phase >= 2) & mt).sum())}, **최종 통 안 {int((stored & mt).sum())}**", flush=True)
    dropped = s["depth"] > 0.05
    print(f"[Chain]   **최종 성공 (컵 안 + 놓고 뺌) {int((stored & (phase >= 3)).sum())}/{n}** (컵 안에만 있음 {int((stored & (phase < 3)).sum())})", flush=True)
    print(f"[Chain]   최종: 통 안 {int(stored.sum())}/{n}, 퍽 떨어뜨림 (통 밖 낮은 곳) {int((dropped & ~stored).sum())}", flush=True)
    if done.any():
        print(f"[Chain] 끝까지 간 대의 단계 시작 시각 평균 s: 옮기기 {t_enter[done, 1].mean().item():.1f}, 넣기 {t_enter[done, 2].mean().item():.1f}, 끝 {t_enter[done, 3].mean().item():.1f}", flush=True)
    # ---- 대별 최초 실패 단계 분류 (2026-10-31) ----
    q = r.data.joint_pos.torch[:, u._arm]
    at_lim = ((q - u._lo).abs() < 0.03) | ((u._hi - q).abs() < 0.03)
    kinds = []
    for i in range(n):
        if bool(stored[i]) and int(phase[i]) >= 3:
            kinds.append("성공")
        elif bool(stored[i]):
            kinds.append("넣기: 넣었으나 못 놓음·못 뺌")
        elif int(phase[i]) == 0:
            if args_cli.lie and int(lie_k[i]) >= 0:
                kinds.append("누운 퍽: 세웠으나 못 넘어감 (직립·쥠)" if bool(stood[i]) else "누운 퍽: 세우는 중 놓침")
            else:
                kinds.append("집기: 목표 퍽 넘어뜨림" if bool(toppled[i]) else ("집기: 쥐었다 놓침·들기 못 함" if bool(ever_held[i]) else "집기: 못 쥠"))
        elif int(phase[i]) == 1:
            kinds.append("옮기기: 운반 중 놓침" if int(lost_phase[i]) == 1 else "옮기기: 컵 위 못 맞춤")
        else:
            if int(lost_phase[i]) == 2:
                kinds.append("넣기: 맞추다 놓침")
            elif args_cli.insert_ik and int(ik_stage[i]) >= 2:
                kinds.append("넣기: 축 맞춤 시간 끝 -> 내려 놓음" if bool(al_tmo[i]) else ("넣기: 맞춘 뒤 내리다 걸림 (깊이 못 감)" if bool(dn_tmo[i]) else "넣기: 깊이 갔으나 놓은 뒤 컵 밖"))
            else:
                kinds.append("넣기: 축 못 맞춤" + (" (관절 한계)" if bool(at_lim[i].any()) else ""))
    from collections import Counter
    print(f"[Chain] ===== 대별 최초 실패 단계 (64 대 중) =====", flush=True)
    for kk, v in Counter(kinds).most_common():
        per = Counter(tube_names[int(tube[i])] for i in range(n) if kinds[i] == kk)
        print(f"[Chain]   {kk}: {v} 대  ({', '.join(f'{a} {b}' for a, b in sorted(per.items()))})", flush=True)
    hp = Counter(PHASES[int(v)] for v in hit_phase.tolist() if v >= 0)
    if args_cli.occ > 0:
        from collections import Counter
        oname = {0: "집기", 1: "옮기기 (정책)", 3: "복귀", 4: "넣기 IK 맞춤·내려옴", 5: "넣기 IK 내리기", 6: "넣기 놓기", 7: "넣기 빼기"}
        # 끝에 꽂힌 퍽이 제 컵 축에서 벗어난 거리 (로봇 좌표, 로봇 몸이 처음에 움직여도 상관없게) + 기울기
        mdist = torch.zeros(n, device=dev)
        mtilt = torch.ones(n, device=dev)
        for j in range(3):
            pj = to_bl(u.scene[f"occ{j}"].data.root_pos_w.torch)
            cj = oth[:, j]
            mdist = torch.maximum(mdist, occ_on[:, j].float() * torch.linalg.norm(pj[:, :2] - tubes[cj, :2], dim=1))
            zj = vec_bl(quat_apply(u.scene[f"occ{j}"].data.root_quat_w.torch, ez))[:, 2].abs()
            mtilt = torch.where(occ_on[:, j], torch.minimum(mtilt, zj), mtilt)
        moved = (mdist > 0.003) | (mtilt < 0.98)
        print(f"[Chain] 꽂힌 퍽 끝 상태: 컵 축에서 벗어난 거리 mm 중앙값 {mdist.median().item() * 1000:.1f} · 최대 {mdist.max().item() * 1000:.1f}, 3 mm 넘게 벗어나거나 11° 넘게 기운 대 {int(moved.sum())}", flush=True)
        oc = Counter(f"{oname.get(int(a), a)} / {'쥔 퍽' if int(b) == 1 else '팔·손가락'}" for a, b in zip(occ_first.tolist(), occ_src.tolist()) if a >= 0)
        print(f"[Chain] 꽂힌 옆 퍽 (대당 {occ_on.sum(1).float().mean().item():.1f} 개): 1 N 넘게 닿은 대 {int((occ_first >= 0).sum())}/{n} (최대 {occ_fph.max().item():.1f} N), 컵에서 밀려나거나 기운 대 {int(moved.sum())} | 처음 닿은 단계 {dict(oc)} | 단계별 최대 N {[round(v, 1) for v in occ_fph.amax(0).tolist()]}", flush=True)
    print(f"[Chain] 팔-몸체 1 N 넘게 닿은 대 {int((hit_phase >= 0).sum())}/{n}, 처음 닿은 단계 {dict(hp)} | 단계별 최대 힘 N: 집기 {f_phase[:, 0].max().item():.0f}, 옮기기 {f_phase[:, 1].max().item():.0f}, 넣기 {f_phase[:, 2].max().item():.0f}", flush=True)
    for i in torch.nonzero(hit_phase >= 0).flatten().tolist():
        j1v = tb_init[i] - torch.tensor([st_cfg.joint1_x, 0.0], device=dev)
        print(f"[Chain]   팔-몸체 닿음 env{i} ({tube_names[int(tube[i])]}) 단계 {PHASES[int(hit_phase[i])]} {hit_t[i].item():.1f} s, 링크 {arm_sensor_names[int(hit_body[i])]}, 최대 {hit_fmax[i].item():.0f} N, "
              f"관절 {[round(v, 2) for v in hit_q[i].tolist()]}, 퍽 joint1 축 {j1v.norm().item():.3f} m · {math.degrees(math.atan2(j1v[1].item(), j1v[0].item())):+.0f}°, 최종 {kinds[i]}", flush=True)
    if not args_cli.no_safety:
        print(f"[Chain] 안전 계층: 집기 실패로 멈춘 대 {int(aborted.sum())}/{n} (목표 퍽 넘어짐 또는 {args_cli.pick_timeout:.0f} s 안에 못 집음)", flush=True)
    if args_cli.lie:
        print(f"[Chain] 누운 퍽: 쥐고 들어 세우기 시작 {int((lie_k >= 0).sum())}/{n}, 세우기 끝 {int(stood.sum())}, 세운 뒤 옮기기로 넘어감 {int((phase >= 1).sum())}", flush=True)
    print(f"[Chain] 집기 중 옆 퍽 닿은 대 {int(nb_touch.sum())}/{n}, 목표 퍽 넘어뜨린 대 {int(toppled.sum())}/{n}", flush=True)
    lim_names = [f"관절{j + 1}" for j in range(5)]
    if not args_cli.no_return:
        qe = (q - q_init).abs().amax(dim=1)
        Nseg = max(1, int(args_cli.ret_s / u.step_dt))
        done_ret = (ret_k >= 3 * Nseg)
        back = done_ret & (qe < 0.05)
        print(f"[Chain] ===== 복귀 (넣기 끝 -> 접기 -> 관절 1 -> 초기 자세) =====", flush=True)
        print(f"[Chain] 복귀 시작 {int((ret_k >= 0).sum())} 대, 끝까지 {int(done_ret.sum())}, **초기 자세 도착 (관절 차 < 0.05 rad) {int(back.sum())}**, "
              f"복귀 중 팔-몸체 1 N 넘음 {int((ret_hit > 1).sum())} (최대 {ret_hit.max().item():.1f} N), 복귀 뒤에도 퍽 컵 안 {int((stored & done_ret).sum())}/{int(done_ret.sum())}, "
              f"복귀 끝 관절 차 최대 {[round(v, 3) for v in qe[done_ret][:8].tolist()]}", flush=True)
        for i in torch.nonzero(ret_hit > 1.0).flatten().tolist():
            print(f"[Chain]   복귀 중 닿음 env{i} ({tube_names[int(tube[i])]}) 구간 {['접기', '관절1 돌리기', '펴기'][int(ret_hit_seg[i])] if int(ret_hit_seg[i]) >= 0 else '-'} 최대 {ret_hit[i].item():.1f} N", flush=True)
    for i in range(n):  # 2026-10-31 확인용: 컵 안인데 '넣기 끝' 으로 못 넘어간 대
        if bool(stored[i]) and int(phase[i]) == 2:
            jp = r.data.joint_pos.torch[i]
            print(f"[Chain]   (넣기 끝 못 넘어감) env{i} 집게 명령 {u.actions[i, -1].item():+.1f} 목표 {r.data.joint_pos_target.torch[i, u._gl[0]].item() if hasattr(r.data, 'joint_pos_target') else float('nan'):+.4f} 왼·오른 손가락 {jp[u._gl[0]].item():+.4f} {jp[u._gr[0]].item():+.4f} "
                  f"손가락-몸체 {force('finger_body_contact')[i].item():.1f} N 팔-몸체 {force('arm_body_contact')[i].item():.1f} N 관절 {[round(v, 3) for v in jp[u._arm].tolist()]} 팔 목표 {[round(v, 3) for v in u.arm_target[i].tolist()]}", flush=True)
            print(f"[Chain]   (넣기 끝 못 넘어감) env{i} {tube_names[int(tube[i])]} IK 단계 {int(ik_stage[i])} 스텝 {int(ik_cnt[i])} 손가락 {s['fing'][i].item():+.4f} 깊이 {s['depth'][i].item() * 1000:+.1f} 축 {s['dxy'][i].item() * 1000:.1f} 쥠(거리) {bool(s['held'][i])} 잡는 점 z {s['gb'][i, 2].item():.3f}", flush=True)
    print("[Chain] ----- 집기 실패 대 세부: 유형 | 잡는 점-목표 최소 | 집게 닫음 | 넘어진 시각 | 끝 퍽 직립·들기 | 옆 퍽 닿음 -----", flush=True)
    for i in range(n):
        if kinds[i].startswith("집기"):
            j1v = tb_init[i] - torch.tensor([st_cfg.joint1_x, 0.0], device=dev)
            rr_ = j1v.norm().item(); th_ = math.degrees(math.atan2(j1v[1].item(), j1v[0].item()))
            nbr_ = (nb_init[i] - torch.tensor([st_cfg.joint1_x, 0.0], device=dev)).norm().item()
            print(f"[Chain]   env{i:2d} 퍽 joint1 축 {rr_:.3f} m · 방향 {th_:+5.1f}° (옆 퍽 {nbr_:.3f} m)", flush=True)
            print(f"[Chain]   env{i:2d} {kinds[i]:18s} | {pk_min_d[i].item() * 1000:5.1f} mm | {'예' if bool(pk_close[i]) else '아니오'} | {pk_top_t[i].item():4.1f} s | "
                  f"{u.puck_up()[i].item():.2f} · {u.lift()[i].item() * 1000:+4.0f} mm | {'예' if bool(nb_touch[i]) else '-'}", flush=True)
    print("[Chain] ----- 내리기 끝 관절 (넣기 실패 대) -----", flush=True)
    for i in range(n):
        if kinds[i].startswith("넣기"):
            print(f"[Chain]   env{i:2d} {tube_names[int(tube[i])]} 관절 {[round(v, 3) for v in dn_q[i].tolist()]} 한계 근처 {[f'관절{j + 1}' for j in range(5) if abs(dn_q[i, j].item() - u._lo[j].item()) < 0.03 or abs(u._hi[j].item() - dn_q[i, j].item()) < 0.03]}", flush=True)
    print("[Chain] ----- 넣기 단계 (손 IK) 대별 세부: 맞춤 결과 | 맞춤 중 최소 축 | 맞춤 끝 축·깊이 (축-퍽 xy) | 내리기 결과 | 내리기 끝 축·깊이·직립 | 내리기 중 손가락-몸체 최대 -----", flush=True)
    for i in range(n):
        if int(phase[i]) >= 2:
            print(f"[Chain]   env{i:2d} {tube_names[int(tube[i])]} {kinds[i][:14]:14s} | 맞춤 {'조건' if bool(al_ok[i]) else ('시간끝' if bool(al_tmo[i]) else '-'):4s} | 최소 {al_min[i].item() * 1000:5.1f} mm | "
                  f"끝 {al_end[i, 0].item() * 1000:5.1f} mm · {al_end[i, 1].item() * 1000:+5.1f} mm ({al_xy[i, 0].item() * 1000:+5.1f}, {al_xy[i, 1].item() * 1000:+5.1f}) | "
                  f"내리기 {'시간끝' if bool(dn_tmo[i]) else '깊이 도달'} | {dn_end[i, 0].item() * 1000:5.1f} mm · {dn_end[i, 1].item() * 1000:+5.1f} mm · {dn_end[i, 2].item():.3f} | {dn_ff[i].item():5.1f} N", flush=True)
    for i in range(n):
        if kinds[i].startswith("넣기: 축 못 맞춤"):
            print(f"[Chain]   env{i} ({tube_names[int(tube[i])]}) 축 {s['dxy'][i].item() * 1000:.1f} mm, 입구 아래 {s['depth'][i].item() * 1000:+.0f} mm, 관절 {[round(v, 2) for v in q[i].tolist()]}, 한계 근처 {[lim_names[j] for j in range(5) if bool(at_lim[i, j])]}", flush=True)
    print(f"[Chain] 퍽 최종 입구 아래 mm {[round(v * 1000) for v in s['depth'].tolist()][:32]}", flush=True)
    print(f"[Chain] 팔 링크-몸체 최대 힘 N {[round(v, 1) for v in f_arm_max.tolist()][:32]} (1 N 넘은 스텝 합 {int(body_steps.sum())}) | 손가락-몸체 최대 N {[round(v, 1) for v in f_fing_max.tolist()][:32]}", flush=True)
    if args_cli.gui:
        while simulation_app.is_running():
            u.sim.step(render=True)
    env.close()


if __name__ == "__main__":
    try:
        main()
    except Exception:  # simulation_app.close() 가 프로세스를 먼저 끝내 오류가 안 보이던 것
        import traceback
        traceback.print_exc()
        sys.stdout.flush()
        raise
    finally:
        simulation_app.close()
