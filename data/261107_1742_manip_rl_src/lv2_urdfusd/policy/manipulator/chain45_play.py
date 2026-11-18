"""45° 집기 -> B안 팔레트 12 칸 놓기 연결 실행 (2026-11-03, 세션인계 0b · 0c).

  상자 위 세운 원기둥 --[1 집기 SP-45 정책]--> 쥐고 3 cm 듦 (0.5 s)
    --[2 운반: 관절 경로]--> 앞 운반 자세 (q_front, 집게 45°) -> 관절1 을 칸 방향으로 (왼쪽 칸 +, 오른쪽 칸 −) -> 칸 위 hover (벽 위 12 mm)
    --[3 놓기: 손 IK]--> 퍽 바닥을 칸 축에 맞추고 벽 위 + release_gap (기본 5 mm, 살짝 위에서 놓기) 또는 칸 바닥 + 1 mm (--place floor)
    --[4 벌림]--> 손가락 FINGER_RELEASED (37.6 mm, 옆 칸 퍽 안 건드림)
    --[5 복귀: 관절 경로]--> hover -> 관절1 앞으로 -> q_front -> 집기 시작 자세 (다음 퍽)

관절 경로 = 팀 `pallet_place_traj.json` (팀 `pallet_traj.py` 가 새 URDF 로 만든 칸 12 개 경유점, 팀 `pallet_place_check.py` 로 Isaac 24/24 확인).
경로를 따를 때 처짐 보정 (팀 check 와 같음): 관절 목표 = 계획 + bias, bias += ki · dt · (계획 − 측정). 팔 강성 18.88 이라 쥔 팔이 6 ~ 13 mm 처짐.
칸 좌표 = 같은 json 의 slots (base_link). 칸 순서 = 채우는 순서 (바깥 열부터). --occ p: 목표 칸보다 먼저 채우는 칸에 퍽이 이미 있을 확률.
손 IK 놓기의 퍽 바닥 위치는 시뮬레이션 참값 (실물은 쥔 위치 가정 또는 카메라, 정책_학습기록 8 절).

    ./run_rl.sh chain45_play 4                 # 창으로 (대 4 개, 칸 0 ~ 3)
    ./run_rl.sh chain45_eval 48 --occ 1.0      # 창 없이, 12 칸 x 4 번, 앞 칸 다 참
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--num_envs", type=int, default=4)
parser.add_argument("--gui", action="store_true")
parser.add_argument("--seconds", type=float, default=40.0)
parser.add_argument("--render_every", type=int, default=1, help="창 (--gui) 에서 제어 스텝 (1/30 s) 몇 번마다 한 번 그림. 1 = 제어 스텝마다 (이전: 물리 스텝마다 = 4 배 느림), 2 ~ 4 = 그만큼 빨리 감기")
parser.add_argument("--sidepick", default=None, help="집기 정책 (기본 policies/sidepick45_latest.pt)")
parser.add_argument("--traj", default=None, help="놓기 궤적 (기본 pallet_place_traj.json = 팀 저장소 policy/manipulator 의 것을 복사)")
parser.add_argument("--slots", default="all", help="목표 칸 '열:칸,...' (예 3:1,1:0). all = 채우는 순서대로 대마다 번갈아 (대 i -> i % 12)")
parser.add_argument("--fill_order", default="team", choices=("team", "lr", "rl"),
                    help="2026-11-05 사용자 요청 (위 -> 아래, 왼 -> 오): 칸 채우는 순서. team = 팀 json 순서 (3 열 -> 1 열, 열 안 가운데 1 · 2 -> 바깥 0 · 3). "
                         "lr = 3 열 (가장 높고 먼 열) -> 1 열, 열 안 칸 0 (base +y = 로봇 앞 보고 왼쪽) -> 3. rl = 열 안 칸 3 -> 0")
parser.add_argument("--occ", type=float, default=0.0, help="목표 칸보다 먼저 채우는 칸에 퍽이 이미 앉아 있을 확률 (1.0 = 다 참). 0 = 빈 팔레트")
parser.add_argument("--place", default="above", choices=("above", "floor"),
                    help="above (기본, 2026-11-03 사용자 결정): 퍽 바닥 = 칸 벽 윗면 + release_gap 에서 놓음. floor: 팀 궤적처럼 칸 바닥 + 1 mm 까지 내려 놓음")
parser.add_argument("--release_gap", type=float, default=None, help="[m] above 모드에서 놓는 높이 (퍽 바닥 - 벽 윗면). 2026-11-03: +5 mm 는 퍽을 수직으로 세워 놓아도 떨어지며 넘어짐 5 ~ 6/11 -> −10 mm (칸 입구 안 10 mm, 손가락 밑은 벽 위) 10/10")
parser.add_argument("--release_open", type=float, default=None, help="[m] 놓을 때·복귀 손가락 목표 (기본 FINGER_RELEASED −0.002 = 37.6 mm, 옆 칸 퍽 안 건드림). 벌림 [mm] = 41.6 + 2000 x 값")
parser.add_argument("--speed", type=float, default=0.8, help="[rad/s] 쥔 채 관절 경로 속도 (팀 check 기본)")
parser.add_argument("--row_speed", default="3:0.4", help="[rad/s] 열별 쥔 구간 속도 '열:속도,...' (팀 check: 3열 0.8 이면 상자 퍽이 벽에 걸림)")
parser.add_argument("--empty_speed", type=float, default=0.8, help="[rad/s] 놓은 뒤 빈 집게 속도")
parser.add_argument("--ki", type=float, default=4.0, help="[1/s] 처짐 보정 적분 (팀 check 와 같음). 0 = 끔")
parser.add_argument("--bias_max", type=float, default=0.15)
parser.add_argument("--ik_step", type=float, default=0.002, help="[m / 스텝] 놓기 손 IK 이동 한도")
parser.add_argument("--upright", action=argparse.BooleanOptionalAction, default=True,
                    help="놓기 IK 에서 집게 피치를 조금씩 바꿔 쥔 퍽을 수직으로 세운 뒤 놓음 (2026-11-03: 운반 뒤 쥔 퍽이 4 ~ 12° 기울어 벽 위에서 놓으면 6/12 넘어짐)")
parser.add_argument("--est_meas", type=float, default=-1.0,
                    help="2026-11-06 칸에 못 앉음 분석: >= 0 이면 --est 가정 대신 집기 끝에 손 안 퍽 위치를 '잼' (참값 + 축마다 이 [mm] 표준편차 잡음, 축 기울기도 참값) = 손목 카메라로 재는 경우. −1 = 예전 가정 (잡는 점 − 15 mm 수직)")
parser.add_argument("--est_z", default="grasp", choices=("grasp", "box"),
                    help="2026-11-06 --est 손 안 퍽 높이: grasp = 집기 끝 잡는 점 − 15 mm (예전), box = 손가락을 닫는 순간 (퍽이 아직 상자 위) 퍽 중심 높이 = 상자 윗면 + 30 mm 로 잼 "
                         "(카메라 없이 상자 높이 70 mm 로 앎, 수평은 잡는 점 그대로). 실물: '들기 전 집게 높이 변화 2 mm 안' 으로 판정")
parser.add_argument("--perc_bias", type=float, default=None,
                    help="[mm] 2026-11-06 집기 전 퍽 인식 좌표 치우침 (에피소드마다 ±이 값 균등). 기본 = 학습 값 4 mm. 인식 정확도가 놓기에 주는 영향 확인용")
parser.add_argument("--est", action="store_true",
                    help="놓기 IK 가 퍽 참값 대신 추정값을 씀 (실물과 같게): 집기 끝 순간 퍽이 수직 · 중심 = 잡는 점 − 15 mm 라고 가정하고 그 뒤 집게에 붙어 움직인다고 봄")
parser.add_argument("--grip_close", type=float, default=None, help="쥐는 손가락 목표 (기본 팀 값 FINGER_CLOSE −0.011, 팀 check 는 −0.0075: 두 손가락 힘이 같아 퍽이 가운데)")
parser.add_argument("--lie_roll", action="store_true",
                    help="2026-11-04 사용자 제안: 로봇 쪽을 향해 누운 직육면체를 45° 로 쥠 (정책 lieroll45) -> 5 cm 더 들기 -> 손목 롤 180° (퍽 세로, 집게 45° 그대로) -> 원기둥 45° 경로로 12 칸. "
                         "시뮬레이션 joint5 한계를 ±3.3 rad 로 늘림 (실물 EEPROM ±160.4° -> ±185° 필요)")
parser.add_argument("--roll_demo", action="store_true",
                    help="2026-11-04 사용자 B 동작 그대로 보기: 누운 퍽 위에서 롤 맞춰 쥐고 들기 -> 손목 롤 (joint5) 만 −90° 돌리고 멈춤 (손목 숙이기 없음). 퍽 긴 축 기울기를 출력")
parser.add_argument("--lie_regrasp", action="store_true",
                    help="2026-11-04 사용자 요청 (A): 누운 직육면체를 위에서 롤 맞춤 확인 뒤 끝 쪽 쥐기 -> 손 안에서 세워 상자 위 빈 자리에 세워 내려놓기 -> 뒤로 빼고 시작 자세 -> "
                         "학습된 45° 집기 정책 (세운 직육면체) 으로 다시 집기 -> 원기둥과 같은 45° 경로로 12 칸 (넣을 때 관절 3 38 ~ 58°)")
parser.add_argument("--lie_yaw", default="any", choices=("any", "radial", "side"),
                    help="--lie_regrasp 누운 직육면체 방향: any (무작위), radial (세로 = 긴 축이 로봇 쪽 ±10°), side (가로 = 긴 축이 옆 ±10°)")
parser.add_argument("--lie_nb", default="orig", choices=("orig", "tan"),
                    help="2026-11-05 원인 확인: --lie_regrasp 옆 퍽 자리. orig = 누운 퍽 접선 7.5 cm (다시 집을 때 옆 퍽이 세운 퍽 안쪽 15 cm = 학습 때 못 본 자리), "
                         "tan = 세워 놓을 자리 접선 8 cm (학습 배치와 비슷)")
parser.add_argument("--regrasp_by", default="policy", choices=("script", "policy"),
                    help="A 에서 세워 놓은 퍽 다시 집기: script (기본) = 45° 로 퍽 앞 5 cm 에서 맞춤 확인 -> 곧게 들어가 쥐고 4 cm 들기 (놓은 자리 · 방향을 앎). policy = 45° 집기 정책 (16 대 중 7 대 넘어뜨림)")
parser.add_argument("--box_prob", type=float, default=0.0, help="2026-11-04 (세운 퍽): 집을 퍽이 세운 직육면체일 확률 (나머지 원기둥). 롤 없이 원기둥처럼 45° 로 쥠, 칸 (정사각) 에 면 맞춰 넣기")
parser.add_argument("--sequence", type=int, default=1,
                    help="2026-11-04 연속 채우기: 한 대가 퍽 N 개를 차례로 집기 -> 칸 (채우는 순서) 에 놓기 -> 복귀. 놓은 퍽은 칸에 그대로 두고 (치워 둔 퍽으로 바꿔 앉힘) 새 퍽을 상자 위 처음 자리에 놓음. 1 = 예전 (대마다 한 번)")
parser.add_argument("--per_box", type=int, default=1, choices=(1, 2),
                    help="2026-11-05 상자당 2 개: 대마다 짝 칸 (채우는 순서 2i, 2i+1) 에 상자 위 두 퍽을 차례로 (가까운 퍽 -> 넣고 복귀 -> 다른 퍽). "
                         "두 번째 퍽이 관절1 축 0.365 m 밖이면 차체를 앞으로 조금 움직인 것처럼 상자째 옮김. 원기둥 · 세운 직육면체만 (누운 퍽 A 는 아직)")
parser.add_argument("--regrasp_tilt", type=float, default=5.0,
                    help="[°] 2026-11-04 (원기둥): 집기 끝에 쥔 퍽 기울기가 이보다 크면 상자 위 원래 자리에 내려놓고 다시 집음 (실물은 손목 카메라로 재야 함, 시뮬레이션은 참값). 0 = 끔")
parser.add_argument("--regrasp_max", type=int, default=2, help="다시 집기 최대 횟수")
parser.add_argument("--pick_j3_max", type=float, default=1.3,
                    help="[rad] 2026-11-04: 집기 중 관절 3 목표 상한. SP-45b 연결에서 못 쥔 대가 관절 3 1.54 ~ 1.58 로 팔을 접다 몸체에 809 N (정상 집기 자세는 관절 3 −1.4 ~ −0.8)")
parser.add_argument("--j3_max_deg", type=float, default=None,
                    help="[°] 관절 3 상한을 이 값으로 늘림 (시뮬레이션 관절 한계 · IK 한계 같이). 실물 EEPROM 한계 91.7° (3091 tick), 사용자: 기계적으로는 몇 도 더 됨 (2026-11-04). 실물에 쓰려면 EEPROM 도 같이 바꿔야 함")
parser.add_argument("--lie", action="store_true",
                    help="2026-11-04 누운 직육면체 B안: LP-8 로 위에서 쥠 (긴 축 바깥 끝 쪽으로 lie_dx 옮겨) -> 손 안에서 세움 (집게 수평) -> lie_b_traj.json 경로로 바깥 열 4 칸")
parser.add_argument("--lie_dx", type=float, default=0.010, help="[m] 누운 퍽 쥐는 점을 긴 축 방향으로 가운데에서 옮기는 거리 (세운 뒤 위쪽 끝 쪽). B안 IK: +10 mm 이상이어야 바깥 열에 닿음")
parser.add_argument("--hang_d", type=float, default=0.020, help="[m] hang: 잡는 점을 퍽 가운데에서 긴 축 따라 끝 쪽으로 (매달렸을 때 퍽 중심이 잡는 점 아래 이만큼)")
parser.add_argument("--hang_gz", type=float, default=0.008, help="[m] hang: 잡는 점 높이 = 누운 퍽 축 + 이것 (손끝이 상자 위)")
parser.add_argument("--hang_grip", type=float, default=-0.0061, help="hang: 매달리는 동안 손가락 목표 (30 mm 퍽 = −0.0058, 작을수록 세게). 매달려 45° 로 숙인 뒤 FINGER_CLOSE 로 세게 쥠")
parser.add_argument("--lie_pick", default="script", choices=("script", "policy", "hang"),
                    help="누운 퍽 집기. hang (2026-11-04): 사진처럼 위에서 끝 쪽을 약하게 쥐고 들어 퍽이 손가락 축으로 돌아 매달리게 -> 롤을 원기둥 집기와 같게 -> 45° 로 숙임 -> 세게 쥠 -> 원기둥 45° 경로로 12 칸. script (2026-11-04 사용자 요청, 기본): 퍽 위 6 cm 에서 위치 · 손목 롤 (긴 축) 을 맞춰 확인 -> 곧게 내려 쥠 -> 4 cm 들기 -> 바로 운반 자세 (세우기 경유 자세 없음). policy: LP-8 + 경유 자세로 세우기 (이전)")
parser.add_argument("--debug_lie", action="store_true")
parser.add_argument("--lie_r", default=None, help="누운 퍽 중심의 관절1 축 거리 범위 'lo,hi' [m] (LiePick 기본 0.185,0.235). 위에서 수직으로 쥐는 점은 약 0.228 m 까지만 닿음")
parser.add_argument("--stand_s", type=float, default=1.5, help="[s] 누운 퍽 세우기 구간 (들기 -> 경유 / 경유 -> 세움) 하나 시간")
parser.add_argument("--pitch_lim", type=float, default=None, help="[°] 놓기 IK 퍽 수직 맞춤 때 집게 피치를 기본에서 바꿀 수 있는 한도 (기본 45° 판 20, --lie 10)")
parser.add_argument("--puck_mu", type=float, default=0.5, help="퍽 마찰 (min 결합). 팀 check 와 같은 0.5 (팀 값 1.0 이면 1 mm 틈 칸에 끼임). 0 = 집기 환경 기본 그대로")
parser.add_argument("--pick_timeout", type=float, default=8.0)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
LIEROLL = args_cli.lie_roll  # 누운 직육면체 45° 쥐기 + 롤 180°
if LIEROLL:
    args_cli.lie = False
    args_cli.regrasp_tilt = 0.0
REGRASP = args_cli.lie_regrasp  # 누운 직육면체 A: 세워 내려놓고 45° 정책으로 다시 집기 (환경 = 45° 집기 환경, 목표 = 직육면체)
if REGRASP:
    args_cli.lie = False
    args_cli.box_prob = 1.0
    if args_cli.regrasp_by == "script":
        args_cli.regrasp_tilt = 0.0  # 각본으로 다시 집으면 기울기 확인 · 정책 다시 집기 안 씀
HANG = args_cli.lie and args_cli.lie_pick == "hang"  # 누운 퍽을 매달아 45° 로 바꿔 원기둥 경로 사용
if args_cli.release_gap is None:  # 2026-11-04: 원기둥 45° = 칸 입구 안 10 mm, 누운 퍽 B안 (수평) = 벽 높이 (팔꿈치 한계 여유 때문에 더 못 내림)
    args_cli.release_gap = -0.005 if (args_cli.lie and not HANG) else -0.010  # 누운 퍽 B: 관절 3 95° 일 때 입구 안 5 mm (2026-11-04 15/16)
if args_cli.lie and args_cli.pitch_lim is None:
    args_cli.pitch_lim = 20.0 if HANG else 5.0
if args_cli.lie and not HANG:  # 2026-11-04 누운 퍽 B 기본: 관절 3 상한 95° (사용자: 기계적으로 몇 도 더 됨), 쥐기 −0.0075 (두 손가락 힘 같게 -> 접선 치우침 3 ~ 4 mm 없앰)
    if args_cli.j3_max_deg is None:
        args_cli.j3_max_deg = 95.0
    if args_cli.grip_close is None:
        args_cli.grip_close = -0.0075
if args_cli.lie and args_cli.lie_r is None:  # 위에서 수직으로 쥐는 점은 관절1 축 약 0.228 m 까지 -> 퍽 중심 0.205 m 안 (주행 조건)
    args_cli.lie_r = "0.185,0.205"
args_cli.headless = not args_cli.gui
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import gymnasium as gym  # noqa: E402
import torch  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
import waffle_isaac  # noqa: E402,F401
from isaaclab.envs import ViewerCfg  # noqa: E402
from isaaclab.sensors import ContactSensorCfg  # noqa: E402
from isaaclab.utils.math import quat_apply, quat_apply_inverse, quat_box_minus, quat_from_euler_xyz, quat_from_matrix  # noqa: E402
from waffle_isaac.dice2_cfg import BOX_H, PUCK_H, puck_cfg  # noqa: E402
from waffle_isaac.robot_cfg import ARM_LINKS_REGEX, BODY_LINK, FINGER_LINKS_REGEX, FINGER_RELEASED  # noqa: E402
from waffle_isaac.liepick_env_cfg import LIE_READY, WaffleLiePickEnvCfg  # noqa: E402
from waffle_isaac.sidepick_env_cfg import READY_POSE_45, WaffleLieRoll45EnvCfg, WaffleSidePick45EnvCfg  # noqa: E402
from isaaclab.utils.math import quat_mul  # noqa: E402

PK = "puckT_box" if (args_cli.lie or LIEROLL) else "puckT_cyl"  # 목표 퍽 (scene 이름)
PK_PRIM = "{ENV_REGEX_NS}/PuckT_box" if (args_cli.lie or LIEROLL) else ("{ENV_REGEX_NS}/PuckT_(cyl|box)" if args_cli.box_prob > 0 else "{ENV_REGEX_NS}/PuckT_cyl")

PHASES = ("1 집기", "2 운반", "3 놓기 IK", "4 벌림", "5 복귀", "끝")
BODY = "{ENV_REGEX_NS}/Robot/" + BODY_LINK
N_OCC = 11  # 목표 칸 앞에 채워져 있을 수 있는 칸 수 (최대)
TRAJ = json.load(open(args_cli.traj or os.path.join(HERE, "lie_b_traj.json" if (args_cli.lie and not HANG) else "pallet_place_traj.json"), encoding="utf-8"))
SLOTS = TRAJ["slots"]  # 채우는 순서
if args_cli.fill_order != "team":  # 열 높은 쪽 (3, 바닥 0.136 m · 관절1 축 0.191 m) 부터, 열 안은 칸 번호 순 (lr) / 거꾸로 (rl)
    SLOTS = sorted(SLOTS, key=lambda s_: (-s_["row"], s_["index"] if args_cli.fill_order == "lr" else -s_["index"]))


def main():
    p_sp = args_cli.sidepick or os.path.join(HERE, "policies/liepick_LP8.pt" if args_cli.lie else
                                             ("policies/lieroll45_latest.pt" if (LIEROLL and os.path.exists(os.path.join(HERE, "policies/lieroll45_latest.pt"))) else
                                              ("policies/sidepick45box_latest.pt" if (REGRASP and os.path.exists(os.path.join(HERE, "policies/sidepick45box_latest.pt"))) else "policies/sidepick45_latest.pt")))
    cfg = WaffleLiePickEnvCfg() if args_cli.lie else (WaffleLieRoll45EnvCfg() if LIEROLL else WaffleSidePick45EnvCfg())
    cfg.scene.num_envs = args_cli.num_envs
    cfg.box_shape_prob = 0.0 if args_cli.lie else (1.0 if LIEROLL else args_cli.box_prob)
    cfg.near_start_prob = 0.0
    if args_cli.perc_bias is not None:  # 2026-11-06: 집기 전 퍽 인식 치우침 (에피소드마다 ±이 값, 학습 기본 4 mm)
        cfg.perc_bias = args_cli.perc_bias / 1000.0
    cfg.dock_second_prob = 0.0  # 연결은 늘 첫 퍽부터 (두 번째는 --per_box 2 가 바꿔 줌)
    cfg.episode_length_s = args_cli.seconds + 5.0
    if args_cli.lie and args_cli.lie_r:
        cfg.r_range = tuple(float(v) for v in args_cli.lie_r.split(","))
    if args_cli.grip_close is not None:
        cfg.gripper_closed = args_cli.grip_close
    cfg.viewer = ViewerCfg(eye=(0.55, -0.75, 0.70), lookat=(0.05, 0.0, 0.15), origin_type="env", env_index=0)
    col = sim_utils.CollisionPropertiesCfg(contact_offset=0.002, rest_offset=0.0)  # 칸 틈 1 mm (팀 check 와 같음)
    mat = (sim_utils.RigidBodyMaterialCfg(static_friction=args_cli.puck_mu, dynamic_friction=args_cli.puck_mu, friction_combine_mode="min")
           if args_cli.puck_mu > 0 else None)

    def puck_sp(sp):
        sp = sp.replace(collision_props=col, activate_contact_sensors=True)
        return sp.replace(physics_material=mat) if mat is not None else sp

    cfg.scene.robot = cfg.scene.robot.replace(spawn=cfg.scene.robot.spawn.replace(collision_props=col))
    for k_ in ((PK,) if (args_cli.lie or args_cli.box_prob <= 0) else ("puckT_cyl", "puckT_box")):
        setattr(cfg.scene, k_, getattr(cfg.scene, k_).replace(spawn=puck_sp(getattr(cfg.scene, k_).spawn)))
    occ_paths = [f"{{ENV_REGEX_NS}}/PuckOcc{j}" for j in range(N_OCC)]
    for j in range(N_OCC):  # 치워 두는 곳 = 환경 사이 (y 0.75) 바닥
        oc = puck_cfg(f"Occ{j}", "cyl", (0.55, 0.55, 0.6), (-0.55 + 0.1 * j, 0.75))
        setattr(cfg.scene, f"occ{j}", oc.replace(spawn=puck_sp(oc.spawn)))
    touch = dict(update_period=0.0, history_length=1)
    cfg.scene.arm_body_contact = ContactSensorCfg(prim_path="{ENV_REGEX_NS}/Robot/" + ARM_LINKS_REGEX, filter_prim_paths_expr=[BODY], **touch)
    cfg.scene.finger_body_contact = ContactSensorCfg(prim_path="{ENV_REGEX_NS}/Robot/" + FINGER_LINKS_REGEX, filter_prim_paths_expr=[BODY], **touch)
    cfg.scene.puck_body_contact = ContactSensorCfg(prim_path=PK_PRIM, filter_prim_paths_expr=[BODY], **touch)
    cfg.scene.robot_occ_contact = ContactSensorCfg(prim_path="{ENV_REGEX_NS}/Robot/(link[2-5]|gripper_roll_link|gripper_left_link|gripper_right_link)",
                                                   filter_prim_paths_expr=occ_paths, **touch)
    cfg.scene.puck_occ_contact = ContactSensorCfg(prim_path=PK_PRIM, filter_prim_paths_expr=occ_paths, **touch)
    env = gym.make("Waffle-LiePick-Direct" if args_cli.lie else ("Waffle-LieRoll45-Direct" if LIEROLL else "Waffle-SidePick45-Direct"), cfg=cfg)
    u = env.unwrapped
    r, n, dev = u.robot, u.num_envs, u.device
    dt = u.step_dt
    pol = torch.jit.load(p_sp, map_location=dev).eval()
    print(f"[Chain45] 놓기 IK 퍽 위치: {'추정 (집기 끝 수직 · 잡는 점 − 15 mm 가정)' if args_cli.est else '참값'}, 쥐기 손가락 {cfg.gripper_closed}", flush=True)
    print(f"[Chain45] 집기 정책 {p_sp}\n[Chain45] 놓기 궤적 {args_cli.traj or 'pallet_place_traj.json'} (칸 {len(SLOTS)}), 놓기 {args_cli.place}"
          f"{' 벽 위 ' + str(round(args_cli.release_gap * 1000)) + ' mm' if args_cli.place == 'above' else ' 바닥 + 1 mm'}, 앞 칸 참 확률 {args_cli.occ}, 퍽 마찰 {args_cli.puck_mu}", flush=True)
    obs, _ = env.reset()
    if LIEROLL:  # joint5 한계 ±3.3 rad (롤 180°)
        j5 = u._arm[4]
        lim5 = r.data.joint_pos_limits.torch.clone()
        lim5[:, j5, 0], lim5[:, j5, 1] = -3.3, 3.3
        r.write_joint_position_limit_to_sim_index(limits=lim5[:, [j5]], joint_ids=[j5])
        u._lo[4], u._hi[4] = -3.3, 3.3
        print("[Chain45] joint5 한계 ±3.3 rad (±189°) 로 늘림 (실물 EEPROM ±160.4°)", flush=True)
    if args_cli.j3_max_deg is not None:  # 관절 3 상한 늘림 (시뮬레이션 관절 · 소프트 한계 · 환경 클램프)
        j3 = u._arm[2]
        lim = r.data.joint_pos_limits.torch.clone()
        lim[:, j3, 1] = math.radians(args_cli.j3_max_deg)
        r.write_joint_position_limit_to_sim_index(limits=lim[:, [j3]], joint_ids=[j3])
        u._hi[2] = math.radians(args_cli.j3_max_deg)
        print(f"[Chain45] 관절 3 상한 {args_cli.j3_max_deg}° 로 늘림 (실물 EEPROM 91.7°)", flush=True)

    # ---- 칸·경로 ----
    _place_lying_pending = REGRASP
    key = {f"{s['row']}:{s['index']}": k for k, s in enumerate(SLOTS)}
    if args_cli.slots == "all":
        slot = torch.tensor([i % len(SLOTS) for i in range(n)], device=dev)
    else:
        lst = [key[x.strip()] for x in args_cli.slots.split(",")]
        slot = torch.tensor([lst[i % len(lst)] for i in range(n)], device=dev)
    sxyz = torch.tensor([[s["x"], s["y"], s["floor_z"], s["wall_top_z"]] for s in SLOTS], device=dev)
    rs = dict((int(a), float(b)) for a, b in (x.split(":") for x in args_cli.row_speed.split(",") if x))
    held_speed = torch.tensor([rs.get(s["row"], args_cli.speed) for s in SLOTS], device=dev)
    q_ready = torch.tensor([[(LIE_READY if args_cli.lie else READY_POSE_45)[j] for j in ("joint1", "joint2", "joint3", "joint4", "joint5")]], device=dev)

    def seg(s, name):
        return next(g for g in s["segments"] if g["name"] == name)["q"]

    def pad(rows):
        L = max(len(x) for x in rows)
        t = torch.tensor([x + [x[-1]] * (L - len(x)) for x in rows], device=dev)
        return t, torch.tensor([len(x) for x in rows], device=dev)

    if args_cli.lie and not HANG:  # B안: 앞 피벗 -> 관절1 회전 -> 칸 위 (퍽 바닥 z_transit), 복귀는 거꾸로 -> 누운 퍽 집기 시작 자세
        path_go, len_go = pad([[s["q_pivot"]] + s["rotate"] + s["carry"] for s in SLOTS])
        path_bk, len_bk = pad([[s["carry"][-1]] + s["carry"][::-1] + s["rotate"][::-1] + [s["q_pivot"], q_ready[0].tolist()] for s in SLOTS])
    else:
        path_go, len_go = pad([[s["q_front"]] + seg(s, "rotate") + seg(s, "carry") for s in SLOTS])  # 운반 (쥔 채)
    back = []
    for s in ([] if (args_cli.lie and not HANG) else SLOTS):
        head = seg(s, "retract") if args_cli.place == "floor" else [seg(s, "carry")[-1]]  # above: 놓은 높이에서 hover 로 바로
        back.append(head + seg(s, "return") + seg(s, "rotate_back") + [s["q_front"], q_ready[0].tolist()])
    if not args_cli.lie or HANG:
        path_bk, len_bk = pad(back)
    if args_cli.place == "floor":
        z_rel = sxyz[slot, 2] + 0.001
    else:
        z_rel = sxyz[slot, 3] + args_cli.release_gap
    tgt_pb = torch.stack((sxyz[slot, 0], sxyz[slot, 1], z_rel), dim=1)  # 놓는 순간 퍽 바닥 목표 (base_link)
    PB = args_cli.per_box
    assert PB == 1 or not (args_cli.lie or REGRASP or LIEROLL), "--per_box 2 는 원기둥 · 세운 직육면체만 (누운 퍽 A 는 아직)"
    SEQ = 2 if PB == 2 else max(1, args_cli.sequence)
    if PB == 2:  # 대 i -> 짝 칸 (2 (i % 6), 2 (i % 6) + 1), 짝 칸 앞 칸은 --occ 확률로 참
        slot = torch.tensor([2 * (i % (len(SLOTS) // 2)) for i in range(n)], device=dev)
        z0_ = (sxyz[slot, 2] + 0.001) if args_cli.place == "floor" else (sxyz[slot, 3] + args_cli.release_gap)
        tgt_pb = torch.stack((sxyz[slot, 0], sxyz[slot, 1], z0_), dim=1)
    elif SEQ > 1:  # 연속 채우기: 모든 대가 채우는 순서 처음 칸부터
        slot = torch.zeros(n, dtype=torch.long, device=dev)
        z0_ = (sxyz[slot, 2] + 0.001) if args_cli.place == "floor" else (sxyz[slot, 3] + args_cli.release_gap)
        tgt_pb = torch.stack((sxyz[slot, 0], sxyz[slot, 1], z0_), dim=1)
        args_cli.occ = 0.0  # 앞 칸 퍽 = 실제로 놓은 퍽

    occ_on = torch.zeros(n, N_OCC, dtype=torch.bool, device=dev)  # occ j -> 채우는 순서 j 번째 칸
    if args_cli.occ > 0:
        occ_on = (torch.arange(N_OCC, device=dev)[None, :] < slot[:, None]) & (torch.rand(n, N_OCC, device=dev) < args_cli.occ)

    def place_occ():
        allid = torch.arange(n, device=dev)
        for j in range(min(N_OCC, len(SLOTS))):
            ob = u.scene[f"occ{j}"]
            s = SLOTS[j]
            pb_ = torch.tensor([[s["x"], s["y"], s["floor_z"] + PUCK_H / 2 + 0.001]], device=dev).expand(n, 3)
            pw = r.data.root_link_pos_w.torch + quat_apply(r.data.root_link_quat_w.torch, pb_)
            pos = torch.where(occ_on[:, j:j + 1], pw, ob.data.root_pos_w.torch)
            ob.write_root_pose_to_sim_index(root_pose=torch.cat((pos, ob.data.default_root_pose.torch[:, 3:7]), dim=1), env_ids=allid)
            ob.write_root_velocity_to_sim_index(root_velocity=torch.zeros(n, 6, device=dev), env_ids=allid)

    # ---- 상태 ----
    ez = torch.tensor([[0.0, 0.0, 1.0]], device=dev).repeat(n, 1)

    def to_bl(p_w):
        return quat_apply_inverse(r.data.root_link_quat_w.torch, p_w - r.data.root_link_pos_w.torch)

    def state():
        pc = to_bl(u.puck_pos_w())
        uu = quat_apply_inverse(r.data.root_link_quat_w.torch, quat_apply(u.puck_quat_w(), ez))
        uu = uu * torch.where(uu[:, 2:3] < 0, -1.0, 1.0)
        pb = pc - uu * (PUCK_H / 2)
        gb = to_bl(u.grasp_w())
        return dict(pc=pc, u=uu, pb=pb, gb=gb, held=torch.linalg.norm(pc - gb, dim=1) < 0.03)

    est_locked = torch.zeros(n, dtype=torch.bool, device=dev)
    pk_close = torch.zeros(n, 3, device=dev)  # --est_z box: 닫는 순간 손 안 퍽 중심 (집게 몸, 참값) -> 집기 끝까지 미끄러진 양
    pk_slip = torch.zeros(n, 3, device=dev)
    est_used = torch.zeros(n, dtype=torch.bool, device=dev)
    ez_dbg = torch.zeros(n, dtype=torch.bool, device=dev)  # --est_z box 판정 확인: 닫힘 · +쥠 · +상자 위
    ez_dbg2 = torch.zeros_like(ez_dbg)
    ez_dbg3 = torch.zeros_like(ez_dbg)  # 집기 끝에 닫는 순간 값을 쓴 대  # --est_z box: 닫는 순간 저장함
    est_err = torch.zeros(n, 3, device=dev)  # 2026-11-06: 집기 끝 손 안 퍽 중심 가정 오차 (집게 몸 x 앞 · y 손가락 · z 카메라) [mm]
    est_c = torch.zeros(n, 3, device=dev)  # 집게 몸 (gripper_roll_link) 좌표의 추정 퍽 중심 · 축
    est_u = torch.zeros(n, 3, device=dev)

    def est_store(mask):
        """집기 끝 순간: 퍽 수직 (월드 z), 중심 = 잡는 점 − 15 mm (집기 목표 grasp_dz) 라고 가정해 집게 몸 좌표로 저장."""
        nonlocal est_c, est_u, est_err, pk_slip, est_used
        mask0 = mask
        if args_cli.est_z == "box":  # 닫는 순간 이미 저장한 대는 그대로 둠
            mask = mask & ~est_locked
        pa, qa = r.data.body_link_pos_w.torch[:, u._a5], r.data.body_link_quat_w.torch[:, u._a5]
        upw = quat_apply(r.data.root_link_quat_w.torch, ez)
        if LIEROLL:  # 롤 180° 뒤: 퍽 중심 = 잡는 점 − 15 mm 위 + grasp_dz 바깥 (팔 방향), 축 = 위
            hh_ = u.grasp_w() - (r.data.root_link_pos_w.torch + quat_apply(r.data.root_link_quat_w.torch, torch.tensor([[u.cfg.joint1_x, 0.0, 0.0]], device=dev).expand(n, 3)))
            hh_ = hh_ - (hh_ * upw).sum(1, keepdim=True) * upw
            hh_ = hh_ / hh_.norm(dim=1, keepdim=True).clamp(min=1e-6)
            c_w = u.grasp_w() - upw * 0.015 + hh_ * u.cfg.grasp_dz
            ax_w = upw
        elif LIE_SCRIPT and not HANG:  # 누운 퍽 각본 (B): 계획한 잡는 점 = 퍽 가운데 + lie_dx (긴 축 바깥 끝 쪽) + 11 mm 위, 퍽 축 = 그 긴 축 (수평)
            Lw = quat_apply(r.data.root_link_quat_w.torch, lie_Lb)
            c_w = u.grasp_w() - upw * u.cfg.grasp_dz_lie - Lw * args_cli.lie_dx
            ax_w = Lw
        else:
            c_w = u.grasp_w() - upw * u.cfg.grasp_dz
            ax_w = upw
        if args_cli.est_meas >= 0:  # 잼: 참 중심 + 잡음, 참 축
            c_w = u.puck_pos_w() + torch.randn(n, 3, device=dev) * (args_cli.est_meas / 1000.0)
            ax_w = quat_apply(u.puck_quat_w(), ez)
            ax_w = ax_w * torch.where(ax_w[:, 2:3] < 0, -1.0, 1.0)
        est_c = torch.where(mask[:, None], quat_apply_inverse(qa, c_w - pa), est_c)
        est_err = torch.where(mask0[:, None], (quat_apply_inverse(qa, u.puck_pos_w() - pa) - est_c) * 1000, est_err)
        est_used = torch.where(mask0, est_locked, est_used)
        pk_slip = torch.where((mask0 & est_locked)[:, None], (quat_apply_inverse(qa, u.puck_pos_w() - pa) - pk_close) * 1000, pk_slip)  # 집기 끝 (3 cm 든 뒤) 가정 오차 (참 − 가정) 집게 몸 좌표 [mm]
        est_u = torch.where(mask[:, None], quat_apply_inverse(qa, ax_w), est_u)

    def state_est():
        """--est: 쥔 퍽 위치·축 = 저장한 집게 몸 좌표 값을 지금 집게 자세로 옮김 (참값은 안 씀)."""
        s_ = state()
        pa, qa = r.data.body_link_pos_w.torch[:, u._a5], r.data.body_link_quat_w.torch[:, u._a5]
        pc = to_bl(pa + quat_apply(qa, est_c))
        uu = quat_apply_inverse(r.data.root_link_quat_w.torch, quat_apply(qa, est_u))
        uu = uu * torch.where(uu[:, 2:3] < 0, -1.0, 1.0)
        return dict(pc=pc, u=uu, pb=pc - uu * (PUCK_H / 2), gb=s_["gb"], held=s_["held"])

    def lean_of(s_):
        """쥔 퍽 기울기 (base): (팔 방향 = 퍽 위가 바깥으로, 접선) [rad]. 팔 방향은 집게 피치로 바로잡을 수 있음."""
        h = torch.stack((s_["gb"][:, 0] - u.cfg.joint1_x, s_["gb"][:, 1], torch.zeros(n, device=dev)), dim=1)
        h = h / h.norm(dim=1, keepdim=True).clamp(min=1e-6)
        tg = torch.stack((-h[:, 1], h[:, 0], torch.zeros(n, device=dev)), dim=1)
        uz = s_["u"][:, 2].clamp(min=1e-3)
        return torch.atan2((s_["u"] * h).sum(1), uz), torch.atan2((s_["u"] * tg).sum(1), uz)

    def ik_pitch(dp_b, pitch):
        """집기 환경 _ik_step 과 같은 감쇠 최소제곱 IK (집게 = 바깥 방향에서 pitch 만큼 아래, 손가락 닫는 축 수평), pitch 만 대마다."""
        c = u.cfg
        rq = d_().root_link_quat_w.torch
        J = d_().body_link_jacobian_w.torch[:, u._a5][:, :, u._cols]
        q5 = d_().body_link_quat_w.torch[:, u._a5]
        rr = quat_apply(q5, u._off)
        Jl = J[:, 0:3] - torch.cross(rr.unsqueeze(-1).expand(-1, -1, 5), J[:, 3:6], dim=1)
        w = c.rot_hold_weight
        Jf = torch.cat((Jl, J[:, 3:6] * w), dim=1)
        j1 = d_().root_link_pos_w.torch + quat_apply(rq, torch.tensor([[c.joint1_x, 0.0, 0.0]], device=dev).expand(n, 3))
        up = quat_apply(rq, ez)
        a = u.grasp_w() - j1
        a = a - (a * up).sum(1, keepdim=True) * up
        a = a / a.norm(dim=1, keepdim=True).clamp(min=1e-6)
        cp, sp = torch.cos(pitch)[:, None], torch.sin(pitch)[:, None]
        a, up = cp * a - sp * up, sp * a + cp * up
        R = torch.stack((a, torch.cross(up, a, dim=1), up), dim=2)
        flip = (j5_off.abs() > 1.0)[:, None, None]  # 롤 180° 한 대: 같은 집게 방향, 닫는 축 · 카메라 축 반대
        R = torch.where(flip, R * torch.tensor([1.0, -1.0, -1.0], device=dev).view(1, 1, 3), R)
        err = torch.cat((quat_apply(rq, dp_b), quat_box_minus(quat_from_matrix(R), q5) * w), dim=1)
        JJt = Jf @ Jf.transpose(1, 2) + c.ik_damping * torch.eye(6, device=dev)
        return (Jf.transpose(1, 2) @ torch.linalg.solve(JJt, err.unsqueeze(-1))).squeeze(-1)

    def d_():
        return r.data

    def force(k):
        return torch.linalg.norm(u.scene[k].data.normal_force_matrix_w.torch, dim=-1).amax(dim=(1, 2))

    phase = torch.zeros(n, dtype=torch.long, device=dev)
    cnt = torch.zeros(n, dtype=torch.long, device=dev)
    pcnt = torch.zeros(n, dtype=torch.long, device=dev)  # 단계 안 스텝
    t_enter = torch.full((n, len(PHASES)), -1.0, device=dev)
    qp = torch.zeros(n, 5, device=dev)  # 관절 경로 계획 (보정 전)
    bias = torch.zeros(n, 5, device=dev)
    idx = torch.zeros(n, dtype=torch.long, device=dev)
    aborted = torch.zeros(n, dtype=torch.bool, device=dev)
    abort_q = torch.zeros(n, 5, device=dev)
    toppled = torch.zeros(n, dtype=torch.bool, device=dev)
    nb_touch = torch.zeros(n, dtype=torch.bool, device=dev)
    nb_first = torch.full((n,), -99, dtype=torch.long, device=dev)  # 옆 퍽에 처음 닿은 때: A 각본 단계 rp (0 ~ 14), −1 = 정책 다시 집기, 100 + phase = 운반 뒤
    tp_first = torch.full((n,), -99, dtype=torch.long, device=dev)  # 목표 퍽이 처음 넘어간 (축 z < 0.7) 때, 같은 표기
    lost_phase = torch.full((n,), -1, dtype=torch.long, device=dev)
    NP = len(PHASES)
    f_arm, f_fing, f_puck, f_occ = (torch.zeros(n, NP, device=dev) for _ in range(4))
    hit_first = torch.full((n,), -1, dtype=torch.long, device=dev)  # 팔·손가락-몸체 처음 1 N 넘은 단계
    hit_q = torch.zeros(n, 5, device=dev)
    al_end = torch.zeros(n, 3, device=dev)  # 놓는 순간 (퍽 바닥 - 목표) xyz
    al_tmo = torch.zeros(n, dtype=torch.bool, device=dev)
    al_tilt = torch.zeros(n, device=dev)  # 놓는 순간 퍽 기울기 [°]
    al_lean = torch.zeros(n, 2, device=dev)  # 놓는 순간 (팔 방향, 접선) 기울기 [°], 놓기 시작 때 (팔 방향, 접선)
    al_lean0 = torch.zeros(n, 2, device=dev)
    pk_lean = torch.zeros(n, 2, device=dev)  # 집기 끝 (운반 시작) 순간 참 기울기 (팔 방향, 접선) [°], 잡는 점 - 퍽 중심 높이 [mm]
    pk_dz = torch.zeros(n, device=dev)
    tr_lean_max = torch.zeros(n, device=dev)  # 운반 중 팔 방향 기울기 최대 |°|
    pitch0 = 0.0 if (args_cli.lie and not HANG) else math.radians(45.0)  # 놓기 IK 기본 집게 피치 (누운 퍽 B안 = 수평)
    plim = math.radians(args_cli.pitch_lim if args_cli.pitch_lim is not None else (10.0 if args_cli.lie else 20.0))
    pitch_t = torch.full((n,), pitch0, device=dev)
    rel_q = torch.zeros(n, 5, device=dev)  # 놓는 순간 관절 (한계 여유 확인)
    ov_pair = torch.full((n, 6), -9.0, device=dev)  # 쌍마다 최대 겹침 깊이 (arm_selfcol.PAIRS 순서)
    ov_qd = torch.full((n,), -9.0, device=dev)
    ov_q = torch.zeros(n, 5, device=dev)  # 가장 깊이 겹친 순간 관절
    ov_ph = torch.full((n, 6), -9.0, device=dev)  # 2026-11-06 단계별 팔끼리 충돌 상자 겹침 깊이 최대 [m] (팀 USD 는 팔끼리 충돌 뺌 -> 판정으로 막음)
    sd_ph = torch.full((n, 6), 9.0, device=dev)  # 2026-11-04 단계별 자기 충돌 근접 최소 거리 (집게 · 쥔 퍽 - 윗팔 · 아랫팔)
    # 누운 퍽: 세우기 (chain_play --lie 와 같은 관절 보간 + 경유 자세), 쥐는 점 옮기기 (인식 좌표에 더함)
    lie_k = torch.full((n,), -1, dtype=torch.long, device=dev)
    lie_cnt = torch.zeros(n, dtype=torch.long, device=dev)
    lie_q0, lie_via, lie_to = (torch.zeros(n, 5, device=dev) for _ in range(3))
    stood = torch.zeros(n, dtype=torch.bool, device=dev)
    ever_held = torch.zeros(n, dtype=torch.bool, device=dev)
    bias0 = u.perc_bias.clone()
    ls = torch.zeros(n, dtype=torch.long, device=dev)  # 누운 퍽 각본 집기 단계: 0 위에서 맞춤, 1 내림, 2 쥠, 3 들기, 4 끝
    ls_cnt = torch.zeros(n, dtype=torch.long, device=dev)
    LIE_SCRIPT = args_cli.lie and args_cli.lie_pick in ("script", "hang")
    fc0 = float(cfg.gripper_closed)
    lie_plan = torch.zeros(n, 5, 5, device=dev)
    lie_bad = torch.zeros(n, dtype=torch.bool, device=dev)
    lie_G = torch.zeros(n, 3, device=dev)
    lie_psi = torch.zeros(n, device=dev)
    lie_Lb = torch.zeros(n, 3, device=dev)  # 누운 퍽 긴 축 (base, 위로 갈 끝 쪽)
    lie_ok_cnt = torch.zeros(n, dtype=torch.long, device=dev)
    hang_cnt = torch.zeros(n, dtype=torch.long, device=dev)

    def plan_lie_grasp(s_):
        """퍽 인식 (참값 + 에피소드 치우침) -> 잡는 점 G = 가운데 + lie_dx (긴 축 바깥 쪽 끝) + grasp_dz_lie, 집게 아래 · 카메라 위 = 그 끝.
        위 6 cm · G · 위 4 cm 의 관절 자세 (팀 URDF FK + scipy, 앞 해에서 이어 풂)."""
        nonlocal lie_G, lie_psi
        import numpy as np
        sys.path.insert(0, os.path.join(HERE, "tools_ik"))
        from td_ik import LIM, fk, least_squares
        lim = LIM.copy(); lim[0] = [-3.0, 3.0]
        pc = to_bl(u.puck_pos_w() + bias0)
        Lb = quat_apply_inverse(r.data.root_link_quat_w.torch, u.long_axis_w()); Lb[:, 2] = 0.0
        Lb = Lb / Lb.norm(dim=1, keepdim=True).clamp(min=1e-6)
        rad = pc[:, :2] - torch.tensor([[u.cfg.joint1_x, 0.0]], device=dev)
        Lb = Lb * torch.where((Lb[:, :2] * rad).sum(1, keepdim=True) >= 0, 1.0, -1.0)
        if HANG:
            lie_G = pc + Lb * args_cli.hang_d + torch.tensor([[0.0, 0.0, args_cli.hang_gz]], device=dev)
        else:
            lie_G = pc + Lb * args_cli.lie_dx + torch.tensor([[0.0, 0.0, u.cfg.grasp_dz_lie]], device=dev)
        lie_psi = torch.atan2(Lb[:, 1], Lb[:, 0])
        lie_Lb[:] = Lb
        plan = np.zeros((n, 5, 5)); bad = np.zeros(n, dtype=bool)
        q_now = r.data.joint_pos.torch[:, u._arm].cpu().numpy()
        for i in range(n):
            G = lie_G[i].cpu().numpy(); e = Lb[i].cpu().numpy()
            qg = q_now[i].copy(); qg[4] = float(np.clip(math.atan2(e[1], e[0]) - math.atan2(G[1], G[0] - u.cfg.joint1_x), -2.7, 2.7))
            hh = np.array([G[0] - u.cfg.joint1_x, G[1], 0.0]); hh /= max(np.linalg.norm(hh), 1e-6)
            zz = np.array([0.0, 0.0, 1.0]); s45 = math.sqrt(0.5)
            # (잡는 점 높이, 집게 방향, 카메라 z): 0 위 6 cm, 1 쥐기, 2 들기, 3 (hang) 롤 = 원기둥 집기처럼 (닫는 축 접선), 4 (hang) 45° 숙임 (닫는 축 둘레라 잡는 점 그대로)
            lift_ = 0.06 if HANG else 0.04
            poses = [(0.06, -zz, e), (0.0, -zz, e), (lift_, -zz, e)]
            if HANG:
                poses += [(lift_, -zz, hh), (lift_, s45 * hh - s45 * zz, s45 * hh + s45 * zz)]
            for j, (dz, gdir, cz_) in enumerate(poses):
                p_ = G + np.array([0.0, 0.0, dz])

                def res(qq):
                    P, R = fk(qq)
                    return list((P - p_) * 100) + list((R[:, 0] - gdir) * 3) + list((R[:, 2] - cz_) * 3)
                seeds = [qg] + [np.array([qg[0], a_, b_, c_, qg[4]]) for a_ in (0.3, 0.7, 1.1) for b_ in (-1.2, -0.6, 0.0) for c_ in (1.2, 1.7)]
                sol = None
                for sd in seeds:
                    so = least_squares(res, np.clip(sd, lim[:, 0] + 1e-4, lim[:, 1] - 1e-4), bounds=(lim[:, 0], lim[:, 1]))
                    if sol is None or so.cost < sol.cost:
                        sol = so
                    if so.cost < 1e-6:
                        break
                P, R = fk(sol.x)
                if np.linalg.norm(P - p_) > 0.002 or R[:, 0] @ gdir < 0.999:
                    bad[i] = True
                plan[i, j] = sol.x; qg = sol.x
            for j in range(len(poses), 5):
                plan[i, j] = plan[i, len(poses) - 1]
        rr_ = torch.linalg.norm(lie_G[:, :2] - torch.tensor([[u.cfg.joint1_x, 0.0]], device=dev), dim=1).tolist()
        print(f"[Chain45] 누운 퍽 각본 집기 IK: 실패 {int(bad.sum())}/{n}" + (f" (잡는 점 반지름 mm {[round(rr_[i] * 1000) for i in range(n) if bad[i]]}, 성공 범위 {round(min(rr_[i] for i in range(n) if not bad[i]) * 1000)} ~ {round(max(rr_[i] for i in range(n) if not bad[i]) * 1000)})" if bad.any() and (~bad).any() else ""), flush=True)
        return torch.tensor(plan, dtype=torch.float32, device=dev), torch.tensor(bad, device=dev)
    th_init = None
    rl_k = torch.full((n,), -1, dtype=torch.long, device=dev)  # LIEROLL 단계: −1 정책 집기, 0 5 cm 더 들기, 1 롤 180°, 2 세로 확인, 3 끝
    rl_cnt = torch.zeros(n, dtype=torch.long, device=dev)
    rl_q5 = torch.zeros(n, device=dev)
    j5_off = torch.zeros(n, device=dev)
    rl_ok = torch.zeros(n, dtype=torch.bool, device=dev)
    rl_fail = torch.zeros(n, dtype=torch.bool, device=dev)
    rl_hold = torch.zeros(n, dtype=torch.long, device=dev)
    # ---- A (REGRASP): 누운 직육면체 각본 단계 rp: 0 위 6 cm 맞춤 확인, 1 내림, 2 쥠, 3 들기, 4 내려놓을 자리 위 (집게 수평 · 퍽 세로), 5 내려놓기, 6 벌림, 7 뒤로 빼기, 8 시작 자세, −1 = 정책 (7 = 벌린 뒤 위로 빼기) ----
    rp = torch.full((n,), 0 if REGRASP else -1, dtype=torch.long, device=dev)
    rp_cnt = torch.zeros(n, dtype=torch.long, device=dev)
    rp_ok = torch.zeros(n, dtype=torch.long, device=dev)
    rp_plan = torch.zeros(n, 9, 5, device=dev)
    rp_D = torch.zeros(n, 3, device=dev)  # 세워 내려놓을 퍽 중심 (base)
    rp_fail = torch.zeros(n, dtype=torch.bool, device=dev)
    rp_stood = torch.zeros(n, dtype=torch.bool, device=dev)

    def place_lying():
        """누운 직육면체를 관절1 축 0.185 ~ 0.205 m · 방향 ±0.25 rad · 긴 축 요 무작위로, 상자를 그 바깥으로 (세워 내려놓을 자리 포함), 옆 퍽은 접선 쪽 7.5 cm."""
        ids = torch.arange(n, device=dev)
        org = u.scene.env_origins
        rr = 0.185 + torch.rand(n, device=dev) * 0.02
        an = (torch.rand(n, device=dev) * 2 - 1) * 0.25
        hx, hy = torch.cos(an), torch.sin(an)
        px, py = u.cfg.joint1_x + rr * hx, rr * hy
        if args_cli.lie_yaw == "any":
            lyaw = (torch.rand(n, device=dev) * 2 - 1) * math.pi / 2 + an
        else:  # 긴 축 (몸 z) 의 수평 방향 = lyaw (x 축 회전 없이 y 축 90° 눕힘)
            lyaw = an + (0.0 if args_cli.lie_yaw == "radial" else math.pi / 2) + (torch.rand(n, device=dev) * 2 - 1) * math.radians(10)
        zero = torch.zeros(n, device=dev)
        def put(k_, x_, y_, z_, q_):
            o_ = u._obj(k_)
            pose = torch.stack((x_ + org[:, 0], y_ + org[:, 1], z_ + org[:, 2]), dim=1)
            o_.write_root_pose_to_sim_index(root_pose=torch.cat((pose, q_), dim=1), env_ids=ids)
            o_.write_root_velocity_to_sim_index(root_velocity=torch.zeros(n, 6, device=dev), env_ids=ids)
        put("boxA", px + 0.075 * hx, py + 0.075 * hy, torch.full((n,), BOX_H / 2, device=dev), quat_from_euler_xyz(zero, zero, an))
        q_lie = quat_mul(quat_from_euler_xyz(zero, zero, lyaw), quat_from_euler_xyz(zero, torch.full((n,), math.pi / 2, device=dev), zero))
        put("puckT_box", px, py, torch.full((n,), BOX_H + 0.016, device=dev), q_lie)
        sg = torch.where(torch.rand(n, device=dev) < 0.5, 1.0, -1.0)
        if args_cli.lie_nb == "tan":  # 세워 놓을 자리 (plan_regrasp 의 D = 퍽 + 반지름 0.135) 의 접선 8 cm
            dx_, dy_ = px + 0.135 * hx, py + 0.135 * hy
            put("puckN", dx_ - sg * 0.08 * hy, dy_ + sg * 0.08 * hx, torch.full((n,), BOX_H + PUCK_H / 2 + 0.001, device=dev), quat_from_euler_xyz(zero, zero, an))
        else:
            put("puckN", px - sg * 0.075 * hy, py + sg * 0.075 * hx, torch.full((n,), BOX_H + PUCK_H / 2 + 0.001, device=dev), quat_from_euler_xyz(zero, zero, zero))
        u.is_box[:] = True
        u.puck_z0[:] = BOX_H + PUCK_H / 2 + 0.001 + org[:, 2]  # 세워 놓은 뒤 들기 높이 기준
        u.nb_prev[:] = u._obj("puckN").data.root_pos_w.torch[:, :2]

    def plan_regrasp():
        """누운 퍽 인식값으로 경유 관절 자세 6 개 (팀 URDF FK + scipy)."""
        nonlocal rp_D
        import numpy as np
        sys.path.insert(0, os.path.join(HERE, "tools_ik"))
        from td_ik import LIM, fk, least_squares
        lim = LIM.copy(); lim[0] = [-3.0, 3.0]
        pc = to_bl(u.puck_pos_w())
        Lb = quat_apply_inverse(r.data.root_link_quat_w.torch, quat_apply(u.puck_quat_w(), ez)); Lb[:, 2] = 0.0
        Lb = Lb / Lb.norm(dim=1, keepdim=True).clamp(min=1e-6)
        rad = torch.stack((pc[:, 0] - u.cfg.joint1_x, pc[:, 1], torch.zeros(n, device=dev)), dim=1)
        rad = rad / rad.norm(dim=1, keepdim=True).clamp(min=1e-6)
        Lb = Lb * torch.where((Lb * rad).sum(1, keepdim=True) >= 0, 1.0, -1.0)  # 위로 갈 끝 = 바깥 쪽
        G = pc + Lb * args_cli.lie_dx + torch.tensor([[0.0, 0.0, 0.011]], device=dev)
        top = pc[:, 2] - 0.016  # 상자 윗면 (base)
        D = pc + rad * 0.135  # 세울 자리 = 관절1 축 약 0.32 ~ 0.34 m (45° 집기 학습 배치 0.32 ~ 0.345 와 같게, 시작 자세 잡는 점 0.28 m 와 5 cm 떨어짐)
        D[:, 2] = top + PUCK_H / 2 + 0.002
        rp_D = D
        plan = np.zeros((n, 9, 5)); bad = np.zeros(n, dtype=bool)
        q_now = r.data.joint_pos.torch[:, u._arm].cpu().numpy()
        zz = np.array([0.0, 0.0, 1.0]); s45 = math.sqrt(0.5)
        for i in range(n):
            g_ = G[i].cpu().numpy(); e = Lb[i].cpu().numpy(); d_ = D[i].cpu().numpy()
            h = np.array([d_[0] - u.cfg.joint1_x, d_[1], 0.0]); h /= max(np.linalg.norm(h), 1e-6)
            tcp_d = d_ - 0.011 * h + args_cli.lie_dx * zz  # 세운 퍽: 퍽 중심 = 잡는 점 + 11 mm 집게 방향 − lie_dx 위
            poses = [(g_ + 0.06 * zz, -zz, e), (g_, -zz, e), (g_ + 0.05 * zz, -zz, e),
                     (tcp_d + 0.06 * zz, h, zz), (tcp_d, h, zz), (tcp_d + 0.07 * zz, h, zz)]  # 벌린 뒤 위로 7 cm (로봇 쪽으로 빼면 수평 집게가 안 닿음)
            g45, c45 = s45 * h - s45 * zz, s45 * h + s45 * zz  # 다시 집기: 집게 45° 아래 · 바깥, 잡는 점 = 세운 퍽 중심 + 15 mm
            T = d_ + 0.015 * zz
            poses += [(T - 0.05 * g45, g45, c45), (T, g45, c45), (T + 0.04 * zz, g45, c45)]
            qg = q_now[i].copy(); qg[4] = float(np.clip(math.atan2(e[1], e[0]) - math.atan2(g_[1], g_[0] - u.cfg.joint1_x), -2.7, 2.7))
            for j, (p_, gd, cz_) in enumerate(poses):
                def res(qq):
                    P, R = fk(qq)
                    return list((P - p_) * 100) + list((R[:, 0] - gd) * 3) + list((R[:, 2] - cz_) * 3)
                best = None
                for sd in [qg] + [np.array([qg[0], a_, b_, c_, qg[4] if j < 3 else 0.0]) for a_ in (0.3, 0.7, 1.1) for b_ in (-1.2, -0.4, 0.4, 1.2) for c_ in (-1.5, 0.0, 1.2, 1.7)]:
                    so = least_squares(res, np.clip(sd, lim[:, 0] + 1e-4, lim[:, 1] - 1e-4), bounds=(lim[:, 0], lim[:, 1]))
                    if best is None or so.cost < best.cost:
                        best = so
                    if so.cost < 1e-6:
                        break
                P, R = fk(best.x)
                if np.linalg.norm(P - p_) > 0.002 or R[:, 0] @ gd < 0.999:
                    bad[i] = True
                plan[i, j] = best.x; qg = best.x
        print(f"[Chain45] A 누운 퍽 다시 집기 경로 IK: 실패 {int(bad.sum())}/{n}", flush=True)
        return torch.tensor(plan, dtype=torch.float32, device=dev), torch.tensor(bad, device=dev)
    rg = torch.full((n,), -1, dtype=torch.long, device=dev)  # 다시 집기 단계: −1 안 함, 0 내려놓기, 1 벌림, 2 위로 빼기
    rg_cnt = torch.zeros(n, dtype=torch.long, device=dev)
    rg_n = torch.zeros(n, dtype=torch.long, device=dev)
    pb0 = None  # 집을 퍽 처음 바닥 (base)
    rg_tilts = []  # 다시 집기를 부른 기울기 [°]
    go0 = float(cfg.gripper_open)
    arm_prev = u.arm_target.clone()
    lead = u.cfg.target_lead_max
    steps = int(args_cli.seconds / dt)

    def follow(path, plen, mask, speed):
        """관절 경로를 속도 한도로 따라감 (경유점 사이 직선, 최대 성분 기준)."""
        nonlocal qp, idx
        ii = idx.clamp(max=path.shape[1] - 1)
        w = path[slot, ii].clone()
        if path is path_go:
            w[:, 4] += j5_off  # 롤 180° 한 대는 운반 경로 joint5 도 +-π
        dq = w - qp
        m = dq.abs().amax(dim=1)
        stp = (speed * dt).clamp(min=1e-6)
        reach = m <= stp
        qn = torch.where(reach[:, None], w, qp + dq * (stp / m.clamp(min=1e-9))[:, None])
        qp = torch.where(mask[:, None], qn, qp)
        idx = torch.where(mask & reach & (idx < plen[slot]), idx + 1, idx)
        return mask & (idx >= plen[slot])

    seq_i = torch.zeros(n, dtype=torch.long, device=dev)  # 연속 채우기: 지금 몇 번째 퍽 (0 부터)
    seq_ok = torch.zeros(n, SEQ, dtype=torch.long, device=dev) - 1  # 퍽마다 결과: 1 칸에 앉음, 0 실패, −1 아직
    seq_swap = torch.full((n,), -1, dtype=torch.long, device=dev)  # 이 스텝에 다음 퍽으로 바꿈
    seq_t0 = torch.zeros(n, device=dev)  # 지금 퍽 집기 시작 시각
    puck_init = None
    steps = steps if SEQ == 1 else int(args_cli.seconds * SEQ / dt)
    for k in range(steps):
        s = state()
        d = r.data
        q = d.joint_pos.torch[:, u._arm]
        if th_init is None:
            th_init = torch.atan2(s["pc"][:, 1], s["pc"][:, 0] - u.cfg.joint1_x)
        if pb0 is None and k == 3:
            pb0 = s["pb"].clone()
            puck_init = {kk: torch.cat((u._obj(kk).data.root_pos_w.torch, u._obj(kk).data.root_quat_w.torch), dim=1).clone() for kk in ("puckT_cyl", "puckT_box")}
        if SEQ > 1 and (seq_swap == k).any():  # 놓은 퍽 -> 치워 둔 퍽 j 로 바꿔 칸에 그대로, 목표 퍽은 상자 위 처음 자리로, 다음 칸으로
            sw = seq_swap == k
            ids = torch.nonzero(sw).flatten()
            pos_t, quat_t = u.puck_pos_w(), u.puck_quat_w()  # 대마다 원기둥 / 직육면체
            for i in ids.tolist():
                j = int(slot[i]) if PB == 2 else int(seq_i[i])  # 상자당 2 개: 놓은 칸 번호의 occ (앞 칸 참과 겹치지 않음)
                if j >= N_OCC:
                    continue
                ob = u.scene[f"occ{j}"]
                ii = torch.tensor([i], device=dev)
                ob.write_root_pose_to_sim_index(root_pose=torch.cat((pos_t[ii], quat_t[ii]), dim=1), env_ids=ii)
                ob.write_root_velocity_to_sim_index(root_velocity=torch.zeros(1, 6, device=dev), env_ids=ii)
            if PB == 2:  # 두 번째 퍽: 목표 퍽을 옆 퍽 자리 · 방향으로 (모양 그대로), 옆 퍽은 치움 (관측 nb = 0, 학습 '두 번째 퍽' 과 같음)
                nbo = u._obj("puckN")
                nb_pose = torch.cat((nbo.data.root_pos_w.torch, nbo.data.root_quat_w.torch), dim=1)[ids].clone()
                j1w = r.data.root_link_pos_w.torch + quat_apply(r.data.root_link_quat_w.torch, torch.tensor([[u.cfg.joint1_x, 0.0, 0.0]], device=dev).expand(n, 3))
                rt_ = torch.linalg.norm(nb_pose[:, :2] - j1w[ids, :2], dim=1)
                shv = quat_apply(r.data.root_link_quat_w.torch[ids], torch.tensor([[1.0, 0.0, 0.0]], device=dev).expand(len(ids), 3)) * torch.clamp(rt_ - 0.345, min=0.0)[:, None] * (rt_ > 0.365).float()[:, None]
                nb_pose[:, :3] -= shv
                bxo = u._obj("boxA")  # 차체를 shv 만큼 앞으로 = 상자 · 퍽을 로봇 쪽으로
                bpose = torch.cat((bxo.data.root_pos_w.torch, bxo.data.root_quat_w.torch), dim=1)[ids].clone()
                bpose[:, :3] -= shv
                bxo.write_root_pose_to_sim_index(root_pose=bpose, env_ids=ids)
                bxo.write_root_velocity_to_sim_index(root_velocity=torch.zeros(len(ids), 6, device=dev), env_ids=ids)
                for kk in ("puckT_cyl", "puckT_box"):
                    pk_ = u._obj(kk)
                    use = u.is_box[ids] == (kk == "puckT_box")
                    pose_ = torch.where(use[:, None], nb_pose, puck_init[kk][ids])
                    pk_.write_root_pose_to_sim_index(root_pose=pose_, env_ids=ids)
                    pk_.write_root_velocity_to_sim_index(root_velocity=torch.zeros(len(ids), 6, device=dev), env_ids=ids)
                park = nbo.data.root_pos_w.torch[ids].clone()
                park[:, :2] = u.scene.env_origins[ids, :2] + torch.tensor([[0.2, 0.7]], device=dev)
                nbo.write_root_pose_to_sim_index(root_pose=torch.cat((park, nb_pose[:, 3:7]), dim=1), env_ids=ids)
                nbo.write_root_velocity_to_sim_index(root_velocity=torch.zeros(len(ids), 6, device=dev), env_ids=ids)
                u.nb_present[ids] = False
                for i in ids.tolist():  # 첫 퍽이 앉은 칸도 '앞 칸 퍽' 으로 닿음 · 밀림 검사
                    if int(slot[i]) < N_OCC:
                        occ_on[i, int(slot[i])] = True
                u.nb_prev[ids] = park[:, :2]
                u.puck_z0[ids] = nb_pose[:, 2]
                v_ = quat_apply_inverse(r.data.root_link_quat_w.torch[ids], nb_pose[:, :3] - j1w[ids])  # 집기 중 관절1 안전 범위 = 새 퍽 방향 ±0.6 rad
                th_init = th_init.clone()
                th_init[ids] = torch.atan2(v_[:, 1], v_[:, 0])
                for q_, i in enumerate(ids.tolist()):
                    print(f"[Chain45] t={k * dt:5.1f}s env{i} 두 번째 퍽: 관절1 축 {float(rt_[q_]) * 1000:.0f} mm" + (f" -> 차체 {float(shv[q_].norm()) * 1000:.0f} mm 앞으로" if float(shv[q_].norm()) > 0 else ""), flush=True)
            else:
                for kk in ("puckT_cyl", "puckT_box"):  # 목표 퍽을 상자 위 처음 자리로 (쓰는 쪽만 의미 있음, 안 쓰는 쪽은 처음 치운 자리)
                    pk_ = u._obj(kk)
                    pk_.write_root_pose_to_sim_index(root_pose=puck_init[kk][ids], env_ids=ids)
                    pk_.write_root_velocity_to_sim_index(root_velocity=torch.zeros(len(ids), 6, device=dev), env_ids=ids)
            seq_i = torch.where(sw, seq_i + 1, seq_i)
            slot = torch.where(sw, (slot + 1).clamp(max=len(SLOTS) - 1), slot) if PB == 2 else torch.where(sw, seq_i.clamp(max=len(SLOTS) - 1), slot)
            z_new = (sxyz[slot, 2] + 0.001) if args_cli.place == "floor" else (sxyz[slot, 3] + args_cli.release_gap)
            tgt_pb = torch.where(sw[:, None], torch.stack((sxyz[slot, 0], sxyz[slot, 1], z_new), dim=1), tgt_pb)
            phase = torch.where(sw, torch.zeros_like(phase), phase)
            for t_ in (cnt, pcnt, rg_cnt, rg_n, idx):
                t_[sw] = 0
            rg[sw] = -1
            lost_phase[sw] = -1
            ever_held[sw] = False
            toppled[sw] = False
            seq_t0 = torch.where(sw, torch.full_like(seq_t0, k * dt), seq_t0)
            u.actions[sw] = 0.0
            u.actions[sw, -1] = 1.0
            u.prev_actions[sw] = u.actions[sw]
            seq_swap[sw] = -1
            for i in ids.tolist():
                sl = SLOTS[int(slot[i])]
                print(f"[Chain45] t={k * dt:5.1f}s env{i} 퍽 {int(seq_i[i]) + 1} 번째 -> {sl['row']}열 {sl['index']}", flush=True)
        if args_cli.lie and not LIE_SCRIPT:  # 쥐는 점 = 퍽 가운데 + lie_dx (긴 축 중 집게 카메라 위 (+z) 쪽 끝 = 세운 뒤 위쪽). 닫기 전까지만 다시 정함
            L = u.long_axis_w()
            L = L - L[:, 2:3] * torch.tensor([[0.0, 0.0, 1.0]], device=dev)
            L = L / L.norm(dim=1, keepdim=True).clamp(min=1e-6)
            camz = quat_apply(r.data.body_link_quat_w.torch[:, u._a5], torch.tensor([[0.0, 0.0, 1.0]], device=dev).expand(n, 3))
            sg = torch.where((L * camz).sum(1, keepdim=True) >= 0, 1.0, -1.0)
            upd = (phase == 0) & (lie_k < 0) & (u.actions[:, -1] > 0) & ~ever_held
            u.perc_bias = torch.where(upd[:, None], bias0 + sg * L * args_cli.lie_dx, u.perc_bias)
        o_sp = u._get_observations()["policy"]
        with torch.inference_mode():
            a_sp = pol(o_sp).clamp(-1.0, 1.0)
        m0, m1, m2, m3, m4 = (phase == i for i in range(5))
        tgt = arm_prev.clone()
        grip = torch.ones(n, device=dev)
        # 1 집기 = 정책 (잡는 점 5 mm 직교 + IK, 집게 45° 아래)
        if REGRASP and k == 1:
            place_lying()
        if REGRASP and k == 4:
            rp_plan, rp_bad = plan_regrasp()
            rp_fail |= rp_bad
            qp = q.clone()
            bias = u.arm_target - q
        mrp = m0 & (rp >= 0)
        if REGRASP and k > 4 and mrp.any():  # A 각본 (정책 대신)
            # 0 위 · 1 내림 · 2 쥠 · 3 들기 · 4 세울 자리 위 · 5 내려놓기 · 6 벌림 · 7 위로 빼기 · 8 팔 들기 · 9 시작 자세 · (script) 10 퍽 앞 5 cm 맞춤 확인 · 11 들어가기 · 12 쥠 · 13 들기 · 14 끝 (넘김 기다림)
            wmap = torch.tensor([0, 1, 1, 2, 3, 4, 4, 5, 5, 5, 6, 7, 7, 8, 8], device=dev)
            q_hi = q_ready.clone(); q_hi[:, 1] -= 0.45  # 시작 자세보다 어깨를 뒤로 = 팔을 들어 위에서 시작 자세로 (세운 퍽을 안 치게)
            wp = torch.where((rp == 8)[:, None], q_hi.expand(n, 5), torch.where((rp == 9)[:, None], q_ready.expand(n, 5), rp_plan[torch.arange(n, device=dev), wmap[rp.clamp(0, 14)]]))
            dqp = wp - qp
            mm_ = dqp.abs().amax(dim=1)
            stp = args_cli.speed * dt * torch.where((rp == 1) | (rp == 5) | (rp == 11), 0.3, torch.where((rp == 4) | (rp == 13), 0.5, 1.0))
            reach = mm_ <= stp
            qp = torch.where(mrp[:, None], torch.where(reach[:, None], wp, qp + dqp * (stp / mm_.clamp(min=1e-9))[:, None]), qp)
            if args_cli.ki > 0:
                bias = torch.where(mrp[:, None], (bias + args_cli.ki * dt * (qp - q)).clamp(-args_cli.bias_max, args_cli.bias_max), bias)
            tgt[mrp] = (qp + bias)[mrp]
            grip[mrp] = torch.where(((rp[mrp] >= 2) & (rp[mrp] <= 5)) | (rp[mrp] >= 12), -1.0, 1.0)
            u.actions[mrp, -1] = grip[mrp]
            rp_cnt = torch.where(mrp, rp_cnt + 1, rp_cnt)
            # 0 단계 맞춤 확인: 관절 경로 끝 + 0.3 s 머묾 (실제 잡는 점이 계획 위치 3 mm 안 = 처짐 보정 끝)
            okA = ((rp == 0) | (rp == 10)) & reach & (rp_cnt > int(0.3 / dt))  # 맞춤 확인 (경로 끝에서 0.3 s 머묾)
            okB = ((rp == 1) | (rp == 3) | (rp == 4) | (rp == 5) | (rp == 7) | (rp == 8) | (rp == 9) | (rp == 11) | (rp == 13)) & reach & (rp_cnt > 3)
            okC = ((rp == 2) | (rp == 6) | (rp == 12)) & (rp_cnt > int(0.5 / dt))
            nx = mrp & ~rp_fail & (okA | okB | okC)
            stood_now = quat_apply(u.puck_quat_w(), ez)[:, 2].abs() > 0.95
            rp_stood |= nx & (rp == 6) & stood_now
            rp_fail |= nx & (rp == 6) & ~stood_now  # 벌린 뒤 퍽이 서 있지 않으면 실패
            rp = torch.where(nx, rp + 1, rp)
            rp_cnt = torch.where(nx, torch.zeros_like(rp_cnt), rp_cnt)
            back = (rp >= 10) & torch.tensor(args_cli.regrasp_by == "policy", device=dev)
            rp = torch.where(rp >= 14, torch.full_like(rp, 14), rp)
            if back.any():  # 정책으로 넘김: 세워 놓은 퍽 방향으로 관절1 안전 범위, 행동 초기화
                th_init = torch.where(back, torch.atan2(rp_D[:, 1], rp_D[:, 0] - u.cfg.joint1_x), th_init)
                u.actions[back] = 0.0
                u.actions[back, -1] = 1.0
                u.prev_actions[back] = u.actions[back]
                seq_t0 = torch.where(back, torch.full_like(seq_t0, k * dt), seq_t0)
                for i in torch.nonzero(back).flatten().tolist():
                    print(f"[Chain45] t={k * dt:5.1f}s env{i} A: 세워 내려놓음 -> 45° 정책으로 다시 집기", flush=True)
            rp = torch.where(back, torch.full_like(rp, -1), rp)
        mrl = m0 & (rl_k >= 0)
        if LIEROLL and mrl.any():  # 쥔 뒤: 0 잡는 점 5 cm 더 위로 (IK, 2 mm / 스텝) -> 1 joint5 를 180° (2.5 s) -> 2 퍽 세로 확인 0.3 s
            dpz = torch.zeros(n, 3, device=dev); dpz[:, 2] = 0.002
            dq_ = u._ik_step(dpz)
            t_ = arm_prev.clone()
            t_ = torch.where((rl_k == 0)[:, None], arm_prev + dq_, t_)
            frac = (rl_cnt.float() * dt / 2.5).clamp(0, 1)
            sgn = torch.where(rl_q5 <= 0, 1.0, -1.0)  # 뒤집은 채 쥠 (joint5 ≈ ±π) -> 0 쪽으로 180° = 정상 방향
            t_[:, 4] = torch.where(rl_k == 1, rl_q5 + sgn * math.pi * (0.5 - 0.5 * torch.cos(math.pi * frac)), t_[:, 4])
            tgt[mrl] = t_[mrl]
            grip[mrl] = -1.0
            u.actions[mrl, -1] = -1.0
            rl_cnt = torch.where(mrl, rl_cnt + 1, rl_cnt)
            start_roll = mrl & (rl_k == 0) & (rl_cnt > 25)
            rl_q5 = torch.where(start_roll, q[:, 4], rl_q5)
            az_ = quat_apply(u.puck_quat_w(), ez)[:, 2].abs()
            done_roll = mrl & (rl_k == 1) & (rl_cnt > int(2.8 / dt))
            if not u.cfg.grip_roll_pi:  # (v1) 정상 방향으로 쥐었으면 롤 뒤 joint5 = ±π 라 운반 경로도 옮김. v2 (뒤집어 쥠) 는 롤 뒤 0 = 팀 경로 그대로
                j5_off = torch.where(done_roll, sgn * math.pi, j5_off)
            ok_v = mrl & (rl_k == 2) & (rl_cnt > int(0.3 / dt))
            rl_ok |= ok_v & (az_ > 0.95)
            rl_fail |= ok_v & (az_ <= 0.95)
            nx = start_roll | done_roll | ok_v
            rl_k = torch.where(nx, rl_k + 1, rl_k)
            rl_cnt = torch.where(nx, torch.zeros_like(rl_cnt), rl_cnt)
            for i in torch.nonzero(done_roll).flatten().tolist():
                print(f"[Chain45] t={k * dt:5.1f}s env{i} 롤 180° 끝, 퍽 긴 축 기울기 {math.degrees(math.asin(min(1.0, float(az_[i])))):.0f}° (90 = 세로)", flush=True)
        m0p = m0 & (rg < 0) & (rp < 0) & (rl_k < 0)  # 정책이 움직이는 대 (다시 집기 · A 각본 중이 아님)
        if m0p.any() and not args_cli.lie:
            dq0 = u._ik_step(a_sp[:, :3] * u.cfg.cart_scale)
            tgt[m0p] = (arm_prev + dq0)[m0p]
            grip[m0p] = a_sp[m0p, -1]
            u.prev_actions[m0p] = u.actions[m0p]
            u.actions[m0p] = a_sp[m0p]
        mr = m0 & (rg >= 0)
        if mr.any():  # 다시 집기: 0 퍽 바닥을 처음 자리 (+2 mm) 로 내림 -> 1 벌림 0.5 s -> 2 잡는 점 4 cm 위로 -> 정책으로 다시 집기
            dpr = torch.zeros(n, 3, device=dev)
            e0 = torch.cat((pb0[:, :2], pb0[:, 2:3] + 0.002), dim=1) - s["pb"]
            dpr = torch.where((rg == 0)[:, None], (e0 * 0.5).clamp(-0.002, 0.002), dpr)
            dpr[:, 2] = torch.where(rg == 2, torch.full_like(dpr[:, 2], 0.002), dpr[:, 2])
            dqr = u._ik_step(dpr)
            tgt[mr] = (arm_prev + dqr)[mr]
            m3r = mr & (rg == 3)  # 3: 집기 시작 자세로 돌아감 (정책이 학습 때와 같은 자세에서 다시 시작)
            if m3r.any():
                dqh = q_ready - arm_prev
                stp_ = args_cli.empty_speed * dt
                mm_ = dqh.abs().amax(dim=1, keepdim=True)
                tgt[m3r] = (arm_prev + dqh * torch.clamp(stp_ / mm_.clamp(min=1e-9), max=1.0))[m3r]
            grip[mr] = torch.where(rg[mr] == 0, -1.0, 1.0)
            u.actions[mr, -1] = grip[mr]
            rg_cnt = torch.where(mr, rg_cnt + 1, rg_cnt)
            n0 = (rg == 0) & ((e0.norm(dim=1) < 0.0015) | (rg_cnt > int(3.0 / dt)))
            n1 = (rg == 1) & (rg_cnt > int(0.5 / dt))
            n2 = (rg == 2) & (rg_cnt > 20)
            n3 = (rg == 3) & ((q - q_ready).abs().amax(dim=1) < 0.03)
            nx = mr & (n0 | n1 | n2 | n3)
            rg = torch.where(nx, rg + 1, rg)
            rg_cnt = torch.where(nx, torch.zeros_like(rg_cnt), rg_cnt)
            back = rg >= 4
            if back.any():  # 끝 -> 정책으로 (행동 · 이전 행동 초기화 = 학습 때 리셋과 같게)
                u.actions[back] = 0.0
                u.actions[back, -1] = 1.0
                u.prev_actions[back] = u.actions[back]
            rg = torch.where(back, torch.full_like(rg, -1), rg)
        if LIE_SCRIPT and k == 3:  # 리셋 뒤 자리 잡으면 퍽 인식값으로 관절 자세 3 개를 IK 로 풂 (팀 URDF FK, tools_ik/td_ik)
            lie_plan, lie_bad = plan_lie_grasp(s)
            qp = q.clone()
            bias = u.arm_target - q
        if m0.any() and LIE_SCRIPT and k > 3:  # 각본 집기: 0 위 6 cm 로 (롤 = 긴 축) -> 맞춤 확인 -> 1 곧게 내림 -> 2 쥠 0.5 s -> 3 4 cm 들기 -> 4 끝
            if args_cli.roll_demo:  # 들기 끝 (ls 4) 부터: 들기 자세에서 joint5 만 −90° (2 s 동안), 그 뒤 멈춤
                if not hasattr(main, "_rd_t0"):
                    main._rd_t0 = torch.full((n,), -1, dtype=torch.long, device=dev)
                t0_ = main._rd_t0
                t0_ = torch.where((ls >= 4) & (t0_ < 0), torch.full_like(t0_, k), t0_)
                main._rd_t0 = t0_
                frac = ((k - t0_).float() * dt / 2.0).clamp(0.0, 1.0) * (t0_ >= 0).float()
                q_rd = lie_plan[:, 2].clone()
                q_rd[:, 4] = (q_rd[:, 4] - math.pi / 2 * frac).clamp(-2.8, 2.8)
            if HANG:  # 단계 -> 경유 자세: 0 위, 1 쥐기, 2 약하게 쥠, 3 들기, 4 매달림 기다림, 5 롤, 6 45° 숙임, 7 세게 쥠
                wmap = torch.tensor([0, 1, 1, 2, 2, 3, 4, 4, 4], device=dev)
                wp = lie_plan[torch.arange(n, device=dev), wmap[ls.clamp(max=8)]]
            else:
                wp = lie_plan[torch.arange(n, device=dev), torch.clamp(torch.where(ls >= 3, 2, torch.where(ls >= 1, 1, 0)), max=2)]
            dqp = wp - qp
            mm_ = dqp.abs().amax(dim=1)
            stp = args_cli.speed * dt * torch.where(ls == 1, 0.4, 1.0)  # 내릴 때 천천히
            if HANG:  # 들기 · 롤 · 숙이기는 천천히 (매달린 퍽이 흔들리지 않게)
                stp = args_cli.speed * dt * torch.where(ls == 1, 0.4, torch.where((ls == 3) | (ls == 6), 0.25, torch.where(ls == 5, 0.5, 1.0)))
            reach = mm_ <= stp
            qp = torch.where(m0[:, None], torch.where(reach[:, None], wp, qp + dqp * (stp / mm_.clamp(min=1e-9))[:, None]), qp)
            if args_cli.ki > 0:
                bias = torch.where(m0[:, None], (bias + args_cli.ki * dt * (qp - q)).clamp(-args_cli.bias_max, args_cli.bias_max), bias)
            tgt[m0] = (qp + bias)[m0]
            if args_cli.roll_demo:
                rdm = m0 & (ls >= 4)
                tgt[rdm] = (q_rd + bias)[rdm]
                if k % 15 == 0 and rdm.any():
                    az_ = quat_apply(u.puck_quat_w(), torch.tensor([[0.0, 0.0, 1.0]], device=dev).expand(n, 3))[:, 2].abs()
                    print(f"[RollDemo] t={k * dt:4.1f}s 손목 롤 돌린 양 {[round(math.degrees(math.pi / 2 * v)) for v in frac[:4].tolist()]}° | 퍽 긴 축 기울기 (0° = 누움, 90° = 섬) {[round(math.degrees(math.asin(min(1.0, v)))) for v in az_[:4].tolist()]}° | 집게 방향 z {[round(v, 2) for v in quat_apply(r.data.body_link_quat_w.torch[:, u._a5], torch.tensor([[1.0, 0.0, 0.0]], device=dev).expand(n, 3))[:4, 2].tolist()]}", flush=True)
            grip[m0] = torch.where(ls[m0] >= 2, -1.0, 1.0)
            G_b = lie_G  # 잡는 점 목표 (base)
            off_b = torch.where(ls[:, None] == 0, 0.06, torch.where(ls[:, None] >= 3, 0.04, 0.0)) * torch.tensor([[0.0, 0.0, 1.0]], device=dev)
            pos_err = (G_b + off_b - s["gb"]).norm(dim=1)
            u.psi = torch.where(m0, lie_psi, u.psi)
            u.phi = torch.where(m0, torch.zeros_like(u.phi), u.phi)
            qt_ = quat_mul(r.data.root_link_quat_w.torch, quat_from_matrix(u.target_R_base()))
            rot_err = quat_box_minus(qt_, r.data.body_link_quat_w.torch[:, u._a5]).norm(dim=1)
            ls_cnt = torch.where(m0, ls_cnt + 1, ls_cnt)
            arrived = reach & (pos_err < 0.003) & (rot_err < math.radians(3.0))
            okA = (ls == 0) & arrived  # 위에서 위치 · 롤이 맞았는지 확인
            okB = (ls == 1) & (reach & (pos_err < 0.002) | (ls_cnt > int(3.0 / dt)))
            okC = (ls == 2) & (ls_cnt > int(0.5 / dt))
            okD = (ls == 3) & reach & (pos_err < 0.005)
            if HANG:  # 3 들기 끝 -> 4 매달림 (퍽 긴 축 세로 0.3 s, 또는 3 s) -> 5 롤 -> 6 숙임 -> 7 세게 쥠 0.5 s -> 8 끝
                az_ = quat_apply(u.puck_quat_w(), torch.tensor([[0.0, 0.0, 1.0]], device=dev).expand(n, 3))[:, 2].abs()
                hang_cnt = torch.where(m0 & (ls == 4) & (az_ > 0.9), hang_cnt + 1, torch.zeros_like(hang_cnt))
                okD = (ls == 3) & reach
                okE = (ls == 4) & ((hang_cnt > int(0.3 / dt)) | (ls_cnt > int(3.0 / dt)))
                okF = ((ls == 5) | (ls == 6)) & reach & (ls_cnt > 3)
                okG = (ls == 7) & (ls_cnt > int(0.5 / dt))
                okD = okD | okE | okF | okG
            cntA = torch.where(okA, ls_cnt, torch.zeros_like(ls_cnt))
            adv_s = m0 & ~lie_bad & ((okA & (lie_ok_cnt >= 5)) | okB | okC | okD)
            lie_ok_cnt = torch.where(okA, lie_ok_cnt + 1, torch.zeros_like(lie_ok_cnt))
            ls = torch.where(adv_s, ls + 1, ls)
            ls_cnt = torch.where(adv_s, torch.zeros_like(ls_cnt), ls_cnt)
            if args_cli.debug_lie and k % 10 == 0:
                fz = r.data.joint_pos.torch[:, u._gl[0]]
                print(f"[LieDbg] t={k * dt:4.1f} 단계 {ls[:4].tolist()} 위치 오차 mm {[round(v * 1000, 1) for v in pos_err[:4].tolist()]} 자세 오차 ° {[round(math.degrees(v), 1) for v in rot_err[:4].tolist()]}"
                      f" 손가락 {[round(v, 4) for v in fz[:4].tolist()]} 퍽 들림 mm {[round(v * 1000) for v in u.lift()[:4].tolist()]} 퍽 축 |z| {[round(v, 2) for v in quat_apply(u.puck_quat_w(), torch.tensor([[0.0, 0.0, 1.0]], device=dev).expand(n, 3))[:4, 2].abs().tolist()]} IK 실패 {lie_bad[:4].int().tolist()}", flush=True)
        if m0.any() and args_cli.lie and not LIE_SCRIPT:  # LP-8 행동 해석 (학습 때와 같게: 집게 요 ψ · 숙임 φ + 자세 목표 IK) -> 들면 관절 보간으로 세우기
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
                Ns = max(1, int(args_cli.stand_s / dt))
                kk = lie_k.float().unsqueeze(1)

                def sm2(x):
                    x = x.clamp(0.0, 1.0)
                    return 0.5 - 0.5 * torch.cos(math.pi * x)
                qs = torch.where(kk < Ns, lie_q0 * (1 - sm2(kk / Ns)) + lie_via * sm2(kk / Ns), lie_via * (1 - sm2((kk - Ns) / Ns)) + lie_to * sm2((kk - Ns) / Ns))
                tgt[ms] = qs[ms]
                grip[ms] = -1.0
                lie_k[ms] += 1
                stood |= ms & (lie_k >= 2 * Ns)
        # 2 운반 = 관절 경로 (쥔 채)
        if m1.any():
            done1 = follow(path_go, len_go, m1, held_speed[slot])
            tgt[m1] = (qp + bias)[m1]
            grip[m1] = -1.0
        else:
            done1 = torch.zeros(n, dtype=torch.bool, device=dev)
        # 3 놓기 = 손 IK: 퍽 바닥을 칸 축 · 놓는 높이로 (자세 = 집게 45° 아래·바깥, 집기 환경 _ik_step 과 같은 목표)
        if m2.any():
            se = state_est() if args_cli.est else s
            e = tgt_pb - se["pb"]
            dpb = e * 0.5
            nrm = dpb.norm(dim=1, keepdim=True)
            dpb = dpb * torch.clamp(args_cli.ik_step / nrm.clamp(min=1e-9), max=1.0)
            ln, lt = lean_of(se)
            if args_cli.upright:  # 퍽 위가 바깥으로 기울면 피치를 줄임 (피치를 키우면 쥔 퍽 위가 바깥으로 감)
                pitch_t = torch.where(m2, (pitch_t - 0.3 * ln).clamp(pitch0 - plim, pitch0 + plim), pitch_t)
            al_lean0 = torch.where((m2 & (pcnt == 0))[:, None], torch.rad2deg(torch.stack((ln, lt), dim=1)), al_lean0)
            dq2 = ik_pitch(dpb, pitch_t)
            tgt[m2] = (arm_prev + dq2)[m2]
            grip[m2] = -1.0
        # 4 벌림 (팔 그대로)
        grip[m3] = 1.0
        # 5 복귀 = 관절 경로 (빈 집게)
        if m4.any():
            done4 = follow(path_bk, len_bk, m4, torch.full((n,), args_cli.empty_speed, device=dev))
            tgt[m4] = (qp + bias)[m4]
            grip[m4] = 1.0
        else:
            done4 = torch.zeros(n, dtype=torch.bool, device=dev)
        grip[phase >= 5] = 1.0
        if HANG:  # 매달리는 동안 (2 ~ 6) 약하게, 그 뒤 세게
            loose = (phase == 0) & (ls >= 2) & (ls <= 6)
            u.cfg.gripper_closed = torch.where(loose[:, None], torch.full((n, 1), args_cli.hang_grip, device=dev), torch.full((n, 1), fc0, device=dev))
        u.cfg.gripper_open = torch.where((phase >= 2)[:, None], torch.full((n, 1), FINGER_RELEASED if args_cli.release_open is None else args_cli.release_open, device=dev), torch.full((n, 1), go0, device=dev))
        u.arm_target[:] = torch.clamp(torch.clamp(tgt, q - lead, q + lead), u._lo, u._hi)
        # 안전 계층 (chain_play 와 같음): 집기 중 관절 1 을 퍽 방향 ±0.6 rad, 집기 실패면 그 자리 멈춤
        lo1, hi1 = th_init - 0.6, th_init + 0.6
        u.arm_target[:, 0] = torch.where(m0, torch.maximum(torch.minimum(u.arm_target[:, 0], hi1), lo1), u.arm_target[:, 0])
        if not args_cli.lie:  # 집기 중 관절 3 상한 (팔을 접어 몸체에 부딪히지 않게)
            u.arm_target[:, 2] = torch.where(m0, torch.clamp(u.arm_target[:, 2], max=args_cli.pick_j3_max), u.arm_target[:, 2])
        u.arm_target[aborted] = abort_q[aborted]
        u.arm_target[phase >= 5] = arm_prev[phase >= 5]
        u.actions[:, -1] = torch.where(aborted, torch.ones_like(grip), grip)
        # 처짐 보정 (관절 경로 단계만): bias += ki dt (계획 − 측정)
        mj = m1 | m4
        if args_cli.ki > 0 and mj.any():
            bias = torch.where(mj[:, None], (bias + args_cli.ki * dt * (qp - q)).clamp(-args_cli.bias_max, args_cli.bias_max), bias)
        draw = args_cli.gui and (k % max(1, args_cli.render_every) == 0)
        for j_ in range(cfg.decimation):
            u._apply_action()
            u.scene.write_data_to_sim()
            u.sim.step(render=draw and j_ == cfg.decimation - 1)
            u.scene.update(u.physics_dt)
        u.episode_length_buf[:] = 0
        arm_prev = u.arm_target.clone()
        if k == 2 and args_cli.occ > 0:
            place_occ()
        # 기록
        ph = phase.clamp(max=NP - 1).unsqueeze(1)
        fa, ff, fp = force("arm_body_contact"), force("finger_body_contact"), force("puck_body_contact")
        fo = torch.maximum(force("robot_occ_contact"), force("puck_occ_contact")) if k >= 10 else torch.zeros(n, device=dev)
        for buf, f in ((f_arm, fa), (f_fing, ff), (f_puck, fp), (f_occ, fo)):
            buf.scatter_reduce_(1, ph, f.unsqueeze(1), reduce="amax")
        newhit = (hit_first < 0) & ((fa > 1.0) | ((ff > 1.0) & (phase != 0)))
        hit_q = torch.where(newhit[:, None], q, hit_q)
        hit_first = torch.where(newhit, phase, hit_first)
        if not args_cli.lie:
            toppled |= m0 & (u.puck_up() < 0.7) & (rl_k < 0)
        sd_ph.scatter_reduce_(1, phase.clamp(max=5).unsqueeze(1), u.self_dist().unsqueeze(1), reduce="amin")
        ovp_ = u.armcol.depth_pairs()
        ovd = ovp_.amax(dim=1)
        ov_pair = torch.maximum(ov_pair, ovp_)
        ov_q = torch.where((ovd > ov_qd)[:, None], q, ov_q)
        ov_qd = torch.maximum(ov_qd, ovd)
        ov_ph.scatter_reduce_(1, phase.clamp(max=5).unsqueeze(1), ovd.unsqueeze(1), reduce="amax")
        nb_touch |= m0 & u._contact("arm_nb_contact")
        _st = torch.where(m0, rp, 100 + phase)
        nb_first = torch.where((nb_first == -99) & u._contact("arm_nb_contact"), _st, nb_first)
        tp_first = torch.where((tp_first == -99) & (k > 30) & (u.puck_up() < 0.7) & ~((rp >= 0) & (rp <= 5)), _st, tp_first)
        s = state()
        lost_now = (phase >= 1) & (phase <= 2) & ~s["held"] & (lost_phase < 0)
        lost_phase = torch.where(lost_now, phase, lost_phase)
        ever_held |= m0 & s["held"] & (r.data.joint_pos.torch[:, u._gl[0]] < -0.003)
        if args_cli.est and args_cli.est_z == "box" and not args_cli.lie and not LIEROLL:  # 손가락을 닫아 멈췄고 퍽이 아직 상자 위 -> 퍽 중심 높이 = 상자 윗면 + 30 mm
            fg_ = r.data.joint_pos.torch[:, u._gl[0]]
            closed_ = (u.actions[:, -1] < 0) & (fg_ > u.cfg.grasp_finger_range[0]) & (fg_ < u.cfg.grasp_finger_range[1])
            onb_ = m0 & closed_ & s["held"] & (u.lift() < 0.002) & (rp < 0)  # 닫혀 쥐었고 퍽이 아직 상자 위: 들기 전 마지막 순간까지 계속 새로 잼
            est_locked = torch.where(m0 & (~s["held"] | (u.actions[:, -1] > 0)), torch.zeros_like(est_locked), est_locked)  # 놓쳤거나 벌리면 다시
            ez_dbg |= m0 & closed_
            ez_dbg2 |= m0 & closed_ & s["held"]
            ez_dbg3 |= onb_
            if onb_.any():
                pa_, qa_ = r.data.body_link_pos_w.torch[:, u._a5], r.data.body_link_quat_w.torch[:, u._a5]
                upw_ = quat_apply(r.data.root_link_quat_w.torch, ez)
                g_ = u.grasp_w()
                zc_ = u.scene.env_origins[:, 2] + BOX_H + PUCK_H / 2  # 상자 (바닥 위 70 mm) 윗면 + 퍽 반높이
                c_ = g_ - upw_ * ((g_[:, 2] - zc_) / upw_[:, 2].clamp(min=0.5))[:, None]
                est_c = torch.where(onb_[:, None], quat_apply_inverse(qa_, c_ - pa_), est_c)
                est_u = torch.where(onb_[:, None], quat_apply_inverse(qa_, upw_), est_u)
                pk_close = torch.where(onb_[:, None], quat_apply_inverse(qa_, u.puck_pos_w() - pa_), pk_close)
                est_locked |= onb_
        if args_cli.lie:  # 누운 퍽: 들었다가 놓침 · 시간 끝 · 각본 IK 실패 (원래 누워 있으니 넘어짐 판정 없음)
            ab = m0 & ~aborted & ((ever_held & ~s["held"]) | ((k + 1) * dt > args_cli.pick_timeout + 4.0) | lie_bad)
        else:
            ab = m0 & ~aborted & (rg < 0) & (rp < 0) & (rl_k < 0) & ((u.puck_up() < 0.7) | ((k + 1) * dt - seq_t0 > args_cli.pick_timeout + 4.0 * rg_n.float()))
            ab |= m0 & ~aborted & rp_fail  # A 각본 실패 (IK · 세워 놓기)
            ab |= m0 & ~aborted & (fa > 1.0)  # 2026-11-04 안전: 집기 중 팔-몸체 1 N 넘으면 그 자리에서 멈춤 (뒤집은 집게 정책이 1396 N)
        ab |= ~aborted & (phase < 5) & (ovd > u.cfg.self_box_tol)  # 2026-11-06 안전: 어느 단계든 팔끼리 겹치면 그 자리에서 멈춤
        abort_q[ab] = q[ab]
        aborted |= ab
        # 단계 전환
        pcnt += 1
        se = state_est() if args_cli.est else s
        e2 = tgt_pb - se["pb"]
        ln_, lt_ = lean_of(se)
        e2_true = tgt_pb - s["pb"]
        lnt_, ltt_ = lean_of(s)
        tr_lean_max = torch.where(m1, torch.maximum(tr_lean_max, torch.rad2deg(lnt_).abs()), tr_lean_max)
        ok2 = (e2[:, :2].norm(dim=1) < 0.0015) & (e2[:, 2].abs() < 0.002) & ((ln_.abs() < math.radians(1.5)) | (not args_cli.upright))
        if args_cli.lie:
            fing_l = r.data.joint_pos.torch[:, u._gl[0]]
            hold_l = (fing_l > u.cfg.grasp_finger_range[0]) & (fing_l < u.cfg.grasp_finger_range[1]) & s["held"]
            lie_cnt = torch.where(m0 & (lie_k < 0) & hold_l & (u.lift() > u.cfg.lift_ok), lie_cnt + 1, torch.zeros_like(lie_cnt))
            go_s = m0 & (lie_k < 0) & (lie_cnt >= 9) & ~aborted & torch.tensor(not LIE_SCRIPT, device=dev)
            if go_s.any():  # 세우기 시작: 지금 -> 경유 (어깨 뒤로) -> 세움 (집게 수평 · 바깥, 관절5 0)
                qn_ = r.data.joint_pos.torch[:, u._arm]
                th_a = torch.atan2(s["gb"][:, 1], s["gb"][:, 0] - u.cfg.joint1_x)
                v_ = qn_.clone(); v_[:, 0] = th_a; v_[:, 1:4] = torch.tensor([[-0.7, 0.25, 0.2]], device=dev)
                t_ = v_.clone(); t_[:, 1:4] = torch.tensor([[0.30, 1.42, -1.72]], device=dev); t_[:, 4] = 0.0
                lie_q0 = torch.where(go_s[:, None], qn_, lie_q0)
                lie_via = torch.where(go_s[:, None], v_, lie_via)
                lie_to = torch.where(go_s[:, None], t_, lie_to)
                lie_k = torch.where(go_s, torch.zeros_like(lie_k), lie_k)
            c0 = m0 & ~aborted & ((ls >= (8 if HANG else 4)) if LIE_SCRIPT else (stood & (u.puck_up() > 0.95))) & s["held"] & (not args_cli.roll_demo)
        else:
            c0 = m0 & ~aborted & (rg < 0) & ((rp < 0) | (rp >= 14)) & (u.lift() > 0.03) & s["held"] & (u.puck_up() > 0.9)
            if LIEROLL:  # 정책이 쥐고 3 cm (누운 채) 0.5 s -> 롤 단계 시작, 롤 끝 · 세로 확인 -> 운반으로
                go_rl = m0 & ~aborted & (rl_k < 0) & c0
                rl_hold = torch.where(go_rl, rl_hold + 1, torch.zeros_like(rl_hold))
                st_ = go_rl & (rl_hold >= 15)
                rl_k = torch.where(st_, torch.zeros_like(rl_k), rl_k)
                rl_cnt = torch.where(st_, torch.zeros_like(rl_cnt), rl_cnt)
                aborted |= m0 & rl_fail
                abort_q = torch.where((m0 & rl_fail)[:, None], q, abort_q)
                c0 = m0 & ~aborted & (rl_k >= 3) & rl_ok & s["held"]
        c2 = m2 & (ok2 | (pcnt > int(4.0 / dt)) | (ff > 5.0))  # 2026-11-04: 내리다 손가락이 몸체 (칸 벽) 에 5 N 넘게 닿으면 거기서 놓음
        c3 = m3 & (pcnt > int(0.6 / dt))
        go = torch.where(m0, c0, torch.where(m1, done1, torch.where(m2, c2, torch.where(m3, c3, done4))))
        go &= ~(m1 & (lost_phase == 1))  # 운반 중 놓치면 그 자리 (끝 판정에서 실패)
        cnt = torch.where(go, cnt + 1, torch.zeros_like(cnt))
        need = torch.where(m0, 15, torch.where(m2, 3, 1))
        adv = go & (cnt >= need) & (phase < 5)
        if args_cli.regrasp_tilt > 0 and not args_cli.lie:  # 집기 끝 기울기 확인 -> 크면 내려놓고 다시
            tilt_now = torch.rad2deg(torch.acos(s["u"][:, 2].clamp(-1, 1)))
            bad = adv & m0 & (tilt_now > args_cli.regrasp_tilt) & (rg_n < args_cli.regrasp_max)
            if bad.any():
                rg_tilts += [round(v, 1) for v in tilt_now[bad].tolist()]
                for i in torch.nonzero(bad).flatten().tolist():
                    print(f"[Chain45] t={(k + 1) * dt:5.1f}s env{i} 쥔 퍽 기울기 {tilt_now[i]:.1f}° > {args_cli.regrasp_tilt}° -> 내려놓고 다시 집기 ({int(rg_n[i]) + 1} 번째)", flush=True)
                rg = torch.where(bad, torch.zeros_like(rg), rg)
                rg_cnt = torch.where(bad, torch.zeros_like(rg_cnt), rg_cnt)
                rg_n = torch.where(bad, rg_n + 1, rg_n)
                cnt = torch.where(bad, torch.zeros_like(cnt), cnt)
                adv = adv & ~bad
        if adv.any():
            al_tmo |= adv & m2 & ~ok2
            al_end = torch.where((adv & m2)[:, None], e2_true, al_end)
            al_tilt = torch.where(adv & m2, torch.rad2deg(torch.acos(s["u"][:, 2].clamp(-1, 1))), al_tilt)
            rel_q = torch.where((adv & m2)[:, None], r.data.joint_pos.torch[:, u._arm], rel_q)
            al_lean = torch.where((adv & m2)[:, None], torch.rad2deg(torch.stack((lnt_, ltt_), dim=1)), al_lean)
            est_store(adv & m0)
            pk_lean = torch.where((adv & m0)[:, None], torch.rad2deg(torch.stack((lnt_, ltt_), dim=1)), pk_lean)
            pk_dz = torch.where(adv & m0, (s["gb"][:, 2] - s["pc"][:, 2]) * 1000, pk_dz)
            into_j = adv & (m0 | m3)  # 다음이 관절 경로: 계획 = 지금 측정, bias = 지금 처짐 (이어서 매끄럽게)
            qn_ = r.data.joint_pos.torch[:, u._arm]
            qp = torch.where(into_j[:, None], qn_, qp)
            bias = torch.where(into_j[:, None], u.arm_target - qn_, bias)
            idx = torch.where(into_j, torch.zeros_like(idx), idx)
            phase[adv] += 1
            cnt[adv] = 0
            pcnt[adv] = 0
            t_enter[adv, phase[adv]] = (k + 1) * dt
            for i in torch.nonzero(adv).flatten().tolist():
                sl = SLOTS[int(slot[i])]
                print(f"[Chain45] t={(k + 1) * dt:5.1f}s env{i} ({sl['row']}열 {sl['index']}) -> {PHASES[int(phase[i])]}", flush=True)
            fin = adv & (phase == 5)
            if SEQ > 1 and fin.any():  # 이번 퍽 결과 (복귀 끝 순간): 칸 축 3 mm · 바닥 3 mm · 5° 안
                dxy_ = torch.linalg.norm(s["pb"][:, :2] - sxyz[slot, :2], dim=1)
                dz_ = s["pb"][:, 2] - sxyz[slot, 2]
                tl_ = torch.rad2deg(torch.acos(s["u"][:, 2].clamp(-1, 1)))
                okp = (dxy_ < 0.003) & (dz_.abs() < 0.003) & (tl_ < 5.0)
                for i in torch.nonzero(fin).flatten().tolist():
                    seq_ok[i, int(seq_i[i])] = int(okp[i])
                    print(f"[Chain45]   env{i} 퍽 {int(seq_i[i]) + 1} {'앉음' if bool(okp[i]) else '못 앉음'} (축 {dxy_[i] * 1000:.1f} mm, 바닥 위 {dz_[i] * 1000:+.1f} mm, 기울기 {tl_[i]:.1f}°)", flush=True)
                nxt = fin & okp & (seq_i + 1 < SEQ)
                seq_swap = torch.where(nxt, torch.full_like(seq_swap, k + 10), seq_swap)
        if k % 90 == 89:
            print(f"[Chain45] t={(k + 1) * dt:5.1f}s 단계 {[int(v) + 1 for v in phase.tolist()][:16]}", flush=True)

    # ---- 결과 (1 s 더 기다린 뒤) ----
    for _ in range(int(1.0 / dt)):
        u._apply_action()
        u.scene.write_data_to_sim()
        u.sim.step(render=False)
        u.scene.update(u.physics_dt)
    s = state()
    sl_xy, floor = sxyz[slot, :2], sxyz[slot, 2]
    dxy = torch.linalg.norm(s["pb"][:, :2] - sl_xy, dim=1)
    dz = s["pb"][:, 2] - floor
    tilt = torch.rad2deg(torch.acos(s["u"][:, 2].clamp(-1, 1)))
    seated = (dxy < 0.003) & (dz.abs() < 0.003) & (tilt < 5.0) & (phase >= 4)
    qe = (r.data.joint_pos.torch[:, u._arm] - q_ready).abs().amax(dim=1)
    returned = phase >= 5
    occ_moved = torch.zeros(n, device=dev)
    for j in range(min(N_OCC, len(SLOTS))):
        ob = u.scene[f"occ{j}"]
        pj = to_bl(ob.data.root_pos_w.torch)
        sj = torch.tensor([SLOTS[j]["x"], SLOTS[j]["y"]], device=dev)
        occ_moved = torch.where(occ_on[:, j], torch.maximum(occ_moved, torch.linalg.norm(pj[:, :2] - sj, dim=1)), occ_moved)
    occ_hit = f_occ.amax(dim=1) > 1.0

    def why(i):
        if float(ov_ph[i].max()) > u.cfg.self_box_tol:
            return "팔끼리 겹침"
        if bool(seated[i]) and bool(returned[i]) and not bool(occ_hit[i]) and float(f_arm[i].max()) <= 1.0:
            return "성공"
        if int(phase[i]) == 0 and args_cli.lie:
            return "집기·세우기: 놓침" if bool(ever_held[i]) else "집기: 못 쥠"
        if int(phase[i]) == 0:
            return "집기: 넘어뜨림" if bool(toppled[i]) else ("집기: 옆 퍽 닿음·못 쥠" if bool(nb_touch[i]) else "집기: 못 쥠 (시간 끝)")
        if int(lost_phase[i]) == 1:
            return "운반: 놓침"
        if int(lost_phase[i]) == 2:
            return "놓기 IK: 놓침"
        if not bool(seated[i]):
            return "칸에 못 앉음"
        if float(f_arm[i].max()) > 1.0:
            return "팔-몸체 접촉"
        if bool(occ_hit[i]):
            return "앞 칸 퍽 닿음"
        return "복귀 못 끝냄"

    from collections import Counter
    reasons = [why(i) for i in range(n)]
    if SEQ > 1:
        for i in range(n):
            if int(phase[i]) < 5 and int(seq_ok[i, int(seq_i[i])]) < 0:
                seq_ok[i, int(seq_i[i])] = 0
        run = [(seq_ok[i] == 1).long().cumprod(0).sum().item() for i in range(n)]
        print(f"[Chain45] **연속 채우기 (퍽 {SEQ} 개 목표)**: 대마다 연속으로 앉힌 수 {run}, 평균 {sum(run) / n:.1f}, 다 채운 대 {sum(1 for v in run if v >= SEQ)}/{n}", flush=True)
        print(f"[Chain45]   첫 실패 이유 (끝 상태): {dict(Counter(reasons[i] for i in range(n) if run[i] < SEQ))}", flush=True)
    for i in range(n):
        sl = SLOTS[int(slot[i])]
        print(f"[Chain45]   env{i:2d} {sl['row']}열 {sl['index']} | {reasons[i]:16s} | 단계 {int(phase[i]) + 1} | 앉음 {'O' if bool(seated[i]) else 'X'} 축 {dxy[i] * 1000:5.1f} mm"
              f" 바닥 위 {dz[i] * 1000:+6.1f} mm 기울기 {tilt[i]:4.1f}° | 놓는 순간 퍽-목표 mm {[round(v * 1000, 1) for v in al_end[i].tolist()]}{' (시간끝)' if bool(al_tmo[i]) else ''} 기울기 {al_tilt[i]:.1f}° (팔 방향 {al_lean[i, 0]:+.1f} 접선 {al_lean[i, 1]:+.1f}, 놓기 시작 {al_lean0[i, 0]:+.1f} {al_lean0[i, 1]:+.1f}, 피치 {math.degrees(pitch_t[i]):.0f})"
              f" | 팔-몸체 N 단계별 {[round(v) for v in f_arm[i].tolist()]} 손가락-몸체 {[round(v) for v in f_fing[i].tolist()]} 퍽-몸체 {[round(v) for v in f_puck[i].tolist()]}"
              f" | 앞 칸 퍽 {int(occ_on[i].sum())} 개 닿음 {f_occ[i].max():.1f} N 밀림 {occ_moved[i] * 1000:.1f} mm | 복귀 관절 차 {qe[i]:.3f}", flush=True)
        print(f"[Chain45]        쥔 퍽: 집기 끝 기울기 (팔 방향 {pk_lean[i, 0]:+.1f}°, 접선 {pk_lean[i, 1]:+.1f}°) 잡는 점-퍽 중심 높이 {pk_dz[i]:+.1f} mm"
              f" -> 운반 중 팔 방향 최대 {tr_lean_max[i]:.1f}° -> 놓기 시작 {al_lean0[i, 0]:+.1f}°"
              f" | 손 안 퍽 가정 오차 mm (앞 {est_err[i, 0]:+.1f}, 손가락 {est_err[i, 1]:+.1f}, 위 {est_err[i, 2]:+.1f})", flush=True)
        if int(hit_first[i]) >= 0:
            print(f"[Chain45]        처음 몸체 닿음: {PHASES[int(hit_first[i])]} q {[round(v, 2) for v in hit_q[i].tolist()]}", flush=True)
    print(f"[Chain45] 끝 분류 (대 {n}): {dict(Counter(reasons))}", flush=True)
    if args_cli.est and args_cli.est_z == "box":
        print(f"[Chain45] --est_z box: 집기 끝에 닫는 순간 높이를 쓴 대 {int(est_used.sum())}/{n} (나머지는 예전 가정)"
              f" | 판정 단계별 대: 손가락 닫혀 쥠 범위 {int(ez_dbg.sum())}, +퍽 잡는 점 3 cm 안 {int(ez_dbg2.sum())}, +퍽 아직 상자 위 (2 mm) {int(ez_dbg3.sum())}", flush=True)
    if args_cli.est:  # 2026-11-06: 손 안 퍽 가정 오차 (집기 끝, 마지막 퍽) 를 결과별로
        for nm_ in ("성공", "칸에 못 앉음"):
            ii_ = [i for i in range(n) if reasons[i] == nm_]
            if ii_:
                ea_ = est_err[ii_].abs()
                print(f"[Chain45] 손 안 퍽 가정 오차 |mm| 중앙 ({nm_} {len(ii_)} 대): 앞 {ea_[:, 0].median():.1f} 손가락 {ea_[:, 1].median():.1f} 위 {ea_[:, 2].median():.1f}"
                      + (f" | 닫은 뒤 손 안 미끄러짐 |mm| 중앙 앞 {pk_slip[ii_].abs()[:, 0].median():.1f} 위 {pk_slip[ii_].abs()[:, 2].median():.1f}" if args_cli.est_z == "box" else "")
                      + f" | 앞·손가락 합 3 mm 넘은 대 {int((torch.linalg.norm(ea_[:, :2], dim=1) > 3).sum())}", flush=True)
    if LIEROLL:
        print(f"[Chain45] 누운 퍽 45° 쥐기 + 롤 180°: 세로 확인 {int(rl_ok.sum())}/{n}, 롤 뒤 세로 아님 {int(rl_fail.sum())}", flush=True)
    if REGRASP:
        print(f"[Chain45] A (세워 내려놓고 다시 집기): 세워 놓기 성공 {int(rp_stood.sum())}/{n}, 각본 실패 {int(rp_fail.sum())}", flush=True)
        print(f"[Chain45] A 옆 퍽 처음 닿은 단계 (−99 안 닿음, 0 ~ 14 각본, −1 정책 다시 집기, 10x 운반 뒤) {nb_first.tolist()}", flush=True)
        print(f"[Chain45] A 목표 퍽 처음 넘어간 단계 (같은 표기, 각본 0 ~ 5 = 원래 누운 상태라 뺌) {tp_first.tolist()}", flush=True)
    if args_cli.box_prob > 0 and not args_cli.lie:  # 세운 직육면체: 면 방향 - 칸 방향 (정사각 칸, 90° 주기)
        pxw = quat_apply(u.puck_quat_w(), torch.tensor([[1.0, 0.0, 0.0]], device=dev).expand(n, 3))
        pxb = quat_apply_inverse(r.data.root_link_quat_w.torch, pxw)
        yaw_p = torch.rad2deg(torch.atan2(pxb[:, 1], pxb[:, 0]))
        th_s = torch.tensor([sl["theta_deg"] for sl in SLOTS], device=dev)[slot]
        fe = ((yaw_p - th_s + 45.0) % 90.0) - 45.0
        bx = u.is_box
        if bx.any():
            print(f"[Chain45] 세운 직육면체 {int(bx.sum())} 대: 성공 {sum(1 for i in range(n) if bool(bx[i]) and reasons[i] == '성공')}, 칸에 앉은 대의 면 오차 ° {[round(v, 1) for v in fe[bx & seated].tolist()]} | 원기둥 {int((~bx).sum())} 대 성공 {sum(1 for i in range(n) if not bool(bx[i]) and reasons[i] == '성공')}", flush=True)
    sm_ = sd_ph.amin(1)
    ovm_ = ov_ph.amax(0)
    print(f"[Chain45] 팔끼리 충돌 상자 겹침 ({u.cfg.self_box_tol * 1000:.0f} mm 넘음) 대 {int((ov_ph.amax(1) > u.cfg.self_box_tol).sum())}/{n}, 단계별 최대 깊이 mm {[round(v * 1000, 1) for v in ovm_.tolist()]}", flush=True)
    from waffle_isaac.arm_selfcol import PAIRS as _AP
    print(f"[Chain45]   쌍별 최대 깊이 mm {dict((a.replace('gripper_roll_link', '집게') + '-' + b.replace('gripper_roll_link', '집게'), round(v * 1000, 1)) for (a, b), v in zip(_AP, ov_pair.amax(0).tolist()))}"
          f" | 가장 깊은 대 env{int(ov_qd.argmax())} 관절 ° {[round(math.degrees(v)) for v in ov_q[int(ov_qd.argmax())].tolist()]}", flush=True)
    print(f"[Chain45] 자기 충돌 근접 (< {u.cfg.self_margin * 1000:.0f} mm) 대 {int((sm_ < u.cfg.self_margin).sum())}/{n}, 단계별 최소 mm {[round(v * 1000) for v in sd_ph.amin(0).tolist()]}", flush=True)
    okr = rel_q.abs().sum(1) > 0
    if okr.any():
        mg = torch.minimum(rel_q - u._lo, u._hi - rel_q)[okr]
        print(f"[Chain45] 놓는 순간 관절 3 (°) {[round(math.degrees(v)) for v in rel_q[okr, 2].tolist()]} | 관절 2 · 3 · 4 한계 여유 최소 (°) {[round(math.degrees(v), 1) for v in mg[:, 1:4].amin(0).tolist()]}", flush=True)
    if args_cli.regrasp_tilt > 0 and not args_cli.lie:
        print(f"[Chain45] 다시 집기: {len(rg_tilts)} 번 (부른 기울기 ° {rg_tilts}), 다시 집은 대 {int((rg_n > 0).sum())}/{n} 중 성공 {sum(1 for i in range(n) if int(rg_n[i]) > 0 and reasons[i] == '성공')}", flush=True)
    print(f"[Chain45] **성공 (칸에 앉음 3 mm · 5° + 복귀 + 팔-몸체 1 N 이하 + 앞 칸 퍽 안 닿음) {reasons.count('성공')}/{n}**, 칸에 앉음 {int(seated.sum())}/{n}, "
          f"집기 통과 {int((phase >= 1).sum())}/{n}, 복귀 끝 {int(returned.sum())}/{n}", flush=True)
    per = {}
    for i in range(n):
        sl = SLOTS[int(slot[i])]
        k_ = f"{sl['row']}열 {sl['index']}"
        a, b = per.get(k_, (0, 0))
        per[k_] = (a + (reasons[i] == "성공"), b + (phase[i] >= 1).item())
    print(f"[Chain45] 칸별 성공 / 집기 통과: {', '.join(f'{k_} {a}/{b}' for k_, (a, b) in per.items())}", flush=True)
    print(f"[Chain45] 팔-몸체 1 N 넘은 대 {int((f_arm.amax(1) > 1).sum())}/{n} (최대 {f_arm.max():.1f} N), 손가락-몸체 (집기 뒤) 1 N 넘은 대 {int((f_fing[:, 1:].amax(1) > 1).sum())}/{n}"
          f" (최대 {f_fing[:, 1:].max():.1f} N), 퍽-몸체 운반 중 1 N 넘은 대 {int((f_puck[:, 1] > 1).sum())}/{n}, 앞 칸 퍽 닿은 대 {int(occ_hit.sum())}/{n}", flush=True)
    tt = [(t_enter[:, j][t_enter[:, j] >= 0].mean().item() if (t_enter[:, j] >= 0).any() else float('nan')) for j in range(1, NP)]
    print(f"[Chain45] 단계 시작 시각 평균 s: {', '.join(f'{PHASES[j + 1]} {v:.1f}' for j, v in enumerate(tt))}", flush=True)
    env.close()


if __name__ == "__main__":
    try:
        main()
    except Exception:
        import traceback
        traceback.print_exc()
    simulation_app.close()
