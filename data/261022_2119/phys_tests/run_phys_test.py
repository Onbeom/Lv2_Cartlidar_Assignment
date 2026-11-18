"""Waffle 4WD 3단계 물리 시험(P1~P8)과 진단 실험(D1·D2), 모의 시험 실행기.

기준 문서: 261022_1243/검증계획_261022.md (v3) 3장 C1~C6, 4장 P1~P8, 8장 D1·D2.
시험 하나를 프로세스 하나로 실행한다. 결과는 --out 폴더에 log.csv(물리 스텝마다)와 summary.json으로 남긴다.
--gui를 주면 창을 띄워 실제 시간 속도로 보여 주고, 끝난 뒤 창을 닫을 때까지 마지막 자세를 그대로 둔다.

실행 예(PC):
  ~/isaacsim-venv/bin/python 261022_2119/phys_tests/run_phys_test.py --test P1 --gui \
      --usd 261022_2119/phys_tests/usd_copy/waffle_4wd/waffle_4wd.usda \
      --urdf 261022_2119/urdf/waffle_4wd_description_261022_2119/urdf/waffle_4wd.urdf \
      --out 261022_2119/phys_tests/results --/crashreporter/url=

원칙
- 로봇 USD는 새 장면(메모리)에 참조로 넣는다. 이득·재질·바닥·자기충돌 설정은 이 메모리 장면에만 쓰고 저장하지 않는다.
- 가설 값은 rad 기준(SI)으로 적는다. USD 회전 drive 속성은 도 단위라서 USD에 쓸 때만 deg_gain()으로 바꾼다.
- 속도 한계(newton:velocityLimit)는 건드리지 않는다. PhysX가 그 값을 읽는지 보는 것이 P5다.
"""
import argparse
import csv
import json
import math
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

TESTS = ["P1", "P2", "P3", "P4on", "P4off", "P5", "P6", "P7", "V1", "V1b", "V2", "V3", "V4", "V5", "V6", "V6b", "V7", "V6c", "V6cb", "V7c", "V8", "D1A", "D1B", "D2A", "D2B", "D2C", "D3A", "D3B", "D3C", "D3D", "D3E"]
parser = argparse.ArgumentParser()
parser.add_argument("--test", required=True, choices=TESTS)
parser.add_argument("--usd", required=True, type=Path)
parser.add_argument("--urdf", required=True, type=Path)
parser.add_argument("--out", required=True, type=Path)
parser.add_argument("--gui", action="store_true", help="창을 띄워 실제 시간 속도로 보여 준다")
# 조건 바꾸기(이득 조정 G 시험, 마찰 진단). 주지 않으면 C3·C4 값 그대로
parser.add_argument("--arm-kp", type=float, help="팔 위치 drive 강성 N·m/rad")
parser.add_argument("--arm-kd", type=float, help="팔 위치 drive 감쇠 N·m·s/rad")
parser.add_argument("--mu", type=float, nargs=2, metavar=("STATIC", "DYNAMIC"), help="바퀴·바닥 마찰")
parser.add_argument("--tag", default="", help="결과 폴더 이름에 붙일 표시")
# 침투 진단 D5
parser.add_argument("--arm-maxforce", type=float, help="팔 drive 최대 힘(N·m). 주지 않으면 USD 값(3.8)")
parser.add_argument("--pos-iters", type=int, help="articulation 솔버 위치 반복 수")
parser.add_argument("--solver", choices=["TGS", "PGS"], help="PhysX 솔버 종류")
parser.add_argument("--dt", type=float, help="물리 dt(s). 주지 않으면 1/120")
parser.add_argument("--ccd", choices=["none", "sweep", "speculative"], default="speculative",
                    help="팔·집게 강체 연속 충돌 검출. v9 기본 speculative(D5 결과), none은 끔")
# v11(2026-10-23 사용자 지시 "Isaac도 맞춰"): MuJoCo 주행 시험과 같은 armature(모터 회전자 반영 관성, kg·m², SI).
# 주지 않으면 USD 값(0) 그대로. 출처: Haarnoja et al. arXiv 2304.13653 XM430-W350 0.045, 바퀴는 × (212.6/353.5)²
parser.add_argument("--wheel-armature", type=float, help="바퀴 관절 armature(kg·m²)")
parser.add_argument("--arm-armature", type=float, help="팔 관절 armature(kg·m²)")
# G3-1(검증계획 15장, 2026-10-23 승인): 팔 위치 drive의 목표 속도에 제어기 목표 변화율을 넣는다(속도 피드포워드).
# 주지 않으면 지금까지처럼 목표 속도 0(감쇠가 움직임을 거스름)
parser.add_argument("--arm-ff", action="store_true", help="팔 목표 속도 = 제어기 위치 목표의 변화율")
# V1b(검증계획 18장, 2026-10-24 승인): 접은 채 회전하기 전에 앞으로 옮기는 거리(회전 중심 오프셋 보정)
parser.add_argument("--pivot-offset-m", type=float, help="V1b 직진 거리 d(m). V1b에서만 쓰고 반드시 준다")
# V4·V5(검증계획 19장): 미로 조각 종류와 회전 반경. 구간 순서는 mujoco/stow/maneuver.py(MuJoCo와 같은 파일)
parser.add_argument("--corner", choices=["A", "B"], help="V4 들어오는 통로: A 720 판 사이, B 380 판 사이")
parser.add_argument("--side", choices=["L", "R"], help="V4 회전 방향")
parser.add_argument("--radius-m", type=float, help="V4 호 회전 반경 명령(m)")
parser.add_argument("--deadend", choices=["A", "B"], help="V5 막다른 칸 통로 종류")
parser.add_argument("--start-heading-deg", type=int, choices=[90, 180, -90], help="V6·V7 출발 칸에서 열린 쪽 기준 시작 방향(°)")
# V7(검증계획 21장): 제어기에 주는 위치의 오차. 판정은 실제 자세로 한다
parser.add_argument("--loc-bias", type=float, nargs=2, metavar=("LAT_M", "HEAD_DEG"), default=[0.0, 0.0], help="V7 고정 편향")
parser.add_argument("--loc-noise", type=float, nargs=2, metavar=("SIGMA_XY_M", "SIGMA_HEAD_DEG"), default=[0.0, 0.0],
                    help="V7 가우스 잡음(10 Hz로 새로 뽑음)")
parser.add_argument("--seed", type=int, default=1, help="V7 잡음 난수 seed")
parser.add_argument("--speed", type=float, help="V8 직진·호 회전 속도(m/s)")
args, _ = parser.parse_known_args()  # 나머지(--/crashreporter/url= 등)는 Kit 설정으로 넘긴다
if args.test == "V1b" and args.pivot_offset_m is None:
    parser.error("V1b에는 --pivot-offset-m이 필요하다")
if args.test in ("V4", "V6b", "V6cb", "V8") and None in (args.corner, args.side, args.radius_m):
    parser.error("V4·V6b·V6cb·V8에는 --corner, --side, --radius-m이 필요하다")
if args.test == "V8" and args.speed is None:
    parser.error("V8에는 --speed가 필요하다")
if args.test in ("V6", "V7", "V6c", "V7c") and args.start_heading_deg is None:
    parser.error("V6·V7·V6c·V7c에는 --start-heading-deg가 필요하다")
if args.test == "V5" and args.deadend is None:
    parser.error("V5에는 --deadend가 필요하다")
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "mujoco" / "stow"))
import maneuver  # noqa: E402

# ---------- 시험 조건 (검증계획 v2, C1~C5) ----------
DT = 1.0 / 120.0                      # C1 물리 dt (s)
GRAVITY = 9.81                        # C2 (m/s², −Z)
MU_STATIC, MU_DYNAMIC, RESTITUTION = 0.3, 0.225, 0.0  # C3 v10: 임의 값(사용자 위임). v1~v9는 0.8/0.6(가정). 정지:운동 비 0.75 유지
ARM_KP, ARM_KD = 400.0, 24.0          # C4 팔 위치 drive (N·m/rad, N·m·s/rad). v7: (200, 12), v8: (400, 24)(G2 결과 중 사용자 결정)
GL_KP, GL_KD = 1000.0, 10.0           # C4 왼쪽 집게 (N/m, N·s/m)
GR_KP, GR_KD = 0.0, 0.0               # C4 오른쪽 집게(mimic) 이득 0
WHEEL_KP, WHEEL_KD = 0.0, 1.0         # C4 바퀴 속도 drive (강성 0, 감쇠 N·m·s/rad)
SETTLE = 1.0                          # P2~P7 명령 전 정지 구간 (s), 모든 목표 0

# 진단 실험(검증계획 8장): 시험 이름별로 바꾸는 조건. 나머지는 C1~C6 그대로
DIAG = {
    "D1A": {"fix_base": True, "gravity": 0.0},                         # 팔꿈치만, 차체 고정·중력 끔
    "D1B": {"fix_base": True, "gravity": 0.0, "no_vel_limit": True},   # 위 + 속도 한계 없앰
    "D2A": {},                                                         # P7과 같은 조건
    "D2B": {"wheel_kd": 10.0},                                         # 바퀴 감쇠 1 → 10
    "D2C": {"mu": (0.4, 0.3)},                                         # 마찰 0.8/0.6 → 0.4/0.3
    "D3A": {"fix_base": True},                                         # 중력 켬 + 차체 고정
    "D3B": {"gravity": 0.0},                                           # 중력 끔 + 차체 바닥 위
    "D3C": {"vel_iters": 4},                                           # P 조건 + 솔버 속도 반복 1 → 4
    "D3D": {"solver": "PGS"},                                          # P 조건 + PGS 솔버
    "D3E": {"fix_base": True, "no_vel_limit": True},                   # 중력 켬 + 차체 고정 + 속도 한계 없앰
}.get(args.test, {})
# v4(사용자 승인 2026-10-22, A안): P 시험은 PhysX 관절 속도 한계를 끄고(최대 관절 속도 1000),
# 대신 제어기에서 속도 한계를 준다. 위치 목표는 한 스텝에 (한계 × dt)까지만 바꾸고, 바퀴 속도 목표는 ±한계로 자른다.
# 한계값은 USD newton:velocityLimit(= URDF velocity: 바퀴 2.0, 팔 0.5 rad/s, 집게 0.01 m/s)을 엔진에서 읽은 값.
CONTROLLER_LIMIT = args.test.startswith(("P", "V"))
# v5(사용자 승인 2026-10-22): 제어기 속도 한계를 ROBOTIS 공식 사양의 무부하 속도(11.1 V, TurtleBot 배터리 전압)로 바꾼다.
#   XM430-W210(바퀴): 70 rev/min = 7.3304 rad/s, XM430-W350(팔): 43 rev/min = 4.5029 rad/s
#   출처: https://emanual.robotis.com/docs/en/dxl/x/xm430-w210/ , .../xm430-w350/ (No Load Speed)
#   집게는 모터 회전 → 손가락 직선 속도 환산식이 없어 USD 값(0.01 m/s)을 그대로 쓴다.
OFFICIAL_RATE = {"Wheel_": 70 * 2 * math.pi / 60, "RMX52_": 43 * 2 * math.pi / 60}
if CONTROLLER_LIMIT:
    DIAG = dict(DIAG, no_vel_limit=True)
GRAVITY = DIAG.get("gravity", GRAVITY)
if args.arm_kp is not None:
    ARM_KP = args.arm_kp
if args.arm_kd is not None:
    ARM_KD = args.arm_kd
if args.mu is not None:
    DIAG = dict(DIAG, mu=tuple(args.mu))
if args.solver:
    DIAG = dict(DIAG, solver=args.solver)
if args.dt:
    DT = args.dt
MU_STATIC, MU_DYNAMIC = DIAG.get("mu", (MU_STATIC, MU_DYNAMIC))
WHEEL_KD = DIAG.get("wheel_kd", WHEEL_KD)

ROBOT = "/World/waffle_4wd"
WHEEL_BODIES = ["wheel_front_left", "wheel_front_right", "wheel_rear_left", "wheel_rear_right"]
ROOT = ROBOT + "/Geometry/base_footprint/base_link"


def deg_gain(value_per_rad):
    """rad 기준 이득을 USD 회전 drive 속성(도 기준) 값으로 바꾼다."""
    return value_per_rad * math.pi / 180.0


# ---------- URDF 순방향 기구학 (P2 비교용) ----------
def rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def axis_matrix(axis, angle):
    k = np.asarray(axis, float) / np.linalg.norm(axis)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(angle) * K + (1 - math.cos(angle)) * K @ K


def load_urdf_joints(path):
    joints = {}
    for j in ET.parse(path).getroot().findall("joint"):
        o = j.find("origin")
        xyz = [float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]
        rpy = [float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
        a = j.find("axis")
        joints[j.find("child").get("link")] = {
            "name": j.get("name"), "type": j.get("type"), "parent": j.find("parent").get("link"),
            "xyz": np.array(xyz), "R0": rpy_matrix(*rpy),
            "axis": [float(v) for v in a.get("xyz").split()] if a is not None else [1, 0, 0]}
    return joints


def fk_in_base(joints, link, q_by_joint):
    """base_link 좌표계에서 본 link 원점 위치(m)."""
    chain = []
    while link != "base_link":
        chain.append(joints[link])
        link = joints[link]["parent"]
    R, p = np.eye(3), np.zeros(3)
    for j in reversed(chain):
        p = p + R @ j["xyz"]
        R = R @ j["R0"]
        q = q_by_joint.get(j["name"], 0.0)
        if j["type"] in ("revolute", "continuous"):
            R = R @ axis_matrix(j["axis"], q)
        elif j["type"] == "prismatic":
            p = p + R @ (np.asarray(j["axis"]) * q)
    return p


def quat_to_matrix(wxyz):
    w, x, y, z = wxyz
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def roll_pitch_yaw(wxyz):
    R = quat_to_matrix(wxyz)
    return (math.atan2(R[2, 1], R[2, 2]), math.asin(max(-1.0, min(1.0, -R[2, 0]))), math.atan2(R[1, 0], R[0, 0]))


# ---------- 시험별 명령 일정 ----------
ARM_POSE_P4 = [0.0, 1.5, 1.5, 1.7, 0.0]
# 팔 접은 주행 자세 S0(검증계획 17장, 사용자 결정 2026-10-23): yaw, 어깨, 팔꿈치, 손목 pitch, roll (rad)
ARM_POSE_STOW = [0.0, 1.4, -1.0, -1.3, 0.0]


def build_schedule(test, arm, wheels_left, wheels_right):
    """(시작 s, 끝 s, {dof 이름: 목표}) 목록과 총 시간을 돌려준다. 목록에 없는 dof 목표는 0."""
    seg, t = [], SETTLE
    if test == "P1":
        return [], 5.0
    if test == "P2":
        for name in arm + ["gripper_left_joint"]:
            targets = [0.005, 0.015] if name == "gripper_left_joint" else [0.1, -0.1]
            for value in targets:
                seg.append((t, t + 3.0, {name: value}, f"{name}={value:+g}"))
                t += 3.0
                seg.append((t, t + 3.0, {}, f"{name} back 0"))
                t += 3.0
        return seg, t
    if test == "P3":
        return [(t, t + 4.0, {"gripper_left_joint": 0.02}, "left +0.02 m")], t + 4.0
    if test in ("P4on", "P4off"):
        return [(t, t + 6.0, dict(zip(arm, ARM_POSE_P4)), "arm [0,1.5,1.5,1.7,0]")], t + 6.0
    if test == "P5":  # v5: 공식 한계(7.3304 rad/s)의 2배를 명령
        w = 2 * OFFICIAL_RATE["Wheel_"]
        return [(t, t + 3.0, {n: w for n in wheels_left + wheels_right}, f"wheels {w:.3f} rad/s")], t + 3.0
    if test == "P6":  # P8(정지)을 함께 기록한다
        w = 0.05 / 0.0330006                   # 0.05 m/s ÷ 바퀴 반지름(URDF 0.0330006 m)
        allw = wheels_left + wheels_right
        seg = [(t, t + 4.0, {n: w for n in allw}, "forward"),
               (t + 4.0, t + 6.0, {}, "stop1"),
               (t + 6.0, t + 10.0, {n: -w for n in allw}, "backward"),
               (t + 10.0, t + 12.0, {}, "stop2 (P8)")]
        return seg, t + 12.0
    if test in ("D1A", "D1B", "D3A", "D3B", "D3C", "D3D", "D3E"):
        e = "RMX52_Elbow"
        seg = [(t, t + 3.0, {e: 0.1}, "elbow=+0.1"), (t + 3.0, t + 6.0, {}, "elbow back 0"),
               (t + 6.0, t + 9.0, {e: -0.1}, "elbow=-0.1"), (t + 9.0, t + 12.0, {}, "elbow back 0 (2)")]
        return seg, t + 12.0
    if test in ("P7", "D2A", "D2B", "D2C"):
        tgt = {n: -1.0 for n in wheels_left}
        tgt.update({n: 1.0 for n in wheels_right})
        return [(t, t + 4.0, tgt, "spin CCW")], t + 4.0
    if test in ("V1", "V1b", "V2", "V3"):  # 17장 확인 시험. 처음 2 s 동안 0 자세에서 S0으로 접고(제어기 속도 한계) 유지한다
        stow = dict(zip(arm, ARM_POSE_STOW))
        if test == "V1b":  # 18장: 접은 채 d만큼 직진(V2 속도) → 1 s 정지 → V1과 같은 4 s 회전
            w = 0.05 / 0.0330006
            t1 = 2.0 + args.pivot_offset_m / 0.05
            fwd = dict(stow, **{n: w for n in wheels_left + wheels_right})
            spin = dict(stow, **{n: -1.0 for n in wheels_left}, **{n: 1.0 for n in wheels_right})
            return [(0.0, 2.0, stow, "stow"), (2.0, t1, fwd, "forward d (stowed)"), (t1, t1 + 1.0, stow, "stop (stowed)"),
                    (t1 + 1.0, t1 + 5.0, spin, "spin CCW (stowed)")], t1 + 5.0
        if test == "V1":  # 접은 채 P7과 같은 제자리 회전
            spin = dict(stow, **{n: -1.0 for n in wheels_left}, **{n: 1.0 for n in wheels_right})
            return [(0.0, 2.0, stow, "stow"), (2.0, 6.0, spin, "spin CCW (stowed)")], 6.0
        if test == "V2":  # 접은 채 P6·P8과 같은 직진·정지·후진·정지
            w = 0.05 / 0.0330006
            allw = wheels_left + wheels_right
            return [(0.0, 2.0, stow, "stow"), (2.0, 6.0, dict(stow, **{n: w for n in allw}), "forward (stowed)"),
                    (6.0, 8.0, stow, "stop1"), (8.0, 12.0, dict(stow, **{n: -w for n in allw}), "backward (stowed)"),
                    (12.0, 14.0, stow, "stop2")], 14.0
        return [(0.0, 1.0, {}, "zero"), (1.0, 4.0, stow, "to stow"), (4.0, 7.0, {}, "back to zero")], 7.0
    raise ValueError(test)


def active_segment(schedule, t):
    for s in schedule:
        if s[0] <= t < s[1]:
            return s
    return None


# ---------- 앱 시작 ----------
from isaacsim import SimulationApp  # noqa: E402

if args.gui:
    app = SimulationApp({"headless": False, "width": 1280, "height": 720, "multi_gpu": False})
else:
    app = SimulationApp({"headless": True, "width": 640, "height": 360, "anti_aliasing": 0, "multi_gpu": False})
exit_code = 1
try:
    import omni.timeline
    import omni.usd
    from isaacsim.core.experimental.prims import Articulation, RigidPrim
    from isaacsim.core.simulation_manager import SimulationEvent, SimulationManager
    from omni.physx import get_physx_simulation_interface
    from omni.physx.scripts import physicsUtils
    from pxr import Gf, PhysicsSchemaTools, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

    started = time.strftime("%Y-%m-%d %H:%M:%S")
    out = args.out / f"{args.test}{'_' + args.tag if args.tag else ''}_{time.strftime('%y%m%d_%H%M%S')}"
    out.mkdir(parents=True, exist_ok=False)

    # --- 새 장면(메모리) ---
    omni.usd.get_context().new_stage()
    app.update()
    stage = omni.usd.get_context().get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.Xform.Define(stage, "/World")

    # C1·C2: 물리 장면
    scene = UsdPhysics.Scene.Define(stage, "/World/physicsScene")
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    scene.CreateGravityMagnitudeAttr(GRAVITY)
    physx_scene = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
    physx_scene.CreateTimeStepsPerSecondAttr(int(round(1.0 / DT)))
    if DIAG.get("solver"):
        physx_scene.CreateSolverTypeAttr(DIAG["solver"])
    if args.gui:  # 새 장면에는 빛이 없어 화면이 검게 나온다. 관찰용 조명(물리 무관)
        from pxr import UsdLux
        UsdLux.DomeLight.Define(stage, "/World/DomeLight").CreateIntensityAttr(1000.0)
        UsdLux.DistantLight.Define(stage, "/World/SunLight").CreateIntensityAttr(2500.0)

    # C2: 평평한 무한 바닥(UsdPhysics.Plane)
    ground = physicsUtils.add_ground_plane(stage, "/World/ground", "Z", 25.0, Gf.Vec3f(0, 0, 0), Gf.Vec3f(0.5))

    # 로봇(참조, Physics variant physx)
    robot_prim = stage.DefinePrim(ROBOT, "Xform")
    robot_prim.GetReferences().AddReference(str(args.usd.resolve()))
    robot_prim.GetVariantSets().GetVariantSet("Physics").SetVariantSelection("physx")
    app.update()
    if not stage.GetPrimAtPath(ROOT).IsValid():
        raise RuntimeError(f"robot root not found: {ROOT}")

    # C3: 재질을 바닥과 바퀴 충돌체 4개에 같게 적용
    mat = UsdShade.Material.Define(stage, "/World/Materials/contact_c3")
    mat_api = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    mat_api.CreateStaticFrictionAttr(MU_STATIC)
    mat_api.CreateDynamicFrictionAttr(MU_DYNAMIC)
    mat_api.CreateRestitutionAttr(RESTITUTION)
    px_mat = PhysxSchema.PhysxMaterialAPI.Apply(mat.GetPrim())
    px_mat.CreateFrictionCombineModeAttr("average")
    px_mat.CreateRestitutionCombineModeAttr("average")
    material_targets = [ground + "/CollisionPlane"] + [
        f"{ROOT}/wheel_{w}/tire_cylinder" for w in ("front_left", "front_right", "rear_left", "rear_right")]
    for path in material_targets:
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid():
            raise RuntimeError(f"material target missing: {path}")
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat, UsdShade.Tokens.weakerThanDescendants, "physics")

    # D1: 차체를 월드에 고정(고정 관절, 메모리 장면에만)
    if DIAG.get("fix_base"):
        fj = UsdPhysics.FixedJoint.Define(stage, "/World/fix_base_to_world")
        fj.CreateBody1Rel().SetTargets([Sdf.Path(ROOT)])
        root_pos = UsdGeom.Xformable(stage.GetPrimAtPath(ROOT)).ComputeLocalToWorldTransform(0).ExtractTranslation()
        fj.CreateLocalPos0Attr(Gf.Vec3f(*root_pos))
        fj.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))

    # 관찰용 카메라(--gui)
    if args.gui:
        cam_path = "/World/ViewCam"
        UsdGeom.Camera.Define(stage, cam_path)
        eye, target = (0.85, -0.85, 0.55), (0.05, 0.0, 0.18)
        xf = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*target), Gf.Vec3d(0, 0, 1)).GetInverse()
        UsdGeom.Xformable(stage.GetPrimAtPath(cam_path)).AddTransformOp().Set(xf)
        try:
            from omni.kit.viewport.utility import get_active_viewport
            vp = get_active_viewport()
            if vp is not None:
                vp.camera_path = cam_path
        except Exception as exc:  # 창이 없을 때만 해당
            print("viewport camera not set:", exc)

    # D3C: 관절 묶음(articulation)의 솔버 속도 반복 수
    if args.pos_iters:
        PhysxSchema.PhysxArticulationAPI.Apply(stage.GetPrimAtPath(ROOT)).CreateSolverPositionIterationCountAttr(args.pos_iters)
    if DIAG.get("vel_iters"):
        PhysxSchema.PhysxArticulationAPI.Apply(stage.GetPrimAtPath(ROOT)).CreateSolverVelocityIterationCountAttr(
            DIAG["vel_iters"])

    # D5: 팔·집게 강체 연속 충돌 검출
    if args.ccd != "none":
        if args.ccd == "sweep":
            physx_scene.CreateEnableCCDAttr(True)
        for p in Usd.PrimRange(stage.GetPrimAtPath(ROOT)):
            if p.HasAPI(UsdPhysics.RigidBodyAPI) and p.GetName().startswith(("arm_link_", "gripper_")):
                rb = PhysxSchema.PhysxRigidBodyAPI.Apply(p)
                (rb.CreateEnableCCDAttr if args.ccd == "sweep" else rb.CreateEnableSpeculativeCCDAttr)(True)

    # 강체 목록, 접촉 보고
    bodies = [str(p.GetPath()) for p in Usd.PrimRange(stage.GetPrimAtPath(ROBOT)) if p.HasAPI(UsdPhysics.RigidBodyAPI)]
    for path in bodies:
        cr = PhysxSchema.PhysxContactReportAPI.Apply(stage.GetPrimAtPath(path))
        cr.CreateThresholdAttr(0.0)

    # P4off: newton:selfCollisionEnabled만 끈다
    self_attr = stage.GetPrimAtPath(ROOT).GetAttribute("newton:selfCollisionEnabled")
    self_before = self_attr.Get() if self_attr.IsValid() else None
    if args.test == "P4off":
        if not self_attr.IsValid():
            raise RuntimeError("newton:selfCollisionEnabled attribute not found on root")
        self_attr.Set(False)

    # C4: drive 이득·목표를 USD 속성에 쓴다(회전은 도 단위로 변환)
    joint_prims = {p.GetName(): p for p in Usd.PrimRange(stage.GetPrimAtPath(ROBOT))
                   if p.IsA(UsdPhysics.RevoluteJoint) or p.IsA(UsdPhysics.PrismaticJoint)}
    authored = {}
    for name, prim in joint_prims.items():
        if name.startswith("Wheel_"):
            kp, kd, kind, angular = WHEEL_KP, WHEEL_KD, "velocity", True
        elif name.startswith("RMX52_"):
            kp, kd, kind, angular = ARM_KP, ARM_KD, "position", True
        elif name == "gripper_left_joint":
            kp, kd, kind, angular = GL_KP, GL_KD, "position", False
        elif name == "gripper_right_joint":
            kp, kd, kind, angular = GR_KP, GR_KD, "position", False
        else:
            continue
        drive = UsdPhysics.DriveAPI.Get(prim, "angular" if angular else "linear")
        if not drive:
            raise RuntimeError(f"drive missing on {name}")
        if args.arm_maxforce is not None and name.startswith("RMX52_"):
            drive.CreateMaxForceAttr(args.arm_maxforce)
        drive.CreateStiffnessAttr(deg_gain(kp) if angular else kp)
        drive.CreateDampingAttr(deg_gain(kd) if angular else kd)
        drive.CreateTargetPositionAttr(0.0)
        drive.CreateTargetVelocityAttr(0.0)
        authored[name] = {"si_kp": kp, "si_kd": kd, "usd_kp": drive.GetStiffnessAttr().Get(),
                          "usd_kd": drive.GetDampingAttr().Get(), "unit": "deg" if angular else "m", "target": kind}

    # 시뮬레이션 설정: CPU 물리(접촉 보고·결정성), dt
    app.update()
    SimulationManager.setup_simulation(dt=DT, device="cpu")

    art = Articulation(ROOT)
    links = RigidPrim(paths=bodies)

    # 기록 상태
    st = {"t": 0.0, "steps": 0, "rows": [], "contacts": {}, "seg_end": {}}
    names_holder = {}

    def pre_step(dt, context):
        names = names_holder.get("dof")
        if names is None:
            return
        if "seq" in names_holder:  # V4~V6b: 시간·yaw·추종 조건 구간. 자세는 직전 스텝 끝 값(센서 한 스텝 지연과 같음)
            name, tgt = names_holder["seq"].update(st["t"], st.get("yaw", 0.0), *st.get("xy", (0.0, 0.0)))
            seg = (None, None, tgt, name)
        else:
            seg = active_segment(names_holder["schedule"], st["t"])
        tgt = seg[2] if seg else {}
        pos = np.array([[tgt.get(n, 0.0) if n not in names_holder["wheels"] else 0.0 for n in names]], np.float32)
        vel = np.array([[tgt.get(n, 0.0) if n in names_holder["wheels"] else 0.0 for n in names]], np.float32)
        if CONTROLLER_LIMIT:  # 제어기 속도 한계(v4)
            rate = names_holder["rate"]
            prev = st.setdefault("sent_pos", np.zeros_like(pos))
            pos = prev + np.clip(pos - prev, -rate * DT, rate * DT)
            st["sent_pos"] = pos
            vel = np.clip(vel, -rate, rate)
        art.set_dof_position_targets(pos[:, names_holder["pos_idx"]], dof_indices=names_holder["pos_idx"])
        if args.arm_ff:  # G3-1: 이번 스텝 목표 변화량 ÷ dt (제어기 한계 안의 값)
            prev_ff = st.setdefault("ff_prev_pos", np.zeros_like(pos))
            ff = (pos - prev_ff) / DT
            st["ff_prev_pos"] = pos.copy()
            ai = names_holder["arm_idx"]
            art.set_dof_velocity_targets(ff[:, ai], dof_indices=ai)
        art.set_dof_velocity_targets(vel[:, names_holder["vel_idx"]], dof_indices=names_holder["vel_idx"])
        sent = np.where([n in names_holder["wheels"] for n in names], vel[0], pos[0])
        st["cmd"] = [float(v) for v in sent]  # 실제로 보낸 목표(v4에서는 속도 한계를 적용한 값)
        label = seg[3] if seg else ("settle" if st["t"] < SETTLE else "idle")
        if args.gui and label != st.get("seg_label"):
            print(f"[{st['t']:6.2f} s] 구간 시작: {label}  명령 {tgt if tgt else '모두 0'}", flush=True)
        st["seg_label"] = label

    def post_step(dt, context):
        names = names_holder.get("dof")
        if names is None:
            return
        st["t"] += float(dt)
        st["steps"] += 1
        q = art.get_dof_positions().numpy()[0]
        dq = art.get_dof_velocities().numpy()[0]
        f = art.get_dof_projected_joint_forces().numpy()[0]
        bp, bq = art.get_world_poses()
        bp, bq = bp.numpy()[0], bq.numpy()[0]
        lv, av = art.get_velocities()
        lv, av = lv.numpy()[0], av.numpy()[0]
        link_lin, _ = links.get_velocities()
        link_speed = float(np.max(np.linalg.norm(link_lin.numpy(), axis=1)))
        headers, data = get_physx_simulation_interface().get_contact_report()[:2]
        pairs = {}  # 쌍 이름 → 이 스텝의 최소 separation(m). 접촉 보고는 contact offset 안의 가까운 쌍도 포함한다
        fn = {w: 0.0 for w in WHEEL_BODIES}  # D6-1: 바퀴별 바닥 수직 반력(N) = 접촉 impulse의 법선 성분 합 ÷ dt
        for h in headers:
            if str(h.type).endswith("CONTACT_LOST"):
                continue
            a = str(PhysicsSchemaTools.intToSdfPath(h.actor0)).rsplit("/", 1)[-1]
            b = str(PhysicsSchemaTools.intToSdfPath(h.actor1)).rsplit("/", 1)[-1]
            seps = [data[i].separation for i in range(h.contact_data_offset, h.contact_data_offset + h.num_contact_data)]
            wb = a if a in fn else (b if b in fn else None)
            if wb is not None and "CollisionPlane" in (a, b):
                for i in range(h.contact_data_offset, h.contact_data_offset + h.num_contact_data):
                    imp, nrm = data[i].impulse, data[i].normal
                    fn[wb] += abs(imp[0] * nrm[0] + imp[1] * nrm[1] + imp[2] * nrm[2]) / float(dt)
            key = "|".join(sorted([a, b]))
            pairs[key] = min([pairs.get(key, math.inf)] + seps)
        for p, sep in pairs.items():
            c = st["contacts"].setdefault(p, {"first_t": st["t"], "steps": 0, "min_sep_m": math.inf,
                                              "steps_sep_le_0": 0, "first_t_sep_le_0": None})
            c["steps"] += 1
            c["min_sep_m"] = min(c["min_sep_m"], sep)
            if sep <= 0.0:
                c["steps_sep_le_0"] += 1
                if c["first_t_sep_le_0"] is None:
                    c["first_t_sep_le_0"] = st["t"]
        lp_all, _ = links.get_world_poses()
        lp_all = lp_all.numpy()
        r, pch, yw = roll_pitch_yaw(bq)
        st["yaw"], st["xy"] = yw, (float(bp[0]), float(bp[1]))
        row = [st["steps"], st["t"], st.get("seg_label", "")] + list(bp) + list(bq) + [r, pch, yw] + list(lv) + list(av) \
            + [link_speed] + list(q) + list(dq) + list(f) + st.get("cmd", [0.0] * len(names)) + list(lp_all.reshape(-1)) \
            + [fn[w] for w in WHEEL_BODIES] \
            + [";".join(f"{k}:{v * 1000:.3f}" for k, v in sorted(pairs.items()))]
        st["rows"].append(row)
        if not (np.isfinite(q).all() and np.isfinite(dq).all() and np.isfinite(bp).all()):
            st["nonfinite"] = st.get("nonfinite", st["t"])
        # 구간 끝 측정(P2): 이 스텝이 구간의 마지막 스텝이면 링크 위치를 저장
        seg = active_segment(names_holder["schedule"], st["t"] - 1e-9)
        if args.test == "P2" and seg and st["t"] + DT > seg[1] - 1e-9 and seg[2]:
            lp, _ = links.get_world_poses()
            lp = lp.numpy()
            Rb = quat_to_matrix(bq)
            st["seg_end"][seg[3]] = {"q": {n: float(v) for n, v in zip(names, q)},
                                     "links": {bodies[i].rsplit("/", 1)[-1]: (Rb.T @ (lp[i] - bp)).tolist()
                                               for i in range(len(bodies))}}

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    for _ in range(3):
        app.update()
    if not art.is_physics_tensor_entity_valid():
        raise RuntimeError("articulation tensor view invalid after play")

    dof = list(art.dof_names)
    wheels = [n for n in dof if n.startswith("Wheel_")]
    wheels_left = [n for n in wheels if n.endswith("_Left")]
    wheels_right = [n for n in wheels if n.endswith("_Right")]
    arm = [n for n in dof if n.startswith("RMX52_")]
    names_holder.update({
        "dof": dof, "wheels": set(wheels),
        "pos_idx": np.array([i for i, n in enumerate(dof) if n not in wheels], np.int32),
        "arm_idx": np.array([i for i, n in enumerate(dof) if n.startswith("RMX52_")], np.int32),
        "vel_idx": np.array([i for i, n in enumerate(dof) if n in wheels], np.int32)})
    usd_rate = [float(v) for v in art.get_dof_max_velocities().numpy()[0]]
    names_holder["rate"] = np.array([[next((r for k, r in OFFICIAL_RATE.items() if n.startswith(k)), u)
                                      for n, u in zip(dof, usd_rate)]], np.float32)
    if args.test in ("V4", "V5", "V6", "V6b", "V7", "V6c", "V6cb", "V7c", "V8"):
        mp = {"corner": args.corner, "side": 1 if args.side == "L" else -1, "radius_m": args.radius_m, "deadend": args.deadend,
              "start_heading_deg": args.start_heading_deg, "speed": args.speed}
        phases, duration = maneuver.build(args.test, mp, wheels_left, wheels_right, dict(zip(arm, ARM_POSE_STOW)))
        names_holder["seq"], schedule = maneuver.Sequencer(phases, maneuver.LocError(
            *args.loc_bias, *args.loc_noise, seed=args.seed) if args.test in ("V7", "V7c") else None), []
    else:
        schedule, duration = build_schedule(args.test, arm, wheels_left, wheels_right)
    names_holder["schedule"] = schedule
    armature_before = [float(v) for v in art.get_dof_armatures().numpy()[0]]
    for value, group in ((args.wheel_armature, wheels), (args.arm_armature, arm)):
        if value is not None:
            art.set_dof_armatures([value], dof_indices=[dof.index(n) for n in group])

    # 시작 시점 확인값(SI): 이득·한계·최대 속도·max force
    kp_si, kd_si = art.get_dof_gains()
    readback = {
        "dof_names": dof,
        "gains_si": {n: [float(a), float(b)] for n, a, b in zip(dof, kp_si.numpy()[0], kd_si.numpy()[0])},
        "max_velocities": {n: float(v) for n, v in zip(dof, art.get_dof_max_velocities().numpy()[0])},
        "max_efforts": {n: float(v) for n, v in zip(dof, art.get_dof_max_efforts().numpy()[0])},
        "armatures_before": dict(zip(dof, armature_before)),
        "armatures": {n: float(v) for n, v in zip(dof, art.get_dof_armatures().numpy()[0])},
        "limits": {n: [float(a), float(b)] for n, (a, b) in zip(dof, np.stack(
            [x.numpy()[0] for x in art.get_dof_limits()], axis=-1))},
        "self_collision_newton_before": self_before,
        "self_collision_newton_now": self_attr.Get() if self_attr.IsValid() else None,
        "physics_device": str(SimulationManager.get_device()),
        "physics_dt": SimulationManager.get_physics_dt(),
        "solver_type": physx_scene.GetSolverTypeAttr().Get(),
        "solver_iterations_usd(None=기본값)": {
            a: (stage.GetPrimAtPath(ROOT).GetAttribute(a).Get() if stage.GetPrimAtPath(ROOT).GetAttribute(a) else None)
            for a in ("physxArticulation:solverPositionIterationCount", "physxArticulation:solverVelocityIterationCount")},
        "bodies": [b.rsplit("/", 1)[-1] for b in bodies]}

    # 시험 시작: 지금까지의 워밍업 스텝은 기록하지 않는다(시간 0부터)
    pre_id = SimulationManager.register_callback(pre_step, SimulationEvent.PHYSICS_PRE_STEP)
    post_id = SimulationManager.register_callback(post_step, SimulationEvent.PHYSICS_POST_STEP)
    expected_steps = int(round(duration / DT))
    if DIAG.get("no_vel_limit"):
        art.set_dof_max_velocities(np.full((1, len(dof)), 1.0e3, np.float32))
        readback["max_velocities_after_override"] = {
            n: float(v) for n, v in zip(dof, art.get_dof_max_velocities().numpy()[0])}
    wall0, next_print = time.time(), 0.0
    cpu0 = time.process_time()
    while st["steps"] < expected_steps and app.is_running():
        app.update()
        if args.gui:  # 실제 시간 속도로 맞춘다
            lag = (wall0 + st["t"]) - time.time()
            if lag > 0:
                time.sleep(lag)
            if st["t"] >= next_print and st["rows"]:
                r = st["rows"][-1]
                qd = {n: round(float(v), 4) for n, v in zip(dof, r[20:20 + len(dof)])
                      if n.startswith(("RMX52_", "gripper_left"))}
                print(f"  t={st['t']:5.2f} s  base x={r[3]:+.4f} y={r[4]:+.4f} m yaw={math.degrees(r[12]):+.3f}°  팔 q={qd}",
                      flush=True)
                next_print += 1.0
    wall_s, cpu_s = time.time() - wall0, time.process_time() - cpu0  # 시험 구간(물리 스텝·기록 포함) 시간
    SimulationManager.deregister_callback(pre_id)
    SimulationManager.deregister_callback(post_id)
    if args.gui:
        timeline.pause()  # 창에서는 마지막 자세를 남긴다
    else:
        timeline.stop()

    # --- CSV ---
    head = ["step", "t_s", "segment", "base_x_m", "base_y_m", "base_z_m", "qw", "qx", "qy", "qz",
            "roll_rad", "pitch_rad", "yaw_rad", "base_vx", "base_vy", "base_vz", "base_wx", "base_wy", "base_wz",
            "max_link_speed_mps"] + [f"q_{n}" for n in dof] + [f"dq_{n}" for n in dof] \
        + [f"force_{n}" for n in dof] + [f"cmd_{n}" for n in dof] \
        + [f"{b.rsplit('/', 1)[-1]}_{ax}_m" for b in bodies for ax in "xyz"] + [f"Fn_{w}_N" for w in WHEEL_BODIES] \
        + ["contact_pairs_minsep_mm"]
    with (out / "log.csv").open("w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(head)
        w.writerows(st["rows"])

    summary = {"test": args.test, "started": started, "usd": str(args.usd.resolve()), "urdf": str(args.urdf.resolve()),
               "conditions": {"wheel_armature_kgm2": args.wheel_armature, "arm_armature_kgm2": args.arm_armature,
                              "arm_velocity_feedforward_g3": bool(args.arm_ff),
                              "controller_velocity_limit_v4": CONTROLLER_LIMIT,
                              "controller_rates": dict(zip(dof, names_holder["rate"][0].tolist())), "dt_s": DT, "gravity": GRAVITY, "mu_static": MU_STATIC, "mu_dynamic": MU_DYNAMIC,
                              "restitution": RESTITUTION, "settle_s": SETTLE, "arm_kp_nm_per_rad": ARM_KP,
                              "arm_kd": ARM_KD, "gl_kp": GL_KP, "gl_kd": GL_KD, "gr_kp": GR_KP, "gr_kd": GR_KD,
                              "wheel_kp": WHEEL_KP, "wheel_kd": WHEEL_KD, "arm_maxforce": args.arm_maxforce,
                              "pos_iters": args.pos_iters, "solver": args.solver, "ccd": args.ccd,
                              "pivot_offset_m": args.pivot_offset_m},
               "authored_usd_drive": authored, "readback": readback,
               "timing": {"wall_s": wall_s, "cpu_s": cpu_s, "gui": args.gui,
                          "sim_s_per_wall_s": (st["t"] / wall_s) if wall_s > 0 else None},
               "steps_recorded": st["steps"], "expected_steps": expected_steps, "duration_s": duration,
               "nonfinite_first_t": st.get("nonfinite"), "contacts": st["contacts"],
               "schedule": [[s[0], s[1], s[2], s[3]] for s in schedule], "seg_end": st["seg_end"]}
    if "seq" in names_holder:
        seq = names_holder["seq"]
        summary["maneuver"] = {
            "params": {"corner": args.corner, "side": args.side, "radius_m": args.radius_m, "deadend": args.deadend,
                       "start_heading_deg": args.start_heading_deg, "k_head": maneuver.K_HEAD, "k_lat": maneuver.K_LAT,
                       "w_max": maneuver.W_MAX, "loc_bias": args.loc_bias, "loc_noise": args.loc_noise,
                       "seed": args.seed,
                       "v_mps": args.speed if args.test == "V8" else maneuver.V, "track_m": maneuver.TRACK, "wheel_r_m": maneuver.WHEEL_R},
            "phases": maneuver.describe(seq.phases), "phase_log": seq.log, "align": seq.align}
    summary["urdf_fk"] = None
    if args.test == "P2":
        joints = load_urdf_joints(args.urdf)
        fk = {}
        for label, rec in st["seg_end"].items():
            child = {"gripper_left_joint": "gripper_left_link"}.get(label.split("=")[0], None)
            jname = label.split("=")[0]
            child = child or next(c for c, j in joints.items() if j["name"] == jname)
            res = {}
            for lk in (child, "gripper_left_link"):
                pred = fk_in_base(joints, lk, rec["q"])
                sim = np.array(rec["links"][lk])
                res[lk] = {"fk_m": pred.tolist(), "sim_m": sim.tolist(), "err_mm": float(np.linalg.norm(pred - sim) * 1000)}
            fk[label] = res
        summary["urdf_fk"] = fk
    (out / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    print("RESULT_DIR:", out)
    print("STEPS:", st["steps"], "/", expected_steps)
    if args.gui and app.is_running():  # 마지막 자세를 그대로 보여 준다(물리 정지)
        print("시험 끝. 창을 닫으면 종료합니다(물리는 멈춘 상태).", flush=True)
        while app.is_running():
            app.update()
    exit_code = 0 if st["steps"] == expected_steps else 2
except Exception:  # app.close()가 프로세스를 끝내 traceback이 사라지지 않게 먼저 출력한다
    import traceback
    traceback.print_exc()
    sys.stdout.flush()
    sys.stderr.flush()
finally:
    app.close()
    sys.exit(exit_code)
