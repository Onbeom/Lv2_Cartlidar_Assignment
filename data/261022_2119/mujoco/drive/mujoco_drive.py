"""MuJoCo 주행 시험(모의): 261022_2119 URDF에 자유 관절·바닥·구동기를 붙여 Isaac P1·P5~P8과 같은 일정으로 돌린다.

시험 하나 = 실행 하나. 결과 폴더에 log.csv(스텝마다), summary.json(조건·모델 값), 선택 시 video.mp4를 쓴다.
판정은 analyze_drive.py가 한다.

조건(검증계획 Isaac v10과 맞춤, 2026-10-23 사용자 승인):
  중력 −9.81, 마찰 0.3(MuJoCo는 정지·운동 구분 없음, Isaac 정지 값 사용), 바퀴 속도 구동기 kv 1·±2.7 N·m,
  바퀴 목표는 제어기에서 ±7.3304 rad/s로 자름, 팔 위치 구동기 kp 400·kv 24·±3.8 N·m,
  왼쪽 집게 kp 1000·kv 10·±10 N, 오른쪽 집게는 URDF mimic(관절 등식), 팔·집게 목표 0, 적분기 implicitfast.

armature(2026-10-23 사용자 요청으로 추가): URDF에는 모터 회전자 관성이 없어 MuJoCo가 0으로 읽는다. 0이면 바퀴가
  30 g 원통 관성만 가져 토크 상한(±2.7 N·m)에 걸릴 때 수치가 발산하고, 제자리 회전량이 dt에 따라 수렴하지 않았다(준비 확인).

실행(PC): .venv/bin/python drive/mujoco_drive.py M6 --dt 0.002 --wheel-armature 0.0163 --arm-armature 0.045 \
            --out drive/results/M6_B [--video] [--view]
"""
import argparse
import csv
import json
import math
import os
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "stow"))
import maneuver  # noqa: E402

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("test", choices=["M1", "M5", "M6", "M7", "V1", "V1b", "V2", "V3", "V4", "V5", "V6", "V6b", "V7", "V6c", "V6cb", "V7c", "V8"],
                    help="M6은 M8(정지)을 함께 기록한다. V1~V3은 팔 접은 자세 확인 시험(검증계획 17장), V1b는 회전 중심 오프셋 보정 회전(18장), V4 모퉁이 호 회전·V5 막다른 길 후진(19장), V6·V6b 회전 + 위치 정상화(20장), V7 위치 오차가 있을 때의 정상화(21장), V6c·V6cb·V7c 정지 후 헤딩 보정(22장), V8 속도를 높인 V4(23장)")
parser.add_argument("--dt", type=float, required=True, help="물리 dt(s). A 1/120, B 0.002, C 0.001")
parser.add_argument("--out", type=Path, required=True, help="결과 폴더(이미 있으면 멈춘다)")
parser.add_argument("--wheel-armature", type=float, required=True,
                    help="바퀴 관절 armature(모터 회전자 반영 관성, kg·m²). 0이면 URDF 그대로")
parser.add_argument("--arm-armature", type=float, required=True, help="팔 관절 armature(kg·m²). 0이면 URDF 그대로")
parser.add_argument("--urdf", type=Path, default=Path(__file__).resolve().parent / "waffle_4wd_mujoco_drive.urdf")
parser.add_argument("--video", action="store_true", help="화면 없이 렌더링해 video.mp4(640×480, 30 fps)를 쓴다. 물리에는 영향 없음")
parser.add_argument("--view", action="store_true", help="MuJoCo 뷰어 창으로 실시간 속도로 보여 준다. 물리에는 영향 없음")
parser.add_argument("--pivot-offset-m", type=float, help="V1b 직진 거리 d(m). V1b에서만 쓰고 반드시 준다")
# V4·V5(검증계획 19장): 미로 조각 종류와 회전 반경. 구간 순서는 stow/maneuver.py(Isaac과 같은 파일)
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
parser.add_argument("--mu", type=float, help="미끄럼 마찰(모든 형상). 주지 않으면 0.3(V9 마찰 민감도, 24장)")
args = parser.parse_args()
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
if args.video and not args.view:
    os.environ.setdefault("MUJOCO_GL", "egl")  # 화면 없이 GPU 렌더링(PC에서 확인함)
import mujoco  # noqa: E402  (MUJOCO_GL을 정한 뒤 불러야 한다)

# ---------- 시험 조건 ----------
GRAVITY = 9.81
MU = 0.3                                # 미끄럼 마찰(모든 형상, MuJoCo는 두 형상 중 큰 값을 씀)
MU_TORSION, MU_ROLL = 0.005, 0.0001     # MuJoCo 기본값(condim 3에서는 쓰이지 않음)
if args.mu is not None:
    MU = args.mu
WHEEL_KV, WHEEL_TORQUE = 1.0, 2.7       # N·m·s/rad, N·m(URDF effort)
ARM_KP, ARM_KV, ARM_TORQUE = 400.0, 24.0, 3.8
GL_KP, GL_KV, GL_FORCE = 1000.0, 10.0, 10.0
WHEEL_RATE = 70 * 2 * math.pi / 60      # XM430-W210 무부하 속도(11.1 V) = 7.3304 rad/s, 제어기 한계
WHEEL_R = 0.0330006                     # URDF 바퀴 반지름(m)
ARM_RATE = 43 * 2 * math.pi / 60        # XM430-W350 무부하 속도(11.1 V) = 4.5029 rad/s, 팔 목표 변화율 한계(Isaac v5~와 같음)
ARM_POSE_STOW = [0.0, 1.4, -1.0, -1.3, 0.0]  # 접은 주행 자세 S0(검증계획 17장): yaw, 어깨, 팔꿈치, 손목 pitch, roll
SETTLE = 1.0                            # 명령 전 정지 구간(s)
VIDEO_W, VIDEO_H, VIDEO_FPS = 640, 480, 30


def build_model(urdf, dt, wheel_armature, arm_armature):
    spec = mujoco.MjSpec.from_file(str(urdf))
    urdf_dir = Path(urdf).resolve().parent
    for mesh in spec.meshes:  # 메시 경로를 절대 경로로: 저장한 model.xml을 어느 폴더에서든 열 수 있게
        mesh.file = str((urdf_dir / mesh.file).resolve())
    spec.option.timestep = dt
    spec.option.gravity = [0, 0, -GRAVITY]
    spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    root = next(b for b in spec.bodies if b.name == "base_footprint")
    root.add_freejoint(name="root")
    # 표시 메시(DAE)는 URDF를 읽을 때 이미 버려졌다. 컴파일 단계의 discardvisual은 아래 바닥 재질까지 지우므로 끈다(형상 수 542 그대로)
    spec.compiler.discardvisual = False
    # 보기용 바닥 무늬·조명(물리 무관)
    tex = spec.add_texture(name="grid", type=mujoco.mjtTexture.mjTEXTURE_2D, builtin=mujoco.mjtBuiltin.mjBUILTIN_CHECKER,
                           width=512, height=512)
    tex.rgb1, tex.rgb2 = [0.82, 0.84, 0.86], [0.55, 0.58, 0.62]
    mat = spec.add_material(name="grid", texrepeat=[4, 4], texuniform=True)  # 칸 0.25 m
    mat.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = "grid"
    floor = spec.worldbody.add_geom(name="floor", type=mujoco.mjtGeom.mjGEOM_PLANE, size=[5, 5, 0.1])
    floor.material = "grid"
    spec.worldbody.add_light(name="sun", type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL, pos=[0, 0, 3], dir=[-0.3, 0.2, -1],
                             diffuse=[0.8, 0.8, 0.8])
    spec.visual.quality.shadowsize = 8192  # 그림자 잡음 줄이기(보기용)
    for g in spec.geoms:
        g.friction = [MU, MU_TORSION, MU_ROLL]
    # 보기용 색(표시 메시를 버려 모두 회색이므로): 바퀴 검정, 팔·집게 주황, 라이다 파랑. 물리 무관
    color = {"wheel_": [0.12, 0.12, 0.13, 1], "arm_link_": [0.93, 0.55, 0.20, 1], "gripper_": [0.93, 0.55, 0.20, 1],
             "lidar_": [0.25, 0.45, 0.80, 1]}
    for b in spec.bodies:
        rgba = next((v for k, v in color.items() if b.name.startswith(k)), None)
        if rgba is not None:
            for g in b.geoms:
                g.rgba = rgba
    wheels = sorted(j.name for j in spec.joints if j.name.startswith("Wheel_"))
    arm = [j.name for j in spec.joints if j.name.startswith("RMX52_")]
    # armature: 모터 회전자 관성 × 기어비². URDF에는 이 항목이 없어 MuJoCo가 0으로 읽는다
    for j in spec.joints:
        if j.name in wheels:
            j.armature = wheel_armature
        elif j.name in arm:
            j.armature = arm_armature
    for name in wheels:
        a = spec.add_actuator(name="vel_" + name, target=name, trntype=mujoco.mjtTrn.mjTRN_JOINT)
        a.set_to_velocity(kv=WHEEL_KV)
        a.forcelimited, a.forcerange = 1, [-WHEEL_TORQUE, WHEEL_TORQUE]
    for name in arm:
        a = spec.add_actuator(name="pos_" + name, target=name, trntype=mujoco.mjtTrn.mjTRN_JOINT)
        a.set_to_position(kp=ARM_KP, kv=ARM_KV)
        a.forcelimited, a.forcerange = 1, [-ARM_TORQUE, ARM_TORQUE]
    a = spec.add_actuator(name="pos_gripper_left_joint", target="gripper_left_joint", trntype=mujoco.mjtTrn.mjTRN_JOINT)
    a.set_to_position(kp=GL_KP, kv=GL_KV)
    a.forcelimited, a.forcerange = 1, [-GL_FORCE, GL_FORCE]
    return spec, spec.compile(), wheels, arm


def build_schedule(test, wheels, arm):
    """(시작 s, 끝 s, {바퀴 이름: 목표 rad/s}, 이름) 목록과 총 시간. Isaac run_phys_test.py와 같은 일정."""
    t = SETTLE
    if test == "M1":
        return [], 5.0
    if test == "M5":  # 공식 한계의 2배를 명령, 제어기가 자른다
        w = 2 * WHEEL_RATE
        return [(t, t + 3.0, {n: w for n in wheels}, f"wheels {w:.3f} rad/s")], t + 3.0
    if test == "M6":  # M8(정지)을 함께 기록한다
        w = 0.05 / WHEEL_R
        return [(t, t + 4.0, {n: w for n in wheels}, "forward"), (t + 4.0, t + 6.0, {}, "stop1"),
                (t + 6.0, t + 10.0, {n: -w for n in wheels}, "backward"), (t + 10.0, t + 12.0, {}, "stop2 (M8)")], t + 12.0
    if test == "M7":  # 왼쪽 −1, 오른쪽 +1 rad/s → 반시계
        return [(t, t + 4.0, {n: (-1.0 if n.endswith("_Left") else 1.0) for n in wheels}, "spin CCW")], t + 4.0
    if test in ("V1", "V1b", "V2", "V3"):  # 처음 2 s 동안 접고 유지(Isaac run_phys_test.py V1~V3·V1b와 같은 일정)
        stow = dict(zip(arm, ARM_POSE_STOW))
        if test == "V1b":  # 18장: 접은 채 d만큼 직진(V2 속도) → 1 s 정지 → V1과 같은 4 s 회전
            w = 0.05 / WHEEL_R
            t1 = 2.0 + args.pivot_offset_m / 0.05
            spin = dict(stow, **{n: (-1.0 if n.endswith("_Left") else 1.0) for n in wheels})
            return [(0.0, 2.0, stow, "stow"), (2.0, t1, dict(stow, **{n: w for n in wheels}), "forward d (stowed)"),
                    (t1, t1 + 1.0, stow, "stop (stowed)"), (t1 + 1.0, t1 + 5.0, spin, "spin CCW (stowed)")], t1 + 5.0
        if test == "V1":
            spin = dict(stow, **{n: (-1.0 if n.endswith("_Left") else 1.0) for n in wheels})
            return [(0.0, 2.0, stow, "stow"), (2.0, 6.0, spin, "spin CCW (stowed)")], 6.0
        if test == "V2":
            w = 0.05 / WHEEL_R
            return [(0.0, 2.0, stow, "stow"), (2.0, 6.0, dict(stow, **{n: w for n in wheels}), "forward (stowed)"),
                    (6.0, 8.0, stow, "stop1"), (8.0, 12.0, dict(stow, **{n: -w for n in wheels}), "backward (stowed)"),
                    (12.0, 14.0, stow, "stop2")], 14.0
        return [(0.0, 1.0, {}, "zero"), (1.0, 4.0, stow, "to stow"), (4.0, 7.0, {}, "back to zero")], 7.0
    raise ValueError(test)


def rpy(q):
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def main():
    out = args.out.resolve()
    if out.exists():
        raise SystemExit(f"결과 폴더가 이미 있다: {out}")
    spec, m, wheels, arm = build_model(args.urdf, args.dt, args.wheel_armature, args.arm_armature)
    out.mkdir(parents=True)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    (out / "model.xml").write_text(spec.to_xml(), encoding="utf-8")  # 실제로 쓴 장면(MJCF). 뷰어로 다시 열 수 있다

    floor = m.geom("floor").id
    root_body = m.body("base_footprint").id
    wheel_body = {n: m.jnt_bodyid[m.joint(n).id] for n in wheels}
    body_wheel = {b: n for n, b in wheel_body.items()}
    links = [b for b in range(1, m.nbody) if m.body_mass[b] > 0]  # 질량 있는 링크(속도 판정용)
    jq = {n: m.jnt_qposadr[m.joint(n).id] for n in wheels + arm + ["gripper_left_joint", "gripper_right_joint"]}
    jv = {n: m.jnt_dofadr[m.joint(n).id] for n in wheels}
    act = {n: m.actuator("vel_" + n).id for n in wheels}
    arm_act = {n: m.actuator("pos_" + n).id for n in arm}
    arm_bodies = {b for b in range(m.nbody) if m.body(b).name.startswith(("arm_link_2", "arm_link_3", "arm_link_4", "arm_link_5", "gripper"))}
    arm_sent = np.zeros(len(arm))
    seq = None
    if args.test in ("V4", "V5", "V6", "V6b", "V7", "V6c", "V6cb", "V7c", "V8"):  # 시간·yaw·중심선 추종·헤딩 보정 구간(19~23장)
        mp = {"corner": args.corner, "side": 1 if args.side == "L" else -1, "radius_m": args.radius_m, "deadend": args.deadend,
              "start_heading_deg": args.start_heading_deg, "speed": args.speed}
        phases, duration = maneuver.build(args.test, mp, [n for n in wheels if n.endswith("_Left")],
                                          [n for n in wheels if n.endswith("_Right")], dict(zip(arm, ARM_POSE_STOW)))
        seq, schedule = maneuver.Sequencer(phases, maneuver.LocError(
            *args.loc_bias, *args.loc_noise, seed=args.seed) if args.test in ("V7", "V7c") else None), []
    else:
        schedule, duration = build_schedule(args.test, wheels, arm)
    n_steps = int(round(duration / args.dt))
    total_mass = float(m.body_subtreemass[root_body])
    wheel_xy0 = {n: [float(v) for v in d.xpos[wheel_body[n]][:2]] for n in wheels}  # 시작 자세에서 바퀴 중심

    header = (["step", "t_s", "segment", "base_x_m", "base_y_m", "base_z_m", "roll_rad", "pitch_rad", "yaw_rad",
               "com_x_m", "com_y_m", "com_z_m"]
              + [f"q_{n}" for n in jq] + [f"dq_{n}" for n in wheels] + [f"cmd_{n}" for n in wheels]
              + [f"Fn_{n}" for n in wheels] + ["Fn_floor_total_N", "Fn_floor_other_N", "cop_x_m", "cop_y_m",
                                                "self_contacts", "finite"]
              + [f"cmd_{n}" for n in arm] + ["arm_contact_min_dist_m"]
              + [f"{m.body(b).name}_{a}_m" for b in links for a in "xyz"])

    writer_video = renderer = cam = None
    if args.video:
        renderer = mujoco.Renderer(m, VIDEO_H, VIDEO_W)
        cam = mujoco.MjvCamera()
        cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        cam.distance, cam.azimuth, cam.elevation = 1.1, 135.0, -28.0
        writer_video = subprocess.Popen(
            ["ffmpeg", "-loglevel", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{VIDEO_W}x{VIDEO_H}",
             "-r", str(VIDEO_FPS), "-i", "-", "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20", str(out / "video.mp4")],
            stdin=subprocess.PIPE)
    viewer = None
    if args.view:
        from mujoco import viewer as mj_viewer  # 함수 안에서 "import mujoco.viewer"를 쓰면 mujoco가 지역 이름이 된다
        viewer = mj_viewer.launch_passive(m, d)

    frames, next_frame_t = [], 0.0
    nonfinite_t = None
    t_wall = time.time()
    with (out / "log.csv").open("w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(header)
        for step in range(n_steps):
            t_now = d.time
            if seq is not None:
                name, tgt = seq.update(t_now, rpy(d.xquat[root_body])[2], *d.xpos[root_body][:2])
                seg = (None, None, tgt, name)
            else:
                seg = next((s for s in schedule if s[0] <= t_now < s[1]), None)
            for n in wheels:  # 제어기 속도 한계(Isaac v4~와 같음)
                d.ctrl[act[n]] = float(np.clip((seg[2] if seg else {}).get(n, 0.0), -WHEEL_RATE, WHEEL_RATE))
            arm_tgt = np.array([(seg[2] if seg else {}).get(n, 0.0) for n in arm])  # 팔 위치 목표, 한 스텝 변화 ≤ ARM_RATE × dt
            arm_sent = arm_sent + np.clip(arm_tgt - arm_sent, -ARM_RATE * args.dt, ARM_RATE * args.dt)
            for n, v in zip(arm, arm_sent):
                d.ctrl[arm_act[n]] = float(v)
            mujoco.mj_step(m, d)

            fn = {n: 0.0 for n in wheels}
            other, self_contacts, cop = 0.0, 0, np.zeros(2)
            arm_min = float("nan")  # 팔(링크 2~5·집게)과 그 밖 형상 사이 접촉의 최소 거리(m, ≤ 0이면 실제 접촉)
            f6 = np.zeros(6)
            for i in range(d.ncon):
                c = d.contact[i]
                b1, b2 = m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2]
                if (b1 in arm_bodies) != (b2 in arm_bodies):
                    arm_min = c.dist if np.isnan(arm_min) else min(arm_min, c.dist)
                if c.geom1 != floor and c.geom2 != floor:
                    self_contacts += 1
                    continue
                mujoco.mj_contactForce(m, d, i, f6)
                body = m.geom_bodyid[c.geom2 if c.geom1 == floor else c.geom1]
                if body in body_wheel:
                    fn[body_wheel[body]] += f6[0]
                else:
                    other += f6[0]
                cop += f6[0] * c.pos[:2]
            total = sum(fn.values()) + other
            cop = cop / total if total > 0 else np.full(2, np.nan)
            finite = bool(np.isfinite(d.qpos).all() and np.isfinite(d.qvel).all())
            if not finite and nonfinite_t is None:
                nonfinite_t = d.time
            r, p, y = rpy(d.xquat[root_body])
            com = d.subtree_com[root_body]
            row = ([step, d.time, seg[3] if seg else "hold0", *d.xpos[root_body], r, p, y, *com]
                   + [d.qpos[jq[n]] for n in jq] + [d.qvel[jv[n]] for n in wheels] + [d.ctrl[act[n]] for n in wheels]
                   + [fn[n] for n in wheels] + [total, other, cop[0], cop[1], self_contacts, finite]
                   + list(arm_sent) + [arm_min]
                   + [v for b in links for v in d.xpos[b]])
            w.writerow(row)

            if renderer is not None and d.time + 1e-9 >= next_frame_t:
                cam.lookat[:] = [d.xpos[root_body][0], d.xpos[root_body][1], 0.12]
                renderer.update_scene(d, camera=cam)
                writer_video.stdin.write(renderer.render().tobytes())
                frames.append(d.time)
                next_frame_t += 1.0 / VIDEO_FPS
            if viewer is not None:
                if not viewer.is_running():
                    break
                viewer.sync()
                lag = d.time - (time.time() - t_wall)
                if lag > 0:
                    time.sleep(lag)
            if not finite:
                break
    wall = time.time() - t_wall
    if writer_video is not None:
        writer_video.stdin.close()
        writer_video.wait()
        renderer.close()
        (out / "frames.json").write_text(json.dumps({"fps": VIDEO_FPS, "t_s": frames}), encoding="utf-8")

    summary = {
        "test": args.test, "engine": "MuJoCo", "mujoco_version": mujoco.__version__, "urdf": str(args.urdf.resolve()),
        "steps_recorded": step + 1, "expected_steps": n_steps, "nonfinite_first_t": nonfinite_t,
        "wall_time_s": wall, "video": bool(args.video), "viewer": bool(args.view),
        "schedule": [[s[0], s[1], s[2], s[3]] for s in schedule],
        "model": {"nq": m.nq, "nv": m.nv, "nu": m.nu, "neq": m.neq, "ngeom": m.ngeom, "total_mass_kg": total_mass,
                  "weight_N": total_mass * GRAVITY, "wheels": wheels, "arm": arm,
                  "wheel_tire_radius_m": {n: float(max(m.geom_size[g][0] for g in range(m.ngeom)
                                                       if m.geom_bodyid[g] == wheel_body[n] and m.geom_type[g] == mujoco.mjtGeom.mjGEOM_CYLINDER))
                                          for n in wheels},
                  "wheel_xy_at_start_m": wheel_xy0},
        "conditions": {"dt_s": args.dt, "wheel_armature_kgm2": args.wheel_armature, "arm_armature_kgm2": args.arm_armature, "gravity": GRAVITY, "mu_slide": MU, "mu_torsion": MU_TORSION, "mu_roll": MU_ROLL,
                       "condim": int(m.geom_condim[floor]), "cone": "pyramidal" if m.opt.cone == 0 else "elliptic",
                       "integrator": "implicitfast", "solref_default": [float(v) for v in m.geom_solref[floor]],
                       "solimp_default": [float(v) for v in m.geom_solimp[floor]],
                       "wheel_kv": WHEEL_KV, "wheel_torque_Nm": WHEEL_TORQUE, "wheel_rate_limit_rad_s": WHEEL_RATE,
                       "arm_kp": ARM_KP, "arm_kv": ARM_KV, "arm_torque_Nm": ARM_TORQUE,
                       "gripper_left_kp": GL_KP, "gripper_left_kv": GL_KV, "settle_s": SETTLE,
                       "arm_rate_limit_rad_s": ARM_RATE, "arm_pose_stow": ARM_POSE_STOW,
                       "pivot_offset_m": args.pivot_offset_m},
        "maneuver": None if seq is None else {
            "params": {"corner": args.corner, "side": args.side, "radius_m": args.radius_m, "deadend": args.deadend,
                       "start_heading_deg": args.start_heading_deg, "k_head": maneuver.K_HEAD, "k_lat": maneuver.K_LAT,
                       "w_max": maneuver.W_MAX, "loc_bias": args.loc_bias, "loc_noise": args.loc_noise,
                       "seed": args.seed,
                       "v_mps": args.speed if args.test == "V8" else maneuver.V, "track_m": maneuver.TRACK, "wheel_r_m": maneuver.WHEEL_R},
            "phases": maneuver.describe(seq.phases), "phase_log": seq.log, "align": seq.align},
    }
    (out / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"{args.test} dt={args.dt:g} steps={step + 1}/{n_steps} wall={wall:.1f}s → {out}")
    if viewer is not None:
        viewer.close()


if __name__ == "__main__":
    main()
