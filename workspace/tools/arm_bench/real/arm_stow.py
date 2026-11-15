#!/usr/bin/env python3
"""팔을 대기 자세 (STOW) 범위 안으로 (2026-11-14, 젯슨, **브링업 끈 상태**). 브링업이 '팔이 대기 자세가 아님 — 시작 거부' 를 낼 때.

    python3 ~/arm_stow.py           확인 -> 범위 밖 관절만 범위 가운데로 (계획을 보여 주고 Enter 를 눌러야 움직임)
    python3 ~/arm_stow.py --check   확인만 (쓰기 없음)

- 팔 보드 포트를 찾음 (/dev/ttyACM0 · 1 중 role 2). 범위 = 팀 opencr.ros2_control.xacro arm_standby_min/max_tick (브링업 검사와 같음).
- 움직임은 팀 opencr_arm_tool.py move --stages (토크 ON -> REARM -> 한 관절씩, 오류 · 추종 오차 · Ctrl-C 면 멈춤).
  속도 = manipulator.yaml startup.max_joint_velocity (0 이면 arm.max_joint_velocity, 2026-11-14 속도 기준 하나) — 노드 켜질 때 정리와 같음.
- 차례 = 시작 정리 (STOW -> DRIVE: 관절 3 -> 2 -> 4 -> 1 -> 5) 의 거꾸로: 5 -> 1 -> 4 -> 2 -> 3.
  범위에서 15° 넘게 벗어난 관절이 있으면 (예: DRIVE 자세) 전부 접는 것이라 한 번 더 묻는다.
- 끝나면 팔은 토크를 켠 채 HOLD (펌웨어). 그대로 브링업을 띄우면 됨 (HOLD 면 대기 자세 검사를 건너뜀).
- [11-14 고침] 예전 STOW (관절 1 1.554 rad = 89°, #277 전 xacro 범위 3004 ~ 3118) 는 접을 때 link3 이 1 열 끝 칸 (1_3) 벽에 닿음
  (팀 계산 −3.5 mm, 실물 11-13 · 11-14 1_3 위에 걸림). 젯슨 저장소가 예전 STOW 범위면 움직이지 않고 업데이트를 안내.
- [11-15] STOW 관절 1 2.200 (126°, #277) -> 1.8715 rad (107°, 범위 3211 ~ 3325) · 관절 5 -> -0.1304 (실물 대기 자리). #277 범위도 예전 STOW 로 봄.
- [11-16] 관절 1 모터를 정상으로 다시 설치 -> STOW 를 #277 로 되돌림 (관절 1 2.200 rad = 126°, 범위 3425 ~ 3539 · 관절 5 0.043).
  11-15 범위 (위 끝 3325) 를 예전 STOW 로 봄.
- [11-16 오후] 평소 대기 자리 실물 읽기로 STOW 를 옮김 (관절 1 −0.9633 rad = 1420 tick, 범위 1363 ~ 1477 · 관절 5 0 = 2048, 범위 1849 ~ 2246 그대로).
  #311 범위 (위 끝 3539) 도 예전 STOW 로 봄 — 그 판으로 돌리면 접힌 팔을 관절 1 로 반 바퀴 돌리려 함 (왼쪽 스토리지 1 열에 닿음).
  움직이기 전에 접는 경로 틈을 계산 (tools/arm_bench/fold_clearance.py, 팀 URDF 팔레트 v5 · 퍽 10 개):
  모델 그대로 + 실측 칸 맞춤 (pallet_slot_teach.json 의 1_3, 관절 2 · 3 · 4) 으로 옮긴 실물 추정, 둘 중 작은 값이 MIN_CLEAR_MM 미만이면 거부.
"""
import glob
import importlib.util
import os
import subprocess
import sys
import time

TOOL = os.path.expanduser("~/Lv2_Cartlidar_Assignment/workspace/ros2_ws/src/opencr_hardware/scripts/opencr_arm_tool.py")
ORDER = [5, 1, 4, 2, 3]          # 접는 차례 (관절 번호)
FAR_DEG = 15.0
WS = os.path.expanduser("~/Lv2_Cartlidar_Assignment/workspace")
OLD_J1_MAX = {3118: "89° (#277 전)", 3325: "107° (11-15, 모터 돌아 설치)", 1266: "−74° (11-15 잘못 조립 때)",
              3539: "126° (#277 · 11-16 오전 #311)"}   # 관절 1 대기 범위 위 끝 -> 예전 STOW (11-16 오후부터 −55°, 1420 tick)
MIN_CLEAR_MM = 5.0


def fold_speed():
    """manipulator.yaml 의 켜질 때 정리 속도 (startup.max_joint_velocity, 0 = arm.max_joint_velocity) [rad/s]."""
    import yaml
    node = next(iter(yaml.safe_load(open(os.path.join(WS, "ros2_ws/src/control/config/manipulator.yaml"), encoding="utf-8")).values()))["ros__parameters"]
    v = float(node.get("startup", {}).get("max_joint_velocity", 0.0) or 0.0)
    if v <= 0.0:
        v = float(node["arm"]["max_joint_velocity"])
    cap = 0.5 if "args.speed <= 0.5" in open(TOOL, encoding="utf-8").read() else 1.0   # opencr_arm_tool 상한 (예전 판 0.5)
    if v > cap:
        print(f"  (opencr_arm_tool 상한 {cap} rad/s 라 {v} -> {cap})")
    return min(v, cap)


def measured_shift():
    """실측 칸 맞춤 1_3: 맞춘 관절 - 모델 쥐는 자리 (관절 2 · 3 · 4, rad). 실물 ≈ 모델 (명령 − 이 값). 없으면 None."""
    import json
    rec = None
    for p in (os.path.join(WS, "ros2_ws/src/control/config/pallet_slot_teach.json"), os.path.expanduser("~/실측/slot_teach.json")):
        if os.path.exists(p):
            d = json.load(open(p, encoding="utf-8"))
            d = d.get("slots", d)
            if "1_3" in d:
                rec = d["1_3"]["q_meas"]
                break
    model = os.path.join(WS, "urdf_and_usd/policy/manipulator/pallet_place_traj.json")   # 모델 판 (실측 보정 안 된 사본)
    if rec is None or not os.path.exists(model):
        return None
    s = next(x for x in json.load(open(model, encoding="utf-8"))["slots"] if (x["row"], x["index"]) == (1, 3))
    g = next(x for x in s["segments"] if x["name"] == "place")["q"][-1]
    return [0.0] + [rec[j] - g[j] for j in (1, 2, 3)] + [0.0]


def path_clearance(q0, q1, order):
    """접는 경로 최소 틈 (mm): (모델, 실측 추정, 가장 가까운 것 설명). fold_clearance.py 가 없으면 None."""
    fc = os.path.join(WS, "tools/arm_bench/fold_clearance.py")
    if not os.path.exists(fc):
        return None
    sp = importlib.util.spec_from_file_location("fold_clearance", fc)
    F = importlib.util.module_from_spec(sp)
    sp.loader.exec_module(F)
    import io
    import contextlib
    import numpy as np
    m = F.Model(os.path.join(F.DESC, "urdf/waffle_4wd_loaded.urdf"))
    obs, tags = m.obstacle_points("base_link")
    boxes = m.arm_boxes()
    wp = F.path(q0, q1, [[j - 1] for j in order])
    out = []
    shift = measured_shift()
    for name, w in (("모델", wp), ("실측 추정", None if shift is None else
                                    [(st, np.r_[q[:5] - np.array(shift), q[5]]) for st, q in wp])):
        if w is None:
            continue
        with contextlib.redirect_stdout(io.StringIO()):
            _, per = F.evaluate(m, obs, tags, boxes, w, name)
        st, (d, ln, tg, _) = min(per.items(), key=lambda kv: kv[1][0])
        out.append((name, d * 1000, f"{st} 단계 {ln} <-> {tg}"))
    return out

spec = importlib.util.spec_from_file_location("oat", TOOL)
oat = importlib.util.module_from_spec(spec)
spec.loader.exec_module(oat)


def main():
    check = "--check" in sys.argv
    if subprocess.run(["pgrep", "-f", "[r]os2_control_node"], capture_output=True).returncode == 0:
        sys.exit("브링업이 떠 있음 (보드 포트 사용 중) -> 브링업 터미널에서 Ctrl+C 로 끈 뒤 다시")
    port, st = None, None
    for p in sorted(glob.glob("/dev/ttyACM*")):
        if not os.path.exists(p):
            continue
        link = oat.Link(p)
        s = link.wait_state(2.0)
        link.ser.close()
        if s and s["role"] == oat.ROLE_ARM:
            port, st = p, s
            break
    if port is None:
        sys.exit("팔 보드 (role 2) 를 못 찾음 — USB · 모터 전원 확인")
    if st["error"] & oat.ERR_ABORT:
        sys.exit(f"팔 보드 오류 {oat.err_text(st['error'])} -> 모터 전원 켠 채 팔 OpenCR 리셋 뒤 다시")
    smin, smax = oat.load_standby()
    if smax[0] in OLD_J1_MAX:
        sys.exit("!! 젯슨 저장소의 대기 자세가 예전 STOW (관절 1 %s, 범위 %d ~ %d).\n"
                 "   저장소를 11-16 오후 STOW (관절 1 −0.9633 rad = 1420 tick, 범위 1363 ~ 1477) 이후로 업데이트 · 빌드한 뒤 다시 (README 참고). 움직이지 않음."
                 % (OLD_J1_MAX[smax[0]], smin[0], smax[0]))
    ticks = [oat.tick_of(q) for q in oat.arm_pos(st)]
    print(f"팔 보드 {port} (오류 {oat.err_text(st['error'])})")
    stages, far = [], False
    for j in range(1, 6):
        t, lo, hi = ticks[j - 1], smin[j - 1], smax[j - 1]
        ok = lo <= t <= hi
        off = 0 if ok else (t - hi if t > hi else t - lo)
        print(f"  관절 {j}: {t} tick (범위 {lo} ~ {hi}) " + ("범위 안" if ok else f"밖 {off:+d} tick = {off * 360 / 4096:+.1f}°"))
        if not ok:
            far |= abs(off) * 360 / 4096 > FAR_DEG
    for j in ORDER:
        t, lo, hi = ticks[j - 1], smin[j - 1], smax[j - 1]
        if not lo <= t <= hi:
            stages.append(f"{j}:{(lo + hi) // 2}")
    print(f"  집게: {ticks[5]} tick (범위 {smin[5]} ~ {smax[5]}, 아무 값이나 됨)" if smin[5] <= ticks[5] <= smax[5]
          else f"  집게: {ticks[5]} tick — 범위 {smin[5]} ~ {smax[5]} 밖 (손으로 벌리거나 닫아 주세요)")
    if not stages:
        print("대기 자세 범위 안 — 그대로 브링업 띄우면 됨")
        return
    speed = fold_speed()
    print(f"옮길 관절 (차례대로, {speed:g} rad/s = manipulator.yaml): " + " -> ".join(stages))
    q0 = list(oat.arm_pos(st))
    q1 = list(q0)
    for sg in stages:
        j, t = (int(v) for v in sg.split(":"))
        q1[j - 1] = oat.rad_of(t)
    print("접는 경로 틈 계산 중 (팀 URDF 팔레트 v5 · 퍽 10 개, 약 20 초) ...")
    cl = path_clearance(q0, q1, [int(sg.split(":")[0]) for sg in stages])
    if cl is None:
        print("  !! fold_clearance.py 없음 (저장소가 #277 전) -> 틈 계산 못 함")
    else:
        for name, mm, where in cl:
            print(f"  {name}: 최소 {mm:+.1f} mm ({where})")
    if check:
        return
    if cl is None or min(mm for _, mm, _ in cl) < MIN_CLEAR_MM:
        sys.exit(f"!! 경로 틈이 {MIN_CLEAR_MM:.0f} mm 미만이거나 계산을 못 함 -> 움직이지 않음 (팔레트 · 몸체에 닿을 수 있음). 손으로 옮기거나 저장소 업데이트")
    if far:
        print(f"!! {FAR_DEG:.0f}° 넘게 벗어난 관절이 있음 = 팔을 크게 접는 동작. 팔 주변 (팔레트 퍽 · 상자 · 사람) 을 비우고 지켜볼 것")
    if input("Enter = 움직임, 다른 키 = 그만: ").strip():
        print("그만둠")
        return
    r = subprocess.run([sys.executable, TOOL, "move", "--port", port, "--stages", ",".join(stages), "--speed", f"{speed:g}", "--yes"])
    if r.returncode != 0:
        sys.exit("이동 중 멈춤 (위 메시지 확인). 팔은 그 자리에서 HOLD")
    time.sleep(0.5)
    link = oat.Link(port); s = link.wait_state(2.0); link.ser.close()
    ticks = [oat.tick_of(q) for q in oat.arm_pos(s)]
    bad = [j for j in range(1, 6) if not smin[j - 1] <= ticks[j - 1] <= smax[j - 1]]
    print("대기 자세 범위 안 — 이제 브링업 띄우면 됨" if not bad else f"아직 범위 밖 관절 {bad} (처짐?) -> 한 번 더 실행")


if __name__ == "__main__":
    main()
