#!/usr/bin/env python3
"""팔을 대기 자세 (STOW) 범위 안으로 (2026-11-14, 젯슨, **브링업 끈 상태**). 브링업이 '팔이 대기 자세가 아님 — 시작 거부' 를 낼 때.

    python3 ~/arm_stow.py           확인 -> 범위 밖 관절만 범위 가운데로 (계획을 보여 주고 Enter 를 눌러야 움직임)
    python3 ~/arm_stow.py --check   확인만 (쓰기 없음)

- 팔 보드 포트를 찾음 (/dev/ttyACM0 · 1 중 role 2). 범위 = 팀 opencr.ros2_control.xacro arm_standby_min/max_tick (브링업 검사와 같음).
- 움직임은 팀 opencr_arm_tool.py move --stages (토크 ON -> REARM -> 한 관절씩 0.1 rad/s, 오류 · 추종 오차 · Ctrl-C 면 멈춤).
- 차례 = 시작 정리 (STOW -> DRIVE: 관절 3 -> 2 -> 4 -> 1 -> 5) 의 거꾸로: 5 -> 1 -> 4 -> 2 -> 3.
  범위에서 15° 넘게 벗어난 관절이 있으면 (예: DRIVE 자세) 전부 접는 것이라 한 번 더 묻는다.
- 끝나면 팔은 토크를 켠 채 HOLD (펌웨어). 그대로 브링업을 띄우면 됨 (HOLD 면 대기 자세 검사를 건너뜀).
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
    print("옮길 관절 (차례대로, 0.1 rad/s): " + " -> ".join(stages))
    if check:
        return
    if far:
        print(f"!! {FAR_DEG:.0f}° 넘게 벗어난 관절이 있음 = 팔을 크게 접는 동작. 팔 주변 (팔레트 퍽 · 상자 · 사람) 을 비우고 지켜볼 것")
    if input("Enter = 움직임, 다른 키 = 그만: ").strip():
        print("그만둠")
        return
    r = subprocess.run([sys.executable, TOOL, "move", "--port", port, "--stages", ",".join(stages), "--yes"])
    if r.returncode != 0:
        sys.exit("이동 중 멈춤 (위 메시지 확인). 팔은 그 자리에서 HOLD")
    time.sleep(0.5)
    link = oat.Link(port); s = link.wait_state(2.0); link.ser.close()
    ticks = [oat.tick_of(q) for q in oat.arm_pos(s)]
    bad = [j for j in range(1, 6) if not smin[j - 1] <= ticks[j - 1] <= smax[j - 1]]
    print("대기 자세 범위 안 — 이제 브링업 띄우면 됨" if not bad else f"아직 범위 밖 관절 {bad} (처짐?) -> 한 번 더 실행")


if __name__ == "__main__":
    main()
