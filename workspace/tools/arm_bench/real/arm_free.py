#!/usr/bin/env python3
"""칸 안에서 멈춘 팔 빼내기 (2026-11-14, 젯슨, 브링업 상태): 집게 벌림 -> 손끝 4 cm 위로 -> 높은 자세 -> 정면.
slot_teach · meas_session 을 중간에 끝내 팔이 팔레트 칸에 남았을 때. 이동마다 계획을 보여 주고 Enter (s = 건너뜀, q = 끝).

    python3 ~/arm_free.py            벌림 47 mm (퍽을 놓고 빠짐)
    python3 ~/arm_free.py --keep     집게 그대로 (퍽을 쥔 채 빠짐)
그다음 주행 자세: ~/arm_cmd.sh real pose DRIVE
"""
import os
import sys

sys.path.insert(0, os.path.expanduser("~/Lv2_Cartlidar_Assignment/workspace/tools/arm_bench/real"))
import meas_session as m  # noqa: E402

OPEN_47MM = -0.30   # rad, 실측 3 점 (-0.764 = 66.7, 0.0798 = 36.4 mm) 사이 직선으로 약 47 mm


def main():
    a = m.Arm()
    q, g = a.show()
    try:
        if "--keep" not in sys.argv:
            a.move(a.cmd, OPEN_47MM, label="집게 벌림 (약 47 mm)")
        up, err = m.cart_step(a.cmd, [0, 0, 0.04])
        if err < 1.0:
            a.move(up, label="손끝 4 cm 위로")
        else:
            m.P("  (위로 4 cm 를 못 풂 -> 건너뜀)")
        a.move([a.cmd[0]] + m.HIGH, label="높은 자세", route=False)
        a.move([0] + m.HIGH, label="정면 (관절 1 0)", route=False)
    except m.Back:
        m.P("  (끝냄)")
    m.P("다음: ~/arm_cmd.sh real pose DRIVE")


if __name__ == "__main__":
    main()
