"""wheel_kinematics 단위 시험 (python3 -m pytest 없이도: python3 test_wheel_kinematics.py)"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from wheel_kinematics import DiffDrive, load_config  # noqa: E402

CFG = load_config(os.path.join(os.path.dirname(os.path.abspath(__file__)), "config", "wheels.yaml"))
DD = DiffDrive(CFG)
R, Y = 0.0330006, 0.1396


def close(a, b, tol=1e-9):
    return all(abs(x - y) <= tol for x, y in zip(a, b))


def test_config():
    assert DD.joints == ["Wheel_Revolute_Front_Left", "Wheel_Revolute_Front_Right",
                         "Wheel_Revolute_Rear_Left", "Wheel_Revolute_Rear_Right"]
    assert DD.r == R and DD.limit == 7.3304 and DD.timeout == 0.5


def test_straight():
    assert close(DD.wheel_rates(0.10, 0.0), [0.10 / R] * 4)


def test_spin_ccw():          # 반시계: 왼쪽(+y) 뒤로, 오른쪽 앞으로 (물리 시험 P7과 같은 부호)
    w = DD.wheel_rates(0.0, 0.5)
    assert close(w, [-0.5 * Y / R, 0.5 * Y / R, -0.5 * Y / R, 0.5 * Y / R])


def test_limit_keeps_ratio():
    w = DD.wheel_rates(0.5, 1.0)
    raw = [(0.5 - 1.0 * Y) / R, (0.5 + 1.0 * Y) / R]
    s = 7.3304 / max(abs(x) for x in raw)
    assert math.isclose(max(abs(x) for x in w), 7.3304)
    assert close(w, [raw[0] * s, raw[1] * s, raw[0] * s, raw[1] * s])


def test_timeout():
    assert DD.rates_at(10.0, 9.6, 0.1, 0.0) == DD.wheel_rates(0.1, 0.0)
    assert DD.rates_at(10.0, 9.4, 0.1, 0.0) == [0.0] * 4
    assert DD.rates_at(10.0, None, 0.1, 0.0) == [0.0] * 4


if __name__ == "__main__":
    n = 0
    for k, f in list(globals().items()):
        if k.startswith("test_"):
            f()
            n += 1
    print(f"{n}개 통과")
