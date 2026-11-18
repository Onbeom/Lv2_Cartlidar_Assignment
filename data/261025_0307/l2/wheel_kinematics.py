"""차동 구동 기구학: 차체 속도 명령(v, ω) → 바퀴별 각속도 목표.

바퀴 목록(관절 이름·y 위치·반지름·방향 부호)은 설정 파일(config/wheels.yaml)에서만 읽는다.
바퀴 수를 가정하지 않고 목록의 바퀴마다 같은 식을 쓴다: ω_i = sign_i · (v − ω·y_i) / r
같은 쪽(같은 y) 바퀴는 같은 속도가 된다. 바퀴마다 한계로 자르면 비율이 깨지므로,
가장 빠른 바퀴가 한계에 닿으면 모든 바퀴를 같은 비율로 줄인다(회전 반경 유지).
표준 라이브러리만 쓴다(Isaac Sim Python과 시스템 Python 모두에서 import).
"""

import re


def load_config(path):
    """wheels.yaml을 읽는다. PyYAML이 없는 환경도 있어서 이 파일 형식(평평한 키 + wheels 목록)만 직접 읽는다."""
    cfg, wheels, cur = {}, [], None
    with open(path) as f:
        for raw in f:
            line = raw.split("#", 1)[0].rstrip()
            if not line.strip():
                continue
            m = re.match(r"^(\s*)(-\s+)?(\w+):\s*(.*)$", line)
            if not m:
                raise ValueError(f"읽을 수 없는 줄: {raw!r}")
            indent, dash, key, val = m.groups()
            if not indent and not dash:
                if key == "wheels":
                    continue
                cfg[key] = _value(val)
                continue
            if dash:
                cur = {}
                wheels.append(cur)
            cur[key] = _value(val)
    cfg["wheels"] = wheels
    for w in wheels:
        for k in ("name", "joint", "y_m", "sign"):
            if k not in w:
                raise ValueError(f"바퀴 {w}에 {k}가 없다")
    return cfg


def _value(s):
    s = s.strip()
    try:
        return int(s)
    except ValueError:
        pass
    try:
        return float(s)
    except ValueError:
        return s


class DiffDrive:
    def __init__(self, cfg):
        self.r = float(cfg["radius_m"])
        self.limit = float(cfg["max_wheel_rate_radps"])
        self.timeout = float(cfg["command_timeout_s"])
        self.wheels = cfg["wheels"]
        self.joints = [w["joint"] for w in self.wheels]

    def wheel_rates(self, v, w):
        """(v [m/s], ω [rad/s]) → 바퀴 목록 순서의 각속도 [rad/s] 목록"""
        raw = [w_["sign"] * (v - w * w_["y_m"]) / self.r for w_ in self.wheels]
        peak = max((abs(x) for x in raw), default=0.0)
        if peak > self.limit:
            raw = [x * self.limit / peak for x in raw]
        return raw

    def rates_at(self, now, last_cmd_time, v, w):
        """명령 끊김 처리 포함: 마지막 명령이 timeout보다 오래됐으면 0"""
        if last_cmd_time is None or now - last_cmd_time > self.timeout:
            return [0.0] * len(self.wheels)
        return self.wheel_rates(v, w)
