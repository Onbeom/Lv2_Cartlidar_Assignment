"""V4 모퉁이 호 회전·V5 막다른 길 후진 탈출(검증계획 19장), V6 출발 칸 회전 + 정상화·V6b 호 회전 + 정상화(20장),
V7 위치 추정 오차가 있을 때의 정상화(21장), V6c·V6cb·V7c 정지 후 헤딩 보정(22장), V8 속도를 높인 V4(23장)의 구간 순서. Isaac run_phys_test.py와 MuJoCo mujoco_drive.py가 함께 쓴다.

구간마다 끝나는 조건이 있다: 시간('t', s), 시작 때 yaw에서 바뀐 양('yaw', rad, 부호 = 방향),
또는 중심선 추종('track': 목표 지점까지 가면 끝, 목표는 매 스텝 위치·yaw로 계산).
22장 헤딩 보정은 'avg'(멈춘 채 정해진 시간 동안 추정 헤딩 오차를 평균)와 'align'(그 평균만큼 제자리에서 돌림,
회전한 양은 실제 yaw 변화로 잰다 = IMU 상대 각도를 흉내 냄) 두 구간으로 한다.
yaw 조건은 실물에서 IMU 헤딩으로 회전을 끝내는 것을 흉내 낸다(시뮬레이션에서는 차체 실제 yaw를 쓴다).
속도 명령 (v, ω) → 바퀴 속도는 차동 구동 기구학으로 바꾼다. 미끄럼 보정은 하지 않는다.
"""
import math
import random
import sys
from pathlib import Path

# 바퀴 정의는 시뮬레이션·실물 공용 설정 파일 하나에서 읽는다(l2/config/wheels.yaml, 값 출처는 그 파일에 적음)
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "l2"))
from wheel_kinematics import load_config  # noqa: E402

WHEELS = load_config(Path(__file__).resolve().parents[2] / "l2" / "config" / "wheels.yaml")

V = 0.05                     # 직진·호 회전 속도(m/s), P6·V2와 같음
_Y = [w["y_m"] for w in WHEELS["wheels"]]
TRACK = max(_Y) - min(_Y)    # 좌우 바퀴 중심 간격(m)
WHEEL_R = WHEELS["radius_m"]  # 바퀴 반지름(m)
WHEEL_RATE = WHEELS["max_wheel_rate_radps"]    # 바퀴 속도 한계(rad/s), 제어기 한계
WHEEL_TORQUE = WHEELS["max_wheel_torque_Nm"]   # 바퀴 토크 상한(N·m)
# 미로 조각 치수(maze_clearance.py와 같음): 들어오는 통로 방향별 칸 길이, 나가는 칸 가운데까지 옆 거리
CELL = {"A": 0.734, "B": 0.788}
EXIT_Y = {"A": 0.394 + 0.788 / 2, "B": 0.367 + 0.734 / 2}
# 정상화(중심선 추종, 20장): ω = −(K_HEAD · 헤딩 오차 + K_LAT · 옆 오차), ±W_MAX로 자른다.
# 옆 오차 응답 고유 진동수 0.8 rad/s·감쇠비 0.9로 잡은 첫 값(조정 안 함): K_LAT · V = 0.8², K_HEAD = 2 · 0.9 · 0.8
K_HEAD, K_LAT, W_MAX = 1.44, 12.8, 0.5
SPIN = 1.0                   # 제자리 회전 바퀴 속도(rad/s), P7·V1과 같음
# 정지 후 헤딩 보정(22장 D1): 1 s 평균한 추정 헤딩 오차가 0.5°보다 크면 바퀴 ±0.3 rad/s로 돌리고,
# 남은 각도가 0.2° 이하가 되면 멈춘다
ALIGN_SPIN, ALIGN_SKIP_DEG, ALIGN_STOP_DEG, ALIGN_AVG_S, ALIGN_SETTLE_S = 0.3, 0.5, 0.2, 1.0, 0.5


def wheel_targets(v, w, wheels_left, wheels_right):
    vl, vr = (v - w * TRACK / 2) / WHEEL_R, (v + w * TRACK / 2) / WHEEL_R
    return {**{n: vl for n in wheels_left}, **{n: vr for n in wheels_right}}


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def tracker(p0, psi, wheels_left, wheels_right, stow):
    """시작 프레임의 직선(점 p0, 방향 psi)을 따라가는 목표 함수(오차 → 목표)와 오차 함수(자세 → 오차)."""
    ux, uy = math.cos(psi), math.sin(psi)

    def errors(x, y, yaw):
        dx, dy = x - p0[0], y - p0[1]
        return dx * ux + dy * uy, -dx * uy + dy * ux, wrap(yaw - psi)   # 진행, 옆(왼쪽 +), 헤딩

    def targets(e_lat, e_head):
        w = max(-W_MAX, min(W_MAX, -(K_HEAD * e_head + K_LAT * e_lat)))
        return dict(stow, **wheel_targets(V, w, wheels_left, wheels_right))
    return targets, errors


class LocError:
    """제어기에 주는 위치의 오차(21장 V7). 추종 오차(진행, 옆, 헤딩)에 고정 편향과 잡음을 더한다.

    잡음은 rate_hz마다 새로 뽑아 다음 갱신까지 유지한다(위치 추정 출력 주기를 흉내 낸다). 진행·옆은 같은 σ를 따로 뽑는다.
    """

    def __init__(self, bias_lat=0.0, bias_head_deg=0.0, sigma_xy=0.0, sigma_head_deg=0.0, rate_hz=10.0, seed=1):
        self.b_lat, self.b_head = bias_lat, math.radians(bias_head_deg)
        self.s_xy, self.s_head, self.period = sigma_xy, math.radians(sigma_head_deg), 1.0 / rate_hz
        self.rng, self.t_next, self.n = random.Random(seed), None, (0.0, 0.0, 0.0)

    def apply(self, t, along, lat, head):
        if self.t_next is None or t >= self.t_next - 1e-9:
            g = self.rng.gauss
            self.n = (g(0.0, self.s_xy) if self.s_xy else 0.0, g(0.0, self.s_xy) if self.s_xy else 0.0,
                      g(0.0, self.s_head) if self.s_head else 0.0)
            self.t_next = t + self.period if self.t_next is None else self.t_next + self.period
        return along + self.n[0], lat + self.b_lat + self.n[1], head + self.b_head + self.n[2]


def _with_align(phases, total, err, wheels_left, wheels_right, stow):
    """정상화(track) 뒤 마지막 정지 앞에 헤딩 보정 구간을 넣는다(22장 D1)."""
    def spin(direction):
        return dict(stow, **{n: -direction * ALIGN_SPIN for n in wheels_left}, **{n: direction * ALIGN_SPIN for n in wheels_right})
    extra = [("settle", dict(stow), "t", ALIGN_SETTLE_S), ("measure", dict(stow), "avg", (err, ALIGN_AVG_S)),
             ("align", spin, "align", (math.radians(ALIGN_SKIP_DEG), math.radians(ALIGN_STOP_DEG)))]
    return phases[:-1] + extra + phases[-1:], total + ALIGN_SETTLE_S + ALIGN_AVG_S + 6.0


def build(test, p, wheels_left, wheels_right, stow):
    """구간 목록 [(이름, 목표 dict, 조건, 값)]과 시뮬레이션 총 시간(s).

    p: corner('A'|'B'), side(+1 왼쪽|-1 오른쪽), radius_m — V4·V6b·V8(+ speed) / deadend('A'|'B') — V5 / start_heading_deg — V6
    좌표는 시작 프레임(t = 0의 차체 원점, 그때 진행 방향 = +x)이다.
    """
    st = (lambda tg: dict(stow, **tg))
    straight = st(wheel_targets(V, 0.0, wheels_left, wheels_right))
    if test in ("V4", "V8"):  # V8은 속도만 p["speed"]로 바꾼다
        v = p["speed"] if test == "V8" else V
        cx, ey, R, s = CELL[p["corner"]], EXIT_Y[p["corner"]], p["radius_m"], p["side"]
        fwd = st(wheel_targets(v, 0.0, wheels_left, wheels_right))
        arc = st(wheel_targets(v, s * v / R, wheels_left, wheels_right))
        arc_nom = (math.pi / 2) * R / v
        phases = [("stow", dict(stow), "t", 2.0), ("approach", fwd, "t", (cx - R) / v),
                  ("arc", arc, "yaw", s * math.pi / 2), ("exit", fwd, "t", (ey - R) / v), ("stop", dict(stow), "t", 1.0)]
        return phases, 2.0 + (cx - R) / v + 2.5 * arc_nom + (ey - R) / v + 1.0
    if test in ("V6c", "V7c", "V6cb"):  # 22장: V6·V7·V6b에 정지 후 헤딩 보정을 더한다
        phases, total = build({"V6c": "V6", "V7c": "V7", "V6cb": "V6b"}[test], p, wheels_left, wheels_right, stow)
        err = next(ph[3][0] for ph in phases if ph[2] == "track")
        return _with_align(phases, total, err, wheels_left, wheels_right, stow)
    if test in ("V6", "V7"):  # 출발 칸: 열린 쪽(칸 프레임 +x)이 시작 프레임에서 −θ0 방향이다. V7은 위치 오차만 더한다
        th0 = math.radians(p["start_heading_deg"])
        turn = {90: -math.pi / 2, -90: math.pi / 2, 180: math.pi}[int(p["start_heading_deg"])]
        s = 1 if turn > 0 else -1
        spin = dict(stow, **{n: -s * SPIN for n in wheels_left}, **{n: s * SPIN for n in wheels_right})
        goal = CELL["A"]
        tg, err = tracker((0.0, 0.0), -th0, wheels_left, wheels_right, stow)
        phases = [("stow", dict(stow), "t", 2.0), ("spin", spin, "yaw", turn),
                  ("normalize", tg, "track", (err, goal)), ("stop", dict(stow), "t", 1.0)]
        return phases, 2.0 + 1.5 * abs(turn) / 0.2 + 1.5 * goal / V + 1.0
    if test == "V6b":  # V4와 같은 접근·호 회전 뒤, 나가는 통로 중심선 추종으로 나가는 칸 가운데까지
        cx, ey, R, s = CELL[p["corner"]], EXIT_Y[p["corner"]], p["radius_m"], p["side"]
        arc = st(wheel_targets(V, s * V / R, wheels_left, wheels_right))
        tg, err = tracker((cx, 0.0), s * math.pi / 2, wheels_left, wheels_right, stow)
        phases = [("stow", dict(stow), "t", 2.0), ("approach", straight, "t", (cx - R) / V),
                  ("arc", arc, "yaw", s * math.pi / 2), ("normalize", tg, "track", (err, ey)), ("stop", dict(stow), "t", 1.0)]
        return phases, 2.0 + (cx - R) / V + 2.5 * (math.pi / 2) * R / V + 1.5 * ey / V + 1.0
    if test == "V5":
        L = CELL[p["deadend"]]
        back = st(wheel_targets(-V, 0.0, wheels_left, wheels_right))
        phases = [("stow", dict(stow), "t", 2.0), ("backward", back, "t", L / V), ("stop", dict(stow), "t", 2.0)]
        return phases, 2.0 + L / V + 2.0
    raise ValueError(test)


def describe(phases):
    """summary.json에 넣을 구간 설명([이름, 조건, 값]). 'track'·'avg'는 함수를 빼고 목표 거리·시간만 남긴다."""
    return [[ph[0], ph[2], ph[3][1] if ph[2] in ("track", "avg") else ph[3]] for ph in phases]


class Sequencer:
    """구간을 차례로 실행한다. update(t, yaw, x, y) → (이름, 목표 dict). 끝난 뒤에는 마지막 구간 목표를 유지한다.

    x, y, yaw는 월드 값을 그대로 넘긴다. 첫 호출의 자세를 시작 프레임 원점으로 삼는다('track' 구간에서 쓴다).
    """

    def __init__(self, phases, loc_error=None):
        self.phases, self.i, self.t0, self.y0 = phases, 0, None, None
        self.loc_error = loc_error                    # 'track'·'avg' 구간에서 제어기에 주는 위치의 오차(V7)
        self.avg_samples, self.align = [], {}         # 'avg' 구간의 추정 헤딩 오차, 'align' 구간 기록
        self.yaw_u, self.prev = 0.0, None
        self.origin = None                            # 시작 자세(월드 x, y, yaw)
        self.log = []                                 # [이름, 시작 s, 끝 s 또는 None]

    def update(self, t, yaw, x=0.0, y=0.0):
        if self.origin is None:
            self.origin = (x, y, yaw)
        ox, oy, oyaw = self.origin
        c, sn = math.cos(-oyaw), math.sin(-oyaw)
        xs, ys = (x - ox) * c - (y - oy) * sn, (x - ox) * sn + (y - oy) * c   # 시작 프레임
        if self.prev is not None:                    # yaw를 이어 붙인다(±π 감김 풀기). yaw_u는 시작 yaw 기준
            self.yaw_u += (yaw - self.prev + math.pi) % (2 * math.pi) - math.pi
        self.prev = yaw
        while self.i < len(self.phases):
            name, tg, kind, val = self.phases[self.i]
            if self.t0 is None:
                self.t0, self.y0 = t, self.yaw_u
                self.log.append([name, t, None])
            if kind == "t":
                done = t - self.t0 >= val - 1e-9
            elif kind == "yaw":
                done = (self.yaw_u - self.y0) * math.copysign(1, val) >= abs(val)
            elif kind == "avg":  # (오차 함수, 시간): 멈춘 채 추정 헤딩 오차를 모은다
                est = val[0](xs, ys, self.yaw_u)
                if self.loc_error is not None:
                    est = self.loc_error.apply(t, *est)
                done = t - self.t0 >= val[1] - 1e-9
                if not done:
                    self.avg_samples.append(est[2])
            elif kind == "align":  # (건너뛸 오차, 멈출 남은 각도): 평균 오차를 줄이는 방향으로 실제 yaw 변화를 재며 돈다
                if not self.align:
                    e = sum(self.avg_samples) / len(self.avg_samples) if self.avg_samples else 0.0
                    self.align = {"avg_head_err_deg": math.degrees(e), "n_samples": len(self.avg_samples),
                                  "skipped": abs(e) <= val[0], "direction": -1 if e > 0 else 1,
                                  "target_deg": math.degrees(max(0.0, abs(e) - val[1]))}
                turned = (self.yaw_u - self.y0) * self.align["direction"]
                done = self.align["skipped"] or turned >= math.radians(self.align["target_deg"])
            else:  # track: (오차 함수, 목표 진행 거리). 끝 판단과 목표 모두 제어기가 아는 위치(오차 포함)로 한다
                est = val[0](xs, ys, self.yaw_u)
                if self.loc_error is not None:
                    est = self.loc_error.apply(t, *est)
                done = est[0] >= val[1]
            if not done:
                if kind == "track":
                    return name, tg(est[1], est[2])
                if kind == "align":
                    return name, tg(self.align["direction"])
                return name, tg
            self.log[-1][2] = t
            self.i, self.t0 = self.i + 1, None
        return "hold", self.phases[-1][1]
