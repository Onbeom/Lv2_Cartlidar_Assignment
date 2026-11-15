"""base_cmd 시험 (Isaac 없이). Isaac Lab venv 의 python 으로 그냥 실행한다: ./run.sh test (venv 에 pytest 없음)."""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import torch  # noqa: E402

from waffle_drive.base_cmd import DD_AX, MOTOR_M_PER, SKILL_VX, SKILL_WZ, BaseCmd  # noqa: E402

R, SEP, WMAX, DT = 0.033, 0.317, 10.995574, 1 / 120
CHECKS = []


def check(name):
    def wrap(f):
        CHECKS.append((name, f))
        return f
    return wrap


def cmd(n=1, mode="twist", **kw):
    return BaseCmd(n, "cpu", R, SEP, WMAX, DT, mode, **kw)


def run(b, a, steps):
    out = None
    for _ in range(steps):
        out = b.step(torch.tensor([a], dtype=torch.float32))
    return out


@check("직진: vx 가 물리 스텝마다 1.5/120 씩 올라 24 스텝 (0.2 s) 에 0.3 m/s, 그 뒤 유지")
def _():
    b = cmd()
    reached = None
    for i in range(1, 121):
        wl, wr = b.step(torch.tensor([[1.0, 0.0]]))
        assert abs(b.vx.item() - min(DD_AX * DT * i, SKILL_VX)) < 1e-5, (i, b.vx.item())
        if reached is None and b.vx.item() >= SKILL_VX - 1e-5:
            reached = i
    assert reached == 24, reached
    assert abs(wl.item() - SKILL_VX / R) < 1e-4 and abs(wr.item() - SKILL_VX / R) < 1e-4   # 9.09 rad/s, 축소 없음


@check("감속: 0.3 m/s 에서 행동 0 이면 24 스텝에 0")
def _():
    b = cmd()
    run(b, [1.0, 0.0], 60)
    for i in range(1, 25):
        b.step(torch.tensor([[0.0, 0.0]]))
        assert abs(b.vx.item() - max(SKILL_VX - DD_AX * DT * i, 0.0)) < 1e-5, (i, b.vx.item())
    assert abs(b.vx.item()) < 1e-6


@check("제자리 회전: wz 1.5 → 바퀴 ∓7.20 rad/s (한계 안), vx 0")
def _():
    wl, wr = run(cmd(), [0.0, 1.0], 5)
    w = SKILL_WZ * SEP / 2 / R
    assert abs(wl.item() + w) < 1e-4 and abs(wr.item() - w) < 1e-4, (wl.item(), wr.item())


@check("직진 + 회전 (1, 1): 바깥 바퀴 16.3 → 10.996 으로 비례 축소, 좌우 비율 (= 회전 반경) 유지")
def _():
    wl, wr = run(cmd(), [1.0, 1.0], 60)
    raw_l, raw_r = (SKILL_VX - SKILL_WZ * SEP / 2) / R, (SKILL_VX + SKILL_WZ * SEP / 2) / R
    assert raw_r > WMAX
    assert abs(max(abs(wl.item()), abs(wr.item())) - WMAX) < 1e-4, (wl.item(), wr.item())
    assert abs(wl.item() / wr.item() - raw_l / raw_r) < 1e-5


@check("범위 밖 행동 (5, -5) 는 (1, -1) 과 같음")
def _():
    a = run(cmd(), [5.0, -5.0], 40)
    b = run(cmd(), [1.0, -1.0], 40)
    assert torch.allclose(a[0], b[0]) and torch.allclose(a[1], b[1])


@check("바퀴 모드: 행동 × 바퀴 한계 그대로")
def _():
    wl, wr = run(cmd(mode="wheels"), [0.5, -1.0], 1)
    assert abs(wl.item() - 0.5 * WMAX) < 1e-4 and abs(wr.item() + WMAX) < 1e-4


@check("환경마다 따로: reset 한 환경만 vx 0, 다른 행동은 서로 섞이지 않음")
def _():
    b = cmd(2)
    for _ in range(60):
        b.step(torch.tensor([[1.0, 0.0], [-1.0, 0.0]]))
    assert abs(b.vx[0].item() - SKILL_VX) < 1e-5 and abs(b.vx[1].item() + SKILL_VX) < 1e-5
    b.reset([0])
    assert b.vx[0].item() == 0.0 and abs(b.vx[1].item() + SKILL_VX) < 1e-5


@check("잘못된 action_mode 는 거부")
def _():
    try:
        cmd(mode="velocity")
    except ValueError:
        return
    raise AssertionError("거부하지 않음")


@check("곡률 모드 (제자리 회전 금지): vx 0 이면 곡률을 끝까지 줘도 두 바퀴 0")
def _():
    b = cmd(mode="curvature", min_turn_r=0.20)
    for a in ([0.0, 1.0], [0.0, -1.0], [0.0, 0.3]):
        wl, wr = b.step(torch.tensor([a]))
        assert abs(wl.item()) < 1e-6 and abs(wr.item()) < 1e-6, (a, wl.item(), wr.item())


@check("곡률 모드: 어떤 행동이든 좌우 바퀴가 같은 방향 (한쪽만 0 은 허용) · |wz| ≤ |vx| / R")
def _():
    torch.manual_seed(0)
    b = cmd(n=4096, mode="curvature", min_turn_r=0.20)
    for _ in range(60):   # 가속 중 · 감속 중 · 방향 바뀜 모두 지나게 무작위 행동을 이어서
        wl, wr = b.step(torch.rand(4096, 2) * 2.4 - 1.2)
        assert torch.all(wl * wr >= -1e-9), "좌우가 반대로 돎 (제자리 회전 성분)"
        vx, wz = (wl + wr) * 0.5 * R, (wr - wl) * R / SEP
        assert torch.all(wz.abs() <= vx.abs() / 0.20 + 1e-5), (wz.abs() - vx.abs() / 0.20).max().item()


@check("곡률 모드: 후진 최대 vx_rev, 반경 < 좌우 간격 / 2 는 거부")
def _():
    b = cmd(mode="curvature", min_turn_r=0.20, vx_rev=0.15)
    run(b, [-1.0, 0.0], 120)
    assert abs(b.vx.item() + 0.15) < 1e-5, b.vx.item()
    try:
        cmd(mode="curvature", min_turn_r=0.15)
    except ValueError:
        return
    raise AssertionError("반경 0.15 (< 0.1585) 를 거부하지 않음")


# ── 규칙 v2 (2026-11-14 사용자 승인) ──
V2 = dict(spin_max=math.radians(90), spin_wz=0.5, spin_v=0.03, reset_dist=0.15, rev_max=0.40, turn_motor_max=3.0)


def v2(n=1, **kw):
    return cmd(n, mode="curvature", min_turn_r=0.20, vx_rev=0.15, **{**V2, **kw})


def twist_of(wl, wr):
    """바퀴 출력 → (vx, wz) — 시험은 base_cmd 안쪽 셈을 믿지 않고 출력에서 다시 셈"""
    return (wl + wr) * 0.5 * R, (wr - wl) * R / SEP


def motor(vx, wz):
    return torch.maximum((vx - wz * SEP / 2).abs(), (vx + wz * SEP / 2).abs()) / MOTOR_M_PER


class Recount:
    """출력 (vx, wz) 만으로 규칙 v2 셈 (주행기록 HTML 판정과 같은 정의)"""

    def __init__(self, n):
        z = lambda: torch.zeros(n, dtype=torch.float64)  # noqa: E731
        self.spin, self.moved, self.rev, self.fwd = z(), z(), z(), z()
        self.spin_max, self.rev_max = z(), z()

    def add(self, vx, wz):
        vx, wz = vx.double(), wz.double()
        slow = vx.abs() < V2["spin_v"]
        self.spin = torch.where(slow, self.spin + wz.abs() * DT, self.spin)
        self.moved = torch.where(slow & (wz.abs() > 0.05), torch.zeros_like(self.moved), self.moved + vx.abs() * DT)
        self.spin_max = torch.maximum(self.spin_max, self.spin)
        self.spin = torch.where(self.moved >= V2["reset_dist"], torch.zeros_like(self.spin), self.spin)
        back = vx < 0
        self.rev = torch.where(back, self.rev - vx * DT, self.rev)
        self.fwd = torch.where(back, torch.zeros_like(self.fwd), self.fwd + vx.clamp(min=0) * DT)
        self.rev_max = torch.maximum(self.rev_max, self.rev)
        self.rev = torch.where(self.fwd >= V2["reset_dist"], torch.zeros_like(self.rev), self.rev)


def drive(b, a, steps, rc=None):
    vx = wz = None
    for _ in range(steps):
        vx, wz = twist_of(*b.step(torch.tensor([a], dtype=torch.float32)))
        if rc:
            rc.add(vx, wz)
    return vx, wz


@check("규칙 v2 제자리 회전: 멈춘 채 a1 = 1 → 0.5 rad/s, 10 s 계속 눌러도 90° 에서 멈춤 (180° · 360° 못 돎)")
def _():
    b, rc = v2(), Recount(1)
    total, wmax = 0.0, 0.0
    for _ in range(1200):
        vx, wz = drive(b, [0.0, 1.0], 1, rc)
        total += wz.item() * DT
        wmax = max(wmax, abs(wz.item()))
    assert abs(wmax - 0.5) < 1e-4, wmax
    assert math.radians(89.5) <= total <= math.radians(90.0) + 1e-4, math.degrees(total)
    assert rc.spin_max.item() <= math.radians(90.0) + 1e-4


@check("규칙 v2 제자리 회전 예산: 90° 뒤 0.10 m 움직여도 안 풀림 · 0.15 m 넘게 움직이면 다시 90° 까지")
def _():
    b = v2()
    drive(b, [0.0, 1.0], 600)                       # 90° 다 씀

    def move_then_spin(dist):
        moved = 0.0
        while moved < dist:
            vx, _ = drive(b, [0.5, 0.0], 1)
            moved += vx.item() * DT
        for _ in range(60):                         # 멈춤 (감속 중에 움직인 거리도 셈)
            drive(b, [0.0, 0.0], 1)
        ang = 0.0
        for _ in range(600):
            _, wz = drive(b, [0.0, 1.0], 1)
            ang += wz.item() * DT
        return ang
    a1 = move_then_spin(0.08)                        # 0.08 + 감속 ≈ 0.09 m < 0.15
    assert abs(a1) < 1e-6, math.degrees(a1)
    a2 = move_then_spin(0.15)
    assert math.radians(89.5) <= a2 <= math.radians(90.0) + 1e-4, math.degrees(a2)


@check("규칙 v2 후진: 계속 후진해도 이어진 후진 ≤ 0.40 m · 0.10 m 전진으론 안 풀림 · 0.15 m 넘게 전진하면 다시")
def _():
    b, rc = v2(), Recount(1)
    back = 0.0
    for _ in range(1200):                            # 10 s 후진 명령
        vx, _ = drive(b, [-1.0, 0.0], 1, rc)
        back -= min(vx.item(), 0.0) * DT
    assert 0.38 <= back <= 0.40 + 1e-4, back
    assert abs(b.vx.item()) < 1e-6, b.vx.item()

    def fwd_then_back(dist):
        moved = 0.0
        while moved < dist:
            vx, _ = drive(b, [0.5, 0.0], 1, rc)
            moved += vx.item() * DT
        bk = 0.0
        for _ in range(600):
            vx, _ = drive(b, [-1.0, 0.0], 1, rc)
            bk -= min(vx.item(), 0.0) * DT
        return bk
    assert fwd_then_back(0.06) < 0.02            # 0.06 + 감속 < 0.15 → 거의 못 감 (전진 감속 끝 무렵 몇 mm)
    assert rc.rev_max.item() <= 0.40 + 1e-4, rc.rev_max.item()
    assert 0.38 <= fwd_then_back(0.16) <= 0.40 + 1e-4


@check("규칙 v2 꺾는 중 모터: 반경 0.20 m 로 꺾으면 가장 빠른 모터 ≤ 3.0 · 반경 그대로, 직진 0.3 m/s 는 그대로")
def _():
    vx, wz = drive(v2(), [1.0, 1.0], 240)
    assert motor(vx, wz).item() <= 3.0 + 1e-4, motor(vx, wz).item()
    assert abs(motor(vx, wz).item() - 3.0) < 1e-3                   # 상한까지는 냄
    assert abs(vx.item() / wz.item() - 0.20) < 1e-4, vx.item() / wz.item()
    vx, wz = drive(v2(), [1.0, 0.0], 120)
    assert abs(vx.item() - SKILL_VX) < 1e-5 and abs(wz.item()) < 1e-9
    vx, wz = drive(v2(), [1.0, 0.04], 120)                         # 반경 5 m (곡률 0.2 < 0.5): 안 줄임
    assert abs(vx.item() - SKILL_VX) < 1e-5


@check("규칙 v2 무작위 행동 4096 대 × 10 s: 반경 · 제자리 회전 합 · 속도 · 후진 · 모터 상한 모두 지킴 (출력으로 다시 셈)")
def _():
    torch.manual_seed(1)
    n = 4096
    b, rc = v2(n), Recount(n)
    a = torch.zeros(n, 2)
    for k in range(1200):
        if k % 30 == 0:   # 0.25 s 마다 새 행동, 1/3 은 멈춘 채 돌기 (a0 = 0)
            a = torch.rand(n, 2) * 2.4 - 1.2
            a[torch.rand(n) < 0.33, 0] = 0.0
        vx, wz = twist_of(*b.step(a))
        rc.add(vx, wz)
        moving = vx.abs() >= V2["spin_v"]
        assert torch.all(~moving | (wz.abs() <= vx.abs() / 0.20 + 1e-4)), "움직이며 반경 < 0.20"
        assert torch.all(wz.abs() <= torch.maximum(vx.abs() / 0.20, torch.tensor(0.5)) + 1e-4), "제자리 회전 > 0.5 rad/s"
        kap = wz.abs() / vx.abs().clamp(min=1e-6)
        allow = torch.where(moving & (kap >= 1.0), torch.tensor(3.0), torch.tensor(99.0))
        allow = torch.where(~moving, torch.tensor(3.0), allow)
        assert torch.all(motor(vx, wz) <= allow + 1e-3), "꺾는 중 모터 > 3.0"
    assert rc.spin_max.max().item() <= math.radians(90.0) + 1e-3, math.degrees(rc.spin_max.max().item())
    assert rc.rev_max.max().item() <= 0.40 + 1e-3, rc.rev_max.max().item()
    assert rc.spin_max.max().item() > math.radians(60), "제자리 회전이 거의 안 나옴 (시험이 약함)"
    assert rc.rev_max.max().item() > 0.30, "후진이 거의 안 나옴 (시험이 약함)"


@check("규칙 v2 관측: 남은 회전 · 남은 후진 비율이 쓴 만큼 줄고 reset 하면 1")
def _():
    b = v2()
    assert torch.allclose(b.limits_obs(), torch.tensor([[1.0, 1.0]]))
    drive(b, [0.0, 1.0], 120)                                      # 1 s × 0.5 rad/s ≈ 28.6° (가속 없음)
    s = b.limits_obs()[0, 0].item()
    assert abs(s - (1 - 0.5 / math.radians(90))) < 0.02, s
    b.reset([0])
    assert torch.allclose(b.limits_obs(), torch.tensor([[1.0, 1.0]]))


@check("규칙 v2 한계는 curvature 모드에서만 (twist 에 주면 거부)")
def _():
    try:
        cmd(mode="twist", spin_max=1.0)
    except ValueError:
        return
    raise AssertionError("거부하지 않음")


if __name__ == "__main__":
    failed = 0
    for name, f in CHECKS:
        try:
            f()
            print(f"통과  {name}")
        except AssertionError as e:
            failed += 1
            print(f"실패  {name}: {e}")
    print(f"base_cmd 시험 {len(CHECKS)} 개 중 {len(CHECKS) - failed} 개 통과")
    sys.exit(1 if failed else 0)
