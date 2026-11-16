"""잰 퍽 자리로 시작 박스 집기 정하기 (사용자 규칙 2026-11-16: 박스당 퍽 2, 퍽 중심 = 박스 중심에서 3 ~ 6 cm, 무작위 자리).

퍽 등록표에 xy (지도 m, 측량이 잰 퍽 중심) 가 있으면 고정 ±40 mm 자리표 (poses.PUCK_DX) 대신 이것으로 집기 자세를 정한다.
xy 가 하나라도 없으면 쓰지 않음 (지금 동작 그대로).

규칙 (Isaac 고정 동작 집기 모의, 벌림 66.7 mm · 측정 σ 2 mm · 384 대: 지금 86 % -> 95 %, 첫 번째 집기 77 % -> 93 %):
  1. 두 퍽 면 사이 틈 >= GAP_FREE (35 mm): 지금 방식. 정해진 면 · 가까운 퍽 먼저 · 상자 퍽만 면 맞추기 회전, 원기둥은 회전 없음.
  2. 틈 < GAP_FREE 이고 둘 다 세운 퍽: 옆 퍽 (아직 박스에 남은 퍽) 이 집을 퍽 바로 뒤 (J1 -> 퍽 방향에서 NB_OK 안) 에 오는
     면 · 먼저 집을 퍽 · 회전각을 고름. 옆 퍽이 손가락 닫는 방향 옆에 있으면 첫 집기가 3 ~ 14 %, 뒤에 있으면 96 ~ 100 %.
       - 꼭: J1 거리 REACH, J1 -> 퍽 방향이 차체 정면에서 J1_YAW_MAX 안 (넘으면 모의 41 %), 회전이 그 접근의 turn_range_deg 안.
       - 상자 퍽은 면을 맞춰야 하므로 회전 = 면 맞추기 각 하나뿐 (원기둥만 회전각을 고름).
       - NB_OK 안이 되는 면이 있으면 그 면들만 남김 (경로 계획이 그중 고름). 없으면 옆 퍽 각이 가장 작은 면만.
     두 번째 퍽 (옆 퍽 없음) 은 1 과 같게 집음.
"""
from __future__ import annotations

import copy
import math
from dataclasses import dataclass, field

from .poses import CENTER_X, J1_X, puck_after_turn, turn_deg

PUCK_R = 0.015        # [m] 퍽 반지름 (원기둥 지름 30, 상자 30 x 30)
GAP_FREE = 0.035      # [m] 두 퍽 면 사이 틈이 이 이상이면 지금 방식 (모의 첫 집기 97 ~ 100 %)
NB_OK = 20.0          # [deg] 옆 퍽 방향 - (J1 -> 집을 퍽 방향) 허용
J1_YAW_MAX = 20.0     # [deg] J1 -> 퍽 방향 한계 (차체 정면 기준, 모의 20 · 25° 같음)
REACH = (0.20, 0.34)  # [m] 규칙 2 의 J1 거리 (모의 검증 범위)
TURN_STEP = 0.5       # [deg] 원기둥 회전각 후보 간격
TURN_MAX = 30.0       # [deg] 회전 한계 (mission start_box.max_turn_deg 와 같게)


@dataclass
class PickGeom:
    dist: float       # J1 -> 퍽 [m] (회전 뒤)
    fwd: float        # 회전 전 J1 앞 [m]
    lat: float        # 회전 전 옆 [m]
    turn_deg: float   # 집기 전 제자리 회전 (+ = 왼쪽)
    look: tuple       # 회전 뒤 base_link 퍽 예상 자리 (target_look)
    j1_yaw: float     # 회전 뒤 J1 -> 퍽 방향 [deg]
    nb_deg: float | None = None   # 옆 퍽 각 (옆 퍽이 있을 때)


def geom(final, wx, wy, deg, nb_xy=None):
    """접근 자세 final 에서 지도 (wx, wy) 퍽을 deg 돈 뒤 집을 때의 기하."""
    px, py = final.inv(wx, wy)
    fwd, lat = px - J1_X, py
    lx, ly = puck_after_turn(fwd, lat, deg)
    vx, vy = lx - J1_X, ly
    g = PickGeom(math.hypot(vx, vy), fwd, lat, deg, (lx, ly), math.degrees(math.atan2(vy, vx)))
    if nb_xy is not None:
        qx, qy = final.inv(*nb_xy)
        nx, ny = puck_after_turn(qx - J1_X, qy, deg)
        wx_, wy_ = nx - lx, ny - ly
        c = (vx * wx_ + vy * wy_) / max(g.dist * math.hypot(wx_, wy_), 1e-9)
        g.nb_deg = math.degrees(math.acos(max(-1.0, min(1.0, c))))
    return g


def default_turn(final, p):
    """지금 방식 회전: 상자 퍽 = 면 맞추기 (J1 을 퍽 앞뒤 줄 위로), 원기둥 = 0."""
    if p.shape != "box":
        return 0.0
    return turn_deg(final.inv(*p.xy)[1])


@dataclass
class BoxRule:
    box: int
    gap: float                    # 두 퍽 면 사이 틈 [m]
    tight: bool                   # 규칙 2 를 씀
    keys: list                    # 남긴 접근 키
    first: dict = field(default_factory=dict)   # 키 -> 먼저 집을 퍽 id
    with_nb: dict = field(default_factory=dict) # (키, 퍽 id) -> 옆 퍽이 있을 때 PickGeom
    alone: dict = field(default_factory=dict)   # (키, 퍽 id) -> 혼자일 때 PickGeom
    note: str = ""


def _turns(final, p, lohi):
    lo, hi = max(lohi[0], -TURN_MAX), min(lohi[1], TURN_MAX)
    if p.shape == "box":
        t = default_turn(final, p)
        return [t] if lo - 1e-9 <= t <= hi + 1e-9 else []
    n = int(math.floor((hi - lo) / TURN_STEP + 1e-9))
    cand = {round(lo + k * TURN_STEP, 3) for k in range(n + 1)}
    if lo <= 0.0 <= hi:
        cand.add(0.0)
    return sorted(cand, key=abs)


def box_rule(world, b, ps):
    """박스 b 의 두 퍽 ps (xy 있음) -> BoxRule"""
    A = world.approaches
    keys = list(world.box_opts[b])
    alt = list(getattr(world, "alt_opts", {}).get(b, []))   # ±X 대안: 규칙 2 에서만
    a, c = ps
    gap = math.hypot(a.xy[0] - c.xy[0], a.xy[1] - c.xy[1]) - 2 * PUCK_R
    r = BoxRule(b, gap, False, keys)
    for k in keys:
        fin = A[k].final
        for p in ps:
            r.alone[(k, p.id)] = geom(fin, *p.xy, default_turn(fin, p))
        for p, q in ((a, c), (c, a)):
            r.with_nb[(k, p.id)] = geom(fin, *p.xy, default_turn(fin, p), q.xy)
        r.first[k] = min(ps, key=lambda p: r.alone[(k, p.id)].dist).id   # 지금 방식: 가까운 퍽 먼저
    if gap >= GAP_FREE or any(p.pose != "standing" for p in ps):
        return r
    # 규칙 2: (키, 먼저 집을 퍽, 회전) 후보 중 옆 퍽 각
    for k in alt:
        fin = A[k].final
        for p in ps:
            r.alone[(k, p.id)] = geom(fin, *p.xy, default_turn(fin, p))
        for p, q in ((a, c), (c, a)):
            r.with_nb[(k, p.id)] = geom(fin, *p.xy, default_turn(fin, p), q.xy)
    best = {}
    for k in keys + alt:
        fin, lohi = A[k].final, A[k].extra.get("turn_range_deg", [-TURN_MAX, TURN_MAX])
        for p, q in ((a, c), (c, a)):
            for t in _turns(fin, p, lohi):
                g = geom(fin, *p.xy, t, q.xy)
                if not (REACH[0] <= g.dist <= REACH[1] and abs(g.j1_yaw) <= J1_YAW_MAX):
                    continue
                cost = (max(g.nb_deg, NB_OK) - NB_OK, abs(t))
                if k not in best or cost < best[k][0]:
                    best[k] = (cost, p.id, g)
    if not best:
        r.note = f"박스 {b}: 틈 {gap * 1000:.0f} mm 인데 규칙 2 후보 없음 (지금 방식)"
        return r
    m = min(v[0][0] for v in best.values())
    r.tight = True
    ok = [k for k in keys if k in best and best[k][0][0] <= m + 1e-9]
    r.keys = ok or [k for k in alt if k in best and best[k][0][0] <= m + 1e-9]   # 기본 면이 같은 값이면 기본 면만
    for k in r.keys:
        _, pid, g = best[k]
        r.first[k] = pid
        r.with_nb[(k, pid)] = g
    worst = max(best[k][2].nb_deg for k in r.keys)
    r.note = (f"박스 {b}: 틈 {gap * 1000:.0f} mm -> 면 {[A[k].extra['side'] for k in r.keys]}, "
              f"옆 퍽 각 ≤ {worst:.0f}°" + (f" (> {NB_OK:.0f}°, 실패 위험)" if m > 0 else ""))
    return r


class Measured:
    """한 문제의 시작 박스 규칙 묶음"""

    def __init__(self, world, pucks):
        by = {}
        for p in pucks:
            by.setdefault(p.box, []).append(p)
        self.rules = {b: box_rule(world, b, ps) for b, ps in by.items() if len(ps) == 2}

    def restrict(self, world):
        """규칙 2 로 면을 줄인 박스가 있으면 box_opts · hybrid 비용표 박스 후보를 바꾼 얕은 사본, 없으면 world 그대로"""
        cut = {b: r.keys for b, r in self.rules.items() if r.tight and set(r.keys) != set(world.box_opts[b])}
        if not cut:
            return world
        w = copy.copy(world)
        w.box_opts = {**world.box_opts, **cut}
        use_alt = any(k in world.alt_opts.get(b, []) for b, ks in cut.items() for k in ks)
        n = copy.copy(world.hybrid_nodes_alt if use_alt else world.hybrid_nodes)
        n.world = w
        n.box_ix = {b: [n.ix[k] for k in ks] for b, ks in w.box_opts.items()}
        n.hk_cache = {}
        w.hybrid_nodes = n
        return w

    def rank(self, key, p):
        """같은 정차 안 집기 차례 (작을수록 먼저). 규칙 2 박스만 정함, 나머지는 계획 차례 그대로 (가까운 퍽 먼저)"""
        r = self.rules.get(p.box)
        return 0 if r is None or not r.tight or r.first.get(key) == p.id else 1

    def pick_geom(self, key, p, alone):
        r = self.rules.get(p.box)
        if r is None:
            return None
        return r.alone[(key, p.id)] if alone else r.with_nb[(key, p.id)]

    def notes(self):
        return [r.note for r in self.rules.values() if r.note]
