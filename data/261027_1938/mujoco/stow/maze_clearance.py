"""미로 모퉁이·막다른 길 조각과 로봇 평면 외곽 사이 간격 계산(검증계획 19장, V4·V5).

좌표: 시험 시작 때 차체 원점(base_footprint) = (0, 0), 진행 방향 = +x(시작 자세 기준 프레임).
미로 치수(Cartlidar/sim_world/data/maze_upper_drawing.json, 도면 값): 기둥 중심 간격 x 0.734 m(720 판), y 0.394 m(380 판).
  - 'A' 방향: 720 판 사이 통로(벽 간격 0.788 m, 벽 기둥은 칸 모서리에만). 칸 길이 0.734 m
  - 'B' 방향: 380 판 사이 통로(벽 간격 0.734 m, 벽 기둥 0.394 m마다). 칸 길이 0.788 m
기둥은 0.06 × 0.06 m 사각형(16장 W_min과 같은 SDF 충돌 근사, 실물 몸통 0.036 m보다 큼), 판은 두께 0.003 m 직사각형.
로봇 외곽은 MuJoCo 충돌 형상(접은 자세 S0)의 평면 투영 볼록 껍질(오목한 곳을 채우므로 간격을 작게 잡는 쪽).
"""
import math

import numpy as np

POST, PANEL_T = 0.06, 0.003


def _rect(cx, cy, hx, hy):
    return np.array([[cx - hx, cy - hy], [cx + hx, cy - hy], [cx + hx, cy + hy], [cx - hx, cy + hy]])


def hull(pts):
    """평면 점들의 볼록 껍질(반시계, monotone chain)."""
    p = sorted(map(tuple, np.round(pts, 6)))
    if len(p) < 3:
        return np.array(p)

    def half(seq):
        h = []
        for q in seq:
            while len(h) >= 2 and (h[-1][0] - h[-2][0]) * (q[1] - h[-2][1]) - (h[-1][1] - h[-2][1]) * (q[0] - h[-2][0]) <= 0:
                h.pop()
            h.append(q)
        return h
    lo, up = half(p), half(reversed(p))
    return np.array(lo[:-1] + up[:-1])


def _seg_pt(a, b, p):
    ab = b - a
    t = np.clip(((p - a) @ ab) / (ab @ ab), 0.0, 1.0)
    return np.linalg.norm(a + np.outer(t, ab) - p, axis=1) if p.ndim == 2 else float(np.linalg.norm(a + t * ab - p))


def _overlap(P, Q):
    """두 볼록 다각형이 겹치는지(분리축 검사)."""
    for poly in (P, Q):
        e = np.roll(poly, -1, axis=0) - poly
        for n in np.stack([-e[:, 1], e[:, 0]], axis=1):
            a, b = P @ n, Q @ n
            if a.max() < b.min() or b.max() < a.min():
                return False
    return True


def poly_dist(P, Q):
    """볼록 다각형 사이 최소 거리(겹치면 0)."""
    if _overlap(P, Q):
        return 0.0
    d = np.inf
    for X, Y in ((P, Q), (Q, P)):
        Y2 = np.roll(Y, -1, axis=0)
        for a, b in zip(Y, Y2):
            d = min(d, float(np.min(_seg_pt(a, b, X))))
    return d


def piece(kind, kind_dir, side=1):
    """미로 조각의 장애물 다각형 목록과 목표점.

    kind: 'corner'(왼쪽 90° 모퉁이, side=-1이면 오른쪽) 또는 'deadend'(앞이 막힌 칸, 뒤로 한 칸 나감).
    kind_dir: 'A'(들어오는 통로가 720 판 사이) 또는 'B'(380 판 사이).
    반환: (장애물 다각형 목록, 정보 dict)
    """
    if kind_dir == "A":
        L, Wh, side_posts = 0.734, 0.394, False      # 진행 방향 칸 길이, 통로 반폭(벽선까지), 옆벽 중간 기둥 없음
        L2, W2h, exit_posts = 0.788, 0.367, True     # 나가는 통로(380 판 사이)
    else:
        L, Wh, side_posts = 0.788, 0.367, True
        L2, W2h, exit_posts = 0.734, 0.394, False
    posts, walls = set(), []

    def wall(p0, p1, mid):
        p0, p1 = np.array(p0, float), np.array(p1, float)
        n = max(1, int(round(np.linalg.norm(p1 - p0) / (0.394 if mid else max(np.linalg.norm(p1 - p0), 1e-9)))))
        for k in range(n + 1):
            posts.add(tuple(np.round(p0 + (p1 - p0) * k / n, 4)))
        walls.append((p0, p1))

    if kind == "corner":
        cx = L                                        # 모퉁이 칸 가운데 (L, 0)
        wall((-L / 2, -Wh), (cx + L / 2, -Wh), side_posts)      # 바깥쪽 옆벽(들어오는 칸 + 모퉁이 칸)
        wall((-L / 2, Wh), (L / 2, Wh), side_posts)              # 안쪽 옆벽(들어오는 칸만)
        wall((cx + L / 2, -Wh), (cx + L / 2, Wh), not side_posts)  # 모퉁이 칸 끝벽
        wall((cx - W2h, Wh), (cx - W2h, Wh + L2), exit_posts)    # 나가는 통로 안쪽 벽
        wall((cx + W2h, Wh), (cx + W2h, Wh + L2), exit_posts)    # 나가는 통로 바깥쪽 벽
        goal = (cx, Wh + L2 / 2)                                 # 나가는 칸 가운데
        info = {"corner_center": [cx, 0.0], "exit_center": list(goal), "cell_len": L, "exit_cell_len": L2,
                "half_width": Wh, "exit_half_width": W2h}
    else:
        wall((-1.5 * L, -Wh), (L / 2, -Wh), side_posts)
        wall((-1.5 * L, Wh), (L / 2, Wh), side_posts)
        wall((L / 2, -Wh), (L / 2, Wh), not side_posts)          # 막힌 끝벽
        goal = (-L, 0.0)
        info = {"exit_center": list(goal), "cell_len": L, "half_width": Wh}
    obs = [_rect(x, y, POST / 2, POST / 2) for x, y in sorted(posts)]
    for p0, p1 in walls:
        c, v = (p0 + p1) / 2, p1 - p0
        hx, hy = (abs(v[0]) / 2, PANEL_T / 2) if abs(v[0]) > abs(v[1]) else (PANEL_T / 2, abs(v[1]) / 2)
        obs.append(_rect(c[0], c[1], hx, hy))
    if side == -1:  # 오른쪽 모퉁이: y를 뒤집는다(다각형 꼭짓점 순서도 뒤집어 반시계 유지)
        obs = [o[::-1] * [1, -1] for o in obs]
        info = {k: ([v[0], -v[1]] if isinstance(v, list) else v) for k, v in info.items()}
    info["n_posts"] = len(posts)
    return obs, info


def clearance(robot_hull, x, y, yaw, obs):
    """자세 (x, y, yaw)에서 로봇 외곽과 장애물 사이 최소 거리(m)."""
    c, s = math.cos(yaw), math.sin(yaw)
    P = robot_hull @ np.array([[c, s], [-s, c]]) + [x, y]
    return min(poly_dist(P, o) for o in obs)


def ideal_corner_path(R, cx, side=1, v=0.05, dt=0.02, exit_len=None):
    """미끄럼 없는 기구학 경로: 직진 → 반경 R 호(90°) → 직진. 모퉁이 칸 가운데 x = cx에서 나가는 중심선에 붙는다."""
    poses, x, y, th = [], 0.0, 0.0, 0.0
    a = cx - R
    while x < a:
        poses.append((x, y, th))
        x += v * dt
    if R > 0:
        for k in range(int(math.ceil((math.pi / 2) * R / (v * dt))) + 1):
            ph = min(math.pi / 2, k * v * dt / R)
            poses.append((a + R * math.sin(ph), side * R * (1 - math.cos(ph)), side * ph))
    else:
        for ph in np.linspace(0, math.pi / 2, 50):
            poses.append((cx, 0.0, side * ph))
    x, y = cx, side * R
    for k in range(int((exit_len or 0.4) / (v * dt)) + 1):
        poses.append((x, y + side * k * v * dt, side * math.pi / 2))
    return poses
