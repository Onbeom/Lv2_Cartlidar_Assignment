"""Smac Lattice 원시 동작 파일(diff 모델)을 '직선 + 제자리 회전'만으로 만든다(설계 30).
4륜 스키드 차체는 회전 중심(rotation_center)을 축으로 제자리 회전과 직진은 잘하지만, 전진하며 도는 호는 실제 회전이 명령의 53~75 % 라 바깥으로 밀림(설계 29).
설치된 예제(sample_primitives)는 5 cm 칸 · 회전 반경 0.5 m 호라 costmap 0.025 m 와 안 맞음 → 직접 만든다.
- 방향 16개: 격자 벡터 (1,0) (2,1) (1,1) (1,2) … 의 각도(예제와 같은 각도 목록) — 직선 끝점이 격자점에 떨어짐.
- 직선: 방향마다 짧은 것(벡터 1배, 0.025~0.056 m)과 긴 것(벡터 n배, 약 0.1 m). 후진은 Smac 의 allow_reverse_expansion 이 뒤집어 씀.
- 제자리 회전: 이웃 방향(±1)으로, 중간 자세를 5° 이하 간격으로 넣어 회전하며 쓸리는 자리도 footprint 검사되게.
- --no_rot_mid(설계 31): 제자리 회전을 시작 · 끝 두 자세만으로(중간 자세 없음). Nav2 경로 처리기는 누적 회전이 minimum_rotation_angle 을 넘는 자세에서 경로를 끊는데,
  중간 자세가 있으면 회전 도중(약 14°)에서 끊어 회전 자리 허용(0.3 rad) 안에 이미 들어가 회전을 건너뜀(설계 30). 회전 중 쓸림은 전역 여백 0.04 와 plan_only 의 쓸림 여유로 확인.
  끝 자세는 두 번 넣음: 경로 처리기는 회전 직전 자세에서 구간을 끝내고 다음 구간을 회전한 자세에서 시작하므로, 회전이 연달아 있으면(90° = 26.6 · 18.4 · 18.4 · 26.6°)
  자세 1개짜리 구간이 생겨 RotationShim 이 경로를 못 읽음(2개 미만이면 예외). 끝 자세가 둘이면 구간마다 자세 2개(test/probe_constraint 합성 경로로 확인).
  Smac Lattice 는 xy · yaw 차이가 1e-4 미만인 이웃 자세를 지우므로(smac_planner_lattice_impl.hpp) 둘째 끝 자세는 같은 방향으로 5e-4 rad 더 돈 자세.
- 메타데이터 turning_radius 는 해석 연장 · 휴리스틱용 값(경로 원시 동작에는 호 없음).
- --arc_r_min R(설계 44, 사용자 2026-10-24 결정 '제자리 회전 최소화 · 모퉁이는 호 회전'으로 되돌림): 방향 ±1 · ±2 칸 바꾸는 호 원시 동작을 더함.
  끝점은 격자점, 꼴은 '직선 + 호' 또는 '호 + 직선'(두 방향 선의 교점에서 접선 길이가 같은 원호), 반지름 R ~ --arc_r_max 안에서 가장 짧은 것.
  회전 중심(rotation_center) 기준 호. 전진 중 회전은 명령 0.1 rad/s 까지 안 돌고 move_ff 곡선 끝이 실제 0.58 rad/s(TOOLCHECK_turn_moving)라
  반지름 R 이면 0.12 m/s 에서 w = 0.12/R(R 0.25 → 0.48). 자세 간격 ≤ --arc_step(쓸림 검사).
사용: python3 make_lattice_straight_rot.py [--res 0.025] [--long 0.1] [--radius 0.1] [--no_rot_mid] [--arc_r_min 0.25 --arc_r_max 0.6] [--out ../config/lattice_straight_rot_0025.json]"""
import argparse
import json
import math
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent
ap = argparse.ArgumentParser()
ap.add_argument('--res', type=float, default=0.025)
ap.add_argument('--long', type=float, default=0.1, help='긴 직선 목표 길이(m), 벡터 정수배로 반올림')
ap.add_argument('--radius', type=float, default=0.1)
ap.add_argument('--rot_step_deg', type=float, default=5.0)
ap.add_argument('--no_rot_mid', action='store_true', help='제자리 회전 중간 자세 없음(설계 31)')
ap.add_argument('--arc_r_min', type=float, default=0.0, help='0 이면 호 없음(설계 30~43 그대로)')
ap.add_argument('--arc_r_max', type=float, default=0.6)
ap.add_argument('--arc_dirs', type=int, nargs='*', default=[1, 2], help='호로 바꾸는 방향 칸 수')
ap.add_argument('--arc_step', type=float, default=0.0125)
ap.add_argument('--out', default=str(HERE / 'config' / 'lattice_straight_rot_0025.json'))
a = ap.parse_args()

VEC = [(1, 0), (2, 1), (1, 1), (1, 2), (0, 1), (-1, 2), (-1, 1), (-2, 1), (-1, 0), (-2, -1), (-1, -1), (-1, -2), (0, -1), (1, -2), (1, -1), (2, -1)]
ANG = [math.atan2(y, x) % (2 * math.pi) for x, y in VEC]
N = len(VEC)


def r5(v):
    return round(v, 5)


prims = []
for i, (vx, vy) in enumerate(VEC):
    tid = 0
    unit = math.hypot(vx, vy) * a.res
    for k in sorted({1, max(1, round(a.long / unit))}):
        L = unit * k
        n = max(1, math.ceil(L / a.res))
        poses = [[r5(vx * a.res * k * j / n), r5(vy * a.res * k * j / n), r5(ANG[i])] for j in range(1, n + 1)]
        prims.append(dict(trajectory_id=tid, start_angle_index=i, end_angle_index=i, left_turn=False, trajectory_radius=0.0,
                          trajectory_length=r5(L), arc_length=0.0, straight_length=r5(L), poses=poses))
        tid += 1
    for d in (1, -1):
        j = (i + d) % N
        da = math.remainder(ANG[j] - ANG[i], 2 * math.pi)
        m = 1 if a.no_rot_mid else max(2, math.ceil(abs(math.degrees(da)) / a.rot_step_deg))
        poses = [[0.0, 0.0, r5((ANG[i] + da * s / m) % (2 * math.pi))] for s in range(0, m + 1)]
        if a.no_rot_mid:
            poses.append([0.0, 0.0, r5((ANG[i] + da + d * 5e-4) % (2 * math.pi))])
        prims.append(dict(trajectory_id=tid, start_angle_index=i, end_angle_index=j, left_turn=d > 0, trajectory_radius=0.0,
                          trajectory_length=0.0, arc_length=0.0, straight_length=0.0, poses=poses))
        tid += 1



def arc_prim(i, j):
    """방향 i → j, 끝점 격자점, 직선 + 호(또는 호 + 직선) 중 반지름 arc_r_min~arc_r_max 에서 가장 짧은 것. 없으면 None."""
    t0, t1 = ANG[i], ANG[j]
    da = math.remainder(t1 - t0, 2 * math.pi)
    u0, u1 = (math.cos(t0), math.sin(t0)), (math.cos(t1), math.sin(t1))
    cross = u0[0] * u1[1] - u0[1] * u1[0]
    best = None
    lim = math.ceil(2 * a.arc_r_max / a.res) + 2
    for gx in range(-lim, lim + 1):
        for gy in range(-lim, lim + 1):
            px, py = gx * a.res, gy * a.res
            # 교점 I = s·u0 = p − q·u1  →  s·u0 + q·u1 = p
            s0 = (px * u1[1] - py * u1[0]) / cross
            q1 = (u0[0] * py - u0[1] * px) / cross
            if s0 < -1e-9 or q1 < -1e-9:
                continue
            tt = math.tan(abs(da) / 2)
            t = min(s0, q1)
            r = t / tt
            if r < a.arc_r_min - 1e-9 or r > a.arc_r_max + 1e-9:
                continue
            first, last = s0 - t, q1 - t       # 호 앞 직선 · 호 뒤 직선
            if first > 1e-9 and last > 1e-9:
                continue
            arc = r * abs(da)
            L = first + arc + last
            if best is None or L < best[0] - 1e-9:
                best = (L, r, first, arc, last, px, py)
    if best is None:
        return None
    L, r, first, arc, last, px, py = best
    sgn = 1 if da > 0 else -1
    poses = []
    n = max(1, math.ceil(L / a.arc_step))
    for k in range(1, n + 1):
        d = L * k / n
        if d <= first:
            x, y, th = d * u0[0], d * u0[1], t0
        elif d <= first + arc:
            ph = (d - first) / r * sgn
            cx, cy = first * u0[0] - sgn * r * u0[1], first * u0[1] + sgn * r * u0[0]   # 원 중심(왼쪽 호면 왼쪽)
            th = t0 + ph
            x, y = cx + sgn * r * math.sin(th), cy - sgn * r * math.cos(th)
        else:
            e = d - first - arc
            ex, ey = px - last * u1[0], py - last * u1[1]
            x, y, th = ex + e * u1[0], ey + e * u1[1], t1
        poses.append([r5(x), r5(y), r5(th % (2 * math.pi))])
    poses[-1] = [r5(px), r5(py), r5(t1 % (2 * math.pi))]
    return dict(start_angle_index=i, end_angle_index=j, left_turn=da > 0, trajectory_radius=r5(r),
                trajectory_length=r5(L), arc_length=r5(arc), straight_length=r5(first + last), poses=poses)


if a.arc_r_min > 0:
    nxt = {}
    for pr in prims:
        nxt[pr['start_angle_index']] = max(nxt.get(pr['start_angle_index'], -1), pr['trajectory_id'])
    miss = 0
    for i in range(N):
        for k in a.arc_dirs:
            for d in (1, -1):
                pr = arc_prim(i, (i + d * k) % N)
                if pr is None:
                    miss += 1
                    continue
                nxt[i] += 1
                prims.append(dict(trajectory_id=nxt[i], **pr))
    prims.sort(key=lambda q: (q['start_angle_index'], q['trajectory_id']))
    print('호 원시 동작', sum(q['trajectory_radius'] > 0 for q in prims), '못 만든 것', miss)

out = dict(version=1.0, date_generated='2026-11-01', lattice_metadata=dict(
    motion_model='diff', turning_radius=a.radius, grid_resolution=a.res, stopping_threshold=5, num_of_headings=N,
    trajectory_distinctness_ratio=0.9, heading_angles=[r5(v) for v in ANG], number_of_trajectories=len(prims)), primitives=prims)
Path(a.out).write_text(json.dumps(out, indent=1))
print(a.out, len(prims), '원시 동작')
