"""261102 B안 사본(261031_1325_nav2/tools/make_goals_v25.py): 바꾼 것 = 카메라 x 를 --cam_x 로(B안 위 D435 x +12.5 mm → −0.0481), 읽는 기준 목표 · geom 은 261031_1325_nav2, 저장은 이 폴더 config/.
설계 25 목표(make_goals_v22.py 고침): 회전 여유를 '최종 자세의 회전 중심'을 축으로 계산한다.
v22 의 sweep_rc 는 base_link 가 (x, y) 에 '도착 방향' y0 으로 있을 때의 회전 중심을 축으로 돌렸지만, 목표 파일의 (x, y, yaw) 는 '최종 자세'이고
run_goals --base_offset_x 가 최종 yaw 로 회전 중심 목표를 만듦 → 두 회전 중심이 |x_icr|·|e^{i·y0} − e^{i·yaw}| 만큼 어긋남
(L1_via 0 → π/2: 0.111 m, L1 · L3 π/2 → 태그 약 0.09 m, 도착 허용 0.06 m 보다 큼). live_nav_v24 L1_via: 회전 중심이 (4.01, 0.40) 에서
돌기만 하고 목표 회전 중심 (4.08, 0.33) 까지 9.8 cm 를 못 옮겨 'Failed to make progress'. 고침: 회전 중심 = 최종 자세 ∘ (x_icr, 0), 들어올 때 · 나갈 때 모두 그 점을 축으로.
아래는 v22 설명(규칙 나머지는 같음).
설계 22 목표: 좁은 자리(L1_via · L1 · L3 · 출발 복귀)를 '회전 중심(x_icr)을 축으로 실제로 도는 구간'의 footprint 여유로 다시 고른다.
설계 8 · 9 규칙(make_goals_v8.py)은 base_link 를 축으로 돈다고 보고 계산했지만, Isaac 4륜 스키드는 base_link 뒤 0.0786 m 를 축으로 돎(설계 14)
→ live_nav_v21 에서 L1 칸 안팎에서 돌 자리가 없어 진행 없음. 바뀐 것:
  - 회전 여유 = 회전 중심 고정, base_link 가 그 둘레를 도는 꼴로 계산(sweep_rc).
  - 회전 구간 여유 문턱 0.07 → 0.025 m(--sweep_clear): 0.07 은 AMCL 때 위치 추정 최대 오차 0.066 m 기준, Cartographer(설계 19~21)는 최대 0.021 m.
    처음엔 0.04(약 2배)로 했으나 법선 ≤ 15° 안에서 L1 최대 0.028 · L3 0.036 m 뿐이라(0.04 이상은 법선 22~24°) 최대 오차 이상인 0.025 로 둠. 법선 15° 는 태그 읽기 조건이라 넓히지 않음.
  - L1_via(yaw +y 고정): 들어올 때 0 → π/2 회전. 출발 복귀(yaw π 고정): π ± 45° 회전. 둘 다 원래 자리에서 가장 가까운 것.
  - L1 · L3(태그 앞): 카메라 0.35~0.55 m · 법선 ≤ 15° · 정지 여유 ≥ 0.05 는 그대로, 들어올 때 π/2 → 태그 방향 · 나갈 때 태그 방향 → 3π/2(반시계) 구간 여유.
나머지 목표는 --base 그대로. 사용: python3 make_goals_v22.py [--base goals_seed1_v9.json] [--out goals_seed1_v22.json]"""
import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

NV = Path('/home/pa4/git/lv2_next_module/261031_1325_nav2')
HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(NV / 'nodes'))
import geom  # noqa: E402

CAM_X0, D_RANGE, D_BEST, MAX_OFF, MIN_CLEAR = -0.0606, (0.35, 0.55), 0.45, 15.0, 0.05
ap = argparse.ArgumentParser()
ap.add_argument('--base', default='goals_seed1_v9.json')
ap.add_argument('--out', default='goals_seed1_v25.json')
ap.add_argument('--x_icr', type=float, default=-0.0786)
ap.add_argument('--cam_x', type=float, default=-0.0606)   # 위 앞 카메라 base_link x(m), 261028 −0.0606 · B안 −0.0481
ap.add_argument('--sweep_clear', type=float, default=0.025)
ap.add_argument('--max_off', type=float, default=15.0, help='태그 법선과 카메라 방향 차 상한(°). 설계 23: 25(인지 세션 합성 확인 11-01: 30° 까지 읽힘, 25° 가 여유 있는 선)')
a = ap.parse_args()
w = geom.load_world()
B = w['body']
XI = a.x_icr
CAM_X = a.cam_x


def sweep_rc(x, y, yf, y0, y1, n=45):
    """최종 base_link 자세 (x, y, yf) 의 회전 중심을 고정하고 y0 → y1 로 돌 때 footprint 최소 여유"""
    rx, ry = x + XI * math.cos(yf), y + XI * math.sin(yf)
    return min(geom.clearance(rx - XI * math.cos(t), ry - XI * math.sin(t), t, B) for t in (y0 + (y1 - y0) * i / n for i in range(n + 1)))


def grid(cx, cy, r=0.15, step=0.01):
    for x in np.arange(cx - r, cx + r + 1e-9, step):
        for y in np.arange(cy - r, cy + r + 1e-9, step):
            yield float(x), float(y)


def fixed_yaw(g, ranges):
    """yaw 그대로, 원래 자리에서 가장 가까운 자리"""
    best = None
    for x, y in grid(g['x'], g['y']):
        c = geom.clearance(x, y, g['yaw'], B)
        if c < MIN_CLEAR:
            continue
        s = min(sweep_rc(x, y, g['yaw'], *r) for r in ranges)
        if s < a.sweep_clear:
            continue
        d = math.hypot(x - g['x'], y - g['y'])
        if best is None or d < best[0]:
            best = (d, dict(g, x=round(x, 4), y=round(y, 4), clearance=round(c, 4), sweep_rc=round(s, 4), moved=round(d, 4)))
    return best


def tag_goal(place, yaw_in=math.pi / 2, yaw_out=1.5 * math.pi):
    t = next(t for t in w['layout']['place_tags'] if t['place'] == place)
    tx, ty = t['center'][:2]
    nx, ny = t['normal'][:2]
    best = None
    for x, y in grid(tx + 0.4 * nx, ty + 0.4 * ny):
        r = math.hypot(tx - x, ty - y)
        d = r - CAM_X
        if not D_RANGE[0] <= d <= D_RANGE[1]:
            continue
        off = math.degrees(math.acos(max(-1.0, min(1.0, ((x - tx) * nx + (y - ty) * ny) / r))))
        if off > a.max_off:
            continue
        yaw = math.atan2(ty - y, tx - x)
        yu = yaw if yaw >= yaw_in else yaw + 2 * math.pi       # 반시계로 이어지게
        c = geom.clearance(x, y, yaw, B)
        if c < MIN_CLEAR:
            continue
        s_in, s_out = sweep_rc(x, y, yaw, yaw_in, yu), sweep_rc(x, y, yaw, yu, yaw_out if yaw_out > yu else yaw_out + 2 * math.pi)
        if min(s_in, s_out) < a.sweep_clear:
            continue
        score = abs(d - D_BEST) + off / 90 if a.max_off <= MAX_OFF else -min(s_in, s_out)   # 넓힌 각도에서는 회전 여유가 큰 쪽
        if best is None or score < best[0]:
            best = (score, dict(name=f"{place}(tag {t['id']} {t['face']}, 회전 중심 규칙)", x=round(x, 4), y=round(y, 4), yaw=round(yaw, 5),
                                camera_dist=round(d, 3), off_normal_deg=round(off, 1), clearance=round(c, 4),
                                sweep_in_rc=round(s_in, 4), sweep_out_rc=round(s_out, 4)))
    return best


goals = json.loads((NV / 'config' / a.base).read_text())['goals']
for i, g in enumerate(goals):
    if g['name'].startswith('L1_via'):
        new = fixed_yaw(g, [(0.0, math.pi / 2)])
    elif g['name'].startswith('start('):
        new = fixed_yaw(g, [(math.pi - math.pi / 4, math.pi + math.pi / 4)])
    elif g['name'].startswith(('L1(tag', 'L3(tag')):
        new = tag_goal(g['name'][:2])
    else:
        continue
    print("끝", g["name"], flush=True)
    if new is None:
        print('못 찾음', g['name'])
        sys.exit(1)
    print('old', {k: g[k] for k in ('name', 'x', 'y', 'yaw')})
    print('new', new[1])
    goals[i] = new[1]
(HERE / 'config' / a.out).write_text(json.dumps({'layout': str(geom.LAYOUT), 'rule': __doc__, 'goals': goals}, ensure_ascii=False, indent=1))
print('저장', HERE / 'config' / a.out)
