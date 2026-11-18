"""설계 4 목표: 장소 태그(L1 · L2 · L3) 목표를 '앞 카메라–태그 거리' 규칙으로 다시 고른다(범퍼 거리 규칙은 카메라가 차체 중심 0.06 m 뒤라 대리 지표로 틀림).
조건: 앞 카메라(URDF top_camera_front x −0.0606) 수평 거리 0.35~0.55 m, 면 법선과 각 ≤ 15°, 카메라가 태그를 정면으로 봄(yaw = 중심→태그 방향),
footprint 여유 ≥ 0.05 m, 제자리 회전 여유 ≥ 0.03 m. 그중 |거리 − 0.45| + 각/90 이 가장 작은 자세. 시작 상자 · 출발 목표는 goals_seed1_rot.json 그대로."""
import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE / 'nodes'))
import geom  # noqa: E402

CAM_X, D_RANGE, D_BEST, MAX_OFF, MIN_CLEAR, ROT_CLEAR = -0.0606, (0.35, 0.55), 0.45, 15.0, 0.05, 0.03
w = geom.load_world()
lay = w['layout']
R = math.hypot(max(-geom.FOOTPRINT[0], geom.FOOTPRINT[1]), geom.FOOTPRINT[2])


def tag_goal(t):
    tx, ty = t['center'][:2]
    nx, ny = t['normal'][:2]
    best = None
    for x in np.arange(tx - 1.0, tx + 1.0, 0.01):
        for y in np.arange(ty - 1.0, ty + 1.0, 0.01):
            r = math.hypot(tx - x, ty - y)
            d = r - CAM_X                                   # 카메라는 중심 뒤 0.0606 m, 태그와 한 줄
            if not D_RANGE[0] <= d <= D_RANGE[1]:
                continue
            off = math.degrees(math.acos(max(-1.0, min(1.0, ((x - tx) * nx + (y - ty) * ny) / r))))
            if off > MAX_OFF:
                continue
            rc = float(geom._point_seg_dist(x, y, w['body']).min() - R)
            if rc < ROT_CLEAR:
                continue
            yaw = math.atan2(ty - y, tx - x)
            c = geom.clearance(x, y, yaw, w['body'])
            if c < MIN_CLEAR:
                continue
            score = abs(d - D_BEST) + off / 90
            if best is None or score < best[0]:
                best = (score, dict(name=f"{t['place']}(tag {t['id']} {t['face']})", x=round(float(x), 4), y=round(float(y), 4), yaw=round(yaw, 5),
                                    camera_dist=round(d, 3), off_normal_deg=round(off, 1), clearance=round(c, 4), rot_clearance=round(rc, 4),
                                    bumper_to_face=round(float((x - tx) * nx + (y - ty) * ny - geom.FOOTPRINT[1]), 3)))
    return best[1] if best else None


base = json.loads((HERE / 'config' / 'goals_seed1_rot.json').read_text())['goals']
tags = {t['place']: tag_goal(t) for t in lay['place_tags']}
order = ['start_box_1', 'L1', 'L2', 'start_box_3', 'L3', 'start']
goals = []
for k in order:
    g = tags[k] if k in tags else next(g for g in base if g['name'] == k)
    goals.append(g)
    print(g)
(HERE / 'config' / 'goals_seed1_cam.json').write_text(json.dumps({'layout': str(geom.LAYOUT), 'rule': __doc__, 'goals': goals}, ensure_ascii=False, indent=1))
