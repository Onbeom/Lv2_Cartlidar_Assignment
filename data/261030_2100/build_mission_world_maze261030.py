"""발제 시험 월드 — 새 미로(maze.zip, 사용자 2026-10-30) 판. 인지 파트 시험용.

사용자 결정(2026-10-30): 이번 maze.zip 속 미로를 기준으로 진행. 바뀐 것은 벽뿐이고 시작 박스 · 바코드 · 신호등 · 출발 위치는
발제 좌표(왼쪽 아래 기둥 중심 원점) 그대로.
  - 벽: maze_src/maze/maze.usda 를 그대로 참조하고 (DX, DY, 0) 만큼 옮김. 발제 = USD + (2.2345, 3.2136), 회전 0
    (YOLO 1030 1 측정값, 261030_1943/README.md '발제 원점 변환').
  - 물체 · 신호 · 퍽 배치 · 조명: Cartlidar/sim_world/build_mission_world.py 의 plan · write_usd 를 그대로 씀(같은 seed 면 같은 물체).
    바코드 벽 면 x 만 새 왼쪽 벽 안쪽 면에서 잼.
  - 로봇: 새 모델 261029_1030/urdf_261028(사용자 2026-10-30, YOLO 1030 1 SLAM 녹화와 같은 모델). Isaac 가벼운 판 USD 를 참조하고
    루트 base_link 를 지면 위 0.00675 m(sensors.json root_height_m), 팔은 모델 기본 자세 drive(robot_params.json, 사용자 결정 2026-10-29).
    생성된 모델 파일은 읽기만 한다(그 폴더 AGENTS.md).
  - 중앙 받침대: 발제 '중앙은 410×410×70 박스 4개', 'L2 격자 중심=(2202,3700)', '820×820 받침대의 북쪽을 L2로' →
    북쪽 박스 줄 중심 y = 3700 → 받침대 중심 (2202, 3495). 이전 월드는 그림에서 읽은 3500 한 장(820)이었음.
    실물은 410 박스 4개일 수도, 820 판 한 장일 수도 있음(사용자 2026-10-30) → seed 마다 둘 중 하나(확률 0.5, 별도 난수라 다른 물체는 그대로).
  - 장소 태그(L1 · L2 · L3, ID 12 · 13 · 14): 실물에서 어느 면에 붙을지 아직 모름(사용자 2026-10-30) → seed · 장소마다
    윗면 · 옆면 4곳 중 하나(균등, 별도 난수 [seed, 13]). 옆면은 발제대로 중심 높이 35 mm(L2 북쪽 면은 x 2202),
    윗면은 L1 · L3 박스 중심, L2 는 L2 격자 중심 (2202, 3700), 윗면 태그 회전은 무작위.
  - 발제에 없는 값은 도메인 랜덤화로 본다(사용자 2026-10-30): 시작 박스 yaw U(−30°, 30°), 직육면체 누움 확률 0.5, 퍽 yaw · 뒤집힘,
    바코드 y · 높이, 조명 세기(build_mission_world.plan 그대로).
  - 벽 확인용 도면: 새 미로 z = 0.2 m 단면(261030_1943/maze_slice_z0.20.json)의 선분을 얇은 판으로 봄(렌더 도구 --drawing 호환).
원본 build_mission_world.py · 이전 worlds/dice_mission · data/maze_dice_mission.json 은 바꾸지 않는다.

실행 (PC): ~/isaacsim-venv/bin/python -I build_mission_world_maze261030.py --seed 1
결과: Cartlidar/sim_world/worlds/dice_mission_maze261030/seed_<seed>/{task_world.usda, layout.json, wall_check.json},
      Cartlidar/sim_world/data/maze_dice_mission_maze261030.json(벽 단면 도면)
"""
import argparse
import json
import math
import os
import sys
from pathlib import Path

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux

TOP = Path('/home/pa4/git/lv2_next_module')
SIM = TOP / 'Cartlidar/sim_world'
sys.path.insert(0, str(SIM))
import build_mission_world as bmw  # noqa: E402

MAZE_USD = TOP / '261030_1943/maze_src/maze/maze.usda'
SLICE = TOP / '261030_1943/maze_slice_z0.20.json'
DX, DY = 2.2345, 3.2136
DRAWING = SIM / 'data/maze_dice_mission_maze261030.json'
WALL_H = 0.39   # maze.zip 벽 높이 (261030_1943/README.md)
MODEL_DIR = TOP / '261029_1030/urdf_261028'
ROBOT_USD = MODEL_DIR / 'isaac/usd/light/waffle_4wd_light/waffle_4wd_light.usda'   # Physics 변형 기본 'physics'
URDF_PKG = MODEL_DIR / 'waffle_4wd_description_261028'
ROOT_HEIGHT = json.loads((MODEL_DIR / 'isaac/urdf/sensors.json').read_text())['root_height_m']
DRIVE = json.loads((MODEL_DIR / 'isaac/robot_params.json').read_text())['poses']['drive']['value']


class RaisedRobot(bmw.Robot):
    """루트(base_link)를 지면 위 ROOT_HEIGHT 에 둔 형상 (bmw.check 는 로봇을 z 0 에 둠)"""

    def world_shapes(self, q, T_world_root=None):
        T = np.eye(4) if T_world_root is None else np.array(T_world_root, float)
        T = T.copy()
        T[2, 3] += ROOT_HEIGHT
        return super().world_shapes(q, T)


def wall_segments():
    d = json.loads(SLICE.read_text())
    return [(g, np.array([a[0] + DX, a[1] + DY]), np.array([b[0] + DX, b[1] + DY])) for g, a, b in d['segments']]


def make_drawing(segs):
    """단면 선분 → 판(렌더 도구 꼴). 선분은 벽 두께의 양쪽 면이라 한 벽이 판 두 장으로 나온다. 기둥은 없음."""
    panels = []
    for g, a, b in segs:
        v = b - a
        L = float(np.linalg.norm(v))
        if L < 1e-6:
            continue
        panels.append({'id': len(panels), 'kind': g, 'axis': 'x' if abs(v[0]) >= abs(v[1]) else 'y', 'length': bmw.r6(L),
                       'center': [bmw.r6((a[0] + b[0]) / 2), bmw.r6((a[1] + b[1]) / 2)], 'a': -1, 'b': -1, 'shared': False})
    xs = [p[0] for _, a, b in segs for p in (a, b)]
    ys = [p[1] for _, a, b in segs for p in (a, b)]
    out = {'description': '새 maze.zip(2026-10-30) z = 0.2 m 단면 선분을 판으로 본 것(벽 양쪽 면). 단위 m, 발제 좌표(왼쪽 아래 기둥 중심 원점).',
           'sources': {'maze_usd': str(MAZE_USD), 'slice': str(SLICE), 'slice_sha256': bmw.sha256(SLICE),
                       'usd_to_mission': [DX, DY, 0.0]},
           'grid': {'x': [bmw.r6(min(xs)), bmw.r6(max(xs))], 'y': [bmw.r6(min(ys)), bmw.r6(max(ys))]},
           'posts': [], 'panels': panels, 'wall_height': WALL_H}
    DRAWING.write_text(json.dumps(out, indent=1, ensure_ascii=False))
    return out


def seg_dist(p, a, b):
    v = b - a
    t = np.clip(np.dot(p - a, v) / max(np.dot(v, v), 1e-12), 0, 1)
    return float(np.linalg.norm(p - (a + t * v)))


def rect_wall_clearance(center, size, yaw, segs, n=40):
    """돌린 사각형 둘레와 벽 선분 사이 최소 거리. 벽 선분 끝점이 사각형 안이면 음수(겹침)."""
    c = np.array(center[:2])
    R = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
    hx, hy = size[0] / 2, size[1] / 2
    corners = [c + R @ np.array(q) for q in ((-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy))]
    pts = [corners[i] + (corners[(i + 1) % 4] - corners[i]) * k / n for i in range(4) for k in range(n)]
    best = min(seg_dist(p, a, b) for p in pts for _, a, b in segs)
    for _, a, b in segs:
        for e in (a, b):
            q = R.T @ (e - c)
            if abs(q[0]) < hx and abs(q[1]) < hy:
                return -1.0
    return best


def left_face_x(segs, y0, y1):
    """왼쪽 바깥 벽의 안쪽 면 x (y0~y1 사이 선분 중 가장 큰 x)."""
    xs = [max(a[0], b[0]) for _, a, b in segs if max(a[0], b[0]) < 0.15 and min(a[1], b[1]) <= y1 and max(a[1], b[1]) >= y0]
    if not xs:
        raise RuntimeError('왼쪽 벽 면을 못 찾음')
    return max(xs)


def split_center_stand(p, n=2):
    """plan() 의 820 받침대 한 장 → 발제대로 410 박스 4개(맞붙임, yaw 0). L2 태그는 plan() 이 받침대 북쪽 면에 이미 둠"""
    st = next(b for b in p['boxes'] if b['name'] == 'center_stand')
    cx, cy, cz = st['center']
    w = st['size'][0] / n
    quads = [{'name': f'center_stand_{i * n + j + 1}', 'kind': 'stand', 'center': [bmw.r6(cx + (j - (n - 1) / 2) * w),
              bmw.r6(cy + (i - (n - 1) / 2) * w), cz], 'size': [bmw.r6(w), bmw.r6(w), st['size'][2]], 'yaw': 0.0}
             for i in range(n) for j in range(n)]
    p['boxes'] = [b for b in p['boxes'] if b['name'] != 'center_stand'] + quads
    p['shelves'] = [s for s in p['shelves'] if s['name'] != 'center_stand'] + [
        {'name': q['name'], 'center': q['center'], 'size': q['size']} for q in quads]
    for t in p['place_tags']:
        if t['box'] == 'center_stand':
            t['box'] = 'center_stand_3'   # 북서쪽 박스 북쪽 면(태그 x 2202 = 북쪽 두 박스 맞닿은 곳)


PLACE_FACES = ('top', '+x', '-x', '+y', '-y')
# 옆면 앞 벽까지 이보다 좁으면 태그 후보에서 뺌: 로봇(좌우 바퀴 간격 0.317 m)이 그 면 앞에 설 수 없다.
# 렌더 확인 카메라 거리(0.45 m)와 같게 둠. 새 미로에서 좁은 면은 L3 −x 0.174 · L1 +y 0.223, 다음은 0.704 m 라 0.23~0.70 어느 값이든 결과 같음
MIN_FACE_CLEAR = 0.45
# 조명 색온도 [K] 무작위(돔 · 천장 모두 같은 값). 교안(yolov8 randomize_mission) 4000~6500 에 실내 전구색 LED 3000 을 더함.
# 시연장 조명 종류를 모르므로 넣는다. 세기(light_level)와 따로 뽑는다
COLOR_TEMP_RANGE = (3000.0, 6500.0)


def face_clearance(pt, nrm, segs, maxd=3.0):
    """옆면 가운데 pt 에서 법선 nrm 방향으로 가장 가까운 벽 선분까지 거리 [m] (없으면 maxd)"""
    q, best = pt + nrm * maxd, maxd
    for _, a, b in segs:
        r, s_ = q - pt, b - a
        den = r[0] * s_[1] - r[1] * s_[0]
        if abs(den) < 1e-12:
            continue
        t = ((a - pt)[0] * s_[1] - (a - pt)[1] * s_[0]) / den
        u = ((a - pt)[0] * r[1] - (a - pt)[1] * r[0]) / den
        if 0 <= t <= 1 and 0 <= u <= 1:
            best = min(best, t * maxd)
    return best


def randomize_place_tags(p, rng, segs):
    """장소 태그 면 · 자리 무작위(plan() 이 둔 접근 면 태그를 바꿈). 옆면 법선은 박스 바깥, 윗면은 +z.
    앞 벽까지 MIN_FACE_CLEAR 보다 좁은 옆면은 후보에서 뺀다"""
    out = []
    for t in p['place_tags']:
        L = t['place']
        if L == 'L2':
            stand = [b for b in p['boxes'] if b['kind'] == 'stand']
            xs = [b['center'][0] + s_ * b['size'][0] / 2 for b in stand for s_ in (-1, 1)]
            ys = [b['center'][1] + s_ * b['size'][1] / 2 for b in stand for s_ in (-1, 1)]
            c, half = np.array([(min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2]), np.array([(max(xs) - min(xs)) / 2, (max(ys) - min(ys)) / 2])
            top_xy = np.array(bmw.A['L']['L2'])
            top_z = stand[0]['size'][2]
        else:
            b = next(x for x in p['boxes'] if x['name'] == f'place_{L}')
            c, half = np.array(b['center'][:2]), np.array(b['size'][:2]) / 2
            top_xy, top_z = c, b['size'][2]
        clear = {}
        for f in PLACE_FACES[1:]:
            ax, sg = (0 if f[1] == 'x' else 1), (1.0 if f[0] == '+' else -1.0)
            n2 = np.zeros(2)
            n2[ax] = sg
            pt = c.copy()
            pt[ax] += sg * half[ax]
            clear[f] = bmw.r6(face_clearance(pt + n2 * 1e-4, n2, segs))
        faces = ['top'] + [f for f in PLACE_FACES[1:] if clear[f] >= MIN_FACE_CLEAR]
        face = faces[int(rng.integers(len(faces)))]
        if face == 'top':
            a = float(rng.uniform(-math.pi, math.pi))
            nrm, center = np.array([0.0, 0.0, 1.0]), np.array([top_xy[0], top_xy[1], top_z + 0.0005])
            u = [bmw.r6(math.cos(a)), bmw.r6(math.sin(a)), 0.0]
        else:
            ax, sg = (0 if face[1] == 'x' else 1), (1.0 if face[0] == '+' else -1.0)
            nrm = np.zeros(3)
            nrm[ax] = sg
            xy = c.copy()
            xy[ax] += sg * half[ax]
            center = np.array([xy[0], xy[1], bmw.A['place_tag_z']]) + nrm * 0.0005
            u = None
        q = {**t, 'face': face, 'normal': nrm.tolist(), 'center': [bmw.r6(v) for v in center],
             'face_clearance_m': clear, 'face_candidates': faces}
        if u is not None:
            q['u'] = u
        out.append(q)
    p['place_tags'] = out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seed', type=int, required=True)
    ap.add_argument('--light-level', type=float, default=None)
    ap.add_argument('--color-temp', type=float, default=None)   # [K] 고정할 때만
    ap.add_argument('--barcode', choices=bmw.A['barcodes'], default=None)   # 바코드 문자열 고정(판정에서 4종을 모두 넣을 때)
    ap.add_argument('--stand', choices=['box410x4', 'board820x1'], default=None)   # 받침대 종류 고정(판정에서 둘 다 넣을 때)
    ap.add_argument('--world-set', default='dice_mission_maze261030')     # worlds/<이 이름>/<--name>
    ap.add_argument('--name', default=None)                               # 기본 seed_<seed>
    args = ap.parse_args()
    out = SIM / f"worlds/{args.world_set}/{args.name or f'seed_{args.seed}'}"
    out.mkdir(parents=True, exist_ok=True)
    segs = wall_segments()
    drawing = make_drawing(segs)
    # 바코드 벽 면: 새 왼쪽 벽 안쪽 면(사용자 범위 y 0.55~0.75 전체에서 가장 안쪽 x)
    bw = bmw.A['barcode_wall']
    x_face = left_face_x(segs, bw['y'][0], bw['y'][1])
    bmw.A['barcode_wall'] = {**bw, 'x_face': bmw.r6(x_face), 'z': [bw['z'][0], min(bw['z'][1], WALL_H)]}
    bmw.DRAWING = DRAWING   # plan() 이 바코드 자리에서 기둥을 피할 때 읽음(새 도면은 기둥 없음)
    for t in bmw.A['barcodes']:
        bmw.barcode_png(t, bmw.HERE / f'textures/barcode648_{t}.png')
    bmw.ROBOT_USD, bmw.URDF_PKG = ROBOT_USD, URDF_PKG
    bmw.A['center_stand'] = {**bmw.A['center_stand'], 'center': [2.202, 3.495]}
    p = bmw.plan(args.seed)
    if args.barcode:   # 문자열만 바꿈(자리 · 다른 무작위 값은 그대로)
        p['barcodes'][0]['text'] = args.barcode
    stand_kind = 'box410x4' if np.random.default_rng([args.seed, 11]).random() < 0.5 else 'board820x1'
    stand_kind = args.stand or stand_kind
    if stand_kind == 'box410x4':
        split_center_stand(p)
    randomize_place_tags(p, np.random.default_rng([args.seed, 13]), segs)
    p['robot']['q'] = dict(DRIVE)
    p['robot']['model'] = '261029_1030/urdf_261028 (Isaac light)'
    p['robot']['z'] = ROOT_HEIGHT
    robot = RaisedRobot(URDF_PKG / 'urdf/waffle_4wd.urdf', URDF_PKG)
    errs, near = bmw.check(p, drawing, robot)   # 퍽 · 로봇 출발 자세 (벽 = 단면 선분 판)
    # 새 벽과 물체: 박스 · 받침대 둘레, 신호등 받침, 출발 테이프
    wall = {b['name']: bmw.r6(rect_wall_clearance(b['center'], b['size'], b['yaw'], segs)) for b in p['boxes']}
    lt = p['signal_light']
    r_sig = bmw.A['signal_light']['base_diameter'] / 2
    wall['signal_light'] = bmw.r6(min(seg_dist(np.array([lt['x'], lt['y']]), a, b) for _, a, b in segs) - r_sig)
    sb = p['start_box']
    wall['start_tape'] = bmw.r6(rect_wall_clearance(sb['center'], (sb['outer'], sb['outer']), 0.0, segs))
    bc = p['barcodes'][0]
    errs += [f'{k} 벽과 겹침({v} m)' for k, v in wall.items() if v < 0]
    if bc['center'][2] + bc['size'][1] / 2 > WALL_H:
        errs.append('바코드가 벽 위로 나감')
    (out / 'wall_check.json').write_text(json.dumps({'wall_clearance_m': wall, 'barcode_x_face': bmw.r6(x_face),
                                                     'robot_min_clearance_m': near, 'errors': errs}, indent=1, ensure_ascii=False))
    if errs:
        raise SystemExit('확인 실패: ' + '; '.join(errs))
    level = args.light_level
    if level is None:
        level = float(np.random.default_rng([args.seed, 7]).uniform(0, 1))
    level = min(max(level, 0.0), 1.0)
    temp = args.color_temp
    if temp is None:
        temp = float(np.random.default_rng([args.seed, 17]).uniform(*COLOR_TEMP_RANGE))
    p.update({'seed': args.seed, 'light_level': bmw.r6(level), 'color_temp_k': round(temp, 1)})
    usd = out / 'task_world.usda'
    if usd.exists():
        usd.unlink()
    bmw.write_usd(usd, {'panels': [], 'posts': []}, p, level)   # 벽은 아래에서 maze.usda 참조로
    st = Usd.Stage.Open(str(usd))
    mz = st.DefinePrim('/World/Maze', 'Xform')   # 옮김만 여기에. maze.usda 의 /maze 는 자기 xformOp 가 있어 자식으로 참조
    UsdGeom.Xformable(mz).AddTranslateOp().Set(Gf.Vec3d(DX, DY, 0.0))
    st.DefinePrim('/World/Maze/maze', 'Xform').GetReferences().AddReference(os.path.relpath(MAZE_USD, out))   # defaultPrim /maze
    mz.CreateAttribute('maze:source', Sdf.ValueTypeNames.String).Set('maze.zip 2026-10-30, 발제 = USD + (2.2345, 3.2136)')
    for lp in st.GetPrimAtPath('/World/Lights').GetChildren():   # 색온도: 돔 · 천장 사각 모두
        lux = UsdLux.LightAPI(lp)
        lux.CreateEnableColorTemperatureAttr(True)
        lux.CreateColorTemperatureAttr(round(temp, 1))
    for t in p['place_tags']:   # 윗면 태그: write_usd 는 옆면(법선 수평)만 그리므로 사각형을 윗면 꼴로 다시 둠
        if t['face'] != 'top':
            continue
        half = t['size'] * 10 / 8 / 2
        u = np.array(t['u'])
        pts, _, _ = bmw.quad(t['center'], u, np.cross(t['normal'], u), half, half)
        m = UsdGeom.Mesh(st.GetPrimAtPath(f"/World/PlaceTags/tag_{t['id']:02d}"))
        m.GetPointsAttr().Set([Gf.Vec3f(*map(float, q)) for q in pts])
        pa = np.array(pts)
        m.GetExtentAttr().Set([Gf.Vec3f(*map(float, pa.min(0))), Gf.Vec3f(*map(float, pa.max(0)))])
    rprim = st.GetPrimAtPath('/World/waffle_4wd')   # write_usd 가 이전 모델 변형 이름(physx)으로 둔 것을 새 모델 값으로
    rprim.GetVariantSets().GetVariantSet('Physics').SetVariantSelection('physics')
    rprim.GetAttribute('xformOp:translate').Set(Gf.Vec3d(p['robot']['x'], p['robot']['y'], ROOT_HEIGHT))
    for j, v in DRIVE.items():   # 렌더용 시작 자세(관절 상태가 아니라 설정 기록). 실제 자세는 시뮬레이터가 drive 로 둠
        rprim.CreateAttribute(f'lv2:start_pose:{j}', Sdf.ValueTypeNames.Double).Set(float(v))
    st.GetRootLayer().customLayerData = {**st.GetRootLayer().customLayerData, 'maze': 'maze261030',
                                         'robot_model': '261029_1030/urdf_261028', 'robot_start_pose': 'drive'}
    st.GetRootLayer().Save()
    meta = {'seed': args.seed, 'task': 'dice_mission_v5', 'maze': 'maze261030', 'light_level': bmw.r6(level),
            'assumptions': {**bmw.A, 'shelf_size': bmw.A['box_small'], 'barcode_label': bmw.A['barcode_label'],
                            'maze_to_mission': [DX, DY, 0.0], 'wall_height': WALL_H,
                            'center_stand_source': '발제: 410 박스 4개, L2 격자 중심 (2202,3700) = 북쪽 박스 줄 중심 → 중심 (2202,3495)'},
            'center_stand_kind': stand_kind,
            'domain_randomization': ['장소 태그 면: 윗면 · 옆면 4곳 균등(장소마다), 윗면 회전 무작위', '중앙 받침대 410 박스 4개 | 820 판 1개 (0.5)', '시작 박스 yaw U(-30,30)°', '퍽 색·모양 구성(풀 수 있는 신호)', '퍽 번호 배정', '퍽 yaw U(-180,180)°',
                                     '원기둥·선 직육면체 뒤집힘 0.5', '직육면체 누움 0.5', '신호 순서', '바코드 문자열·y 0.55~0.75·높이 0.10~0.38 m',
                                     '조명 세기 0~1'],
            'sources': {'maze_usd': str(MAZE_USD), 'maze_usda_sha256': bmw.sha256(MAZE_USD), 'drawing': str(DRAWING),
                        'drawing_sha256': bmw.sha256(DRAWING), 'robot_usd': str(ROBOT_USD), 'robot_usd_sha256': bmw.sha256(ROBOT_USD),
                        'urdf_sha256': bmw.sha256(URDF_PKG / 'urdf/waffle_4wd.urdf'),
                        'robot_params_sha256': bmw.sha256(MODEL_DIR / 'isaac/robot_params.json'), 'builder': str(Path(__file__).resolve())},
            'robot_min_clearance_m': near, 'wall_clearance_m': wall,
            **{k: v for k, v in p.items() if k not in ('seed', 'light_level')}}
    (out / 'layout.json').write_text(json.dumps(meta, indent=1, ensure_ascii=False))
    print(f"seed {args.seed}: 신호 {lt['sequence']}, 바코드 {bc['text']} y {bc['center'][1]:.3f} z {bc['center'][2]:.3f} "
          f"x_face {x_face:.4f}, 받침대 {stand_kind}, 장소 태그 {[t['face'] for t in p['place_tags']]}, 벽 최소 여유 {min(wall.values()):.3f} m ({min(wall, key=wall.get)}), 로봇 최소 거리 {near:.3f} m")


if __name__ == '__main__':
    main()
