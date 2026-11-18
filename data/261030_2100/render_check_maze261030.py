"""새 미로 발제 월드 렌더 확인 (도구 확인, 판정 시험 아님) — Isaac Sim 6.1 헤드리스.

보기: overview_top(위에서 전체), overview_oblique(왼쪽 아래 모서리에서 비스듬히), front_d435 · rear_d435(출발 자세 앞 · 뒤 D435 컬러),
      box1_close(박스1 위 0.25 m 비스듬히), barcode_close(바코드 앞 0.20 m), signal_close(신호등 앞 0.5 m).
D435 내부값은 이전 렌더와 같은 r12_poses.K(640 × 480), 위치는 새 모델 sensors.json(base_link 기준). 로봇은 USD 저작 자세 그대로
(관절을 움직이지 않음 — 팔은 drive 가 아닐 수 있음, 앞 · 뒤 D435 는 차체 고정이라 시점은 맞음).

실행 (PC): ~/isaacsim-venv/bin/python 261030_2100/render_check_maze261030.py --world <seed 폴더> --out <결과 폴더> --/crashreporter/url=
"""
import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

TOP = Path('/home/pa4/git/lv2_next_module')
sys.path.insert(0, str(TOP / 'Cartlidar/sim_render'))
from r12_poses import K as KD  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument('--world', type=Path, required=True)
ap.add_argument('--out', type=Path, required=True)
ap.add_argument('--subframes', type=int, default=8)
ap.add_argument('--tag-dist', type=float, default=0.45)   # 장소 태그 확인 카메라 거리(접근 면 앞 벽이 가까우면 줄임)
ap.add_argument('--only-tags', action='store_true')
ap.add_argument('--color-temp', type=float, default=None)   # [K] 월드 값 대신 이 색온도로(0 = 색온도 끔)
ap.add_argument('--views', nargs='+', default=None)          # 이 화면만
args, _ = ap.parse_known_args()
W, H = 640, 480
from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({'headless': True, 'width': W, 'height': H, 'anti_aliasing': 0, 'multi_gpu': False})
import cv2  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf, UsdGeom, UsdLux  # noqa: E402

lay = json.loads((args.world / 'layout.json').read_text())
sens = json.loads((TOP / '261029_1030/urdf_261028/isaac/urdf/sensors.json').read_text())['sensors']


def quat_R(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def T_of(R, t):
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, t
    return T


def look_at(eye, target, up=(0, 0, 1)):
    """광학 좌표(x 오른쪽 · y 아래 · z 앞) 자세"""
    eye, target = np.array(eye, float), np.array(target, float)
    z = target - eye
    z /= np.linalg.norm(z)
    x = np.cross(z, up)
    if np.linalg.norm(x) < 1e-6:
        x = np.cross(z, (0, 1, 0))
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return T_of(np.column_stack([x, y, z]), eye)


def set_camera(stage, path, T, fx, fy, cx, cy):
    cam = UsdGeom.Camera.Define(stage, path)
    ha = 20.955
    focal = fx * ha / W
    cam.CreateHorizontalApertureAttr(ha)
    cam.CreateVerticalApertureAttr(focal * H / fy)
    cam.CreateFocalLengthAttr(focal)
    cam.CreateHorizontalApertureOffsetAttr(-(cx - W / 2) / W * ha)
    cam.CreateVerticalApertureOffsetAttr((cy - H / 2) / W * ha)
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.01, 50.0))
    R, e = T[:3, :3], T[:3, 3]
    m = Gf.Matrix4d(*R[:, 0].tolist(), 0, *(-R[:, 1]).tolist(), 0, *(-R[:, 2]).tolist(), 0, *e.tolist(), 1)
    xf = UsdGeom.Xformable(cam)
    xf.ClearXformOpOrder()
    xf.AddTransformOp().Set(m)


rb = lay['robot']
T_wb = T_of(np.array([[math.cos(rb['yaw']), -math.sin(rb['yaw']), 0], [math.sin(rb['yaw']), math.cos(rb['yaw']), 0], [0, 0, 1]]),
            [rb['x'], rb['y'], rb.get('z', 0.0)])
d435 = (KD['fx'], KD['fy'], KD['cx'], KD['cy'])
wide = (W / 2 / math.tan(math.radians(35)), W / 2 / math.tan(math.radians(35)), W / 2, H / 2)   # 가로 화각 70°
b1 = next(b for b in lay['boxes'] if b['name'] == 'start_box_1')
bc = lay['barcodes'][0]
sl = lay['signal_light']
views = {
    'overview_top': (look_at((2.25, 3.20, 7.2), (2.25, 3.20, 0.0), up=(0, 1, 0)), wide),
    'overview_oblique': (look_at((-0.9, -1.2, 2.6), (2.0, 2.6, 0.0)), wide),
    'front_d435': (T_wb @ T_of(quat_R(sens['d435_front_color']['quat_xyzw']), sens['d435_front_color']['pos']), d435),
    'rear_d435': (T_wb @ T_of(quat_R(sens['d435_rear_color']['quat_xyzw']), sens['d435_rear_color']['pos']), d435),
    'box1_close': (look_at((b1['center'][0] - 0.12, b1['center'][1] - 0.18, 0.34), (b1['center'][0], b1['center'][1], 0.10)), d435),
    'barcode_close': (look_at((bc['center'][0] + 0.20, bc['center'][1], bc['center'][2]), bc['center'], up=(0, 0, 1)), d435),
    'signal_close': (look_at((sl['x'] + 0.5, sl['y'], 0.12), (sl['x'], sl['y'], 0.12)), d435),
}
if args.only_tags:
    views = {}
for t in lay['place_tags']:   # 장소 태그(L1~L3): 접근 면 법선 방향 --tag-dist 앞에서 태그를 봄(높이는 거리에 비례)
    c, n = np.array(t['center']), np.array(t['normal'])
    views[f"{t['place']}_tag{t['id']}"] = (look_at(c + n * args.tag_dist + np.array([0, 0, 0.165 * args.tag_dist / 0.45]), c), d435)

args.out.mkdir(parents=True, exist_ok=False)
ctx = omni.usd.get_context()
ctx.open_stage(str((args.world / 'task_world.usda').resolve()))
for _ in range(60):
    app.update()
stage = ctx.get_stage()
if args.color_temp is not None:   # 돔 · 천장 사각 색온도를 이 값으로 (0 = 끔, 흰빛)
    for lp in stage.GetPrimAtPath('/World/Lights').GetChildren():
        lux = UsdLux.LightAPI(lp)
        lux.CreateEnableColorTemperatureAttr(args.color_temp > 0)
        lux.CreateColorTemperatureAttr(float(args.color_temp) if args.color_temp > 0 else 6500.0)
    for _ in range(10):
        app.update()
if args.views:
    views = {k: v for k, v in views.items() if k in args.views}
rep.orchestrator.set_capture_on_play(False)
done = {}
for name, (T, k) in views.items():
    path = f'/World/CheckCams/{name}'
    set_camera(stage, path, T, *k)
    rp = rep.create.render_product(path, (W, H))
    an = rep.AnnotatorRegistry.get_annotator('rgb')
    an.attach(rp)
    for _ in range(3):
        rep.orchestrator.step(rt_subframes=args.subframes)
    img = np.asarray(an.get_data())[..., :3][..., ::-1]
    cv2.imwrite(str(args.out / f'{name}.png'), img)
    done[name] = {'T_world_optical': np.round(T, 6).tolist(), 'K': list(map(float, k)), 'mean_rgb': float(img.mean())}
    an.detach()
    rp.destroy()
(args.out / 'render_check.json').write_text(json.dumps({'world': str(args.world), 'views': done}, indent=1, ensure_ascii=False))
print('렌더', len(done), '장:', ', '.join(f"{n} {v['mean_rgb']:.0f}" for n, v in done.items()))
app.close()
