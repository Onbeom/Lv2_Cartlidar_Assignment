#!/usr/bin/env python3
"""팔 카메라 높이: FK 대 실측 (P2b 추가 판정, PC). arm_handeye.py --poses fk_poses.yaml --any 로 찍은 caps.jsonl 을 쓴다.

실측 높이 = 보드 PnP 로 낸 '카메라 → 보드 면' 수직 거리 + 보드 면의 책상 위 높이(그 자리, 기울기 보정).
FK 높이  = URDF 사슬(base_link → link5, 관절각 = 펌웨어 값 그대로, robot_state_publisher 와 같다고 봄) → arm_camera_optical_frame 의 z
           + base_footprint 와 base_link 차(6.75 mm). base_footprint = 책상 면이라고 본다.
판정: 자세(이름) · 메모(빈 손 / 퍽 쥠)별 |FK − 실측| ≤ 10 mm.

fk_height.py <기록 폴더> <robot.urdf.xacro> --square <가로 칸 m> [--square_y <세로 칸 m>] --screen_h <보드 가운데 책상 위 높이 mm>
             [--tilt <앞뒤 °> <좌우 °>] [--intr arm_camera_1280x720.yaml] [--max_rms 1.0]
fk_height.py selftest
"""
import argparse, json, math, os, sys
import numpy as np, yaml
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', '261106_1655_real_drive', 'jetson'))
import handeye_lib as L   # noqa: E402

BF_Z = 0.00675   # base_link 는 base_footprint 위 6.75 mm


def heights(caps, ch, K, D, square, square_y, screen_h, tilt, max_rms, squares_x=10):
    Xd = ch.design('link5', 'arm_camera_optical_frame')
    n = squares_x - 1; sy = square if square_y is None else square_y
    ta, tb = math.radians(tilt[0]), math.radians(tilt[1])
    rows = []
    for c in caps:
        if not c.get('ids') or len(c['ids']) < 6:
            continue
        bx, by = (int(v) for v in c.get('board', f'{squares_x}x7').lower().split('x'))   # 찍을 때 쓴 보드 (caps 에 기록, 옛 기록은 10x7)
        c0 = np.array([bx / 2 * square, by / 2 * sy])     # 보드 가운데
        T, rms = L.board_pose(np.array(c['corners']), c['ids'], K, D, square, squares_x=bx, square_y_m=square_y)
        if T is None or rms > max_rms:
            continue
        Tbc = np.linalg.inv(T)                                # 보드 → 카메라 (보드: x 오른쪽 · y 아래 · z 화면 안)
        cam_b = Tbc[:3, 3]
        perp = -cam_b[2]                                     # 화면 위쪽 수직 거리
        # 카메라 바로 아래 보드 면 높이: 가운데 높이 + 기울기 × 가운데에서 떨어진 거리(화면 위 테두리를 로봇 쪽으로, 화면 긴 변을 로봇 좌우로 놓았다고 봄 — R1 노트북 배치와 같음)
        dx_fwd = cam_b[1] - c0[1]; dy_left = cam_b[0] - c0[0]   # 화면 위 = 로봇 쪽: 보드 y(아래) = 로봇 앞, 보드 x(오른쪽) = 로봇 왼쪽
        plane = screen_h * 1e-3 + dx_fwd * math.tan(ta) + dy_left * math.tan(tb)
        real = perp + plane
        Tfk = ch.fk({f'joint{k + 1}': c['q'][k] for k in range(5)}) @ Xd
        fk = Tfk[2, 3] + BF_Z
        rows.append({'pose': c.get('pose', ''), 'note': c.get('note', ''), 'rms': rms, 'real_mm': real * 1e3, 'fk_mm': fk * 1e3,
                     'diff_mm': (fk - real) * 1e3, 'n': len(c['ids'])})
    return rows


def report(rows, lim=10.0):
    groups = {}
    for r in rows:
        groups.setdefault((r['pose'], r['note']), []).append(r)
    ok_all = True
    for (p, nt), g in sorted(groups.items()):
        d = np.array([r['diff_mm'] for r in g])
        ok = abs(d.mean()) <= lim; ok_all &= ok
        print(f"{p or '(이름 없음)'} · {nt or '-'}: 장 {len(g)}, 실측 {np.mean([r['real_mm'] for r in g]):.1f} mm, FK {np.mean([r['fk_mm'] for r in g]):.1f} mm, "
              f"FK − 실측 {d.mean():+.1f} mm (흩어짐 {d.std():.1f}) → {'통과' if ok else '불통과'} (|차| ≤ {lim:g} mm)")
    print('판정:', '통과' if ok_all and groups else '불통과')


def selftest(board='10x7'):
    bx, by = (int(v) for v in board.split('x'))
    import cv2
    ch = L.Chain(L.urdf_from_xacro(os.path.expanduser('~/git/Lv2_Cartlidar_Assignment_d435pitch/workspace/ros2_ws/src/opencr_hardware/urdf/robot.urdf.xacro')))
    Xd = ch.design('link5', 'arm_camera_optical_frame')
    K = np.array([[865.0, 0, 640], [0, 865, 360], [0, 0, 1]]); D = np.zeros(5)
    sq = 0.030 if bx >= 10 else 0.049; screen_h = 80.0
    caps = []
    rng = np.random.default_rng(0)
    for name, q, dz in (('PLACE', [-0.0283, -0.9699, -0.0276, 1.3892, 0], 0.012), ('TAG', [-0.0283, -0.5540, -0.4264, 1.4594, 0], -0.020)):
        Tcam = ch.fk({f'joint{k + 1}': q[k] for k in range(5)}) @ Xd
        Tcam_true = Tcam.copy(); Tcam_true[2, 3] += dz           # 실제 카메라가 FK 보다 dz 높음
        Tcam_true[2, 3] += BF_Z                                  # base_footprint 기준
        # 보드: 카메라 시선이 책상에 닿는 곳 둘레, 화면 위 = 로봇 쪽(+x 가 로봇 앞이면 보드 y 아래 = 로봇 앞)
        ray = Tcam_true[:3, 2]; t = -(Tcam_true[2, 3] - screen_h * 1e-3) / ray[2]; hit = Tcam_true[:3, 3] + t * ray
        Rb = np.column_stack([[0, 1, 0], [1, 0, 0], [0, 0, -1]])   # 화면 위 = 로봇 쪽 · 화면이 위를 봄: 보드 x → 로봇 +y, y(아래) → +x, z(화면 안) → 아래
        Tb = L.T_of(Rb, hit - Rb @ np.array([bx / 2 * sq, by / 2 * sq, 0]))
        Tcb = np.linalg.inv(Tcam_true) @ Tb
        ids = list(range((bx - 1) * (by - 1)))
        obj = np.array([[((i % (bx - 1)) + 1) * sq, ((i // (bx - 1)) + 1) * sq, 0.0] for i in ids])
        pc = (Tcb[:3, :3] @ obj.T).T + Tcb[:3, 3]
        uv = (K @ pc.T).T; uv = uv[:, :2] / uv[:, 2:] + rng.normal(0, 0.3, (len(ids), 2))
        ok = (uv[:, 0] > 0) & (uv[:, 0] < 1280) & (uv[:, 1] > 0) & (uv[:, 1] < 720) & (pc[:, 2] > 0)
        # 기록은 '돌리기 전 화면' 좌표: 발행 광학 = RZ_PI · 돌리기 전 → 돌리기 전 = 180° 돌림
        uv0 = np.column_stack([1279 - uv[:, 0], 719 - uv[:, 1]])
        for note in ('빈 손', '퍽 쥠'):
            caps.append({'q': q + [0.0], 'corners': uv0[ok].tolist(), 'ids': [i for i, o in zip(ids, ok) if o], 'pose': name, 'note': note, 'board': board})
    K0 = K.copy(); K0[0, 2] = 1279 - K[0, 2]; K0[1, 2] = 719 - K[1, 2]
    rows = heights(caps, ch, K0, D, sq, None, screen_h, (0, 0), 2.0)
    for r in rows:
        print(f"모의 {r['pose']} {r['note']}: 코너 {r['n']}, FK − 실측 {r['diff_mm']:+.1f} mm (참 {'−12.0' if r['pose'] == 'PLACE' else '+20.0'}), RMS {r['rms']:.2f} px")


if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] == 'selftest':
        selftest(sys.argv[2] if len(sys.argv) > 2 else '10x7'); sys.exit(0)
    ap = argparse.ArgumentParser()
    ap.add_argument('dir'); ap.add_argument('xacro')
    ap.add_argument('--square', type=float, required=True); ap.add_argument('--square_y', type=float)
    ap.add_argument('--screen_h', type=float, required=True, help='보드 가운데 책상 위 높이 mm')
    ap.add_argument('--tilt', type=float, nargs=2, default=[0.0, 0.0], help='앞뒤 · 좌우 ° (+ = 로봇 앞쪽 · 왼쪽이 높음)')
    ap.add_argument('--intr', default=os.path.join(HERE, '..', '261106_1655_real_drive', 'jetson', 'arm_camera_1280x720.yaml'))
    ap.add_argument('--max_rms', type=float, default=1.0)
    a = ap.parse_args()
    info = yaml.safe_load(open(a.intr))
    K = np.array(info['camera_matrix']['data']).reshape(3, 3); D = np.array(info['distortion_coefficients']['data'])
    ch = L.Chain(L.urdf_from_xacro(a.xacro))
    caps = [json.loads(l) for l in open(os.path.join(a.dir, 'caps.jsonl'))]
    caps = [c for c in caps if c.get('pose')]
    report(heights(caps, ch, K, D, a.square, a.square_y, a.screen_h, a.tilt, a.max_rms))
