#!/usr/bin/env python3
"""261022 리비전에서 261022_1800 리비전을 만든다. 원본(261022_1243)은 바꾸지 않는다.

바꾸는 것
  1. 라이다 scan 프레임 방향: 라이다가 3층 밑판에 거꾸로 매달린 실물·CAD와 맞게 roll π를 넣는다.
     lidar_front_scan_fixed rpy 0 0 0 -> π 0 0, lidar_rear_scan_fixed rpy 0 0 π -> π 0 π (위치·높이 그대로)
  2. RealSense D435 렌즈 프레임 추가: 기존 top_camera_*_optical_frame(CAD 스테레오 렌즈 원통 중점)의 자식으로
     infra1·infra2·color 광학 프레임을 붙인다. 값은 D435 공장 보정값(rs-enumerate-devices -c)이다.
  3. 패키지 이름: waffle_4wd_description_261022 -> waffle_4wd_description_261022_1800

사용: python3 make_revision_261022_1800.py --src <261022 패키지 폴더> --dst <새 패키지 폴더> --calib <rs-enumerate-devices -c 출력>
"""
import argparse
import json
import math
import re
import shutil
from pathlib import Path

OLD_PKG = 'waffle_4wd_description_261022'
NEW_PKG = 'waffle_4wd_description_261022_1800'
PI = '3.1415926535897931'
CAMERAS = ('top_camera_front', 'top_camera_rear')
# 패키지 이름을 바꿀 파일 (REVISION_261022.md는 지난 기록이라 그대로 둔다)
RENAME_FILES = ('package.xml', 'CMakeLists.txt', 'README.md', 'robot_data.yaml',
                'urdf/waffle_4wd.urdf', 'urdf/waffle_4wd.urdf.xacro', 'urdf/assemblies/main.urdf.xacro',
                'launch/display.launch.py')
URDF_FILES = ('urdf/waffle_4wd.urdf', 'urdf/assemblies/main.urdf.xacro')


def parse_extrinsic(text, src, dst):
    """'Extrinsic from "src" To "dst"' 블록의 회전 행렬(출력 그대로 3줄)과 이동 벡터."""
    m = re.search(rf'Extrinsic from "{src}"\s+To\s+"{dst}" :\s*Rotation Matrix:\s*'
                  r'([-\d.e+\s]+?)\s*Translation Vector:\s*([-\d.e+\s]+?)\n', text)
    if not m:
        raise SystemExit(f'보정값에 {src} -> {dst} 블록이 없음')
    rot = [float(v) for v in m.group(1).split()]
    trans = [float(v) for v in m.group(2).split()]
    assert len(rot) == 9 and len(trans) == 3
    return [rot[0:3], rot[3:6], rot[6:9]], trans


def matvec(m, v):
    return [sum(m[i][j] * v[j] for j in range(3)) for i in range(3)]


def transpose(m):
    return [[m[j][i] for j in range(3)] for i in range(3)]


def rpy_from_matrix(r):
    """URDF rpy (R = Rz(yaw) Ry(pitch) Rx(roll))."""
    pitch = math.asin(max(-1.0, min(1.0, -r[2][0])))
    roll = math.atan2(r[2][1], r[2][2])
    yaw = math.atan2(r[1][0], r[0][0])
    return roll, pitch, yaw


def d435_frames(calib_text):
    """infra1 기준 infra2·color 위치·자세 -> 스테레오 중점(기존 optical frame) 기준으로 바꾼다."""
    _, t_d_ir2 = parse_extrinsic(calib_text, 'Depth', 'Infrared 2')      # p_ir2 = R p_d + t
    r_d_c, t_d_c = parse_extrinsic(calib_text, 'Depth', 'Color')
    r_c_d, t_c_d = parse_extrinsic(calib_text, 'Color', 'Depth')
    # Depth 원점 = Infrared 1 원점 (Depth -> Infrared 1 이동 0)
    _, t_d_ir1 = parse_extrinsic(calib_text, 'Depth', 'Infrared 1')
    assert all(abs(v) < 1e-12 for v in t_d_ir1), 'Depth와 Infrared 1 원점이 다름'
    # 출력 행렬을 행으로 읽은 해석이 맞는지 이동 벡터로 확인: t_c_d = -R_c_d^T ... 대신 t_c_d = -(R_d_c)^T t_d_c
    pred_rows = [-v for v in matvec(transpose(r_d_c), t_d_c)]
    pred_cols = [-v for v in matvec(r_d_c, t_d_c)]
    err_rows = max(abs(a - b) for a, b in zip(pred_rows, t_c_d))
    err_cols = max(abs(a - b) for a, b in zip(pred_cols, t_c_d))
    if err_rows > 1e-6:
        raise SystemExit(f'행렬 해석 확인 실패: 행 해석 오차 {err_rows:.3e} m, 열 해석 오차 {err_cols:.3e} m')
    baseline = -t_d_ir2[0]                     # IR1 원점이 IR2 좌표에서 x = -b → IR2는 IR1의 +x(오른쪽)
    half = baseline / 2.0
    # color 원점의 depth(IR1) 좌표 = t_c_d, color 축의 depth 좌표 표현 = R_c_d (p_d = R_c_d p_c + t_c_d)
    color_xyz = [t_c_d[0] - half, t_c_d[1], t_c_d[2]]
    color_rpy = rpy_from_matrix(r_c_d)
    return {
        'baseline_m': baseline,
        'rotation_check_err_m': err_rows,
        'infra1': ([-half, 0.0, 0.0], (0.0, 0.0, 0.0)),
        'infra2': ([half, 0.0, 0.0], (0.0, 0.0, 0.0)),
        'color': (color_xyz, color_rpy),
    }


def fmt(v):
    return repr(float(v))


def frame_xml(cam, frames, prefix):
    parts = []
    for name in ('infra1', 'infra2', 'color'):
        xyz, rpy = frames[name]
        link = f'{prefix}{cam}_{name}_optical_frame'
        parts.append(
            f'<link name="{link}" /><joint name="{link}_fixed" type="fixed">'
            f'<parent link="{prefix}{cam}_optical_frame" /><child link="{link}" />'
            f'<origin xyz="{" ".join(fmt(v) for v in xyz)}" rpy="{" ".join(fmt(v) for v in rpy)}" /></joint>')
    return ''.join(parts)


def edit_urdf(text, frames, prefix):
    rules = (
        ('lidar_front_scan_fixed', 'rpy="0 -0 0"', f'rpy="{PI} 0 0"'),
        ('lidar_rear_scan_fixed', f'rpy="0 -0 {PI}"', f'rpy="{PI} 0 {PI}"'),
    )
    for joint, old, new in rules:
        pat = re.compile(rf'(<joint name="{re.escape(prefix)}{joint}" type="fixed">.*?<origin [^>]*?){re.escape(old)}')
        text, n = pat.subn(lambda m: m.group(1) + new, text, count=1)
        if n != 1:
            raise SystemExit(f'{joint} rpy를 찾지 못함')
    for cam in CAMERAS:
        end = f'<child link="{prefix}{cam}_optical_frame" />'
        i = text.find(end)
        j = text.find('</joint>', i)
        if i < 0 or j < 0 or text.count(end) != 1:
            raise SystemExit(f'{cam}_optical_frame 관절을 찾지 못함')
        j += len('</joint>')
        text = text[:j] + frame_xml(cam, frames, prefix) + text[j:]
    return text


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--src', required=True)
    ap.add_argument('--dst', required=True)
    ap.add_argument('--calib', required=True)
    args = ap.parse_args()
    src, dst = Path(args.src).resolve(), Path(args.dst).resolve()
    if dst.exists():
        raise SystemExit(f'이미 있음: {dst}')
    frames = d435_frames(Path(args.calib).read_text())
    shutil.copytree(src, dst, symlinks=True)

    counts = {}
    for rel in RENAME_FILES:
        p = dst / rel
        t = p.read_text(encoding='utf-8')
        counts[rel] = t.count(OLD_PKG)
        p.write_text(t.replace(OLD_PKG, NEW_PKG), encoding='utf-8')

    for rel in URDF_FILES:
        p = dst / rel
        prefix = '${prefix}' if rel.endswith('.xacro') else ''
        p.write_text(edit_urdf(p.read_text(encoding='utf-8'), frames, prefix), encoding='utf-8')

    # robot_data.yaml(JSON 형식): 라이다 rpy와 새 프레임
    rd_path = dst / 'robot_data.yaml'
    rd = json.loads(rd_path.read_text(encoding='utf-8'))
    joints, links = rd['joints'], rd['links']
    joints['lidar_front_scan_fixed']['origin']['rpy'] = f'{PI} 0 0'
    joints['lidar_rear_scan_fixed']['origin']['rpy'] = f'{PI} 0 {PI}'
    for cam in CAMERAS:
        for name in ('infra1', 'infra2', 'color'):
            xyz, rpy = frames[name]
            link = f'{cam}_{name}_optical_frame'
            links[link] = {'logical_frame': True, 'inertial': None}
            joints[f'{link}_fixed'] = {
                'type': 'fixed', 'parent': f'{cam}_optical_frame', 'child': link,
                'origin': {'xyz': ' '.join(fmt(v) for v in xyz), 'rpy': ' '.join(fmt(v) for v in rpy)},
                'limits': None, 'axis': None, 'mimic': None}
    rd_path.write_text(json.dumps(rd, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')

    meta_path = dst / 'simulation_metadata.json'
    meta = json.loads(meta_path.read_text(encoding='utf-8'))
    meta['links'] = meta['links'] + 6
    meta['joints'] = meta['joints'] + 6
    meta['revision_261022_1800'] = {
        'lidar_scan': 'roll pi added to lidar_front_scan_fixed / lidar_rear_scan_fixed (LDS hangs upside down '
                      'under floor 3 in CAD and hardware); position and height unchanged, still uncalibrated',
        'd435_frames': 'infra1/infra2/color optical frames added under top_camera_{front,rear}_optical_frame '
                       '(CAD stereo lens midpoint). Values from D435 serial 261822072247 factory extrinsics '
                       '(rs-enumerate-devices -c, 2026-10-22); the same values are assumed for the second D435',
        'baseline_m': frames['baseline_m'],
    }
    meta_path.write_text(json.dumps(meta, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')

    print(json.dumps({'dst': str(dst), 'package_renames': counts, 'baseline_m': frames['baseline_m'],
                      'rotation_check_err_m': frames['rotation_check_err_m'],
                      'frames': {k: frames[k] for k in ('infra1', 'infra2', 'color')}}, indent=1))


if __name__ == '__main__':
    main()
