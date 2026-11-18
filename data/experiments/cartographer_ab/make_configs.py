"""Cartographer 2D 설정 8가지(해상도 × 스캔 매칭 가중치 × 회전 삽입 각도)를 만든다.

원본 Lua(waffle-lyrical-slam의 waffle_2d.lua)는 고치지 않고, 원본 내용을 그대로 복사한 뒤
`return options` 앞에 바꾼 값을 덧붙인다. 바꾼 줄에는 원래 값을 주석으로 남긴다.
만든 설정 목록과 각 설정의 지도 해상도는 configs/manifest.json에 쓴다.

사용: python3 make_configs.py <출력 폴더> [원본 lua]
"""
import itertools
import json
import os
import sys

DEFAULT_SRC = os.path.expanduser(
    '~/git/ros_test/waffle-lyrical-slam/src/waffle_slam_course/config/waffle_2d.lua')

# (이름, A 값, B 값). A는 원본(또는 Cartographer 기본값)과 같다.
RESOLUTION = ('res', 0.05, 0.025)          # [m] 부분 지도 칸 크기 (기본값 0.05)
CERES_WEIGHT = ('w', (10.0, 40.0), (1.0, 4.0))   # ceres_scan_matcher translation/rotation_weight (기본값 10/40)
MAX_ANGLE_DEG = ('ang', 0.1, 1.0)          # [deg] motion_filter.max_angle_radians (원본 0.1)

MOTION_LINE = 'TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)'


def make(src_text, res, w, ang):
    if MOTION_LINE not in src_text or 'return options' not in src_text:
        raise SystemExit('원본 Lua 내용이 예상과 다르다: ' + MOTION_LINE)
    text = src_text.replace(MOTION_LINE, f'TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad({ang})'
                            f'  -- 실험 변수 (원본: math.rad(0.1))')
    extra = [
        '-- ---- cartographer_ab 실험 변수 (원본 파일에는 없는 줄, Cartographer 기본값은 주석) ----',
        f'TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = {res}  -- 기본값 0.05',
        f'TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = {w[0]}  -- 기본값 10.',
        f'TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = {w[1]}  -- 기본값 40.',
        '',
    ]
    return text.replace('return options', '\n'.join(extra) + '\nreturn options')


def main():
    out = sys.argv[1]
    src = sys.argv[2] if len(sys.argv) > 2 else DEFAULT_SRC
    with open(src) as f:
        src_text = f.read()
    cfg_dir = os.path.join(out, 'configs')
    os.makedirs(cfg_dir, exist_ok=True)
    manifest = []
    combos = itertools.product((RESOLUTION[1], RESOLUTION[2]), (CERES_WEIGHT[1], CERES_WEIGHT[2]),
                               (MAX_ANGLE_DEG[1], MAX_ANGLE_DEG[2]))
    for idx, (res, w, ang) in enumerate(combos, start=1):
        name = f'c{idx}_res{res:g}_w{w[0]:g}-{w[1]:g}_ang{ang:g}'
        with open(os.path.join(cfg_dir, name + '.lua'), 'w') as f:
            f.write(make(src_text, res, w, ang))
        manifest.append({'name': name, 'resolution': res, 'translation_weight': w[0], 'rotation_weight': w[1],
                         'max_angle_deg': ang,
                         'changed': [k for k, a, v in (('res', RESOLUTION[1], res), ('w', CERES_WEIGHT[1], w),
                                                      ('ang', MAX_ANGLE_DEG[1], ang)) if a != v]})
    with open(os.path.join(cfg_dir, 'manifest.json'), 'w') as f:
        json.dump({'source_lua': src, 'configs': manifest}, f, indent=2, ensure_ascii=False)
    for m in manifest:
        print(m['name'], '바꾼 변수:', ','.join(m['changed']) or '없음(기준)')


if __name__ == '__main__':
    main()
