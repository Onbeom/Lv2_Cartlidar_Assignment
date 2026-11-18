"""발제(mission.pdf v5) 2쪽 그림에서 새 맵 벽 좌표 초안을 만들고 그림과 겹쳐 확인한다.

좌표: 왼쪽 아래 기둥 중심 = 원점, +X 오른쪽, +Y 위, 단위 mm (발제와 같음).
바깥 기둥 간격 x 734 · y 394(옛 twin maze 와 같음), 안쪽 벽 끝점은 734 격자를 (367, 587) 만큼 민 자리로 읽힘.
실행: ~/yolo-venv/bin/python -I map_from_mission.py <발제 2쪽 그림 png> <출력 폴더>
  (그림은 `pdfimages -png -f 2 -l 2 mission.pdf img` 의 img-000.png, 2160×2160)
"""
import json
import sys

import numpy as np
from PIL import Image, ImageDraw

# 그림 축 눈금에서 읽은 변환 (원본 2160 px 기준): x 0 → 220.6 px, 4000 → 1345.1 px / y 0 → 1973.4 px, 6000 → 284.5 px
PX0, PXS = 220.6, (1345.1 - 220.6) / 4000.0
PY0, PYS = 1973.4, (1973.4 - 284.5) / 6000.0

OUTER = (4404, 6304)   # 바깥 기둥 중심 끝 (기둥 머리 Ø60 포함 4464 × 6364)

# 안쪽 벽 (x1, y1, x2, y2) mm. note 는 그림에서 확실하지 않은 것
INNER = [
    (1101, 4991, 1101, 5725, ''),
    (1835, 4991, 2569, 4991, ''),
    (2569, 4991, 2569, 5725, ''),
    (3303, 4991, 3303, 5725, ''),
    (367, 4257, 1101, 4257, ''),
    (1101, 3523, 1101, 4257, ''),
    (3303, 4257, 4037, 4257, ''),
    (3303, 2055, 3303, 3523, ''),
    (3303, 3523, 4037, 3523, ''),
    (2569, 2055, 3303, 2055, ''),
    (1101, 2055, 1101, 2789, ''),
    (1101, 2055, 1835, 2055, ''),
    (1835, 1321, 1835, 2055, ''),
    (1101, 1321, 1835, 1321, ''),
    (1101, 587, 1101, 1321, '아래 끝은 그림 글상자에 가려 y 650 mm 아래가 안 보임. 격자로 587 이라 둠 (실물 확인 필요)'),
    (2569, 1321, 4037, 1321, ''),
]

# 좁은 틈 (참고 정보, 지도에 넣지 않음 — 사용자 2026-10-28: 맵 · SLAM 은 빈 곳, 주행 costmap 외곽 · inflation 이 못 지나가게 판단). 벽 끝 ↔ 바깥 벽 기둥 중심 367 mm, 기둥 머리 Ø60 빼면 307 mm < 차체 폭 335 mm(URDF 261027_1711 외곽 y ±0.1676)
NARROW_GAPS = [
    (0, 4257, 367, 4257, '실물 확인(사용자 2026-10-28, (300, 4200) 근처 좁아 못 지나감)'),
    (4037, 4257, 4404, 4257, '그림상 같은 307 mm 틈 — 실물 확인 전'),
    (4037, 3523, 4404, 3523, '그림상 같은 307 mm 틈 — 실물 확인 전'),
    (4037, 1321, 4404, 1321, '그림상 같은 307 mm 틈 — 실물 확인 전'),
]

# 발제 본문 좌표 (mm)
TEXT = {
    'signal_base': [80, 950, 0], 'barcode_center': [80, 650, 450], 'start_center': [734, 950],
    'L': {'L1': [3670, 950], 'L2': [2202, 3700], 'L3': [3670, 5354]},
    'start_boxes': [[1468, 950], [3670, 3152], [2936, 5354], [734, 5354], [2202, 5354], [3670, 3886]],
    'box_small': [205, 205, 70], 'center_stand': {'boxes_410': 4, 'size': [820, 820, 70]},
    'puck_on_box_dx': [-40, 40], 'dice_grid': [-60, 0, 60],
}


def to_px(x, y):
    return PX0 + x * PXS, PY0 - y * PYS


def segments():
    sx, sy = OUTER
    out = [(0, 0, sx, 0, ''), (sx, 0, sx, sy, ''), (sx, sy, 0, sy, ''), (0, sy, 0, 0, '')]
    return [('outer', *s) for s in out] + [('inner', *s) for s in INNER]


def main(img_path, out_dir):
    im = Image.open(img_path).convert('RGB')
    a = np.asarray(im).astype(int)
    # 벽 색 (진한 남색) 화소
    dark = (np.abs(a[..., 0] - 40) < 25) & (np.abs(a[..., 1] - 55) < 25) & (np.abs(a[..., 2] - 70) < 25)
    # 지도 영역 안만 (오른쪽 신호등 그림 · 글자 제외)
    x_lo, y_hi = to_px(-150, -150)
    x_hi, y_lo = to_px(OUTER[0] + 150, OUTER[1] + 150)
    roi = np.zeros_like(dark)
    roi[int(y_lo):int(y_hi), int(x_lo):int(x_hi)] = True
    dark &= roi

    mask = Image.new('L', im.size, 0)
    d = ImageDraw.Draw(mask)
    for _, x1, y1, x2, y2, _ in segments():
        d.line([to_px(x1, y1), to_px(x2, y2)], fill=255, width=9)
    m = np.asarray(mask) > 0
    # 그린 벽 선 위의 진한 화소 비율 (선 가운데 3 px 만 봄)
    core = Image.new('L', im.size, 0)
    dc = ImageDraw.Draw(core)
    per = []
    for kind, x1, y1, x2, y2, note in segments():
        one = Image.new('L', im.size, 0)
        ImageDraw.Draw(one).line([to_px(x1, y1), to_px(x2, y2)], fill=255, width=3)
        o = np.asarray(one) > 0
        dc.line([to_px(x1, y1), to_px(x2, y2)], fill=255, width=3)
        per.append({'kind': kind, 'mm': [x1, y1, x2, y2], 'on_dark': round(float(dark[o].mean()), 3), 'note': note})
    unexplained = dark & ~m
    res = {
        'wall_pixels_dark_explained': round(float((dark & m).sum() / max(dark.sum(), 1)), 4),
        'unexplained_dark_px': int(unexplained.sum()),
        'segments': per,
    }
    # 겹친 그림: 초안 벽 = 빨강 선, 설명 안 된 진한 화소 = 주황
    ov = im.copy()
    do = ImageDraw.Draw(ov)
    for _, x1, y1, x2, y2, _ in segments():
        do.line([to_px(x1, y1), to_px(x2, y2)], fill=(230, 30, 30), width=2)
    ys, xs = np.nonzero(unexplained)
    for x, y in zip(xs, ys):
        ov.putpixel((int(x), int(y)), (255, 140, 0))
    ov.save(f'{out_dir}/map_overlay.png')

    walls = [{'kind': k, 'p1': [x1, y1], 'p2': [x2, y2], 'note': n} for k, x1, y1, x2, y2, n in segments()]
    json.dump({
        'description': '주사위 기억 미션 새 맵 벽 초안 (발제 mission.pdf v5 2쪽 그림에서 읽음, 실물 확인 전). 단위 mm, 왼쪽 아래 기둥 중심 원점.',
        'outer_posts_step_mm': {'x': 734, 'y': 394},
        'inner_grid_mm': {'step': 734, 'x0': 367, 'y0': 587},
        'walls': walls,
        'narrow_gaps_reference_only': [{'p1': [x1, y1], 'p2': [x2, y2], 'note': n} for x1, y1, x2, y2, n in NARROW_GAPS],
        'field_notes': [
            '시작 박스가 벽과 평행하지 않게 돌아가 놓일 수 있음 → 퍽 자리는 박스 중심 + 박스 yaw 로 돌린 (±40, 0) (사용자 2026-10-28)',
            '원기둥은 항상 서 있고 직육면체는 누워 있을 수 있음 (발제 윗면 130 mm 와 다름, 사용자 2026-10-28)',
        ],
        'from_text': TEXT, 'check': {k: v for k, v in res.items() if k != 'segments'},
    }, open(f'{out_dir}/map_dice_draft.json', 'w'), ensure_ascii=False, indent=1)
    print(json.dumps(res, ensure_ascii=False, indent=1))


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2])
