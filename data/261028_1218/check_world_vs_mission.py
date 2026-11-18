"""인지 세션이 만든 발제 월드(dice_mission seed)를 발제 · B0 초안과 따로 대조 (layout.json · 벽 자료만, USD 안 엶).
실행: python3 -I check_world_vs_mission.py <월드 폴더들>
"""
import json, math, sys
sys.path.insert(0, '/home/pa4/git/lv2_next_module/261028_1218')
from alloc_proto import allocate, assign_groups, plan_y   # noqa: E402

D = json.load(open('/home/pa4/git/lv2_next_module/261028_1218/map_dice_draft.json'))
START = [[1468, 950], [3670, 3152], [2936, 5354], [734, 5354], [2202, 5354], [3670, 3886]]


def unit_segments(walls):
    s = set()
    for w in walls:
        (x1, y1), (x2, y2) = w['p1'], w['p2']
        n = round(math.hypot(x2 - x1, y2 - y1) / (734 if y1 == y2 or x1 == x2 and abs(y2 - y1) % 734 == 0 else 394))
        for k in range(n):
            a = (x1 + (x2 - x1) * k / n, y1 + (y2 - y1) * k / n)
            b = (x1 + (x2 - x1) * (k + 1) / n, y1 + (y2 - y1) * (k + 1) / n)
            s.add(tuple(sorted([(round(a[0]), round(a[1])), (round(b[0]), round(b[1]))])))
    return s


def in_box(p, box):
    cx, cy, _ = box['center']; c, s = math.cos(-box['yaw']), math.sin(-box['yaw'])
    dx, dy = p[0] - cx, p[1] - cy
    return c * dx - s * dy, s * dx + c * dy


for d in sys.argv[1:]:
    L = json.load(open(f'{d}/layout.json'))
    dr = json.load(open(L['sources']['drawing']))
    bad = []
    # 벽: 그림 판 중심 ↔ 초안 단위 판 중심
    pc = sorted((round(p['center'][0] * 1000), round(p['center'][1] * 1000), p['axis']) for p in dr['panels'] if p['kind'] == 'inner')
    want = []
    for w in D['walls']:
        if w['kind'] != 'inner':
            continue
        (x1, y1), (x2, y2) = w['p1'], w['p2']
        n = round(math.hypot(x2 - x1, y2 - y1) / 734)
        for k in range(n):
            want.append((round(x1 + (x2 - x1) * (k + .5) / n), round(y1 + (y2 - y1) * (k + .5) / n), 'x' if y1 == y2 else 'y'))
    want.sort()
    if [(x, y) for x, y, _ in pc] != [(x, y) for x, y, _ in want] or any(abs(a[0] - b[0]) > 1 for a, b in zip(pc, want)):
        bad.append(f'안쪽 벽 다름 {len(pc)} vs {len(want)}')
    if any(p['kind'] not in ('outer', 'inner') for p in dr['panels']):
        bad.append('막힌 판 있음')
    boxes = {b['name']: b for b in L['boxes']}
    for i, c in enumerate(START, 1):
        b = boxes[f'start_box_{i}']
        if math.hypot(b['center'][0] * 1000 - c[0], b['center'][1] * 1000 - c[1]) > 1 or b['size'] != [0.205, 0.205, 0.07]:
            bad.append(f'시작 박스 {i}')
    yaws = [round(math.degrees(boxes[f'start_box_{i}']['yaw']), 1) for i in range(1, 7)]
    pk = L['pucks']
    ids = sorted(p['tag_id'] for p in pk)
    if ids != list(range(12)):
        bad.append(f'ID {ids}')
    lying = 0
    for p in pk:
        b = boxes[p['location']]
        u, v = in_box(p['center'], b)
        if abs(u - p['side'] * 0.04) > 0.002 or abs(v) > 0.002:
            bad.append(f"{p['name']} 칸 어긋남 {u:.3f},{v:.3f}")
        if p['color'] not in ('red', 'green'):
            bad.append(f"{p['name']} 색 {p['color']}")
        if p['shape'] == 'cylinder' and not p['standing']:
            bad.append(f"{p['name']} 누운 원기둥")
        lying += not p['standing']
        if abs(p['aabb'][0][2] - 0.07) > 0.001:
            bad.append(f"{p['name']} 바닥 z {p['aabb'][0][2]}")
        # 박스 윗면 안 (aabb 네 모서리를 박스 좌표로)
        for x in (p['aabb'][0][0], p['aabb'][1][0]):
            for y in (p['aabb'][0][1], p['aabb'][1][1]):
                uu, vv = in_box((x, y), b)
                if max(abs(uu), abs(vv)) > 0.1025 + 0.01:   # aabb 는 실제보다 큼 → 1 cm 여유
                    bad.append(f"{p['name']} 박스 밖 의심")
    sl = L['signal_light']
    cnt = {'R': sl['red'], 'Y': sl['yellow'], 'G': sl['green']}
    seqcnt = {g: sl['sequence'].count(g) for g in 'RYG'}
    if cnt != seqcnt or len(sl['sequence']) != 12:
        bad.append('신호 횟수')
    pucks = [{'id': p['tag_id'], 'color': p['color'], 'shape': p['shape']} for p in pk]
    bc = L['barcodes'][0]
    try:
        a = allocate(pucks, cnt, bc['text'])
        alloc = ' '.join(f"{x['location']}={x['group']}{len(x['cells'])}" + (f"({x['y_shape']})" if x['y_shape'] else '') for x in a)
    except ValueError as e:
        alloc = f'오류 {e}'
    r = L['robot']
    if abs(r['x'] - 0.734) > 1e-3 or abs(r['y'] - 0.95) > 1e-3 or abs(abs(r['yaw']) - math.pi) > 1e-3:
        bad.append('출발 자세')
    print(f"{d.split('/')[-1]}: {'통과' if not bad else bad} | 박스 yaw° {yaws} | 빨강 {sum(p['color']=='red' for p in pk)} · 누운 직육면체 {lying} | 신호 {sl['sequence']} {cnt} 월드 Y모양 {sl['yellow_shape']} | 바코드 {bc['text']} 중심 mm ({bc['center'][0]*1000:.1f}, {bc['center'][1]*1000:.0f}, {bc['center'][2]*1000:.0f}) | 할당 {alloc}")
