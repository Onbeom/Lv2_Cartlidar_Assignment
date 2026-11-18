"""2단계 채점 준비: 장면 truth.csv(USD 좌표, 50 Hz) → Nav2 채점(261031_1325_nav2/test/evaluate.py)이 읽는 <결과>/truth.csv
(t, x, y, yaw 발제 좌표, clear_body = 참값 footprint 와 벽 · 모든 고정 물체(배치 seed_1) 사이 여유, 2D 모의기와 같은 geom.clearance).
Isaac 장면에는 벽만 있고 물체가 없으므로 물체와의 여유는 몸이 실제로 닿는지가 아니라 '닿았을 자리'로 셈. 사용: python3 make_truth_nav.py <결과 폴더>"""
import csv
import sys
from pathlib import Path

sys.path.insert(0, '/home/pa4/git/lv2_next_module/261031_1325_nav2/nodes')
import geom  # noqa: E402

OUT = Path(sys.argv[1])
ox, oy = geom.USD_OFFSET
body = geom.load_world()['body']
rows = list(csv.DictReader(open(OUT / 'scene' / 'truth.csv')))
with open(OUT / 'truth.csv', 'w', newline='') as f:
    w = csv.writer(f)
    w.writerow(['t', 'x', 'y', 'yaw', 'cmd_v', 'cmd_w', 'clear_body'])
    for r in rows:
        x, y, yaw = float(r['x']) + ox, float(r['y']) + oy, float(r['yaw'])
        w.writerow([r['sim_s'], f'{x:.4f}', f'{y:.4f}', f'{yaw:.4f}', r['cmd_v'], r['cmd_w'], f'{geom.clearance(x, y, yaw, body):.4f}'])
print(OUT / 'truth.csv', len(rows))
