"""도구 확인용(판정 아님): 목표 파일의 각 구간 출발(앞 목표, 첫 구간은 출발 자세)을 ±dxy m · ±dyaw rad 고르게 흔든 출발 n 개를 만들고,
'흔든 출발 → 목표' 순서로 늘어놓은 목표 파일과 그 구간 번호(plan_only.py --legs)를 낸다. 11-03 손으로 만든 goals_B2_perturb_all.json 과 같은 꼴.
사용: python3 make_perturb_goals.py <goals.json> <out.json> [n 5] [seed 0]"""
import json
import math
import random
import sys

src, out = sys.argv[1], sys.argv[2]
n = int(sys.argv[3]) if len(sys.argv) > 3 else 5
random.seed(int(sys.argv[4]) if len(sys.argv) > 4 else 0)
DXY, DYAW = 0.03, 0.06
goals = json.load(open(src))['goals']
prev = {'x': 0.734, 'y': 0.95, 'yaw': math.pi}
seq, legs = [], []
for i, g in enumerate(goals, 1):
    for k in range(n):
        seq.append({'name': f'P{i}_{k}', 'x': round(prev['x'] + random.uniform(-DXY, DXY), 4), 'y': round(prev['y'] + random.uniform(-DXY, DXY), 4),
                    'yaw': round(prev['yaw'] + random.uniform(-DYAW, DYAW), 4)})
        legs.append(len(seq))          # pts[0] = start → pts[j] = seq[j-1]; 구간 j = pts[j] → pts[j+1] = 흔든 출발 → 목표
        seq.append(g)
    prev = g
json.dump({'goals': seq}, open(out, 'w'), ensure_ascii=False, indent=1)
print(' '.join(map(str, legs)))
