"""replay_amcl 채점: 원본 실행의 첫 목표 보낸 시각(run_goals.json goals[0].t_start)부터 끝까지, 위치 오차 RMSE · 최대 · 95 백분위, yaw 오차, 빈칸 수.
라이브 G4(run_goals 의 loc, 10 Hz)와 같은 뜻."""
import csv
import json
import math
import sys
from pathlib import Path

out, src = Path(sys.argv[1]), Path(sys.argv[2])
t0 = json.loads((src / 'run_goals.json').read_text())['goals'][0]['t_start']
pe, ye, miss = [], [], 0
for r in csv.DictReader(open(out / 'loc_eval.csv')):
    if float(r['t']) < t0:
        continue
    if r['ex'] == '':
        miss += 1
        continue
    pe.append(math.hypot(float(r['ex']) - float(r['tx']), float(r['ey']) - float(r['ty'])))
    d = float(r['eyaw']) - float(r['tyaw'])
    ye.append(abs(math.atan2(math.sin(d), math.cos(d))))
pe_s = sorted(pe)
res = {'n': len(pe), 'miss': miss,
       'rmse': round(math.sqrt(sum(e * e for e in pe) / len(pe)), 4) if pe else None,
       'max': round(max(pe), 4) if pe else None, 'p95': round(pe_s[int(0.95 * (len(pe_s) - 1))], 4) if pe else None,
       'yaw_rmse': round(math.sqrt(sum(e * e for e in ye) / len(ye)), 4) if ye else None, 'yaw_max': round(max(ye), 4) if ye else None}
print(json.dumps(res))
