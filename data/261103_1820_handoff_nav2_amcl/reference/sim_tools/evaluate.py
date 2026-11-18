"""Nav2 모의 확인 채점(기준은 README, 사용자 승인 2026-10-31):
 G1 목표마다 SUCCEEDED, G2 끝 참값 위치 오차 ≤ 0.10 m · yaw ≤ 0.15 rad, G3 주행 내내 footprint 여유 > 0(참값 기준, 벽 · 모든 고정 물체),
 G4 위치 추정 오차 RMSE ≤ 0.05 m · 최대 ≤ 0.15 m(첫 목표를 보낸 뒤부터), G5 /cmd_vel_nav frame base_link · |vx| ≤ 0.20 · |wz| ≤ 0.8(+1 % 반올림 여유)."""
import argparse
import csv
import json
import math
from pathlib import Path

ap = argparse.ArgumentParser()
ap.add_argument('--run', required=True)
a = ap.parse_args()
run = Path(a.run)
rg = json.loads((run / 'run_goals.json').read_text())
goals = rg['goals']
rows = list(csv.DictReader(open(run / 'truth.csv')))
t_first = goals[0]['t_start'] if goals else 0.0
clear = [float(r['clear_body']) for r in rows if float(r['t']) >= t_first]
loc = [r for r in rg.get('loc', []) if r[1] >= t_first]
pe = [r[2] for r in loc]
ye = [r[3] for r in loc]
cmd = rg.get('cmd', {})
res = {
    'n_goals': len(goals),
    'G1_succeeded': all(g.get('status') == 'SUCCEEDED' for g in goals) and len(goals) > 0,
    'G2_end_error': all(g.get('pos_err', 9) <= 0.10 and g.get('yaw_err', 9) <= 0.15 for g in goals) and len(goals) > 0,
    'G3_min_clearance_m': round(min(clear), 4) if clear else None,
    'G3_n_overlap_samples': sum(c < 0 for c in clear),
    'G4_loc_rmse_m': round(math.sqrt(sum(e * e for e in pe) / len(pe)), 4) if pe else None,
    'G4_loc_max_m': round(max(pe), 4) if pe else None,
    'G4_yaw_rmse_rad': round(math.sqrt(sum(e * e for e in ye) / len(ye)), 4) if ye else None,
    'G4_n_samples': len(pe),
    'G5_cmd': cmd,
    'goals': [{k: g.get(k) for k in ('name', 'status', 'duration_s', 'pos_err', 'yaw_err', 'error_code', 'error_msg')} for g in goals],
}
res['G3_pass'] = res['G3_min_clearance_m'] is not None and res['G3_min_clearance_m'] > 0
res['G4_pass'] = res['G4_loc_rmse_m'] is not None and res['G4_loc_rmse_m'] <= 0.05 and res['G4_loc_max_m'] <= 0.15
res['G5_pass'] = cmd.get('n', 0) > 0 and cmd.get('frames') == ['base_link'] and cmd.get('vx_max', 9) <= 0.202 and cmd.get('vx_min', -9) >= -0.202 \
    and cmd.get('wz_abs_max', 9) <= 0.808
res['pass'] = all(res[k] for k in ('G1_succeeded', 'G2_end_error', 'G3_pass', 'G4_pass', 'G5_pass'))
(run / 'evaluate.json').write_text(json.dumps(res, ensure_ascii=False, indent=1))
print(json.dumps(res, ensure_ascii=False, indent=1))
