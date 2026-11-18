"""B4 채점 요약: evaluate.json(G1~G5, RMSE) + loc_events.jsonl(SUSPECT · LOST 횟수, 시각, 잔차 최대). 11-03 두 판도 비교로 같이 찍음."""
import json
from pathlib import Path
H = Path(__file__).resolve().parent
OLD = Path('/home/pa4/git/lv2_next_module/261103_1502_loc_pkg/results')
runs = [('11-03 옛 팀 EKF', OLD / 'K0_pkg_amcl'), ('11-03 우리 ICR', OLD / 'K0_pkg_icr_amcl')] + [(p.name, p) for p in sorted((H / 'results').glob('K0_dev_*')) if p.is_dir()]
print('판 | G1 G2 G3 G4 G5 | RMSE 최대 | G3 여유 | SUSPECT LOST 그밖 | 잔차 최대 | 나쁜 상태 들어간 시각(s) | isaac')
for name, p in runs:
    e = json.load(open(p / 'evaluate.json')) if (p / 'evaluate.json').exists() else {}
    ev = [json.loads(l) for l in open(p / 'loc_events.jsonl')] if (p / 'loc_events.jsonl').exists() else []
    t0 = ev[0]['t'] if ev else 0
    st = [x for x in ev if x.get('event') == 'state']
    bad = [x for x in st if x['new'] != 'OK']
    cnt = lambda s: sum(x['new'] == s for x in st)
    resmax = max([x.get('res_max', 0) for x in ev if x.get('event') == 'report'] or [0])
    other = sorted({x['event'] for x in ev} - {'start', 'report', 'state'})
    ex = (p / 'exit_codes.txt').read_text().split('isaac_exit ')[-1].split()[0] if (p / 'exit_codes.txt').exists() else '?'
    print(f"{name} | {e.get('G1_succeeded')} {e.get('G2_end_error')} {e.get('G3_pass')} {e.get('G4_pass')} {e.get('G5_pass')} | {e.get('G4_loc_rmse_m')} {e.get('G4_loc_max_m')} | {e.get('G3_min_clearance_m')} | {cnt('SUSPECT')} {cnt('LOST')} {len(bad) - cnt('SUSPECT') - cnt('LOST')} | {resmax} | {[round(x['t'] - t0, 1) for x in bad]} | {ex} {other}")
