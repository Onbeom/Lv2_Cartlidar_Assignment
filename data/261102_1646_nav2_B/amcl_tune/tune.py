"""AMCL 설정 후보를 여러 녹화에 오프라인 재생(replay_amcl.sh)으로 돌려 점수를 모은다(도구, 판정 아님).
사용: python3 tune.py <라운드 이름> <녹화 묶음 tune|hold> <rate> <동시 수> '<이름>:키=값 키=값' ...  (빈 덮어쓰기 = 'base:')
결과: runs/<라운드>_<이름>_<녹화>/score.json, 요약 rounds/<라운드>.json"""
import json
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

T = Path(__file__).resolve().parent
R = T.parent / 'results'
SETS = {'tune': {'amcl': R / 'live_nav_v45B3amcl_s20261101', 'c_s1': R / 'live_nav_v45B3_s1', 'c_s2': R / 'live_nav_v45B3_s2'},
        'hold': {'c_s2026': R / 'live_nav_v45B3_s20261101', 'k0': R / 'K0_loc2_v45B'}}
rnd, bags, rate, par = sys.argv[1], SETS[sys.argv[2]], sys.argv[3], int(sys.argv[4])
cands = []
for c in sys.argv[5:]:
    name, kv = c.split(':', 1)
    cands.append((name, kv.split()))
(T / 'params').mkdir(exist_ok=True)
(T / 'rounds').mkdir(exist_ok=True)
jobs = []
for name, kv in cands:
    pf = T / 'params' / f'{rnd}_{name}.yaml'
    subprocess.run(['python3', str(T / 'mkparams.py'), str(pf)] + kv, check=True)
    for b, src in bags.items():
        jobs.append((name, b, pf, src))


def run(i_job):
    i, (name, b, pf, src) = i_job
    out = T / 'runs' / f'{rnd}_{name}_{b}'
    if not (out / 'score.json').exists() or (out / 'score.json').stat().st_size == 0:
        subprocess.run(['bash', str(T / 'replay_amcl.sh'), str(out), str(src), str(pf), rate, str(110 + i % par)],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        return name, b, json.loads((out / 'score.json').read_text())
    except Exception as e:
        return name, b, {'error': str(e)}


with ThreadPoolExecutor(par) as ex:
    res = list(ex.map(run, enumerate(jobs)))
summ = {}
for name, b, s in res:
    summ.setdefault(name, {})[b] = s
for name, d in summ.items():
    ok = [v for v in d.values() if v.get('rmse') is not None]
    d['mean_rmse'] = round(sum(v['rmse'] for v in ok) / len(ok), 4) if ok and len(ok) == len(bags) else None
    d['max_max'] = max(v['max'] for v in ok) if ok and len(ok) == len(bags) else None
(T / 'rounds' / f'{rnd}.json').write_text(json.dumps({'cands': dict(cands), 'res': summ}, ensure_ascii=False, indent=1))
for name in sorted(summ, key=lambda n: summ[n]['mean_rmse'] or 9):
    d = summ[name]
    print(name, d['mean_rmse'], d['max_max'], {b: (d[b].get('rmse'), d[b].get('max'), d[b].get('miss')) for b in bags})
