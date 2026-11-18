"""D435 찾기 시제품 평가: 장면마다 실제 있는 라벨 모두(옮긴 prim 의 원래 자리는 뺌)를 정답으로."""
import json, numpy as np, cv2, sys, collections
sys.path.insert(0, '.')
from locate_proto import locate, CFG
from pathlib import Path
root = Path(sys.argv[1])
rep = json.loads((root / 'render_r3.json').read_text()); K = rep['d435']['K']
labs = {lb['id']: lb for lb in rep['labels']}
res = collections.Counter(); errs = []; rows = []
for p in rep['poses']:
    if p['kind'] != 'd435': continue
    cur = labs[p['label']]
    present = [cur['center']] + [lb['center'] for lb in rep['labels'] if lb['world'] == cur['world'] and lb['source'] == 'world'
                                 and lb['prim'] != (cur['prim'] if cur['source'] == 'moved' else None) and lb['id'] != cur['id']]
    bgr = cv2.imread(str(root / p['images']['color'])); dep = np.load(root / p['images']['depth'])
    T = np.array(p['T_world_cam']); Ti = np.linalg.inv(T)
    found = locate(bgr, dep, K, T_base_cam=T)
    tr = (Ti @ np.append(cur['center'], 1))[:3]
    hit = [np.linalg.norm(f['center'] - tr) for f in found if np.linalg.norm(f['center'] - tr) <= 0.05]
    for f in found:
        w = (T @ np.append(f['center'], 1))[:3]
        d = min(np.linalg.norm(w - np.array(c)) for c in present)
        rows.append((f['kind'], round(f['stripe'], 2), d <= 0.05, p['name'], [round(x, 3) for x in w], round(f['L'] * 1000), round(f['S'] * 1000)))
        if d > 0.05: res['false'] += 1
    judged = p['label_visible'] >= 0.95 and p['base_hdist_m'] <= 1.0
    if judged:
        res[(cur['where'], 'n')] += 1
        if hit: res[(cur['where'], 'hit')] += 1; errs.append(min(hit))
        if hit and min(hit) > 0.03: res['err>30'] += 1
print(dict(res), '오차 최대 mm', round(max(errs) * 1000, 1) if errs else None)
if '-v' in sys.argv:
    for r in rows:
        if not r[2]: print('헛', r)
    st = collections.defaultdict(list)
    for r in rows: st[(r[0], r[2])].append(r[1])
    for k, v in st.items(): print(k, len(v), 'stripe 분위 10/50/90', np.percentile(v, [10, 50, 90]).round(2))
