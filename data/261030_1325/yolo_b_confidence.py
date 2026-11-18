"""후보 B 평가 행(yolo_evaluation/B_negative_*_rows.json)에서 confidence 분포를 맞힌 상자(TP) · 헛검출(FP)로 나눠 집계(도구, 판정 아님, 11-02).
짝 = 같은 클래스 · IoU ≥ 0.5 · 점수 높은 순 탐욕(evaluate_yolo.py 와 같은 방식), 상자는 conf ≥ 0.25 만.
사용: python3 yolo_b_confidence.py → 같은 폴더 yolo_b_confidence.json, photo/261102_YOLO_B_점수_confidence.png(matplotlib 있으면)"""
import json
from pathlib import Path

import numpy as np

R = Path('/home/pa4/git/lv2_next_module/Cartlidar/sim_render/results/followup_261028_1457/improvement_261030_1325')
NAMES = ['red_cylinder', 'green_cylinder', 'red_cuboid', 'green_cuboid']


def iou(a, b):
    ix = max(0.0, min(a[2], b[2]) - max(a[0], b[0])); iy = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    u = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - ix * iy
    return ix * iy / u if u > 0 else 0.0


def collect(rows):
    tp, fp = {n: [] for n in NAMES}, {n: [] for n in NAMES}
    for r in rows:
        used = set()
        for d in sorted((d for d in r['detections'] if d['score'] >= 0.25), key=lambda d: -d['score']):
            best, bi = 0.5, None
            for i, g in enumerate(r['gt']):
                if i in used or g['class'] != d['class']:
                    continue
                v = iou(d['box'], g['box'])
                if v >= best:
                    best, bi = v, i
            if bi is None:
                fp[NAMES[d['class']]].append(d['score'])
            else:
                used.add(bi); tp[NAMES[d['class']]].append(d['score'])
    return tp, fp


st = lambda a: {'n': len(a), **({k: round(float(np.percentile(a, p)), 3) for k, p in (('min', 0), ('p5', 5), ('p50', 50), ('max', 100))} if a else {})}
out = {}
for split, f in (('real_eval_203', 'B_negative_eval_rows.json'), ('sim_eval_240', 'B_negative_sim_eval_rows.json')):
    tp, fp = collect(json.load(open(R / 'yolo_evaluation' / f)))
    out[split] = {'tp': {n: st(v) for n, v in tp.items()}, 'fp': {n: st(v) for n, v in fp.items()},
                  'tp_all': st(sum(tp.values(), [])), 'fp_all': st(sum(fp.values(), [])), '_tp': tp, '_fp': fp}
json.dump({k: {kk: vv for kk, vv in v.items() if not kk.startswith('_')} for k, v in out.items()}, open(Path(__file__).with_suffix('.json'), 'w'), ensure_ascii=False, indent=1)
for k, v in out.items():
    print(k, 'TP', v['tp_all'], 'FP', v['fp_all'])
    for n in NAMES:
        print('  ', n, 'TP', v['tp'][n], 'FP', v['fp'][n])
try:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
except ImportError:
    raise SystemExit
from matplotlib import font_manager
font_manager.fontManager.addfont('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
plt.rcParams['font.family'] = font_manager.FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc').get_name()
fig, ax = plt.subplots(1, 2, figsize=(12, 4.5))
for a, (k, title) in zip(ax, (('real_eval_203', '실물 평가 203장(앞 D435)'), ('sim_eval_240', '합성 평가 240장'))):
    v = out[k]
    data = [v['_tp'][n] for n in NAMES] + [sum(v['_fp'].values(), [])]
    a.boxplot(data, tick_labels=[f'{n}\nTP {len(v["_tp"][n])}' for n in NAMES] + [f'헛검출\nFP {len(data[-1])}'], showfliers=True)
    a.axhline(0.25, color='gray', ls='--', lw=1); a.text(0.6, 0.265, 'score 문턱 0.25', fontsize=8, color='gray')
    a.set_ylim(0, 1.02); a.set_ylabel('confidence'); a.set_title(title); a.tick_params(axis='x', labelsize=8)
fig.suptitle('YOLO 후보 B(yolo11n_task4_B) confidence — 맞힌 상자(TP) · 헛검출(FP), IoU ≥ 0.5')
fig.tight_layout()
fig.savefig('/home/pa4/git/lv2_next_module/photo/261102_YOLO_B_점수_confidence.png', dpi=120)
print('그림 저장')
