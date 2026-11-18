"""PC: Jetson 에서 가져온 results/(dets_J32 · dets_J16 · summary.json)로 FP16 확인 기준 K1~K3 를 계산한다(11-04).
K1 같은 장 검출 일치 · K2 영상 판정(judge.py, D 고정 때와 같은 설정) · K3 속도. 결과 → fp16_result.json 과 화면."""
import json, os, subprocess, shutil
from pathlib import Path

HERE = Path(__file__).resolve().parent; R = HERE / 'results'; V = Path('/home/pa4/git/lv2_next_module/261103_1053_video_eval')
LAB = Path('/home/pa4/git/lv2_next_module/261030_1325/B_negative/labels/eval')
# 꾸러미의 eval203 은 심볼릭 링크 원본 이름(…_color.png)으로 복사됨 → B_negative 이름(…_rep0)으로 되돌림
EVN = {Path(os.path.realpath(f)).name: f.stem for f in Path(str(LAB).replace('/labels/', '/images/')).glob('*.png')}
J32, J16 = json.load(open(R / 'dets_J32.json')), json.load(open(R / 'dets_J16.json'))
S = json.load(open(R / 'summary.json'))


def iou(a, b):
    ix = max(0.0, min(a[2], b[2]) - max(a[0], b[0])); iy = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    u = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - ix * iy
    return ix * iy / u if u > 0 else 0.0


# K1: 0.6 이상 검출이 장마다 같은가(수 · 클래스 같고 짝 IoU ≥ 0.9). 점수 차도 기록
same, diff, dscore = 0, [], []
for k in J32:
    a = [d for d in J32[k] if d[1] >= 0.6]; b = [d for d in J16[k] if d[1] >= 0.6]; used = set(); ok = len(a) == len(b)
    for d in a:
        m = max(((iou(d[2:], e[2:]), j) for j, e in enumerate(b) if j not in used and e[0] == d[0]), default=(0, None))
        if m[0] >= 0.9:
            used.add(m[1]); dscore.append(abs(d[1] - b[m[1]][1]))
        else:
            ok = False
    same += ok
    if not ok: diff.append(k)
k1 = {'images': len(J32), 'same': same, 'ratio': round(same / len(J32), 4), 'diff_images': diff,
      'score_diff_max': round(max(dscore), 4), 'score_diff_median': round(sorted(dscore)[len(dscore) // 2], 5), 'pass': same / len(J32) >= 0.98}

# K2: 영상 판정(D 고정 때와 같은 설정: 쌓인 퍽 뺌, 뒤는 있다/없다) + 옛 실물 평가 203장 @0.6
for n in ('J32', 'J16'):
    shutil.copy(R / f'dets_{n}.json', V / f'dets_{n}.json')
env = dict(os.environ, JUDGE_STACKED='stacked.json', JUDGE_MIN_SHARP='0', JUDGE_AGNOSTIC='judge_rear,video_rear',
           JUDGE_CAMS='judge_front,judge_rear,video_front,video_rear,video_arm', JUDGE_OUT='judge_fp16.json')
subprocess.run(['/home/pa4/yolo-venv/bin/python', 'judge.py', 'all_D', 'J32', 'J16'], cwd=V, env=env, check=True, capture_output=True)
J = json.load(open(V / 'judge_fp16.json'))
cams = ['judge_front', 'judge_rear', 'video_front', 'video_rear', 'video_arm']
k2 = {n: {c: [J[n]['0.6'][c]['recall'], J[n]['0.6'][c]['precision']] for c in cams} for n in ('all_D', 'J32', 'J16')}
ev = {}
for n, D in (('J32', J32), ('J16', J16)):
    tp = fp = gt_n = 0
    for k in [k for k in D if k.startswith('eval203/')]:
        import cv2
        h, w = cv2.imread(str(HERE / 'pkg/images' / k)).shape[:2]
        G = []
        for ln in (LAB / (EVN[Path(k).name] + '.txt')).read_text().split('\n'):
            p = ln.split()
            if p:
                c, x, y, bw, bh = int(p[0]), *map(float, p[1:]); G.append((c, [(x - bw / 2) * w, (y - bh / 2) * h, (x + bw / 2) * w, (y + bh / 2) * h]))
        gt_n += len(G); used = set()
        for d in sorted([d for d in D[k] if d[1] >= 0.6], key=lambda d: -d[1]):
            m = max(((iou(d[2:], g[1]), j) for j, g in enumerate(G) if j not in used and g[0] == d[0]), default=(0, None))
            if m[0] >= 0.5: used.add(m[1]); tp += 1
            else: fp += 1
    ev[n] = {'targets': gt_n, 'tp': tp, 'fp': fp}
d = {c: [round(k2['J16'][c][i] - k2['J32'][c][i], 4) for i in (0, 1)] for c in cams}
k2_pass = all(abs(x) <= 0.01 for v in d.values() for x in v) and min(k2['J16']['video_arm']) >= 0.95 and \
    abs(ev['J16']['tp'] - ev['J32']['tp']) <= 2 and abs(ev['J16']['fp'] - ev['J32']['fp']) <= 2

# K3: 속도
E = S['engines']
k3 = {n: {'trtexec_median_ms': E[n]['trtexec_gpu_ms']['median'] if E[n]['trtexec_gpu_ms'] else None,
          'python_infer_median_ms': E[n]['python_ms']['infer']['median'], 'python_total_median_ms':
          round(sum(E[n]['python_ms'][x]['median'] for x in ('letterbox', 'infer', 'decode')), 3)} for n in ('J32', 'J16')}
ratio = k3['J16']['python_infer_median_ms'] / k3['J32']['python_infer_median_ms']
out = {'K1': k1, 'K2': {'recall_precision@0.6': k2, 'J16_minus_J32': d, 'eval203@0.6': ev, 'pass': k2_pass},
       'K3': {**k3, 'infer_ratio_J16_J32': round(ratio, 3), 'pass': ratio <= 0.7}, 'board': S['board']}
json.dump(out, open(HERE / 'fp16_result.json', 'w'), ensure_ascii=False, indent=1)
print(json.dumps({k: (v if k != 'K1' else {x: v[x] for x in v if x != 'diff_images'}) for k, v in out.items()}, ensure_ascii=False, indent=1))
