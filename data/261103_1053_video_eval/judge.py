"""영상 판정(261102_1634_yolo_D/README.md '영상 판정 기준', 사용자 11-02 결정). dets_<모델>.json + 사용자 고친 라벨 → 카메라별 재현율 · 정밀도.
- D435: 정답 상자 깊이 = 상자 가운데 절반 안 depth_aligned 유효 화소 중앙값 × depth_unit_m. 3.0 m 넘거나 깊이 없음 = 셈 밖.
  1.0 m 안은 클래스(색 + 모양)가 맞아야 맞힘, 1~3 m 는 색만 맞으면 맞힘(작게 찍힌 퍽은 가까이 가서 모양 확인 — 사용자 규칙).
  짝 없는 검출: 셈 밖 정답과 IoU ≥ 0.5 이면 무시, 아니면 그 검출 상자 깊이가 3.0 m 넘거나 없으면 무시, 그 밖은 헛검출.
- Arducam: 영상(묶음)마다 첫 장 정답 중 가장 큰 상자의 긴 변 = s0(목표 퍽 0.5 m). 긴 변 < s0 인 정답 = 셈 밖, 짝 없는 검출도 긴 변 < s0 면 무시.
  셈 안은 클래스까지 맞아야 맞힘.
- 짝: 점수 높은 순, IoU ≥ 0.5, 조건 맞는 정답 중 IoU 최대. 클래스가 틀린 검출은 헛검출(정답은 놓침으로 남음).
사용: judge.py [B C D] → judge.json + 표 출력."""
import json, sys
from collections import defaultdict
from pathlib import Path
import cv2, numpy as np

HERE = Path(__file__).resolve().parent
TH = (0.25, 0.5, 0.6); NEAR, FAR = 1.0, 3.0
import os
CAMS = tuple(os.environ.get('JUDGE_CAMS', 'video_front,video_rear,video_arm').split(','))
OUT_JSON = os.environ.get('JUDGE_OUT', 'judge.json')
AGN = set(filter(None, os.environ.get('JUDGE_AGNOSTIC', '').split(',')))  # 이 카메라는 '있다/없다'만(클래스 · 색 무관, 사용자 11-03: 뒤 D435)
STK = json.load(open(HERE / os.environ['JUDGE_STACKED'])) if os.environ.get('JUDGE_STACKED') else {}  # 쌓인 퍽 정답 번호(stacked.py) → 셈 밖(사용자 11-03: 과제에 쌓인 퍽 없음)
MIN_SHARP = float(os.environ.get('JUDGE_MIN_SHARP', '0'))  # 라플라시안 분산이 이보다 낮은(흐린) 장은 뺌(0 = 안 뺌)


def iou(a, b):
    ix = max(0.0, min(a[2], b[2]) - max(a[0], b[0])); iy = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    u = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - ix * iy
    return ix * iy / u if u > 0 else 0.0


def box_depth(dep, b, unit):
    h, w = dep.shape; x0, y0, x1, y1 = b; cx, cy, bw, bh = (x0 + x1) / 2, (y0 + y1) / 2, (x1 - x0) / 4, (y1 - y0) / 4
    p = dep[max(0, int(cy - bh)):min(h, int(cy + bh) + 1), max(0, int(cx - bw)):min(w, int(cx + bw) + 1)]
    p = p[p > 0]
    return float(np.median(p)) * unit if p.size else None


# 정답 읽기(카메라 · 장마다), D435 깊이 · Arducam s0
GT, META, DUP = {}, {}, []
for cam in CAMS:
    L = json.load(open(HERE / cam / 'auto_label.json')); first = {}
    for e in L:
        img = cv2.imread(str(HERE / cam / 'images' / e['image'])); H, W = img.shape[:2]
        g = []
        for l in (HERE / cam / 'labels' / (e['image'][:-4] + '.txt')).read_text().split('\n'):
            if l.strip():
                k, x, y, bw, bh = l.split(); x, y, bw, bh = float(x) * W, float(y) * H, float(bw) * W, float(bh) * H
                g.append({'c': int(k), 'cs': {int(k)}, 'b': [x - bw / 2, y - bh / 2, x + bw / 2, y + bh / 2]})
        for i in STK.get(f'{cam}/{e["image"]}', []):
            g[i]['stk'] = True
        # 같은 퍽에 거의 같은 상자 둘(IoU ≥ 0.9, 사용자 라벨 실수) → 정답 하나로 세고, 두 클래스 중 어느 쪽이든 맞힘(11-03 13:4x)
        for i in range(len(g) - 1, 0, -1):
            j = next((j for j in range(i) if iou(g[i]['b'], g[j]['b']) >= 0.9), None)
            if j is not None:
                g[j]['cs'] |= g[i]['cs']; g.pop(i); DUP.append(f'{cam}/{e["image"]}')
        m = {'batch': e['batch'], 'W': W, 'H': H, 'sharp': cv2.Laplacian(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), cv2.CV_64F).var()}
        if e.get('depth'):
            dep = cv2.imread(e['depth'], cv2.IMREAD_UNCHANGED); unit = json.load(open(Path(e['depth']).parent.parent / 'capture.json'))['calibration']['depth_unit_m']
            m['dep'], m['unit'] = dep, unit
            for x in g:
                x['z'] = box_depth(dep, x['b'], unit)
        GT[f'{cam}/{e["image"]}'] = g; META[f'{cam}/{e["image"]}'] = m
        if cam.endswith('_arm') and e['batch'] not in first:
            first[e['batch']] = max((max(x['b'][2] - x['b'][0], x['b'][3] - x['b'][1]) for x in g), default=None)
    if cam.endswith('_arm'):
        META['s0'] = first
print('겹친 정답 하나로 셈:', len(DUP), sorted(set(DUP)))
print('Arducam s0(px):', {k: round(v, 1) if v else v for k, v in META.get('s0', {}).items()})

side = lambda b: max(b[2] - b[0], b[3] - b[1])
rep = {}
for name in (sys.argv[1:] or ['B', 'C', 'D']):
    D = json.load(open(HERE / f'dets_{name}.json'))
    for th in TH:
        S = defaultdict(lambda: defaultdict(int))
        for key, g in GT.items():
            cam = key.split('/')[0]; m = META[key]
            if m['sharp'] < MIN_SHARP: continue
            grp = (cam, cam + ':' + m['batch'])
            if cam.endswith('_arm'):
                s0 = META['s0'][m['batch']]
                for x in g: x['in'] = s0 is not None and side(x['b']) >= s0 - 1e-6; x['near'] = True
            else:
                for x in g: x['in'] = x['z'] is not None and x['z'] <= FAR and not x.get('stk'); x['near'] = x['z'] is not None and x['z'] <= NEAR
            dets = sorted([d for d in D[key] if d[1] >= th], key=lambda d: -d[1]); used = set()
            for gk in grp:
                S[gk]['targets'] += sum(x['in'] for x in g); S[gk]['near'] += sum(x['in'] and x['near'] for x in g)
            for c, s, *b in dets:
                ok = [(iou(b, x['b']), j) for j, x in enumerate(g) if j not in used and iou(b, x['b']) >= 0.5 and
                      (not x['in'] or cam in AGN or (c in x['cs'] if x['near'] else c % 2 in {k % 2 for k in x['cs']}))]
                if ok:
                    j = max(ok)[1]; used.add(j)
                    if g[j]['in']:
                        for gk in grp: S[gk]['tp'] += 1; S[gk]['tp_near'] += g[j]['near']
                    continue
                if cam.endswith('_arm'):
                    ignore = side(b) < META['s0'][m['batch']] - 1e-6 if META['s0'][m['batch']] else True
                else:
                    z = box_depth(m['dep'], b, m['unit']); ignore = z is None or z > FAR
                    ignore = ignore or any(not x['in'] and iou(b, x['b']) >= 0.5 for x in g)
                if not ignore:
                    for gk in grp: S[gk]['fp'] += 1
        for gk, s in S.items():
            r = dict(s); r['fn'] = s['targets'] - s['tp']
            r['recall'] = round(s['tp'] / s['targets'], 4) if s['targets'] else None
            r['precision'] = round(s['tp'] / (s['tp'] + s['fp']), 4) if s['tp'] + s['fp'] else None
            r['recall_near'] = round(s['tp_near'] / s['near'], 4) if s['near'] else None
            rep.setdefault(name, {}).setdefault(str(th), {})[gk] = r
json.dump(rep, open(HERE / OUT_JSON, 'w'), ensure_ascii=False, indent=1)
for th in ('0.6',):
    for cam in CAMS:
        print(f'== {cam} @{th}')
        for name in rep:
            r = rep[name][th].get(cam, {})
            ok = r.get('recall') is not None and r['recall'] >= 0.95 and (r.get('precision') or 0) >= 0.95
            print(f"  {name}: 대상 {r.get('targets')}(1 m 안 {r.get('near')}) 맞힘 {r.get('tp')} 헛 {r.get('fp')} 놓침 {r.get('fn')} | 재현율 {r.get('recall')} 정밀도 {r.get('precision')} 1 m 안 재현율 {r.get('recall_near')} | {'통과' if ok else '불통과'}")
