"""F0 도구 확인: 팀 TensorRT 검출기(YOLO 후보 B)가 B0′ 판정 렌더의 D435 영상에서 퍽을 잡는지 본다.
정답 퍽 중심(render_b0.json truth)을 카메라로 투영해, 그 점을 품은 검출 상자가 있으면 잡음, 상자 클래스가 색 · 모양과 같으면 맞음.
보임(visible) 0.5 이상인 퍽만 셈. 어느 정답 중심도 품지 않은 상자는 헛검출.

실행 (PC): python3 -I eval_f0.py <trt_team.jsonl> <렌더 폴더 ...> > summary.json
"""
import json
import os
import sys

import numpy as np

NAMES = ['red_cylinder', 'green_cylinder', 'red_cuboid', 'green_cuboid']


def project(T_world_cam, K, p):
    pc = np.linalg.inv(np.array(T_world_cam)) @ np.array([*p, 1.0])
    if pc[2] <= 0.05:
        return None
    return K['fx'] * pc[0] / pc[2] + K['cx'], K['fy'] * pc[1] / pc[2] + K['cy']


def main():
    dets = {}
    for line in open(sys.argv[1]):
        r = json.loads(line)
        dets[os.path.realpath(r['image'])] = r['dets']
    total = {'frames': 0, 'expected': 0, 'found': 0, 'class_ok': 0, 'false_boxes': 0, 'dup_boxes': 0}
    renders, misses, wrong = {}, [], []
    for rd in sys.argv[2:]:
        meta = json.load(open(os.path.join(rd.removesuffix('_awb'), 'render_b0.json')))
        truth = next(iter(meta['truth'].values()))
        by_name = {t['name']: t for t in truth}
        K = meta['d435']['K']
        s = {k: 0 for k in total}
        for p in meta['poses']:
            if p['kind'] != 'd435':
                continue
            img = os.path.realpath(os.path.join(rd, p['images']['color']))
            ds = dets[img]
            s['frames'] += 1
            used = [0] * len(ds)
            for t in truth:
                uv = project(p['T_world_cam'], K, t['center'])
                if uv is None:
                    continue
                hit = [i for i, d in enumerate(ds) if d['box'][0] <= uv[0] <= d['box'][2] and d['box'][1] <= uv[1] <= d['box'][3]]
                for i in hit:
                    used[i] += 1
                if p['visible'].get(t['name'], 0) < 0.5:
                    continue
                s['expected'] += 1
                want = f"{t['color']}_{t['shape']}"
                if not hit:
                    misses.append({'image': img, 'puck': t['name'], 'want': want, 'standing': t['standing'],
                                   'visible': p['visible'][t['name']], 'uv': [round(uv[0], 1), round(uv[1], 1)]})
                    continue
                s['found'] += 1
                best = max(hit, key=lambda i: ds[i]['score'])
                got = NAMES[ds[best]['class']]
                if got == want:
                    s['class_ok'] += 1
                else:
                    wrong.append({'image': img, 'puck': t['name'], 'want': want, 'got': got, 'standing': t['standing'],
                                  'score': round(ds[best]['score'], 3)})
                s['dup_boxes'] += len(hit) - 1
            s['false_boxes'] += sum(1 for u in used if u == 0)
        renders[os.path.basename(rd)] = s
        for k in total:
            total[k] += s[k]
    total['recall'] = total['found'] / max(total['expected'], 1)
    total['class_acc_of_found'] = total['class_ok'] / max(total['found'], 1)
    print(json.dumps({'total': total, 'renders': renders, 'misses': misses, 'wrong_class': wrong}, ensure_ascii=False, indent=1))


if __name__ == '__main__':
    main()
