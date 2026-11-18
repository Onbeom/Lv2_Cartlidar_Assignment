#!/usr/bin/env python3
"""사람이 볼 퍽을 고른다(11-09 사용자 승인 안): 태그 있는데 자동 통과 못 한 것 전부 + 태그 없는 퍽 무작위 표본.
상태(status)를 지금 auto_label3d 기준으로 다시 매기고, 고른 줄에 review=True 를 붙인다. 이미 사람이 본 줄(human)은 그대로 둔다.
    pick_review.py labels_train3.json labels_final.json [--n_notag 100] [--seed 0]"""
import argparse, json, os, random, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))
import auto_label3d as A

def pick(path, n_notag, seed):
    J = json.load(open(path)); fx = J['K'][0][0]; R = J['rows']
    for r in R:
        if 'human' in r: continue
        r['status'] = 'fail' if 'c' not in r else ('no_tag' if r['n_tags'] == 0 else ('auto' if r.get('sil_iou', 0) >= A.OK_SIL and A.tag_mm(r, fx) <= A.OK_TAG_MM and r['box_iou'] >= A.OK_BOX else 'check'))
        r.pop('review', None)
    notag = [i for i, r in enumerate(R) if r['status'] == 'no_tag']
    for i, r in enumerate(R):
        if r['status'] in ('check', 'fail'): r['review'] = True
    for i in random.Random(seed).sample(notag, min(n_notag, len(notag))): R[i]['review'] = True
    t = Path(path).with_suffix('.tmp'); json.dump(J, open(t, 'w'), ensure_ascii=False); os.replace(t, path)
    c = lambda s: sum(r['status'] == s for r in R)
    print(path, '자동', c('auto'), '확인', c('check'), '실패', c('fail'), '태그 없음', c('no_tag'), '→ 볼 것', sum(bool(r.get('review')) for r in R))

if __name__ == '__main__':
    p = argparse.ArgumentParser(); p.add_argument('labels', nargs='+'); p.add_argument('--n_notag', type=int, default=100); p.add_argument('--seed', type=int, default=0); a = p.parse_args()
    for f in a.labels: pick(f, a.n_notag, a.seed)
