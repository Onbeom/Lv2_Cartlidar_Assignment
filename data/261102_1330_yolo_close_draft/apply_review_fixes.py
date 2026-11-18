"""검수(review_labels.py · 그림 확인, 11-02 15:5x)에서 분명한 실수로 본 곳만 고친다. 기본은 미리 보기(파일 안 바꿈), --apply 일 때만 씀.
쓰기 전에 <폴더>/labels_before_review/ 에 원래 라벨을 한 번 복사해 둔다(이미 있으면 다시 복사하지 않음).
규칙마다 정확히 상자 1개가 걸려야 하고, 아니면 아무것도 쓰지 않고 멈춘다.
대상 고르기: overlap = 같은 장에서 다른 상자와 IoU ≥ 0.5 로 겹친 그 클래스 상자 · only = 그 클래스 상자가 하나뿐
            nb:<이웃 장 앞부분>:<클래스> = 이웃 장의 그 클래스 상자와 IoU ≥ 0.3 · orphan = 앞뒤 장 어느 상자와도 IoU < 0.1"""
import json
import shutil
import sys
from pathlib import Path

HERE = Path(__file__).parent
AB = ['RC', 'GC', 'RB', 'GB']
F, R, A = 'real_labels_close_front_C', 'real_labels_close_rear_C', 'real_labels_arm720_C'
RULES = [  # (폴더, 장, 할 일, 클래스, 대상, 새 클래스, 이유)
    (F, 'f1223__169', 'del', 'RC', 'overlap', None, 'RB 와 같은 퍽에 겹친 상자'),
    (F, 'f1223__170', 'del', 'RC', 'overlap', None, 'RB 와 같은 퍽에 겹친 상자'),
    (F, 'f1223__177', 'del', 'GC', 'overlap', None, 'GB 와 같은 퍽에 겹친 상자'),
    (F, 'f1223__183', 'del', 'GC', 'overlap', None, 'GB 와 같은 퍽에 겹친 상자'),
    (R, 'r1221__093', 'del', 'GC', 'overlap', None, '같은 초록 원기둥에 GC 두 개(하나만 지움)'),
    (R, 'r1226__235', 'del', 'RB', 'only', None, '화면 위 가장자리 갈색 얼룩(퍽 아님, 초안 0.20)'),
    (A, 'a1244__020', 'del', 'GC', 'orphan', None, '빈 바닥에 그려진 상자'),
    (A, 'a1244__047', 'del', 'RB', 'only', None, '가는 초록 줄(퍽 아님)'),
    (A, 'a1244__073', 'del', 'GB', 'overlap', None, '초록 원기둥에 GC · GB 겹침'),
    (A, 'a1245__114', 'del', 'RB', 'overlap', None, '빨강 원기둥에 RC · RB 겹침'),
    (A, 'a1245__155', 'del', 'GC', 'overlap', None, '왼쪽 작은 초록 직육면체에 GC · GB 겹침'),
    (A, 'a1245__163', 'cls', 'RC', 'nb:a1245__164:RB', 'RB', '박스 위 빨강 직육면체(앞뒤 장 RB)'),
    (A, 'a1245__167', 'cls', 'RB', 'nb:a1245__168:GB', 'GB', '왼쪽 구석 초록 직육면체를 RB 로 적음'),
    (A, 'a1245__225', 'del', 'RC', 'overlap', None, '빨강 직육면체에 RC · RB 겹침(앞 장 RB)'),
    (A, 'a1247__349', 'del', 'RC', 'overlap', None, '박스 위 빨강 직육면체에 RB · RC 겹침'),
    (A, 'a1248__500', 'cls', 'GC', 'nb:a1248__499:GB', 'GB', '초록 직육면체를 GC 로 적음'),
    (A, 'a1248__500', 'cls', 'RC', 'nb:a1248__499:RB', 'RB', '왼쪽 가장자리 빨강(앞뒤 장 RB)'),
    (A, 'a1248__504', 'del', 'GC', 'overlap', None, '초록 직육면체에 GB · GC 겹침'),
]


def iou(a, b):
    ix = max(0.0, min(a[1] + a[3] / 2, b[1] + b[3] / 2) - max(a[1] - a[3] / 2, b[1] - b[3] / 2))
    iy = max(0.0, min(a[2] + a[4] / 2, b[2] + b[4] / 2) - max(a[2] - a[4] / 2, b[2] - b[4] / 2))
    u = a[3] * a[4] + b[3] * b[4] - ix * iy
    return ix * iy / u if u > 0 else 0.0


def path(d, pre):
    m = sorted((HERE / d / 'labels').glob(pre + '_*.txt'))
    assert len(m) == 1, (d, pre, m)
    return m[0]


def load(p):
    return [[int(l.split()[0])] + [float(v) for v in l.split()[1:]] for l in p.read_text().split('\n') if l.strip()]


def neighbors(d, pre):
    b, n = pre.split('__')
    ps = []
    for k in (int(n) - 1, int(n) + 1):
        m = sorted((HERE / d / 'labels').glob(f'{b}__{k:03d}_*.txt'))
        ps += [load(x) for x in m]
    return [x for bs in ps for x in bs]


apply = '--apply' in sys.argv
work = {}
for d, pre, op, cls, sel, new, why in RULES:
    p = path(d, pre)
    B = work.setdefault(p, load(p))
    k = AB.index(cls)
    cand = [i for i, b in enumerate(B) if b[0] == k]
    if sel == 'overlap':
        cand = [i for i in cand if any(j != i and iou(B[i], B[j]) >= 0.5 for j in range(len(B)))]
        if len(cand) == 2 and B[cand[0]][0] == B[cand[1]][0]:
            cand = cand[1:]  # 같은 클래스 두 개면 뒤의 것 하나만
    elif sel == 'only':
        pass
    elif sel == 'orphan':
        nb = neighbors(d, pre)
        cand = [i for i in cand if all(iou(B[i], x) < 0.1 for x in nb)]
    else:
        _, npre, ncls = sel.split(':')
        ref = [x for x in load(path(d, npre)) if x[0] == AB.index(ncls)]
        cand = [i for i in cand if any(iou(B[i], x) >= 0.3 for x in ref)]
    if len(cand) != 1:
        sys.exit(f'멈춤: {pre} {op} {cls} {sel} → 걸린 상자 {len(cand)}개')
    i = cand[0]
    print(f'{pre}: {cls} 상자({B[i][1]:.3f}, {B[i][2]:.3f}) {"지움" if op == "del" else "→ " + new} — {why}')
    if op == 'del':
        B[i] = None
    else:
        B[i][0] = AB.index(new)
    work[p] = [b for b in B if b is not None]
if apply:
    for p, B in work.items():
        bak = p.parent.parent / 'labels_before_review'
        if not bak.exists():
            shutil.copytree(p.parent, bak)
        p.write_text(''.join(f'{b[0]} {b[1]:.6f} {b[2]:.6f} {b[3]:.6f} {b[4]:.6f}\n' for b in B))
    print(f'씀: 파일 {len(work)}개')
else:
    print(f'미리 보기만(파일 {len(work)}개 대상). 쓰려면 --apply')
