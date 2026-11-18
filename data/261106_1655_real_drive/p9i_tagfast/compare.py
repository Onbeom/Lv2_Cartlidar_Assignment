# 사용: compare.py 기준.txt 비교.txt 이름. 줄 = 경로 시간 (번호 x0 y0 ... x3 y3)*
import collections, math, os, re, sys
def load(f):
    d = {}
    for l in open(f):
        a = l.split()
        v = a[2:]; tags = []
        for i in range(0, len(v), 9):
            tags.append((int(v[i]), [float(x) for x in v[i+1:i+9]]))
        d[a[0]] = (float(a[1]), tags)
    return d
def truth(p):
    n = os.path.basename(os.path.dirname(p))
    m = re.match(r'tag(\d+)_', n)
    if m: return int(m.group(1))
    m = re.match(r'L([123])_', n)
    if m: return 11 + int(m.group(1))
    return None
def pct(v, q):
    v = sorted(v); return v[min(len(v)-1, int(q*len(v)))] if v else float('nan')
b, n = load(sys.argv[1]), load(sys.argv[2]); name = sys.argv[3]
keys = [p for p in b if p in n]
tags = [p for p in keys if truth(p) is not None]; empty = [p for p in keys if truth(p) is None]
hit = lambda d, p: truth(p) in [t[0] for t in d[p][1]]
nb, nv = sum(hit(b, p) for p in tags), sum(hit(n, p) for p in tags)
lost = [p for p in tags if hit(b, p) and not hit(n, p)]
gain = [p for p in tags if hit(n, p) and not hit(b, p)]
wrong = sum(sum(t[0] != truth(p) for t in n[p][1]) for p in tags)
emp = sum(len(n[p][1]) for p in empty)
d = []
for p in tags:
    B = dict(b[p][1]); N = dict(n[p][1]); t = truth(p)
    if t in B and t in N:
        d.append(max(math.hypot(B[t][i]-N[t][i], B[t][i+1]-N[t][i+1]) for i in range(0, 8, 2)))
print(f'{name}: 장 {len(keys)} · 정답 ID 찾은 장 {nv}/{len(tags)} (기준 {nb}) · 잃음 {len(lost)} · 얻음 {len(gain)} · 다른 ID {wrong} · 태그 없는 폴더 검출 {emp}({len(empty)}장)')
print(f'  꼭짓점 최대 차이 px: 중앙 {pct(d,.5):.2f} · p95 {pct(d,.95):.2f} · p99 {pct(d,.99):.2f} · 최대 {max(d) if d else float("nan"):.2f} ({len(d)} 장)')
tb, tn = [b[p][0] for p in keys], [n[p][0] for p in keys]
print(f'  시간 ms: 기준 평균 {sum(tb)/len(tb):.1f} p99 {pct(tb,.99):.1f} · 이번 평균 {sum(tn)/len(tn):.1f} p99 {pct(tn,.99):.1f}')
by = collections.Counter(os.path.basename(os.path.dirname(p)) for p in lost)
if by: print('  잃은 폴더', dict(by))
