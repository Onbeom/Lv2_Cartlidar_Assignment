"""s6_fine.txt 후보를 손목 롤 크기별로 묶고, 기준 대비 가장 빠듯한 비율(A1/30 px · j4 여유/0.1 · 실은 퍽/15 mm · 차체/15 mm)이 큰 순으로 보임."""
import collections, re, sys
rows = []
for l in open(sys.argv[1]).read().splitlines()[1:]:
    m = re.search(r'A1 여유 (\d+) px  q \[([^\]]+)\].*?광축 아래 ([\d.]+).*?회전 반지름 ([\d.]+).*?실은 퍽 ([\d.]+)/([\d.]+) mm.*?차체 ([\d.]+) mm', l)
    if not m:
        continue
    q = [float(x) for x in m.group(2).split(',')]
    r = dict(a1=int(m.group(1)), q=q, down=float(m.group(3)), rad=float(m.group(4)), pk=min(float(m.group(5)), float(m.group(6))),
             body=float(m.group(7)), j4m=round(2.0 - q[3], 2), j5=abs(q[4]))
    r['worst'] = min(r['a1'] / 30, r['j4m'] / 0.1, r['pk'] / 15, r['body'] / 15)
    rows.append(r)
print('후보', len(rows), '(상위 400 줄만 읽음)')
print('|j5| 분포', sorted(collections.Counter(round(r['j5'], 1) for r in rows).items()))
for lim5 in (0.2, 0.3, 0.4, 0.6, 0.8):
    c = sorted([r for r in rows if r['j5'] <= lim5 + 1e-9], key=lambda r: -r['worst'])
    print(f'|j5| <= {lim5}: {len(c)} 개')
    for r in c[:3]:
        print(f"   q {r['q']}  A1 {r['a1']} px · j4 여유 {r['j4m']} · 실은 퍽 {r['pk']} · 차체 {r['body']} mm · 회전 반지름 {r['rad']} · 광축 아래 {r['down']}° | 가장 빠듯한 비 {r['worst']:.2f}")
