import json, re, math, sys
R = sys.argv[1]
d = json.load(open(f'{R}/mock_out.json'))
truth = [p for e in d['start_box'] for p in e['pucks'].values()]
ox, oy, oyaw = 0.3, -0.2, math.radians(10.0)
errs = []
for l in open(f'{R}/mission.log'):
    m = re.search(r'-> target_look:odom:([-\d.]+):([-\d.]+):', l)
    if not m:
        continue
    x, y = float(m.group(1)), float(m.group(2))
    mx, my = ox + math.cos(oyaw) * x - math.sin(oyaw) * y, oy + math.sin(oyaw) * x + math.cos(oyaw) * y
    e = min(math.hypot(mx - p[0], my - p[1]) for p in truth)
    errs.append(e * 1000)
print(R, '조준점', len(errs), '개, 참 퍽 자리와 차이 mm: 최대 %.2f · 평균 %.2f' % (max(errs), sum(errs) / len(errs)) if errs else '없음')
for l in open(f'{R}/mission.log'):
    if 'align BOX' in l or 'sign' in l.lower() or '부호' in l:
        print('  ', l.strip()[60:200])
