"""라이다 높이 띠(base_link z 0.048~0.088 m)에 걸친 로봇 충돌 형상의 x · y 외곽 (관절 0, 도구 확인).
실행: python3 -I extent.py <urdf> [z_lo z_hi]"""
import sys, math, itertools
import xml.etree.ElementTree as ET
import numpy as np
u = sys.argv[1]; zlo, zhi = (float(sys.argv[2]), float(sys.argv[3])) if len(sys.argv) > 3 else (0.048, 0.088)
r = ET.parse(u).getroot()
def T(o):
    xyz = [float(v) for v in (o.get('xyz', '0 0 0').split() if o is not None else [0, 0, 0])]
    rpy = [float(v) for v in (o.get('rpy', '0 0 0').split() if o is not None else [0, 0, 0])]
    cr, sr, cp, sp, cy, sy = math.cos(rpy[0]), math.sin(rpy[0]), math.cos(rpy[1]), math.sin(rpy[1]), math.cos(rpy[2]), math.sin(rpy[2])
    R = np.array([[cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr], [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr], [-sp, cp*sr, cp*cr]])
    M = np.eye(4); M[:3, :3] = R; M[:3, 3] = xyz; return M
child = {j.find('child').get('link'): j for j in r.findall('joint')}
def world(link):
    M = np.eye(4)
    while link in child:
        j = child[link]; M = T(j.find('origin')) @ M; link = j.find('parent').get('link')
    return M, link
xs, ys, hits = [], [], []
for l in r.findall('link'):
    Mw, root = world(l.get('name'))
    for c in l.findall('collision'):
        g = c.find('geometry')[0]; M = Mw @ T(c.find('origin'))
        if g.tag == 'box':
            h = [float(v) / 2 for v in g.get('size').split()]
        elif g.tag == 'cylinder':
            rr, L = float(g.get('radius')), float(g.get('length')); h = [rr, rr, L / 2]
        elif g.tag == 'sphere':
            rr = float(g.get('radius')); h = [rr] * 3
        else:
            continue
        P = np.array([M @ np.array([sx*h[0], sy*h[1], sz*h[2], 1]) for sx, sy, sz in itertools.product((-1, 1), repeat=3)])
        if P[:, 2].max() < zlo or P[:, 2].min() > zhi:
            continue
        xs += [P[:, 0].min(), P[:, 0].max()]; ys += [P[:, 1].min(), P[:, 1].max()]; hits.append(l.get('name'))
print(f"뿌리 {root} · 띠 z {zlo}~{zhi}: x {min(xs):.4f}~{max(xs):.4f}, y {min(ys):.4f}~{max(ys):.4f} (링크 {len(set(hits))}개)")
