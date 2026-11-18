"""팔 자세별 팔 · 집게 충돌 상자 ↔ 팔레트에 실은 퍽 12 개 최소 거리 (계산, 2026-11-11 미션 파트 261111 1).
퍽 = loaded URDF 의 실제 모양 (원기둥 r 0.015 · 높이 0.06 은 32 각기둥, 사각 0.03 × 0.03 × 0.06), 자리 = pallet_slots.yaml 과 같음 (퍽 가운데 = 칸 바닥 + 0.03).
loaded 판 Isaac 퍽은 겉모양 · 질량만 (충돌 없음) 이라 Isaac 접촉 힘으로는 이 간섭이 안 잡힘.
  python loaded_pucks.py <isaac urdf 폴더>
"""
import math, os, sys, xml.etree.ElementTree as ET
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'rl_probe'))
from align_geom import Geom, box_pts, tf
I = sys.argv[1]
G = Geom(I + '/waffle_4wd_loaded.urdf')
r = ET.parse(I + '/waffle_4wd_loaded.urdf').getroot()
shape = {}
for l in r.findall('link'):
    if l.get('name').startswith('puck_'):
        g = l.find('visual').find('geometry')[0]
        shape[l.get('name')] = g.tag
P = []
cyl = np.array([[0.015 * math.cos(a), 0.015 * math.sin(a), z] for a in np.linspace(0, 2 * math.pi, 32, endpoint=False) for z in (-0.03, 0.03)])
for j in r.findall('joint'):
    ch = j.find('child').get('link')
    if ch.startswith('puck_'):
        o = j.find('origin'); T = tf(list(map(float, o.get('xyz').split())), list(map(float, o.get('rpy').split())))
        B = cyl if shape[ch] == 'cylinder' else box_pts([0.03, 0.03, 0.06])
        P.append((ch, (T[:3, :3] @ B.T).T + T[:3, 3]))
poses = dict(DRIVE=[-0.0245, -1.6122, 1.2901, 1.1888, 0], TRANSIT_TILT=[0, -1.32032, 0.67766, 1.42805, 0],
             E1=[0, -1.35, -0.02, 1.88, 0], E2=[0, -1.37, -0.01, 1.85, 0], E1m=[0, -1.367, 0.004, 1.897, 0], E2m=[0, -1.388, 0.013, 1.867, 0], STOW=[1.554, -2.088, 1.549, -1.868, 0.043], ALIGN_A_dev=[0, -1.854, 0.540, 1.645, 0],
             C1=[0, -1.59, 0.15, 1.88, 0], D1=[0, -1.63, 0.18, 1.89, 0], D2=[0, -1.62, 0.21, 1.88, 0], D3=[0, -1.62, 0.20, 1.89, 0],
             BOX_A=[0.30, -1.25, 0.15, 1.85, 0], TL_I=[0, -0.8610, 0.4326, 1.3883, 0])
for n, q in poses.items():
    rows = []
    for pn, B in P:
        best = (9, None)
        for ln, an, A in G.arm_pts(q, 0.010):
            d = Geom.dist(A, B)
            if d < best[0]: best = (d, an)
        rows.append((best[0], pn, best[1]))
    rows.sort()
    low = [f'{p} {d*1000:.1f}' for d, p, _ in rows if d < 0.015]
    print(f'{n:13s} 최소 {rows[0][0]*1000:6.1f} mm ({rows[0][2]} – {rows[0][1]}) | 15 mm 미만 퍽: {", ".join(low) or "없음"}')
