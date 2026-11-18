"""장소 상자 위에서 Arducam 으로 내려다보는 팔 자세 찾기 (B URDF 정기구학, 도구 확인).
로봇 최종 자리(앞 끝 − 상자 면 0.02 m)에서 상자 중심 = base_link x 0.1328 + 0.02 + 0.1025 = 0.2553.
조건: 광학축이 아래(−z)와 이루는 각 최소, 상자 윗면(z 0.07+0.00675 base_link) 네 꼭짓점 + 여유 30 mm 가 화면 안 (가정 화각 가로 113° · 1280x720)."""
import sys, math, itertools, numpy as np, xml.etree.ElementTree as ET
U = sys.argv[1]
r = ET.parse(U).getroot(); J = {}
def rpy(a, b, c):
    ca, sa, cb, sb, cc, sc = map(lambda f: f, (math.cos(a), math.sin(a), math.cos(b), math.sin(b), math.cos(c), math.sin(c)))
    return np.array([[cc*cb, cc*sb*sa-sc*ca, cc*sb*ca+sc*sa], [sc*cb, sc*sb*sa+cc*ca, sc*sb*ca-cc*sa], [-sb, cb*sa, cb*ca]])
for j in r.findall('joint'):
    o = j.find('origin'); T = np.eye(4)
    if o is not None:
        T[:3, :3] = rpy(*map(float, o.get('rpy', '0 0 0').split())); T[:3, 3] = list(map(float, o.get('xyz', '0 0 0').split()))
    a = j.find('axis'); ax = np.array(list(map(float, a.get('xyz').split()))) if a is not None else None
    J[j.find('child').get('link')] = (j.find('parent').get('link'), T, j.get('name'), j.get('type'), ax)
def rot(ax, q):
    ax = ax / np.linalg.norm(ax); K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
    R = np.eye(4); R[:3, :3] = np.eye(3) + math.sin(q) * K + (1 - math.cos(q)) * K @ K; return R
def fk(link, q):
    T = np.eye(4)
    while link in J:
        p, Tj, n, t, ax = J[link]
        if t in ('revolute', 'continuous') and n in q: Tj = Tj @ rot(ax, q[n])
        T = Tj @ T; link = p
    return T
lim = {j.get('name'): (float(j.find('limit').get('lower')), float(j.find('limit').get('upper'))) for j in r.findall('joint') if j.find('limit') is not None and j.get('type') == 'revolute'}
W, H, HF = 1280, 720, 1.9721; fx = W / 2 / math.tan(HF / 2); fy = fx; cx, cy = W / 2, H / 2
ztop = 0.07 + 0.00675; bc = np.array([0.2553, 0.0, ztop]); half = 0.1025 + 0.03
corn = [bc + np.array([sx * half, sy * half, 0]) for sx in (-1, 1) for sy in (-1, 1)]
best = []
for q2, q3, q4 in itertools.product(np.arange(-2.1, 1.6, 0.05), np.arange(-1.6, 1.6, 0.05), np.arange(-1.9, 2.0, 0.05)):
    q = {'joint1': 0.0, 'joint2': q2, 'joint3': q3, 'joint4': q4, 'joint5': 0.0}
    T = fk('arm_camera_optical_frame', q)
    zc = T[:3, 2]; down = math.degrees(math.acos(max(-1, min(1, -zc[2]))))
    if down > 25: continue
    Ti = np.linalg.inv(T); ok = True; m = 1e9
    for p in corn:
        c = Ti[:3, :3] @ p + Ti[:3, 3]
        if c[2] <= 0.05: ok = False; break
        u, v = fx * c[0] / c[2] + cx, fy * c[1] / c[2] + cy
        m = min(m, u, W - u, v, H - v)
        if not (0 <= u <= W and 0 <= v <= H): ok = False; break
    if not ok: continue
    # 팔 링크가 상자 윗면보다 위 (gripper_roll_link 원점 z > 윗면 + 0.05)
    g = fk('gripper_roll_link', q)[2, 3]
    if g < ztop + 0.05: continue
    best.append((down, -T[2, 3], q2, q3, q4, T[2, 3], m, g))
best.sort()
print('후보 수', len(best))
for b in best[:8]:
    print(f"아래와 {b[0]:.1f}°  카메라 높이(base_link) {b[5]:.3f} m  여유 {b[6]:.0f} px  집게 높이 {b[7]:.3f}  q2 {b[2]:.2f} q3 {b[3]:.2f} q4 {b[4]:.2f}")
