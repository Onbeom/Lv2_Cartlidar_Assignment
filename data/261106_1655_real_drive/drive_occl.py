"""DRIVE 자세에서 앞 위 D435 컬러 화면을 팔이 얼마나 가리는지 (팀 URDF 시각 메시 투영, 모의 계산)."""
import sys, os, struct, numpy as np, cv2, xml.etree.ElementTree as ET
U, OUT = sys.argv[1], sys.argv[2]
q = [float(v) for v in sys.argv[3].split(',')]
W_, H_, FX, FY, CX, CY = 640, 480, 606.0, 606.0, 320.0, 240.0   # D435 컬러 640x480 대략값 (perception.yaml 주석 fx 약 606)
r = ET.parse(U).getroot()
def rpy(r_, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r_), np.sin(r_), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr], [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])
def T(xyz, R): M = np.eye(4); M[:3, :3] = R; M[:3, 3] = xyz; return M
def org(e):
    o = e.find('origin')
    if o is None: return np.eye(4)
    return T([float(v) for v in o.get('xyz', '0 0 0').split()], rpy(*[float(v) for v in o.get('rpy', '0 0 0').split()]))
def axrot(a, t):
    a = a/np.linalg.norm(a); K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]]); return np.eye(3)+np.sin(t)*K+(1-np.cos(t))*K@K
J = {}
for j in r.findall('joint'):
    ax = j.find('axis'); a = np.array([float(v) for v in ax.get('xyz').split()]) if ax is not None else np.array([1., 0, 0])
    J[j.find('child').get('link')] = (j.get('name'), j.get('type'), j.find('parent').get('link'), org(j), a)
pose = dict(zip(['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'], q))
print('관절 이름:', [v[0] for v in J.values() if v[1] in ('revolute', 'prismatic', 'continuous')])
cache = {}
def Wl(l):
    if l in cache: return cache[l]
    if l not in J: cache[l] = np.eye(4); return cache[l]
    n, t, p, M, a = J[l]; t_ = pose.get(n, 0.0)
    X = Wl(p)@M@(T(np.zeros(3), axrot(a, t_)) if t in ('revolute', 'continuous') else T(a*t_, np.eye(3)) if t == 'prismatic' else np.eye(4))
    cache[l] = X; return X
def stl(path):
    d = open(path, 'rb').read(); n = struct.unpack('<I', d[80:84])[0]
    a = np.frombuffer(d, dtype=np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')]), count=n, offset=84)
    return a['v'].astype(float)
root = [l.get('name') for l in r.findall('link') if l.get('name') not in J][0]
C = np.linalg.inv(Wl('top_camera_front_optical_frame'))
print('root', root, '카메라 자리(root 기준)', np.round(Wl('top_camera_front_optical_frame')[:3, 3], 3))
mask = np.zeros((H_, W_), np.uint8); per = {}
for l in r.findall('link'):
    name = l.get('name')
    if not any(k in name for k in ('link2', 'link3', 'link4', 'link5', 'gripper', 'camera_link', 'tool', 'link1')): continue
    m1 = np.zeros((H_, W_), np.uint8)
    for v in l.findall('visual'):
        me = v.find('geometry/mesh')
        if me is None: continue
        sc = np.array([float(s) for s in me.get('scale', '1 1 1').split()])
        fn = me.get('filename'); fn = os.path.join(os.path.dirname(U), '..', fn.split('waffle_4wd_description/')[-1]) if fn.startswith('package:') else os.path.join(os.path.dirname(U), fn); tri = stl(os.path.normpath(fn)) * sc
        X = C@Wl(name)@org(v)
        p = tri.reshape(-1, 3)@X[:3, :3].T + X[:3, 3]
        p = p.reshape(-1, 3, 3)
        p = p[(p[:, :, 2] > 0.03).all(1)]
        uv = np.stack([FX*p[:, :, 0]/p[:, :, 2]+CX, FY*p[:, :, 1]/p[:, :, 2]+CY], -1)
        keep = (uv[:, :, 0].max(1) > 0) & (uv[:, :, 0].min(1) < W_) & (uv[:, :, 1].max(1) > 0) & (uv[:, :, 1].min(1) < H_)
        for t in np.round(uv[keep]).astype(np.int32): cv2.fillConvexPoly(m1, t, 255)
    if m1.any(): per[name] = round(100*m1.mean()/255, 1); mask |= m1
print('링크별 가림 %:', per)
ys, xs = np.nonzero(mask)
print('전체 가림 %.1f %%, 가로 px %d~%d, 세로 px %d~%d' % (100*mask.mean()/255, xs.min(), xs.max(), ys.min(), ys.max()))
# 신호등: 출발 자세 (0.694, 0.95, 180 deg) 에서 (0.08, 0.95) → base_footprint 앞 0.614 m, 등 세 칸 (위 빨 · 노 · 초), 바닥 위 68.9~173.6 mm, 폭 39.6 mm
foot = np.linalg.inv(Wl('base_footprint')) if 'base_footprint' in J or root == 'base_footprint' else np.eye(4)
Cf = C@np.linalg.inv(foot) if root != 'base_footprint' else C
img = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)//2
for name, zc, col in (('R', 0.1736-0.0349/2, (0, 0, 255)), ('Y', (0.0689+0.1736)/2, (0, 255, 255)), ('G', 0.0689+0.0349/2, (0, 255, 0))):
    pts = np.array([[0.614, y, z, 1] for y in np.linspace(-0.0198, 0.0198, 9) for z in np.linspace(zc-0.01745, zc+0.01745, 9)])
    p = pts@Cf.T
    u, v = FX*p[:, 0]/p[:, 2]+CX, FY*p[:, 1]/p[:, 2]+CY
    inv = (u >= 0) & (u < W_) & (v >= 0) & (v < H_)
    hid = np.array([mask[int(b), int(a)] > 0 if i else False for a, b, i in zip(u, v, inv)])
    print('신호등 %s: 화면 (%.0f, %.0f) px, 화면 안 %d %%, 팔에 가림 %d %%' % (name, u.mean(), v.mean(), 100*inv.mean(), 100*hid.mean()))
    cv2.rectangle(img, (int(u.min()), int(v.min())), (int(u.max()), int(v.max())), col, 2)
cv2.imwrite(OUT, img)
