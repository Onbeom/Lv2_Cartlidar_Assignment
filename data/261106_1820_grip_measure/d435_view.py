"""쥔 퍽을 앞 D435 로 볼 수 있는 팔 자세가 있는지 (기하 계산, 렌더 아님). 집게 45° 유지, f 606 @ 640x480 (perception 주석의 값)."""
import math
import sys
import xml.etree.ElementTree as ET

import numpy as np

sys.path.insert(0, '../Cartlidar/sim_render')
from urdf_chain import fixed_transform

U = '/home/pa4/git/Lv2_Cartlidar_Assignment_armcam/workspace/urdf_and_usd/waffle_4wd_description/urdf/waffle_4wd.urdf'
links = [j.find('child').get('link') for j in ET.parse(U).getroot().findall('joint')]
print([l for l in links if 'top_camera_front' in l][:8])
cam = [l for l in links if 'top_camera_front' in l and 'optical' in l]
cam = cam[0] if cam else 'top_camera_front_link'
T = fixed_transform(U, cam, root='base_footprint')
print(cam, T[:3, 3].round(3).tolist(), 'z축', T[:3, 2].round(3).tolist(), '광학' if 'optical' in cam else '링크(x 앞)')
if 'optical' not in cam:
    Ro = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], float)
    T = T.copy()
    T[:3, :3] = T[:3, :3] @ Ro
f, W, H, S = 606, 640, 480, math.sqrt(.5)
res = []
L = np.array([.139, 0, 0]) - .015 * np.array([-S, 0, S])
Ti = np.linalg.inv(T)
for q2 in np.arange(-2.1, 1.61, .15):
    for q3 in np.arange(-1.6, 1.61, .15):
        q4 = .7854 - q2 - q3
        if not -1.92 < q4 < 2.0:
            continue
        G = fixed_transform(U, 'gripper_roll_link', root='base_footprint', q={'joint2': q2, 'joint3': q3, 'joint4': q4})
        c = (G @ np.append(L, 1))[:3]
        p = Ti @ np.append(c, 1)
        if p[2] < .1 or c[2] < .14:
            continue
        u, v = f * p[0] / p[2] + W / 2, f * p[1] / p[2] + H / 2
        if 60 < u < W - 60 and 60 < v < H - 60:
            res.append((round(float(p[2]), 3), round(u), round(v), round(float(q2), 2), round(float(q3), 2), round(float(q4), 2), round(float(c[0]), 3), round(float(c[2]), 3)))
res.sort()
print(len(res), '자세. (거리, u, v, q2, q3, q4, 퍽 x, 퍽 z)')
print(res[:4])
print(res[-2:])
if res:
    print('가장 가까울 때 1 mm =', round(f * .001 / res[0][0], 1), 'px')
