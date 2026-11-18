"""두 번 보기(survey · verify, 면 0.127, 0.24 m 60°, 보는 점 상자 가운데 −0.02 · +0.06) 와 place_box(면 0.20, 0.38 m 55°) 자세의
관절값 · 차체 CoACD · 실은 퍽 거리 (계산). python body_check.py <dev urdf> <isaac urdf 폴더>"""
import math, os, subprocess, sys
import numpy as np
dev, isaac = sys.argv[1], sys.argv[2]
AV = "/home/pa4/git/lv2_next_module/261111_1700_align_view"
sys.argv = ["align_search.py", dev, "--pose", "-1.37", "-0.01", "1.85"]
ns = {"__name__": "x"}; o_ = sys.stdout; sys.stdout = open(os.devnull, "w"); exec(open(os.path.join(AV, "align_search.py")).read(), ns); sys.stdout = o_
fk = ns["fk"]
sys.path.insert(0, os.path.join(AV, "rl_probe"))
sys.argv = ["loaded_pucks.py", isaac]
from align_geom import Geom, box_pts, tf
import xml.etree.ElementTree as ET
FG = Geom(os.path.join(isaac, "waffle_4wd_full.urdf")); LG = Geom(os.path.join(isaac, "waffle_4wd_loaded.urdf"))
r = ET.parse(os.path.join(isaac, "waffle_4wd_loaded.urdf")).getroot()
shape = {l.get("name"): l.find("visual").find("geometry")[0].tag for l in r.findall("link") if l.get("name").startswith("puck_")}
cyl = np.array([[0.015 * math.cos(t), 0.015 * math.sin(t), z] for t in np.linspace(0, 2 * math.pi, 32, endpoint=False) for z in (-0.03, 0.03)])
P = []
for j in r.findall("joint"):
    ch = j.find("child").get("link")
    if ch.startswith("puck_"):
        o = j.find("origin"); T = tf(list(map(float, o.get("xyz").split())), list(map(float, o.get("rpy").split())))
        B = cyl if shape[ch] == "cylinder" else box_pts([0.03, 0.03, 0.06]); P.append((ch, (T[:3, :3] @ B.T).T + T[:3, 3]))
def pk(q):
    return min(Geom.dist(A, B) for _, _, A in LG.arm_pts(q, 0.010) for _, B in P)
T_ab = fk("arm_base_link", {})
for name, face, off, d, an in (("survey·verify 앞", 0.127, -0.02, 0.24, 60), ("survey·verify 뒤", 0.127, 0.06, 0.24, 60), ("place_box", 0.20, 0.0, 0.38, 55)):
    xc = 0.1328 + face + 0.1025 + off
    aim = np.linalg.inv(T_ab) @ np.array([xc, 0, 0.07, 1])
    out = subprocess.run(["./look_tool"], input=f"{aim[0]} {aim[1]} {aim[2]} 0.04 1 {d} 1 {an}\n", capture_output=True, text=True).stdout.split()
    q = [float(x) for x in out[3:8]]; qs = [q[0], q[1] - 0.02, q[2], q[3] + 0.02, q[4]]
    print(f"{name}: 면 {face} 보는 점 +{off} {d} m {an}° → q {np.round(q, 3).tolist()} | 차체 {min(FG.min_dist(q)[0], FG.min_dist(qs)[0]) * 1000:.1f} mm | 실은 퍽 {min(pk(q), pk(qs)) * 1000:.1f} mm")
