"""접은 자세 S0에서 팔·집게와 다른 형상 사이 가까운 쌍(margin 안)을 찾는다(정적 계산, 시험 아님).
사용(PC): $MJ_PY tools/s0_near.py <model.xml> [margin m, 기본 0.03]"""
import sys
import mujoco
ARM = ["RMX52_Base_Yaw", "RMX52_Shoulder", "RMX52_Elbow", "RMX52_Wrist_Pitch", "RMX52_Wrist_Roll_Reversed"]
S0 = [0.0, 1.4, -1.0, -1.3, 0.0]
m = mujoco.MjModel.from_xml_path(sys.argv[1]); d = mujoco.MjData(m)
mg = float(sys.argv[2]) if len(sys.argv) > 2 else 0.03
armb = {b for b in range(m.nbody) if m.body(b).name.startswith(("arm_link_2", "arm_link_3", "arm_link_4", "arm_link_5", "gripper"))}
for g in range(m.ngeom):
    if m.geom_bodyid[g] in armb:
        m.geom_margin[g] = mg
d.qpos[3] = 1
for n, v in zip(ARM, S0):
    d.qpos[m.jnt_qposadr[m.joint(n).id]] = v
mujoco.mj_forward(m, d)
rows = {}
for i in range(d.ncon):
    c = d.contact[i]; b1, b2 = m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2]
    if (b1 in armb) == (b2 in armb):
        continue
    key = (m.body(b1).name, m.geom(c.geom1).name or f"#{c.geom1}", m.body(b2).name, m.geom(c.geom2).name or f"#{c.geom2}")
    rows[key] = min(rows.get(key, 9), c.dist)
for k, v in sorted(rows.items(), key=lambda kv: kv[1])[:10]:
    print(f"{v * 1000:7.1f} mm  {k}")
print("쌍 수", len(rows))
