"""#82: 집게 45° 자세에서 팔 Arducam 시야 (정기구학 · 바늘구멍 투영, 렌더 없음, 도구 계산).
URDF = 팀 dev 98ef275 waffle_4wd_description/urdf/waffle_4wd.urdf. 화각 = sensors.json arm hfov 1.9721 rad, 1280x720 (가정값, 실물 미확인).
퍽: 박스 윗면(바닥 70 mm) 위에 선 퍽 30x30x60, 태그 16 mm 는 윗면 가운데. base_link 는 바닥 위 6.75 mm.
    python3 tilt_view.py <dev 작업 트리>"""
import sys, math, json, numpy as np, xml.etree.ElementTree as ET
WT = sys.argv[1]
URDF = f"{WT}/workspace/urdf_and_usd/waffle_4wd_description/urdf/waffle_4wd.urdf"
sys.path.insert(0, f"{WT}/workspace/urdf_and_usd/isaac/rl")
import pallet_traj as PT
r = ET.parse(URDF).getroot(); J = {}
def rpy(a, b, c):
    ca, sa, cb, sb, cc, sc = math.cos(a), math.sin(a), math.cos(b), math.sin(b), math.cos(c), math.sin(c)
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
    T = np.eye(4); chain = []
    while link in J:
        p, Tj, n, t, ax = J[link]
        if t in ('revolute', 'continuous') and n in q: Tj = Tj @ rot(ax, q[n])
        T = Tj @ T; chain.append(p); link = p
    return T, chain
W, H, HF = 1280, 720, 1.9721; F = W / 2 / math.tan(HF / 2)
Z0 = -0.00675; BOX_TOP = 0.07 + Z0; TAG = 0.016
arm = PT.Arm(URDF); j1 = arm.j1
cam_chain = fk('arm_camera_optical_frame', {})[1]
def proj(Tc, p):
    c = np.linalg.inv(Tc) @ np.append(p, 1)
    if c[2] <= 0.005: return None, c[2]
    return (F * c[0] / c[2] + W / 2, F * c[1] / c[2] + H / 2), c[2]
def view(q, puck_xy, puck_yaw, top_z, label):
    Tc, _ = fk('arm_camera_optical_frame', dict(zip(PT.ARM, q)))
    ctr = np.array([*puck_xy, top_z])
    R = np.array([[math.cos(puck_yaw), -math.sin(puck_yaw), 0], [math.sin(puck_yaw), math.cos(puck_yaw), 0], [0, 0, 1]])
    tag = [ctr + R @ np.array([sx * TAG / 2, sy * TAG / 2, 0]) for sx, sy in ((1, 1), (-1, 1), (-1, -1), (1, -1))]
    body = [np.array([*puck_xy, z]) + R @ np.array([sx * 0.015, sy * 0.015, 0]) for z in (top_z - 0.06, top_z) for sx in (1, -1) for sy in (1, -1)]
    uv, depth = proj(Tc, ctr)
    tuv = [proj(Tc, p)[0] for p in tag]
    buv = [proj(Tc, p)[0] for p in body]
    inside = lambda u: u is not None and 0 <= u[0] <= W and 0 <= u[1] <= H
    side = None
    if all(t is not None for t in tuv):
        side = float(np.mean([math.dist(tuv[i], tuv[(i + 1) % 4]) for i in range(4)]))
    ray = ctr - Tc[:3, 3]; ang = math.degrees(math.acos(abs(ray[2]) / np.linalg.norm(ray)))   # 태그 법선(위)과 시선 사이
    return {"case": label, "q": [round(v, 3) for v in q], "cam_z_bl": round(float(Tc[2, 3]), 3),
            "cam_axis_down_deg": round(math.degrees(math.acos(max(-1, min(1, -Tc[2, 2])))), 1),
            "puck_top_uv": [round(v) for v in uv] if uv else None, "puck_top_in": inside(uv),
            "tag_all_in": all(inside(t) for t in tuv), "tag_side_px": round(side, 1) if side else None,
            "view_angle_deg": round(ang, 1), "range_m": round(float(np.linalg.norm(ray)), 3),
            "body_in_frac": sum(inside(b) for b in buv) / 8}
READY_TILT = [0.0, -0.462, 0.771, 0.476, 0.0]
cases = [("A 원기둥", 0.203, 0.04), ("B 상자(회전 뒤)", 0.205, 0.06), ("C inline 앞", 0.139, 0.0), ("C inline 뒤", 0.219, 0.0)]
out = {"urdf": URDF, "camera_chain": cam_chain, "j1_bl": np.round(j1, 4).tolist(), "hfov_rad": HF, "rows": []}
top = BOX_TOP + 0.06
for name, d, y in cases:
    for sy in ((1, -1) if y else (1,)):
        yy = sy * y; xx = math.sqrt(d * d - yy * yy)
        p = (j1[0] + xx, j1[1] + yy)
        yaw_t = math.atan2(yy, xx)   # 상자 퍽 면이 J1 → 퍽 방향에 맞춰짐(#78 B)
        out["rows"].append({"pose": "READY_TILT (J1 0)", **view(READY_TILT, p, yaw_t, top, f"{name} y{yy:+.2f}")})
        q = list(READY_TILT); q[0] = yaw_t
        out["rows"].append({"pose": "READY_TILT, J1 퍽 쪽", **view(q, p, yaw_t, top, f"{name} y{yy:+.2f}")})
# 칸 놓기 직전: 쥔 퍽 바닥 = 박스 윗면 + 0.02 m, 집게 45° (pallet_traj.Arm.ik_down tilt). 박스 중심 J1 + 0.2355 (칸 0.139 ~ 0.332 의 가운데)
bc = j1[0] + 0.2355
for dx in (-0.06, 0.0, 0.06):
    for dy in (-0.06, 0.0, 0.06):
        cx, cy = bc + dx, j1[1] + dy
        rr, yaw = math.hypot(cx - j1[0], cy - j1[1]), math.atan2(cy - j1[1], cx - j1[0])
        q, e, t = arm.ik_down(np.array([cx, cy, BOX_TOP + 0.02]), yaw, None, off=PT.BOTTOM, tilt=math.radians(45))
        v = view(q, (cx, cy), yaw, BOX_TOP, f"칸 dx{dx:+.2f} dy{dy:+.2f}")   # 칸 바닥(박스 윗면) 가운데를 봄
        # 쥔 퍽(카메라 앞): 퍽 중심 = gripper_roll_link x 0.155 (TCP 0.14 + 15 mm)
        Tg, _ = fk('gripper_roll_link', dict(zip(PT.ARM, q)))
        held = (Tg @ np.array([0.155, 0, 0, 1]))[:3]
        Tc, _ = fk('arm_camera_optical_frame', dict(zip(PT.ARM, q)))
        huv, hz = proj(Tc, held)
        v.update({"ik_err_mm": round(e * 1000, 2), "tilt_err_deg": round(math.degrees(t), 2),
                  "held_puck_uv": [round(x) for x in huv] if huv else None, "held_puck_range_m": round(float(hz), 3),
                  "held_vs_cell_px": round(math.dist(huv, v["puck_top_uv"]), 0) if huv and v["puck_top_uv"] else None})
        out["rows"].append({"pose": "칸 놓기 직전 45°", **v})
json.dump(out, open(__file__.replace(".py", ".json"), "w"), ensure_ascii=False, indent=1, default=lambda o: o.item() if hasattr(o, "item") else str(o))
print('카메라 사슬', ' <- '.join(cam_chain[:4]), '| J1', out['j1_bl'])
for rw in out["rows"]:
    print(rw["pose"], rw["case"], {k: rw[k] for k in ("puck_top_in", "puck_top_uv", "tag_all_in", "tag_side_px", "view_angle_deg", "range_m", "body_in_frac") if k in rw},
          {k: rw[k] for k in ("ik_err_mm", "tilt_err_deg", "held_puck_uv", "held_puck_range_m", "held_vs_cell_px") if k in rw})
