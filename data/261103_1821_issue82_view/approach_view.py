"""#82 집는 동안(근사): 집게 45° · TCP 를 퍽 잡는 점 위 h 에서 잡는 점까지 내릴 때 퍽 윗면 태그가 화면 어디에 있는지 (정기구학, 렌더 없음).
TopPick 정책 궤적이 아니라 '똑바로 내려가는' 근사다. 잡는 점 = 퍽 중심 + 15 mm (pallet_traj GRASP_DZ). 경우 A(J1 0.203 m, 옆 0) 기준."""
import sys, math, json, numpy as np
sys.argv = [sys.argv[0], sys.argv[1]]
exec(open('tilt_view.py').read().split('READY_TILT = ')[0])
rows = []
for d, y in ((0.203, 0.0), (0.205, 0.06)):
    yaw = math.atan2(y, math.sqrt(d * d - y * y)); px, py = j1[0] + d * math.cos(yaw), j1[1] + d * math.sin(yaw)
    top = BOX_TOP + 0.06; grasp = np.array([px, py, BOX_TOP + 0.03 + 0.015])
    q0 = None
    for h in (0.10, 0.08, 0.06, 0.04, 0.03, 0.02, 0.01, 0.0):
        tgt = grasp + np.array([0, 0, h])
        q, e, t = arm.ik_down(tgt, yaw, q0, tilt=math.radians(45)); q0 = q[1:4]
        v = view(q, (px, py), yaw, top, f"J1 {d} 옆 {y} h {h:.2f}")
        v.update({"ik_err_mm": round(e * 1000, 2), "tilt_err_deg": round(math.degrees(t), 2)})
        rows.append(v)
        print(v["case"], v["q"], {k: v[k] for k in ("puck_top_uv", "puck_top_in", "tag_all_in", "tag_side_px", "range_m", "body_in_frac", "ik_err_mm")})
json.dump(rows, open('approach_view.json', 'w'), ensure_ascii=False, indent=1, default=lambda o: o.item() if hasattr(o, "item") else str(o))
