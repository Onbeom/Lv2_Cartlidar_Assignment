"""#81 · #82: 45° 집기 전에 퍽 윗면 태그를 화면 가운데 가까이 보는 팔 자세 찾기 (정기구학 격자, 렌더 없음).
조건: J1 = 퍽 방향, joint5 0, 퍽 윗면 가운데가 화면 가운데에서 가장 가깝고, 시선과 태그 법선 사이 ≤ 20°, 거리 0.12~0.25 m,
gripper_roll_link 와 TCP 가 퍽 윗면보다 0.05 m 위, 관절 한계 안(여유 0.02)."""
import sys, math, itertools, json, numpy as np
sys.argv = [sys.argv[0], sys.argv[1]]
exec(open('tilt_view.py').read().split('READY_TILT = ')[0])
res = {}
for d, y in ((0.203, 0.0), (0.205, 0.06), (0.139, 0.0), (0.219, 0.0)):
    yaw = math.atan2(y, math.sqrt(max(d * d - y * y, 0))); px, py = j1[0] + d * math.cos(yaw), j1[1] + d * math.sin(yaw)
    top = BOX_TOP + 0.06; best = None
    def score(q2, q3, q4):
        q = [yaw, q2, q3, q4, 0.0]
        Tg, _ = fk('gripper_roll_link', dict(zip(PT.ARM, q)))
        if Tg[2, 3] < top + 0.05 or (Tg @ np.array([0.14, 0, 0, 1]))[2] < top + 0.05: return None
        v = view(q, (px, py), yaw, top, '')
        if not v["tag_all_in"] or v["view_angle_deg"] > 20 or not (0.12 <= v["range_m"] <= 0.25): return None
        return math.dist(v["puck_top_uv"], (W / 2, H / 2)), v
    def search(r2, r3, r4):
        b = None
        for q2, q3, q4 in itertools.product(r2, r3, r4):
            s_ = score(q2, q3, q4)
            if s_ and (b is None or s_[0] < b[0]): b = (s_[0], s_[1], (q2, q3, q4))
        return b
    best = search(np.arange(arm.lo[1], arm.hi[1], 0.1), np.arange(arm.lo[2], arm.hi[2], 0.1), np.arange(arm.lo[3], arm.hi[3], 0.1))
    if best:
        c = best[2]; rg = lambda i, k: np.clip(np.arange(c[i] - 0.1, c[i] + 0.1001, 0.02), arm.lo[k], arm.hi[k])
        best = search(rg(0, 1), rg(1, 2), rg(2, 3)) or best
    res[f"J1 {d} 옆 {y}"] = best[1] if best else None
    print(f"J1 {d} 옆 {y}:", (round(best[0]), {k: best[1][k] for k in ('q', 'puck_top_uv', 'tag_side_px', 'view_angle_deg', 'range_m', 'cam_z_bl', 'cam_axis_down_deg')}) if best else '없음')
json.dump(res, open('observe_pose.json', 'w'), ensure_ascii=False, indent=1, default=lambda o: o.item() if hasattr(o, "item") else str(o))
