"""노드 단계 판정 (든 뒤 벌리면 놓음, 놓음 = 잡는 점이 D 수평 30 mm · 위 60 mm 안에서 처음 벌림 · β 1.4 이상에서 처음 벌림 (밖이면 떨어뜨림), 놓은 뒤 1 s 지났고 잡는 점이 D 에서 3 cm 넘으면 끝) 을 Isaac 기록에 대어 봄 (도구). 쥠 = 손가락만 히스테리시스."""
import json, numpy as np
from pathlib import Path
D = Path("/home/pa4/git/lv2_next_module/261110_2010_lie_rl/handoff/r2e_parity")
for f in sorted(D.glob("ep*.json")):
    st = json.loads(f.read_text())["episode"]["steps"]
    gz = 0.0; lifted = released = False; clear = 0; on = off = 0; held = False; t_rel = t_end = t_lift = None; env_rel = None
    for k, s in enumerate(st):
        raw = -0.0108 < s["finger_left"] < -0.003
        on = on + 1 if raw else 0; off = 0 if raw else off + 1
        held = True if on >= 8 else (False if off >= 3 else held)
        g = np.array(s["grasp_base"])
        if held and not lifted and gz == 0.0: gz = g[2]
        if not held and not lifted: gz = 0.0
        if held and gz != 0.0 and g[2] > gz + 0.03 and not lifted: lifted, t_lift = True, k
        if lifted and not released and not held: print(f.name, "떨어뜨림 판정", k); break
        if lifted and not released and s["action_raw"][5] > 0:
            pc = g + 0.011 * np.array(s["ee_a_base"])   # 퍽 중심 추정 (잡는 점 + a × 11 mm), 높이는 잡는 점
            dD = g - np.array(s["D"]); dh, dz = float(np.hypot(*(pc - np.array(s["D"]))[:2])), float(dD[2])
            if dh > 0.03 or dz > 0.06 or s["beta_before"] < 1.4:
                print(f.name, f"D 밖에서 벌림 → 떨어뜨림 실패 (스텝 {k}, 수평 {dh*1000:.1f} mm, 위 {dz*1000:.1f} mm)"); break
            released, t_rel, rel_dh, rel_dz = True, k, dh, dz
            q_end = None
        if env_rel is None and s["released"]: env_rel = k
        clear = clear + 1 if released else 0
        if clear >= 30 and np.linalg.norm(g - np.array(s["D"])) > 0.03 and t_end is None: t_end, q_end = k, s["q"]
    print(f"{f.name}: 스텝 {len(st)}, 듦 {t_lift}, 놓음(노드) {t_rel} (추정 중심-D 수평 {rel_dh*1000:.1f} mm · 위 {rel_dz*1000:.1f} mm) · (학습 released) {env_rel}, "
          f"끝(노드) {t_end}, 끝 관절 {[round(v, 4) for v in q_end]}")
