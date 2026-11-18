"""r2g2 Isaac 기록 (261110_2010_lie_rl/handoff/r2g2_parity) 과 C++ control/lieupright (dev) 계산을 스텝마다 비교 (모의 기록 대 C++).
관측 45 (C++ 가 FK · 상태 · 노드 D 규칙 · 쥠 필터로 만드는 칸), β · ρ, 비례 축소 뒤 관절 목표 5, 고른 배율, 잡는 점 · 집게 축."""
import json, math, subprocess, sys
from pathlib import Path
import numpy as np
D = Path("/home/pa4/git/lv2_next_module/261110_2010_lie_rl/handoff/r2g2_parity")
TOOL = str(Path(__file__).resolve().parent / "parity_tool")
NAMES = [("joint_pos_rel", 5), ("joint_vel", 5), ("finger_pos", 1), ("puck_pos", 3), ("long_axis", 2), ("grasp_pos", 3), ("approach", 3),
         ("close", 3), ("stand_spot", 3), ("nb_rel", 2), ("nb_present", 1), ("box_c", 2), ("box_yaw", 2), ("held", 1), ("rho_beta", 3), ("prev_action", 6)]
worst, where = {}, {}
def upd(k, v, tag):
    if v > worst.get(k, -1.0):
        worst[k], where[k] = float(v), tag
for f in sorted(D.glob("ep*.json")):
    e = json.loads(f.read_text())["episode"]
    st = e["steps"]
    fp, fth, bc, by, vmax = e["fix_p"], e["fix_th"], e["box_c_obs"], e["box_yaw_obs"], e["vmax"]
    rows = []
    for k, s in enumerate(st):
        o = s["obs"]
        nbp = o[30]
        nb = [fp[0] + o[28], fp[1] + o[29]] if nbp > 0.5 else [0.0, 0.0]
        rows.append(" ".join(map(repr, [1 if k == 0 else 0, *s["q"], *s["qd"], s["finger_left"], *s["arm_target_before"], s["beta_before"],
                                        s["rho_before"], *s["action_raw"], *fp, fth, o[27], nbp, *nb, *bc, by, *s["prev_action"], vmax / 30.0])))
    out = subprocess.run([TOOL], input="\n".join(rows) + "\n", capture_output=True, text=True, check=True).stdout.split("\n")
    nsc = held_mis = 0
    for k, s in enumerate(st):
        c = [float(x) for x in out[k].split()]
        ob, beta, rho, tgt, sc, g, a, cl = np.array(c[:45]), c[45], c[46], c[47:52], c[52], c[53:56], c[56:59], c[59:62]
        o = np.array(s["obs"])
        tag = f"{f.stem}:{k}"
        i = 0
        for n, m in NAMES:
            upd("obs:" + n, np.abs(ob[i:i + m] - o[i:i + m]).max(), tag); i += m
        held_mis += ob[35] != o[35]
        upd("beta", abs(beta - s["beta"]), tag); upd("rho", abs(rho - s["rho"]), tag)
        upd("arm_target", np.abs(np.array(tgt) - np.array(s["arm_target"])).max(), tag)
        upd("vl_scale", abs(sc - s["vl_scale"]), tag)
        nsc += s["vl_scale"] < 1.0
        upd("grasp(FK 대 Isaac)", np.linalg.norm(np.array(g) - np.array(s["grasp_base"])), tag)
        upd("ee_a", np.linalg.norm(np.array(a) - np.array(s["ee_a_base"])), tag)
        upd("ee_c", np.linalg.norm(np.array(cl) - np.array(s["ee_c_base"])), tag)
    print(f"{f.name}: 스텝 {len(st)}, 옆 퍽 {e['nb_present']}, 줄인 스텝 {nsc}, 쥠 칸 다른 스텝 {held_mis}, 끝 {e['end']}")
print("스텝 최대 차이 (C++ − Isaac):")
for k, v in worst.items():
    print(f"  {k:<22} {v:.3g}   ({where[k]})")
