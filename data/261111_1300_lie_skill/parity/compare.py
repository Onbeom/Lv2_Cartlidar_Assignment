"""r2e Isaac 기록 (handoff/r2e_parity) 과 C++ lieupright.hpp 계산을 스텝마다 비교 (모의 기록 대 C++, 도구 · 판정).
관측 45 (C++ 가 FK · 상태로 만드는 칸), β · ρ, arm_target(5), 잡는 점 · 집게 축, 쥠 (손가락만 히스테리시스) 를 본다."""
import json, math, subprocess, sys
from pathlib import Path
import numpy as np
D = Path("/home/pa4/git/lv2_next_module/261110_2010_lie_rl/handoff/r2e_parity")
NAMES = [("joint_pos_rel", 5), ("joint_vel", 5), ("finger_pos", 1), ("puck_pos", 3), ("long_axis", 2), ("grasp_pos", 3), ("approach", 3),
         ("close", 3), ("stand_spot", 3), ("nb_rel", 2), ("nb_present", 1), ("box_c", 2), ("box_yaw", 2), ("held", 1), ("rho_beta", 3), ("prev_action", 6)]
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from gen_ref import stand_spot  # noqa: E402  학습 _stand_spot 식 (torch)
import torch  # noqa: E402
worst = {}
def upd(k, v):
    worst[k] = max(worst.get(k, 0.0), float(v))
for f in sorted(D.glob("ep*.json")):
    d = json.loads(f.read_text())
    e, st = d["episode"], d["episode"]["steps"]
    rows = []
    for s in st:
        o = s["obs"]
        rows.append(" ".join(map(repr, [*s["q"], *s["qd"], s["finger_left"], *s["arm_target_before"], s["beta_before"], s["rho_before"],
                                        *s["action_raw"], *o[11:14], *o[14:16], *o[25:28], o[30], *o[28:30], *o[31:33],
                                        math.atan2(o[34], o[33]), o[35], *s["prev_action"]])))
    out = subprocess.run(["./parity_tool"], input="\n".join(rows) + "\n", capture_output=True, text=True, check=True).stdout.split("\n")
    # 쥠: 손가락만 히스테리시스 (실물 판정) 를 기록 ho_finger 와
    on = off = 0; state = False; hold_mis = 0
    for k, s in enumerate(st):
        c = [float(x) for x in out[k].split()]
        obs, beta, rho, tgt, g, a, cl = c[:45], c[45], c[46], c[47:52], c[52:55], c[55:58], c[58:61]
        o = np.array(s["obs"]); ob = np.array(obs)
        i = 0
        for n, m in NAMES:
            upd("obs:" + n, np.abs(ob[i:i + m] - o[i:i + m]).max()); i += m
        upd("beta", abs(beta - s["beta"])); upd("rho", abs(rho - s["rho"]))
        upd("arm_target", np.abs(np.array(tgt) - np.array(s["arm_target"])).max())
        upd("grasp_base(FK 대 Isaac)", np.linalg.norm(np.array(g) - np.array(s["grasp_base"])))
        upd("ee_a", np.linalg.norm(np.array(a) - np.array(s["ee_a_base"]))); upd("ee_c", np.linalg.norm(np.array(cl) - np.array(s["ee_c_base"])))
        # 히스테리시스: 학습은 직전 물리 뒤 (= 이 스텝 관측의 손가락) 로 갱신 → 노드도 이 tick 손가락으로 갱신한 뒤 관측에 넣음
        raw = -0.0108 < s["finger_left"] < -0.003
        on = on + 1 if raw else 0; off = 0 if raw else off + 1
        state = True if on >= 8 else (False if off >= 3 else state)
        hold_mis += state != bool(s["ho_finger"])
    # 세울 자리: 옆 퍽 없으면 '있었다면' 자리 = 상자 중심 대칭 (C++ 와 같은 근사) 으로 학습 식
    bc = torch.tensor([e["box_c_base"]], dtype=torch.float64); by = torch.tensor([e["box_yaw"]], dtype=torch.float64)
    nb = e["nb_true_base0"][:2] if e["nb_present"] else [2 * e["box_c_base"][0] - e["puck_true_base0"][0], 2 * e["box_c_base"][1] - e["puck_true_base0"][1]]
    Dm, _ = stand_spot(bc, by, torch.tensor([nb], dtype=torch.float64))
    dD = np.linalg.norm(Dm[0].numpy() - np.array(e["D_base"][:2]))
    print(f"{f.name}: 스텝 {len(st)}, 옆 퍽 {e['nb_present']}, 쥠(손가락만) 기록과 다른 스텝 {hold_mis}, 세울 자리 D 차이 {dD * 1000:.2f} mm"
          f"{' (옆 퍽 없음: 대칭 근사)' if not e['nb_present'] else ''}")
print("스텝 최대 차이 (C++ − Isaac):")
for k, v in worst.items():
    print(f"  {k:<28} {v:.3g}")
