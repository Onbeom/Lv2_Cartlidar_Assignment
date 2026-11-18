"""2026-11-11 (RL 261111 1, 미션 파트 5번 #154, 기준 3판 A4) FINE 정렬 팔 자세 후보 — Isaac B안 USD 측정 (모의).

  python align_probe.py --out <json>

대 하나 = 측정 하나:
  고정 (후보마다): 후보 자세에 두고 1 s. 팔 링크 · 손가락 – 차체 (base_link) 접촉 힘 최대, 자기 충돌 근사 거리 최소, 측정 관절.
  궤적 (C1 빼고 후보마다 5 개): DRIVE → 후보, 후보 → target_look L · R · I, 후보 → TRANSIT_TILT.
    노드 sendArm 과 같게: 시간 = max(1.0 s, 관절 최대 변화 / 0.5 rad/s), 끝 속도 0 3 차 보간 (JTC 한 점), 30 Hz 목표, 끝난 뒤 0.5 s 더.
    스텝마다 접촉 힘 최대 · 자기 충돌 근사 · 측정 관절 기록 (차체 CoACD 거리는 align_geom.py 로 따로).
  집게 벌림 0.010 m. 상자 · 퍽은 멀리 치움 (상자 거리는 align_geom 계산). 팔레트는 빈 채 (퍽 12 개 실은 상태는 못 넣음).
"""
import argparse
import json
import os
import sys

from isaaclab.app import AppLauncher

ap = argparse.ArgumentParser()
ap.add_argument("--out", required=True)
AppLauncher.add_app_launcher_args(ap)
a = ap.parse_args()
a.headless = True
app = AppLauncher(a).app
MAN = "/home/pa4/git/lv2_next_module/261110_2010_lie_rl/ws/urdf_and_usd/policy/manipulator"
sys.path.insert(0, MAN)

import gymnasium as gym  # noqa: E402
import torch  # noqa: E402
from isaaclab.sensors import ContactSensorCfg  # noqa: E402

import waffle_isaac  # noqa: E402,F401
from waffle_isaac.lieupright_env_cfg import WaffleLieUprightB4EnvCfg  # noqa: E402
from waffle_isaac.robot_cfg import BODY_LINK, FINGER_LINKS_REGEX  # noqa: E402

CAND = {"C4": [0, -1.64, 0.20, 1.88, 0], "C5": [0, -1.69, 0.35, 1.78, 0], "C6": [0, -1.74, 0.25, 1.88, 0], "C0": [0, -1.84, 0.35, 1.88, 0], "C1": [0, -1.59, 0.15, 1.88, 0]}
if os.environ.get("ALIGN_CAND"):  # 2026-11-11 2차: 후보를 JSON 으로 받음 (예: {"D1": [...]}), 고정만은 C1 처럼 이름에 "!static" 없음 = 궤적도 함
    CAND = json.loads(os.environ["ALIGN_CAND"])
DRIVE = [-0.0245, -1.6122, 1.2901, 1.1888, 0]
TL = {"TL_L": [-0.2976, -0.5214, 0.2755, 1.2058, 0], "TL_R": [0.2976, -0.5214, 0.2755, 1.2058, 0], "TL_I": [0, -0.8610, 0.4326, 1.3883, 0]}
TRANSIT = [0, -1.32032, 0.67766, 1.42805, 0]
JOBS = [dict(kind="static", name=c, q0=q, q1=q) for c, q in CAND.items()]
for c, q in CAND.items():
    if c == "C1":
        continue
    JOBS.append(dict(kind="traj", name=f"DRIVE->{c}", q0=DRIVE, q1=q))
    for k, v in TL.items():
        JOBS.append(dict(kind="traj", name=f"{c}->{k}", q0=q, q1=v))
    JOBS.append(dict(kind="traj", name=f"{c}->TRANSIT_TILT", q0=q, q1=TRANSIT))


def main():
    cfg = WaffleLieUprightB4EnvCfg()
    n = len(JOBS)
    cfg.scene.num_envs = n
    cfg.start_probs = (1.0, 0.0, 0.0, 0.0, 0.0)
    cfg.episode_length_s = 120.0
    cfg.scene.finger_body_contact = ContactSensorCfg(prim_path="{ENV_REGEX_NS}/Robot/" + FINGER_LINKS_REGEX, filter_prim_paths_expr=["{ENV_REGEX_NS}/Robot/" + BODY_LINK],
                                                     update_period=0.0, history_length=1)
    env = gym.make("Waffle-LieUprightB4-Direct", cfg=cfg)
    u = env.unwrapped
    dev = u.device
    env.reset()
    ids = torch.arange(n, device=dev)
    far = torch.tensor([[3.0, 3.0]], device=dev).expand(n, 2)
    z0 = torch.zeros(n, device=dev)
    u._place("boxA", ids, far, z0, torch.full((n,), 0.035, device=dev))
    for k, dx in (("puckT_box", 0.4), ("puckT_cyl", 0.6), ("puckN", 0.8)):
        u._place(k, ids, far + torch.tensor([dx, 0.0], device=dev), z0, torch.full((n,), 0.031, device=dev))
    d = u.robot.data
    Q0 = torch.tensor([j["q0"] for j in JOBS], device=dev)
    Q1 = torch.tensor([j["q1"] for j in JOBS], device=dev)
    q = d.joint_pos.torch.clone()
    q[:, u._arm] = Q0
    q[:, u._gl[0]] = 0.010
    q[:, u._gr[0]] = 0.010
    u.robot.write_joint_position_to_sim_index(position=q, env_ids=ids)
    u.robot.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(q), env_ids=ids)
    u.arm_target[:] = Q0
    f = lambda k: torch.linalg.norm(u.scene[k].data.normal_force_matrix_w.torch, dim=-1).amax(dim=(1, 2))  # noqa: E731

    def tick():
        u.actions[:, -1] = 1.0
        for _ in range(cfg.decimation):
            u._apply_action()
            u.scene.write_data_to_sim()
            u.sim.step(render=False)
            u.scene.update(u.physics_dt)

    for _ in range(15):  # 0.5 s 자리 잡기 (시작 자세)
        tick()
    dt = u.step_dt
    T = torch.clamp((Q1 - Q0).abs().amax(1) / 0.5, min=1.0)
    static = torch.tensor([j["kind"] == "static" for j in JOBS], device=dev)
    T = torch.where(static, torch.full_like(T, 1.0), T)  # 고정: 1 s
    steps = int((T.max().item() + 0.5) / dt) + 1
    rec = [[] for _ in range(n)]
    for k in range(steps):
        t = k * dt
        s = (t / T).clamp(0, 1)
        h = 3 * s ** 2 - 2 * s ** 3
        u.arm_target[:] = Q0 + (Q1 - Q0) * h[:, None]
        tick()
        fa, ff, sd = f("arm_body_contact"), f("finger_body_contact"), u.self_dist()
        qm = d.joint_pos.torch[:, u._arm]
        for i in range(n):
            if t <= float(T[i]) + 0.5:
                rec[i].append([round(t, 4), float(fa[i]), float(ff[i]), float(sd[i])] + [float(v) for v in qm[i]])
    out = []
    for i, j in enumerate(JOBS):
        R = rec[i]
        out.append(dict(job=j, T=float(T[i]), steps=R, arm_body_max=max(r[1] for r in R), finger_body_max=max(r[2] for r in R), self_dist_min=min(r[3] for r in R),
                        q_end=R[-1][4:]))
        print(f"[Align] {j['kind']:6s} {j['name']:22s} T {float(T[i]):.2f} s | 팔-몸체 최대 {out[-1]['arm_body_max']:.1f} N · 손가락-몸체 {out[-1]['finger_body_max']:.1f} N | 자기 충돌 근사 최소 {out[-1]['self_dist_min'] * 1000:.1f} mm | 끝 관절 {[round(v, 3) for v in out[-1]['q_end']]}", flush=True)
    json.dump(dict(note="Isaac B안 USD 모의, 상자 · 퍽 치움, 빈 팔레트, 집게 0.010", jobs=out), open(a.out, "w"))
    env.close()


if __name__ == "__main__":
    main()
    app.close()
