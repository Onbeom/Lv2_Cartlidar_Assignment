"""2026-11-11 (RL 261111 1) A단계 도구 확인 뒤 진단: 고정 자세 (ALIGN_A · target_look L · 상자 보기 left · READY45) 에 팔을 두고
팔-몸체 접촉 힘, 자기 충돌 근사 값 (self_dist · armcol 깊이 · 관절 3) 을 잼. 모의."""
import os, sys
from isaaclab.app import AppLauncher
import argparse
ap = argparse.ArgumentParser(); AppLauncher.add_app_launcher_args(ap); a = ap.parse_args(); a.headless = True
app = AppLauncher(a).app
W = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(W, "ws", "urdf_and_usd", "policy", "manipulator"))
import gymnasium as gym, torch  # noqa
import waffle_isaac  # noqa
from waffle_isaac.lieupright_env_cfg import WaffleLieUprightB4EnvCfg  # noqa
cfg = WaffleLieUprightB4EnvCfg(); cfg.scene.num_envs = 2; cfg.start_probs = (1.0, 0, 0, 0, 0); cfg.episode_length_s = 120.0
env = gym.make("Waffle-LieUprightB4-Direct", cfg=cfg); u = env.unwrapped; env.reset()
d = u.robot.data; ids = torch.arange(2, device=u.device)
P = {"ALIGN_A": [0.0, -1.854, 0.540, 1.645, 0.0], "TL_L": [-0.2976, -0.5214, 0.2755, 1.2058, 0.0], "READY45": [0.0, -0.338, 0.559, 0.564, 0.0]}
for side, j1 in (("L", -0.300), ("R", 0.300), ("I", 0.0)):  # 2026-11-11 미션 파트 보기 자세 후보 (지금 값 R 은 +0.305)
    P[f"view_now_{side}"] = [j1 if side != "R" else 0.305, -1.80, 0.50, 1.90, 0.0]
    P[f"view_A_{side}"] = [j1, -1.25, 0.15, 1.85, 0.0]
    P[f"view_B_{side}"] = [j1, -1.15, 0.15, 1.80, 0.0]
f = lambda k: torch.linalg.norm(u.scene[k].data.normal_force_matrix_w.torch, dim=-1).amax(dim=(1, 2))  # noqa
print("self_margin", cfg.self_margin, "j3_bend_end", cfg.j3_bend_end, "self_box_tol", cfg.self_box_tol, "센서", [k for k in u.scene.keys() if "contact" in k], flush=True)
for nm, qv in P.items():
    q = d.joint_pos.torch.clone(); q[:, u._arm] = torch.tensor(qv, device=u.device)
    u.robot.write_joint_position_to_sim_index(position=q, env_ids=ids); u.robot.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(q), env_ids=ids)
    u.arm_target[:] = torch.tensor(qv, device=u.device)
    mx = 0.0
    for t in range(30):
        u.actions[:, -1] = 1.0
        for _ in range(cfg.decimation):
            u._apply_action(); u.scene.write_data_to_sim(); u.sim.step(render=False); u.scene.update(u.physics_dt)
        mx = max(mx, float(f("arm_body_contact").max()))
    print(f"[Probe] {nm}: 팔-몸체 힘 최대 {mx:.1f} N (마지막 {float(f('arm_body_contact').max()):.1f}), self_dist {float(u.self_dist().min())*1000:.1f} mm, armcol 깊이 {float(u.armcol.depth().max())*1000:.1f} mm, "
          f"관절 3 {float(d.joint_pos.torch[0, u._arm[2]]):.3f}, 측정 관절 {[round(v, 3) for v in d.joint_pos.torch[0, u._arm].tolist()]}", flush=True)
env.close(); app.close()
