"""진단: v2 환경, 표 리셋 자세에서 행동 0으로 2 s → 관절별 위치·속도·위치 목표·속도 목표·계산 토크."""
import sys, json
import warp as wp
wp.config.enable_backward = False
sys.path.insert(0, "/home/pa4/git/lv2_next_module/261025_2117/rl1_arm_reach")
import argparse
from isaaclab.app import add_launcher_args, launch_simulation
ap = argparse.ArgumentParser(); add_launcher_args(ap); args = ap.parse_args()
import gymnasium as gym, torch
import waffle_rl
from waffle_rl.reach_env_cfg import RL1ArmReachV2EnvCfg
cfg = RL1ArmReachV2EnvCfg(); cfg.scene.num_envs = 2; cfg.seed = 0
with launch_simulation(cfg, args):
    env = gym.make("Waffle-RL1-ArmReachV2", cfg=cfg); u = env.unwrapped; r = u.scene["robot"]
    env.reset()
    names = r.joint_names
    def dump(tag):
        d = r.data
        print(tag, json.dumps({n: [round(float(d.joint_pos.torch[0, i]), 4), round(float(d.joint_vel.torch[0, i]), 4),
              round(float(d.joint_pos_target.torch[0, i]), 4), round(float(d.joint_vel_target.torch[0, i]), 4),
              round(float(d.computed_torque.torch[0, i]), 3)] for i, n in enumerate(names) if not n.startswith("Wheel")}), flush=True)
    dump("after_reset [q, qd, q*, qd*, tau]")
    a = torch.zeros(2, 4, device=u.device)
    for k in range(60):
        env.step(a)
        if k in (0, 1, 5, 29, 59): dump(f"step{k+1}")
    env.close()
