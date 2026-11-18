"""진단: yaw 계단 명령(+1 = 4.5 rad/s, 0.4 s) 응답을 조건별로 비교. 환경 변수 EFFORT(팔 최대 토크), GRAV(0/1), POSE(시작 자세)."""
import os, sys, json
import warp as wp
wp.config.enable_backward = False
sys.path.insert(0, "/home/pa4/git/lv2_next_module/261025_2117/rl1_arm_reach")
import argparse
from isaaclab.app import add_launcher_args, launch_simulation
ap = argparse.ArgumentParser(); add_launcher_args(ap); args = ap.parse_args()
import gymnasium as gym, torch
import waffle_rl
from waffle_rl.reach_env_cfg import RL1ArmReachV2EnvCfg, CONTACT_SENSORS
JI = int(os.environ.get("JI", "0"))
EFF = float(os.environ.get("EFFORT", "3.8")); GRAV = os.environ.get("GRAV", "1") == "1"
POSE = [float(x) for x in os.environ.get("POSE", "-1.0 0.0 0.0 0.0").split()]
cfg = RL1ArmReachV2EnvCfg(); cfg.scene.num_envs = 1; cfg.seed = 0
cfg.scene.robot.actuators["arm"].effort_limit_sim = EFF
SLEEP = os.environ.get("SLEEP")
if SLEEP is not None:
    cfg.scene.robot.spawn.articulation_props.sleep_threshold = float(SLEEP)
    cfg.scene.robot.spawn.articulation_props.stabilization_threshold = float(os.environ.get("STAB", "0.0"))
SELF = os.environ.get("SELF", "1") == "1"
if not SELF:
    cfg.scene.robot.spawn.articulation_props.enabled_self_collisions = False
MODE = os.environ.get("MODE", "pd")      # pd: v2 그대로, kd0: kd 0, torque: kp·kd 0 + yaw에 순수 토크 EFF
if MODE == "kd0":
    cfg.scene.robot.actuators["arm"].damping = 0.0
if MODE == "torque":
    cfg.scene.robot.actuators["arm"].stiffness = 0.0
    cfg.scene.robot.actuators["arm"].damping = 0.0
if not GRAV:
    cfg.sim.gravity = (0.0, 0.0, 0.0)
with launch_simulation(cfg, args):
    env = gym.make("Waffle-RL1-ArmReachV2", cfg=cfg); u = env.unwrapped; r = u.scene["robot"]
    env.reset()
    from waffle_rl.robot_cfg import ARM_JOINTS
    ids = [r.joint_names.index(j) for j in ARM_JOINTS]; yi = ids[0]
    pos = r.data.default_joint_pos.torch.clone(); pos[0, ids] = torch.tensor(POSE, device=u.device)
    r.write_joint_state_to_sim_index(position=pos, velocity=torch.zeros_like(pos))
    r.set_joint_position_target_index(target=pos)
    u.action_manager.reset()
    a = torch.zeros(1, 4, device=u.device)
    for k in range(10): env.step(a)
    rows = []
    for k in range(30):
        if MODE == "torque":
            eff = torch.zeros_like(r.data.joint_pos.torch); eff[0, ids[JI]] = EFF if k < 12 else 0.0
            r.set_joint_effort_target_index(target=eff)
            a[:] = 0.0
            for _ in range(u.cfg.decimation):
                r.set_joint_effort_target_index(target=eff); r.write_data_to_sim(); u.sim.step(render=False); r.update(u.physics_dt)
            u.scene.update(u.step_dt) if False else None
        else:
            a[:, 0] = 1.0 if k < 12 else 0.0
            env.step(a)
        cm = max(float(torch.linalg.norm(u.scene.sensors[n].data.force_matrix_w.torch[0], dim=-1).max()) for n in CONTACT_SENSORS)
        rows.append([round((k + 1) * u.step_dt, 3), round(float(r.data.joint_pos.torch[0, ids[JI]]), 4), round(float(r.data.joint_vel.torch[0, ids[JI]]), 3),
                     round(float(r.root_view.get_dof_projected_joint_forces().numpy()[0][ids[JI]]), 3), round(cm, 2)])
    q = [x[1] for x in rows]
    print("RES", json.dumps({"self_collision": SELF, "mode": MODE, "effort": EFF, "gravity": GRAV, "pose": POSE,
          "mean_speed_0_0.4s": round((q[11] - q[0]) / (11 * u.step_dt), 3), "max_qd": max(x[2] for x in rows),
          "max_contact_N": max(x[4] for x in rows), "rows_t_q_qd_physxF_contact": rows[:14]}), flush=True)
    from pxr import Usd as _U
    rp = u.sim.stage.GetPrimAtPath("/World/envs/env_0/Robot/Geometry/base_footprint")
    print("ROOTATTR", {a_.GetName(): a_.Get() for a_ in rp.GetAttributes() if "physxArticulation" in a_.GetName()}, flush=True)
    if os.environ.get("DUMP") == "1":
        from pxr import Usd
        st = u.sim.stage
        for p in Usd.PrimRange(st.GetPrimAtPath("/World/envs/env_0/Robot")):
            if p.GetName() == "RMX52_Base_Yaw":
                print("PRIM", p.GetPath(), p.GetTypeName(), p.GetMetadata("apiSchemas"), flush=True)
                for a_ in p.GetAttributes():
                    if a_.HasAuthoredValue():
                        print("ATTR", a_.GetName(), a_.Get(), flush=True)
                for r_ in p.GetRelationships():
                    print("REL", r_.GetName(), r_.GetTargets(), flush=True)
    env.close()
