"""RL1 목표 표 만들기: 관절 한계 안 무작위 자세 → FK TCP 위치, 바닥·자기충돌 걸러냄.

- 팔 4관절(yaw·shoulder·elbow·pitch)을 URDF 한계 안에서 고르게 뽑는다. roll 0, 집게는 열림(상한) 고정.
  한계를 넓히거나 값을 자르지 않는다.
- 바닥 거름: 팔 링크·집게 원점·손가락 끝(집게 링크 x +0.065 m, 충돌 메시 끝)의 z가 FLOOR_MARGIN 미만이면 뺀다.
- 자기충돌: move_group /check_state_validity(그룹 arm)로 충돌 0쌍인 자세만 남긴다.
- 출력 npz: q(N, 4) rad, tcp(N, 3) m(base_footprint 기준), joint_names, 조건 기록.

사용(PC, move_group_headless.launch.py 실행 중, ROS 2 Lyrical source 후):
  python3 make_targets.py --n 20000 --seed 1 --out ../results/targets_train.npz
"""
import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from urdf_fk import UrdfFK  # noqa: E402

ARM = ["RMX52_Base_Yaw", "RMX52_Shoulder", "RMX52_Elbow", "RMX52_Wrist_Pitch"]
ROLL = "RMX52_Wrist_Roll_Reversed"
GRIPPER = "gripper_left_joint"
TCP_OFFSET_ARM5 = np.array([0.0, 0.0, -0.08295])  # 두 손가락 충돌 메시의 가운데 (arm_link_5 좌표, m). reach_env_cfg.TCP_OFFSET과 같게
FINGER_TIP_X = 0.065                               # 집게 링크 좌표에서 손가락 충돌 메시 끝 x (m)
FLOOR_MARGIN = 0.01                                # 바닥(z = 0)에서 띄울 거리 (m)


def tcp_and_floor(fk, q4, grip):
    n = len(q4)
    q = {name: q4[:, i] for i, name in enumerate(ARM)}
    q[ROLL] = np.zeros(n)
    q[GRIPPER] = np.full(n, grip)
    links = ["arm_link_1", "arm_link_2", "arm_link_3", "arm_link_4", "arm_link_5",
             "gripper_left_link", "gripper_right_link"]
    T = fk.link_transforms(q, links)
    tcp = (T["arm_link_5"][:, :3, :3] @ TCP_OFFSET_ARM5 + T["arm_link_5"][:, :3, 3])
    zs = [T[l][:, 2, 3] for l in links]
    for g in ("gripper_left_link", "gripper_right_link"):
        tip = T[g][:, :3, :3] @ np.array([FINGER_TIP_X, 0, 0]) + T[g][:, :3, 3]
        zs.append(tip[:, 2])
    return tcp, np.min(np.stack(zs, 1), 1)


def check_collisions(q4, grip, batch=8):  # 서비스 요청 대기열(기본 깊이 10)을 넘지 않게
    import rclpy
    from moveit_msgs.msg import RobotState
    from moveit_msgs.srv import GetStateValidity
    from sensor_msgs.msg import JointState

    rclpy.init()
    node = rclpy.create_node("rl1_make_targets")
    cli = node.create_client(GetStateValidity, "/check_state_validity")
    if not cli.wait_for_service(timeout_sec=20.0):
        sys.exit("/check_state_validity 없음: move_group_headless.launch.py를 먼저 실행한다")
    names = ARM + [ROLL, GRIPPER]
    valid = np.zeros(len(q4), bool)
    pairs = {}
    for s in range(0, len(q4), batch):
        futs = []
        for i in range(s, min(s + batch, len(q4))):
            req = GetStateValidity.Request()
            req.group_name = "arm"
            req.robot_state = RobotState(
                joint_state=JointState(name=names, position=[*map(float, q4[i]), 0.0, float(grip)]), is_diff=True)
            futs.append((i, cli.call_async(req)))
        for i, f in futs:
            rclpy.spin_until_future_complete(node, f, timeout_sec=10.0)
            r = f.result()
            if r is None:
                sys.exit(f"응답 없음: {i}")
            valid[i] = r.valid
            for c in r.contacts:
                key = "|".join(sorted((c.contact_body_1, c.contact_body_2)))
                pairs[key] = pairs.get(key, 0) + 1
    node.destroy_node()
    rclpy.shutdown()
    return valid, pairs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, required=True, help="뽑을 자세 수(거르기 전)")
    ap.add_argument("--seed", type=int, required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--no-moveit", action="store_true", help="자기충돌 검사 생략(도구 시험용)")
    a = ap.parse_args()

    fk = UrdfFK()
    lo = np.array([fk.joints[j]["lower"] for j in ARM])
    hi = np.array([fk.joints[j]["upper"] for j in ARM])
    grip = fk.joints[GRIPPER]["upper"]
    rng = np.random.default_rng(a.seed)
    q4 = rng.uniform(lo, hi, size=(a.n, 4))
    tcp, zmin = tcp_and_floor(fk, q4, grip)
    floor_ok = zmin >= FLOOR_MARGIN
    t0 = time.time()
    if a.no_moveit:
        coll_ok, pairs = np.ones(a.n, bool), {}
    else:
        coll_ok = np.zeros(a.n, bool)
        idx = np.flatnonzero(floor_ok)
        v, pairs = check_collisions(q4[idx], grip)
        coll_ok[idx] = v
    keep = floor_ok & coll_ok
    info = {"n_sampled": a.n, "seed": a.seed, "n_floor_ok": int(floor_ok.sum()),
            "n_kept": int(keep.sum()), "moveit_checked": not a.no_moveit, "moveit_s": round(time.time() - t0, 1),
            "lower": lo.tolist(), "upper": hi.tolist(), "gripper": grip, "roll": 0.0,
            "tcp_offset_arm5_m": TCP_OFFSET_ARM5.tolist(), "floor_margin_m": FLOOR_MARGIN,
            "collision_pairs": dict(sorted(pairs.items(), key=lambda kv: -kv[1])),
            "tcp_min_m": tcp[keep].min(0).round(4).tolist(), "tcp_max_m": tcp[keep].max(0).round(4).tolist()}
    out = Path(a.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    np.savez(out, q=q4[keep], tcp=tcp[keep], joint_names=np.array(ARM), info=json.dumps(info))
    out.with_suffix(".json").write_text(json.dumps(info, ensure_ascii=False, indent=1))
    print(json.dumps({k: v for k, v in info.items() if k != "collision_pairs"}, ensure_ascii=False))
    print("충돌 쌍 상위:", list(info["collision_pairs"].items())[:8])


if __name__ == "__main__":
    main()
