"""팔 계획기 새 모델 이식 도구 확인 (모의 아님 · 계산만).
1) 이전 모델(261027_1711): 고치기 전 arm_pose(before/)와 지금 arm_pose 가 같은 입력에 같은 관절 · 카메라 자세 · 충돌 결과를 내는지.
2) 새 모델(261028 light): ArmRealizer.cam 이 독립 FK(261029_new_model_audit/model_frames.py)의 base_footprint → arm_camera_optical_frame,
   tool_link 와 맞는지(무작위 관절 200개).
3) 새 모델 자세 drive · zero · stow 충돌 판정(빈 장면) — drive · zero 는 겹침 없음이 예상.
4) 새 모델 IK: 앞 0.30 m · 높이 0.20 m 벽을 수평으로 보는 카메라 자세 풀이가 되는지.
"""
import importlib.util
import json
import math
import sys
from pathlib import Path

import numpy as np

TOP = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(TOP / "Cartlidar/sim_render"))
sys.path.insert(0, str(TOP / "Cartlidar/sim_world"))
sys.path.insert(0, str(TOP / "261029_new_model_audit"))
import arm_pose  # noqa: E402
from model_frames import ModelFrames  # noqa: E402
from urdf_chain import planar_pose  # noqa: E402

spec = importlib.util.spec_from_file_location("arm_pose_before", TOP / "261031_1718_r3port/before/arm_pose.py")
before = importlib.util.module_from_spec(spec)
spec.loader.exec_module(before)


class S:
    boxes = [(0.70, 0.0, 0.0115, 1.0, 0.40, "wall")]


out = {}
# 1) 이전 모델 회귀
U0 = TOP / "261027_1711/urdf/waffle_4wd_description_261027_1711/urdf/waffle_4wd.urdf"
a_new = arm_pose.ArmRealizer(U0, U0.parent.parent, S(), {"pucks": []}, gripper=0.0)
a_old = before.ArmRealizer(U0, U0.parent.parent, S(), {"pucks": []}, gripper=0.0)
rng = np.random.default_rng(1)
lo, hi = [l for l, _ in a_new.lim], [h for _, h in a_new.lim]
dmax, hit_same = 0.0, 0
T_bf = planar_pose(0.3, 0.1, 0.4)
for _ in range(200):
    v = rng.uniform(lo, hi)
    dmax = max(dmax, np.abs(a_new.cam(T_bf, v) - a_old.cam(T_bf, v)).max(),
               np.abs(a_new.cam(T_bf, v, "arm_link_5") - a_old.cam(T_bf, v, "arm_link_5")).max())
    hit_same += a_new.arm_hits(T_bf, v) == a_old.arm_hits(T_bf, v)
seeds = [rng.uniform(lo, hi) for _ in range(8)]
T_des = a_old.cam(T_bf, [0.0, 0.4, 0.6, 0.6, 0.0])
f_new, f_old = a_new.frame(T_bf, T_des, seeds), a_old.frame(T_bf, T_des, seeds)
same_frame = (f_new is None and f_old is None) or (f_new is not None and f_old is not None and
                                                   np.allclose(f_new[0], f_old[0]) and f_new[1] == f_old[1])
t_new = a_new.frame_tcp(T_bf, [0.55, 0.15, 0.08], 0.0, seeds)
t_old = a_old.frame_tcp(T_bf, [0.55, 0.15, 0.08], 0.0, seeds)
same_tcp = (t_new is None and t_old is None) or (t_new is not None and t_old is not None and np.allclose(t_new[0], t_old[0]))
out["old_model"] = {"fk_max_abs_diff": float(dmax), "arm_hits_same": f"{hit_same}/200", "frame_same": bool(same_frame),
                    "frame_tcp_same": bool(same_tcp), "body_shapes_same": len(a_new.body) == len(a_old.body) and
                    all(np.allclose(x[1], y[1]) for x, y in zip(a_new.body, a_old.body))}

# 2) 새 모델 FK 대조
U1 = TOP / "261029_1030/urdf_261028/isaac/urdf/waffle_4wd_light.urdf"
a = arm_pose.ArmRealizer(U1, U1.parent, S(), {"pucks": []}, gripper=0.0)
mf = ModelFrames(U1)
lo, hi = [l for l, _ in a.lim], [h for _, h in a.lim]
d_cam, d_tcp = 0.0, 0.0
for _ in range(200):
    v = rng.uniform(lo, hi)
    q = dict(zip(a.arm_j, v))
    ref = mf.transform("base_footprint", "arm_camera_optical_frame", q)
    d_cam = max(d_cam, np.abs(T_bf @ ref - a.cam(T_bf, v)).max())
    ref_t = (T_bf @ mf.transform("base_footprint", "tool_link", q))[:3, 3]
    d_tcp = max(d_tcp, np.abs(ref_t - (a.cam(T_bf, v, "gripper_roll_link") @ a.prof["tcp"])[:3]).max())
out["new_model"] = {"profile_joints": a.arm_j, "cam": a.cam_frame, "root": a.rb.root, "arm_links": list(a.arm_links),
                    "fk_cam_max_abs_diff": float(d_cam), "fk_tcp_max_abs_diff": float(d_tcp),
                    "T_bf_root_z": float(a.T_bf_root[2, 3])}

# 3) 자세별 충돌 (벽은 0.70 m 앞이라 닿지 않음)
P = json.loads((TOP / "261029_1030/urdf_261028/isaac/robot_params.json").read_text())["poses"]
T0 = planar_pose(0.0, 0.0, 0.0)
out["pose_hits"] = {k: a.arm_hits(T0, [P[k]["value"][j] for j in a.arm_j]) for k in ("drive", "zero", "stow")}
# 바닥 높이: drive 에서 팔 최저점
arm = a.arm_shapes(T0, [P["drive"]["value"][j] for j in a.arm_j])
out["drive_arm_min_z"] = float(min(V[:, 2].min() for _, V in arm))
out["body_min_z"] = float(min(V[:, 2].min() for _, V in a.body))

# 4) 벽 수평 보기 IK (벽 면 x = 0.70 − 0.0115, 카메라 0.12 m 앞, 높이 0.20)
x_wall = 0.70 - 0.0115
T = np.eye(4)
z = np.array([1.0, 0.0, 0.0]); xx = np.array([0.0, -1.0, 0.0])
T[:3, 0], T[:3, 1], T[:3, 2], T[:3, 3] = xx, np.cross(z, xx), z, [x_wall - 0.12, 0.0, 0.20]
seeds4 = a.prof["seeds"] + [rng.uniform(lo, hi) for _ in range(12)]
r = a.frame(T0, T, seeds4)
out["ik_wall"] = None if r is None else {"q": [round(float(x), 4) for x in r[0]], "backoff": r[1],
                                         "pos_err": float(np.linalg.norm(r[2][:3, 3] - T[:3, 3]))}
print(json.dumps(out, indent=1, ensure_ascii=False))
