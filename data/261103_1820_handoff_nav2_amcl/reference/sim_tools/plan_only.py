"""도구 확인(판정 아님): 계획기만 띄운 상태(plan_only.sh)에서 목표 목록의 구간마다 ComputePathToPose(use_start)를 불러 경로를 받고,
경로 자세마다 차체 여유(geom.clearance, 기둥 포함 참값 형상)를 잰다.
쓸림 여유(min_clearance_sweep_m): 이웃 자세 사이를 회전 2° · 이동 0.005 m 이하 간격으로 채운 자세들의 최소 여유(중간 자세 없는 제자리 회전 원시 동작 확인용, 설계 31). 목표는 run_goals --base_offset_x 와 같이 회전 중심 점으로 보낸다.
사용: python3 plan_only.py --goals <json> --out <json> [--base_offset_x -0.0786] [--start 0.734 0.95 3.14159265] [--legs 0 1]"""
import argparse
import json
import math
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient
from rclpy.node import Node

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE / 'nodes'))
import geom  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument('--goals', required=True)
ap.add_argument('--out', required=True)
ap.add_argument('--frame', default='mission_map')
ap.add_argument('--base_offset_x', type=float, default=-0.0786)
ap.add_argument('--start', type=float, nargs=3, default=[0.734, 0.95, math.pi])
ap.add_argument('--legs', type=int, nargs='*', default=None, help='몇 번째 구간만(0 = 출발 → 첫 목표)')
a = ap.parse_args()
OX = a.base_offset_x


def to_rc(x, y, t):
    return x + OX * math.cos(t), y + OX * math.sin(t), t


def to_base(x, y, t):
    return x - OX * math.cos(t), y - OX * math.sin(t), t


def pose(node, x, y, t):
    p = PoseStamped()
    p.header.frame_id = a.frame
    p.header.stamp = node.get_clock().now().to_msg()
    p.pose.position.x, p.pose.position.y = x, y
    p.pose.orientation.z, p.pose.orientation.w = math.sin(t / 2), math.cos(t / 2)
    return p


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


goals = json.loads(Path(a.goals).read_text())['goals']
pts = [('start', *a.start)] + [(g['name'], g['x'], g['y'], g['yaw']) for g in goals]
B = geom.load_world()['body']
rclpy.init()
node = Node('plan_only')
cli = ActionClient(node, ComputePathToPose, 'compute_path_to_pose')
assert cli.wait_for_server(timeout_sec=60), 'compute_path_to_pose 없음'
legs = []
for i in range(len(pts) - 1):
    if a.legs is not None and i not in a.legs:
        continue
    (n0, *s), (n1, *g) = pts[i], pts[i + 1]
    req = ComputePathToPose.Goal()
    req.start, req.goal = pose(node, *to_rc(*s)), pose(node, *to_rc(*g))
    req.use_start, req.planner_id = True, 'GridBased'
    t0 = time.time()
    for _ in range(120):             # 계획기 활성화 전이면 거절됨 → 받아들일 때까지
        fut = cli.send_goal_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=30)
        h = fut.result()
        if h.accepted:
            break
        time.sleep(1.0)
    t0 = time.time()
    rf = h.get_result_async()
    rclpy.spin_until_future_complete(node, rf, timeout_sec=60)
    res = rf.result().result
    rc = [(p.pose.position.x, p.pose.position.y, yaw_of(p.pose.orientation)) for p in res.path.poses]
    base = [to_base(*p) for p in rc]
    clr = [geom.clearance(*p, B) for p in base]
    sweep = []
    for (x0, y0, t0_), (x1, y1, t1) in zip(rc, rc[1:]):
        dt = math.remainder(t1 - t0_, 2 * math.pi)
        m = max(1, math.ceil(abs(math.degrees(dt)) / 2.0), math.ceil(math.hypot(x1 - x0, y1 - y0) / 0.005))
        for j in range(1, m):
            sweep.append(geom.clearance(*to_base(x0 + (x1 - x0) * j / m, y0 + (y1 - y0) * j / m, t0_ + dt * j / m), B))
    sweep += clr
    # 진행 방향(앞 · 뒤): 이동 벡터와 자세 방향의 내적
    dirs = []
    for (x0, y0, t), (x1, y1, _) in zip(rc, rc[1:]):
        d = (x1 - x0) * math.cos(t) + (y1 - y0) * math.sin(t)
        if abs(d) > 1e-4:
            dirs.append(1 if d > 0 else -1)
    cusps = sum(1 for u, v in zip(dirs, dirs[1:]) if u != v)
    length = sum(math.hypot(x1 - x0, y1 - y0) for (x0, y0, _), (x1, y1, _) in zip(rc, rc[1:]))
    turn = sum(math.remainder(t1 - t0_, 2 * math.pi) for (_, _, t0_), (_, _, t1) in zip(rc, rc[1:]))
    k = min(range(len(clr)), key=clr.__getitem__) if clr else None
    leg = dict(i=i, frm=n0, to=n1, error_code=int(res.error_code), error_msg=res.error_msg, n=len(rc),
               plan_s=round(time.time() - t0, 2), planning_time_s=res.planning_time.sec + res.planning_time.nanosec * 1e-9,
               length_m=round(length, 3), net_turn_deg=round(math.degrees(turn), 1), reverse_frac=round(dirs.count(-1) / max(1, len(dirs)), 3),
               cusps=cusps, min_clearance_m=round(min(clr), 4) if clr else None,
               worst_base=[round(v, 4) for v in base[k]] if clr else None, n_overlap=sum(c < 0 for c in clr),
               min_clearance_sweep_m=round(min(sweep), 4) if sweep else None, n_overlap_sweep=sum(c < 0 for c in sweep),
               path_rc=[[round(v, 4) for v in p] for p in rc])
    legs.append(leg)
    print(f"[{i}] {n0} → {n1}: code {leg['error_code']} {leg['error_msg']} n {leg['n']} 길이 {leg['length_m']} 회전 {leg['net_turn_deg']}° "
          f"후진 {leg['reverse_frac']} 끊음 {cusps} 최소 여유 {leg['min_clearance_m']} 겹침 {leg['n_overlap']} 쓸림 {leg['min_clearance_sweep_m']} / {leg['n_overlap_sweep']} 시간 {leg['plan_s']} s", flush=True)
Path(a.out).write_text(json.dumps(dict(goals=a.goals, base_offset_x=OX, start=a.start, legs=legs), ensure_ascii=False, indent=1))
node.destroy_node()
rclpy.shutdown()
