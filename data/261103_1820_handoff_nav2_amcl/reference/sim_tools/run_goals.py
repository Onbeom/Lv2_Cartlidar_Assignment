"""모의 확인: goals json 의 목표를 차례로 NavigateToPose(/navigate_to_pose) 로 보내고 기록한다.
기록: 목표별 결과 상태 · 걸린 모의 시간 · 끝난 순간의 참값 자세(/sim/truth), 10 Hz 위치 추정 오차(TF <--frame, 기본 map>→base_link 최신 값 vs 같은 stamp 참값 보간),
/cmd_vel_nav 메시지(frame · 속도 최대). 결과 = --out json. 채점은 evaluate.py."""
import argparse
import bisect
import json
import math
import time

import rclpy
import tf2_ros
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, TwistStamped
import numpy as np
from nav2_msgs.action import BackUp, DriveOnHeading, NavigateToPose, Spin
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time

FP_RC = (-0.0764, 0.3416, 0.171)    # Nav2 footprint(rotation_center 기준 x 뒤 · 앞, |y|) = nav2_params footprint

ap = argparse.ArgumentParser()
ap.add_argument('--goals', required=True)
ap.add_argument('--out', required=True)
ap.add_argument('--goal_timeout', type=float, default=240.0)   # 모의 초
ap.add_argument('--settle', type=float, default=3.0)           # 목표 끝난 뒤 멈춤 확인 시간(모의 초)
ap.add_argument('--base_offset_x', type=float, default=0.0)   # 설계 20: 목표를 base_link 자세 ∘ (이 값, 0) 점으로 보냄(rotation_center = x_icr). 채점은 base_link 그대로
ap.add_argument('--final_spin_tol', type=float, default=0.0)   # 설계 32: > 0 이면 SUCCEEDED 뒤 추정 방향(frame→base_link)과 목표 방향 차가 이 값보다 클 때 Spin 행동으로 맞춤
ap.add_argument('--final_spin_max', type=int, default=1)       # 설계 34: 방향 차가 허용 안에 들 때까지 Spin 을 이 횟수까지 반복(1 = 설계 32 · 33 과 같음)
ap.add_argument('--loc_retry', type=int, default=0)          # 11-02: > 0 이면 목표가 실패했고 그 사이 /localization/status 가 OK 가 아니었던 적이 있으면 OK 로 돌아올 때까지(최대 60 모의 초) 기다렸다 같은 목표를 이 횟수까지 다시 보냄(위치 다시 찾은 뒤 주행 이어 가기, K2 · K3 에서 G1 실패)
ap.add_argument('--escape', type=int, default=0)             # 11-02: > 0 이면 목표가 'Start occupied'(205)로 실패할 때 이 횟수까지 빠져나오기(Runner.escape) 뒤 같은 목표를 다시 보냄
ap.add_argument('--escape_clear', type=float, default=0.08)  # 빠져나온 자리의 목표 차체 여유(m, 전역 costmap 치명 칸까지) — 여백 0.04 + 칸 0.025 + 0.015
ap.add_argument('--escape_max', type=float, default=0.30)    # 앞 · 뒤로 움직일 최대 거리(m)
ap.add_argument('--escape_spin_margin', type=float, default=0.03)   # 회전 중심 → 가장 가까운 치명 칸 ≥ footprint 외접 반지름 + 이 값이면 한 바퀴 돌며 위치 확인
ap.add_argument('--frame', default='map')                     # 목표 · 위치 추정 frame(설계 18 Cartographer 구성은 mission_map)
ap.add_argument('--loc_gate', type=int, default=0)           # 11-02 K2d 원인: > 0 이면 ① /localization/status 가 OK 일 때만 목표를 보냄(최대 60 모의 초 기다림) ② 주행 중 OK 가 아니었다가 OK 로 돌아오면 목표를 취소하고 다시 보냄(이 횟수까지) — 강제 이동 전 자세로 만든 낡은 경로를 쫓지 않게
ap.add_argument('--fp_rc', type=float, nargs=3, default=None)   # 11-02 B안: Nav2 footprint(rotation_center 기준 x 뒤 · 앞, |y|), 안 주면 FP_RC(설계 45 = 261028 모델)
a, _ = ap.parse_known_args()
if a.fp_rc:
    FP_RC = tuple(a.fp_rc)


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Runner(Node):
    def __init__(self):
        super().__init__('nav_goal_runner', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.truth_t, self.truth = [], []
        self.loc = []
        self.cmd = {'n': 0, 'frames': set(), 'vx_max': 0.0, 'vx_min': 0.0, 'wz_abs_max': 0.0}
        self.create_subscription(PoseStamped, '/sim/truth', self.on_truth, 50)
        self.create_subscription(TwistStamped, '/cmd_vel_nav', self.on_cmd, 50)
        from std_msgs.msg import String
        from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
        self.loc_status, self.loc_bad, self.loc_recovered = 'OK', False, False
        self.create_subscription(String, '/localization/status', self.on_loc, QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf, self)
        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.spin_client = ActionClient(self, Spin, '/spin')
        self.backup_client = ActionClient(self, BackUp, '/backup')
        self.drive_client = ActionClient(self, DriveOnHeading, '/drive_on_heading')
        self.obst, self.obst_res = None, 0.0
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.on_costmap, QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(0.1, self.sample)

    def on_costmap(self, m):
        # 치명 칸(OccupancyGrid 100 = costmap 254) 중심점(전역 frame). 계획기 'Start occupied' 판단과 같은 칸
        g = np.asarray(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
        iy, ix = np.nonzero(g >= 100)
        r, o = m.info.resolution, m.info.origin
        oyaw = yaw_of(o.orientation)
        lx, ly = (ix + 0.5) * r, (iy + 0.5) * r
        self.obst = np.stack([o.position.x + math.cos(oyaw) * lx - math.sin(oyaw) * ly, o.position.y + math.sin(oyaw) * lx + math.cos(oyaw) * ly], 1)
        self.obst_res = r

    def rc_pose(self):
        """추정 회전 중심 자세(frame): base_link ∘ (base_offset_x, 0). 참값은 쓰지 않음."""
        try:
            tf = self.buf.lookup_transform(a.frame, 'base_link', Time())
        except tf2_ros.TransformException:
            return None
        yaw = yaw_of(tf.transform.rotation)
        return (tf.transform.translation.x + a.base_offset_x * math.cos(yaw), tf.transform.translation.y + a.base_offset_x * math.sin(yaw), yaw)

    def clear_at(self, x, y, yaw, pts):
        """footprint(FP_RC, 여백 없음) 에서 치명 칸 경계까지 최소 거리(m, 안이면 음수 근사)."""
        c, s = math.cos(yaw), math.sin(yaw)
        dx, dy = pts[:, 0] - x, pts[:, 1] - y
        lx, ly = c * dx + s * dy, -s * dx + c * dy
        ex = np.maximum(np.maximum(FP_RC[0] - lx, lx - FP_RC[1]), 0.0)
        ey = np.maximum(np.abs(ly) - FP_RC[2], 0.0)
        return float(np.hypot(ex, ey).min()) - self.obst_res / 2 if len(pts) else 9.9

    def call(self, client, goal, wall_s):
        if not client.wait_for_server(timeout_sec=2.0):
            return 'NO_SERVER', ''
        f = client.send_goal_async(goal)
        while rclpy.ok() and not f.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        h = f.result()
        if not h.accepted:
            return 'REJECTED', ''
        r = h.get_result_async()
        w0 = time.monotonic()
        while rclpy.ok() and not r.done() and time.monotonic() - w0 < wall_s:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not r.done():
            return 'TIMEOUT', ''
        return ({GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED', GoalStatus.STATUS_ABORTED: 'ABORTED'}.get(r.result().status, str(r.result().status)),
                getattr(r.result().result, 'error_msg', ''))

    def escape(self):
        """출발 칸이 막혔을 때: ① 추정 자세 · 전역 costmap 치명 칸으로 앞 · 뒤 직선 이동 후보를 보고, 가는 길이 막히지 않으면서
        차체 여유가 escape_clear 이상이 되는 가장 짧은 쪽으로 움직임(같으면 뒤, BackUp / DriveOnHeading)
        ② 그 자리에서 회전 중심 → 치명 칸 거리가 외접 반지름 + escape_spin_margin 이상이면 한 바퀴(Spin 2π) 돌고 위치 상태 OK 확인."""
        self.spin_for(0.5)
        p = self.rc_pose()
        rec = {'t': round(self.now_s(), 3)}
        if p is None or self.obst is None:
            rec['skip'] = 'TF 또는 전역 costmap 없음'
            return rec
        x, y, yaw = p
        pts = self.obst[np.hypot(self.obst[:, 0] - x, self.obst[:, 1] - y) < 1.5]
        c0 = self.clear_at(x, y, yaw, pts)
        rec |= {'pose': [round(x, 4), round(y, 4), round(yaw, 4)], 'clear0': round(c0, 4)}
        step = 0.025
        cand = {}
        for sgn in (-1, 1):
            best = None
            for k in range(1, int(round(a.escape_max / step)) + 1):
                d = sgn * k * step
                cl = self.clear_at(x + d * math.cos(yaw), y + d * math.sin(yaw), yaw, pts)
                if cl < min(0.005, c0):      # 가는 길에 닿음 → 이 방향은 여기까지
                    break
                if best is None or cl > best[1]:
                    best = (d, cl)
                if cl >= a.escape_clear:
                    best = (d, cl)
                    break
            if best:
                cand[sgn] = best
        rec['cand'] = {('back' if k < 0 else 'fwd'): [round(v[0], 3), round(v[1], 4)] for k, v in cand.items()}
        ok = [v for v in cand.values() if v[1] >= a.escape_clear]
        pick = min(ok, key=lambda v: (abs(v[0]), v[0] > 0)) if ok else (max(cand.values(), key=lambda v: v[1]) if cand else None)
        if pick is None or pick[1] <= c0 + 0.01:
            rec['skip'] = '앞 · 뒤 모두 여유가 늘지 않음'
            return rec
        d = pick[0]
        if d < 0:
            gl = BackUp.Goal()
            gl.target.x = -d
        else:
            gl = DriveOnHeading.Goal()
            gl.target.x = d
        gl.speed = 0.1
        gl.time_allowance.sec = 20
        rec['move'] = round(d, 3)
        rec['move_status'], rec['move_msg'] = self.call(self.backup_client if d < 0 else self.drive_client, gl, 60)
        self.spin_for(1.0)
        p = self.rc_pose()
        if p is None:
            return rec
        x, y, yaw = p
        r_need = math.hypot(max(abs(FP_RC[0]), abs(FP_RC[1])), FP_RC[2]) + a.escape_spin_margin
        pts = self.obst[np.hypot(self.obst[:, 0] - x, self.obst[:, 1] - y) < 1.5]
        r_have = float(np.hypot(pts[:, 0] - x, pts[:, 1] - y).min()) - self.obst_res / 2 if len(pts) else 9.9
        rec |= {'pose1': [round(x, 4), round(y, 4), round(yaw, 4)], 'clear1': round(self.clear_at(x, y, yaw, pts), 4),
                'spin_r_have': round(r_have, 4), 'spin_r_need': round(r_need, 4)}
        if r_have >= r_need:
            sg = Spin.Goal()
            sg.target_yaw = 2 * math.pi
            sg.time_allowance.sec = 40
            rec['spin_status'], rec['spin_msg'] = self.call(self.spin_client, sg, 120)
            w1 = self.now_s()
            self.spin_for(1.0)
            while rclpy.ok() and self.loc_status != 'OK' and self.now_s() - w1 < 30.0:
                rclpy.spin_once(self, timeout_sec=0.05)
        rec['loc'] = self.loc_status
        rec['t_end'] = round(self.now_s(), 3)
        return rec

    def on_loc(self, m):
        if m.data == 'OK' and self.loc_status != 'OK':
            self.loc_recovered = True     # 위치를 다시 찾음 = 추정 자세가 뛰었을 수 있음(--loc_gate)
        self.loc_status = m.data
        if m.data != 'OK':
            self.loc_bad = True

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_truth(self, m):
        self.truth_t.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
        self.truth.append((m.pose.position.x, m.pose.position.y, yaw_of(m.pose.orientation)))

    def on_cmd(self, m):
        c = self.cmd
        c['n'] += 1
        c['frames'].add(m.header.frame_id)
        c['vx_max'] = max(c['vx_max'], m.twist.linear.x)
        c['vx_min'] = min(c['vx_min'], m.twist.linear.x)
        c['wz_abs_max'] = max(c['wz_abs_max'], abs(m.twist.angular.z))

    def truth_at(self, t):
        i = bisect.bisect_left(self.truth_t, t)
        if i <= 0 or i >= len(self.truth_t):
            return None
        t0, t1 = self.truth_t[i - 1], self.truth_t[i]
        f = (t - t0) / max(t1 - t0, 1e-9)
        p0, p1 = self.truth[i - 1], self.truth[i]
        dyaw = math.atan2(math.sin(p1[2] - p0[2]), math.cos(p1[2] - p0[2]))
        return (p0[0] + f * (p1[0] - p0[0]), p0[1] + f * (p1[1] - p0[1]), p0[2] + f * dyaw)

    def sample(self):
        try:
            tf = self.buf.lookup_transform(a.frame, 'base_link', Time())
        except tf2_ros.TransformException:
            return
        st = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
        tr = self.truth_at(st)
        if tr is None:
            return
        x, y = tf.transform.translation.x, tf.transform.translation.y
        yaw = yaw_of(tf.transform.rotation)
        self.loc.append([round(self.now_s(), 3), round(st, 3), round(math.hypot(x - tr[0], y - tr[1]), 4),
                         round(abs(math.atan2(math.sin(yaw - tr[2]), math.cos(yaw - tr[2]))), 4)])

    def spin_for(self, sim_s):
        t0, w0 = self.now_s(), time.monotonic()
        while rclpy.ok() and self.now_s() - t0 < sim_s and time.monotonic() - w0 < sim_s * 5 + 5:   # 모의 시계가 멈춰도 끝남
            rclpy.spin_once(self, timeout_sec=0.05)


def main():
    rclpy.init()
    n = Runner()
    goals = json.load(open(a.goals))['goals']
    rep = {'goals': [], 'wait_server_s': None}
    w0 = time.monotonic()
    while not n.client.wait_for_server(timeout_sec=1.0):
        rclpy.spin_once(n, timeout_sec=0.1)
        if time.monotonic() - w0 > 120:
            rep['error'] = 'navigate_to_pose 서버 없음(120 s)'
            break
    rep['wait_server_s'] = round(time.monotonic() - w0, 1)
    # AMCL map→odom 이 나올 때까지(최대 60 s)
    w1 = time.monotonic()
    while 'error' not in rep and not n.buf.can_transform(a.frame, 'base_link', Time()):
        rclpy.spin_once(n, timeout_sec=0.1)
        if time.monotonic() - w1 > 60:
            rep['error'] = f'{a.frame}→base_link TF 없음(60 s)'
    n.spin_for(2.0)
    for g in goals if 'error' not in rep else []:
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = a.frame
        goal.pose.header.stamp = n.get_clock().now().to_msg()
        goal.pose.pose.position.x = g['x'] + a.base_offset_x * math.cos(g['yaw'])
        goal.pose.pose.position.y = g['y'] + a.base_offset_x * math.sin(g['yaw'])
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(g['yaw'] / 2), math.cos(g['yaw'] / 2)
        t0 = n.now_s()
        retries, escapes, gate, resends = [], [], [], []
        while True:
            if a.loc_gate and n.loc_status != 'OK':
                w1 = n.now_s()
                while rclpy.ok() and n.loc_status != 'OK' and n.now_s() - w1 < 60.0:
                    rclpy.spin_once(n, timeout_sec=0.05)
                gate.append({'t0': round(w1, 3), 't1': round(n.now_s(), 3), 'loc': n.loc_status})
            n.loc_recovered = False
            n.loc_bad = n.loc_status != 'OK'
            goal.pose.header.stamp = n.get_clock().now().to_msg()
            fut = n.client.send_goal_async(goal)
            while rclpy.ok() and not fut.done():
                rclpy.spin_once(n, timeout_sec=0.05)
            gh = fut.result()
            row = {'name': g['name'], 'goal': [g['x'], g['y'], g['yaw']], 'accepted': bool(gh.accepted), 't_start': round(t0, 3)}
            if gh.accepted:
                rf = gh.get_result_async()
                w0 = time.monotonic()
                resend = lambda: a.loc_gate and n.loc_recovered and len(resends) < a.loc_gate   # noqa: E731
                while rclpy.ok() and not rf.done() and n.now_s() - t0 < a.goal_timeout and time.monotonic() - w0 < a.goal_timeout * 3 and not resend():
                    rclpy.spin_once(n, timeout_sec=0.05)
                if not rf.done():
                    cf = gh.cancel_goal_async()
                    while rclpy.ok() and not cf.done():
                        rclpy.spin_once(n, timeout_sec=0.05)
                    row['status'] = 'RESEND' if resend() else 'TIMEOUT'
                else:
                    st = rf.result().status
                    row['status'] = {GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED', GoalStatus.STATUS_ABORTED: 'ABORTED',
                                     GoalStatus.STATUS_CANCELED: 'CANCELED'}.get(st, str(st))
                    res = rf.result().result
                    row['error_code'] = int(getattr(res, 'error_code', 0))
                    row['error_msg'] = getattr(res, 'error_msg', '')
            if row.get('status') == 'RESEND':
                n.spin_for(0.5)                 # 취소가 끝나고 BT 가 멈출 시간
                resends.append({'t': round(n.now_s(), 3), 'loc': n.loc_status})
                continue
            if row.get('status') == 'SUCCEEDED':
                break
            if len(escapes) < a.escape and (row.get('error_code') == 205 or 'Start occupied' in row.get('error_msg', '')):
                escapes.append(n.escape())      # 출발 자세가 여백 footprint 안에서 막힘 → 바로 다시 계획하지 않고 빠져나온 뒤 다시 보냄
                continue
            if not n.loc_bad or len(retries) >= a.loc_retry:
                break
            w1 = n.now_s()
            while rclpy.ok() and n.loc_status != 'OK' and n.now_s() - w1 < 60.0:
                rclpy.spin_once(n, timeout_sec=0.05)
            n.spin_for(1.0)
            retries.append({'status': row.get('status'), 'error_msg': row.get('error_msg', ''), 't': round(n.now_s(), 3), 'loc': n.loc_status})
        if retries:
            row['loc_retries'] = retries
        if escapes:
            row['escapes'] = escapes
        if gate:
            row['loc_gate_waits'] = gate
        if resends:
            row['loc_resends'] = resends
        if a.final_spin_tol > 0 and row.get('status') == 'SUCCEEDED':
            # Lattice 경로 끝 방향은 16 방향 칸(목표와 최대 11.25°) → 도착 뒤 남은 방향만 제자리로(Nav2 behavior_server Spin, 지역 costmap 충돌 검사).
            # 멈춤 지나침이 있으면 허용 안에 들 때까지 final_spin_max 번까지 반복.
            for k in range(a.final_spin_max):
                n.spin_for(0.5)
                try:
                    tf = n.buf.lookup_transform(a.frame, 'base_link', Time())
                    d = math.atan2(math.sin(g['yaw'] - yaw_of(tf.transform.rotation)), math.cos(g['yaw'] - yaw_of(tf.transform.rotation)))
                except tf2_ros.TransformException:
                    d = None
                row.setdefault('spin_deltas', []).append(None if d is None else round(d, 4))
                if k == 0:
                    row['spin_delta'] = None if d is None else round(d, 4)
                if d is None or abs(d) <= a.final_spin_tol:
                    break
                sg = Spin.Goal()
                sg.target_yaw = d
                sg.time_allowance.sec = 15
                sf = n.spin_client.send_goal_async(sg)
                while rclpy.ok() and not sf.done():
                    rclpy.spin_once(n, timeout_sec=0.05)
                sh = sf.result()
                row['spin_n'] = k + 1
                if not sh.accepted:
                    row['spin_status'] = 'REJECTED'
                    break
                sr = sh.get_result_async()
                w0 = time.monotonic()
                while rclpy.ok() and not sr.done() and time.monotonic() - w0 < 60:
                    rclpy.spin_once(n, timeout_sec=0.05)
                row['spin_status'] = {GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED', GoalStatus.STATUS_ABORTED: 'ABORTED'}.get(
                    sr.result().status, str(sr.result().status)) if sr.done() else 'TIMEOUT'
        row['t_end'] = round(n.now_s(), 3)
        row['duration_s'] = round(row['t_end'] - t0, 2)
        n.spin_for(a.settle)
        tr = n.truth[-1] if n.truth else None
        if tr:
            row['truth_end'] = [round(v, 4) for v in tr]
            row['pos_err'] = round(math.hypot(tr[0] - g['x'], tr[1] - g['y']), 4)
            row['yaw_err'] = round(abs(math.atan2(math.sin(tr[2] - g['yaw']), math.cos(tr[2] - g['yaw']))), 4)
        n.get_logger().info(json.dumps(row, ensure_ascii=False))
        rep['goals'].append(row)
        json.dump(rep | {'loc': n.loc, 'cmd': n.cmd | {'frames': sorted(n.cmd['frames'])}}, open(a.out, 'w'), ensure_ascii=False)
    rep['loc'] = n.loc
    rep['cmd'] = n.cmd | {'frames': sorted(n.cmd['frames'])}
    json.dump(rep, open(a.out, 'w'), ensure_ascii=False)
    print('[run_goals] 끝', json.dumps([(r['name'], r.get('status'), r.get('pos_err')) for r in rep['goals']], ensure_ascii=False))


if __name__ == '__main__':
    main()
