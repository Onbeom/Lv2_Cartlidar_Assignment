#!/usr/bin/env python3
"""규칙 v2 계획 시연: demo_plan.json 을 Nav2 (controller_server FollowPath · behavior_server Spin) 로 따라가며 기록 (run_demo.sh 가 부름).

  python3 demo_follow.py --plan demo_plan.json --out <기록 폴더> [--sim] [--step] [--legs 0-3]

  --sim   노트북 loopback 시뮬 (모의 시각, 시작 자세를 /initialpose 로 맞출 때까지 되풀이)
  실물    벽시계. 시작 전 확인 (11-15 강화): map → base_link 가 출발 자세에서 15 cm · 15° 안 그리고
            위치 감시 (loc_monitor) 가 떠 있으면 /localization/status = OK 만 (SUSPECT · 상태 없음은 안 됨),
            꺼져 있으면 /amcl_pose σ ≤ 5 cm · 5° + RViz 로 라이다 ↔ 벽 겹침을 보고 y 입력.
          60 s 안에 안 맞으면 이유를 알려 주고 끝. --set-initial 이면 출발 자세를 /initialpose 로 한 번 냄 (로봇을 출발 자리에 정확히 놓았을 때만)
  --step  구간마다 Enter 를 눌러야 출발 (q = 끝). Ctrl+C = 지금 동작 취소 → Nav2 가 0 을 냄 → 멈춤 (소프트웨어 정지, 물리 비상정지 아님)
조각: 전진 · 후진 = FollowPath (Forward · Reverse, 끝 뒤 연장선 → 원래 끝을 지나면 취소), 제자리 = Spin, 정지점 방향 2° 넘게 다르면 Spin.
      11-15: 전진 · 후진 조각을 시작할 때 방향이 4° 넘게 틀어져 있으면 먼저 Spin 으로 맞춤 (--no-head-fix 로 끔).
11-15 기본 경로 + 들르기 (계획 파일 "mode": "route"): 기본 경로 (상자 순회) 를 달리다가 들르기 요청이 오면 그 정지점에서
      요청한 곳 (예 L2) 을 먼저 들른 뒤 기본 경로 다음 곳으로 복귀. 길은 길 모음 (legs.json, 90 개) 에서 꺼냄.
      요청 = step 모드에서 L1 · L2 · L3 (또는 상자 이름) 입력 · 토픽 /drive_demo/detour (std_msgs/String) · 계획 파일 auto_detour (시뮬 시험).
      정지점에 도착하면 /drive_demo/arrived (std_msgs/String, 그 이름) 를 냄. 어느 L 이 완성 가능한지는 정하지 않음 (요청을 따를 뿐).
기록: 0.1 s 마다 map → base_link + cmd_vel_raw (Nav2 명령) + 필터 뒤 명령 → track.csv · course_result.json · moves.json · summary.txt (주행기록 HTML 형식)
"""
from __future__ import annotations

import argparse
import json
import math
import os
import select
import signal
import sys
import threading
import time
from collections import deque

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TwistStamped
from nav2_msgs.action import FollowPath, Spin
from nav_msgs.msg import Path
from std_srvs.srv import Empty
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

POS_OK, YAW_OK, YAW_FIX = 0.05, math.radians(10.0), math.radians(2.0)
EXT_F, EXT_R = 0.15, 0.10
START_TOL, START_YAW_TOL = 0.15, math.radians(15.0)
AMCL_XY_OK, AMCL_YAW_OK = 0.05, 5.0   # 위치 감시를 껐을 때 출발 허용 AMCL σ [m] · [deg]
OFF_PATH = 0.10   # 11-15 21:4x 사용자 결정: 전진 · 후진 조각 중 AMCL 자리가 조각 경로에서 max(이 값, 시작 때 어긋남 + 3 cm) 넘게
                  #   벗어나면 그 조각을 취소하고 멈춤 (S6 벽 닿음: 경로에서 24 cm). 소프트웨어 정지, 물리 비상정지 아님. 0 = 끔 (--off-path)
RECOVER_TRIES, BACK_MAX = 2, 0.30   # 11-16 사용자 결정: 조각 실패 시 되찾기 (자기 위치 찾기 → 왔던 길 ≤ BACK_MAX m → 다시) 최대 횟수 (0 = 끔)
STEP_MODE = False                  # step 모드면 되찾기도 안 될 때 사람에게 물음
HEAD_FIX = math.radians(4.0)          # 전진 · 후진 조각 시작 때 방향이 이보다 틀어져 있으면 먼저 제자리로 맞춤 (0 = 끔, --no-head-fix)
STOP = threading.Event()
LIVE = None   # 11-15 21:3x: track_live.csv — 0.1 s 마다 바로 씀 (Ctrl+C · 강제 종료로 save() 를 못 해도 남음)
NAMES = ["HOME", "box1", "box2", "box3", "box4", "box5", "box6", "L1", "L2", "L3"]
SPIN_MAX_DEG, REV_MAX = 90.0, 0.40   # 규칙 v2 이음매 확인 (길모음/compose.py 와 같음)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_q(y):
    return math.sin(y / 2), math.cos(y / 2)


class Follower(Node):
    def __init__(self, sim):
        super().__init__("demo_follower", parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, sim)])
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)
        self.fp = ActionClient(self, FollowPath, "follow_path")
        self.spin = ActionClient(self, Spin, "spin")
        self.sim = sim
        self.nomotion = self.create_client(Empty, "/request_nomotion_update")   # AMCL: 멈춘 채 위치 다시 계산 (되찾기)
        self.recover = []   # 이 구간 되찾기 기록
        self.init_pub = self.create_publisher(PoseWithCovarianceStamped, "initialpose", 10)
        self.raw = (0.0, 0.0, -9.0)
        self.out = (0.0, 0.0, -9.0)
        self.create_subscription(TwistStamped, "cmd_vel_raw", lambda m: setattr(self, "raw", (m.twist.linear.x, m.twist.angular.z, self.now_s())), 20)
        self.create_subscription(TwistStamped, "cmd_vel_v2", lambda m: setattr(self, "out", (m.twist.linear.x, m.twist.angular.z, self.now_s())), 20)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.loc = None
        self.create_subscription(String, "/localization/status", lambda m: setattr(self, "loc", m.data), latched)
        # AMCL 자세 확신도 (위치 감시를 끈 실물 시연용). 발행 쪽 durability 를 몰라 두 가지로 받음 (맞는 쪽만 연결됨)
        self.amcl = None   # (σxy [m], σyaw [deg], 받은 벽시계)
        for dur in (DurabilityPolicy.TRANSIENT_LOCAL, DurabilityPolicy.VOLATILE):
            self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._amcl,
                                     QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=dur))
        self.step = -1
        self.head_fix = []   # 조각 시작 방향 맞추기 각 [deg]
        self.head_skip = []  # 11-16: 거부 · 실패해서 건너뛴 방향 맞추기 각 [deg]
        self.rows = []
        self.lock = threading.Lock()
        self.create_timer(0.1, self._rec)
        # 11-15 들르기: 요청 (언제 와도 다음 정지점에서 씀) · 도착 알림
        self.pending = deque()
        self.plock = threading.Lock()
        self.create_subscription(String, "/drive_demo/detour", self._detour, 10)
        self.arrived = self.create_publisher(String, "/drive_demo/arrived", 10)

    def _detour(self, m):
        nm = norm(m.data)
        if nm not in NAMES:
            print(f"\n  ⚠ 들르기 요청 '{m.data}' 모름 (쓸 수 있는 이름: {' '.join(NAMES)})", flush=True)
            return
        with self.plock:
            self.pending.append(nm)
        print(f"\n  ↪ 들르기 요청 받음: {nm} (다음 정지점에서)", flush=True)

    def _amcl(self, m):
        c = m.pose.covariance
        self.amcl = (math.sqrt(max(c[0], c[7], 0.0)), math.degrees(math.sqrt(max(c[35], 0.0))), time.time())

    def loc_live(self):
        """위치 감시 (loc_monitor) 가 지금 떠 있나 — 꺼진 뒤 남은 예전 상태는 안 씀"""
        return self.count_publishers("/localization/status") > 0

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def pose(self):
        try:
            t = self.tf.lookup_transform("map", "base_link", Time())
        except Exception:  # noqa: BLE001
            return None
        q = t.transform.rotation
        return (t.transform.translation.x, t.transform.translation.y,
                math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

    def _rec(self):
        p = self.pose()
        if p is None:
            return
        t = self.now_s()
        v, w, ts = self.out
        if t - ts > 0.25:
            v = w = 0.0
        rv, rw, rts = self.raw
        if t - rts > 0.25:
            rv = rw = 0.0
        with self.lock:
            self.rows.append((t, p[0], p[1], math.degrees(p[2]), v, w, self.step, rv, rw, self.loc or ""))
        if LIVE is not None:
            try:
                LIVE.write(f"{t:.2f},{p[0]:.4f},{p[1]:.4f},{math.degrees(p[2]):.1f},{v:.3f},{w:.3f},{self.step},{rv:.3f},{rw:.3f},{self.loc or ''}\n")
            except (OSError, ValueError):
                pass

    def set_initial(self, x, y, th):
        m = PoseWithCovarianceStamped()
        m.header.frame_id = "map"
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.pose.position.x, m.pose.pose.position.y = x, y
        m.pose.pose.orientation.z, m.pose.pose.orientation.w = yaw_q(th)
        m.pose.covariance[0] = m.pose.covariance[7] = 0.02 ** 2
        m.pose.covariance[35] = math.radians(3) ** 2
        self.init_pub.publish(m)


def norm(t):
    t = (t or "").strip()
    if t.upper() in ("L1", "L2", "L3", "HOME"):
        return t.upper()
    return t.lower() if t.lower().startswith("box") else t


def first_rev(pieces):
    """길 처음의 후진 길이 (전진 0.15 m 전까지 이어지는 후진 합) — 길모음/compose.py 와 같음"""
    rev = fw = 0.0
    for pc in pieces:
        if pc["kind"] == "S":
            continue
        L = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pc["poses"], pc["poses"][1:]))
        if pc["kind"] == "R":
            rev += L
            fw = 0.0
        else:
            fw += L
            if fw >= 0.15:
                break
    return rev


def first_spin(pieces):
    """길 처음 (0.15 m 움직이기 전) 의 제자리 회전 각 합 [deg] — 길모음/compose.py 와 같음"""
    sp = moved = 0.0
    for pc in pieces:
        if pc["kind"] == "S":
            sp += abs(math.degrees(pc["dth"]))
            moved = 0.0
            continue
        moved += sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pc["poses"], pc["poses"][1:]))
        if moved >= 0.15:
            break
    return sp


def wait_future(fut, timeout):
    t0 = time.time()
    while not fut.done():
        if time.time() - t0 > timeout:
            return False
        time.sleep(0.02)
    return True


def sleep_s(node, s):
    t0 = node.now_s()
    while node.now_s() - t0 < s and not STOP.is_set():
        time.sleep(0.02)


def run_action(client, goal, timeout, done_when=None, abort_when=None):
    if not client.wait_for_server(timeout_sec=10.0):
        return False, "action server 없음"
    gf = client.send_goal_async(goal)
    if not wait_future(gf, 10.0):
        return False, "goal 응답 없음"
    gh = gf.result()
    if not gh.accepted:
        return False, "goal 거절"
    rf = gh.get_result_async()
    t0 = time.time()
    while not rf.done():
        if STOP.is_set():
            wait_future(gh.cancel_goal_async(), 5.0)
            return False, "Ctrl+C 로 멈춤"
        why = abort_when() if abort_when is not None else None
        if why:
            wait_future(gh.cancel_goal_async(), 5.0)
            wait_future(rf, 5.0)
            return False, why
        if done_when is not None and done_when():
            wait_future(gh.cancel_goal_async(), 5.0)
            wait_future(rf, 5.0)
            return True, ""
        if time.time() - t0 > timeout:
            wait_future(gh.cancel_goal_async(), 5.0)
            return False, f"{timeout:.0f} s 안에 안 끝남 (취소)"
        time.sleep(0.01)
    res = rf.result()
    ok = res.status == 4
    r = res.result
    return ok, ("" if ok else f"상태 {res.status} · 오류 {getattr(r, 'error_code', 0)} {getattr(r, 'error_msg', '') or ''}".strip())


def follow_piece(node, pc):
    if pc["kind"] == "S":
        g = Spin.Goal()
        g.target_yaw = float(pc["dth"])
        g.time_allowance = Duration(sec=int(abs(pc["dth"]) / 0.2) + 10)
        return run_action(node.spin, g, abs(pc["dth"]) / 0.2 + 30)
    poses = pc["poses"]
    cur = node.pose()
    if len(poses) < 2 or (cur and math.hypot(poses[-1][0] - cur[0], poses[-1][1] - cur[1]) < 0.03):
        return True, ""
    # 11-15: 조각 시작 방향 맞추기 — 앞 조각 (특히 짧은 후진) 끝에서 방향이 틀어진 채 출발하면 반경 0.20 한계 때문에 못 돌아와 옆으로 밀림
    #   (11-14 22:57 시뮬 S5: 후진 끝 +8.5° → 전진 중 기둥 쪽으로 2 ~ 3 cm). 멈춘 순간이라 작은 제자리 회전 (규칙 v2 예산 안)
    if HEAD_FIX > 0 and cur is not None:
        err = wrap(poses[0][2] - cur[2])
        if abs(err) > HEAD_FIX:
            print(f"    · 조각 시작 방향 맞추기 {math.degrees(err):+.1f}°", flush=True)
            node.head_fix.append(round(math.degrees(err), 1))
            if os.environ.get("DEMO_TEST_HEADFIX_FAIL") == "1":   # 시험용: 방향 맞추기가 모두 거부된 것처럼 (건너뛰기 경로 확인)
                r, msg = False, "시험용 거부 (DEMO_TEST_HEADFIX_FAIL=1)"
            else:
                r, msg = follow_piece(node, {"kind": "S", "dth": err})
            if not r:
                if STOP.is_set():
                    return False, f"시작 방향 맞추기 {math.degrees(err):+.0f}°: {msg}"
                # 11-16 (사용자 결정): Nav2 Spin 이 충돌 예상 (Collision Ahead) · 시간 초과로 거부하면 구간 전체를 멈추지 않고
                #   방향 맞추기만 건너뛰고 조각을 그대로 진행 (RPP 가 따라가며 바로잡음, 경로 이탈 멈춤이 지킴). 시뮬 P2 S2 에서 나옴
                print(f"    · 시작 방향 맞추기 건너뜀 ({msg}) — 조각을 그대로 진행 (경로 이탈 멈춤 {OFF_PATH * 100:.0f} cm 가 지킴)", flush=True)
                node.head_skip.append(round(math.degrees(err), 1))
    d = 1.0 if pc["kind"] == "F" else -1.0
    ex, ey, eth = poses[-1]
    ext = EXT_F if d > 0 else EXT_R
    full = list(poses) + [(ex + d * s * math.cos(eth), ey + d * s * math.sin(eth), eth) for s in [0.01 * i for i in range(1, int(ext / 0.01) + 1)]]
    path = Path()
    path.header.frame_id = "map"
    path.header.stamp = node.get_clock().now().to_msg()
    for x, y, th in full:
        ps = PoseStamped()
        ps.header = path.header
        ps.pose.position.x, ps.pose.position.y = x, y
        ps.pose.orientation.z, ps.pose.orientation.w = yaw_q(th)
        path.poses.append(ps)
    g = FollowPath.Goal()
    g.path = path
    g.controller_id = "Forward" if d > 0 else "Reverse"
    g.goal_checker_id = "piece_checker"
    if hasattr(g, "progress_checker_id"):
        g.progress_checker_id = "progress_checker"
    ux, uy = d * math.cos(eth), d * math.sin(eth)

    def passed():
        p = node.pose()
        if p is None:
            return False
        rx, ry = p[0] - ex, p[1] - ey
        return rx * ux + ry * uy >= -0.003 and math.hypot(rx, ry) < 0.2
    length = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(poses, poses[1:]))

    def dev():   # 조각 경로 (연장 포함) 에서 가장 가까운 점까지 [m]
        p = node.pose()
        return None if p is None else min(math.hypot(q[0] - p[0], q[1] - p[1]) for q in full)
    d0 = dev() or 0.0
    lim = max(OFF_PATH, d0 + 0.03)
    last = [0.0]

    t_start = time.time()
    test_fail = os.environ.get("DEMO_TEST_PIECE_FAIL") == "1" and pc["kind"] == "F" and length > 0.6 and not pc.get("_tested")

    def off_path():
        if test_fail and time.time() - t_start > 3.0:   # 시험용: 되찾기 확인 (DEMO_TEST_PIECE_FAIL=1, 조각마다 처음 한 번)
            pc["_tested"] = True
            return "시험용 실패 (DEMO_TEST_PIECE_FAIL=1)"
        if OFF_PATH <= 0 or time.time() - last[0] < 0.1:
            return None
        last[0] = time.time()
        d = dev()
        if d is not None and d > lim:
            return f"경로에서 {d * 100:.0f} cm 벗어남 (한도 {lim * 100:.0f} cm) — 멈춤 (소프트웨어 정지)"
        return None
    return run_action(node.fp, g, length / 0.03 + 30, done_when=passed, abort_when=off_path)


def relocalize(node, label):
    """멈춘 채 자기 위치 다시 찾기 (11-16): 실물이면 AMCL nomotion 갱신을 몇 번 요청 (움직이지 않아도 라이다로 다시 맞춤) → σ 를 알림"""
    sleep_s(node, 0.5)
    n = 0
    if not node.sim and node.nomotion.wait_for_service(timeout_sec=1.0):
        for _ in range(5):
            if STOP.is_set():
                break
            wait_future(node.nomotion.call_async(Empty.Request()), 2.0)
            n += 1
            sleep_s(node, 0.3)
    am = node.amcl
    sig = f"AMCL σ {am[0] * 100:.1f} cm · {am[1]:.1f}°" if am and time.time() - am[2] < 10 else ("시뮬 (AMCL 없음)" if node.sim else "AMCL σ 모름")
    print(f"    · {label}: 자기 위치 다시 찾기 (nomotion 갱신 {n} 번) · {sig}", flush=True)


def backtrack_piece(node, pc):
    """왔던 길 되돌아가기 조각 (11-16): 이 조각에서 이미 지나온 부분을 거꾸로, 최대 BACK_MAX m. 전진 조각이면 후진으로 (반대도)"""
    p = node.pose()
    poses = pc["poses"]
    if p is None or len(poses) < 2 or BACK_MAX <= 0:
        return None
    k = min(range(len(poses)), key=lambda i: math.hypot(poses[i][0] - p[0], poses[i][1] - p[1]))
    back, length = [tuple(poses[k])], 0.0
    for i in range(k - 1, -1, -1):
        step = math.hypot(poses[i][0] - back[-1][0], poses[i][1] - back[-1][1])
        if length + step > BACK_MAX:
            break
        back.append(tuple(poses[i])); length += step
    if length < 0.05:
        return None
    return {"kind": "R" if pc["kind"] == "F" else "F", "poses": back, "len": length}


def ask_choice(prompt):
    """되찾기도 안 될 때 (step 모드): Enter = 한 번 더 · s = 이 조각 건너뛰기 · q = 구간 끝"""
    print(prompt, end="", flush=True)
    while not STOP.is_set():
        r, _, _ = select.select([sys.stdin], [], [], 0.2)
        if r:
            t = sys.stdin.readline().strip().lower()
            return "quit" if t == "q" else ("skip" if t == "s" else "retry")
    return "quit"


def ask(prompt):
    """Enter = 진행, q = 끝. Ctrl+C 도 끝"""
    print(prompt, end="", flush=True)
    while not STOP.is_set():
        r, _, _ = select.select([sys.stdin], [], [], 0.2)
        if r:
            return sys.stdin.readline().strip().lower() != "q"
    return False


def ask_next(prompt):
    """Enter = 그대로 출발 → (True, None) · 이름 (L2 · box3 …) = 그곳 먼저 → (True, 이름) · q = 끝 → (False, None)"""
    print(prompt, end="", flush=True)
    while not STOP.is_set():
        r, _, _ = select.select([sys.stdin], [], [], 0.2)
        if r:
            t = sys.stdin.readline().strip()
            if t.lower() == "q":
                return False, None
            return True, (norm(t) if t else None)
    return False, None


def confirm(prompt):
    """y = 진행, 그 밖 (q · 빈 Enter) = 안 함. Ctrl+C 도 안 함"""
    print(prompt, end="", flush=True)
    while not STOP.is_set():
        r, _, _ = select.select([sys.stdin], [], [], 0.2)
        if r:
            return sys.stdin.readline().strip().lower() == "y"
    return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--plan", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--sim", action="store_true")
    ap.add_argument("--step", action="store_true")
    ap.add_argument("--set-initial", action="store_true")
    ap.add_argument("--legs", default="")
    ap.add_argument("--no-head-fix", action="store_true", help="조각 시작 방향 맞추기 끔 (11-14 판처럼)")
    ap.add_argument("--off-path", type=float, default=None, help="경로 이탈 멈춤 한도 [m] (기본 0.10, 0 = 끔)")
    ap.add_argument("--no-recover", action="store_true", help="조각 실패 시 되찾기 끔 (11-16 전처럼)")
    a = ap.parse_args()
    if a.off_path is not None:
        global OFF_PATH
        OFF_PATH = a.off_path
    global RECOVER_TRIES, STEP_MODE
    STEP_MODE = a.step
    if a.no_recover:
        RECOVER_TRIES = 0
    if a.no_head_fix:
        global HEAD_FIX
        HEAD_FIX = 0.0
    data = json.load(open(a.plan, encoding="utf-8"))
    route = data.get("mode") == "route"
    if route:   # 11-15 기본 경로 + 들르기: 길은 길 모음에서 그때그때 꺼냄
        lp = data.get("lib", "legs.json")
        lp = lp if os.path.isabs(lp) else os.path.join(os.path.dirname(os.path.abspath(a.plan)), lp)
        lib = json.load(open(lp, encoding="utf-8"))
        LIB, PTS = {g["name"]: g for g in lib["legs"] if g.get("ok")}, lib["points"]
        bad = [x for x in data["route"] if x not in NAMES]
        if bad:
            raise SystemExit(f"모르는 이름 {bad}")
        data["start"], data["legs"] = PTS["HOME"]["start"], []
        data.setdefault("course", data["plan"] + ".yaml")
        legs, start = [], data["start"]
    else:
      legs = data["legs"]
      if a.legs:
        lo, _, hi = a.legs.partition("-")
        keep = set(range(int(lo), int(hi or lo) + 1))
        legs = [g for g in legs if g["leg"] in keep]
      first = legs[0]["leg"]
      start = data["start"] if first == data["legs"][0]["leg"] else data["legs"][first - 1]["goal"]

    signal.signal(signal.SIGINT, lambda *_: STOP.set())
    global LIVE
    os.makedirs(a.out, exist_ok=True)
    LIVE = open(os.path.join(a.out, "track_live.csv"), "w", encoding="utf-8", buffering=1)
    LIVE.write("t,x,y,yaw,v,w,step,v_nav,w_nav,loc\n")
    rclpy.init()
    node = Follower(a.sim)
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    def _spin():
        try:
            ex.spin()
        except Exception:  # noqa: BLE001  (끝낼 때 rclpy 종료와 겹치면 나는 오류 — 기록에는 영향 없음)
            pass
    threading.Thread(target=_spin, daemon=True).start()
    print(f"출발 자세 (계획) {start[0]:.3f}, {start[1]:.3f}, {math.degrees(start[2]):.0f}° · "
          + (f"기본 경로 {' → '.join(data['route'])} · 들르기 요청 /drive_demo/detour" + (" 또는 step 입력" if a.step else "")
             if route else f"구간 {legs[0]['name']} ~ {legs[-1]['name']}"), flush=True)

    # 출발 확인
    t0 = time.time()
    if a.sim:
        while not STOP.is_set():
            node.set_initial(*start)
            time.sleep(1.0)
            p = node.pose()
            if p and math.hypot(p[0] - start[0], p[1] - start[1]) < 0.01:
                break
            if time.time() - t0 > 90:
                print("✗ 시뮬 TF 가 시작 자세로 안 옴", flush=True)
                STOP.set()
    else:
        if a.set_initial:
            time.sleep(1.0)
            node.set_initial(*start)
            print("  /initialpose = 출발 자세 (한 번)", flush=True)
            time.sleep(2.0)
        # 출발 조건 (11-15): 위치 감시가 떠 있으면 상태 OK 만 (SUSPECT · 상태 없음은 안 됨),
        #   꺼져 있으면 AMCL 자세가 있고 σ ≤ 5 cm · 5° + 사람이 RViz 로 라이다 ↔ 벽 겹침 확인 (y)
        why = ""
        while not STOP.is_set():
            p = node.pose()
            live = node.loc_live()
            am = node.amcl
            if live:
                loc_ok = node.loc == "OK"
                loc_txt = f"위치 감시 {node.loc or '상태 아직 없음'}"
            else:
                loc_ok = am is not None and am[0] <= AMCL_XY_OK and am[1] <= AMCL_YAW_OK
                loc_txt = "위치 감시 꺼짐 · AMCL " + (f"σ {am[0] * 100:.1f} cm · {am[1]:.1f}°" if am else "자세 아직 없음")
            if p is None:
                why = "map → base_link TF 없음 (위치 추정이 안 떴거나 초기 자세 없음)"
                print(f"\r  {why} · {loc_txt}   ", end="", flush=True)
            else:
                dp, dy = math.hypot(p[0] - start[0], p[1] - start[1]), abs(wrap(p[2] - start[2]))
                print(f"\r  지금 위치 ({p[0]:.3f}, {p[1]:.3f}, {math.degrees(p[2]):.0f}°) · 출발과 {dp * 100:.1f} cm · {math.degrees(dy):.1f}° · "
                      f"{loc_txt}   ", end="", flush=True)
                if dp < START_TOL and dy < START_YAW_TOL and loc_ok:
                    print("", flush=True)
                    break
                why = (f"출발 자리와 {dp * 100:.0f} cm · {math.degrees(dy):.0f}° (한계 {START_TOL * 100:.0f} cm · {math.degrees(START_YAW_TOL):.0f}°)"
                       if not (dp < START_TOL and dy < START_YAW_TOL) else
                       f"위치 감시 상태가 OK 아님 ({node.loc or '없음'})" if live else
                       "AMCL 자세 없음 — RViz 2D Pose Estimate 또는 run_demo.sh … init" if am is None else
                       f"AMCL σ {am[0] * 100:.1f} cm · {am[1]:.1f}° 가 한계 ({AMCL_XY_OK * 100:.0f} cm · {AMCL_YAW_OK:.0f}°) 넘음 — 조금 기다리거나 초기 자세 다시")
            if time.time() - t0 > 60:
                print(f"\n✗ 60 s 안에 출발 조건이 안 맞음: {why}", flush=True)
                STOP.set()
            time.sleep(0.5)
        if not STOP.is_set() and not node.loc_live():
            if not confirm("  위치 감시가 꺼져 있음 — RViz 에서 라이다 점이 지도 벽과 겹치면 y + Enter (아니면 q) : "):
                print("✗ 출발 안 함 (위치 확인 안 됨)", flush=True)
                STOP.set()
    if not STOP.is_set():
        for c, n in ((node.fp, "follow_path"), (node.spin, "spin")):
            if not c.wait_for_server(timeout_sec=30.0):
                print(f"✗ {n} action server 없음 (run_demo.sh 의 Nav2 노드가 떴는지)", flush=True)
                STOP.set()

    results = []
    if route:
        run_route(a, node, data, LIB, PTS, results)
    else:
        for g in legs:
            if STOP.is_set():
                break
            if a.step and not ask(f"\n▶ {g['name']} — Enter 출발 · q 끝 : "):
                break
            if not run_leg(node, g, results):
                break
    node.step = -1
    time.sleep(1.0)
    with node.lock:
        rows = list(node.rows)
    ex.shutdown()
    rclpy.try_shutdown()
    save(a, data, results, rows, len(data["legs"]) if route else len(legs))



def run_leg(node, g, results):
    """구간 하나 (조각들 → 정지점 방향 맞추기 → 어긋남 판정). False = 조각이 실패해 여기서 끝냄"""
    node.step = g["leg"]
    tl = node.now_s()
    node.head_fix = []
    node.head_skip = []
    node.recover = []
    tries, ok = [], True
    for i, pc in enumerate(g["pieces"]):
        what = {"F": "전진", "R": "후진", "S": "제자리"}[pc["kind"]] + (f" {math.degrees(pc['dth']):+.0f}°" if pc["kind"] == "S" else "")
        print(f"  · 조각 {i + 1}/{len(g['pieces'])} {what}", flush=True)
        r, msg = follow_piece(node, pc)
        k = 0
        while not r and not STOP.is_set() and k < RECOVER_TRIES:   # 11-16 되찾기: 자기 위치 찾기 → 왔던 길 되돌아가기 → 다시
            k += 1
            print(f"  ↺ 되찾기 {k}/{RECOVER_TRIES} ({msg})", flush=True)
            tries.append({"status": "recover", "msg": f"조각 {i + 1} ({what}): {msg}", "t": round(node.now_s() - tl, 1)})
            relocalize(node, "멈춤")
            back = backtrack_piece(node, pc) if pc["kind"] in "FR" else None
            if back:
                print(f"    · 왔던 길 {back['len']:.2f} m 되돌아가기 ({'후진' if back['kind'] == 'R' else '전진'})", flush=True)
                rb, mb = follow_piece(node, back)
                if not rb:
                    print(f"    · 되돌아가기 못 함 ({mb}) — 이 자리에서 다시", flush=True)
                relocalize(node, "되돌아간 뒤")
            if STOP.is_set():
                break
            print(f"    · 조각 {i + 1}/{len(g['pieces'])} {what} 다시", flush=True)
            r, msg = follow_piece(node, pc)
        if k:
            node.recover.append(k)
        while not r and not STOP.is_set() and STEP_MODE:
            c = ask_choice(f"  ▶ 조각 {i + 1} 되찾기 실패 ({msg}) — Enter 한 번 더 · s 이 조각 건너뛰기 · q 구간 끝 : ")
            if c == "quit":
                break
            if c == "skip":
                print(f"    · 조각 {i + 1} 건너뜀 (사람)", flush=True)
                tries.append({"status": "skip", "msg": f"조각 {i + 1} ({what}) 사람이 건너뜀", "t": round(node.now_s() - tl, 1)})
                r, msg = True, ""
                break
            relocalize(node, "사람 요청")
            r, msg = follow_piece(node, pc)
        if not r:
            ok = False
            tries.append({"status": "fail", "msg": f"조각 {i + 1} ({what}): {msg}", "t": round(node.now_s() - tl, 1)})
            print(f"  ✗ {msg}", flush=True)
            break
    gx, gy, gth = g["goal"]
    p = node.pose()
    if ok and gth is not None and p:
        err = wrap(gth - p[2])
        if abs(err) > YAW_FIX:
            print(f"  · 정지점 방향 맞추기 {math.degrees(err):+.0f}°", flush=True)
            r, msg = follow_piece(node, {"kind": "S", "dth": err})
            if not r:
                ok = False
                tries.append({"status": "fail", "msg": f"방향 맞추기 {math.degrees(err):+.0f}°: {msg}", "t": round(node.now_s() - tl, 1)})
        sleep_s(node, 0.5)
        p = node.pose()
    dpos = math.hypot(gx - p[0], gy - p[1]) if p else 9.0
    dyaw = abs(wrap(gth - p[2])) if (p and gth is not None) else 0.0
    if ok and (dpos > (POS_OK if gth is not None else 0.15) or dyaw > YAW_OK):
        ok = False
        tries.append({"status": "fail", "msg": f"정지점 어긋남 {dpos * 100:.1f} cm · {math.degrees(dyaw):.1f}°"})
    if ok:
        tries.append({"status": "ok", "msg": "", "t": round(node.now_s() - tl, 1)})
    results.append({"seg": f"S{g['leg']}", "name": g["name"], "points": g["points"], "ok": ok, "tries": tries,
                    "sim_s": round(node.now_s() - tl, 1), "end_err_cm": round(dpos * 100, 1), "end_err_deg": round(math.degrees(dyaw), 1),
                    "head_fix_deg": list(node.head_fix), "head_skip_deg": list(node.head_skip), "recover": list(node.recover), "detour": bool(g.get("detour"))})
    print(f"{'✓' if ok else '✗'} {g['name']}: {node.now_s() - tl:.1f} s · 정지점 어긋남 {dpos * 100:.1f} cm · {math.degrees(dyaw):.1f}°"
          f" · 위치 추정 {node.loc or '-'}" + (f" · 조각 방향 맞추기 {node.head_fix}" if node.head_fix else "") + (f" · 건너뜀 {node.head_skip}" if node.head_skip else "") + (f" · 되찾기 {sum(node.recover)} 번" if node.recover else ""), flush=True)
    if ok:   # 도착 알림 (팔 · 미션 쪽이 들을 수 있게)
        node.arrived.publish(String(data=g.get("to") or g["name"].split("→")[-1]))
    # 11-16: 되찾기 (recover) · 사람 건너뛰기 (skip) 기록은 실패가 아님 — 실패 (fail) 만 봄
    return not any(t["status"] == "fail" and ("조각" in t["msg"] or "방향 맞추기" in t["msg"]) for t in tries)


def run_route(a, node, data, LIB, PTS, results):
    """11-15 기본 경로 + 들르기. 정지점마다: 들르기 요청이 있으면 그곳 먼저 (길 모음 cur>요청), 없으면 기본 경로 다음 곳"""
    base = list(data["route"])
    if base and base[0] == "HOME":
        base = base[1:]
    auto = {norm(k): [norm(x) for x in v] for k, v in data.get("auto_detour", {}).items()}
    dwell = float(data.get("dwell_s", 0.5))
    cur, idx, k, prev = "HOME", 0, 0, None
    visited = ["HOME"]
    while not STOP.is_set():
        if k > 0:
            for nm in auto.pop(cur, []):   # 시뮬 시험: 이 정지점에서 들를 곳 (계획 파일 auto_detour)
                with node.plock:
                    node.pending.append(nm)
                print(f"  ↪ 들르기 (계획 파일 auto_detour): {nm}", flush=True)
            if not a.step and dwell > 0:
                sleep_s(node, dwell)   # 도착 알림 뒤 들르기 요청을 받을 틈
        detour = None
        with node.plock:
            while node.pending and detour is None:
                nm = node.pending.popleft()
                if nm == cur or f"{cur}>{nm}" not in LIB:
                    print(f"  ⚠ 들르기 {nm} 건너뜀 ({'지금 자리' if nm == cur else f'길 {cur}>{nm} 이 길 모음에 없음'})", flush=True)
                    continue
                detour = nm
        if detour is None and idx >= len(base):
            break
        nxt = detour or base[idx]
        if a.step:
            go, nm = ask_next(f"\n▶ 지금 {cur} · 다음 {nxt}" + (" (들르기)" if detour else "")
                              + " — Enter 출발 · L1 / L2 / L3 (또는 상자 이름) = 그곳 먼저 · q 끝 : ")
            if not go:
                break
            if nm and nm != nxt:
                if nm not in NAMES or nm == cur or f"{cur}>{nm}" not in LIB:
                    print(f"  ✗ {nm} 으로는 못 감 (지금 {cur})", flush=True)
                    if detour:
                        with node.plock:
                            node.pending.appendleft(detour)
                    continue
                if detour:   # 밀린 들르기는 다음 정지점에서
                    with node.plock:
                        node.pending.appendleft(detour)
                detour, nxt = nm, nm
        if detour is None:
            idx += 1
        L = LIB[f"{cur}>{nxt}"]
        if prev is not None:   # 이음매 규칙 v2 (앞 길 도착 제자리 + 이 길 처음 제자리 ≤ 90° · 이어지는 후진 ≤ 0.40 m)
            ds, fr = first_spin(L["pieces"]), first_rev(L["pieces"])
            if prev["arrive_spin_deg"] + ds > SPIN_MAX_DEG + 1e-6:
                print(f"  ⚠ 이음매: 앞 길 도착 제자리 {prev['arrive_spin_deg']}° + 이 길 처음 제자리 {ds:.0f}° > {SPIN_MAX_DEG:.0f}°", flush=True)
            if prev["arrive_rev_m"] + fr > REV_MAX + 1e-6:
                print(f"  ⚠ 이음매: 앞 길 끝 후진 {prev['arrive_rev_m']} + 이 길 첫 후진 {fr:.2f} > {REV_MAX} m", flush=True)
        g = {"leg": k, "name": f"S{k} {'signal' if cur == 'HOME' else cur}→{'A1' if nxt == 'HOME' else nxt}", "points": [k, k + 1],
             "goal": L["goal"], "pieces": L["pieces"], "detour": bool(detour), "to": nxt}
        data["legs"].append(g)
        if detour:
            print(f"  ↪ 들르기 {nxt} → 그 뒤 기본 경로 {base[idx] if idx < len(base) else '(끝)'} 로 복귀", flush=True)
        if not run_leg(node, g, results):
            break
        visited.append(nxt)
        prev, cur, k = L, nxt, k + 1
        if cur == "HOME":
            with node.plock:
                left = list(node.pending)
            if left:
                print(f"  (출발 구역 도착 — 남은 들르기 요청 {left} 은 안 씀)", flush=True)
            break
    # 기록용 경로 yaml (주행기록 HTML 이 정지점을 읽음)
    lines = [f"# 기본 경로 + 들르기 (demo_follow route, {time.strftime('%Y-%m-%d %H:%M')}): 기본 {' → '.join(data['route'])} · 실제 {' → '.join(visited)}",
             "# 정지점 = 길 모음 (상자 실측) · 들른 곳은 detour", "route:"]
    s0 = PTS["HOME"]["start"]
    lines.append(f"  - {{stop: signal, x: {s0[0]:.4f}, y: {s0[1]:.4f}, face: {math.degrees(s0[2]):.1f}}}")
    for g in data["legs"][:len(visited) - 1]:
        y = g["to"]
        if y == "HOME":
            lines.append("  - A1   # 출발 구역으로 돌아옴")
        else:
            q = PTS[y]["goal"]
            lines.append(f"  - {{stop: {y}, x: {q[0]:.4f}, y: {q[1]:.4f}, face: {math.degrees(q[2]):.1f}}}" + ("   # 들르기" if g.get("detour") else ""))
    data["course_text"] = "\n".join(lines) + "\n"
    print(f"\n실제로 간 곳: {' → '.join(visited)}", flush=True)

def save(a, data, results, rows, n_legs):
    os.makedirs(a.out, exist_ok=True)
    open(os.path.join(a.out, data["course"]), "w", encoding="utf-8").write(data["course_text"])
    with open(os.path.join(a.out, "track.csv"), "w", encoding="utf-8") as f:
        f.write("t,x,y,yaw,v,w,step,v_nav,w_nav,loc\n")
        for r in rows:
            f.write(f"{r[0]:.2f},{r[1]:.4f},{r[2]:.4f},{r[3]:.1f},{r[4]:.3f},{r[5]:.3f},{r[6]},{r[7]:.3f},{r[8]:.3f},{r[9]}\n")
    json.dump(results, open(os.path.join(a.out, "course_result.json"), "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    moves = []
    for res in results:
        k = int(res["seg"][1:])
        R = [r for r in rows if r[6] == k]
        if len(R) < 2:
            continue
        L = rev = 0.0
        for p, q in zip(R, R[1:]):
            ds = math.hypot(q[1] - p[1], q[2] - p[2])
            L += ds
            rev += ds if q[4] < -0.02 else 0.0
        g = next(x for x in data["legs"] if x["leg"] == k)
        moves.append({"단계번호": k, "목표이름": res["name"].split(" ", 1)[1], "단계": res["name"], "결과": "성공" if res["ok"] else "실패",
                      "시간_s": round(R[-1][0] - R[0][0], 1), "길이_m": round(L, 2), "후진_m": round(rev, 2), "제자리_번": 0, "제자리_deg": 0,
                      "재계획_바뀜": 0, "복구": 0, "멈춤_s": 0, "최소여유_cm": "", "목표_x": g["goal"][0], "목표_y": g["goal"][1],
                      "도착_x": round(R[-1][1], 3), "도착_y": round(R[-1][2], 3)})
    json.dump(moves, open(os.path.join(a.out, "moves.json"), "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    n_ok = sum(1 for r in results if r["ok"])
    with open(os.path.join(a.out, "summary.txt"), "w", encoding="utf-8") as f:
        f.write(f"계획 따라가기 ({'loopback 시뮬' if a.sim else '실물'}): 구간 {n_ok}/{n_legs} 성공 · 계획 {data['plan']}\n")
        for r in results:
            f.write(f"{r['name']}: {'성공' if r['ok'] else '실패'} · {r['sim_s']} s · 정지점 어긋남 {r['end_err_cm']} cm · {r['end_err_deg']}°"
                    + (f" · {r['tries'][-1]['msg']}" if not r["ok"] else "") + "\n")
    print(f"\n구간 {n_ok}/{n_legs} 성공 → 기록 {a.out}", flush=True)


if __name__ == "__main__":
    main()
