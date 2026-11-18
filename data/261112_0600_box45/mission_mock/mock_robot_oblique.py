"""mission_node plan 모드 연동 시험용 가짜 서버 (drive · manipulator · perception). 실물 · 시뮬레이터 대신.

[비스듬히 서기 시험용 사본, 261112_0600_box45/mission_mock] 팀 mission_planner/test/mock_robot.py 를 복사해 늘림 (원본은 그대로):
  - 차체 자세 흉내: map 자세를 들고 odom -> base_link TF (20 Hz) 를 냄. /drive/navigate 목표가 0.25 m 안이면 icr_x 둘레 제자리 회전
    (drive_node 처럼 spin), 멀면 목표로 순간 이동. /drive/align box_only 면 렌즈 (base x 0.13) 에서 x 축 빔을 진짜 상자 (지도 칸 +
    --box_shift, --box_rot) 에 쏴 렌즈 -> 면 거리가 목표가 되게 축을 따라 옮김 (align_sim 에 기록). 상자가 없으면 움직이지 않음
  - /perception/place_box/measure_front (Trigger): 지금 차체 자리에서 가까운 장소 상자를 status {"ok": true, "camera": "front"} +
    odom 자세로 냄 (measure 와 같은 latched 토픽). /perception/place_box/prior 받은 값 · 시각도 기록 (measure_front 에 prior_age_s)
  - 기록 (mock_out.json) 에 measure_front · align_sim · robot_at_front (재기 때 차체 map 자세) 추가

  - /drive/navigate · /drive/align · /drive/execute_skill · /manipulator/pose · /manipulator/pick · /manipulator/execute_skill: 잠깐 뒤 성공
    (execute_skill 은 이름을 기록, --reject <접두어> 면 그 이름은 거절)
  - /perception/set_mode: READ_SIGNAL 이면 /perception/signal CONFIRMED (--signal), READ_BARCODE 면 /perception/barcode READ (--barcode)
  - /perception/check_dice: observed · passed true (요청 칸 · 기대값 기록)
  - /perception/place_box/measure (#73): 마지막 navigate 목표에서 가까운 장소 L 의 지도 칸 중심을 --box_shift 만큼 밀고 --box_rot 만큼 돌린
    상자를 odom 기준 /perception/place_box · /status 로 냄 (map -> odom 정적 TF = --odom_offset). --box_fail 이면 ok false
끝에 기록을 --out JSON 으로 씀 (/mission/status 가 DONE · FAILED 가 되면).
--real_nav: 주행은 진짜 (drive_node /drive/navigate → Nav2, Gazebo 등). /drive/navigate 서버 · map -> odom 정적 TF 를 내지 않고,
  이동 기록은 map -> base_link TF 로 (navigate 목표 대신 도착 자세), 장소 상자 측정도 그 TF · map -> odom TF 로.
  drive_node 의 /drive/align · /drive/execute_skill 은 다른 이름으로 remap 해 두고 이 mock 이 받는다 (정렬 · 회전은 가짜).
"""
import argparse
import sys
import json
import threading
import time

import rclpy
import rclpy.time
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from robot_interfaces.action import AlignBase, ArmPose, ExecuteSkill, Pick
from robot_interfaces.msg import Barcode, MissionStatus, SignalSequence
from robot_interfaces.srv import CheckDice, SetPerceptionMode
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformBroadcaster, TransformListener
import math
import os
import yaml


class Mock(Node):
    def __init__(self, a):
        super().__init__("mock_robot")
        self.a = a
        self.log = dict(navigate=[], align=[], drive_skill=[], pose=[], pick=[], skill=[], set_mode=[], check_dice=[], place_box=[],
                        measure_front=[], align_sim=[], nav_sim=[])
        self.robot = [float(v) for v in a.start.split(",")]   # 차체 map 자세 (x, y, yaw rad)
        self.robot_lock = threading.Lock()
        self.prior = None                                      # (/perception/place_box/prior 메시지, 받은 시각)
        self.mode = 0
        cg = ReentrantCallbackGroup()

        def server(kind, typ, name, result_fn):
            def cb(gh):
                self.log[kind].append(self.describe(kind, gh.request))
                time.sleep(a.delay)
                r = result_fn(gh.request)
                if r is None:
                    gh.abort()
                    return typ.Result(success=False, message="mock reject") if hasattr(typ.Result(), "message") else typ.Result()
                gh.succeed()
                return r
            ActionServer(self, typ, name, cb, callback_group=cg)

        if not a.real_nav:
            server("navigate", NavigateToPose, "/drive/navigate", self.sim_navigate)
        server("align", AlignBase, "/drive/align", self.sim_align)
        server("drive_skill", ExecuteSkill, "/drive/execute_skill",
               lambda g: None if (a.reject and g.policy.startswith(a.reject)) else ExecuteSkill.Result(success=True))
        server("pose", ArmPose, "/manipulator/pose", lambda g: ArmPose.Result(success=True))
        server("pick", Pick, "/manipulator/pick", lambda g: Pick.Result(success=True, grasped=True))
        server("skill", ExecuteSkill, "/manipulator/execute_skill",
               lambda g: None if (a.reject and g.policy.startswith(a.reject)) else ExecuteSkill.Result(success=True))
        self.create_service(SetPerceptionMode, "/perception/set_mode", self.on_mode, callback_group=cg)
        self.create_service(CheckDice, "/perception/check_dice", self.on_dice, callback_group=cg)
        # 장소 상자 측정 (#73)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pb_pub = self.create_publisher(PoseStamped, "/perception/place_box", latched)
        self.pb_status_pub = self.create_publisher(String, "/perception/place_box/status", latched)
        self.create_service(Trigger, "/perception/place_box/measure", self.on_place_box, callback_group=cg)
        self.create_service(Trigger, "/perception/place_box/measure_front", self.on_measure_front, callback_group=cg)
        self.create_subscription(PoseStamped, "/perception/place_box/prior", self.on_prior, latched, callback_group=cg)
        ap_file = a.approach or os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "config", "approach_poses.yaml")
        table = yaml.safe_load(open(ap_file, encoding="utf-8"))
        self.places = {L: (table[L]["cells"]["0,0"]["x"], table[L]["cells"]["0,0"]["y"], table[L]["stage"]) for L in ("L1", "L2", "L3")}
        self.odom_off = [float(v) for v in a.odom_offset.split(",")]   # map -> odom (x, y, yaw deg)
        self.tf_buf = Buffer()
        self.tf_listener = TransformListener(self.tf_buf, self)
        self.tf_pub = StaticTransformBroadcaster(self)
        if not a.real_nav:
            self.send_odom_tf()
            self.tf_dyn = TransformBroadcaster(self)
            self.create_timer(0.05, self.send_base_tf, callback_group=cg)
        q = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.sig_pub = self.create_publisher(SignalSequence, "/perception/signal", q)
        self.bar_pub = self.create_publisher(Barcode, "/perception/barcode", q)
        self.estop_pub = self.create_publisher(Bool, "/e_stop", QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.estop_pub.publish(Bool(data=False))
        self.create_subscription(MissionStatus, "/mission/status", self.on_status, 10, callback_group=cg)
        self.create_timer(0.3, self.pub_perception, callback_group=cg)
        self.t_mode = {}
        self.final = None
        self.steps_seen = []
        self.last_step = None

    def send_odom_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id, t.child_frame_id = "map", "odom"
        t.transform.translation.x, t.transform.translation.y = self.odom_off[0], self.odom_off[1]
        yo = math.radians(self.odom_off[2])
        t.transform.rotation.z, t.transform.rotation.w = math.sin(yo / 2), math.cos(yo / 2)
        self.tf_pub.sendTransform(t)

    def lookup(self, parent, child):
        """TF parent -> child (x, y, yaw). 없으면 None."""
        try:
            t = self.tf_buf.lookup_transform(parent, child, rclpy.time.Time())
        except Exception:  # noqa: BLE001
            return None
        r = t.transform.rotation
        return (t.transform.translation.x, t.transform.translation.y,
                math.atan2(2 * (r.w * r.z + r.x * r.y), 1 - 2 * (r.y * r.y + r.z * r.z)))

    # ---------------- 비스듬히 서기 시험: 차체 자세 · 진짜 상자 ----------------
    def send_base_tf(self):
        """odom -> base_link = inv(map -> odom) * map 자세."""
        with self.robot_lock:
            x, y, yaw = self.robot
        ox, oy, oyaw = self.odom_off[0], self.odom_off[1], math.radians(self.odom_off[2])
        c, s_ = math.cos(-oyaw), math.sin(-oyaw)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id, t.child_frame_id = "odom", "base_link"
        t.transform.translation.x = c * (x - ox) - s_ * (y - oy)
        t.transform.translation.y = s_ * (x - ox) + c * (y - oy)
        yo = yaw - oyaw
        t.transform.rotation.z, t.transform.rotation.w = math.sin(yo / 2), math.cos(yo / 2)
        self.tf_dyn.sendTransform(t)

    def true_box(self, L):
        """진짜 상자 (지도 중심 x, y, 요 rad) = 지도 칸 + --box_shift, --box_rot."""
        dx, dy = (float(v) for v in self.a.box_shift.split(","))
        return self.places[L][0] + dx, self.places[L][1] + dy, math.radians(self.a.box_rot)

    def nearest_place(self, x, y):
        return min(self.places, key=lambda k: math.hypot(self.places[k][0] - x, self.places[k][1] - y))

    def sim_navigate(self, g):
        p = g.pose.pose
        gx, gy = p.position.x, p.position.y
        gyaw = 2 * math.atan2(p.orientation.z, p.orientation.w)
        with self.robot_lock:
            x, y, yaw = self.robot
            if math.hypot(gx - x, gy - y) <= 0.25:   # drive_node: 가까우면 Nav2 spin (실제 회전 중심 icr_x 둘레)
                dyaw = math.remainder(gyaw - yaw, 2 * math.pi)
                ix, iy = x + math.cos(yaw) * self.a.icr_x, y + math.sin(yaw) * self.a.icr_x
                ny = yaw + dyaw
                self.robot = [ix - math.cos(ny) * self.a.icr_x, iy - math.sin(ny) * self.a.icr_x, ny]
                kind = "spin"
            else:
                self.robot = [gx, gy, gyaw]
                kind = "goto"
            self.log["nav_sim"].append(dict(kind=kind, goal=[round(gx, 4), round(gy, 4), round(gyaw, 4)],
                                            at=[round(v, 5) for v in self.robot]))
        self.settle_tf()
        return NavigateToPose.Result()

    def settle_tf(self):
        """옮긴 자세를 바로 TF 로 내고 잠깐 기다림 (실물은 도착 뒤 TF 가 이어짐. 안 기다리면 mission 이 옮기기 전 TF 를 봄)."""
        self.send_base_tf()
        time.sleep(0.2)
        self.send_base_tf()

    def ray_box(self, x0, y0, yaw, L):
        """(x0, y0) 에서 yaw 방향 빔이 상자 L 에 처음 닿는 거리 (없으면 None). 슬랩 방법."""
        bx, by, byaw = self.true_box(L)
        c, s_ = math.cos(-byaw), math.sin(-byaw)
        ox, oy = c * (x0 - bx) - s_ * (y0 - by), s_ * (x0 - bx) + c * (y0 - by)
        dx, dy = math.cos(yaw - byaw), math.sin(yaw - byaw)
        h = self.a.half
        t0, t1 = -math.inf, math.inf
        for o, d in ((ox, dx), (oy, dy)):
            if abs(d) < 1e-12:
                if abs(o) > h:
                    return None
                continue
            ta, tb = sorted(((-h - o) / d, (h - o) / d))
            t0, t1 = max(t0, ta), min(t1, tb)
        return t0 if t0 <= t1 and t0 > 0 else None

    def sim_align(self, g):
        if not g.box_only:
            return AlignBase.Result(success=True)
        with self.robot_lock:
            x, y, yaw = self.robot
            L = self.nearest_place(x, y)
            lx, ly = x + math.cos(yaw) * self.a.tf_x, y + math.sin(yaw) * self.a.tf_x
            d = self.ray_box(lx, ly, yaw, L)
            rec = dict(place=L, target=round(g.target_distance, 4), before=[round(v, 5) for v in self.robot],
                       lens_to_face_before=None if d is None else round(d, 5))
            if d is not None:
                mv = d - g.target_distance
                self.robot = [x + math.cos(yaw) * mv, y + math.sin(yaw) * mv, yaw]
                bx, by, byaw = self.true_box(L)
                cx = math.cos(-yaw) * (bx - self.robot[0]) - math.sin(-yaw) * (by - self.robot[1])
                cy = math.sin(-yaw) * (bx - self.robot[0]) + math.cos(-yaw) * (by - self.robot[1])
                rec.update(moved=round(mv, 5), after=[round(v, 5) for v in self.robot], center_base_after=[round(cx, 5), round(cy, 5)],
                           beta_after_deg=round(math.degrees(math.remainder(byaw - yaw, math.pi / 2)), 3))
            self.log["align_sim"].append(rec)
        self.settle_tf()
        return AlignBase.Result(success=True)

    def on_prior(self, m):
        self.prior = (m, time.time())

    def on_measure_front(self, req, res):
        if not (self.mode & 1):   # SEARCH_FRONT 비트 (perception 과 같게: 꺼져 있으면 거절)
            res.success, res.message = False, "mock: SEARCH_FRONT 꺼짐"
            self.log["measure_front"].append(dict(ok=False, why="SEARCH_FRONT off", mode=int(self.mode)))
            return res
        with self.robot_lock:
            here = list(self.robot)
        L = self.nearest_place(here[0], here[1])
        mx, my, byaw = self.true_box(L)
        ox, oy, oyaw = self.odom_off[0], self.odom_off[1], math.radians(self.odom_off[2])
        c, s_ = math.cos(-oyaw), math.sin(-oyaw)
        px, py = c * (mx - ox) - s_ * (my - oy), s_ * (mx - ox) + c * (my - oy)
        yaw = byaw - oyaw
        pr = None
        if self.prior is not None:
            pm, pt = self.prior
            pr = dict(frame=pm.header.frame_id, xy=[round(pm.pose.position.x, 4), round(pm.pose.position.y, 4)], age_s=round(time.time() - pt, 2))
        self.pb_status_pub.publish(String(data=f'{{"ok": true, "frames": 3, "x": {px:.4f}, "y": {py:.4f}, "yaw": {yaw:.4f}, "frame": "odom", "camera": "front"}}'))
        m = PoseStamped()
        m.header.stamp, m.header.frame_id = self.get_clock().now().to_msg(), "odom"
        m.pose.position.x, m.pose.position.y, m.pose.position.z = px, py, 0.07
        m.pose.orientation.z, m.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.pb_pub.publish(m)
        self.log["measure_front"].append(dict(ok=True, place=L, map_center=[round(mx, 4), round(my, 4)], rot_deg=self.a.box_rot,
                                              robot=[round(v, 5) for v in here], prior=pr))
        res.success, res.message = True, "mock measure_front"
        return res

    def describe(self, kind, g):
        if kind == "navigate":
            p = g.pose.pose
            import math
            return [round(p.position.x, 4), round(p.position.y, 4), round(2 * math.atan2(p.orientation.z, p.orientation.w), 4)]
        if kind == "align":
            return round(g.target_distance, 4)
        if kind == "pose":
            return g.name
        if kind == "pick":
            p = g.target.pose.position
            return [g.target.header.frame_id, round(p.x, 4), round(p.y, 4), round(p.z, 4)]
        if kind in ("skill", "drive_skill"):
            return g.policy
        return None

    def on_mode(self, req, res):
        self.mode = req.mode
        self.log["set_mode"].append(int(req.mode))
        for bit in (32, 64):
            if req.mode & bit:
                self.t_mode[bit] = time.time()
        res.success = True
        return res

    def on_dice(self, req, res):
        self.log["check_dice"].append(dict(frame=req.header.frame_id, cells=[[round(c.x, 3), round(c.y, 3), round(c.z, 3)] for c in req.cells],
                                           expect="".join(str(int(e)) for e in req.expect), yellow_shape=int(req.yellow_shape)))
        res.observed, res.passed, res.message = True, True, "mock ok"
        return res

    def on_place_box(self, req, res):
        nav = self.log["navigate"][-1] if self.log["navigate"] else [0.0, 0.0, 0.0]
        if self.a.real_nav:   # 지금 로봇 자리 (map) 에서 가까운 장소, 상자는 진짜 map -> odom 으로
            here = self.lookup("map", "base_link")
            mo = self.lookup("map", "odom")
            if here is None or mo is None:
                self.pb_status_pub.publish(String(data='{"ok": false, "why": "mock: TF 없음"}'))
                res.success, res.message = True, "mock measure (no TF)"
                return res
            nav = list(here)
            self.odom_off = [mo[0], mo[1], math.degrees(mo[2])]
        L = min(self.places, key=lambda k: math.hypot(self.places[k][2][0] - nav[0], self.places[k][2][1] - nav[1]))
        if self.a.box_fail:
            self.log["place_box"].append(dict(place=L, ok=False))
            self.pb_status_pub.publish(String(data='{"ok": false, "why": "mock"}'))
            res.success, res.message = True, "mock measure (fail)"
            return res
        dx, dy = (float(v) for v in self.a.box_shift.split(","))
        mx, my = self.places[L][0] + dx, self.places[L][1] + dy               # 지도에서 밀린 상자 중심
        ox, oy, oyaw = self.odom_off[0], self.odom_off[1], math.radians(self.odom_off[2])
        c, s_ = math.cos(-oyaw), math.sin(-oyaw)
        px, py = c * (mx - ox) - s_ * (my - oy), s_ * (mx - ox) + c * (my - oy)   # odom 좌표
        yaw = math.radians(self.a.box_rot) - oyaw                              # 상자 요 (odom)
        self.pb_status_pub.publish(String(data=f'{{"ok": true, "frames": 3, "x": {px:.4f}, "y": {py:.4f}, "yaw": {yaw:.4f}, "frame": "odom"}}'))
        m = PoseStamped()
        m.header.stamp, m.header.frame_id = self.get_clock().now().to_msg(), "odom"
        m.pose.position.x, m.pose.position.y, m.pose.position.z = px, py, 0.07
        m.pose.orientation.z, m.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.pb_pub.publish(m)
        self.log["place_box"].append(dict(place=L, map_center=[round(mx, 4), round(my, 4)], rot_deg=self.a.box_rot))
        res.success, res.message = True, "mock measure"
        return res

    def pub_perception(self):
        now = self.get_clock().now().to_msg()
        if self.mode & 32 and time.time() - self.t_mode.get(32, 0) > self.a.signal_after:
            m = SignalSequence(status=SignalSequence.CONFIRMED, sequence=self.a.signal, cycles=2)
            m.header.stamp, m.header.frame_id = now, "base_link"
            self.sig_pub.publish(m)
        if self.mode & 64 and time.time() - self.t_mode.get(64, 0) > 1.0:
            b = Barcode(source=Barcode.READ, read_ok=True, text=self.a.barcode, symbols=3)
            b.header.stamp, b.header.frame_id = now, "base_link"
            self.bar_pub.publish(b)

    def on_status(self, m):
        if m.step_name and (not self.steps_seen or self.steps_seen[-1] != (m.step_index, m.step_name)):
            self.steps_seen.append((m.step_index, m.step_name))
            if self.a.real_nav:   # 단계가 바뀔 때 로봇 자리 (진짜 주행 도착 확인용)
                prev = self.last_step
                self.last_step = m.step_name
                if prev and prev.startswith("goto"):
                    here = self.lookup("map", "base_link")
                    self.log["navigate"].append(dict(step=prev, at=[round(v, 4) for v in here] if here else None,
                                                     t=round(time.time(), 1)))
        if m.state in (MissionStatus.DONE, MissionStatus.FAILED) and self.final is None:
            self.final = dict(state="DONE" if m.state == MissionStatus.DONE else "FAILED", reason=m.reason, step=m.step_name)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--signal", default="RGRRGRGGYGYG")
    ap.add_argument("--barcode", default="P2-01")
    ap.add_argument("--signal_after", type=float, default=2.0)
    ap.add_argument("--delay", type=float, default=0.02)
    ap.add_argument("--reject", default="")
    ap.add_argument("--timeout", type=float, default=300.0)
    ap.add_argument("--out", required=True)
    ap.add_argument("--approach", default="", help="approach_poses.yaml (기본 ../config)")
    ap.add_argument("--box_shift", default="0.02,-0.01", help="장소 상자를 지도 칸에서 민 거리 dx,dy [m]")
    ap.add_argument("--box_rot", type=float, default=5.0, help="장소 상자 요 [deg]")
    ap.add_argument("--box_fail", action="store_true", help="place_box 측정 실패 (ok false)")
    ap.add_argument("--odom_offset", default="0.3,-0.2,10", help="map -> odom 정적 TF x,y,yaw[deg] (--real_nav 면 안 냄)")
    ap.add_argument("--real_nav", action="store_true", help="주행은 진짜 drive_node · Nav2 (/drive/navigate 서버 · 정적 TF 안 냄)")
    ap.add_argument("--start", default="0.69365,0.95,3.14159", help="처음 차체 map 자세 x,y,yaw[rad] (출발 구역)")
    ap.add_argument("--icr_x", type=float, default=-0.0658, help="제자리 회전 중심 base x [m]")
    ap.add_argument("--tf_x", type=float, default=0.13, help="TFmini 렌즈 base x [m]")
    ap.add_argument("--half", type=float, default=0.1025, help="장소 상자 반 변 [m]")
    a, ros_args = ap.parse_known_args()
    rclpy.init(args=[sys.argv[0]] + ros_args)
    n = Mock(a)
    ex = MultiThreadedExecutor(num_threads=8)
    ex.add_node(n)
    th = threading.Thread(target=ex.spin, daemon=True)
    th.start()
    t0 = time.time()
    while n.final is None and time.time() - t0 < a.timeout:
        time.sleep(0.2)
    out = dict(final=n.final or dict(state="TIMEOUT"), elapsed_s=round(time.time() - t0, 1), **n.log,
               steps_seen=[f"[{i}] {s}" for i, s in n.steps_seen])
    json.dump(out, open(a.out, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    print(f"[MOCK] {out['final']} navigate {len(n.log['navigate'])} align {len(n.log['align'])} drive_skill {len(n.log['drive_skill'])} pick {len(n.log['pick'])} "
          f"skill {len(n.log['skill'])} check_dice {len(n.log['check_dice'])} ({out['elapsed_s']} s)", flush=True)
    ex.shutdown()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
