#!/usr/bin/env python3
"""PAUSED 재개 · 정지 뒤 새 시작 모의 시험 조종기.
사용: pause_ctl.py <log_root> <동작:접두어조각> ...   동작 = pause | stop
  기록 jsonl 의 snapshot 줄 prefix 에 조각이 처음 나오면 delay 뒤 그 동작:
  pause: /e_stop true → /mission/status PAUSED 확인 → hold 초 → /e_stop false → /mission/start (재개)
  stop : /mission/stop → IDLE 확인 → /mission/start (새 미션)
결과를 한 줄씩 stdout 에 (t, 동작, 확인한 상태)."""
import glob, json, os, sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from robot_interfaces.msg import MissionStatus

NAMES = {0: "IDLE", 1: "RUNNING", 2: "PAUSED", 3: "DONE", 4: "FAILED"}


class Ctl(Node):
    def __init__(self):
        super().__init__("pause_ctl")
        q = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.estop = self.create_publisher(Bool, "/e_stop", q)
        self.state = None
        self.step = ""
        self.create_subscription(MissionStatus, "/mission/status", self.on_st, 10)
        self.start = self.create_client(Trigger, "/mission/start")
        self.stop = self.create_client(Trigger, "/mission/stop")

    def on_st(self, m):
        self.state, self.step = m.state, m.step_name

    def spin_for(self, s):
        end = time.time() + s
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_state(self, want, s=10.0):
        end = time.time() + s
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.state == want:
                return True
        return False

    def call(self, cli):
        cli.wait_for_service(timeout_sec=5.0)
        f = cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, f, timeout_sec=5.0)
        r = f.result()
        return (r.success, r.message) if r else (None, "no response")


def snapshot_lines(root):
    out = []
    for p in sorted(glob.glob(os.path.join(root, "*", "mission_log.jsonl"))):
        with open(p) as f:
            for line in f:
                try:
                    d = json.loads(line)
                except ValueError:
                    continue
                if d.get("kind") == "snapshot":
                    out.append(d.get("prefix", ""))
    return out


def log(*a):
    print(f"{time.time():.3f}", *a, flush=True)


def main():
    root = sys.argv[1]
    plan = [a.split(":", 1) for a in sys.argv[2:]]
    delay, hold = 1.0, 3.0   # 요청 뒤 1.0 s = 3 장 중 첫 장(0.5 s) 뒤 · 둘째(1.0 s) 즈음에 끊음
    rclpy.init()
    n = Ctl()
    n.estop.publish(Bool(data=False))
    for act, frag in plan:
        log("wait", act, frag)
        t_end = time.time() + 240
        while time.time() < t_end and not any(frag in p for p in snapshot_lines(root)):
            n.spin_for(0.1)
        if time.time() >= t_end:
            log("TIMEOUT waiting", frag); break
        n.spin_for(delay)
        if act == "pause":
            n.estop.publish(Bool(data=True))
            ok = n.wait_state(2)
            log("estop true -> PAUSED", ok, "step", n.step)
            n.spin_for(hold)
            n.estop.publish(Bool(data=False))
            n.spin_for(0.5)
            log("start", *n.call(n.start))
            log("RUNNING", n.wait_state(1))
        elif act == "stop":
            log("stop", *n.call(n.stop))
            log("IDLE", n.wait_state(0), "step", n.step)
            n.spin_for(1.0)
            log("start", *n.call(n.start))
            log("RUNNING", n.wait_state(1))
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
