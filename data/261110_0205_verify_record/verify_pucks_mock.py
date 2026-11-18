"""mock 미션 보조 (팀 저장소 밖, 재검증 기록 확인용): verify 단계 동안 /perception/pucks 를 지도 좌표로 낸다.

mock_robot.py 는 /perception/pucks 를 내지 않아서, mission_node 의 퍽 ID 기록을 보려면 이 노드를 같이 띄운다.
  /mission/status 의 단계 이름이 "verify L cx cy z expect9 yshape" 이면 expect9 의 퍽 칸 (0 이 아닌 칸) 에 퍽을 놓는다.
  ID 는 처음 본 장소 순서로 0 부터 차례로 (같은 장소를 다시 재면 같은 ID).
  태그는 --read_every 장에 한 번만 읽힌 것으로 (나머지 장은 tag_id -1) — 기록이 태그 못 읽은 장을 같은 퍽으로 묶는지 본다.
  --scenario ok | dup (두 번째 장소 첫 퍽 = 첫 장소 첫 퍽 ID) | unread (세 번째 장소 마지막 퍽은 태그를 한 번도 못 읽음)
  위치는 ±--jitter 흔들림. 칸 밖 퍽 하나 (장소 중심에서 +X 0.20 m) 와 바닥 퍽 하나 (z 0.03) 를 섞어 걸러지는지 본다.
"""
import argparse
import random
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from robot_interfaces.msg import MissionStatus, Puck, PuckArray

GRID = 0.06


class VerifyPucks(Node):
    def __init__(self, a):
        super().__init__("verify_pucks_mock")
        self.a = a
        self.step = ""
        self.ids = {}       # place -> [id per occupied cell]
        self.next_id = 0
        self.frame = 0
        self.pub = self.create_publisher(PuckArray, "/perception/pucks", QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(MissionStatus, "/mission/status", self.on_status, 10)
        self.create_timer(1.0 / a.rate, self.tick)

    def on_status(self, m):
        self.step = m.step_name

    def tick(self):
        f = self.step.split()
        if len(f) != 7 or f[0] != "verify":
            return
        place, cx, cy, expect = f[1], float(f[2]), float(f[3]), f[5]
        cells = [k for k in range(9) if expect[k] != "0"]
        if place not in self.ids:
            self.ids[place] = list(range(self.next_id, self.next_id + len(cells)))
            self.next_id += len(cells)
            order = list(self.ids)
            if self.a.scenario == "dup" and len(order) == 2:
                self.ids[place][0] = self.ids[order[0]][0]
        ids = self.ids[place]
        self.frame += 1
        msg = PuckArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.target_index = -1
        last_place = self.a.scenario == "unread" and list(self.ids).index(place) == 2
        for i, k in enumerate(cells):
            r, c = k // 3, k % 3
            p = Puck()
            p.pose.position.x = cx + (c - 1) * GRID + random.uniform(-self.a.jitter, self.a.jitter)
            p.pose.position.y = cy + (1 - r) * GRID + random.uniform(-self.a.jitter, self.a.jitter)
            p.pose.position.z = 0.10
            p.pose.orientation.w = 1.0
            read = self.frame % self.a.read_every == 0 and not (last_place and i == len(cells) - 1)
            p.tag_id = ids[i] if read else -1
            p.camera = Puck.CAMERA_ARM
            msg.pucks.append(p)
        for dx, z in ((0.20, 0.10), (0.0, 0.03)):   # 칸 밖 · 바닥 퍽 (기록에 들어가면 안 됨)
            p = Puck()
            p.pose.position.x, p.pose.position.y, p.pose.position.z = cx + dx, cy, z
            p.pose.orientation.w = 1.0
            p.tag_id = 11 - len(msg.pucks) % 3
            msg.pucks.append(p)
        self.pub.publish(msg)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenario", default="ok", choices=["ok", "dup", "unread"])
    ap.add_argument("--read_every", type=int, default=3)
    ap.add_argument("--jitter", type=float, default=0.004)
    ap.add_argument("--rate", type=float, default=10.0)
    a, ros_args = ap.parse_known_args()
    rclpy.init(args=[sys.argv[0]] + ros_args)
    rclpy.spin(VerifyPucks(a))


if __name__ == "__main__":
    main()
