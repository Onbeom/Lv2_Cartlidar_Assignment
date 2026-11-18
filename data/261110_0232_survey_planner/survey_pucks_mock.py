"""mock 미션 보조 (팀 저장소 밖, survey → planner 연결 확인용): survey_view 단계 동안 그 박스의 퍽 2 개를 /perception/pucks 로 낸다.

진짜 perception_survey(puck_survey) 노드가 이 관측을 쌓아 완료 파일을 쓰고, mission_node 가 그것을 계획기 등록표로 바꾸는지 본다.
  --truth <yaml>   pucks [{id, color, shape, box, side}] (계획기 등록표 형식). 이 배치를 '실제' 퍽으로 냄
  /mission/status 단계 이름이 "survey_view <box> <arm_pose>" 이면 그 박스 퍽을 (박스 중심 X + side × 0.04, Y, z 0.10) 에 냄 (frame map)
  --unread_box <n> 첫 방문 때 그 박스 side −1 퍽은 태그를 못 읽음 (id −1) → finish 실패 → revisit 로 다시 보는 길 (#53)
  팔 카메라 퍽처럼: camera ARM, confidence 0.5 (tagged_min_confidence 0.4 이상), orientation_valid · 선 자세 (z 축 위)
"""
import argparse
import random
import sys

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from robot_interfaces.msg import MissionStatus, Puck, PuckArray

BOX = {1: (1.468, 0.950), 2: (3.670, 3.152), 3: (2.936, 5.354), 4: (0.734, 5.354), 5: (2.202, 5.354), 6: (3.670, 3.886)}
COLOR = {"red": Puck.COLOR_RED, "green": Puck.COLOR_GREEN}
SHAPE = {"cylinder": Puck.SHAPE_CYLINDER, "box": Puck.SHAPE_CUBOID}


class SurveyPucks(Node):
    def __init__(self, a):
        super().__init__("survey_pucks_mock")
        self.a = a
        self.truth = yaml.safe_load(open(a.truth, encoding="utf-8"))["pucks"]
        self.step = ""
        self.visits = {}
        self.pub = self.create_publisher(PuckArray, "/perception/pucks", QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(MissionStatus, "/mission/status", self.on_status, 10)
        self.create_timer(0.1, self.tick)

    def on_status(self, m):
        if m.step_name != self.step and m.step_name.startswith("survey_view"):
            box = int(m.step_name.split()[1])
            self.visits[box] = self.visits.get(box, 0) + 1
            self.get_logger().info(f"{m.step_name} (visit {self.visits[box]})")
        self.step = m.step_name

    def tick(self):
        f = self.step.split()
        if len(f) < 2 or f[0] != "survey_view":
            return
        box = int(f[1])
        msg = PuckArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.target_index = -1
        for t in self.truth:
            if t["box"] != box:
                continue
            p = Puck()
            p.pose.position.x = BOX[box][0] + t["side"] * 0.04 + random.uniform(-0.004, 0.004)
            p.pose.position.y = BOX[box][1] + random.uniform(-0.004, 0.004)
            p.pose.position.z = 0.10
            p.pose.orientation.w = 1.0
            p.orientation_valid = True
            p.standing = True
            p.color = COLOR[t["color"]]
            p.shape = SHAPE[t["shape"]]
            p.confidence = 0.5
            p.camera = Puck.CAMERA_ARM
            unread = box == self.a.unread_box and t["side"] == -1 and self.visits.get(box, 0) == 1
            p.tag_id = -1 if unread else int(t["id"])
            msg.pucks.append(p)
        self.pub.publish(msg)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--truth", required=True)
    ap.add_argument("--unread_box", type=int, default=0)
    a, ros_args = ap.parse_known_args()
    rclpy.init(args=[sys.argv[0]] + ros_args)
    rclpy.spin(SurveyPucks(a))


if __name__ == "__main__":
    main()
