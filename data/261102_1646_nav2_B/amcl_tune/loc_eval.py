"""재생용 채점 기록: /sim/truth(base_link 참값, mission_map) 0.1 s 마다, 그 시각의 TF mission_map → base_link(위치 추정)를 찾아 csv 로 남긴다.
TF 가 늦게 오므로 참값을 0.6 s 묵혔다가 찾고, 못 찾으면 빈칸."""
import csv
import math
import sys

import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Eval(Node):
    def __init__(self, out):
        super().__init__('loc_eval')
        self.buf = tf2_ros.Buffer(cache_time=Duration(seconds=30))
        self.tl = tf2_ros.TransformListener(self.buf, self)
        self.q, self.last = [], -1.0
        self.f = open(out, 'w', newline='')
        self.w = csv.writer(self.f)
        self.w.writerow(['t', 'tx', 'ty', 'tyaw', 'ex', 'ey', 'eyaw'])
        self.create_subscription(PoseStamped, '/sim/truth', self.on_truth, 50)
        self.create_timer(0.1, self.flush)

    def on_truth(self, m):
        t = Time.from_msg(m.header.stamp).nanoseconds * 1e-9
        if t - self.last >= 0.1:
            self.last = t
            self.q.append((m.header.stamp, t, m.pose.position.x, m.pose.position.y, yaw(m.pose.orientation)))

    def flush(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        while self.q and now - self.q[0][1] > 0.6:
            st, t, x, y, a = self.q.pop(0)
            try:
                tf = self.buf.lookup_transform('mission_map', 'base_link', Time.from_msg(st))
                tr = tf.transform
                self.w.writerow([f'{t:.3f}', x, y, a, tr.translation.x, tr.translation.y, yaw(tr.rotation)])
            except Exception:
                self.w.writerow([f'{t:.3f}', x, y, a, '', '', ''])
        self.f.flush()


rclpy.init()
rclpy.spin(Eval(sys.argv[1]))
