"""Nav2 확인용 2D 운동학 모의기(Isaac Sim 아님). /cmd_vel_nav(TwistStamped) 를 받아 참값 자세를 적분하고 다음을 낸다:
  /clock(모의 시각), /odom(바퀴 odom 흉내: 직진 · 회전 배수 오차 + 잡음, TF 는 안 냄 → EKF 가 냄), /scan_front · /scan_rear(400 칸 10 Hz,
  laser_front/laser_rear frame, 벽 · 신호등 · 퍽에 광선 계산, 차체 가림 칸은 Isaac 녹화에서 배운 칸을 몸체 거리로), /sim/truth(PoseStamped, map=발제 좌표).
참값 기록(CSV): 시각 · 참값 자세 · 명령 · footprint 여유(벽 · 모든 물체). 라이다 위치는 robot_state_publisher 의 TF(base_link→laser_*)를 읽어 씀."""
import csv
import math
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
import tf2_ros
import yaml
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan

sys.path.insert(0, str(Path(__file__).resolve().parent))
import geom  # noqa: E402


def quat_yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def yaw_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


class Sim2D(Node):
    def __init__(self):
        super().__init__('sim2d', parameter_overrides=[])
        p = self.declare_parameter
        p('start', [0.734, 0.95, math.pi])           # 참값 출발 자세(발제 좌표)
        p('rate', 100.0)                             # 적분 Hz
        p('rtf', 1.0)                                # 실시간 배수
        p('tau', [0.10, 0.10])                       # 명령 → 실제 속도 1차 지연 [s] (vx, wz)
        p('cmd_timeout', 0.5)
        p('odom_scale', [1.003, 1.02])               # 바퀴 odom 이 세는 배수(직진, 회전) — 회전 배수 보정 뒤 남은 오차 가정
        p('odom_noise', [0.005, 0.01])               # 속도 잡음 표준편차(m/s, rad/s)
        p('scan_noise', 0.005)                       # 거리 잡음 표준편차 m
        p('scan_range', [0.05, 12.0])                # Isaac 녹화 스캔과 같음
        p('body_mask', '/home/pa4/git/lv2_next_module/261031_0335_slam_launch/test/results/bright/body_mask.yaml')
        p('body_range', 0.12)                        # 가림 칸에 넣을 거리(몸체 상자 안)
        p('log', 'truth.csv')
        p('seed', 20261031)
        g = lambda n: self.get_parameter(n).value
        self.x, self.y, self.yaw = [float(v) for v in g('start')]
        self.dt = 1.0 / g('rate')
        self.rtf = g('rtf')
        self.tau = [float(v) for v in g('tau')]
        self.cmd_timeout = g('cmd_timeout')
        self.oscale = [float(v) for v in g('odom_scale')]
        self.onoise = [float(v) for v in g('odom_noise')]
        self.snoise = g('scan_noise')
        self.rmin, self.rmax = [float(v) for v in g('scan_range')]
        self.body_range = g('body_range')
        self.rng = np.random.default_rng(g('seed'))
        mask = yaml.safe_load(open(g('body_mask')))
        self.mask = {'/scan_front': np.array(mask['/scan_front'], int), '/scan_rear': np.array(mask['/scan_rear'], int)}
        self.world = geom.load_world()
        self.t = 0.0
        self.v = self.w = 0.0                        # 실제 속도
        self.cmd = (0.0, 0.0)
        self.cmd_t = -1e9
        self.ox = self.oy = self.oyaw = 0.0          # odom 적분(odom frame, 출발 = 원점)
        self.min_clear = 1e9
        self.n_collide = 0
        self.clock_pub = self.create_publisher(Clock, '/clock', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.truth_pub = self.create_publisher(PoseStamped, '/sim/truth', 10)
        sensor = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.scan_pub = {k: self.create_publisher(LaserScan, k, sensor) for k in ('/scan_front', '/scan_rear')}
        self.create_subscription(TwistStamped, '/cmd_vel_nav', self.on_cmd, 10)
        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf, self)
        self.extr = {}
        self.n = 0
        self.logf = open(g('log'), 'w', newline='')
        self.log = csv.writer(self.logf)
        self.log.writerow(['t', 'x', 'y', 'yaw', 'v', 'w', 'cmd_v', 'cmd_w', 'clear_body'])

    def on_cmd(self, msg):
        self.cmd = (msg.twist.linear.x, msg.twist.angular.z)
        self.cmd_t = self.t

    def extrinsics(self, frame):
        if frame not in self.extr:
            tf = self.buf.lookup_transform('base_link', frame, Time())   # 정적 TF(시각 무관)
            q = tf.transform.rotation
            # 라이다 평면 각 a 의 방향 → base_link 방향: 회전 행렬 첫 두 열의 xy
            x, y, z, w = q.x, q.y, q.z, q.w
            r = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w)], [2 * (x * y + z * w), 1 - 2 * (x * x + z * z)]])
            self.extr[frame] = (r, tf.transform.translation.x, tf.transform.translation.y)
        return self.extr[frame]

    def stamp(self):
        s = int(self.t)
        return rclpy.time.Time(seconds=s, nanoseconds=int(round((self.t - s) * 1e9))).to_msg()

    def step(self):
        vc, wc = self.cmd if self.t - self.cmd_t <= self.cmd_timeout else (0.0, 0.0)
        self.v += (vc - self.v) * min(1.0, self.dt / self.tau[0])
        self.w += (wc - self.w) * min(1.0, self.dt / self.tau[1])
        self.x += self.v * math.cos(self.yaw + self.w * self.dt / 2) * self.dt
        self.y += self.v * math.sin(self.yaw + self.w * self.dt / 2) * self.dt
        self.yaw = math.atan2(math.sin(self.yaw + self.w * self.dt), math.cos(self.yaw + self.w * self.dt))
        self.t += self.dt
        self.n += 1
        clk = Clock()
        clk.clock = self.stamp()
        self.clock_pub.publish(clk)
        if self.n % round(1 / (30 * self.dt)) == 0:       # 약 30 Hz: odom · 참값 · 기록
            self.publish_odom(vc, wc)
        if self.n % round(1 / (10 * self.dt)) == 0:       # 10 Hz 앞 라이다
            self.publish_scan('/scan_front', 'laser_front')
        if self.n % round(1 / (10 * self.dt)) == round(1 / (20 * self.dt)):   # 뒤 라이다는 반 주기 어긋남
            self.publish_scan('/scan_rear', 'laser_rear')

    def publish_odom(self, vc, wc):
        dt = round(1 / (30 * self.dt)) * self.dt
        vo = self.v * self.oscale[0] + self.rng.normal(0, self.onoise[0])
        wo = self.w * self.oscale[1] + self.rng.normal(0, self.onoise[1])
        self.ox += vo * math.cos(self.oyaw + wo * dt / 2) * dt
        self.oy += vo * math.sin(self.oyaw + wo * dt / 2) * dt
        self.oyaw = math.atan2(math.sin(self.oyaw + wo * dt), math.cos(self.oyaw + wo * dt))
        m = Odometry()
        m.header.stamp = self.stamp()
        m.header.frame_id = 'odom'
        m.child_frame_id = 'base_link'
        m.pose.pose.position.x, m.pose.pose.position.y = self.ox, self.oy
        q = yaw_quat(self.oyaw)
        m.pose.pose.orientation.x, m.pose.pose.orientation.y, m.pose.pose.orientation.z, m.pose.pose.orientation.w = q
        m.twist.twist.linear.x, m.twist.twist.angular.z = vo, wo
        cov = [0.0] * 36
        cov[0] = cov[7] = 1e-3
        cov[35] = 1e-2
        m.pose.covariance = cov
        tc = [0.0] * 36
        tc[0], tc[7], tc[14], tc[21], tc[28], tc[35] = 1e-4, 1e-4, 1e6, 1e6, 1e6, 4e-4
        m.twist.covariance = tc
        self.odom_pub.publish(m)
        t = PoseStamped()
        t.header = m.header
        t.header.frame_id = 'map'
        t.pose.position.x, t.pose.position.y = self.x, self.y
        q = yaw_quat(self.yaw)
        t.pose.orientation.x, t.pose.orientation.y, t.pose.orientation.z, t.pose.orientation.w = q
        self.truth_pub.publish(t)
        c = geom.clearance(self.x, self.y, self.yaw, self.world['body'])
        self.min_clear = min(self.min_clear, c)
        self.n_collide += c < 0
        self.log.writerow([f'{self.t:.3f}', f'{self.x:.4f}', f'{self.y:.4f}', f'{self.yaw:.4f}', f'{self.v:.4f}', f'{self.w:.4f}',
                           f'{vc:.4f}', f'{wc:.4f}', f'{c:.4f}'])

    def publish_scan(self, topic, frame):
        try:
            r, lx, ly = self.extrinsics(frame)
        except tf2_ros.TransformException:
            self.get_logger().warning(f'{frame} TF 아직 없음', throttle_duration_sec=5.0)
            return
        n = 400
        inc = 2 * math.pi / n
        a = -math.pi + inc * np.arange(n)
        d_base = (r @ np.stack([np.cos(a), np.sin(a)])).T          # base_link 방향
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        d_map = np.stack([c * d_base[:, 0] - s * d_base[:, 1], s * d_base[:, 0] + c * d_base[:, 1]], 1)
        px, py = self.x + c * lx - s * ly, self.y + s * lx + c * ly
        rng = geom.raycast(px, py, d_map, self.world['lidar'], self.rmax)
        rng = rng + self.rng.normal(0, self.snoise, n) * np.isfinite(rng)
        rng[rng < self.rmin] = np.inf
        rng[self.mask[topic]] = self.body_range                      # 차체 가림(몸체 상자 안 거리)
        m = LaserScan()
        m.header.stamp = self.stamp()
        m.header.frame_id = frame
        m.angle_min, m.angle_max, m.angle_increment = -math.pi, -math.pi + inc * (n - 1), inc
        m.time_increment, m.scan_time = 0.0, 0.1
        m.range_min, m.range_max = self.rmin, self.rmax
        m.ranges = [float(v) for v in rng]
        self.scan_pub[topic].publish(m)

    def run(self):
        wall0 = time.monotonic()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            self.step()
            lag = wall0 + self.t / self.rtf - time.monotonic()
            if lag > 0:
                time.sleep(lag)
            if self.n % 3000 == 0:
                self.logf.flush()
                self.get_logger().info(f't={self.t:.0f} 참값 ({self.x:.3f}, {self.y:.3f}, {math.degrees(self.yaw):.1f}°) 최소 여유 {self.min_clear:.3f} m 충돌 기록 {self.n_collide}')


def main():
    rclpy.init()
    node = Sim2D()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.logf.close()
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
