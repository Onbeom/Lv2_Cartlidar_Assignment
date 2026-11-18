"""시험용 TF 탐침: rate Hz 로 map→base_link 를 ① TimePointZero(가장 최근 공통 시각) ② 지금 시각으로 조회해 CSV 로 남긴다.
열: now_s, zero_ok, zero_age_s(지금 − 돌려받은 stamp), now_ok. 사용: python3 tf_probe.py --ros-args -p out:=<csv> -p use_sim_time:=true"""
import rclpy
import tf2_ros
from rclpy.node import Node
from rclpy.time import Time


class Probe(Node):
    def __init__(self):
        super().__init__('tf_probe')
        self.declare_parameter('out', 'tf_probe.csv')
        self.declare_parameter('rate', 10.0)
        self.declare_parameter('target', 'map')
        self.declare_parameter('source', 'base_link')
        self.f = open(self.get_parameter('out').value, 'w')
        self.f.write('now_s,zero_ok,zero_age_s,now_ok\n')
        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf, self, spin_thread=True)
        self.tg, self.src = self.get_parameter('target').value, self.get_parameter('source').value
        self.create_timer(1.0 / self.get_parameter('rate').value, self.tick)

    def tick(self):
        now = self.get_clock().now()
        if now.nanoseconds == 0:
            return
        try:
            t = self.buf.lookup_transform(self.tg, self.src, Time())
            zok, age = 1, (now - Time.from_msg(t.header.stamp)).nanoseconds * 1e-9
        except tf2_ros.TransformException:
            zok, age = 0, float('nan')
        nok = int(self.buf.can_transform(self.tg, self.src, now))
        self.f.write(f'{now.nanoseconds * 1e-9:.4f},{zok},{age:.4f},{nok}\n')
        self.f.flush()


def main():
    rclpy.init()
    rclpy.spin(Probe())


if __name__ == '__main__':
    main()
