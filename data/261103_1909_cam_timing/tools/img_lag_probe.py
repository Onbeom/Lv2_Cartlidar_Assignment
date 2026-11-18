"""영상 구독 탐침(모의 확인용, 11-03): 영상을 받을 때마다 인지 노드가 하는 TF 조회를 흉내 내 CSV 로 남긴다.
열: rx_s(받은 때 지금 시각), img_s(영상 stamp),
    ekf_s(그때 버퍼에 있는 odom → base_link 최신 stamp), lag_s(= img_s − ekf_s, 인지 조건 1),
    map_ok · map_s(map → 영상 frame TimePointZero 조회 성공 · 돌려받은 stamp), map_age_now_s(rx_s − map_s), map_age_img_s(img_s − map_s),
    near_ok(odom → base_footprint 를 영상 stamp 에 조회 성공 = 인지 lookup_near 가 그대로 쓰는 경우).
사용: python3 img_lag_probe.py --ros-args -p out:=<csv> -p use_sim_time:=true [-p image:=<토픽>]"""
import rclpy
import tf2_ros
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image


def sec(t):
    return t.nanoseconds * 1e-9


class Probe(Node):
    def __init__(self):
        super().__init__('img_lag_probe')
        self.declare_parameter('out', 'img_lag.csv')
        self.declare_parameter('image', '/top_camera_front/infra1/image_rect_raw')
        self.declare_parameter('map_frame', 'map')
        self.f = open(self.get_parameter('out').value, 'w')
        self.f.write('rx_s,img_s,ekf_s,lag_s,map_ok,map_s,map_age_now_s,map_age_img_s,near_ok\n')
        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf, self, spin_thread=True)
        self.map = self.get_parameter('map_frame').value
        self.create_subscription(Image, self.get_parameter('image').value, self.on_image, qos_profile_sensor_data)

    def on_image(self, msg):
        now = self.get_clock().now()
        img = Time.from_msg(msg.header.stamp)
        try:
            ekf = sec(Time.from_msg(self.buf.lookup_transform('odom', 'base_link', Time()).header.stamp))
        except tf2_ros.TransformException:
            ekf = float('nan')
        try:
            ms, mok = sec(Time.from_msg(self.buf.lookup_transform(self.map, msg.header.frame_id, Time()).header.stamp)), 1
        except tf2_ros.TransformException:
            ms, mok = float('nan'), 0
        near = int(self.buf.can_transform('odom', 'base_footprint', img))
        self.f.write(f'{sec(now):.4f},{sec(img):.4f},{ekf:.4f},{sec(img) - ekf:.4f},{mok},{ms:.4f},{sec(now) - ms:.4f},{sec(img) - ms:.4f},{near}\n')
        self.f.flush()


def main():
    rclpy.init()
    rclpy.spin(Probe())


if __name__ == '__main__':
    main()
