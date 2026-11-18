"""4륜 스키드 회전 중심(ICR) 보정 odom: 바퀴 odom(/odom, vx) + IMU yaw rate(/imu/data) → /odom_icr(vx 그대로, vy = −x_icr · w_imu, wz = w_imu).
차동 구동 odom 은 base_link 를 중심으로 돈다고 보지만, 스키드 차체는 회전 중심이 몸 x 축 위 x_icr 에 있어 base_link 가 옆으로 w · (−x_icr) 만큼 움직인다.
EKF 는 이 토픽의 vx · vy 를 받고(ekf_sim_imu_icr.yaml), 회전은 IMU 에서 받는다. 자세(pose) 칸은 쓰지 않으므로 적분하지 않는다.
x_icr: Isaac 실시간(261101_0016_isaac_live/results/live_nav_v12 제자리 회전 3129 표본) 참값으로 잰 −0.0786 m(뒤 바퀴 축 −0.08 근처, 모의 값).
실물은 제자리 회전 시험으로 따로 재야 함. 사용: python3 icr_odom.py --ros-args -p use_sim_time:=true [-p x_icr:=-0.0786]"""
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


class IcrOdom(Node):
    def __init__(self):
        super().__init__('icr_odom')
        self.declare_parameter('odom_in', '/odom')
        self.declare_parameter('imu_in', '/imu/data')
        self.declare_parameter('odom_out', '/odom_icr')
        self.declare_parameter('x_icr', -0.0786)
        self.declare_parameter('imu_max_age', 0.1)
        g = lambda n: self.get_parameter(n).value
        self.x_icr, self.max_age = float(g('x_icr')), float(g('imu_max_age'))
        self.w, self.w_t = None, None
        self.pub = self.create_publisher(Odometry, g('odom_out'), 10)
        self.create_subscription(Imu, g('imu_in'), self.on_imu, qos_profile_sensor_data)
        self.create_subscription(Odometry, g('odom_in'), self.on_odom, 10)
        self.n = {'out': 0, 'no_imu': 0}
        self.create_timer(30.0, lambda: self.get_logger().info(f'icr_odom: 냄 {self.n["out"]}, IMU 없어 vy 0 {self.n["no_imu"]}'))

    def on_imu(self, m):
        self.w, self.w_t = m.angular_velocity.z, m.header.stamp.sec + m.header.stamp.nanosec * 1e-9

    def on_odom(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if self.w is not None and abs(t - self.w_t) <= self.max_age:
            m.twist.twist.linear.y = -self.x_icr * self.w
            m.twist.twist.angular.z = self.w
        else:
            m.twist.twist.linear.y = 0.0
            self.n['no_imu'] += 1
        self.n['out'] += 1
        self.pub.publish(m)


def main():
    rclpy.init()
    rclpy.spin(IcrOdom())


if __name__ == '__main__':
    main()
