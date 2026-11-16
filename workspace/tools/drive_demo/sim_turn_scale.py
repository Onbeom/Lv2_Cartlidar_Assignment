#!/usr/bin/env python3
"""시뮬 전용 (11-16): 실물처럼 덜 도는 로봇 흉내 — 규칙 v2 필터 출력 cmd_vel_turn → 회전만 줄여 → cmd_vel (loopback 시뮬).
  실물 11-15 (벨트 v7 1 : 2.0) 기록: 움직이며 꺾기 = 명령 회전의 약 79 %, 제자리 회전 = 66 ~ 80 %.
  loopback 은 명령대로 정확히 움직이므로, 회전만 이 비율로 줄여 넣으면 오도메트리 · TF 도 줄어든 회전을 따름 (실물 EKF 가 IMU 로 실제 회전을 보는 것과 같음).
  근사일 뿐 (미끄럼 · 지연 · 잡음 없음). 실물 검증이 아님.
  ./run_demo.sh 가 SIM_TURN=0.79,0.70 일 때만 띄움."""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped


class TurnScale(Node):
    def __init__(self):
        super().__init__("sim_turn_scale")
        self.k_curve = self.declare_parameter("k_curve", 0.79).value
        self.k_spin = self.declare_parameter("k_spin", 0.70).value
        self.spin_v = self.declare_parameter("spin_v", 0.03).value   # |vx| 이 이보다 작으면 제자리 회전으로 봄
        self.pub = self.create_publisher(TwistStamped, "cmd_vel", 10)
        self.create_subscription(TwistStamped, "cmd_vel_turn", self._in, 20)
        self.get_logger().info(f"실물형 회전: 꺾기 × {self.k_curve} · 제자리 × {self.k_spin} (cmd_vel_turn → cmd_vel)")

    def _in(self, m):
        k = self.k_spin if abs(m.twist.linear.x) < self.spin_v else self.k_curve
        m.twist.angular.z *= k
        self.pub.publish(m)


def main():
    rclpy.init()
    n = TurnScale()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, Exception):
        pass


if __name__ == "__main__":
    main()
