"""제자리 회전 명령 바닥값(4륜 스키드): controller · behavior 출력(/cmd_vel_ctrl) → 이 노드 → /cmd_vel_turn_floor → velocity_smoother.
전진 속도가 v_eps 보다 작을 때만 |w| 를 고친다: |w| < w_zero 면 0(차체는 안 돌고 바퀴만 돌아 odom 회전이 부풀던 명령),
w_zero ≤ |w| < w_min 이면 부호 그대로 w_min(차체가 도는 가장 작은 회전 속도), 그 밖은 그대로. 전진 중 회전은 고치지 않는다.
근거: Isaac 실시간 Nav2(261101_0016_isaac_live/results/live_nav_v12) 제자리 회전 명령 0.02~0.10 rad/s 에서 참 회전 0 %, 0.10~0.20 은 3 %,
0.20~0.35 는 40 %, 0.35 이상 65~75 %. w_min 0.35 = 녹화 장면 MIN_TURN_W · behavior_server min_rotational_vel 과 같은 값(모의 값, 실물은 실측).
out_frame(설계 20): Nav2 가 회전 중심(rotation_center)을 제어점으로 쓰면 명령 header 가 rotation_center 로 나옴. 스키드 차체는 옆 속도를 못 내므로
(vx, wz) 바퀴 명령은 어느 점 기준이든 같고, 팀 twist_mux · 판정 G5 가 받는 base_link 로 이름만 바꿈.
yaw_loop(설계 23, 기본 꺼짐): IMU yaw rate 되먹임 PI. Isaac 4륜 스키드는 실제 회전 속도가 명령의 53~75 %(제자리, 설계 19~21 참값)라
MPPI 예측과 실제가 어긋남 → w_out = w + kp·e + ki·∫e (e = w − w_imu), |w_out| ≤ w_max, 적분은 ±i_max 로 묶고 명령 w 가 0 이거나 부호가 바뀌면 0 으로.
turn_ff(설계 24, 기본 꺼짐): 제자리 회전만 정적 앞먹임 — 바닥값을 거친 원하는 회전 |w| 를 측정 곡선(명령 ff_cmd → 실제 ff_act, 설계 21 · 22 참값
자유 회전 중앙값 0.35 → 0.186 · 0.55 → 0.358 · 0.8 → 0.60)의 역함수로 명령으로 바꿈, 곡선 밖은 끝값(상한 w_max). 설계 23 PI 가 MPPI 와 엇갈려 흔들린 것 대신.
이때 w_min 은 '원하는 실제 회전'의 바닥(예 0.19 → 명령 0.35).
move_ff(설계 32, 기본 꺼짐): 전진 중(|v| ≥ v_eps) 회전 정적 앞먹임 — 원하는 실제 회전 |w| 를 전진 중 측정 곡선(명령 move_ff_cmd → 실제 move_ff_act,
261101_0016_isaac_live/results/TOOLCHECK_turn_moving)의 역함수로 명령으로 바꿈. 전진 중에는 명령 |w| 약 0.17 까지 차체가 거의 안 돌아(설계 31) RPP 방향 보정이 안 들었음.
|w| < move_w_zero 는 그대로(경로 추종 잡음을 키우지 않게), 곡선 밖은 끝값(상한 w_max).
바퀴 쪽 각속도 되먹임은 원래 구동(제어 파트) 계층 일 — 모의 Nav2 확인용으로 여기 둠.
사용: python3 turn_floor.py --ros-args -p use_sim_time:=true [-p w_min:=0.35]"""
import math

import numpy as np

import rclpy
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Imu
from rclpy.node import Node


class TurnFloor(Node):
    def __init__(self):
        super().__init__('turn_floor')
        self.declare_parameter('in_topic', '/cmd_vel_ctrl')
        self.declare_parameter('out_topic', '/cmd_vel_turn_floor')
        self.declare_parameter('v_eps', 0.02)
        self.declare_parameter('w_zero', 0.05)
        self.declare_parameter('w_min', 0.35)
        self.declare_parameter('yaw_loop', False)
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('kp', 0.5)
        self.declare_parameter('ki', 3.0)
        self.declare_parameter('i_max', 0.4)
        self.declare_parameter('w_max', 0.8)
        self.declare_parameter('turn_ff', False)
        self.declare_parameter('ff_cmd', [0.35, 0.55, 0.80])
        self.declare_parameter('ff_act', [0.186, 0.358, 0.60])
        self.declare_parameter('move_ff', False)
        self.declare_parameter('move_w_zero', 0.01)
        self.declare_parameter('move_ff_cmd', [0.15, 0.80])
        self.declare_parameter('move_ff_act', [0.0, 0.60])
        self.declare_parameter('out_frame', 'none')   # 'none' 이 아니면 header.frame_id 를 이 값으로(설계 20: rotation_center → base_link)
        g = lambda n: self.get_parameter(n).value
        self.v_eps, self.w_zero, self.w_min = float(g('v_eps')), float(g('w_zero')), float(g('w_min'))
        self.out_frame = g('out_frame')
        self.pub = self.create_publisher(TwistStamped, g('out_topic'), 10)
        self.create_subscription(TwistStamped, g('in_topic'), self.on_cmd, 10)
        self.yaw_loop, self.kp, self.ki, self.i_max, self.w_max = bool(g('yaw_loop')), float(g('kp')), float(g('ki')), float(g('i_max')), float(g('w_max'))
        self.turn_ff, self.ff_cmd, self.ff_act = bool(g('turn_ff')), [float(x) for x in g('ff_cmd')], [float(x) for x in g('ff_act')]
        self.move_ff, self.move_w_zero = bool(g('move_ff')), float(g('move_w_zero'))
        self.move_ff_cmd, self.move_ff_act = [float(x) for x in g('move_ff_cmd')], [float(x) for x in g('move_ff_act')]
        self.w_imu, self.integ, self.t_last, self.w_prev = 0.0, 0.0, None, 0.0
        if self.yaw_loop:
            self.create_subscription(Imu, g('imu_topic'), lambda m: setattr(self, 'w_imu', m.angular_velocity.z), 50)
        self.n = {'in': 0, 'zeroed': 0, 'raised': 0, 'ff': 0, 'move_ff': 0}
        self.create_timer(30.0, self.report)

    def on_cmd(self, m):
        self.n['in'] += 1
        w = m.twist.angular.z
        if abs(m.twist.linear.x) < self.v_eps:
            if abs(w) < self.w_zero:
                if w != 0.0:
                    self.n['zeroed'] += 1
                m.twist.angular.z = 0.0
            elif abs(w) < self.w_min:
                m.twist.angular.z = math.copysign(self.w_min, w)
                self.n['raised'] += 1
        if self.turn_ff and abs(m.twist.linear.x) < self.v_eps and m.twist.angular.z != 0.0:
            a = abs(m.twist.angular.z)
            c = min(self.w_max, float(np.interp(a, self.ff_act, self.ff_cmd)))
            m.twist.angular.z = math.copysign(c, m.twist.angular.z)
            self.n['ff'] += 1
        if self.move_ff and abs(m.twist.linear.x) >= self.v_eps and abs(m.twist.angular.z) >= self.move_w_zero:
            a = abs(m.twist.angular.z)
            c = min(self.w_max, float(np.interp(a, self.move_ff_act, self.move_ff_cmd)))
            m.twist.angular.z = math.copysign(c, m.twist.angular.z)
            self.n['move_ff'] += 1
        if self.yaw_loop:
            w = m.twist.angular.z
            now = self.get_clock().now().nanoseconds * 1e-9
            dt = 0.0 if self.t_last is None else min(max(now - self.t_last, 0.0), 0.2)
            self.t_last = now
            if w == 0.0 or w * self.w_prev < 0.0:
                self.integ = 0.0
            else:
                e = w - self.w_imu
                self.integ = max(-self.i_max, min(self.i_max, self.integ + e * dt))
                m.twist.angular.z = max(-self.w_max, min(self.w_max, w + self.kp * e + self.ki * self.integ))
            self.w_prev = w
        if self.out_frame and self.out_frame != 'none':
            m.header.frame_id = self.out_frame
        self.pub.publish(m)

    def report(self):
        self.get_logger().info(f'turn_floor: 받음 {self.n["in"]}, 0 으로 {self.n["zeroed"]}, w_min 으로 올림 {self.n["raised"]}, 앞먹임 {self.n["ff"]}, 전진 앞먹임 {self.n["move_ff"]}')


def main():
    rclpy.init()
    rclpy.spin(TurnFloor())


if __name__ == '__main__':
    main()
