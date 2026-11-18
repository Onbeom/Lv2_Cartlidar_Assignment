"""move_group의 /check_state_validity로 팔 자세의 충돌 여부를 확인한다.

사용 (collision_check.launch.py 실행 중):
  python3 check_arm_collision.py                      # 관절 0 자세
  python3 check_arm_collision.py 0 1.2 -1.0 0.5 0      # yaw shoulder elbow pitch roll [rad]
출력: 유효 여부와 충돌한 링크 쌍(접촉 깊이 m).
"""
import sys

import rclpy
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetStateValidity
from sensor_msgs.msg import JointState

ARM_JOINTS = ['RMX52_Base_Yaw', 'RMX52_Shoulder', 'RMX52_Elbow', 'RMX52_Wrist_Pitch', 'RMX52_Wrist_Roll_Reversed']


def main():
    values = [float(v) for v in sys.argv[1:]] or [0.0] * len(ARM_JOINTS)
    if len(values) != len(ARM_JOINTS):
        sys.exit(f'관절 값 {len(ARM_JOINTS)}개가 필요하다: {ARM_JOINTS}')
    rclpy.init()
    node = rclpy.create_node('check_arm_collision')
    client = node.create_client(GetStateValidity, '/check_state_validity')
    if not client.wait_for_service(timeout_sec=10.0):
        sys.exit('/check_state_validity 서비스가 없다. collision_check.launch.py를 먼저 실행한다')
    request = GetStateValidity.Request()
    request.group_name = 'arm'
    request.robot_state = RobotState(joint_state=JointState(name=ARM_JOINTS, position=values), is_diff=True)
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
    result = future.result()
    pairs = sorted({(c.contact_body_1, c.contact_body_2, round(c.depth, 4)) for c in result.contacts})
    print(f'자세 {dict(zip(ARM_JOINTS, values))}')
    print(f'유효(충돌 없음): {result.valid}, 충돌 쌍 {len(pairs)}개')
    for a, b, depth in pairs:
        print(f'  {a} <-> {b}  깊이 {depth} m')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
