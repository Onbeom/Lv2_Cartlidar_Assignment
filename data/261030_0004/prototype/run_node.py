"""개인 Nav2 연결 노드 실행기. 장비/팀 실행기는 별도로 합의한다."""
import sys
import rclpy
from command_gate import CommandGate
from navigation_coordinator import NavigationCoordinator
from observation_monitor import ObservationMonitor

nodes={'gate':CommandGate,'coordinator':NavigationCoordinator,'monitor':ObservationMonitor}
kind=sys.argv[1]
if kind not in nodes:raise ValueError('노드 종류: gate/coordinator/monitor')
rclpy.init(args=sys.argv[2:]);node=nodes[kind]()
try:rclpy.spin(node)
finally:node.destroy_node();rclpy.shutdown()
