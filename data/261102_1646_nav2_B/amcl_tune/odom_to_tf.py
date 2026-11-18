"""재생용: /odometry/filtered(EKF 출력)를 odom → base_link TF 로 다시 낸다(녹화의 /tf 는 위치 추정 TF 가 섞여 있어 틀지 않음)."""
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class OdomToTf(Node):
    def __init__(self):
        super().__init__('odom_to_tf')
        self.br = TransformBroadcaster(self)
        self.create_subscription(Odometry, '/odometry/filtered', self.cb, 50)

    def cb(self, m):
        t = TransformStamped()
        t.header = m.header
        t.child_frame_id = m.child_frame_id or 'base_link'
        p = m.pose.pose
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = p.position.x, p.position.y, p.position.z
        t.transform.rotation = p.orientation
        self.br.sendTransform(t)


rclpy.init()
rclpy.spin(OdomToTf())
