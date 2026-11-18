"""측정 전용: 사진 한 장을 /arm_camera/image_raw 로 30 Hz 발행(시각은 지금). camera_info 는 180° 돌린 보정값."""
import sys, time, cv2, yaml, rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
img = cv2.imread(sys.argv[1]); c = yaml.safe_load(open(sys.argv[2])); W, H = c['image_width'], c['image_height']
rclpy.init(); n = rclpy.create_node('arm_replay', namespace='/arm_camera')
q = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)
pi, pc = n.create_publisher(Image, 'image_raw', q), n.create_publisher(CameraInfo, 'camera_info', q)
F = lambda n: [float(v) for v in c[n]['data']]
k, p, d = F('camera_matrix'), F('projection_matrix'), F('distortion_coefficients')
k[2], k[5] = W-1-k[2], H-1-k[5]; p[2], p[6] = W-1-p[2], H-1-p[6]; d[2], d[3] = -d[2], -d[3]
info = CameraInfo(width=W, height=H, distortion_model=c['distortion_model'], d=d, k=k, r=F('rectification_matrix'), p=p)
data = img.tobytes()
while rclpy.ok():
    m = Image(height=H, width=W, encoding='bgr8', step=W*3, data=data)
    m.header.stamp = n.get_clock().now().to_msg(); m.header.frame_id = 'arm_camera_optical_frame'; info.header = m.header
    pi.publish(m); pc.publish(info); time.sleep(0.0327)
