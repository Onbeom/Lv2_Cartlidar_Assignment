"""측정 전용: 받은 시각 - header.stamp 로 영상 토픽의 나이를 잰다. 인수: 초 토픽..."""
import sys, time, numpy as np, rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
secs = float(sys.argv[1]); topics = sys.argv[2:]
rclpy.init(); n = rclpy.create_node('img_age'); s = {t: [] for t in topics}
def mk(k):
    def cb(m):
        s[k].append(time.time() - (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9))
    return cb
for t in topics: n.create_subscription(Image, t, mk(t), qos_profile_sensor_data)
t0 = time.monotonic()
while time.monotonic() - t0 < secs: rclpy.spin_once(n, timeout_sec=0.05)
for t in topics:
    a = np.array(s[t]) * 1000
    print(f'  {t}: {len(a)} 장 · 나이 p50 {np.median(a):.0f} p99 {np.percentile(a,99):.0f} ms' if len(a) > 1 else f'  {t}: {len(a)} 장')
