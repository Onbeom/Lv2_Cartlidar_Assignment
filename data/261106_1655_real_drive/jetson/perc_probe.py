import sys, time, numpy as np, rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import Range
secs = float(sys.argv[1]); rtype = sys.argv[2]
rclpy.init(); n = rclpy.create_node('perc_probe')
R = get_message(rtype); s = {'r': ([], [], []), 'a': ([], [], [])}
def mk(k):
    def cb(m):
        t = time.time(); st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        s[k][0].append(time.monotonic()); s[k][1].append(t - st); s[k][2].append(getattr(m, 'mode', None))
    return cb
q = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
n.create_subscription(R, '/perception/result', mk('r'), q); n.create_subscription(Range, '/arm_range', mk('a'), q)
t0 = time.monotonic()
while time.monotonic() - t0 < secs: rclpy.spin_once(n, timeout_sec=0.05)
for k, name in (('r', 'result'), ('a', 'arm_range')):
    t, a, md = s[k]
    if len(t) < 2: print(f'  {name}: {len(t)} 개'); continue
    d = np.diff(t); a = np.array(a) * 1000
    print(f'  {name}: {len(t)} 개 · {1/d.mean():.1f} Hz · 간격 최대 {d.max()*1000:.0f} ms · 나이 p50 {np.median(a):.0f} p99 {np.percentile(a,99):.0f} ms · >300ms {(a>300).mean()*100:.1f} % · mode {md[-1]}')
