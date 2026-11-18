"""진단: /global_costmap/costmap · /map · /map_objects 를 한 번씩 받아 npz 로 저장."""
import sys
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

rclpy.init()
n = rclpy.create_node('dump_costmap')
got = {}
tl = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
for t in ('/global_costmap/costmap', '/map', '/map_objects', '/local_costmap/costmap'):
    for q in (tl, 10):
        n.create_subscription(OccupancyGrid, t, lambda m, t=t: got.setdefault(t, m), q)
w0 = time.monotonic()
while len(got) < 4 and time.monotonic() - w0 < 30:
    rclpy.spin_once(n, timeout_sec=0.2)
out = {}
for t, m in got.items():
    k = t.strip('/').replace('/', '_')
    out[k] = np.array(m.data, np.int16).reshape(m.info.height, m.info.width)
    out[k + '_info'] = np.array([m.info.resolution, m.info.origin.position.x, m.info.origin.position.y])
np.savez(sys.argv[1], **out)
print({t: (m.info.width, m.info.height, m.header.frame_id) for t, m in got.items()})
