"""실시간 재생 중 /map을 시뮬레이션 시간 PERIOD초마다 pgm·yaml로 저장한다 (ROS 2 Lyrical rclpy).

map_saver와 같은 trinary 규칙(점유 ≥ 0.65 → 0, 빈칸 ≤ 0.196 → 254, 나머지 205)으로 저장해
evaluate_maps.py의 load_map으로 그대로 읽을 수 있게 한다.

사용: python3 snapshot_map.py <저장 폴더> [주기 s, 기본 30]
"""
import os
import sys

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from PIL import Image
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

OCC_T, FREE_T = 0.65, 0.196


class Snapshot(Node):
    def __init__(self, out, period):
        super().__init__('map_snapshot', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.out, self.period, self.next_t, self.count = out, period, None, 0
        os.makedirs(out, exist_ok=True)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, qos)

    def on_map(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if self.next_t is None:
            self.next_t = t + self.period
        if t < self.next_t:
            return
        self.next_t += self.period * max(1, int((t - self.next_t) // self.period) + 1)
        g = np.array(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
        img = np.full(g.shape, 205, np.uint8)
        img[(g >= 0) & (g / 100.0 >= OCC_T)] = 0
        img[(g >= 0) & (g / 100.0 <= FREE_T)] = 254
        stem = os.path.join(self.out, f'map_t{t:08.1f}')
        Image.fromarray(np.flipud(img)).save(stem + '.pgm')
        o = m.info.origin.position
        with open(stem + '.yaml', 'w') as f:
            f.write(f'image: {os.path.basename(stem)}.pgm\nmode: trinary\nresolution: {m.info.resolution:.3f}\n'
                    f'origin: [{o.x:.3f}, {o.y:.3f}, 0]\nnegate: 0\noccupied_thresh: {OCC_T}\nfree_thresh: {FREE_T}\n')
        self.count += 1


def main():
    rclpy.init()
    node = Snapshot(sys.argv[1], float(sys.argv[2]) if len(sys.argv) > 2 else 30.0)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f'snapshots: {node.count}')
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
