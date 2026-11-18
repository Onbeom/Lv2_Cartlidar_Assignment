#!/usr/bin/env python3
"""라이다 점 보기 (Jetson 모니터, 11-15). URDF 를 고치기 전에 '지금 TF' 와 '요 180° 바꾼 안' 을 나란히 그려 손으로 확인.
  python3 lidar_view.py [보이는 범위 m=1.0] [--tf]
  --tf (11-16): 그림 하나, 실행 중인 TF(robot_state_publisher 의 base_link → laser_*)로 그림 = 지금 시스템이 보는 그대로
- /scan_front(빨강) · /scan_rear(파랑) 점을 base_link 위에서 내려다본 그림으로. 점은 3 s 동안 흐려지다 사라짐.
- 왼쪽 그림 = 지금 robot.urdf.xacro (laser_front rpy π 0 0, laser_rear rpy π 0 π)
  오른쪽 그림 = 요 180° 바꾼 안 (laser_front π 0 π, laser_rear π 0 0)
- 위가 로봇 앞(+x), 왼쪽이 로봇 왼쪽(+y). 회색 = nav2 footprint. 상자를 댄 자리에 점이 생기는 쪽 그림이 맞는 TF.
"""
import math
import sys
import threading
import time

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt   # noqa: E402
# 그림 글자는 영어 (Jetson matplotlib 이 한글 글꼴을 못 찾음, 11-15)
import numpy as np   # noqa: E402
import rclpy   # noqa: E402
from rclpy.qos import qos_profile_sensor_data   # noqa: E402
from rclpy.time import Time   # noqa: E402
from sensor_msgs.msg import LaserScan   # noqa: E402

ARGS = [a for a in sys.argv[1:] if not a.startswith('--')]
VIEW = float(ARGS[0]) if ARGS else 1.0
USE_TF = '--tf' in sys.argv
KEEP = 3.0   # s, 점이 남는 시간
XYZ = {'/scan_front': (0.1318, 0.0), '/scan_rear': (-0.1318, 0.0)}   # robot.urdf.xacro laser_* (z 는 그림에 안 씀)
TF = {   # (roll, yaw)
    'NOW: current URDF (front rpy pi 0 0, rear pi 0 pi)': {'/scan_front': (math.pi, 0.0), '/scan_rear': (math.pi, math.pi)},
    'FIX: yaw +180 (front pi 0 pi, rear pi 0 0)': {'/scan_front': (math.pi, math.pi), '/scan_rear': (math.pi, 0.0)},
}
COLOR = {'/scan_front': (0.85, 0.1, 0.1), '/scan_rear': (0.1, 0.3, 0.9)}
FOOT = [(0.25, 0.07), (0.25, -0.07), (0.1328, -0.1676), (-0.2135, -0.1676), (-0.2135, 0.1676), (0.1328, 0.1676)]   # nav2_params footprint

lock = threading.Lock()
buf = []   # (받은 시각, 토픽, 센서 좌표 x, y)


def on(topic, m):
    r = np.array(m.ranges, dtype=float)
    a = m.angle_min + m.angle_increment * np.arange(len(r))
    ok = np.isfinite(r) & (r >= max(float(m.range_min), 0.02)) & (r <= VIEW * 1.5)
    with lock:
        buf.append((time.time(), topic, r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])))


def to_base(topic, x, y, roll, yaw):
    # 센서 → base_link: Rz(yaw) · Rx(roll). roll π 는 y 부호를 뒤집음 (위 · 아래 뒤집어 단 라이다)
    y = y * math.cos(roll)
    c, s = math.cos(yaw), math.sin(yaw)
    tx, ty = XYZ[topic]
    return tx + c * x - s * y, ty + s * x + c * y


def tf_from_buffer(tfbuf):
    """실행 중인 TF 로 (x, y, roll, yaw) — 못 찾으면 None"""
    out = {}
    for t, frame in (('/scan_front', 'laser_front'), ('/scan_rear', 'laser_rear')):
        try:
            tr = tfbuf.lookup_transform("base_link", frame, Time()).transform
        except Exception:   # noqa: BLE001
            return None
        q = tr.rotation
        roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        out[t] = (tr.translation.x, tr.translation.y, roll, yaw)
    return out


def main():
    rclpy.init()
    node = rclpy.create_node('lidar_view')
    tfbuf = None
    if USE_TF:
        import tf2_ros
        tfbuf = tf2_ros.Buffer()
        tf2_ros.TransformListener(tfbuf, node)
    for t in XYZ:
        node.create_subscription(LaserScan, t, lambda m, t=t: on(t, m), qos_profile_sensor_data)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    if USE_TF:
        live = None
        t_end = time.time() + 15
        while live is None and time.time() < t_end:
            time.sleep(0.5)
            live = tf_from_buffer(tfbuf)
        if live is None:
            print('base_link → laser_front · laser_rear TF 를 15 s 안에 못 받음 (bringup · robot_state_publisher 확인)')
            return
        for t, (x, y, roll, yaw) in live.items():
            XYZ[t] = (x, y)
            print(f'{t}: 실행 중 TF xy ({x:+.4f}, {y:+.4f}) · roll {math.degrees(roll):+.1f}° · yaw {math.degrees(yaw):+.1f}°', flush=True)
        TF.clear()
        TF[f'LIVE TF  front yaw {math.degrees(live["/scan_front"][3]):+.0f} deg, rear yaw {math.degrees(live["/scan_rear"][3]):+.0f} deg'] = {k: (v[2], v[3]) for k, v in live.items()}
    fig, axes = plt.subplots(1, len(TF), figsize=(6.8 * len(TF), 6.8), squeeze=False)
    axes = axes[0]
    fig.canvas.manager.set_window_title('lidar_view: red = front lidar, blue = rear lidar, points fade in 3 s')
    sc = {}
    for ax, name in zip(axes, TF):
        # 위 = 로봇 앞(+x), 왼쪽 = 로봇 왼쪽(+y) → 그림 가로축 = −y, 세로축 = x
        ax.fill([-p[1] for p in FOOT], [p[0] for p in FOOT], color=(0.85, 0.85, 0.85), zorder=0)
        ax.annotate('', xy=(0, 0.33), xytext=(0, 0.1), arrowprops=dict(arrowstyle='->', lw=2, color='k'))
        ax.text(0, 0.36, 'FRONT', ha='center', fontsize=12)
        ax.text(-VIEW * 0.92, 0, 'LEFT', ha='left', va='center', fontsize=11)
        ax.text(VIEW * 0.92, 0, 'RIGHT', ha='right', va='center', fontsize=11)
        for t in XYZ:
            ax.plot(-XYZ[t][1], XYZ[t][0], 'o', color=COLOR[t], ms=8, mec='k')
        ax.set_xlim(-VIEW, VIEW)
        ax.set_ylim(-VIEW, VIEW)
        ax.set_aspect('equal')
        ax.grid(alpha=0.3)
        ax.set_title(name, fontsize=13)
        sc[name] = ax.scatter([], [], s=4)
    try:
        while plt.fignum_exists(fig.number):
            now = time.time()
            with lock:
                buf[:] = [b for b in buf if now - b[0] < KEEP]
                items = list(buf)
            for name, tf in TF.items():
                xs, ys, cs = [], [], []
                for t0, t, x, y in items:
                    bx, by = to_base(t, x, y, *tf[t])
                    alpha = 1.0 - (now - t0) / KEEP
                    xs.append(-by)
                    ys.append(bx)
                    cs.append(np.tile([*COLOR[t], max(alpha, 0.05)], (len(bx), 1)))
                if xs:
                    sc[name].set_offsets(np.column_stack([np.concatenate(xs), np.concatenate(ys)]))
                    sc[name].set_facecolors(np.concatenate(cs))
                    sc[name].set_edgecolors('none')
            plt.pause(0.15)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
