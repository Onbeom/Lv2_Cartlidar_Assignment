"""카메라 토픽 수신 측정 (Jetson 에서 실행, 직렬화된 바이트로 받아 역직렬화 부담을 줄임).

토픽마다 받은 장 수 · 평균 Hz · 가장 긴 간격 · header frame_id · 메시지 크기 · 받은 시각 − header 시각(지연) 을 json 으로 낸다.
실행: python3 measure_topics.py <초> <출력 json> [be|rel]
  be = best effort(인지 노드 image_qos: sensor_data 와 같음, 기본), rel = reliable
"""
import json
import struct
import sys
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

IMAGES = [
    '/top_camera_front/color/image_raw', '/top_camera_front/aligned_depth_to_color/image_raw',
    '/top_camera_rear/color/image_raw', '/top_camera_rear/aligned_depth_to_color/image_raw',
    '/arm_camera/image_raw',
    # 참고(판정 밖): 정렬 전 깊이 — 정렬 깊이가 느릴 때 원본이 오는지 가리려고
    '/top_camera_front/depth/image_rect_raw', '/top_camera_rear/depth/image_rect_raw',
]
INFOS = ['/top_camera_front/color/camera_info', '/top_camera_rear/color/camera_info', '/arm_camera/camera_info']


def header(raw):
    """CDR: 4바이트 머리 + stamp(sec int32, nanosec uint32) + frame_id(uint32 길이 + 글자)."""
    sec, nsec, n = struct.unpack_from('<iII', raw, 4)
    return sec + nsec * 1e-9, raw[16:16 + n - 1].decode(errors='replace')


def main():
    dur, out = float(sys.argv[1]), sys.argv[2]
    qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE) if sys.argv[3:] == ['rel'] else qos_profile_sensor_data
    rclpy.init()
    node = rclpy.create_node('measure_camera_topics')
    rec = {t: [] for t in IMAGES + INFOS}
    meta = {}

    def cb(topic):
        def f(raw):
            now = time.time()
            stamp, fid = header(raw)
            rec[topic].append((now, stamp))
            if topic not in meta:
                meta[topic] = {'frame_id': fid, 'bytes': len(raw)}
        return f

    for t in IMAGES:
        node.create_subscription(Image, t, cb(t), qos, raw=True)
    for t in INFOS:
        node.create_subscription(CameraInfo, t, cb(t), qos, raw=True)
    t_end = time.time() + dur
    while time.time() < t_end:
        rclpy.spin_once(node, timeout_sec=0.05)
    res = {'duration_s': dur, 'qos': sys.argv[3] if sys.argv[3:] else 'be', 'topics': {}}
    for t, r in rec.items():
        if len(r) < 2:
            res['topics'][t] = {'count': len(r), **meta.get(t, {})}
            continue
        tr = [x[0] for x in r]
        gaps = [b - a for a, b in zip(tr, tr[1:])]
        lag = sorted(x[0] - x[1] for x in r)
        res['topics'][t] = {
            'count': len(r), 'hz': round((len(r) - 1) / (tr[-1] - tr[0]), 2),
            'max_gap_s': round(max(gaps), 3), 'lag_median_s': round(lag[len(lag) // 2], 3), **meta.get(t, {}),
        }
    json.dump(res, open(out, 'w'), indent=1, ensure_ascii=False)
    print(json.dumps(res, indent=1, ensure_ascii=False))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
