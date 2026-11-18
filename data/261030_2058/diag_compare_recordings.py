"""녹화끼리 /scan_front · /scan_rear 가 같은지(칸 단위) 비교(모의, 도구 확인). 사용: python diag_compare_recordings.py <녹화 A> <녹화 B>"""
import sys
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def scans(p, topic):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=p + '/bag', storage_id='mcap'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    meta = {v.name: v for v in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    out = []
    while r.has_next():
        t, d, _ = r.read_next()
        out.append(np.nan_to_num(np.array(deserialize_message(d, get_message(meta[t].type)).ranges), posinf=-1))
    return out


for topic in ('/scan_front', '/scan_rear'):
    a, b = scans(sys.argv[1], topic), scans(sys.argv[2], topic)
    k = np.array([(x != y).sum() for x, y in zip(a, b)])
    print(topic, len(a), len(b), '다른 스캔', int((k > 0).sum()), f'({(k > 0).mean() * 100:.1f} %)', '다른 칸 중앙(다른 스캔 중)', float(np.median(k[k > 0])) if (k > 0).any() else 0)
