"""채점용 bag · 자세 함수(261027_0953/slam/eval_slam.py 에서 이 시험에 쓰는 것만 그대로 옮김 — 원본은 scipy 를 불러 시스템 python3 에서 안 뜸)."""
import csv
import math
import os

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def read_bag(path, topics):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id="mcap"), rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=[t for t in topics if t in types]))
    while r.has_next():
        topic, data, t_ns = r.read_next()
        yield topic, deserialize_message(data, get_message(types[topic])), t_ns * 1e-9


def stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def read_tf(path):
    """출력 bag 의 /tf → {(부모, 자식): [(stamp, x, y, yaw, 수신)]}"""
    out = {}
    for _, m, rx in read_bag(path, ["/tf"]):
        for tr in m.transforms:
            t = tr.transform
            out.setdefault((tr.header.frame_id, tr.child_frame_id), []).append(
                (stamp(tr.header), t.translation.x, t.translation.y, yaw_of(t.rotation), rx))
    return {k: np.array(sorted(v)) for k, v in out.items()}



def compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return np.array([a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2]])


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi



def truth(rec):
    rows = list(csv.DictReader(open(os.path.join(rec, "truth.csv"))))
    return np.array([[float(r["sim_s"]), float(r["x"]), float(r["y"]), float(r["yaw"])] for r in rows])


def to_world(p_map, start):
    c, s = math.cos(start[2]), math.sin(start[2])
    return np.stack([start[0] + c * p_map[:, 0] - s * p_map[:, 1], start[1] + s * p_map[:, 0] + c * p_map[:, 1]], 1)

