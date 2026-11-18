"""SLAM 비교(모의) 평가: 기록 폴더(truth.csv)와 재생 결과(cuvslam_* · carto_*)로 계획 문서 3장의 측정값을 낸다.

  A1 위치 오차 RMSE [m]  : 추정 map → base_footprint 와 정답의 평면 거리, 0.1 s 격자
  A2 마지막 위치 오차 [m] : 정답 마지막 시각(출발 칸으로 돌아온 뒤)
  A3 헤딩 오차 RMSE [°]
  A4 추적 끊김            : cuVSLAM /visual_slam/status vo_state ≠ 1(성공)로 바뀐 횟수 · 그 시간 비율.
                            보조(두 방식 같게): 0.1 s 사이 추정 이동이 정답 이동과 0.10 m 넘게 다른 '튐' 횟수
  C1 계산 부담            : 노드 CPU 시간 · 최대 RSS(proc_usage.csv), GPU 메모리 최대(cuVSLAM gpu_mem.csv, 재생 전 대비)
  C2 출력 지연 [ms]       : 출력 bag 수신 시각(시뮬레이션 시계) − 메시지 stamp, 중앙값 · 95 %
  M1 지도(Cartographer)   : 유령 벽 비율 · 벽 위치 오차 · 채움률(experiments/cartographer_ab/evaluate_maps.py 와 같은 정의),
                            정답 벽 = layout.json 칸 벽 선분 + 선반 둘레(라이다 높이 0.068 m 에 걸리는 것)

좌표: 두 방식 모두 map 원점 = 시작 때 base_footprint. 정답 첫 줄 자세로 world 에 옮긴다(정렬 맞춤 없음 = 실제 쓸 때와 같음).
참고로 궤적 전체를 강체 맞춤한 RMSE 도 함께 적는다(시작 자세 오차를 뺀 모양 오차).

실행 (PC, ROS 환경): source /opt/ros/lyrical/setup.bash && python3 261027_0953/slam/eval_slam.py <기록 폴더> [<기록 폴더> ...] --out <결과 폴더>
"""
import argparse
import csv
import glob
import json
import math
import os
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.spatial import cKDTree

HERE = os.path.dirname(os.path.abspath(__file__))
TOP = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(TOP, "experiments", "cartographer_ab"))
import evaluate_maps as em  # noqa: E402  (load_map · metrics · align · transform 재사용)

GRID_S = 0.1
JUMP_M = 0.10
LIDAR_Z = 0.068   # 라이다 높이(URDF lidar_*_scan z). 선반 0.08 m 은 걸림


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


def at(series, t):
    """t 이전 가장 최근 값(x, y, yaw). 없으면 None"""
    i = np.searchsorted(series[:, 0], t, side="right") - 1
    return None if i < 0 else series[i, 1:4]


def compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return np.array([a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2]])


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def rigid_fit(p, q):
    """p → q 평면 강체 맞춤(최소제곱)"""
    pc, qc = p.mean(0), q.mean(0)
    h = (p - pc).T @ (q - qc)
    th = math.atan2(h[0, 1] - h[1, 0], h[0, 0] + h[1, 1])
    c, s = math.cos(th), math.sin(th)
    rot = np.array([[c, -s], [s, c]])
    return (p - pc) @ rot.T + qc


def truth(rec):
    rows = list(csv.DictReader(open(os.path.join(rec, "truth.csv"))))
    return np.array([[float(r["sim_s"]), float(r["x"]), float(r["y"]), float(r["yaw"])] for r in rows])


def gt_walls(layout, step=0.01):
    d = json.load(open(layout))
    segs = set()
    for c in d["cells"]:
        for e in c["edges"]:
            a, b = sorted(e["span"])
            segs.add((e["axis"], round(e["line"], 4), round(a, 4), round(b, 4)))
    pts = []
    for axis, line, a, b in segs:
        for u in np.arange(a, b + 1e-9, step):
            pts.append((u, line) if axis == "x" else (line, u))
    for sh in d["shelves"]:
        if sh["size"][2] > LIDAR_Z:
            (cx, cy, _), (sx, sy, _) = sh["center"], sh["size"]
            for u in np.arange(-sx / 2, sx / 2 + 1e-9, step):
                pts += [(cx + u, cy - sy / 2), (cx + u, cy + sy / 2)]
            for v in np.arange(-sy / 2, sy / 2 + 1e-9, step):
                pts += [(cx - sx / 2, cy + v), (cx + sx / 2, cy + v)]
    return np.array(pts)


def to_world(p_map, start):
    c, s = math.cos(start[2]), math.sin(start[2])
    return np.stack([start[0] + c * p_map[:, 0] - s * p_map[:, 1], start[1] + s * p_map[:, 0] + c * p_map[:, 1]], 1)


def pose_metrics(tf, tr, start, kind):
    m2o, o2b = tf.get(("map", "odom")), tf.get(("odom", "base_footprint"))
    if m2o is None or o2b is None:
        return {"ok": False, "why": f"TF 없음: map→odom {m2o is not None}, odom→base_footprint {o2b is not None}"}
    t0 = max(tr[0, 0], m2o[0, 0], o2b[0, 0])
    grid = np.arange(t0, tr[-1, 0], GRID_S)
    est, ref = [], []
    for t in grid:
        a, b = at(m2o, t), at(o2b, t)
        est.append(compose(a, b))
        j = min(np.searchsorted(tr[:, 0], t), len(tr) - 1)
        ref.append(tr[j, 1:4])
    est, ref = np.array(est), np.array(ref)
    w = to_world(est[:, :2], start)
    wyaw = wrap(est[:, 2] + start[2])
    e = np.linalg.norm(w - ref[:, :2], axis=1)
    eh = np.degrees(np.abs(wrap(wyaw - ref[:, 2])))
    fit = rigid_fit(w, ref[:, :2])
    de = np.linalg.norm(np.diff(w, axis=0) - np.diff(ref[:, :2], axis=0), axis=1)
    own = (m2o, o2b) if kind == "cuvslam" else (m2o,)   # Cartographer 구성의 odom → base 는 bag 의 바퀴 TF(재생)라 지연에서 뺌
    lat = np.concatenate([(s[:, 4] - s[:, 0]) for s in own])
    return {"ok": True, "n": len(grid), "A1_rmse_m": float(np.sqrt((e ** 2).mean())), "A1_max_m": float(e.max()),
            "A1_fit_rmse_m": float(np.sqrt((np.linalg.norm(fit - ref[:, :2], axis=1) ** 2).mean())),
            "A2_final_m": float(e[-1]), "A3_yaw_rmse_deg": float(np.sqrt((eh ** 2).mean())),
            "jumps": int((de > JUMP_M).sum()),
            "C2_latency_ms_median": float(np.median(lat) * 1e3), "C2_latency_ms_p95": float(np.percentile(lat, 95) * 1e3),
            "traj": {"t": grid.tolist(), "est": w.tolist(), "ref": ref[:, :2].tolist()}}


def proc_usage(d):
    p = os.path.join(d, "proc_usage.csv")
    if not os.path.exists(p):
        return {}
    kv = dict(ln.strip().split(",") for ln in open(p) if "," in ln)
    return {"C1_cpu_s": float(kv.get("cpu_s", "nan")), "C1_max_rss_mb": float(kv.get("max_rss_kb", "nan")) / 1024}


def gpu_peak(d):
    p = os.path.join(d, "gpu_mem.csv")
    if not os.path.exists(p):
        return {}
    v = [float(r["used_mib"]) for r in csv.DictReader(open(p)) if r["used_mib"].strip()]
    return {"C1_gpu_peak_mib": max(v), "C1_gpu_over_start_mib": max(v) - v[0]} if v else {}


def vo_state(d):
    s = [(stamp(m.header), m.vo_state) for _, m, _ in read_bag(os.path.join(d, "out_bag"), ["/visual_slam/status"])]
    if not s:
        return {}
    ok = np.array([v == 1 for _, v in s])
    return {"A4_lost_events": int(((~ok[1:]) & ok[:-1]).sum() + (not ok[0])), "A4_lost_ratio": float((~ok).mean()),
            "vo_state_counts": {int(k): int(n) for k, n in zip(*np.unique([v for _, v in s], return_counts=True))}}


def map_metrics(d, start, gt_tree, layout_pts):
    stem = os.path.join(d, "map")
    if not os.path.exists(stem + ".yaml"):
        return {"M1_ok": False}
    occ, free, _ = em.load_map(stem)
    occ_w = to_world(occ, start)
    free_w = to_world(free, start)
    region = cKDTree(free_w).query(layout_pts)[0] <= em.REGION_M
    reg_tree = cKDTree(layout_pts[region])
    raw = em.metrics(occ_w, gt_tree, reg_tree)
    dx, dy, dth = em.align(occ_w - start[:2], cKDTree(layout_pts - start[:2]))   # 시작 위치 둘레로 돌려 맞춤
    al = em.metrics(em.transform(occ_w - start[:2], dx, dy, dth) + start[:2], gt_tree, reg_tree)
    return {"M1_ok": True, **{f"M1_{k}": v for k, v in raw.items()}, **{f"M1_aligned_{k}": v for k, v in al.items()},
            "M1_align": [dx, dy, math.degrees(dth)]}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("recs", nargs="+")
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    rows = []
    for rec in args.recs:
        meta = json.load(open(os.path.join(rec, "meta.json")))
        tr = truth(rec)
        start = tr[0, 1:4]
        layout = os.path.join(meta["world"], "layout.json")
        walls = gt_walls(layout)
        gt_tree = cKDTree(walls)
        for kind in ("cuvslam", "carto"):
            for d in sorted(glob.glob(os.path.join(rec, f"{kind}_*"))):
                name = os.path.basename(d)
                var = name.split("_")[1] if kind == "cuvslam" and name.split("_")[1].startswith("V") else "V0"
                method = f"cuvslam_{var}" if kind == "cuvslam" else kind      # 변형: cuvslam_<V0|V1|V2>_<시각>(이름에 없으면 V0)
                r = {"rec": os.path.basename(rec), "light": meta["light_level"], "method": method, "run": name}
                r.update(pose_metrics(read_tf(os.path.join(d, "out_bag")), tr, start, kind))
                r.update(proc_usage(d))
                if kind == "cuvslam":
                    r.update(gpu_peak(d))
                    r.update(vo_state(d))
                else:
                    r.update(map_metrics(d, start, gt_tree, walls))
                rows.append(r)
                print(r["rec"], kind, {k: (round(v, 3) if isinstance(v, float) else v) for k, v in r.items()
                                       if k not in ("traj", "rec", "method", "run")})
    json.dump(rows, open(os.path.join(args.out, "eval_slam.json"), "w"), ensure_ascii=False, indent=1)
    keys = ["A1_rmse_m", "A1_fit_rmse_m", "A1_max_m", "A2_final_m", "A3_yaw_rmse_deg", "A4_lost_events", "A4_lost_ratio", "jumps",
            "C1_cpu_s", "C1_max_rss_mb", "C1_gpu_over_start_mib", "C2_latency_ms_median", "C2_latency_ms_p95",
            "M1_ghost_ratio", "M1_mean_err_cm", "M1_coverage", "M1_aligned_mean_err_cm"]
    lines = ["| 기록 | 조명 | 방식 | " + " | ".join(keys) + " |", "|" + "---|" * (len(keys) + 3)]
    for r in rows:
        lines.append(f"| {r['rec']} | {r['light']} | {r['method']} | " + " | ".join(
            (f"{r[k]:.3f}" if isinstance(r.get(k), float) else str(r.get(k, "-"))) for k in keys) + " |")
    # 조명 · 방식별 평균(반복 기록 사이)
    lines += ["", "조명 · 방식별 평균 (n = 기록 수)", "", "| 조명 | 방식 | n | " + " | ".join(keys) + " |", "|" + "---|" * (len(keys) + 3)]
    for light, method in sorted({(r["light"], r["method"]) for r in rows}):
        g = [r for r in rows if r["light"] == light and r["method"] == method]
        cells = []
        for k in keys:
            v = [r[k] for r in g if isinstance(r.get(k), (int, float))]
            cells.append(f"{np.mean(v):.3f}" if v else "-")
        lines.append(f"| {light} | {method} | {len(g)} | " + " | ".join(cells) + " |")
    open(os.path.join(args.out, "summary.md"), "w").write("\n".join(lines) + "\n")
    print("\n".join(lines))


if __name__ == "__main__":
    main()
