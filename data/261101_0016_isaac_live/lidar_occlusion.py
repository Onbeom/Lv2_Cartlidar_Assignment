"""라이다 차체 가림을 센서 frame 에서 계산한다(장면 scene_live.py --occluder_mode analytic 이 씀, 2026-11-01).
가림 상자(sensors.json base_boxes, 센서마다 sensors[이름].occluder_boxes)는 base_link 에 붙어 있고 라이다도 base_link 에 붙어 있으므로
센서에서 본 가림 거리는 칸마다 늘 같다. 그래서 각도 칸 중심마다 광선 하나를 상자 · 원기둥과 교차해 한 번만 구해 두고 스캔에 min 으로 넣는다.
(레이캐스터 추적 방식은 가림 상자가 출발 자세 자리에 굳어 월드에 허상 상자를 남김 — isaac_live README '설계 40 결과'.)
사용(확인): python3 lidar_occlusion.py <sensors.json> → 센서마다 가려진 칸 수 · 거리 범위"""
import json
import math
import sys

import numpy as np

START_RADIUS = 0.025   # sensors_cfg.LDS03_START_RADIUS 와 같게: 광선은 머리 반지름 바깥에서 시작


def quat_xyzw_to_mat(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _hit_box(o, d, c, R, size):
    """광선 o + t d 와 상자(중심 c, 회전 R, 크기 size)의 첫 교차 t(없으면 inf). 좌표는 base_link"""
    lo, ld = R.T @ (o - c), R.T @ d
    h = np.asarray(size) / 2
    t0, t1 = -math.inf, math.inf
    for i in range(3):
        if abs(ld[i]) < 1e-12:
            if abs(lo[i]) > h[i]:
                return math.inf
            continue
        a, b = (-h[i] - lo[i]) / ld[i], (h[i] - lo[i]) / ld[i]
        t0, t1 = max(t0, min(a, b)), min(t1, max(a, b))
    if t1 < max(t0, 0.0):
        return math.inf
    return t0 if t0 >= 0 else t1


def _hit_cyl(o, d, c, R, size):
    """원기둥(축 = 자기 Z, 지름 size[0], 높이 size[2]) 옆면 · 뚜껑 첫 교차"""
    lo, ld = R.T @ (o - c), R.T @ d
    r, hz = size[0] / 2, size[2] / 2
    best = math.inf
    A, B, C = ld[0] ** 2 + ld[1] ** 2, 2 * (lo[0] * ld[0] + lo[1] * ld[1]), lo[0] ** 2 + lo[1] ** 2 - r * r
    if A > 1e-12:
        disc = B * B - 4 * A * C
        if disc >= 0:
            for t in ((-B - math.sqrt(disc)) / (2 * A), (-B + math.sqrt(disc)) / (2 * A)):
                if t >= 0 and abs(lo[2] + t * ld[2]) <= hz:
                    best = min(best, t)
    if abs(ld[2]) > 1e-12:
        for zc in (-hz, hz):
            t = (zc - lo[2]) / ld[2]
            p = lo + t * ld
            if t >= 0 and p[0] ** 2 + p[1] ** 2 <= r * r:
                best = min(best, t)
    return best


def occluder_ranges(sensors, name, n_bins, r_min=0.05, r_max=12.0):
    """센서 이름(lds03_front 등) → 칸마다 가림 거리(센서 원점에서, 없으면 inf). 칸 k 중심 각도 = −π + (k + 0.5)·(2π/n_bins), 센서 frame.
    광선 시작은 START_RADIUS 바깥(센서 머리 안쪽 교차는 버림)."""
    s = sensors['sensors'][name]
    boxes = {b['name']: b for b in sensors['base_boxes']}
    Rs, ps = quat_xyzw_to_mat(s['quat_xyzw']), np.asarray(s['pos'], float)
    inc = 2 * math.pi / n_bins
    out = np.full(n_bins, np.inf)
    for k in range(n_bins):
        a = -math.pi + (k + 0.5) * inc
        d = Rs @ np.array([math.cos(a), math.sin(a), 0.0])
        o = ps + START_RADIUS * d
        best = math.inf
        for n in s['occluder_boxes']:
            b = boxes[n]
            c, R = np.asarray(b['pos'], float), quat_xyzw_to_mat(b['quat_xyzw'])
            t = (_hit_box if b['shape'] == 'box' else _hit_cyl)(o, d, c, R, b['size'])
            best = min(best, t)
        rng = best + START_RADIUS
        if r_min <= rng <= r_max:
            out[k] = rng
    return out


if __name__ == '__main__':
    S = json.load(open(sys.argv[1]))
    for nm in ('lds03_front', 'lds03_rear'):
        r = occluder_ranges(S, nm, int(S['spec']['lds03']['samples']))
        f = np.isfinite(r)
        print(nm, '가려진 칸', int(f.sum()), '/', len(r), '거리', round(float(r[f].min()), 4), '~', round(float(r[f].max()), 4))
