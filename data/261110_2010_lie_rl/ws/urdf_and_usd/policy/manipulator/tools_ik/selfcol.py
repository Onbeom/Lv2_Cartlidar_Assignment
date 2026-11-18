"""팔 자기 충돌 근사 (2026-11-04): 팀 URDF 사슬로 어깨 · 팔꿈치 · 손목 · 집게 점을 구해, 집게 (손목 롤 링크 + 4 cm ~ 잡는 점 + 2 cm) 및 쥔 퍽이
윗팔 (어깨 -> 팔꿈치) · 아랫팔 (팔꿈치 -> 손목) 선분에 얼마나 가까운지 [m]. 모터 · 링크 반폭 약 25 mm, 집게 · 손가락 반폭 약 30 mm -> 0.055 m 아래면 겹침."""
import math, numpy as np
from td_ik import Rz, Ry, Rx

def chain(q):
    p = np.array([0.096, 0, 0.131]); R = Rz(q[0])
    p = p + R @ [0, 0, 0.0595]; R = R @ Ry(q[1]); sh = p.copy()
    p = p + R @ [0.024, 0, 0.128]; R = R @ Ry(q[2]); el = p.copy()
    p = p + R @ [0.124, 0, 0]; R = R @ Ry(q[3]); wr = p.copy()
    p = p + R @ [0.028, 0, 0]; R = R @ Rx(q[4]); gr = p.copy()
    return sh, el, wr, gr, R

def seg_dist(a, b, c, d):
    u, v, w = b - a, d - c, a - c
    A, B, C, D, E = u @ u, u @ v, v @ v, u @ w, v @ w
    den = A * C - B * B
    s = np.clip((B * E - C * D) / den, 0, 1) if den > 1e-12 else 0.0
    t = np.clip((B * s + E) / C, 0, 1) if C > 1e-12 else 0.0
    s = np.clip((B * t - D) / A, 0, 1) if A > 1e-12 else 0.0
    return float(np.linalg.norm((a + s * u) - (c + t * v)))

def self_dist(q, puck=None):
    """puck = (중심 · 축) 집게 몸 좌표 (없으면 집게만)."""
    sh, el, wr, gr, R = chain(q)
    g0, g1 = gr + R @ [0.04, 0, 0], gr + R @ [0.16, 0, 0]
    d = min(seg_dist(g0, g1, sh, el), seg_dist(g0, g1, el, wr) if True else 9)
    if puck is not None:
        c, ax = puck
        pc = gr + R @ c; pa = R @ ax
        d = min(d, seg_dist(pc - 0.03 * pa, pc + 0.03 * pa, sh, el), seg_dist(pc - 0.03 * pa, pc + 0.03 * pa, el, wr))
    return d
