"""팔 카메라 라벨 인식 색 검출 시제품 (A안, C++ 이식 전 설계용).
지금 color_detector.cpp 와 같은 마스크 위에: 태그(어둡고 채도 낮은 사각형) → 라벨(태그 × label_scale) →
둘레 띠 색으로 라벨 색 정함 → 그 색 마스크에 라벨을 더해 조각을 묶음 → 덩어리 구멍 채움 →
라벨 2개 이상이면 가까운 라벨로 나눔 → 회전 외접 사각형으로 채움 · 가로세로비."""
import math

import cv2
import numpy as np

HSV = {0: [([0, 80, 35], [10, 255, 255]), ([170, 80, 35], [179, 255, 255])], 1: [([40, 80, 35], [85, 255, 255])]}
CFG = dict(min_area_ratio=0.0004, min_fill=0.35, score_thr=0.3, dark_v_max=90, dark_s_max=90, tag_close=5,
           tag_min_px=30, tag_square_min=0.5, label_scale=1.25, ring_px_frac=0.2, ring_min_frac=0.25, crop_factor=0.0, body_v_min=35)


def color_masks(hsv, k, c):
    out = {}
    for cls, rs in HSV.items():
        m = np.zeros(hsv.shape[:2], np.uint8)
        for lo, hi in rs:
            low = list(lo)
            low[2] = c["body_v_min"]
            m |= cv2.inRange(hsv, np.array(low), np.array(hi))
        out[cls] = cv2.morphologyEx(cv2.morphologyEx(m, cv2.MORPH_OPEN, k), cv2.MORPH_CLOSE, k)
    return out


def find_labels(hsv, c):
    v = hsv[..., 2]
    dark = (v <= c["dark_v_max"]) & (hsv[..., 1] <= c["dark_s_max"])
    if c.get("dark_win", 0):   # 상대 문턱(2026-10-27 밤): 주변 평균보다 dark_c 넘게 어두운 곳만 — 어두운 조명에서 흰 선반이 V 70 대라 절대 문턱만 쓰면 배경 전체가 '검정'
        mean = cv2.blur(v.astype(np.float32), (c["dark_win"], c["dark_win"]))
        dark &= v.astype(np.float32) < mean - c["dark_c"]
    dark = dark.astype(np.uint8) * 255
    kk = cv2.getStructuringElement(cv2.MORPH_RECT, (c["tag_close"], c["tag_close"]))
    dark = cv2.morphologyEx(dark, cv2.MORPH_CLOSE, kk)
    cs, _ = cv2.findContours(dark, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    labels = []
    for cnt in cs:
        a = cv2.contourArea(cnt)
        if a < c["tag_min_px"]:
            continue
        (cx, cy), (w, h), ang = cv2.minAreaRect(cnt)
        if min(w, h) / max(w, h, 1e-6) < c["tag_square_min"] or a / max(w * h, 1e-6) < c["tag_square_min"]:
            continue
        s = c["label_scale"]
        poly = cv2.boxPoints(((cx, cy), (w * s, h * s), ang)).astype(np.int32)
        labels.append({"c": (cx, cy), "poly": poly, "side": math.sqrt(w * h) * s})
    return labels


def region_stats(mask):
    pts = cv2.findNonZero(mask)
    if pts is None:
        return None
    area = len(pts)
    (_, _), (w, h), _ = cv2.minAreaRect(pts)
    w, h = max(w, 1.0), max(h, 1.0)
    x, y, bw, bh = cv2.boundingRect(pts)
    return area, area / (w * h), min(w, h) / max(w, h), (x, y, bw, bh)


def detect(bgr, c=CFG, label_aware=True):
    hsv = cv2.cvtColor(cv2.GaussianBlur(bgr, (3, 3), 0), cv2.COLOR_BGR2HSV)
    H, W = hsv.shape[:2]
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    masks = color_masks(hsv, k, c)
    labels = find_labels(hsv, c) if label_aware else []
    # 라벨 색: 라벨 바로 바깥 띠에서 그 색 화소 비율이 가장 큰 색 (ring_min_frac 이상)
    for L in labels:
        lm = np.zeros((H, W), np.uint8)
        cv2.fillPoly(lm, [L["poly"]], 255)
        r = max(2, int(round(L["side"] * c["ring_px_frac"])))
        ring = cv2.dilate(lm, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r + 1, 2 * r + 1))) & ~lm
        n = max(1, cv2.countNonZero(ring))
        fr = {cls: cv2.countNonZero(ring & m) / n for cls, m in masks.items()}
        best = max(fr, key=fr.get)
        L["cls"] = best if fr[best] >= c["ring_min_frac"] else -1
        L["mask"] = lm
    out = []
    for cls, m in masks.items():
        own = [L for L in labels if L["cls"] == cls]
        comb = m.copy()
        for L in own:
            comb |= L["mask"]
        cs, _ = cv2.findContours(comb, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in cs:
            reg = np.zeros((H, W), np.uint8)
            cv2.drawContours(reg, [cnt], -1, 255, -1)   # 구멍 채움
            if not label_aware:
                reg &= m
            inside = [L for L in own if reg[int(L["c"][1]), int(L["c"][0])]]
            parts = [reg]
            if len(inside) >= 2:   # 가까운 라벨로 나눔
                ys, xs = np.nonzero(reg)
                cen = np.array([L["c"] for L in inside])
                lab = np.argmin((xs[:, None] - cen[:, 0]) ** 2 + (ys[:, None] - cen[:, 1]) ** 2, axis=1)
                parts = []
                for i in range(len(inside)):
                    p = np.zeros((H, W), np.uint8)
                    p[ys[lab == i], xs[lab == i]] = 255
                    parts.append(p)
            if len(inside) == 1 and c.get("crop_factor", 0) > 0 and (max(cv2.boundingRect(cnt)[2:]) / max(1,min(cv2.boundingRect(cnt)[2:])) > 2.5):
                L = inside[0]
                ys, xs = np.indices(reg.shape)
                # 라벨 중심 주위만 남김: 같은 색 이웃과 붙은 긴 꼬리를 줄이는 설계 변수.
                radius = L["side"] * c["crop_factor"]
                parts = [part & (((xs - L["c"][0])**2 + (ys - L["c"][1])**2) <= radius**2).astype(np.uint8)*255 for part in parts]
            for p in parts:
                st = region_stats(p)
                if st is None:
                    continue
                area, fill, aspect, (x, y, bw, bh) = st
                if not label_aware:
                    fill = area / max(1, bw * bh)
                    aspect = min(bw, bh) / max(bw, bh)
                if area < c["min_area_ratio"] * H * W or fill < c["min_fill"]:
                    continue
                score = min(1.0, 0.5 * fill + 0.5 * aspect)
                if score >= c["score_thr"]:
                    out.append(((x + bw / 2 - W / 2) / (W / 2), (y + bh / 2 - H / 2) / (H / 2), bw / W, cls, round(score, 3)))
    return out
