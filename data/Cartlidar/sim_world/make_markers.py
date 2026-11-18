"""가상 월드 질감 만들기: AprilTag(퍽 · 장소)와 바코드 라벨.

- tag36h11 번호 0~14 (과제 자료 2026-10-23: 퍽 0~11 = 검은 사각형 16 mm, 장소 L1~L3 = 12~14 = 48 mm).
  그림 = 흰 여백 1칸 포함 10 x 10칸(검은 테두리 8칸). 크기는 월드에서 정한다(퍽 라벨 20 mm, 장소 60 mm).
- 바코드 Code128 "P2-01"~"P2-04"(과제 자료 그림의 작업 지시 문자열). 라벨 35 x 20 mm(사용자 실측) 안에
  조용한 영역 포함 막대를 가로 전체로, 막대 높이 14 mm, 아래 글자 — 라벨 안 배치는 가정(labels-A4.pdf 못 받음).

실행 (PC): ~/isaacsim-venv/bin/python -I make_markers.py
"""
import sys
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import code128  # noqa: E402

PX_PER_CELL = 32
BARCODES = ["P2-01", "P2-02", "P2-03", "P2-04"]
LABEL_MM = (35.0, 20.0)       # 가로 · 세로 (사용자 실측)
BAR_TOP_MM, BAR_H_MM = 1.5, 14.0
PX_PER_MM = 24


def marker_png(dictionary, marker_id, black_cells, path):
    side = black_cells * PX_PER_CELL
    img = cv2.aruco.generateImageMarker(dictionary, marker_id, side, borderBits=1)
    canvas = np.full((side + 2 * PX_PER_CELL,) * 2, 255, np.uint8)
    canvas[PX_PER_CELL:PX_PER_CELL + side, PX_PER_CELL:PX_PER_CELL + side] = img
    cv2.imwrite(str(path), canvas)
    # 다시 읽어 검출되는지 확인(그림이 맞게 만들어졌는지)
    det = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
    big = cv2.copyMakeBorder(canvas, 64, 64, 64, 64, cv2.BORDER_CONSTANT, value=255)
    _, ids, _ = det.detectMarkers(big)
    if ids is None or [int(i) for i in ids.ravel()] != [marker_id]:
        raise SystemExit(f"{path.name}: 다시 검출 실패 ({ids})")


def barcode_png(text, path):
    w, h = int(LABEL_MM[0] * PX_PER_MM), int(LABEL_MM[1] * PX_PER_MM)
    canvas = np.full((h, w), 255, np.uint8)
    mods = code128.modules(text)
    # 모듈 경계를 화소에 고르게 나눔 (가로 전체 = 조용한 영역 포함 모듈 수)
    edges = np.round(np.linspace(0, w, len(mods) + 1)).astype(int)
    y0, y1 = int(BAR_TOP_MM * PX_PER_MM), int((BAR_TOP_MM + BAR_H_MM) * PX_PER_MM)
    for k, m in enumerate(mods):
        if m:
            canvas[y0:y1, edges[k]:edges[k + 1]] = 0
    font, scale, th = cv2.FONT_HERSHEY_SIMPLEX, 1.6, 3
    (tw, tht), _ = cv2.getTextSize(text, font, scale, th)
    cv2.putText(canvas, text, ((w - tw) // 2, h - int(0.8 * PX_PER_MM)), font, scale, 0, th, cv2.LINE_AA)
    cv2.imwrite(str(path), canvas)
    return len(mods)


def main():
    out = HERE / "textures"
    out.mkdir(exist_ok=True)
    tag_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    for i in range(15):
        marker_png(tag_dict, i, 8, out / f"tag36h11_{i:02d}.png")
    for t in BARCODES:
        n = barcode_png(t, out / f"barcode_{t}.png")
        print(f"{t}: {n} 모듈, 모듈 폭 {LABEL_MM[0] / n:.3f} mm")
    print("wrote", sorted(p.name for p in out.glob("*.png")))


if __name__ == "__main__":
    main()
