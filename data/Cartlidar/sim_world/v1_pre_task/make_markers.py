"""퍽 AprilTag와 과제 Aruco 마커 그림 만들기 (가상 월드 질감용).

- 퍽: tag36h11 번호 0~11. 검은 테두리 바깥 한 변 = TAG_SIZE(가정 0.018 m).
  그림 = 흰 여백 1칸 포함 10 x 10칸 (검은 테두리 8칸).
- 과제 마커: DICT_4X4_50 번호 10·11·12(가상 과제 A·B·C, 가정). 그림 = 흰 여백 1칸 포함 8 x 8칸 (검은 테두리 6칸).
  검은 테두리 바깥 한 변 = 0.10 m(가정).

실행 (PC): ~/isaacsim-venv/bin/python -I make_markers.py
"""
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
PX_PER_CELL = 32


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


def main():
    out = HERE / "textures"
    out.mkdir(exist_ok=True)
    tag_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    for i in range(12):
        marker_png(tag_dict, i, 8, out / f"tag36h11_{i:02d}.png")
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    for i in (10, 11, 12):
        marker_png(aruco_dict, i, 6, out / f"aruco4x4_{i:02d}.png")
    print("wrote", sorted(p.name for p in out.glob("*.png")))


if __name__ == "__main__":
    main()
