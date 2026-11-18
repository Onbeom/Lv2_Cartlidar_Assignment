"""ChArUco 10x7 DICT_5X5_100 검출. OpenCV 4.6(옛 API) · 4.7 이상(새 API) 모두에서 돈다."""
import cv2
import numpy as np

SQUARES_X, SQUARES_Y = 10, 7
MARKER_RATIO = 0.72  # 마커 / 칸
N_CORNERS = (SQUARES_X - 1) * (SQUARES_Y - 1)


def _dict():
    return cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)


class Detector:
    def __init__(self, nx=SQUARES_X, ny=SQUARES_Y, erode=0):
        """nx × ny 칸 보드 (기본 10x7 = 내부값 보정 보드, 손-눈 보정은 6x4 도 씀).
        erode > 0: 마커는 흰색을 erode×erode 로 깎은 영상에서 찾고 코너는 원본에서 다듬음
        (밝은 노트북 화면이 과노출로 번져 마커 비트가 뭉개질 때, 11-13 실물)."""
        self.erode = erode
        self.d = _dict()
        if hasattr(cv2.aruco, 'CharucoDetector'):
            self.board = cv2.aruco.CharucoBoard((nx, ny), 1.0, MARKER_RATIO, self.d)
            self.det = cv2.aruco.CharucoDetector(self.board)
        else:
            self.board = cv2.aruco.CharucoBoard_create(nx, ny, 1.0, MARKER_RATIO, self.d)
            self.det = None

    def detect(self, gray):
        """(코너 N×2, id N, 마커 수). 없으면 코너는 None."""
        if self.erode:
            mk = cv2.erode(gray, np.ones((self.erode, self.erode), np.uint8))
            if hasattr(cv2.aruco, 'ArucoDetector'):
                mc, mi, _ = cv2.aruco.ArucoDetector(self.d).detectMarkers(mk)
            else:
                mc, mi, _ = cv2.aruco.detectMarkers(mk, self.d)
            n = 0 if mi is None else len(mi)
            if n == 0:
                return None, None, 0
            k, cc, ci = cv2.aruco.interpolateCornersCharuco(mc, mi, gray, self.board)   # 코너 다듬기는 원본에서
            if not k or cc is None:
                return None, None, n
            return cc.reshape(-1, 2), ci.ravel(), n
        if self.det is not None:
            cc, ci, mc, mi = self.det.detectBoard(gray)
            n = 0 if mi is None else len(mi)
            if cc is None or ci is None or len(cc) == 0:
                return None, None, n
            return cc.reshape(-1, 2), ci.ravel(), n
        mc, mi, _ = cv2.aruco.detectMarkers(gray, self.d)
        n = 0 if mi is None else len(mi)
        if n == 0:
            return None, None, 0
        k, cc, ci = cv2.aruco.interpolateCornersCharuco(mc, mi, gray, self.board)
        if not k or cc is None or k == 0:
            return None, None, n
        return cc.reshape(-1, 2), ci.ravel(), n


def object_points(ids, square=1.0):
    """코너 id → 보정판 평면 좌표 (칸 단위). 내부값은 칸 크기와 무관하다."""
    ids = np.asarray(ids)
    x = (ids % (SQUARES_X - 1) + 1) * square
    y = (ids // (SQUARES_X - 1) + 1) * square
    return np.stack([x, y, np.zeros_like(x)], axis=1).astype(np.float32)
