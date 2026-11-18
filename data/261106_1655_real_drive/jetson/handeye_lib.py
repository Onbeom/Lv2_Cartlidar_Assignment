"""P2b 팔 카메라 손-눈 보정 공통: URDF 기구학(base_link → 카메라가 달린 링크), ChArUco 보드 자세, 풀이 · 검사.

카메라 프레임 = URDF arm_camera_optical_frame (팔 카메라 노드가 180° 돌려 발행하는 화면 기준). 보정 파일(내부값)은 돌리기 전 화면 기준이라
보드는 돌리기 전 화면에서 찾고, 광학 프레임으로는 z 축 180° 회전(Rz(π))을 곱해 옮긴다.
"""
import math
import subprocess
import xml.etree.ElementTree as ET

import cv2
import numpy as np


# ---------------- 회전 · 변환 ----------------
def rpy_to_R(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def R_to_rpy(R):
    p = math.atan2(-R[2, 0], math.hypot(R[0, 0], R[1, 0]))
    return math.atan2(R[2, 1], R[2, 2]), p, math.atan2(R[1, 0], R[0, 0])


def axis_angle_R(axis, a):
    k = np.asarray(axis, float) / np.linalg.norm(axis)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * K @ K


def T_of(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.ravel(t)
    return T


def rot_deg(R):
    return math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(R) - 1) / 2))))


RZ_PI = T_of(np.diag([-1.0, -1.0, 1.0]), [0, 0, 0])   # 돌리기 전 광학 → 돌린(발행) 광학 프레임


# ---------------- URDF 기구학 ----------------
class Chain:
    """URDF 에서 base → tip 링크 사슬. q: {'joint1': rad, ...} (없는 움직이는 관절은 0)."""

    def __init__(self, urdf_xml, base='base_link', tip='link5'):
        r = ET.fromstring(urdf_xml)
        by_child = {j.find('child').get('link'): j for j in r.findall('joint')}
        self.joints = []
        link = tip
        while link != base:
            j = by_child[link]
            o = j.find('origin')
            xyz = [float(v) for v in (o.get('xyz', '0 0 0') if o is not None else '0 0 0').split()]
            rpy = [float(v) for v in (o.get('rpy', '0 0 0') if o is not None else '0 0 0').split()]
            ax = j.find('axis')
            axis = [float(v) for v in ax.get('xyz').split()] if ax is not None else [1, 0, 0]
            self.joints.append((j.get('name'), j.get('type'), T_of(rpy_to_R(*rpy), xyz), axis))
            link = j.find('parent').get('link')
        self.joints.reverse()
        self.fixed = {}
        for j in r.findall('joint'):   # 카메라 링크 설계값(비교용)
            o = j.find('origin')
            xyz = [float(v) for v in (o.get('xyz', '0 0 0') if o is not None else '0 0 0').split()]
            rpy = [float(v) for v in (o.get('rpy', '0 0 0') if o is not None else '0 0 0').split()]
            self.fixed[j.find('child').get('link')] = (j.find('parent').get('link'), T_of(rpy_to_R(*rpy), xyz))

    def fk(self, q):
        T = np.eye(4)
        for name, typ, To, axis in self.joints:
            T = T @ To
            if typ in ('revolute', 'continuous'):
                T = T @ T_of(axis_angle_R(axis, q.get(name, 0.0)), [0, 0, 0])
            elif typ == 'prismatic':
                T = T @ T_of(np.eye(3), np.asarray(axis) * q.get(name, 0.0))
        return T

    def design(self, tip, frame):
        """tip 링크 → frame 의 URDF 설계 변환 (고정 관절만 따라감)."""
        T = np.eye(4)
        f = frame
        while f != tip:
            parent, To = self.fixed[f]
            T = To @ T
            f = parent
        return T


def urdf_from_xacro(path):
    return subprocess.check_output(['xacro', path], text=True)


# ---------------- 보드 자세 ----------------
def board_pose(corners, ids, K, D, square_m, squares_x=10, square_y_m=None):
    """ChArUco 코너(돌리기 전 화면 px) → T_cam_board (돌리기 전 광학 프레임), 재투영 RMS px. 코너 id k = (k % (sx-1)+1, k // (sx-1)+1) 칸 꼭짓점.
    square_m = 가로 칸 크기, square_y_m = 세로 칸 크기 (모니터 화면 비율이 늘어진 경우, 없으면 가로와 같음)."""
    n = squares_x - 1
    sy = square_m if square_y_m is None else square_y_m
    obj = np.array([[((i % n) + 1) * square_m, ((i // n) + 1) * sy, 0.0] for i in ids], np.float64)
    img = np.asarray(corners, np.float64).reshape(-1, 1, 2)
    ok, rv, tv = cv2.solvePnP(obj, img, K, D, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return None, None
    rv, tv = cv2.solvePnPRefineLM(obj, img, K, D, rv, tv)
    proj, _ = cv2.projectPoints(obj, rv, tv, K, D)
    rms = float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - img.reshape(-1, 2)) ** 2, 1))))
    return T_of(cv2.Rodrigues(rv)[0], tv), rms


# ---------------- 풀이 · 검사 ----------------
METHODS = {'TSAI': cv2.CALIB_HAND_EYE_TSAI, 'PARK': cv2.CALIB_HAND_EYE_PARK, 'HORAUD': cv2.CALIB_HAND_EYE_HORAUD,
           'ANDREFF': cv2.CALIB_HAND_EYE_ANDREFF, 'DANIILIDIS': cv2.CALIB_HAND_EYE_DANIILIDIS}


def solve(T_base_link, T_cam_board, method='PARK'):
    """eye-in-hand: X = T_link_cam. 입력은 같은 길이 목록 (base→link, cam→board)."""
    Rg = [T[:3, :3] for T in T_base_link]
    tg = [T[:3, 3] for T in T_base_link]
    Rt = [T[:3, :3] for T in T_cam_board]
    tt = [T[:3, 3] for T in T_cam_board]
    R, t = cv2.calibrateHandEye(Rg, tg, Rt, tt, method=METHODS[method])
    return T_of(R, t)


def consistency(T_base_link, T_cam_board, X):
    """각 장에서 계산한 보드 자세 T_base_board = T_base_link · X · T_cam_board 의 흩어짐: (위치 편차 mm 목록, 회전 편차 ° 목록) — 평균 자세 기준."""
    Tb = [Tl @ X @ Tc for Tl, Tc in zip(T_base_link, T_cam_board)]
    P = np.array([T[:3, 3] for T in Tb])
    pm = P.mean(0)
    U, _, Vt = np.linalg.svd(sum(T[:3, :3] for T in Tb))
    Rm = U @ Vt
    dpos = [float(np.linalg.norm(p - pm) * 1e3) for p in P]
    drot = [rot_deg(Rm.T @ T[:3, :3]) for T in Tb]
    return dpos, drot, T_of(Rm, pm)


def diversity(T_base_link, min_pair_rad=0.3, min_axis_deg=30.0):
    """자세 다양성 (검토 조건 1): 장 쌍의 상대 회전(링크 프레임) 중 min_pair_rad 넘는 쌍 수, 그 쌍들의 회전축 중
    서로 min_axis_deg 넘게 벌어진 축 묶음 수(평행하지 않은 축 개수, 탐욕 군집). 반환 (쌍 수, 전체 쌍 수, 축 묶음 수, 축 목록)."""
    pairs, axes = 0, []
    n = len(T_base_link)
    for i in range(n):
        for j in range(i + 1, n):
            R = T_base_link[i][:3, :3].T @ T_base_link[j][:3, :3]
            rv = cv2.Rodrigues(R)[0].ravel()
            a = float(np.linalg.norm(rv))
            if a > min_pair_rad:
                pairs += 1
                axes.append(rv / a)
    groups = []
    for ax in axes:
        if all(math.degrees(math.acos(min(1.0, abs(float(ax @ g))))) > min_axis_deg for g in groups):
            groups.append(ax)
    return pairs, n * (n - 1) // 2, len(groups), groups
