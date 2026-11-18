"""R1 · R2 신호등 장면의 로봇 자세 · 방해 퍽 뽑기 (Isaac Sim 없이, numpy · trimesh · scipy).

자세 조건 (R1R2_설계_가설_261024.md 1장):
  (1) 접은 팔 S0 차체 바닥 윤곽(URDF 충돌 형상의 볼록 껍질)이 벽 · 기둥 · 선반에서 clearance 넘게 떨어짐, 미로 안
  (2) 카메라 → 칸 중심 3개 시선이 벽(0.38 m) · 기둥(0.40 m) · 선반(+퍽 0.16 m)에 안 막힘
  (3) 칸 3개(초록 아래 끝 ~ 빨강 위 끝, 지름 폭)가 모두 화면 안
거리 · 방향 · 정면에서 벗어난 각 · 카메라 흔들림을 무작위로 뽑고 조건을 만족할 때까지 다시 뽑는다.
"""
import math
import sys
from pathlib import Path

import numpy as np
from scipy.spatial import ConvexHull

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "sim_world"))
from robot_geom import Robot  # noqa: E402
from urdf_chain import fixed_transform, planar_pose, rpy_matrix  # noqa: E402

W, H = 640, 480
K = {"fx": 605.850769, "fy": 605.67987, "cx": 324.365814, "cy": 245.139175}   # 실물 261822072247 컬러 640x480
S0 = {"RMX52_Base_Yaw": 0.0, "RMX52_Shoulder": 1.4, "RMX52_Elbow": -1.0, "RMX52_Wrist_Pitch": -1.3,
      "RMX52_Wrist_Roll_Reversed": 0.0, "gripper_left_joint": 0.0}
WALL_TOP, POST_TOP, POST_HALF, SHELF_TOP = 0.38, 0.40, 0.018, 0.16   # 선반 0.08 + 서 있는 퍽 0.06 + 여유


class Scene:
    """한 월드의 장애물(벽 · 기둥 · 선반 상자)과 로봇 · 카메라."""

    def __init__(self, layout, drawing, urdf, pkg_dir, frame="top_camera_front_color_optical_frame", arm_q=S0,
                 shelf_top=SHELF_TOP):
        self.lay = layout
        self.boxes = []   # (cx, cy, hx, hy, top, 종류)
        for pn in drawing["panels"]:
            hx, hy = (pn["length"] / 2, 0.0015) if pn["axis"] == "x" else (0.0015, pn["length"] / 2)
            self.boxes.append((pn["center"][0], pn["center"][1], hx, hy, WALL_TOP, "wall"))
        for p in drawing["posts"]:
            self.boxes.append((p["x"], p["y"], POST_HALF, POST_HALF, POST_TOP, "post"))
        for s in layout["shelves"]:
            self.boxes.append((s["center"][0], s["center"][1], s["size"][0] / 2, s["size"][1] / 2, shelf_top, "shelf"))
        self.box_arr = np.array([b[:5] for b in self.boxes])   # 계산용 (N, 5)
        gx, gy = drawing["grid"]["x"], drawing["grid"]["y"]
        self.bounds = (gx[0], gx[-1], gy[0], gy[-1])
        robot = Robot(urdf, pkg_dir)
        pts = np.vstack([V[:, :2] for _, V in robot.world_shapes(arm_q)])
        self.footprint = pts[ConvexHull(pts).vertices]
        self.T_bf_cam = fixed_transform(str(urdf), frame)
        sl = layout["signal_light"]
        A = layout["assumptions"]["signal_light"]
        self.light = np.array([sl["x"], sl["y"]])
        self.seg_len, self.head_r = A["segment_length"], A["head_diameter"] / 2
        self.seg_z = [A["first_led_z"] + (i + 0.5) * A["segment_length"] for i in range(3)]   # 초록 · 노랑 · 빨강

    # ---------- 기하 ----------
    @staticmethod
    def _pt_box(x, y, b):
        return math.hypot(max(abs(x - b[0]) - b[2], 0.0), max(abs(y - b[1]) - b[3], 0.0))

    def clearance(self, pts):
        """점들(2D)에서 가장 가까운 장애물까지 거리."""
        P = np.asarray(pts, float).reshape(-1, 1, 2)
        B = self.box_arr
        dx = np.maximum(np.abs(P[..., 0] - B[:, 0]) - B[:, 2], 0.0)
        dy = np.maximum(np.abs(P[..., 1] - B[:, 1]) - B[:, 3], 0.0)
        return float(np.hypot(dx, dy).min())

    def los_blocked(self, p0, p1, end_skip=0.03):
        """3D 선분 p0 → p1 이 장애물 상자(바닥 ~ top)를 지나는지 (끝 end_skip 비율은 신호등 몸통이라 뺌). 슬랩 방법."""
        B = self.box_arr
        lo = np.stack([B[:, 0] - B[:, 2], B[:, 1] - B[:, 3], np.full(len(B), -1.0)], 1)
        hi = np.stack([B[:, 0] + B[:, 2], B[:, 1] + B[:, 3], B[:, 4]], 1)
        d = np.asarray(p1, float) - np.asarray(p0, float)
        t0 = np.zeros(len(B))
        t1 = np.full(len(B), 1.0 - end_skip)
        ok = np.ones(len(B), bool)
        for k in range(3):
            if abs(d[k]) < 1e-12:
                ok &= (p0[k] >= lo[:, k]) & (p0[k] <= hi[:, k])
            else:
                a, c = (lo[:, k] - p0[k]) / d[k], (hi[:, k] - p0[k]) / d[k]
                t0 = np.maximum(t0, np.minimum(a, c))
                t1 = np.minimum(t1, np.maximum(a, c))
        return bool((ok & (t0 <= t1)).any())

    @staticmethod
    def project(T_w_opt, p):
        q = np.linalg.inv(T_w_opt) @ np.append(np.asarray(p, float), 1.0)
        return K["fx"] * q[0] / q[2] + K["cx"], K["fy"] * q[1] / q[2] + K["cy"], q[2]

    def footprint_world(self, T_w_bf):
        return self.footprint @ T_w_bf[:2, :2].T + T_w_bf[:2, 3]

    # ---------- 자세 ----------
    def try_pose(self, rng, dist, bearing_max_deg, pos_jit, ang_jit_deg, clearance, margin_px):
        phi = rng.uniform(0, 2 * math.pi)
        d = dist + rng.uniform(-pos_jit, pos_jit)
        bearing = math.radians(rng.uniform(-bearing_max_deg, bearing_max_deg))
        pos = self.light + d * np.array([math.cos(phi), math.sin(phi)])
        yaw = phi + math.pi - bearing        # 신호등이 정면에서 bearing 만큼 왼쪽에 보임
        T_w_bf = planar_pose(pos[0], pos[1], yaw)
        x0, x1, y0, y1 = self.bounds
        fp = self.footprint_world(T_w_bf)
        if fp[:, 0].min() < x0 or fp[:, 0].max() > x1 or fp[:, 1].min() < y0 or fp[:, 1].max() > y1:
            return None
        # 바닥 윤곽 둘레(변 위 점)까지 봐야 얇은 벽이 사이로 빠지지 않음
        ring = np.vstack([fp[i] + (fp[(i + 1) % len(fp)] - fp[i]) * s for i in range(len(fp))
                          for s in np.linspace(0, 1, 15, endpoint=False)])
        if self.clearance(ring) < clearance:
            return None
        jit = [math.radians(rng.uniform(-ang_jit_deg, ang_jit_deg)) for _ in range(3)]
        J = np.eye(4)
        J[:3, :3] = rpy_matrix(*jit)   # 광학 좌표에서의 흔들림 (장착 오차 · 진동)
        T_w_cam = T_w_bf @ self.T_bf_cam @ J
        cam = T_w_cam[:3, 3]
        segs = []
        for z in self.seg_z:
            p = np.array([self.light[0], self.light[1], z])
            if self.los_blocked(cam, p):
                return None
            u, v, zc = self.project(T_w_cam, p)
            segs.append((u, v, zc))
        # 칸 3개 전체가 화면 안: 빨강 위 끝 · 초록 아래 끝 · 좌우 반지름
        u_top, v_top, _ = self.project(T_w_cam, [*self.light, self.seg_z[2] + self.seg_len / 2])
        u_bot, v_bot, _ = self.project(T_w_cam, [*self.light, self.seg_z[0] - self.seg_len / 2])
        half_w = K["fx"] * self.head_r / segs[1][2]
        if min(v_top, v_bot) < margin_px or max(v_top, v_bot) > H - 1 - margin_px:
            return None
        if min(s[0] for s in segs) - half_w < margin_px or max(s[0] for s in segs) + half_w > W - 1 - margin_px:
            return None
        return {"dist": float(d), "dist_nominal": float(dist), "phi_deg": math.degrees(phi) % 360,
                "bearing_deg": math.degrees(bearing), "cam_jitter_deg": [math.degrees(j) for j in jit],
                "robot_xy_yaw": [float(pos[0]), float(pos[1]), float(yaw)], "T_world_base_footprint": T_w_bf.tolist(),
                "T_world_cam": T_w_cam.tolist(), "min_clearance_m": round(float(self.clearance(ring)), 4)}

    def sample_start_poses(self, rng, start, n, pos_jit=0.02, yaw_jit=3.0):
        """출발 영역에서 무조건 n개 뽑음. 화면 밖·가림 표본도 버리지 않고 기록한다."""
        out = []
        nominal = float(np.linalg.norm(np.array(start[:2]) - self.light))
        for _ in range(n):
            x, y = np.array(start[:2]) + rng.uniform(-pos_jit, pos_jit, 2)
            yaw = math.radians(start[2] + rng.uniform(-yaw_jit, yaw_jit))
            T = planar_pose(x, y, yaw)
            jit = rng.uniform(-1.0, 1.0, 3)
            J = np.eye(4)
            J[:3, :3] = rpy_matrix(*np.radians(jit))
            C = T @ self.T_bf_cam @ J
            fp = self.footprint_world(T)
            ring = np.vstack([fp[i] + (fp[(i + 1) % len(fp)] - fp[i]) * a
                              for i in range(len(fp)) for a in np.linspace(0, 1, 15, endpoint=False)])
            segs = [self.project(C, [*self.light, z]) for z in self.seg_z]
            ends = [self.project(C, [*self.light, self.seg_z[i] + sign * self.seg_len / 2])
                    for i, sign in ((0, -1), (2, 1))]
            half = K["fx"] * self.head_r / segs[1][2]
            inside = all(z > 0 and half <= u < W - half and 0 <= v < H for u, v, z in segs + ends)
            out.append({"dist": float(np.linalg.norm([x-self.light[0], y-self.light[1]])),
                        "dist_nominal": nominal, "cam_jitter_deg": jit.tolist(),
                        "robot_xy_yaw": [float(x), float(y), yaw], "T_world_base_footprint": T.tolist(),
                        "T_world_cam": C.tolist(), "min_clearance_m": self.clearance(ring),
                        "geometry_check": {"all_segments_in_image": bool(inside),
                            "segments_px": [list(map(float, q)) for q in segs],
                            "los_blocked": any(self.los_blocked(C[:3, 3], [*self.light, z]) for z in self.seg_z)}})
        return out

    @staticmethod
    def _inside(b, p):
        return abs(p[0] - b[0]) <= b[2] and abs(p[1] - b[1]) <= b[3]

    def sample_poses(self, rng, dist, n, bearing_max_deg=15.0, pos_jit=0.02, ang_jit_deg=1.0, clearance=0.02,
                     margin_px=2.0, min_sep_deg=10.0, tries=20000):
        """조건을 만족하는 자세 n 개. 같은 거리 안에서는 방향 φ 가 min_sep_deg 넘게 다르게 (안 되면 겹침 허용)."""
        out = []
        for sep in (min_sep_deg, 0.0):
            for _ in range(tries):
                if len(out) >= n:
                    return out
                p = self.try_pose(rng, dist, bearing_max_deg, pos_jit, ang_jit_deg, clearance, margin_px)
                if p is None:
                    continue
                if all(abs((p["phi_deg"] - q["phi_deg"] + 180) % 360 - 180) > sep for q in out):
                    out.append(p)
        if len(out) < n:
            raise RuntimeError(f"거리 {dist} m 자세를 {n}개 못 뽑음({len(out)}개)")
        return out

    # ---------- 방해 퍽 ----------
    def sample_pucks(self, rng, pose, k_range=(1, 3), puck_cyl=(0.03, 0.06), puck_box=(0.03, 0.03, 0.06), tries=2000):
        """로봇 앞 바닥 0.25~0.6 m, 화면 안, 신호등 시선(2D)에서 0.08 m 넘게, 서로 0.06 m 넘게 떨어진 퍽."""
        T_w_bf = np.array(pose["T_world_base_footprint"])
        T_w_cam = np.array(pose["T_world_cam"])
        cam = T_w_cam[:3, 3]
        k = int(rng.integers(k_range[0], k_range[1] + 1))
        fp = self.footprint_world(T_w_bf)
        pucks = []
        for _ in range(tries):
            if len(pucks) >= k:
                break
            x, y = rng.uniform(0.25, 0.6), rng.uniform(-0.3, 0.3)
            pw = (T_w_bf @ np.array([x, y, 0.0, 1.0]))[:2]
            shape = "cylinder" if rng.random() < 0.5 else "cuboid"
            standing = bool(rng.random() < 0.5)
            half_h = (puck_cyl[1] if shape == "cylinder" else puck_box[2]) / 2 if standing else \
                (puck_cyl[0] if shape == "cylinder" else puck_box[0]) / 2
            u, v, zc = self.project(T_w_cam, [pw[0], pw[1], half_h])
            if not (20 <= u < W - 20 and 20 <= v < H - 20 and zc > 0):
                continue
            if self.clearance([pw]) < 0.04 or np.linalg.norm(pw - self.light) < 0.15:
                continue
            # 시선(카메라 xy → 신호등 xy) 선분까지 거리
            a, b = cam[:2], self.light
            t = np.clip(np.dot(pw - a, b - a) / np.dot(b - a, b - a), 0, 1)
            if np.linalg.norm(pw - (a + t * (b - a))) < 0.08:
                continue
            if any(np.linalg.norm(pw - np.array(q["xy"])) < 0.06 for q in pucks):
                continue
            if min(np.linalg.norm(fp - pw, axis=1)) < 0.05 or _point_in_poly(pw, fp):
                continue
            pucks.append({"xy": [float(pw[0]), float(pw[1])], "yaw": float(rng.uniform(0, 2 * math.pi)),
                          "color": "red" if rng.random() < 0.5 else "green", "shape": shape, "standing": standing,
                          "half_h": half_h, "px": [round(u, 1), round(v, 1)]})
        return pucks


def _point_in_poly(p, poly):
    inside = False
    for i in range(len(poly)):
        a, b = poly[i], poly[(i + 1) % len(poly)]
        if (a[1] > p[1]) != (b[1] > p[1]) and p[0] < (b[0] - a[0]) * (p[1] - a[1]) / (b[1] - a[1]) + a[0]:
            inside = not inside
    return inside
