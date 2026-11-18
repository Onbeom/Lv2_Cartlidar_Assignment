"""팔 카메라 자세를 실제 팔 자세(차체 위치 + 팔 관절)로 만들기 (모의, Isaac Sim 없이 numpy · scipy · trimesh).

쓰는 곳: render_r3.py · render_r4.py · render_r6.py · render_a1.py 의 팔 장면(2026-10-27 저녁부터, 로봇 · 집게 보이는 렌더).
규칙(사용자 결정 2026-10-27 저녁):
  - 장면 하나(연속 장 포함)에서 차체는 한 자리. 장마다 팔 관절만 풀어 카메라 위치 · 광축 방향을 맞춘다
    (팔이 5축이라 광축 둘레 회전은 팔이 정한다 → 영상 회전은 계획과 다를 수 있음, 정답은 실제 자세로 다시 계산).
  - 관절 한계 안, 충돌 없음: 차체 · 팔 · 집게 충돌 형상(URDF)과 월드 상자(벽판 · 기둥 · 선반 · 퍽 외접 상자 · 신호등)를
    margin(5 mm) 키워 겹치지 않음, 팔과 차체 · 팔끼리(이웃 링크 제외) 겹치지 않음, 팔이 바닥 위 5 mm 넘게.
  - 못 가면 같은 광축 방향으로 step(5 mm)씩 뒤로 물려 첫 가능한 자세(최대 backoff_max 0.05 m), 물린 거리 기록.
집게 자세 장면(realize_tcp): 장마다 집게 가운데(TCP = arm_link_5 (0, 0, −0.08295))를 주어진 점에, 집게를 수직 아래로
  (손목 roll 고정, 기본 0 = 카메라가 차체 쪽), 못 가면 TCP 를 5 mm 씩 위로 물림. 카메라 자세는 그 관절의 FK.
새 모델 261028(2026-10-31): URDF 관절 이름으로 모델을 고른다(PROFILES). joint1(z) · joint2~4(y) · joint5(x, 손목 roll),
  팔 카메라 arm_camera_optical_frame, TCP = tool_link(gripper_roll_link +x 0.139, 접근 = 링크 +x), 뿌리 base_link
  (base_footprint 는 그 자식이라 T_bf 에 뿌리 변환을 곱함). 고정 관절로 붙은 링크는 한 몸(Isaac 관절체와 같음: link2 ↔ 차체 이웃),
  팔 링크끼리 겹침은 보지 않음(사용자 결정 2026-10-29: 팔 충돌 상자가 실물보다 커서 팔끼리는 관절 한계로 다룸, robot_params.json).
차체 자리: 장면 첫 장 카메라(또는 TCP) 아래 점에서 반지름 0.16~0.44 m(0.04 간격) · 15° 간격 후보(차체가 그 점을 정면으로 봄) 중
  바닥 윤곽이 벽 · 선반 · 바닥 퍽에서 떨어진 것부터 풀어 보고, 모든 장을 가장 적게 물리는 자리를 쓴다.
"""
import math
import sys
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "sim_world"))
from robot_geom import Robot, arm_links, axis_R, box_depth, hull_depth, make_T  # noqa: E402
from urdf_chain import planar_pose  # noqa: E402

ARM_J = ["RMX52_Base_Yaw", "RMX52_Shoulder", "RMX52_Elbow", "RMX52_Wrist_Pitch", "RMX52_Wrist_Roll_Reversed"]
CAM = "camera_optical_frame"
TCP = np.array([0.0, 0.0, -0.08295, 1.0])
POS_TOL, AXIS_TOL = 0.001, math.radians(0.5)   # 풀이를 맞은 것으로 볼 오차
# 모델별 정보. 키 = 첫 팔 관절 이름(URDF 에 있으면 그 모델). up = (TCP 링크 열, 부호): 그 축 × 부호가 월드 위면 집게가 아래를 봄
PROFILES = {
    "RMX52_Base_Yaw": {"joints": ARM_J, "cam": CAM, "tcp_link": "arm_link_5", "tcp": TCP, "up": (2, 1.0), "grip_right": -1.0,
                       "fixed_one_body": False, "arm_self": True,
                       "seeds": [[0.0, 0.3, 0.8, 0.5, 0.0], [0.0, -0.3, 1.2, 0.9, 0.0], [0.0, 0.6, 0.3, 1.0, 0.0],
                                 [0.0, 0.0, 0.5, 1.4, 1.5], [0.0, 0.0, 0.5, 1.4, -1.5]]},
    "joint1": {"joints": ["joint1", "joint2", "joint3", "joint4", "joint5"], "cam": "arm_camera_optical_frame",
               "tcp_link": "gripper_roll_link", "tcp": np.array([0.139, 0.0, 0.0, 1.0]), "up": (0, -1.0), "grip_right": 1.0,
               "fixed_one_body": True, "arm_self": False,
               "seeds": [[0.0, 0.0, 0.0, 0.0, 0.0], [0.0, -1.35, 1.1497, 0.8094, 0.0], [0.0, 0.3, 0.3, 0.9, 0.0],
                         [0.0, -0.3, 0.6, 1.2, 0.0], [0.0, 0.0, 0.5, 1.4, 1.5], [0.0, 0.0, 0.5, 1.4, -1.5]]},
}


def _point_in_poly(p, poly):
    inside = False
    for i in range(len(poly)):
        a, b = poly[i], poly[(i + 1) % len(poly)]
        if (a[1] > p[1]) != (b[1] > p[1]) and p[0] < (b[0] - a[0]) * (p[1] - a[1]) / (b[1] - a[1]) + a[0]:
            inside = not inside
    return inside


class ArmRealizer:
    def __init__(self, urdf, pkg_dir, scene, layout, gripper=0.0, margin=0.005, step=0.005, backoff_max=0.05):
        """scene: r12_poses.Scene(shelf_top = 선반 높이 0.08 로 만든 것), gripper: gripper_left_joint 값(m)."""
        self.rb = Robot(urdf, pkg_dir)
        self.scene, self.margin, self.step, self.backoff_max = scene, margin, step, backoff_max
        self.gripper = gripper
        hit = [k for k in PROFILES if k in self.rb.joint_by_name]
        if len(hit) != 1:
            raise ValueError(f"팔 모델을 못 고름: {hit}")
        self.prof = PROFILES[hit[0]]
        self.arm_j, self.cam_frame = self.prof["joints"], self.prof["cam"]
        self.arm_links = arm_links(self.rb, self.arm_j[0])
        self.lim = [(self.rb.joint_by_name[j]["lower"], self.rb.joint_by_name[j]["upper"]) for j in self.arm_j]
        # base_footprint → URDF 뿌리 (이전 모델은 뿌리 = base_footprint 라 단위 행렬)
        self.T_bf_root = (np.eye(4) if self.rb.root == "base_footprint"
                          else np.linalg.inv(self.rb.link_poses({})["base_footprint"]))
        self.adjacent = self.rb.adjacent
        if self.prof["fixed_one_body"]:   # 고정 관절로 이어진 링크 무리를 한 몸으로 보고 움직이는 관절 양쪽 무리를 이웃으로
            def body(l):
                while l in self.rb.joints and self.rb.joints[l]["type"] == "fixed":
                    l = self.rb.joints[l]["parent"]
                return l
            group = {}
            for l in self.rb.links:
                group.setdefault(body(l), []).append(l)
            pairs = set()
            for d in self.rb.joints.values():
                if d["type"] != "fixed":
                    ga, gb = group[body(d["parent"])], group[body(d["child"])]
                    pairs |= {frozenset((a, b)) for a in ga for b in gb if a != b}
            for g in group.values():
                pairs |= {frozenset((a, b)) for a in g for b in g if a != b}
            self.adjacent = self.adjacent | pairs
        # base_footprint → 카메라 · TCP 링크 사슬
        self.chains = {}
        for end in (self.cam_frame, self.prof["tcp_link"]):
            chain, link = [], end
            while link != self.rb.root:
                d = self.rb.joints[link]
                chain.append(d)
                link = d["parent"]
            self.chains[end] = chain[::-1]
        # 월드 상자 (중심, 반 길이) — margin 만큼 키움
        boxes = []
        for cx, cy, hx, hy, top, kind in scene.boxes:
            boxes.append(((cx, cy, top / 2), (hx + margin, hy + margin, top / 2 + margin), kind))
        for p in layout["pucks"]:
            lo, hi = np.array(p["aabb"][0]), np.array(p["aabb"][1])
            boxes.append((tuple((lo + hi) / 2), tuple((hi - lo) / 2 + margin), p["name"]))
        sl = layout.get("signal_light")
        if sl:
            r = layout["assumptions"]["signal_light"]["head_diameter"] / 2
            boxes.append(((sl["x"], sl["y"], sl["height"] / 2), (r + margin, r + margin, sl["height"] / 2 + margin), "signal_light"))
        self.boxes = boxes
        self.box_c = np.array([b[0] for b in boxes])
        self.box_h = np.array([b[1] for b in boxes])
        # 차체 쪽(팔 · 집게가 아닌) 충돌 형상은 base_footprint 기준으로 고정
        self.body = [(l, V) for l, V in self.rb.world_shapes(self.q({}), self.T_bf_root) if l not in self.arm_links]

    # ---------- 기구학 ----------
    def q(self, v):
        d = {j: 0.0 for j in self.arm_j}
        d.update(dict(zip(self.arm_j, map(float, v))) if not isinstance(v, dict) else v)
        d["gripper_left_joint"] = self.gripper
        d["gripper_right_joint"] = self.prof["grip_right"] * self.gripper
        return d

    def cam(self, T_bf, v, end=None):
        """월드 카메라 광학 자세 (차체 T_bf, 팔 관절 v). end 로 다른 링크."""
        qq = dict(zip(self.arm_j, v))
        T = T_bf @ self.T_bf_root
        for d in self.chains[end or self.cam_frame]:
            T = T @ d["T0"]
            if d["type"] in ("revolute", "continuous"):
                T = T @ make_T(axis_R(d["axis"], qq.get(d["name"], 0.0)))
        return T

    def _res(self, T_bf, v, T_des):
        T = self.cam(T_bf, v)
        return np.r_[T[:3, 3] - T_des[:3, 3], 0.05 * (T[:3, 2] - T_des[:3, 2])]

    def solve_arm(self, T_bf, T_des, seeds):
        lo, hi = [l for l, _ in self.lim], [h for _, h in self.lim]
        best = None
        for s in seeds:
            r = least_squares(lambda v: self._res(T_bf, v, T_des), np.clip(s, lo, hi), bounds=(lo, hi), xtol=1e-10, ftol=1e-10)
            T = self.cam(T_bf, r.x)
            pe = np.linalg.norm(T[:3, 3] - T_des[:3, 3])
            ae = math.acos(np.clip(T[:3, 2] @ T_des[:3, 2], -1, 1))
            if pe <= POS_TOL and ae <= AXIS_TOL:
                yield r.x, pe, ae
            elif best is None or pe + 0.05 * ae < best:
                best = pe + 0.05 * ae

    def solve_tcp(self, T_bf, p, roll, seeds):
        """TCP 를 p 에, 집게가 수직 아래(PROFILES up 축이 월드 위), roll 고정. 맞은 해만 낸다."""
        lo, hi = [l for l, _ in self.lim[:4]], [h for _, h in self.lim[:4]]
        link, tcp = self.prof["tcp_link"], self.prof["tcp"]
        col, sign = self.prof["up"]

        def res(v4):
            T5 = self.cam(T_bf, [*v4, roll], link)
            return np.r_[(T5 @ tcp)[:3] - p, 0.05 * (sign * T5[:3, col] - [0, 0, 1.0])]
        for s in seeds:
            r = least_squares(res, np.clip(s[:4], lo, hi), bounds=(lo, hi), xtol=1e-10, ftol=1e-10)
            T5 = self.cam(T_bf, [*r.x, roll], link)
            if (np.linalg.norm((T5 @ tcp)[:3] - p) <= POS_TOL and
                    math.acos(np.clip(sign * T5[2, col], -1, 1)) <= AXIS_TOL):
                yield np.r_[r.x, roll]

    def frame_tcp(self, T_bf, p, roll, seeds):
        """TCP 해는 시작점에 민감해(한계 근처) 고정 무작위 시작점 12개를 더 쓴다."""
        if not hasattr(self, "_tcp_seeds"):
            r = np.random.default_rng(7)
            self._tcp_seeds = [r.uniform([l for l, _ in self.lim], [h for _, h in self.lim]) for _ in range(12)]
        seeds = list(seeds) + self._tcp_seeds
        b = 0.0
        while b <= self.backoff_max + 1e-9:
            for v in self.solve_tcp(T_bf, np.asarray(p, float) + [0, 0, b], roll, seeds):
                if not self.arm_hits(T_bf, v):
                    return v, round(b, 4), self.cam(T_bf, v)
            b += self.step
        return None

    # ---------- 충돌 ----------
    def body_hits(self, T_bf):
        """차체(팔 · 집게 제외)와 월드 상자 겹침 이름 목록."""
        return self._hits([(l, V @ T_bf[:3, :3].T + T_bf[:3, 3]) for l, V in self.body])

    def _hits(self, shapes):
        out = []
        for l, V in shapes:
            lo, hi = V.min(0), V.max(0)
            near = np.all((lo < self.box_c + self.box_h) & (hi > self.box_c - self.box_h), axis=1)
            for i in np.nonzero(near)[0]:
                if box_depth(V, self.box_c[i], self.box_h[i]) >= 0:
                    out.append((l, self.boxes[i][2]))
                    break
        return out

    def _body_world(self, T_bf):
        """차체 충돌 형상을 월드로 (차체 자리마다 한 번)."""
        key = T_bf.tobytes()
        if getattr(self, "_bw_key", None) != key:
            shapes = [(l, V @ T_bf[:3, :3].T + T_bf[:3, 3]) for l, V in self.body]
            self._bw = (shapes, np.array([V.min(0) for _, V in shapes]), np.array([V.max(0) for _, V in shapes]))
            self._bw_key = key
        return self._bw

    def arm_shapes(self, T_bf, v):
        poses = self.rb.link_poses(self.q(v), T_bf @ self.T_bf_root)
        out = []
        for link in self.arm_links:
            for T, V in self.rb.shapes.get(link, []):
                Wt = poses[link] @ T
                out.append((link, V @ Wt[:3, :3].T + Wt[:3, 3]))
        return out

    def arm_hits(self, T_bf, v):
        """팔 · 집게와 월드 · 차체 · 팔끼리 겹침, 바닥 아래. 빈 목록이면 통과."""
        arm = self.arm_shapes(T_bf, v)
        hits = [(l, "floor") for l, V in arm if V[:, 2].min() < self.margin]
        if hits:
            return hits
        hits = self._hits(arm)
        if hits:
            return hits
        body, blo, bhi = self._body_world(T_bf)
        for i, (la, Va) in enumerate(arm):
            lo_a, hi_a = Va.min(0), Va.max(0)
            near = np.all((lo_a < bhi) & (hi_a > blo), axis=1)
            for j in np.nonzero(near)[0]:
                lb = body[j][0]
                if frozenset((la, lb)) in self.adjacent:
                    continue
                if hull_depth(Va, body[j][1]) >= 0:
                    return [(la, lb)]
            for lb, Vb in (arm[i + 1:] if self.prof["arm_self"] else []):
                if la == lb or frozenset((la, lb)) in self.adjacent:
                    continue
                if np.any(lo_a > Vb.max(0)) or np.any(Vb.min(0) > hi_a):
                    continue
                if hull_depth(Va, Vb) >= 0:
                    return [(la, lb)]
        return []

    # ---------- 한 장 · 장면 ----------
    def frame(self, T_bf, T_des, seeds):
        """물리기 포함 한 장. 반환 (관절, 물린 거리, 실제 카메라 자세) 또는 None."""
        b = 0.0
        while b <= self.backoff_max + 1e-9:
            T = T_des.copy()
            T[:3, 3] -= b * T_des[:3, 2]
            for v, pe, ae in self.solve_arm(T_bf, T, seeds):
                if not self.arm_hits(T_bf, v):
                    return v, round(b, 4), self.cam(T_bf, v)
            b += self.step
        return None

    def base_candidates(self, T_des, floor_xy):
        p = T_des[:3, 3][:2]
        sc = self.scene
        x0, x1, y0, y1 = sc.bounds
        out = []
        for r in (0.16, 0.20, 0.24, 0.28, 0.32, 0.36, 0.40, 0.44):
            for a in np.radians(np.arange(0, 360, 15)):
                pos = p + r * np.array([math.cos(a), math.sin(a)])
                T_bf = planar_pose(pos[0], pos[1], a + math.pi)
                fp = sc.footprint_world(T_bf)
                if fp[:, 0].min() < x0 or fp[:, 0].max() > x1 or fp[:, 1].min() < y0 or fp[:, 1].max() > y1:
                    continue
                ring = np.vstack([fp[i] + (fp[(i + 1) % len(fp)] - fp[i]) * s for i in range(len(fp))
                                  for s in np.linspace(0, 1, 15, endpoint=False)])
                c = sc.clearance(ring)
                if c < self.margin or any(_point_in_poly(f, fp) or np.min(np.linalg.norm(ring - f, axis=1)) < 0.02
                                          for f in floor_xy):
                    continue
                out.append((r, c, T_bf))
        out.sort(key=lambda e: (abs(e[0] - 0.24), -e[1]))
        return [T for _, _, T in out]

    def realize_aim(self, T_des, floor_xy, p_world, v_px=145.0, fy=283.0, cy=240.0, iters=4, tol_px=5.0):
        """보기 자세(2026-10-27 밤 설계 변경): 내려다보면 손목 · 집게가 화면 아래 약 40 % 를 가리므로, 카메라를
        카메라 세로축(y) 방향으로 옮겨 p_world 가 화면 세로 v_px(위쪽)에 오게 한다. 광축 방향은 그대로.
        반환: realize 결과(+ "aim_v_px" 실제 세로 위치) 또는 None."""
        T = np.asarray(T_des, float).copy()
        r = None
        for _ in range(iters):
            r = self.realize([T], floor_xy)
            if r is None:
                return None
            Ta = np.array(r["frames"][0]["T_world_cam"])
            q = np.linalg.inv(Ta) @ np.append(np.asarray(p_world, float), 1.0)
            v = fy * q[1] / q[2] + cy
            r["aim_v_px"] = round(float(v), 1)
            if abs(v - v_px) <= tol_px:
                return r
            T = Ta.copy()
            T[:3, 3] = Ta[:3, 3] + Ta[:3, 1] * (v - v_px) * q[2] / fy
        return r

    def realize_tcp(self, p_list, floor_xy, roll=0.0, max_bases=12):
        """집게 자세 장면(TCP 점 목록) → realize 와 같은 꼴. 물림 = TCP 를 위로 올린 거리."""
        T0 = np.eye(4)
        T0[:3, 3] = p_list[0]
        return self.realize(p_list, floor_xy, max_bases, frame_fn=lambda T_bf, p, seeds: self.frame_tcp(T_bf, p, roll, seeds),
                            first=T0)

    def realize(self, T_list, floor_xy, max_bases=12, frame_fn=None, first=None):
        """장면(카메라 자세 목록) → {"T_world_base_footprint", "frames": [{q, backoff_m, T_world_cam}], "tried_bases"}.
        모든 장이 되면서 물린 거리 합이 가장 작은 차체 자리. 한 자리도 안 되면 None."""
        frame_fn = frame_fn or (lambda T_bf, T_des, seeds: self.frame(T_bf, np.asarray(T_des, float), seeds))
        arm_seeds = self.prof["seeds"]
        best, tried = None, 0
        self.last_reach = 0   # 실패 진단: 어느 차체 자리에서든 가장 멀리 간 장 수
        for T_bf in self.base_candidates(first if first is not None else np.asarray(T_list[0], float), floor_xy):
            if tried >= max_bases:
                break
            if self.body_hits(T_bf):
                continue
            tried += 1
            frames, prev = [], None
            for k, T_des in enumerate(T_list):
                seeds = ([prev] if prev is not None else []) + arm_seeds
                f = frame_fn(T_bf, T_des, seeds)
                if f is None:
                    break
                frames.append(f)
                prev = f[0]
            self.last_reach = max(self.last_reach, len(frames))
            if len(frames) < len(T_list):
                continue
            cost = sum(f[1] for f in frames)
            if best is None or cost < best[0]:
                best = (cost, T_bf, frames)
            if cost == 0.0:
                break
        if best is None:
            return None
        _, T_bf, frames = best
        return {"T_world_base_footprint": T_bf.tolist(), "tried_bases": tried,
                "frames": [{"q": dict(zip(self.arm_j, [round(float(x), 6) for x in v])), "backoff_m": b, "T_world_cam": T.tolist()}
                           for v, b, T in frames]}
