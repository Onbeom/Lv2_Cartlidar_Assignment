"""발제 맵 월드(USD) 만들기 — 주사위 기억 미션 v5(mission.pdf, 2026-10-28) 기준, 인지 파트 시험용.

좌표: 발제와 같음(왼쪽 아래 기둥 중심 원점, 바닥 Z = 0), 단위 m(발제 mm / 1000).
구성: 벽(B0 세션 초안 261028_1218/map_dice_draft.json)
      + 좁은 틈은 열어 둠 + 시작 박스 6곳(205 × 205 × 70, yaw 무작위) 위 퍽 2개씩(박스 중심 + 박스 yaw 로 돌린 (±40, 0) mm)
      + L1 · L3 박스(205, 비움) · 중앙 받침대(820 × 820 × 70, L2 = 북쪽) + 장소 태그 12 · 13 · 14(접근 면, 중심 높이 35 mm)
      + 신호등 (80, 950, 0) 세로 · +X + 바코드 64.8 × 17.7 mm 한 장((80, 650) 언저리 벽, 높이 무작위)
      + 출발 구역 테이프 + 로봇(261027_1711, 출발 자세 (734, 950) −X, 접은 팔 S0) + 조명.
퍽: 12개, 빨강 · 초록 수 무작위(파랑 없음), 원기둥은 항상 서 있고 직육면체는 누울 수 있음(사용자 2026-10-28).
신호: 퍽 구성으로 풀 수 있는 R · Y · G 횟수(각 1~6, 합 12) 중 하나를 뽑고 순서 무작위. 1 s 켜짐 / 1 s 소등 × 12 → 3 s 소등.
같은 seed 면 같은 파일. 이전 판(B 경기장 위쪽 반 · 선반)은 build_task_world.py 그대로.

실행 (PC): ~/isaacsim-venv/bin/python -I build_mission_world.py --seed 1
결과: worlds/dice_mission/seed_<seed>/task_world.usda, layout.json, data/maze_dice_mission.json(벽 · 기둥, 렌더 도구 --drawing)
"""
import argparse
import json
import math
import sys
from pathlib import Path

import cv2
import numpy as np
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics, UsdShade

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import code128  # noqa: E402
from build_task_world import (A as A_OLD, COLORS, SIGNAL_SEGMENTS, Builder, Ry, Rz, quad, quat_from_R, r6,  # noqa: E402
                              sha256)
from robot_geom import Robot, box_depth  # noqa: E402

TOP = HERE.parents[1]
MODEL = "261027_1711"
ROBOT_USD = TOP / f"{MODEL}/usd/waffle_4wd_{MODEL}/waffle_4wd/waffle_4wd.usda"
URDF_PKG = TOP / f"{MODEL}/urdf/waffle_4wd_description_{MODEL}"
DRAFT = TOP / "261028_1218/map_dice_draft.json"
DRAWING = HERE / "data/maze_dice_mission.json"
S0 = {"RMX52_Base_Yaw": 0.0, "RMX52_Shoulder": 1.4, "RMX52_Elbow": -1.0, "RMX52_Wrist_Pitch": -1.3,
      "RMX52_Wrist_Roll_Reversed": 0.0, "gripper_left_joint": 0.0}   # r12_poses.S0 와 같음

# ---- 값 (단위 m). 발제 = mission.pdf v5, 사용자 = 2026-10-28 답, 가정 = 확인 전 ----
A = {
    "panel_height": A_OLD["panel_height"], "panel_thickness": A_OLD["panel_thickness"],   # 실측(이전과 같음)
    "post_width": 0.036, "post_thickness": 0.0095,   # 출력 연결기둥(이전과 같음). 3030 은 구분 안 함(가정)
    "post_top": A_OLD["post_top"], "post_body_top": A_OLD["post_body_top"], "post_head_diameter": A_OLD["post_head_diameter"],
    "post_foot_height": A_OLD["post_foot_height"],
    "panel_gap": 0.014,              # 가정: 판 길이 = 기둥 간격 − 14 mm(이전 734 → 720 과 같은 비율)
    "narrow_gaps": "초안의 307 mm 틈 4곳((300, 4200) 포함)은 막지 않고 빈 곳 그대로 — 차체 폭 때문에 못 지나가는 것은 주행 쪽이 판단(사용자 정정 2026-10-28)",
    "start_boxes": [[1.468, 0.950], [3.670, 3.152], [2.936, 5.354], [0.734, 5.354], [2.202, 5.354], [3.670, 3.886]],  # 발제
    "box_small": [0.205, 0.205, 0.070],   # 발제
    "start_box_yaw_deg": 30.0,       # 가정(사용자 확인 전): 시작 박스 yaw U(−30, 30)°. 사용자: 돌아가 놓일 수 있음
    "place_boxes": {"L1": [3.670, 0.950], "L3": [3.670, 5.354]},   # 발제, 비움, yaw 0(가정)
    "center_stand": {"center": [2.202, 3.500], "size": [0.820, 0.820, 0.070]},   # 크기 발제, 중심 y 는 발제 그림에서 읽음(가정)
    "L": {"L1": [3.670, 0.950], "L2": [2.202, 3.700], "L3": [3.670, 5.354]},   # 발제
    "place_tags": {"L1": {"id": 12, "normal": [-1, 0]}, "L2": {"id": 13, "normal": [0, 1]}, "L3": {"id": 14, "normal": [0, -1]}},
    #   접근 면(발제 그림의 접근 점선: L1 −X 쪽, L2 북쪽 +Y, L3 남쪽 −Y). L2 태그 = 중앙 받침대 북쪽 면 x 2.202
    "place_tag_size": 0.048, "place_tag_z": 0.035,   # 크기 이전 결정(흰 여백 포함 60), 중심 높이 발제
    "puck_dx": 0.040,                # 발제: 박스 중심 X ± 40 mm
    "puck_cuboid": A_OLD["puck_cuboid"], "puck_cylinder": A_OLD["puck_cylinder"], "puck_mass": A_OLD["puck_mass"],
    "puck_tag_size": A_OLD["puck_tag_size"],
    "puck_tag_faces": "원기둥 윗면 · 아랫면, 직육면체 6면 모두(옆면 30 × 60 도 16 mm, 같은 번호) — 사용자 2026-10-28",
    "cuboid_lying_prob": 0.5,        # 가정. 원기둥은 항상 서 있음(사용자)
    "red_count": [0, 12],            # 발제: 빨강 · 초록 수 무작위(합 12). 풀 수 있는 신호가 있을 때만
    "signal": {"base": [0.080, 0.950], "yaw_deg": 0.0, "t_on": 1.0, "t_off": 1.0, "t_term": 3.0, "repeat": "무한"},
    "signal_light": A_OLD["signal_light"],   # 실측 치수(이전과 같음)
    "barcodes": ["P2-01", "P2-02", "P2-03", "P2-04"],   # 발제
    "barcode_label": [0.0648, 0.0177],       # 사용자 실측
    "barcode_layout_mm": {"bar_top": 1.5, "bar_h": 11.5, "text": "아래"},   # 가정(라벨 안 배치 모름)
    "barcode_wall": {"x_face": 0.0015, "y": [0.550, 0.750], "z": [0.10, 0.38]},   # 사용자: (80, 650) 언저리 벽(y ±0.10), 높이 다양
    "start_center": [0.734, 0.950], "start_yaw_deg": 180.0,   # 발제: 출발 중심에서 −X 관측. base_footprint 를 둠(가정)
    "start_tape_outer": 0.60, "start_tape_width": 0.03,      # 발제 그림에서 읽음(약 600 × 600), 폭 가정
    "ground_friction": A_OLD["ground_friction"],
    "lights": "돔 300~600 + 천장 사각 6개(x 1.1 · 3.3, y 1.05 · 3.15 · 5.25, 0 ~ 20000) — level 0 ~ 1 선형, 이전과 같은 세기",
}
LIGHT_RANGE = {"dome": (300.0, 600.0), "rect": (0.0, 20000.0)}
CEILING = [(1.1, 1.05), (3.3, 1.05), (1.1, 3.15), (3.3, 3.15), (1.1, 5.25), (3.3, 5.25)]


# ---------------- 벽 · 기둥 ----------------
def make_drawing():
    """초안 벽(mm) → 판(기둥 간격마다) · 기둥. 렌더 도구(r12_poses.Scene)가 읽는 꼴."""
    d = json.loads(DRAFT.read_text())
    segs = [(w["p1"], w["p2"], w["kind"]) for w in d["walls"]]   # 좁은 틈은 막지 않음(사용자 정정 2026-10-28)
    posts, panels = {}, []

    def post(x, y):
        k = (round(x, 4), round(y, 4))
        if k not in posts:
            posts[k] = {"id": len(posts), "x": k[0], "y": k[1], "type": "printed"}
        return posts[k]["id"]
    for p1, p2, kind in segs:
        p1, p2 = np.array(p1, float) / 1000, np.array(p2, float) / 1000
        axis = "x" if abs(p2[1] - p1[1]) < 1e-9 else "y"
        L = float(np.linalg.norm(p2 - p1))
        step = 0.734 if (kind != "outer" or axis == "x") else 0.394
        n = max(1, int(round(L / step)))
        for i in range(n):
            a, b = p1 + (p2 - p1) * i / n, p1 + (p2 - p1) * (i + 1) / n
            ia, ib = post(*a), post(*b)
            panels.append({"id": len(panels), "kind": kind, "axis": axis, "length": r6(L / n - A["panel_gap"]),
                           "center": [r6((a[0] + b[0]) / 2), r6((a[1] + b[1]) / 2)], "a": ia, "b": ib, "shared": False})
    out = {"description": "주사위 기억 미션 v5 맵(발제 2쪽 그림 → B0 초안 map_dice_draft.json). 단위 m, 왼쪽 아래 기둥 중심 원점.",
           "sources": {"draft": str(DRAFT), "draft_sha256": sha256(DRAFT)},
           "grid": {"x": [0.0, 4.404], "y": [0.0, 6.304]}, "posts": list(posts.values()), "panels": panels}
    DRAWING.write_text(json.dumps(out, indent=1, ensure_ascii=False))
    return out


def post_half(drawing):
    axes = {}
    for pn in drawing["panels"]:
        for k in (pn["a"], pn["b"]):
            axes.setdefault(k, set()).add(pn["axis"])
    w, t = A["post_width"] / 2, A["post_thickness"] / 2
    return {po["id"]: ((w, t) if axes[po["id"]] == {"x"} else (t, w) if axes[po["id"]] == {"y"} else (w, w))
            for po in drawing["posts"]}


# ---------------- 배치 ----------------
def feasible_signals(pucks):
    """퍽 구성으로 풀 수 있는 (R, Y, G, Y 모양) 목록(발제 4장)."""
    red = [p for p in pucks if p["color"] == "red"]
    grn = [p for p in pucks if p["color"] == "green"]
    out = []
    for R in range(1, 7):
        for G in range(1, 7):
            Y = 12 - R - G
            if not 1 <= Y <= 6:
                continue
            yr, yg = len(red) - R, len(grn) - G
            if yr < 0 or yg < 0:
                continue
            for s in ("cylinder", "cuboid"):
                if sum(p["shape"] == s for p in red) >= yr and sum(p["shape"] == s for p in grn) >= yg:
                    out.append((R, Y, G, s))
    return out


def plan(seed):
    rng = np.random.default_rng(seed)
    # 퍽 색 · 모양: 풀 수 있는 신호가 하나라도 있을 때까지 다시
    while True:
        n_red = int(rng.integers(A["red_count"][0], A["red_count"][1] + 1))
        colors = ["red"] * n_red + ["green"] * (12 - n_red)
        shapes = [["cylinder", "cuboid"][int(rng.integers(2))] for _ in range(12)]
        specs = [{"color": c, "shape": s} for c, s in zip(colors, shapes)]
        cand = feasible_signals(specs)
        if cand:
            break
    R, Y, G, yshape = cand[int(rng.integers(len(cand)))]
    seq = list("R" * R + "Y" * Y + "G" * G)
    seq = "".join(seq[i] for i in rng.permutation(12))
    order = rng.permutation(12)
    specs = [specs[i] for i in order]
    tag_ids = [int(t) for t in rng.permutation(12)]
    # 시작 박스 · 퍽
    bx, by, bz = A["box_small"]
    boxes, pucks = [], []
    for bi, c in enumerate(A["start_boxes"]):
        yaw = math.radians(float(rng.uniform(-A["start_box_yaw_deg"], A["start_box_yaw_deg"])))
        boxes.append({"name": f"start_box_{bi + 1}", "kind": "start", "center": [c[0], c[1], bz / 2], "size": A["box_small"],
                      "yaw": r6(yaw)})
        for k, side in enumerate((-1, 1)):
            n = 2 * bi + k
            sp = specs[n]
            xy = np.array(c) + Rz(yaw)[:2, :2] @ np.array([side * A["puck_dx"], 0.0])
            pyaw = float(rng.uniform(-math.pi, math.pi))
            lying = sp["shape"] == "cuboid" and rng.random() < A["cuboid_lying_prob"]
            if lying:
                Rm = Rz(pyaw) @ Ry(math.pi / 2)
                half_z = A["puck_cuboid"][0] / 2
            else:
                Rm = Rz(pyaw) @ (np.diag([1.0, -1.0, -1.0]) if rng.random() < 0.5 else np.eye(3))   # 뒤집힘(윗면 · 아랫면 태그)
                half_z = A["puck_cuboid"][2] / 2
            s = (np.array(A["puck_cuboid"]) if sp["shape"] == "cuboid" else
                 np.array([A["puck_cylinder"][0]] * 2 + [A["puck_cylinder"][1]])) / 2
            corners = np.array([[x, y, z] for x in (-s[0], s[0]) for y in (-s[1], s[1]) for z in (-s[2], s[2])]) @ Rm.T
            ctr = np.array([xy[0], xy[1], bz + half_z])
            h = np.abs(corners).max(0)
            pucks.append({"name": f"puck_{tag_ids[n]:02d}", "tag_id": tag_ids[n], "color": sp["color"], "shape": sp["shape"],
                          "standing": not lying, "box": bi + 1, "side": side, "location": f"start_box_{bi + 1}",
                          "center": [r6(v) for v in ctr], "R": [[r6(v) for v in row] for row in Rm],
                          "half_xy": [r6(v) for v in h[:2]],
                          "aabb": [[r6(v) for v in ctr - h], [r6(v) for v in ctr + h]]})
    for name, c in A["place_boxes"].items():
        boxes.append({"name": f"place_{name}", "kind": "place", "center": [c[0], c[1], bz / 2], "size": A["box_small"], "yaw": 0.0})
    cs = A["center_stand"]
    boxes.append({"name": "center_stand", "kind": "stand", "center": [*cs["center"], cs["size"][2] / 2], "size": cs["size"], "yaw": 0.0})
    # 장소 태그: 접근 면 가운데, 중심 높이 35 mm, 면에서 0.5 mm
    place_tags = []
    for L, t in A["place_tags"].items():
        b = next(x for x in boxes if x["name"] == ("center_stand" if L == "L2" else f"place_{L}"))
        nrm = np.array([*t["normal"], 0.0])
        half = np.array(b["size"][:2]) / 2
        if L == "L2":
            xy = np.array([A["L"]["L2"][0], b["center"][1] + half[1]])
        else:
            xy = np.array(b["center"][:2]) + nrm[:2] * half
        c = np.array([xy[0], xy[1], A["place_tag_z"]]) + nrm * 0.0005
        place_tags.append({"id": t["id"], "place": L, "box": b["name"], "normal": nrm.tolist(), "center": [r6(v) for v in c],
                           "size": A["place_tag_size"]})
    # 바코드: 왼쪽 바깥 벽 안쪽 면(+X 를 봄), 기둥에 걸치지 않게
    drawing = json.loads(DRAWING.read_text())
    ph = post_half(drawing)
    bw, bh = A["barcode_label"]
    bwall = A["barcode_wall"]
    wall_posts = [po["y"] for po in drawing["posts"] if abs(po["x"]) < 1e-6]
    for _ in range(1000):
        y = float(rng.uniform(*bwall["y"]))
        if all(abs(y - py) > bw / 2 + ph[0][1] + 0.002 for py in wall_posts):
            break
    else:
        raise RuntimeError("바코드 자리 못 찾음")
    z = float(rng.uniform(bwall["z"][0], bwall["z"][1] - bh / 2 - 0.001))   # 라벨이 벽 위로 나가지 않게
    text = A["barcodes"][int(rng.integers(4))]
    barcode = {"text": text, "where": "wall", "center": [r6(bwall["x_face"] + 0.0005), r6(y), r6(z)], "normal": [1.0, 0.0, 0.0],
               "u": [0.0, 1.0, 0.0], "size": A["barcode_label"]}
    sl = A["signal_light"]
    height = sl["first_led_z"] + 3 * sl["segment_length"] + sl["cap"]
    sg = A["signal"]
    light = {"x": sg["base"][0], "y": sg["base"][1], "yaw": r6(math.radians(sg["yaw_deg"])), "height": r6(height),
             "sequence": seq, "red": R, "yellow": Y, "green": G, "yellow_shape": yshape, "feasible": [list(c) for c in cand],
             "timing": {"on": sg["t_on"], "off": sg["t_off"], "term": sg["t_term"]}}
    robot = {"x": A["start_center"][0], "y": A["start_center"][1], "yaw": r6(math.radians(A["start_yaw_deg"])), "q": S0}
    return {"boxes": boxes, "pucks": pucks, "place_tags": place_tags, "barcodes": [barcode], "signal_light": light,
            "robot": robot, "start_box": {"center": A["start_center"], "outer": A["start_tape_outer"], "tape_width": A["start_tape_width"]},
            # 렌더 도구(r12_poses.Scene · arm_pose)는 'shelves' 를 장애물 상자로 읽음 → 박스를 yaw 반영 외접 상자로 넣음
            "shelves": [{"name": b["name"], "center": b["center"],
                         "size": [r6(abs(math.cos(b["yaw"])) * b["size"][0] + abs(math.sin(b["yaw"])) * b["size"][1]),
                                  r6(abs(math.sin(b["yaw"])) * b["size"][0] + abs(math.cos(b["yaw"])) * b["size"][1]),
                                  b["size"][2]]} for b in boxes]}


def check(p, drawing, robot):
    """정적 확인: 퍽이 박스 윗면 안 · 서로 안 겹침, 로봇 출발 자세가 벽 · 박스 · 신호등과 안 겹침."""
    errs = []
    bz = A["box_small"][2]
    for pk in p["pucks"]:
        b = next(x for x in p["boxes"] if x["name"] == pk["location"])
        Rb = Rz(-b["yaw"])[:2, :2]
        lo, hi = np.array(pk["aabb"][0]), np.array(pk["aabb"][1])
        for cx in (lo[0], hi[0]):
            for cy in (lo[1], hi[1]):
                q = Rb @ (np.array([cx, cy]) - b["center"][:2])
                if np.any(np.abs(q) > np.array(b["size"][:2]) / 2 + 0.006):   # 외접 상자 모서리라 조금 여유
                    errs.append(f"{pk['name']} 박스 밖")
        if abs(lo[2] - bz) > 1e-6:
            errs.append(f"{pk['name']} 바닥 {lo[2]}")
    for i, a in enumerate(p["pucks"]):
        for b in p["pucks"][i + 1:]:
            if np.all(np.array(a["aabb"][0]) < np.array(b["aabb"][1])) and np.all(np.array(b["aabb"][0]) < np.array(a["aabb"][1])):
                errs.append(f"{a['name']}–{b['name']} 겹침")
    H, T = A["panel_height"], A["panel_thickness"]
    obst = [((pn["center"][0], pn["center"][1], H / 2), np.array([pn["length"] / 2, T / 2, H / 2] if pn["axis"] == "x" else
                                                                 [T / 2, pn["length"] / 2, H / 2]), f"panel_{pn['id']}")
            for pn in drawing["panels"]]
    ph = post_half(drawing)
    obst += [((po["x"], po["y"], A["post_top"] / 2), np.array([*ph[po["id"]], A["post_top"] / 2]), f"post_{po['id']}")
             for po in drawing["posts"]]
    obst += [(tuple(s["center"]), np.array(s["size"]) / 2, s["name"]) for s in p["shelves"]]
    lt = p["signal_light"]
    r = A["signal_light"]["head_diameter"] / 2
    obst.append(((lt["x"], lt["y"], lt["height"] / 2), np.array([r, r, lt["height"] / 2]), "signal_light"))
    rp = p["robot"]
    Tr = np.eye(4)
    Tr[:3, :3] = Rz(rp["yaw"])
    Tr[:2, 3] = (rp["x"], rp["y"])
    near = 1e9
    for link, V in robot.world_shapes(rp["q"], Tr):
        for c, h, name in obst:
            d = box_depth(V, c, h)
            if d >= 0:
                errs.append(f"로봇 {link}–{name} 겹침")
            near = min(near, -d)
    return errs, r6(near)


# ---------------- USD ----------------
def barcode_png(text, path, px_per_mm=24):
    lw, lh = A["barcode_label"]
    lay = A["barcode_layout_mm"]
    w, h = int(round(lw * 1000 * px_per_mm)), int(round(lh * 1000 * px_per_mm))
    canvas = np.full((h, w), 255, np.uint8)
    mods = code128.modules(text)
    edges = np.round(np.linspace(0, w, len(mods) + 1)).astype(int)
    y0, y1 = int(lay["bar_top"] * px_per_mm), int((lay["bar_top"] + lay["bar_h"]) * px_per_mm)
    for k, m in enumerate(mods):
        if m:
            canvas[y0:y1, edges[k]:edges[k + 1]] = 0
    font, scale, th = cv2.FONT_HERSHEY_SIMPLEX, 2.0, 4
    (tw, _), _ = cv2.getTextSize(text, font, scale, th)
    cv2.putText(canvas, text, ((w - tw) // 2, h - int(0.8 * px_per_mm)), font, scale, 0, th, cv2.LINE_AA)
    cv2.imwrite(str(path), canvas)


def write_usd(path, drawing, p, level):
    b = Builder(path)
    st = b.stage
    UsdPhysics.Scene.Define(st, "/World/physicsScene").CreateGravityMagnitudeAttr(9.81)
    m_carpet = b.material("carpet_gray", (0.30, 0.31, 0.32), roughness=0.95)
    m_panel = b.material("acrylic_bluegray", (0.30, 0.35, 0.45), roughness=0.15)
    m_print = b.material("printed_blue", (0.10, 0.20, 0.75), roughness=0.5)
    m_box = b.material("box_kraft", (0.62, 0.52, 0.38), roughness=0.85)
    m_tape = b.material("tape_white", (0.92, 0.92, 0.92), roughness=0.7)
    m_black = b.material("plastic_black", (0.05, 0.05, 0.05), roughness=0.5)
    m_floor_phys = b.physics_material("floor_contact", *A["ground_friction"])
    UsdGeom.Xform.Define(st, "/World/Ground")
    plane = UsdGeom.Plane.Define(st, "/World/Ground/CollisionPlane")
    plane.CreateAxisAttr("Z")
    plane.CreateWidthAttr(12.0)
    plane.CreateLengthAttr(12.0)
    plane.CreateExtentAttr([Gf.Vec3f(-6, -6, 0), Gf.Vec3f(6, 6, 0)])
    plane.AddTranslateOp().Set(Gf.Vec3d(2.202, 3.152, 0))
    UsdPhysics.CollisionAPI.Apply(plane.GetPrim())
    b.bind(plane.GetPrim(), m_carpet)
    b.bind(plane.GetPrim(), m_floor_phys, "physics")
    H, T = A["panel_height"], A["panel_thickness"]
    UsdGeom.Xform.Define(st, "/World/Maze")
    for pn in drawing["panels"]:
        L = pn["length"]
        size = (L, T, H) if pn["axis"] == "x" else (T, L, H)
        prim = b.box(f"/World/Maze/panel_{pn['id']:02d}", (pn["center"][0], pn["center"][1], H / 2), size, m_panel).GetPrim()
        prim.CreateAttribute("maze:kind", Sdf.ValueTypeNames.String).Set(pn["kind"])
    ph = post_half(drawing)
    hz, hf = A["post_top"] - A["post_body_top"], A["post_foot_height"]
    for po in drawing["posts"]:
        base = f"/World/Maze/post_{po['id']:02d}"
        UsdGeom.Xform.Define(st, base)
        h = ph[po["id"]]
        b.box(base + "/body", (po["x"], po["y"], A["post_body_top"] / 2), (2 * h[0], 2 * h[1], A["post_body_top"]), m_print)
        b.cylinder(base + "/head", (po["x"], po["y"], A["post_body_top"] + hz / 2), A["post_head_diameter"] / 2, hz, m_print)
        b.cylinder(base + "/foot", (po["x"], po["y"], hf / 2), A["post_head_diameter"] / 2, hf, m_print)
    sb = p["start_box"]
    o, w, th = sb["outer"], sb["tape_width"], 0.001
    c0 = np.array(sb["center"])
    UsdGeom.Xform.Define(st, "/World/StartBox")
    for name, dc, size in (("tape_s", (0, -(o - w) / 2), (o, w)), ("tape_n", (0, (o - w) / 2), (o, w)),
                           ("tape_w", (-(o - w) / 2, 0), (w, o - 2 * w)), ("tape_e", ((o - w) / 2, 0), (w, o - 2 * w))):
        b.box(f"/World/StartBox/{name}", (c0[0] + dc[0], c0[1] + dc[1], th / 2), (size[0], size[1], th), m_tape, collide=False)
    # 박스(시작 6 · L1 · L3 · 중앙 받침대). 렌더 도구 호환을 위해 경로는 /World/Shelves
    UsdGeom.Xform.Define(st, "/World/Shelves")
    for bx in p["boxes"]:
        g = b.box(f"/World/Shelves/{bx['name']}", (0, 0, 0), bx["size"], m_box)
        xf = UsdGeom.Xformable(g)
        xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(Gf.Vec3d(*map(r6, bx["center"])))
        xf.AddRotateZOp().Set(r6(math.degrees(bx["yaw"])))
        xf.AddScaleOp().Set(Gf.Vec3f(*map(r6, bx["size"])))
        g.GetPrim().CreateAttribute("box:kind", Sdf.ValueTypeNames.String).Set(bx["kind"])
    UsdGeom.Xform.Define(st, "/World/PlaceTags")
    for t in p["place_tags"]:
        m_t = b.material(f"tag_{t['id']:02d}", (1, 1, 1), roughness=0.8, texture=HERE / f"textures/tag36h11_{t['id']:02d}.png")
        nrm = np.array(t["normal"])
        half = t["size"] * 10 / 8 / 2
        pts, faces, stc = quad(t["center"], np.cross([0, 0, 1.0], nrm), [0, 0, 1.0], half, half)
        mm = b.mesh(f"/World/PlaceTags/tag_{t['id']:02d}", pts, faces, stc, m_t)
        mm.GetPrim().CreateAttribute("tag:id", Sdf.ValueTypeNames.Int).Set(int(t["id"]))
        mm.GetPrim().CreateAttribute("tag:size", Sdf.ValueTypeNames.Double).Set(t["size"])
    UsdGeom.Xform.Define(st, "/World/Barcodes")
    for k, bc in enumerate(p["barcodes"]):
        m_b = b.material(f"barcode_{bc['text'].replace('-', '_')}", (1, 1, 1), roughness=0.8,
                         texture=HERE / f"textures/barcode648_{bc['text']}.png")
        nrm, u = np.array(bc["normal"]), np.array(bc["u"])
        pts, faces, stc = quad(bc["center"], u, np.cross(nrm, u), bc["size"][0] / 2, bc["size"][1] / 2)
        mm = b.mesh(f"/World/Barcodes/barcode_{k}", pts, faces, stc, m_b)
        mm.GetPrim().CreateAttribute("barcode:text", Sdf.ValueTypeNames.String).Set(bc["text"])
    UsdGeom.Xform.Define(st, "/World/Pucks")
    lab = A["puck_tag_size"] * 10 / 8 / 2
    for pk in p["pucks"]:
        base = f"/World/Pucks/{pk['name']}"
        prim = b.xform(base, pk["center"], quat_from_R(pk["R"])).GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(A["puck_mass"][pk["shape"]])
        for k, v in (("puck:color", pk["color"]), ("puck:shape", pk["shape"]), ("puck:location", pk["location"])):
            prim.CreateAttribute(k, Sdf.ValueTypeNames.String).Set(v)
        prim.CreateAttribute("puck:tag_id", Sdf.ValueTypeNames.Int).Set(pk["tag_id"])
        prim.CreateAttribute("puck:standing", Sdf.ValueTypeNames.Bool).Set(pk["standing"])
        m_body = b.material(f"puck_{pk['color']}", COLORS[pk["color"]], roughness=0.85)
        m_tag = b.material(f"tag_{pk['tag_id']:02d}", (1, 1, 1), roughness=0.8, texture=HERE / f"textures/tag36h11_{pk['tag_id']:02d}.png")
        if pk["shape"] == "cuboid":
            b.box(base + "/body", (0, 0, 0), A["puck_cuboid"], m_body)
            hz_ = A["puck_cuboid"][2] / 2
        else:
            b.cylinder(base + "/body", (0, 0, 0), A["puck_cylinder"][0] / 2, A["puck_cylinder"][1], m_body)
            hz_ = A["puck_cylinder"][1] / 2
        faces = [("top", (0, 0, hz_ + 2e-4), (1, 0, 0), (0, 1, 0)), ("bottom", (0, 0, -hz_ - 2e-4), (1, 0, 0), (0, -1, 0))]
        if pk["shape"] == "cuboid":   # 옆면 4개(가운데, 태그 위쪽 = 퍽 +z)
            e = A["puck_cuboid"][0] / 2 + 2e-4
            faces += [("px", (e, 0, 0), (0, 1, 0), (0, 0, 1)), ("nx", (-e, 0, 0), (0, -1, 0), (0, 0, 1)),
                      ("py", (0, e, 0), (-1, 0, 0), (0, 0, 1)), ("ny", (0, -e, 0), (1, 0, 0), (0, 0, 1))]
        for fn, c, u, v in faces:
            pts, fc, stc = quad(c, u, v, lab, lab)
            b.mesh(f"{base}/label_{fn}", pts, fc, stc, m_tag)
    lt = p["signal_light"]
    sl = A["signal_light"]
    base = "/World/SignalLight"
    xf = UsdGeom.Xform.Define(st, base)
    xf.AddTranslateOp().Set(Gf.Vec3d(lt["x"], lt["y"], 0.0))
    xf.AddRotateZOp().Set(r6(math.degrees(lt["yaw"])))
    pr = xf.GetPrim()
    pr.CreateAttribute("signal:sequence", Sdf.ValueTypeNames.String).Set(lt["sequence"])
    for k in ("on", "off", "term"):
        pr.CreateAttribute(f"signal:t_{k}", Sdf.ValueTypeNames.Double).Set(lt["timing"][k])
    pr.CreateAttribute("signal:yellow_shape", Sdf.ValueTypeNames.String).Set(lt["yellow_shape"])
    b.cylinder(base + "/base", (0, 0, sl["base_height"] / 2), sl["base_diameter"] / 2, sl["base_height"], m_black)
    pole_h = sl["first_led_z"] - sl["base_height"]
    b.cylinder(base + "/pole", (0, 0, sl["base_height"] + pole_h / 2), sl["pole_diameter"] / 2, pole_h, m_black)
    z = sl["first_led_z"]
    for name, off_col, on_col in SIGNAL_SEGMENTS:
        m = b.material(f"signal_{name}", off_col, roughness=0.3)
        UsdShade.Shader(st.GetPrimAtPath(f"/World/Looks/signal_{name}/PBR")).CreateInput(
            "emissiveColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0, 0, 0))
        seg = b.cylinder(f"{base}/{name}", (0, 0, z + sl["segment_length"] / 2), sl["head_diameter"] / 2, sl["segment_length"], m).GetPrim()
        seg.CreateAttribute("signal:on_color", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*on_col))
        z += sl["segment_length"]
    b.cylinder(base + "/cap", (0, 0, z + sl["cap"] / 2), sl["head_diameter"] / 2, sl["cap"], m_black)
    # 로봇: 렌더 전용 참조(물리 drive 값은 넣지 않음 — 물리 시험은 sim 세션 월드를 씀)
    rp = p["robot"]
    rprim = st.DefinePrim("/World/waffle_4wd", "Xform")
    rprim.GetReferences().AddReference(b.rel(ROBOT_USD))
    rprim.GetVariantSets().GetVariantSet("Physics").SetVariantSelection("physx")
    rx = UsdGeom.Xformable(rprim)
    rx.AddTranslateOp().Set(Gf.Vec3d(rp["x"], rp["y"], 0.0))
    rx.AddRotateZOp().Set(r6(math.degrees(rp["yaw"])))
    dome = LIGHT_RANGE["dome"][0] + level * (LIGHT_RANGE["dome"][1] - LIGHT_RANGE["dome"][0])
    rect = LIGHT_RANGE["rect"][0] + level * (LIGHT_RANGE["rect"][1] - LIGHT_RANGE["rect"][0])
    UsdLux.DomeLight.Define(st, "/World/Lights/dome").CreateIntensityAttr(r6(dome))
    for i, (lx, ly) in enumerate(CEILING):   # 렌더 도구가 세기를 바꾸므로 0 이어도 만든다
        rl = UsdLux.RectLight.Define(st, f"/World/Lights/ceiling_{i}")
        rl.CreateWidthAttr(0.6)
        rl.CreateHeightAttr(0.6)
        rl.CreateIntensityAttr(r6(rect))
        rl.AddTranslateOp().Set(Gf.Vec3d(lx, ly, 2.6))
    st.GetRootLayer().customLayerData = {"seed": int(p["seed"]), "task": "dice_mission_v5", "light_level": r6(level)}
    st.GetRootLayer().Save()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, required=True)
    ap.add_argument("--light-level", type=float, default=None)
    ap.add_argument("--out", type=Path, default=None)
    args = ap.parse_args()
    out = args.out or HERE / f"worlds/dice_mission/seed_{args.seed}"
    out.mkdir(parents=True, exist_ok=True)
    drawing = make_drawing()
    for t in A["barcodes"]:
        barcode_png(t, HERE / f"textures/barcode648_{t}.png")
    p = plan(args.seed)
    robot = Robot(URDF_PKG / "urdf/waffle_4wd.urdf", URDF_PKG)
    errs, near = check(p, drawing, robot)
    if errs:
        raise SystemExit("확인 실패: " + "; ".join(errs))
    level = args.light_level
    if level is None:
        level = float(np.random.default_rng([args.seed, 7]).uniform(0, 1))
    level = min(max(level, 0.0), 1.0)
    p.update({"seed": args.seed, "light_level": r6(level)})
    usd = out / "task_world.usda"
    if usd.exists():
        usd.unlink()
    write_usd(usd, drawing, p, level)
    meta = {"seed": args.seed, "task": "dice_mission_v5", "light_level": r6(level),
            "assumptions": {**A, "shelf_size": A["box_small"], "barcode_label": A["barcode_label"]},
            "sources": {"draft": str(DRAFT), "drawing": str(DRAWING), "drawing_sha256": sha256(DRAWING),
                        "robot_usd": str(ROBOT_USD), "urdf_sha256": sha256(URDF_PKG / "urdf/waffle_4wd.urdf")},
            "robot_min_clearance_m": near, **{k: v for k, v in p.items() if k not in ("seed", "light_level")}}
    (out / "layout.json").write_text(json.dumps(meta, indent=1, ensure_ascii=False))
    lt, bc = p["signal_light"], p["barcodes"][0]
    print(f"seed {args.seed}: 신호 {lt['sequence']} (R{lt['red']} Y{lt['yellow']} G{lt['green']}, Y {lt['yellow_shape']}, "
          f"후보 {len(lt['feasible'])}), 빨강 퍽 {sum(q['color'] == 'red' for q in p['pucks'])}, "
          f"누운 직육면체 {sum(not q['standing'] for q in p['pucks'])}, 바코드 {bc['text']} y {bc['center'][1]:.3f} z {bc['center'][2]:.3f}, "
          f"박스 yaw {[round(math.degrees(b['yaw']), 1) for b in p['boxes'][:6]]}, 조명 {level:.2f}, 로봇 최소 거리 {near:.3f} m")


if __name__ == "__main__":
    main()
