"""가상 과제 월드(USD) 만들기 — 인지 파트 시험용. 최종 과제(B조 주사위, 2026-10-23) 기준.

구성: 미로 위쪽 반(도면 방향, data/maze_upper_drawing.json) + 바닥 + 선반 3개(선반마다 주사위 하나)
      + 퍽 12개(빨강 · 초록, 윗면 · 아랫면 20 mm 라벨 안 16 mm tag36h11) + 장소 태그 3개(ID 12~14, 48 mm)
      + 바코드 1~4개(벽 또는 선반 윗면) + 신호등 1개(서 있음, 벽 쪽) + 출발 칸 테이프
      + 로봇(261022_2119 USD 참조, 무작위 위치 · 방향 · 팔 자세) + 조명(어두움~밝음 사이 값).
과제 풀이에 맞게 점멸 순서(빨강 · 초록 · 노랑 횟수 = 주사위 눈)와 퍽 색 · 모양을 함께 뽑는다.
처음에는 퍽이 모두 선반 위지만, 놓쳐 떨어진 경우를 위해 0~3개를 선반 옆 바닥에 둔다(도메인 다양화, 사용자 2026-10-23).
같은 seed면 같은 파일이 나온다(시각 · 임의 순서 없음). 과제 공개 전 판(파란 퍽 · Aruco 마커)은 v1_pre_task/

실행 (PC): ~/isaacsim-venv/bin/python -I build_task_world.py --seed 1
결과: worlds/b_dice/seed_<seed>/task_world.usda, layout.json
"""
import argparse
import hashlib
import json
import math
import os
import sys
from pathlib import Path

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade, Vt

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from robot_geom import ARM_LINKS, Robot, box_depth, hull_depth  # noqa: E402

MODEL = Path.home() / "git/lv2_next_module/261022_2119"
ROBOT_USD = MODEL / "usd/waffle_4wd_261022_2119/waffle_4wd/waffle_4wd.usda"
URDF_PKG = MODEL / "urdf/waffle_4wd_description_261022_2119"

# ---- 가정값 (단위 m, kg). 공개·실측되면 바꾼다 ----
A = {
    "panel_height": 0.380,        # 실측(사용자 2026-10-23)
    "panel_thickness": 0.003,     # 실측
    "panel_bottom": 0.0,          # 가정: 판 아래 틈 없음
    "post_al_width": 0.030,       # 과제 자료: 3030 알루미늄 기둥 30 x 30
    "post_printed_size": [0.036, 0.0095],   # 과제 자료: R3 출력 연결기둥 폭 최대 36 · 두께 9.5 (폭이 벽 방향 — 가정)
    "post_body_top": 0.380,       # 과제 자료: 기둥 높이 380
    "post_head_diameter": 0.060,  # 실측 · 과제 자료(D60 · R5 마운트 지름 60)
    "post_top": 0.400,            # 실측 꼭대기 높이(과제 그림은 전체 390 — 실측을 따름). 머리 두께 0.020은 가정
    "post_foot_height": 0.010,    # 가정: 아래 마운트(지름 60) 두께
    "shelf_size": [0.16, 0.16, 0.08],   # 사용자(확실하지 않음): 높이 8, 바닥 16 x 16 cm
    "shelf_count": 3,             # 선반마다 주사위 하나 → 3개 (사용자 2026-10-23)
    "shelf_wall_gap": 0.02,       # 가정: 선반과 벽판 사이 틈
    "pucks_per_shelf": [2, 6],    # 가정: 처음 선반마다 퍽 수 범위(합 12)
    "floor_pucks": [0, 3],        # 사용자(도메인 다양화): 놓쳐 바닥에 떨어진 퍽 수 범위
    "floor_puck_dist": [0.03, 0.25],  # 가정: 선반 가장자리에서 바닥 퍽까지
    "puck_cuboid": [0.03, 0.03, 0.06],  # 사실(과제 자료)
    "puck_cylinder": [0.03, 0.06],      # 지름 · 높이, 사실(과제 자료)
    "puck_mass": {"cuboid": 0.030, "cylinder": 0.024},  # 가정(PLA, 채움 비율 모름)
    "puck_standing_prob": 0.5,    # 가정(퍽 자세는 출제자 자유)
    "dice_pips": [1, 6],          # 주사위 눈 범위. 빨강 + 초록 + 노랑 = 12 (퍽을 모두 씀, 사용자)
    "tag_family": "tag36h11",     # 사실(과제 자료)
    "puck_tag_size": 0.016,       # 사실: 검은 사각형 16 mm, 흰 여백 포함 라벨 20 mm
    "puck_tag_faces": "윗면 · 아랫면만(사용자 2026-10-23)",
    "place_tag_ids": [12, 13, 14],   # 사실: 장소 L1~L3
    "place_tag_size": 0.048,      # 사용자 결정(인쇄 축척이 틀려 다시 받을 예정): 흰 여백 포함 0.060
    "place_tag_height": [0.13, 0.30],   # 가정: 선반 뒤 벽판, 중심 높이 범위(선반 윗면 0.08 위)
    "barcodes": ["P2-01", "P2-02", "P2-03", "P2-04"],   # 과제 그림의 작업 지시 문자열
    "barcode_count": [1, 4],      # 가정
    "barcode_label": [0.035, 0.020],    # 사용자 실측 가로 · 세로
    "barcode_on_shelf_prob": 0.25,  # 가정: 벽 또는 상자(선반) 윗면, 바닥 · 상자 옆면은 안 됨(사용자)
    "barcode_height": [0.10, 0.30],     # 가정: 벽 바코드 중심 높이
    "signal_light": {             # 사용자 실측(서 있을 때, mm) + 임시 고정값(덮개 3.3 · 받침 높이 10 · 받침 지름 39.6 · 기둥 지름 10)
        "head_diameter": 0.0396, "segment_length": 0.0349, "first_led_z": 0.0689, "cap": 0.0033,
        "base_height": 0.010, "base_diameter": 0.0396, "pole_diameter": 0.010,
        "wall_gap": 0.02},        # 가정: 머리 원통 바깥과 벽판 사이 틈(길을 가리지 않게 벽 쪽)
    "signal_timing": {"on": [0.1, 1.0], "off": [0.1, 1.0], "term_min": "max(3 x off, 1.0)", "term_max": 5.0},
    "start_box_outer": 0.60,      # 가정
    "start_tape_width": 0.03,     # 가정
    "start_tape_thickness": 0.001,
    "start_cell": "c00",          # 사용자 지정(2026-10-23): 왼쪽 아래 구석 칸(왼쪽 바깥 벽·가운데 벽·긴 줄 벽 사이)
    "start_place": "출발 칸(c00, 0.734 x 0.788 m)의 가운데 — 칸은 사용자 지정, 칸 안 위치(가운데)도 사용자 지정",
    "robot_offset_max": 0.02,     # 가정: 출발 칸 가운데에서 로봇 중심의 무작위 어긋남
    # 물리 수치: 시뮬레이션 세션 v12 기준(2026-10-24 채택, 261022_2119/변경요약_시뮬레이션_261023.md 3절). 실측 · 공식 값 아님
    "ground_friction": [0.3, 0.225],  # 바퀴–바닥 정지 · 운동 마찰(v10, 임의 값), 합치는 방식 average
    "arm_drive": {"kp": 400.0, "kd": 36.0, "gripper_kp": 1000.0, "gripper_kd": 10.0,
                  "wheel_kd": 1.0},  # 팔 감쇠 kd 36(v12, 이전 v8은 24)
    "armature": {"wheel": 0.0163, "arm": 0.045, "gripper": 0.0},  # [kg·m²] 모터 회전자 반영 관성(v11)
    "arm_ccd": "speculative",       # 팔 · 집게 강체 연속 충돌 검출(v9)
    "light_level": "0 = 가장 어두움(낮조명off) ~ 1 = 가장 밝음(낮조명on). 값은 화면으로 맞추지 않은 시작값",
}
CELL_Y = 0.788
COLORS = {"red": (0.75, 0.08, 0.07), "green": (0.10, 0.55, 0.18)}
# 신호등 칸: 꺼졌을 때 보이는 색(어두운 반투명 가정)과 켜졌을 때 빛 색. 렌더 스크립트가 emissiveColor 를 바꾼다
SIGNAL_SEGMENTS = [("green", (0.05, 0.18, 0.08), (0.1, 1.0, 0.25)),
                   ("yellow", (0.20, 0.16, 0.03), (1.0, 0.75, 0.05)),
                   ("red", (0.20, 0.04, 0.04), (1.0, 0.08, 0.05))]   # 아래부터
LIGHT_RANGE = {"dome": (300.0, 600.0), "rect": (0.0, 20000.0)}   # light_level 0 ~ 1 사이를 선형으로
ARM_JOINTS = ["RMX52_Base_Yaw", "RMX52_Shoulder", "RMX52_Elbow", "RMX52_Wrist_Pitch", "RMX52_Wrist_Roll_Reversed"]


def r6(v):
    return float(round(float(v), 6))


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def quat_from_R(R):
    q = Gf.Matrix3d(*np.asarray(R, float).T.ravel().tolist()).ExtractRotation().GetQuat()  # Gf는 행벡터 규약
    return q


def Rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def Ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


# ---------------- USD 도우미 ----------------
class Builder:
    def __init__(self, path):
        self.path = Path(path)
        self.stage = Usd.Stage.CreateNew(str(self.path))
        UsdGeom.SetStageUpAxis(self.stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(self.stage, 1.0)
        UsdPhysics.SetStageKilogramsPerUnit(self.stage, 1.0)
        world = UsdGeom.Xform.Define(self.stage, "/World")
        self.stage.SetDefaultPrim(world.GetPrim())
        self.mats = {}

    def rel(self, target):
        return Path(os.path.relpath(target, self.path.parent)).as_posix()

    def material(self, name, color, roughness=0.6, metallic=0.0, texture=None):
        if name in self.mats:
            return self.mats[name]
        mp = f"/World/Looks/{name}"
        mat = UsdShade.Material.Define(self.stage, mp)
        sh = UsdShade.Shader.Define(self.stage, mp + "/PBR")
        sh.CreateIdAttr("UsdPreviewSurface")
        sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
        sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
        if texture:
            rd = UsdShade.Shader.Define(self.stage, mp + "/st_reader")
            rd.CreateIdAttr("UsdPrimvarReader_float2")
            rd.CreateInput("varname", Sdf.ValueTypeNames.String).Set("st")
            tx = UsdShade.Shader.Define(self.stage, mp + "/texture")
            tx.CreateIdAttr("UsdUVTexture")
            tx.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(self.rel(texture)))
            tx.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("clamp")
            tx.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("clamp")
            tx.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(rd.ConnectableAPI(), "result")
            tx.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
            sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(tx.ConnectableAPI(), "rgb")
        else:
            sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
        mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
        self.mats[name] = mat
        return mat

    def physics_material(self, name, static, dynamic):
        mat = UsdShade.Material.Define(self.stage, f"/World/Looks/{name}")
        api = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
        api.CreateStaticFrictionAttr(static)
        api.CreateDynamicFrictionAttr(dynamic)
        api.CreateRestitutionAttr(0.0)
        # PhysxMaterialAPI 속성 이름으로 직접 씀(-I 실행 파이썬에는 PhysxSchema 가 없음). 물리 시험(run_phys_test.py)과 같은 average
        prim = mat.GetPrim()
        prim.AddAppliedSchema("PhysxMaterialAPI")
        prim.CreateAttribute("physxMaterial:frictionCombineMode", Sdf.ValueTypeNames.Token).Set("average")
        prim.CreateAttribute("physxMaterial:restitutionCombineMode", Sdf.ValueTypeNames.Token).Set("average")
        return mat

    @staticmethod
    def bind(prim, mat, purpose=None):
        api = UsdShade.MaterialBindingAPI.Apply(prim)
        if purpose:
            api.Bind(mat, UsdShade.Tokens.weakerThanDescendants, purpose)
        else:
            api.Bind(mat)

    def xform(self, path, t=(0, 0, 0), quat=None):
        x = UsdGeom.Xform.Define(self.stage, path)
        x.AddTranslateOp().Set(Gf.Vec3d(*map(r6, t)))
        if quat is not None:
            x.AddOrientOp().Set(Gf.Quatf(quat))
        return x

    def box(self, path, center, size, mat=None, collide=True):
        c = UsdGeom.Cube.Define(self.stage, path)
        c.CreateSizeAttr(1.0)
        c.CreateExtentAttr([Gf.Vec3f(-0.5, -0.5, -0.5), Gf.Vec3f(0.5, 0.5, 0.5)])
        c.AddTranslateOp().Set(Gf.Vec3d(*map(r6, center)))
        c.AddScaleOp().Set(Gf.Vec3f(*map(r6, size)))
        if collide:
            UsdPhysics.CollisionAPI.Apply(c.GetPrim())
        if mat:
            self.bind(c.GetPrim(), mat)
        return c

    def cylinder(self, path, center, radius, height, mat=None, collide=True):
        c = UsdGeom.Cylinder.Define(self.stage, path)
        c.CreateRadiusAttr(r6(radius))
        c.CreateHeightAttr(r6(height))
        c.CreateAxisAttr("Z")
        c.CreateExtentAttr([Gf.Vec3f(-radius, -radius, -height / 2), Gf.Vec3f(radius, radius, height / 2)])
        c.AddTranslateOp().Set(Gf.Vec3d(*map(r6, center)))
        if collide:
            UsdPhysics.CollisionAPI.Apply(c.GetPrim())
        if mat:
            self.bind(c.GetPrim(), mat)
        return c

    def mesh(self, path, points, faces, st, mat):
        m = UsdGeom.Mesh.Define(self.stage, path)
        pts = np.asarray(points, float)
        m.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*map(r6, p)) for p in pts]))
        m.CreateFaceVertexCountsAttr([len(f) for f in faces])
        m.CreateFaceVertexIndicesAttr([i for f in faces for i in f])
        m.CreateExtentAttr([Gf.Vec3f(*map(r6, pts.min(0))), Gf.Vec3f(*map(r6, pts.max(0)))])
        m.CreateSubdivisionSchemeAttr("none")
        pv = UsdGeom.PrimvarsAPI(m).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
        pv.Set(Vt.Vec2fArray([Gf.Vec2f(*map(r6, s)) for s in st]))
        self.bind(m.GetPrim(), mat)
        return m


def quad(center, u, v, half_u, half_v):
    """u·v 방향 사각형(앞면 = u x v 방향). st: u -> s, v -> t."""
    c, u, v = (np.asarray(a, float) for a in (center, u, v))
    pts = [c - half_u * u - half_v * v, c + half_u * u - half_v * v, c + half_u * u + half_v * v,
           c - half_u * u + half_v * v]
    return pts, [[0, 1, 2, 3]], [(0, 0), (1, 0), (1, 1), (0, 1)]


# ---------------- 배치 ----------------
def cells_and_walls(layout):
    xs = layout["grid"]["x"]
    posts = {p["id"]: (p["x"], p["y"]) for p in layout["posts"]}
    xpan = {(round(pn["center"][0], 3), round(pn["center"][1], 3)): pn for pn in layout["panels"] if pn["axis"] == "x"}
    ypan = {(round(pn["center"][0], 3), round(pn["center"][1], 3)): pn for pn in layout["panels"] if pn["axis"] == "y"}
    rows = [round(k * CELL_Y, 3) for k in range(5)]
    cells = []
    for i in range(len(xs) - 1):
        for k in range(4):
            x0, x1, y0, y1 = xs[i], xs[i + 1], rows[k], rows[k + 1]
            cx, cy = round((x0 + x1) / 2, 4), round((y0 + y1) / 2, 4)
            edges = []
            for side, key, normal in (("bottom", (round(cx, 3), y0), (0, 1)), ("top", (round(cx, 3), y1), (0, -1))):
                if key in xpan:
                    edges.append({"side": side, "panels": [xpan[key]["id"]], "line": key[1], "axis": "x",
                                  "normal": normal, "span": (x0, x1)})
            for side, xx, normal in (("left", x0, (1, 0)), ("right", x1, (-1, 0))):
                keys = [(round(xx, 3), round(y0 + 0.197, 3)), (round(xx, 3), round(y0 + 0.591, 3))]
                if all(kk in ypan for kk in keys):
                    edges.append({"side": side, "panels": [ypan[kk]["id"] for kk in keys], "line": xx, "axis": "y",
                                  "normal": normal, "span": (y0, y1)})
            cells.append({"id": f"c{i}{k}", "x": (x0, x1), "y": (y0, y1), "center": (cx, cy), "edges": edges})
    return cells, posts


def post_shapes(layout):
    """기둥 몸통 (이름, 중심 xy, 반폭 xy). 출력 연결기둥은 이어진 벽판 방향으로 폭 0.036 · 두께 0.0095."""
    axes = {}
    for pn in layout["panels"]:
        for k in (pn["a"], pn["b"]):
            axes.setdefault(k, set()).add(pn["axis"])
    out = []
    for po in layout["posts"]:
        if po["type"] == "al":
            h = (A["post_al_width"] / 2,) * 2
        else:
            w, t = A["post_printed_size"]
            ax = axes.get(po["id"], set())
            if ax == {"x"}:
                h = (w / 2, t / 2)
            elif ax == {"y"}:
                h = (t / 2, w / 2)
            else:  # 꺾이는 곳(도면상 없어야 함): 보수적으로 정사각
                h = (w / 2, w / 2)
        out.append((po, (po["x"], po["y"]), h))
    return out


def world_boxes(layout, shelves, pucks, light=None):
    """겹침 계산용 상자 목록 (center, half, 이름)."""
    out = []
    H, T = A["panel_height"], A["panel_thickness"]
    for pn in layout["panels"]:
        L = pn["length"]
        size = (L, T, H) if pn["axis"] == "x" else (T, L, H)
        out.append(((pn["center"][0], pn["center"][1], A["panel_bottom"] + H / 2), np.array(size) / 2,
                    f"panel_{pn['id']:02d}"))
    hz = A["post_top"] - A["post_body_top"]
    d = A["post_head_diameter"]
    for po, c, h in post_shapes(layout):
        out.append(((c[0], c[1], A["post_body_top"] / 2), np.array([h[0], h[1], A["post_body_top"] / 2]),
                    f"post_{po['id']:02d}_body"))
        out.append(((c[0], c[1], A["post_body_top"] + hz / 2), np.array([d, d, hz]) / 2, f"post_{po['id']:02d}_head"))
        out.append(((c[0], c[1], A["post_foot_height"] / 2), np.array([d, d, A["post_foot_height"]]) / 2,
                    f"post_{po['id']:02d}_foot"))
    for sh in shelves:
        out.append((tuple(sh["center"]), np.array(A["shelf_size"]) / 2, sh["name"]))
    for pk in pucks:
        lo, hi = np.array(pk["aabb"][0]), np.array(pk["aabb"][1])
        out.append((tuple((lo + hi) / 2), (hi - lo) / 2, pk["name"]))
    if light:
        r = A["signal_light"]["head_diameter"] / 2
        out.append(((light["x"], light["y"], light["height"] / 2), np.array([r, r, light["height"] / 2]), "signal_light"))
    return out


def aabb_hit(c, h, boxes, gap=0.0):
    c, h = np.asarray(c, float), np.asarray(h, float)
    return any(np.all(np.abs(c - np.asarray(bc)) < h + np.asarray(bh) + gap) for bc, bh, _ in boxes)


def robot_hits(robot, q, T, boxes, near=0.9):
    ws = robot.world_shapes(q, T)
    base_xy = T[:2, 3]
    hits = []
    for c, h, name in boxes:
        if np.abs(np.asarray(c[:2]) - base_xy).max() > near + h[:2].max():
            continue
        for link, V in ws:
            d = box_depth(V, c, h)
            if d >= 0:
                hits.append((link, name, d))
                break
    return hits


def robot_self_ok(robot, q, T):
    ws = robot.world_shapes(q, T)
    for link, V in ws:  # 바닥 아래로 들어가면 안 됨(바퀴 제외)
        if link in ARM_LINKS and V[:, 2].min() < 0.005:
            return False
    arm = [(l, V) for l, V in ws if l in ARM_LINKS]
    for la, Va in ws:
        for lb, Vb in arm:
            if la == lb or frozenset((la, lb)) in robot.adjacent:
                continue
            if hull_depth(Va, Vb) >= 0:
                return False
    return True


def puck_pose(rng, shape):
    standing = bool(rng.random() < A["puck_standing_prob"])
    yaw = float(rng.uniform(-math.pi, math.pi))
    if standing:
        flip = bool(rng.random() < 0.5)
        R = Rz(yaw) @ (np.diag([1.0, -1.0, -1.0]) if flip else np.eye(3))
        half_z = A["puck_cuboid"][2] / 2
    else:
        roll = float(rng.integers(0, 4)) * math.pi / 2 if shape == "cuboid" else float(rng.uniform(-math.pi, math.pi))
        R = Rz(yaw) @ Ry(math.pi / 2) @ Rz(roll)
        half_z = A["puck_cuboid"][0] / 2
    return standing, R, half_z


def puck_local_corners(shape):
    if shape == "cuboid":
        s = np.array(A["puck_cuboid"]) / 2
    else:
        d, h = A["puck_cylinder"]
        s = np.array([d / 2, d / 2, h / 2])  # 원기둥은 외접 상자(보수적)
    return np.array([[x, y, z] for x in (-s[0], s[0]) for y in (-s[1], s[1]) for z in (-s[2], s[2])])


def dice_plan(rng):
    """점멸 순서와 퍽 색 · 모양. 빨강 · 초록 · 노랑 횟수(각 1~6, 합 12) = 주사위 눈.
    빨강 · 초록 주사위 퍽은 그 색(모양 무관), 노랑 주사위 퍽은 색 무관 · 모양 하나(yellow_shape)."""
    lo, hi = A["dice_pips"]
    while True:
        r, g = int(rng.integers(lo, hi + 1)), int(rng.integers(lo, hi + 1))
        y = 12 - r - g
        if lo <= y <= hi:
            break
    seq = list("R" * r + "G" * g + "Y" * y)
    seq = "".join(seq[i] for i in rng.permutation(12))
    yellow_shape = ["cylinder", "cuboid"][int(rng.integers(2))]
    specs = [("red", ["cylinder", "cuboid"][int(rng.integers(2))], "R") for _ in range(r)]
    specs += [("green", ["cylinder", "cuboid"][int(rng.integers(2))], "G") for _ in range(g)]
    specs += [(["red", "green"][int(rng.integers(2))], yellow_shape, "Y") for _ in range(y)]
    specs = [specs[i] for i in rng.permutation(12)]
    on = float(rng.uniform(*A["signal_timing"]["on"]))
    off = float(rng.uniform(*A["signal_timing"]["off"]))
    tmin = max(3 * off, 1.0)
    term = float(rng.uniform(tmin, A["signal_timing"]["term_max"]))
    timing = {"on": r6(on), "off": r6(off), "term": r6(term)}
    return {"sequence": seq, "red": r, "green": g, "yellow": y, "yellow_shape": yellow_shape, "timing": timing}, specs


def place_on_shelf(rng, shelf, shape, others, keep_out=None):
    sx, sy, _ = A["shelf_size"]
    for _ in range(800):
        standing, R, half_z = puck_pose(rng, shape)
        corners = puck_local_corners(shape) @ R.T
        half_xy = np.abs(corners[:, :2]).max(0)
        lim = np.array([sx, sy]) / 2 - half_xy - 0.003
        if (lim <= 0).any():
            continue
        cxy = np.array(shelf["center"][:2]) + rng.uniform(-lim, lim)
        if keep_out is not None and np.all(np.abs(cxy - keep_out[0]) < half_xy + keep_out[1] + 0.003):
            continue
        if all(np.any(np.abs(cxy - np.array(o["center"][:2])) > half_xy + np.array(o["half_xy"]) + 0.005)
               for o in others):
            return standing, R, half_z, corners, half_xy, cxy
    return None


def plan(seed, layout, robot):
    rng = np.random.default_rng(seed)
    cells, posts = cells_and_walls(layout)
    walled = [c for c in cells if c["edges"]]
    start = next(c for c in cells if c["id"] == A["start_cell"])
    dice, specs = dice_plan(rng)
    # 선반 3개: 출발 칸이 아닌 서로 다른 칸의 벽 옆
    others = [c for c in walled if c["id"] != start["id"]]
    pick = rng.choice(len(others), size=A["shelf_count"], replace=False)
    sx, sy, sz = A["shelf_size"]
    W_post = max(A["post_al_width"], A["post_printed_size"][0])
    shelves = []
    for n, idx in enumerate(sorted(int(i) for i in pick)):
        c = others[idx]
        e = c["edges"][int(rng.integers(len(c["edges"])))]
        lo, hi = e["span"]
        room = (hi - lo) / 2 - W_post / 2 - sx / 2 - 0.01
        along = (lo + hi) / 2 + float(rng.uniform(-room, room))
        inward = e["line"] + (A["panel_thickness"] / 2 + A["shelf_wall_gap"] + sy / 2) * (
            e["normal"][1] if e["axis"] == "x" else e["normal"][0])
        center = (along, inward, sz / 2) if e["axis"] == "x" else (inward, along, sz / 2)
        shelves.append({"name": f"shelf_{n}", "cell": c["id"], "wall_side": e["side"], "edge": e, "along": r6(along),
                        "center": [r6(v) for v in center], "size": A["shelf_size"], "top_z": sz,
                        "place_tag_id": A["place_tag_ids"][n]})
    # 바코드 자리: 개수 1~4, 벽 또는 선반 윗면(선반 하나에 하나까지, 벽 쪽 가장자리 띠)
    n_bar = int(rng.integers(A["barcode_count"][0], A["barcode_count"][1] + 1))
    bar_where = []
    shelf_free = list(range(len(shelves)))
    for k in range(n_bar):
        if shelf_free and rng.random() < A["barcode_on_shelf_prob"]:
            si = shelf_free.pop(int(rng.integers(len(shelf_free))))
            bar_where.append(("shelf", si))
        else:
            bar_where.append(("wall", None))
    keep_out = {}   # 선반 이름 → (중심 xy, 반폭 xy): 선반 윗면 바코드 자리(퍽을 두지 않음)
    bw, bh = A["barcode_label"]
    for kind, si in bar_where:
        if kind != "shelf":
            continue
        sh = shelves[si]
        e = sh["edge"]
        nrm = np.array(e["normal"], float)
        c = np.array(sh["center"][:2]) - nrm * (sy / 2 - bh / 2 - 0.004)   # 벽 쪽 가장자리에서 4 mm 안
        half = np.array([bw / 2, bh / 2]) if e["axis"] == "x" else np.array([bh / 2, bw / 2])
        keep_out[sh["name"]] = (c, half)
    # 퍽: 처음엔 모두 선반 위(선반마다 2~6개), 그중 0~3개는 선반 옆 바닥으로
    lo_n, hi_n = A["pucks_per_shelf"]
    while True:
        split = [int(rng.integers(lo_n, hi_n + 1)) for _ in range(2)]
        last = 12 - sum(split)
        if lo_n <= last <= hi_n:
            split.append(last)
            break
    n_floor = int(rng.integers(A["floor_pucks"][0], A["floor_pucks"][1] + 1))
    floor_idx = set(int(i) for i in rng.choice(12, size=n_floor, replace=False))
    tag_ids = [int(t) for t in rng.permutation(12)]
    static_boxes = world_boxes(layout, shelves, [])
    pucks = []
    n = 0
    for si, cnt in enumerate(split):
        shelf = shelves[si]
        for _ in range(cnt):
            color, shape, role = specs[n]
            if n in floor_idx:
                cell = next(c for c in cells if c["id"] == shelf["cell"])
                got = None
                for _ in range(2000):
                    standing, R, half_z = puck_pose(rng, shape)
                    corners = puck_local_corners(shape) @ R.T
                    half_xy = np.abs(corners[:, :2]).max(0)
                    ang = float(rng.uniform(-math.pi, math.pi))
                    dist = float(rng.uniform(*A["floor_puck_dist"])) + sx / 2 + float(half_xy.max())
                    cxy = np.array(shelf["center"][:2]) + dist * np.array([math.cos(ang), math.sin(ang)])
                    if not (cell["x"][0] + 0.02 < cxy[0] < cell["x"][1] - 0.02 and cell["y"][0] + 0.02 < cxy[1] < cell["y"][1] - 0.02):
                        continue
                    if aabb_hit((*cxy, half_z), (*half_xy, half_z), static_boxes, 0.005):
                        continue
                    if any(np.all(np.abs(cxy - np.array(o["center"][:2])) < half_xy + np.array(o["half_xy"]) + 0.005)
                           for o in pucks):
                        continue
                    got = (standing, R, half_z, corners, half_xy, cxy)
                    break
                where, base_z = "floor", 0.0
            else:
                got = place_on_shelf(rng, shelf, shape, [o for o in pucks if o["location"] == shelf["name"]],
                                     keep_out.get(shelf["name"]))
                where, base_z = shelf["name"], shelf["top_z"]
            if got is None:
                raise RuntimeError(f"퍽 {n} 자리 못 찾음 ({where})")
            standing, R, half_z, corners, half_xy, cxy = got
            center = [r6(cxy[0]), r6(cxy[1]), r6(base_z + half_z)]
            aabb_half = np.abs(corners).max(0)
            pucks.append({"name": f"puck_{tag_ids[n]:02d}", "tag_id": tag_ids[n], "color": color, "shape": shape,
                          "dice_role": role, "standing": standing, "home_shelf": shelf["name"], "location": where,
                          "center": center, "R": [[r6(v) for v in row] for row in R],
                          "half_xy": [r6(v) for v in half_xy],
                          "aabb": [[r6(c - h) for c, h in zip(center, aabb_half)],
                                   [r6(c + h) for c, h in zip(center, aabb_half)]]})
            n += 1
    # 장소 태그: 선반 뒤 벽판(선반이 붙은 벽), 칸 안쪽을 향함
    place_tags = []
    for sh in shelves:
        e = sh["edge"]
        nrm = np.array([*e["normal"], 0.0])
        half = A["place_tag_size"] * 10 / 8 / 2
        pid, along = None, None
        for _ in range(200):
            al = sh["along"] + float(rng.uniform(-0.04, 0.04))
            for q in e["panels"]:
                pn = next(x for x in layout["panels"] if x["id"] == q)
                pc = pn["center"][0] if e["axis"] == "x" else pn["center"][1]
                if abs(al - pc) < pn["length"] / 2 - half - 0.002:
                    pid, along = q, al
                    break
            if pid is not None:
                break
        if pid is None:
            raise RuntimeError(f"{sh['name']} 장소 태그 자리 못 찾음")
        z = float(rng.uniform(*A["place_tag_height"]))
        off = A["panel_thickness"] / 2 + 0.0005
        xy = (along, e["line"]) if e["axis"] == "x" else (e["line"], along)
        c = np.array([xy[0], xy[1], z]) + nrm * off
        place_tags.append({"id": sh["place_tag_id"], "shelf": sh["name"], "panel": pid, "normal": nrm.tolist(),
                           "center": [r6(v) for v in c], "size": A["place_tag_size"]})
    # 신호등: 출발 칸 · 선반 칸이 아닌 칸의 벽 옆(벽판 가운데 쪽), 서 있음
    sl = A["signal_light"]
    height = sl["first_led_z"] + 3 * sl["segment_length"] + sl["cap"]
    used = {start["id"]} | {sh["cell"] for sh in shelves}
    cand = [c for c in walled if c["id"] not in used]
    light = None
    for _ in range(200):
        c = cand[int(rng.integers(len(cand)))]
        e = c["edges"][int(rng.integers(len(c["edges"])))]
        lo, hi = e["span"]
        along = float(rng.uniform(lo + 0.08, hi - 0.08))
        inward = e["line"] + (A["panel_thickness"] / 2 + sl["wall_gap"] + sl["head_diameter"] / 2) * (
            e["normal"][1] if e["axis"] == "x" else e["normal"][0])
        x, y = (along, inward) if e["axis"] == "x" else (inward, along)
        r = sl["head_diameter"] / 2
        if aabb_hit((x, y, height / 2), (r, r, height / 2), static_boxes, 0.005):
            continue
        light = {"cell": c["id"], "x": r6(x), "y": r6(y), "yaw": r6(float(rng.uniform(-math.pi, math.pi))),
                 "height": r6(height), "wall_side": e["side"], **dice}
        break
    if light is None:
        raise RuntimeError("신호등 자리 못 찾음")
    # 바코드: 벽은 무작위 칸 · 벽판(장소 태그 · 신호등과 겹치지 않게), 선반은 벽 쪽 가장자리 띠
    barcodes = []
    for k, (kind, si) in enumerate(bar_where):
        text = A["barcodes"][k]
        if kind == "shelf":
            sh = shelves[si]
            c, half = keep_out[sh["name"]]
            e = sh["edge"]
            u = np.array([1.0, 0, 0]) if e["axis"] == "x" else np.array([0, 1.0, 0])   # 라벨 가로 = 벽 방향
            barcodes.append({"text": text, "where": "shelf_top", "shelf": sh["name"],
                             "center": [r6(c[0]), r6(c[1]), r6(sh["top_z"] + 0.0005)], "normal": [0.0, 0.0, 1.0],
                             "u": u.tolist(), "size": A["barcode_label"]})
            continue
        for _ in range(500):
            c = walled[int(rng.integers(len(walled)))]
            e = c["edges"][int(rng.integers(len(c["edges"])))]
            q = e["panels"][int(rng.integers(len(e["panels"])))]
            pn = next(x for x in layout["panels"] if x["id"] == q)
            pc = pn["center"][0] if e["axis"] == "x" else pn["center"][1]
            room = pn["length"] / 2 - bw / 2 - 0.01
            along = pc + float(rng.uniform(-room, room))
            z = float(rng.uniform(*A["barcode_height"]))
            nrm = np.array([*e["normal"], 0.0])
            xy = (along, e["line"]) if e["axis"] == "x" else (e["line"], along)
            ctr = np.array([xy[0], xy[1], z]) + nrm * (A["panel_thickness"] / 2 + 0.0005)
            clash = any(np.allclose(t["normal"], nrm) and np.linalg.norm(np.array(t["center"]) - ctr) < 0.06
                        for t in place_tags + [b for b in barcodes if b["where"] == "wall"])
            if clash or (abs(ctr[0] - light["x"]) < 0.08 and abs(ctr[1] - light["y"]) < 0.08):
                continue
            barcodes.append({"text": text, "where": "wall", "cell": c["id"], "panel": q,
                             "center": [r6(v) for v in ctr], "normal": nrm.tolist(),
                             "u": np.cross([0, 0, 1.0], nrm).tolist(), "size": A["barcode_label"]})
            break
        else:
            raise RuntimeError(f"바코드 {text} 자리 못 찾음")
    # 로봇: 출발 칸 가운데, 무작위 방향 · 팔 자세. 겹치면 다시 뽑는다
    boxes = world_boxes(layout, shelves, pucks, light)
    ws0 = robot.world_shapes({})
    body = np.vstack([V for l, V in ws0 if l not in ARM_LINKS])
    body_c = (body[:, :2].min(0) + body[:, :2].max(0)) / 2
    lims = {j: (robot.joint_by_name[j]["lower"], robot.joint_by_name[j]["upper"]) for j in ARM_JOINTS}
    g_lo, g_hi = 0.0, robot.joint_by_name["gripper_left_joint"]["upper"]
    tries = 0
    while True:
        tries += 1
        if tries > 400:
            raise RuntimeError("로봇 출발 자세를 못 찾음")
        yaw = float(rng.uniform(-math.pi, math.pi))
        off = rng.uniform(-A["robot_offset_max"], A["robot_offset_max"], 2)
        q = {j: float(rng.uniform(*lims[j])) for j in ARM_JOINTS}
        q["gripper_left_joint"] = float(rng.uniform(g_lo, g_hi))
        R = Rz(yaw)
        pos = np.array(start["center"]) + off - R[:2, :2] @ body_c
        T = np.eye(4)
        T[:3, :3] = R
        T[:2, 3] = pos
        if not robot_self_ok(robot, q, T):
            continue
        if robot_hits(robot, q, T, boxes):
            continue
        break
    robot_pose = {"x": r6(pos[0]), "y": r6(pos[1]), "yaw": r6(yaw), "q": {k: r6(v) for k, v in q.items()},
                  "tries": tries, "start_cell": start["id"]}
    start_box = {"cell": start["id"], "center": [r6(v) for v in start["center"]], "outer": A["start_box_outer"],
                 "tape_width": A["start_tape_width"]}
    for sh in shelves:
        sh.pop("edge")
    return {"cells": cells, "start_box": start_box, "shelves": shelves, "pucks": pucks, "place_tags": place_tags,
            "barcodes": barcodes, "signal_light": light, "robot": robot_pose}


# ---------------- USD 쓰기 ----------------
def write_usd(path, layout, p, robot, level):
    b = Builder(path)
    st = b.stage
    UsdPhysics.Scene.Define(st, "/World/physicsScene").CreateGravityMagnitudeAttr(9.81)
    # 재질
    m_carpet = b.material("carpet_gray", (0.30, 0.31, 0.32), roughness=0.95)
    m_panel = b.material("acrylic_bluegray", (0.30, 0.35, 0.45), roughness=0.15)
    m_al = b.material("aluminium", (0.75, 0.76, 0.78), roughness=0.35, metallic=1.0)
    m_print = b.material("printed_blue", (0.10, 0.20, 0.75), roughness=0.5)
    m_shelf = b.material("shelf_gray", (0.55, 0.55, 0.55), roughness=0.8)
    m_tape = b.material("tape_white", (0.92, 0.92, 0.92), roughness=0.7)
    m_black = b.material("plastic_black", (0.05, 0.05, 0.05), roughness=0.5)
    m_floor_phys = b.physics_material("floor_contact", *A["ground_friction"])
    # 바닥(무한 평면 충돌 + 보이는 판)
    xs, ys = layout["grid"]["x"], layout["grid"]["y"]
    cx, cy = (xs[0] + xs[-1]) / 2, (ys[0] + ys[-1]) / 2
    UsdGeom.Xform.Define(st, "/World/Ground")
    plane = UsdGeom.Plane.Define(st, "/World/Ground/CollisionPlane")
    plane.CreateAxisAttr("Z")
    plane.CreateWidthAttr(12.0)
    plane.CreateLengthAttr(12.0)
    plane.CreateExtentAttr([Gf.Vec3f(-6, -6, 0), Gf.Vec3f(6, 6, 0)])
    plane.AddTranslateOp().Set(Gf.Vec3d(r6(cx), r6(cy), 0))
    UsdPhysics.CollisionAPI.Apply(plane.GetPrim())
    b.bind(plane.GetPrim(), m_carpet)
    b.bind(plane.GetPrim(), m_floor_phys, "physics")
    # 미로
    H, T = A["panel_height"], A["panel_thickness"]
    UsdGeom.Xform.Define(st, "/World/Maze")
    for pn in layout["panels"]:
        L = pn["length"]
        size = (L, T, H) if pn["axis"] == "x" else (T, L, H)
        prim = b.box(f"/World/Maze/panel_{pn['id']:02d}", (pn["center"][0], pn["center"][1], A["panel_bottom"] + H / 2),
                     size, m_panel).GetPrim()
        prim.CreateAttribute("maze:shared", Sdf.ValueTypeNames.Bool).Set(bool(pn["shared"]))
        prim.CreateAttribute("maze:kind", Sdf.ValueTypeNames.String).Set(pn["kind"])
    hz = A["post_top"] - A["post_body_top"]
    hf = A["post_foot_height"]
    for po, c, h in post_shapes(layout):
        base = f"/World/Maze/post_{po['id']:02d}"
        x = UsdGeom.Xform.Define(st, base)
        x.GetPrim().CreateAttribute("maze:post_type", Sdf.ValueTypeNames.String).Set(po["type"])
        b.box(base + "/body", (c[0], c[1], A["post_body_top"] / 2), (2 * h[0], 2 * h[1], A["post_body_top"]),
              m_al if po["type"] == "al" else m_print)
        b.cylinder(base + "/head", (c[0], c[1], A["post_body_top"] + hz / 2), A["post_head_diameter"] / 2, hz, m_print)
        b.cylinder(base + "/foot", (c[0], c[1], hf / 2), A["post_head_diameter"] / 2, hf, m_print)
    # 출발 칸 테이프(충돌 없음)
    sb = p["start_box"]
    o, w, th = sb["outer"], sb["tape_width"], A["start_tape_thickness"]
    c0 = np.array(sb["center"])
    UsdGeom.Xform.Define(st, "/World/StartBox")
    for name, dc, size in (("tape_s", (0, -(o - w) / 2), (o, w)), ("tape_n", (0, (o - w) / 2), (o, w)),
                           ("tape_w", (-(o - w) / 2, 0), (w, o - 2 * w)), ("tape_e", ((o - w) / 2, 0), (w, o - 2 * w))):
        b.box(f"/World/StartBox/{name}", (c0[0] + dc[0], c0[1] + dc[1], th / 2), (size[0], size[1], th), m_tape,
              collide=False)
    # 선반
    UsdGeom.Xform.Define(st, "/World/Shelves")
    for sh in p["shelves"]:
        prim = b.box(f"/World/Shelves/{sh['name']}", sh["center"], sh["size"], m_shelf).GetPrim()
        prim.CreateAttribute("shelf:place_tag_id", Sdf.ValueTypeNames.Int).Set(int(sh["place_tag_id"]))
    # 장소 태그
    UsdGeom.Xform.Define(st, "/World/PlaceTags")
    for t in p["place_tags"]:
        m_t = b.material(f"tag_{t['id']:02d}", (1, 1, 1), roughness=0.8, texture=HERE / f"textures/tag36h11_{t['id']:02d}.png")
        nrm = np.array(t["normal"])
        half = t["size"] * 10 / 8 / 2
        pts, faces, stc = quad(t["center"], np.cross([0, 0, 1.0], nrm), [0, 0, 1.0], half, half)
        mm = b.mesh(f"/World/PlaceTags/tag_{t['id']:02d}", pts, faces, stc, m_t)
        mm.GetPrim().CreateAttribute("tag:id", Sdf.ValueTypeNames.Int).Set(int(t["id"]))
        mm.GetPrim().CreateAttribute("tag:size", Sdf.ValueTypeNames.Double).Set(t["size"])
    # 바코드
    UsdGeom.Xform.Define(st, "/World/Barcodes")
    for k, bc in enumerate(p["barcodes"]):
        m_b = b.material(f"barcode_{bc['text'].replace('-', '_')}", (1, 1, 1), roughness=0.8,
                         texture=HERE / f"textures/barcode_{bc['text']}.png")
        nrm, u = np.array(bc["normal"]), np.array(bc["u"])
        v = np.cross(nrm, u)
        bw, bh = bc["size"]
        pts, faces, stc = quad(bc["center"], u, v, bw / 2, bh / 2)
        mm = b.mesh(f"/World/Barcodes/barcode_{k}", pts, faces, stc, m_b)
        mm.GetPrim().CreateAttribute("barcode:text", Sdf.ValueTypeNames.String).Set(bc["text"])
    # 퍽: 윗면 · 아랫면 라벨(흰 여백 포함 20 mm, 그 안 검은 사각형 16 mm)
    UsdGeom.Xform.Define(st, "/World/Pucks")
    lab = A["puck_tag_size"] * 10 / 8 / 2
    e = 0.0002
    for pk in p["pucks"]:
        base = f"/World/Pucks/{pk['name']}"
        x = b.xform(base, pk["center"], quat_from_R(pk["R"]))
        prim = x.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(A["puck_mass"][pk["shape"]])
        for k, v in (("puck:color", pk["color"]), ("puck:shape", pk["shape"]), ("puck:location", pk["location"]),
                     ("puck:dice_role", pk["dice_role"])):
            prim.CreateAttribute(k, Sdf.ValueTypeNames.String).Set(v)
        prim.CreateAttribute("puck:tag_id", Sdf.ValueTypeNames.Int).Set(pk["tag_id"])
        prim.CreateAttribute("puck:standing", Sdf.ValueTypeNames.Bool).Set(pk["standing"])
        m_body = b.material(f"puck_{pk['color']}", COLORS[pk["color"]], roughness=0.85)
        m_tag = b.material(f"tag_{pk['tag_id']:02d}", (1, 1, 1), roughness=0.8,
                           texture=HERE / f"textures/tag36h11_{pk['tag_id']:02d}.png")
        if pk["shape"] == "cuboid":
            sx_, sy_, sz_ = A["puck_cuboid"]
            b.box(base + "/body", (0, 0, 0), (sx_, sy_, sz_), m_body)
            hz_ = sz_ / 2
        else:
            d, h = A["puck_cylinder"]
            b.cylinder(base + "/body", (0, 0, 0), d / 2, h, m_body)
            hz_ = h / 2
        for fn, c, u, v in (("top", (0, 0, hz_ + e), (1, 0, 0), (0, 1, 0)), ("bottom", (0, 0, -hz_ - e), (1, 0, 0), (0, -1, 0))):
            pts, fc, stc = quad(c, u, v, lab, lab)
            b.mesh(f"{base}/label_{fn}", pts, fc, stc, m_tag)
    # 신호등: 받침 · 기둥 · 칸 3개(아래부터 초록 · 노랑 · 빨강) · 덮개. 칸 재질의 emissiveColor 를 렌더 스크립트가 바꾼다
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
        sh = UsdShade.Shader(st.GetPrimAtPath(f"/World/Looks/signal_{name}/PBR"))
        sh.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0, 0, 0))   # 꺼짐
        seg = b.cylinder(f"{base}/{name}", (0, 0, z + sl["segment_length"] / 2), sl["head_diameter"] / 2,
                         sl["segment_length"], m).GetPrim()
        seg.CreateAttribute("signal:on_color", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*on_col))
        z += sl["segment_length"]
    b.cylinder(base + "/cap", (0, 0, z + sl["cap"] / 2), sl["head_diameter"] / 2, sl["cap"], m_black)
    # 로봇
    rp = p["robot"]
    rprim = st.DefinePrim("/World/waffle_4wd", "Xform")
    rprim.GetReferences().AddReference(b.rel(ROBOT_USD))
    rprim.GetVariantSets().GetVariantSet("Physics").SetVariantSelection("physx")
    rx = UsdGeom.Xformable(rprim)
    rx.AddTranslateOp().Set(Gf.Vec3d(rp["x"], rp["y"], 0.0))
    rx.AddRotateZOp().Set(r6(math.degrees(rp["yaw"])))
    root = "/World/waffle_4wd/Geometry/base_footprint/base_link"
    for w in ("front_left", "front_right", "rear_left", "rear_right"):
        tp = st.GetPrimAtPath(f"{root}/wheel_{w}/tire_cylinder")
        if not tp.IsValid():
            raise RuntimeError(f"바퀴 충돌체 없음: wheel_{w}")
        b.bind(tp, m_floor_phys, "physics")
    q = robot.full_q(rp["q"])
    ad = A["arm_drive"]
    for jp in st.GetPrimAtPath("/World/waffle_4wd/Physics").GetChildren():
        name = jp.GetName()
        if name.startswith("RMX52_"):
            deg = r6(math.degrees(q[name]))
            jp.CreateAttribute("state:angular:physics:position", Sdf.ValueTypeNames.Float).Set(deg)
            jp.CreateAttribute("drive:angular:physics:targetPosition", Sdf.ValueTypeNames.Float).Set(deg)
            jp.CreateAttribute("drive:angular:physics:stiffness", Sdf.ValueTypeNames.Float).Set(
                r6(ad["kp"] * math.pi / 180))
            jp.CreateAttribute("drive:angular:physics:damping", Sdf.ValueTypeNames.Float).Set(r6(ad["kd"] * math.pi / 180))
        elif name.startswith("gripper_"):
            v = r6(q[name])
            jp.CreateAttribute("state:linear:physics:position", Sdf.ValueTypeNames.Float).Set(v)
            jp.CreateAttribute("drive:linear:physics:targetPosition", Sdf.ValueTypeNames.Float).Set(v)
            left = name == "gripper_left_joint"
            jp.CreateAttribute("drive:linear:physics:stiffness", Sdf.ValueTypeNames.Float).Set(
                ad["gripper_kp"] if left else 0.0)
            jp.CreateAttribute("drive:linear:physics:damping", Sdf.ValueTypeNames.Float).Set(
                ad["gripper_kd"] if left else 0.0)
        elif name.startswith("Wheel_"):
            jp.CreateAttribute("drive:angular:physics:stiffness", Sdf.ValueTypeNames.Float).Set(0.0)
            jp.CreateAttribute("drive:angular:physics:damping", Sdf.ValueTypeNames.Float).Set(
                r6(ad["wheel_kd"] * math.pi / 180))
            jp.CreateAttribute("drive:angular:physics:targetVelocity", Sdf.ValueTypeNames.Float).Set(0.0)
        # armature: PhysxJointAPI 속성 이름으로 직접 씀. 물리 시험은 실행 중 set_dof_armatures 로 넣었다(값은 같음)
        group = "arm" if name.startswith("RMX52_") else "gripper" if name.startswith("gripper_") else \
            "wheel" if name.startswith("Wheel_") else None
        if group:
            jp.AddAppliedSchema("PhysxJointAPI")
            jp.CreateAttribute("physxJoint:armature", Sdf.ValueTypeNames.Float).Set(A["armature"][group])
    if A["arm_ccd"] == "speculative":
        for lp in Usd.PrimRange(st.GetPrimAtPath("/World/waffle_4wd")):
            if lp.HasAPI(UsdPhysics.RigidBodyAPI) and lp.GetName().startswith(("arm_link_", "gripper_")):
                lp.AddAppliedSchema("PhysxRigidBodyAPI")
                lp.CreateAttribute("physxRigidBody:enableSpeculativeCCD", Sdf.ValueTypeNames.Bool).Set(True)
    # 조명: level 0 ~ 1 사이 선형
    dome = LIGHT_RANGE["dome"][0] + level * (LIGHT_RANGE["dome"][1] - LIGHT_RANGE["dome"][0])
    rect = LIGHT_RANGE["rect"][0] + level * (LIGHT_RANGE["rect"][1] - LIGHT_RANGE["rect"][0])
    UsdLux.DomeLight.Define(st, "/World/Lights/dome").CreateIntensityAttr(r6(dome))
    if rect > 0:
        for i, (lx, ly) in enumerate([(xs[1], cy - 0.8), (xs[3], cy - 0.8), (xs[5], cy - 0.8),
                                       (xs[1], cy + 0.8), (xs[3], cy + 0.8), (xs[5], cy + 0.8)]):
            rl = UsdLux.RectLight.Define(st, f"/World/Lights/ceiling_{i}")
            rl.CreateWidthAttr(0.6)
            rl.CreateHeightAttr(0.6)
            rl.CreateIntensityAttr(r6(rect))
            rl.AddTranslateOp().Set(Gf.Vec3d(r6(lx), r6(ly), 2.6))  # RectLight는 -Z로 빛을 낸다
    st.GetRootLayer().customLayerData = {"seed": int(p["seed"]), "task": "B_dice", "light_level": r6(level)}
    st.GetRootLayer().Save()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, required=True)
    ap.add_argument("--light-level", type=float, default=None,
                    help="0 = 가장 어두움 ~ 1 = 가장 밝음. 주지 않으면 seed 로 무작위(도메인 다양화)")
    ap.add_argument("--layout", type=Path, default=HERE / "data/maze_upper_drawing.json")
    ap.add_argument("--out", type=Path, default=None)
    args = ap.parse_args()
    out = args.out or HERE / f"worlds/b_dice/seed_{args.seed}"
    out.mkdir(parents=True, exist_ok=True)
    layout = json.loads(args.layout.read_text())
    robot = Robot(URDF_PKG / "urdf/waffle_4wd.urdf", URDF_PKG)
    p = plan(args.seed, layout, robot)
    level = args.light_level
    if level is None:   # 배치와 다른 난수열(같은 seed 면 같은 값)
        level = float(np.random.default_rng([args.seed, 7]).uniform(0, 1))
    level = min(max(level, 0.0), 1.0)
    p.update({"seed": args.seed, "light_level": r6(level)})
    usd = out / "task_world.usda"
    if usd.exists():
        usd.unlink()
    write_usd(usd, layout, p, robot, level)
    meta = {"seed": args.seed, "task": "B_dice", "light_level": r6(level), "assumptions": A,
            "sources": {"layout": str(args.layout), "layout_sha256": sha256(args.layout),
                        "robot_usd": str(ROBOT_USD), "urdf": str(URDF_PKG / "urdf/waffle_4wd.urdf"),
                        "urdf_sha256": sha256(URDF_PKG / "urdf/waffle_4wd.urdf")},
            **{k: v for k, v in p.items() if k not in ("seed", "light_level")}}
    (out / "layout.json").write_text(json.dumps(meta, indent=1, ensure_ascii=False))
    lt = p["signal_light"]
    print(f"seed {args.seed}: 순서 {lt['sequence']} (R{lt['red']} G{lt['green']} Y{lt['yellow']}, 노랑 {lt['yellow_shape']}), "
          f"선반 {[s['cell'] for s in p['shelves']]}, 바닥 퍽 {sum(pk['location'] == 'floor' for pk in p['pucks'])}, "
          f"바코드 {[(b_['text'], b_['where']) for b_ in p['barcodes']]}, 신호등 {lt['cell']}, 조명 {level:.2f}, "
          f"로봇 시도 {p['robot']['tries']}")


if __name__ == "__main__":
    main()
