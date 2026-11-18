"""261027_1711 리비전 완성 그림(정적, CAD 표시 메시): 비스듬히 · 위에서 · 왼쪽 옆에서. 부품 묶음마다 색을 나누고
바퀴 축 · 베어링 블록 핀 구멍 · 새 rail · 알루미늄 프로파일 · 전체 무게중심을 표시한다. 팔은 접은 자세 S0.
사용(PC): ~/pa-course/IsaacLab-6.1/.venv/bin/python tools/render_assembly.py <출력 png>
"""
import json
import sys
from pathlib import Path

import fast_simplification
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import trimesh  # noqa: E402
from matplotlib.collections import PolyCollection  # noqa: E402
from mpl_toolkits.mplot3d.art3d import Poly3DCollection  # noqa: E402

plt.rcParams["font.family"] = ["Noto Sans CJK JP", "DejaVu Sans"]
W = Path(__file__).resolve().parents[1]
TOP = W.parent
H = TOP / "261027_1057"
PKG = W / "urdf/waffle_4wd_description_261027_1711"
sys.path.insert(0, str(TOP / "261025_2117/rl1_arm_reach/tools"))
from urdf_fk import UrdfFK  # noqa: E402

ARM = ["RMX52_Base_Yaw", "RMX52_Shoulder", "RMX52_Elbow", "RMX52_Wrist_Pitch", "RMX52_Wrist_Roll_Reversed"]
S0 = [0.0, 1.4, -1.0, -1.3, 0.0]
fk = UrdfFK(str(PKG / "urdf/waffle_4wd.urdf"), root="base_link")
_ROOT = __import__("xml.etree.ElementTree", fromlist=["x"]).parse(PKG / "urdf/waffle_4wd.urdf").getroot()


def dec(m, target):
    if len(m.faces) <= target:
        return m
    v, f = fast_simplification.simplify(m.vertices.astype(np.float32), m.faces.astype(np.int32), 1 - target / len(m.faces))
    return trimesh.Trimesh(v, f, process=False)


def link_mesh(link, q):
    """링크 표시 메시(base_link 좌표, mm). URDF <visual><origin>을 링크 변환 뒤에 곱한다."""
    tf = fk.link_transforms({k: np.array([v], float) for k, v in zip(ARM, q)}, [link])
    vo = _ROOT.find(f"link[@name='{link}']/visual/origin")
    xyz = [float(v) for v in vo.get("xyz", "0 0 0").split()] if vo is not None else [0, 0, 0]
    rpy = [float(v) for v in vo.get("rpy", "0 0 0").split()] if vo is not None else [0, 0, 0]
    Tv = trimesh.transformations.euler_matrix(*rpy, axes="sxyz"); Tv[:3, 3] = xyz
    T = tf[link][0] @ Tv
    T[:3, 3] *= 1000
    m = trimesh.load(PKG / f"meshes/main/{link}.dae", force="mesh")
    m.apply_scale(1000); m.apply_transform(T)
    return m


groups = {}   # 이름: (색, [메시])
rep = json.loads((W / "build_base/build_report.json").read_text())
base = trimesh.load(PKG / "meshes/main/base_link.dae", force="mesh"); base.apply_scale(1000)
n_kept = rep["faces_old"] - rep["faces_removed"]
kept = base.submesh([np.arange(n_kept)], append=True)
# 전체를 한 번에 줄이면 면이 적은 상자 모양 부품(알루미늄 프로파일 · 배터리 · Jetson 외곽)이 사라진다(첫 판의 잘못).
# 연결된 덩어리마다 나눠 면이 많은 덩어리만 줄인다. 알루미늄 프로파일 2020(행 75 · 76 · 464 외곽 안)은 따로 색칠한다.
PROFILE_BOXES = [((-150.5, 119.5, 19.0), (-130.5, 139.5, 469.0)), ((-150.5, -139.5, 19.0), (-130.5, -119.5, 469.0)),
                 ((-150.5, -139.5, 469.0), (-130.5, 139.5, 489.0))]
in_prof = np.zeros(len(kept.faces), bool)
for lo, hi in PROFILE_BOXES:
    in_prof |= np.all((kept.triangles_center > np.add(lo, -0.5)) & (kept.triangles_center < np.add(hi, 0.5)), axis=1)
groups["알루미늄 프로파일 2020"] = ("#7f8fa6", [kept.submesh([np.where(in_prof)[0]], append=True)])
rest = kept.submesh([np.where(~in_prof)[0]], append=True)
comps = rest.split(only_watertight=False)
groups["차체(그대로 둔 부품)"] = ("#b9bec6", [dec(c, max(200, len(c.faces) // 12)) for c in comps])
pv3 = json.loads((H / "placed_v3/placement.json").read_text())
br = []
for e in pv3.values():
    m = trimesh.load(H / "out_TO_A" / f"{e['source_print']}_TO.stl"); m.apply_transform(np.asarray(e["T_print_to_base_link_mm"])); br.append(dec(m, 6000))
groups["층간 긴 브라켓(위상최적화 A, v3)"] = ("#2f6fb0", br)
groups["저장소 housing·magazine"] = ("#8fbf6a", [dec(trimesh.load(f), 4000) for f in sorted((H / "parts_storage").glob("*.stl"))
                                                 if f.stem.startswith(("housing", "magazine"))])
groups["새 rail(옛 자리)"] = ("#e08a1e", [dec(trimesh.load(H / f"parts_storage/{n}_new.stl"), 6000) for n in ("rail_hi", "rail_lo")])
belt = json.loads((H / "parts_belt/manifest.json").read_text())["parts"]
groups["증속 벨트 모듈"] = ("#3a9d8f", [dec(trimesh.load(H / "parts_belt" / p["file"]), 1500) for p in belt
                                       if p["name"] not in ("ballast_cup", "ballast_steel")])
groups["바퀴"] = ("#4a4a4a", [dec(link_mesh(w, S0), 3000) for w in ("wheel_front_left", "wheel_front_right", "wheel_rear_left", "wheel_rear_right")])
groups["라이다"] = ("#7a5fb0", [dec(link_mesh(w, S0), 4000) for w in ("lidar_front", "lidar_rear")])
groups["팔(S0)"] = ("#d9b44a", [dec(link_mesh(w, S0), 6000) for w in ("arm_link_1", "arm_link_2", "arm_link_3", "arm_link_4", "arm_link_5",
                                                                     "gripper_left_link", "gripper_right_link")])
rd = json.loads((PKG / "robot_data.yaml").read_text())
tot = rd["metadata"]["total_mass_kg"]
mc = np.zeros(3)
q = {k: np.array([v], float) for k, v in zip(ARM, S0)}
for ln, L in rd["links"].items():
    if not L.get("mass_kg"):
        continue
    T = fk.link_transforms(q, [ln])[ln][0]
    c = np.array([float(v) for v in L["com_link_m"].split()])
    mc += L["mass_kg"] * (T[:3, :3] @ c + T[:3, 3])
com = mc / tot * 1000

LIGHT = np.array([0.3, -0.5, 0.8]) / np.linalg.norm([0.3, -0.5, 0.8])
fig = plt.figure(figsize=(18, 13))
ax3 = fig.add_subplot(2, 2, 1, projection="3d")
for name, (col, ms) in groups.items():
    for m in ms:
        sh = np.clip(0.4 + 0.6 * np.abs(m.face_normals @ LIGHT), 0, 1)
        ax3.add_collection3d(Poly3DCollection(m.triangles, facecolors=np.outer(sh, matplotlib.colors.to_rgb(col)), edgecolor="none"))
ax3.set_xlim(-240, 240); ax3.set_ylim(-240, 240); ax3.set_zlim(-10, 520); ax3.set_box_aspect((480, 480, 530))
ax3.view_init(elev=22, azim=-130); ax3.set_xlabel("x 앞(+) mm"); ax3.set_ylabel("y 왼쪽(+)"); ax3.set_zlabel("z")
ax3.set_title("비스듬히(왼쪽 뒤에서)", fontsize=11)


def ortho(ax, axes, view_dir, title):
    i, j = axes
    polys, cols, depth = [], [], []
    for name, (col, ms) in groups.items():
        rgb = np.array(matplotlib.colors.to_rgb(col))
        for m in ms:
            t = m.triangles
            sh = np.clip(0.45 + 0.55 * np.abs(m.face_normals @ view_dir), 0, 1)
            polys.extend(t[:, :, [i, j]]); cols.extend(np.outer(sh, rgb)); depth.extend(t.mean(1) @ view_dir)
    o = np.argsort(depth)           # 먼 것부터
    ax.add_collection(PolyCollection([polys[k] for k in o], facecolors=[cols[k] for k in o], edgecolors="none"))
    ax.set_aspect("equal"); ax.set_title(title, fontsize=11); ax.grid(alpha=0.3)


axt = fig.add_subplot(2, 2, 2)
ortho(axt, (0, 1), np.array([0, 0, 1.0]), "위에서 본 모습")
axt.set_xlim(-250, 250); axt.set_ylim(-190, 190); axt.set_xlabel("x 앞(+) mm"); axt.set_ylabel("y 왼쪽(+) mm")
for x in (80, -80):
    axt.axvline(x, color="red", lw=1, ls="--")
for x in (72, 88, -72, -88):
    for y in (121.6, -121.6):
        axt.plot(x, y, "o", ms=5, mfc="none", mec="red")
axt.plot(com[0], com[1], "X", color="magenta", ms=12)
axt.annotate("바퀴 축 x ±80 (빨간 점선)\n= 베어링 블록 핀 구멍 x 72·88 (빨간 원) 가운데", (80, 160), (110, 170), fontsize=9, color="red",
             arrowprops=dict(arrowstyle="->", color="red"))
axs = fig.add_subplot(2, 2, 4)
ortho(axs, (0, 2), np.array([0, 1.0, 0]), "왼쪽 옆에서 본 모습 (팔 S0)")
axs.set_xlim(-250, 250); axs.set_ylim(-20, 530); axs.set_xlabel("x 앞(+) mm"); axs.set_ylabel("z mm")
for x in (80, -80):
    axs.axvline(x, color="red", lw=1, ls="--")
axs.plot(com[0], com[2], "X", color="magenta", ms=12)
axs.annotate("새 rail(위·아래, 깊이 87 mm)", (-90, 300), (-245, 470), fontsize=9, color="#b06000", arrowprops=dict(arrowstyle="->", color="#b06000"))
axs.annotate("알루미늄 프로파일 2020 (뒤 세로 2 + 위 가로대)", (-140, 440), (-60, 500), fontsize=9, color="#3d4a5c", arrowprops=dict(arrowstyle="->", color="#3d4a5c"))

axl = fig.add_subplot(2, 2, 3); axl.axis("off")
handles = [plt.Rectangle((0, 0), 1, 1, color=c) for c, _ in groups.values()]
axl.legend(handles + [plt.Line2D([], [], marker="X", color="magenta", ls="none", ms=10)], list(groups) + ["전체 무게중심(S0)"],
           loc="upper left", fontsize=10, frameon=False)
base_m = json.loads((W / "build_base/base_link_mass.json").read_text())
txt = (f"261027_1711 (261027_1530에서 바꾼 것)\n"
       f" - 새 rail: 옛 rail 자리 그대로(평행 이동, 맞춤 오차 0 mm), 깊이 31.6 → 87.1 mm\n"
       f" - 앞 무게추(컵 + 강철 블록) 뺌 — 벨트 자료에만 있던 부품\n"
       f" - 출력 부품 채움률: 껍질 {base_m['infill_assumption']['shell_mm']} mm + 채움 {base_m['infill_assumption']['infill'] * 100:.0f} % (가정)\n"
       f" - 바퀴 축 x ±80 mm 유지(브라켓 핀 구멍 기준), Arducam 그대로\n\n"
       f"base_link 4.6156 → {base_m['mass_kg']:.4f} kg, 전체 5.7850 → {tot:.4f} kg\n"
       f"전체 무게중심(S0) x {com[0]:.1f}, y {com[1]:.1f}, z {com[2]:.1f} mm\n"
       f"정지 앞바퀴 하중 비율(S0, 정적) {(80 + com[0]) / 160:.3f}")
axl.text(0.0, 0.35, txt, fontsize=11, va="top", family=["Noto Sans CJK JP"])
fig.suptitle("Waffle 4WD 261027_1711 완성 모습(CAD 표시 메시, 정적)", fontsize=14)
plt.tight_layout(rect=(0, 0, 1, 0.96))
plt.savefig(sys.argv[1], dpi=90)
print("saved", sys.argv[1], "com_mm", np.round(com, 1).tolist(), "mass", round(tot, 4))
