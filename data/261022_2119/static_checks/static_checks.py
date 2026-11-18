"""정적 검사 S1·S2·S3·S5 (검증계획 1장). 파일을 읽기만 한다.

실행(PC): ~/isaacsim-venv/bin/python -I static_checks.py --urdf <URDF> --pkg <패키지 폴더> --usd <USD> --out <json>
- S1 관성: 질량 > 0, 관성 고윳값 > 0, 2·max ≤ 합 + 1e-10 (URDF)
- S2 관절: 축 길이 1(오차 1e-6), effort·velocity > 0, revolute·prismatic lower ≤ upper,
          continuous는 lower/upper 없음, mimic 대상이 있고 자기 자신이 아님 (URDF)
- S3 형상: 메시 scale > 0, 메시 파일 있음, box 치수 > 0, cylinder 반지름·길이 > 0 (URDF)
- S5 USD: Physics variant physx로 연 USD에서 S1(질량·주관성)·S2(축·한계·속도) 같은 조건, 한계는 도 → rad 환산해 URDF와 비교
판정: 성공 / 실패. 실패 항목은 이름과 값을 남긴다.
"""
import argparse
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("--urdf", required=True, type=Path)
ap.add_argument("--pkg", required=True, type=Path)
ap.add_argument("--usd", required=True, type=Path)
ap.add_argument("--out", required=True, type=Path)
a = ap.parse_args()

root = ET.parse(a.urdf).getroot()
pkg_name = root.find(".//mesh").get("filename").split("/")[2]
res = {}


def vec(s):
    return np.array([float(x) for x in s.split()])


# ---------- S1 ----------
fails, n = [], 0
for L in root.findall("link"):
    i = L.find("inertial")
    if i is None:
        continue
    n += 1
    m = float(i.find("mass").get("value"))
    t = i.find("inertia")
    x, y, z, xy, xz, yz = (float(t.get(k)) for k in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz"))
    e = np.linalg.eigvalsh([[x, xy, xz], [xy, y, yz], [xz, yz, z]])
    if not (m > 0 and e.min() > 0 and 2 * e.max() <= e.sum() + 1e-10):
        fails.append({"link": L.get("name"), "mass": m, "eig": e.tolist()})
res["S1"] = {"links_with_inertial": n, "fails": fails, "verdict": "성공" if not fails else "실패"}

# ---------- S2 ----------
fails, n = [], 0
jnames = {j.get("name") for j in root.findall("joint")}
for j in root.findall("joint"):
    kind = j.get("type")
    if kind not in ("revolute", "continuous", "prismatic"):
        continue
    n += 1
    name = j.get("name")
    ax = vec(j.find("axis").get("xyz")) if j.find("axis") is not None else None
    lim = j.find("limit")
    prob = []
    if ax is None or abs(np.linalg.norm(ax) - 1) > 1e-6:
        prob.append("axis length")
    if lim is None:
        prob.append("no limit")
    else:
        if not float(lim.get("effort", "0")) > 0:
            prob.append("effort <= 0")
        if not float(lim.get("velocity", "0")) > 0:
            prob.append("velocity <= 0")
        if kind == "continuous" and (lim.get("lower") is not None or lim.get("upper") is not None):
            prob.append("continuous has lower/upper")
        if kind != "continuous" and not float(lim.get("lower")) <= float(lim.get("upper")):
            prob.append("lower > upper")
    mm = j.find("mimic")
    if mm is not None and (mm.get("joint") not in jnames or mm.get("joint") == name):
        prob.append("bad mimic target")
    if prob:
        fails.append({"joint": name, "problems": prob})
res["S2"] = {"movable_joints": n, "fails": fails, "verdict": "성공" if not fails else "실패"}

# ---------- S3 ----------
fails = []
counts = {"mesh": 0, "box": 0, "cylinder": 0, "sphere": 0}
for g in root.iter("mesh"):
    counts["mesh"] += 1
    f = g.get("filename")
    rel = f.split(f"package://{pkg_name}/", 1)[-1]
    if not (a.pkg / rel).is_file():
        fails.append({"mesh_missing": f})
    if (vec(g.get("scale", "1 1 1")) <= 0).any():
        fails.append({"mesh_scale": f})
for g in root.iter("box"):
    counts["box"] += 1
    if (vec(g.get("size")) <= 0).any():
        fails.append({"box": g.get("size")})
for g in root.iter("cylinder"):
    counts["cylinder"] += 1
    if not (float(g.get("radius")) > 0 and float(g.get("length")) > 0):
        fails.append({"cylinder": dict(g.attrib)})
for g in root.iter("sphere"):
    counts["sphere"] += 1
    if not float(g.get("radius")) > 0:
        fails.append({"sphere": dict(g.attrib)})
res["S3"] = {"counts": counts, "fails": fails, "verdict": "성공" if not fails else "실패"}

# ---------- S5 (USD) ----------
from pxr import Gf, Usd, UsdPhysics  # noqa: E402

stage = Usd.Stage.Open(str(a.usd), load=Usd.Stage.LoadAll)
top = stage.GetDefaultPrim()
top.GetVariantSets().GetVariantSet("Physics").SetVariantSelection("physx")
fails, nb, nj = [], 0, 0
urdf_j = {j.get("name"): j for j in root.findall("joint")}
for p in Usd.PrimRange(top, Usd.TraverseInstanceProxies()):
    if p.HasAPI(UsdPhysics.MassAPI) and p.HasAPI(UsdPhysics.RigidBodyAPI):
        nb += 1
        mapi = UsdPhysics.MassAPI(p)
        m = mapi.GetMassAttr().Get()
        d = mapi.GetDiagonalInertiaAttr().Get()
        e = np.array([d[0], d[1], d[2]]) if d is not None else np.zeros(3)
        if not (m and m > 0 and e.min() > 0 and 2 * e.max() <= e.sum() + 1e-10):
            fails.append({"body": p.GetName(), "mass": m, "diag": e.tolist()})
    if p.IsA(UsdPhysics.RevoluteJoint) or p.IsA(UsdPhysics.PrismaticJoint):
        nj += 1
        name = p.GetName()
        uj = urdf_j.get(name)
        prob = []
        if uj is None:
            prob.append("not in URDF")
        else:
            rev = p.IsA(UsdPhysics.RevoluteJoint)
            api = UsdPhysics.RevoluteJoint(p) if rev else UsdPhysics.PrismaticJoint(p)
            lo, hi = api.GetLowerLimitAttr().Get(), api.GetUpperLimitAttr().Get()
            ul = uj.find("limit")
            if uj.get("type") == "continuous":
                if lo is not None and math.isfinite(lo) and hi is not None and math.isfinite(hi):
                    prob.append(f"continuous has limits {lo},{hi}")
            else:
                k = math.pi / 180 if rev else 1.0
                if lo is None or hi is None or not lo <= hi:
                    prob.append(f"lower/upper {lo},{hi}")
                elif abs(lo * k - float(ul.get("lower"))) > 1e-6 or abs(hi * k - float(ul.get("upper"))) > 1e-6:
                    prob.append(f"limit mismatch usd {lo * k},{hi * k} urdf {ul.get('lower')},{ul.get('upper')}")
            v = p.GetAttribute("newton:velocityLimit").Get()
            k = math.pi / 180 if rev else 1.0
            if v is None or not v > 0 or abs(v * k - float(ul.get("velocity"))) > 1e-5 * max(1, float(ul.get("velocity"))):
                prob.append(f"velocity usd {v} urdf {ul.get('velocity')}")
            drive = UsdPhysics.DriveAPI.Get(p, "angular" if rev else "linear")
            f = drive.GetMaxForceAttr().Get() if drive else None
            if f is None or abs(f - float(ul.get("effort"))) > 1e-6:
                prob.append(f"effort usd {f} urdf {ul.get('effort')}")
        if prob:
            fails.append({"joint": name, "problems": prob})
res["S5"] = {"rigid_bodies": nb, "movable_joints": nj, "fails": fails, "verdict": "성공" if not fails else "실패"}

res["전체"] = "성공" if all(v["verdict"] == "성공" for v in res.values() if isinstance(v, dict)) else "실패"
a.out.write_text(json.dumps(res, ensure_ascii=False, indent=1), encoding="utf-8")
print(json.dumps({k: (v["verdict"] if isinstance(v, dict) else v) for k, v in res.items()}, ensure_ascii=False))
for k, v in res.items():
    if isinstance(v, dict) and v["fails"]:
        print(k, v["fails"][:5])
