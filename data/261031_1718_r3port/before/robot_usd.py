"""렌더 도구 공용: 월드의 로봇 USD 를 기준 모델로 바꾸고 차체 · 팔 자세를 넣는다 (Isaac Sim 실행 중에만 import).

월드(task_world.usda)는 만들 때의 로봇 USD(261022_2119)를 참조한다. 월드를 다시 만들면 무작위 배치가 바뀔 수 있어
월드 파일은 그대로 두고, 렌더할 때 참조만 기준 모델(261027_1711, 2026-10-27 저녁)로 바꾼다.
"""
from pathlib import Path

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom

from urdf_chain import joint_locals

TOP = Path(__file__).resolve().parents[2]
MODEL = "261027_1711"
ROBOT_USD = TOP / f"{MODEL}/usd/waffle_4wd_{MODEL}/waffle_4wd/waffle_4wd.usda"
URDF = TOP / f"{MODEL}/urdf/waffle_4wd_description_{MODEL}/urdf/waffle_4wd.urdf"
ROBOT = "/World/waffle_4wd"


def use_robot_usd(stage, usd=ROBOT_USD):
    """/World/waffle_4wd 참조를 usd 로 바꾸고 물리 variant 를 끈다(렌더 전용). 바꾼 경로를 돌려준다."""
    prim = stage.GetPrimAtPath(ROBOT)
    refs = prim.GetReferences()
    refs.ClearReferences()
    refs.AddReference(Sdf.Reference(str(Path(usd).resolve())))
    prim.GetVariantSets().GetVariantSet("Physics").SetVariantSelection("none")
    return str(Path(usd).resolve())


def set_xform(prim, T):
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    xf.AddTransformOp().Set(Gf.Matrix4d(*np.asarray(T, float).T.reshape(-1).tolist()))


def find_link(stage, name):
    hits = [p for p in Usd.PrimRange(stage.GetPrimAtPath(f"{ROBOT}/Geometry")) if p.GetName() == name and
            p.GetParent().GetName() != name]
    if len(hits) != 1:
        raise RuntimeError(f"링크 {name} 이 {len(hits)}개")
    return hits[0]


def pose_robot(stage, urdf, T_w_bf, q):
    """차체를 T_w_bf 에, 팔 · 집게 관절을 q 로 (q 에 없는 관절 0)."""
    set_xform(stage.GetPrimAtPath(ROBOT), T_w_bf)
    for link, (jname, T) in joint_locals(str(urdf), q).items():
        if jname.startswith(("RMX52_", "gripper_")):
            set_xform(find_link(stage, link), T)
