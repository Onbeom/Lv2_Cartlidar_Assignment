"""렌더 도구 공용: 월드의 로봇 USD 를 기준 모델로 바꾸고 차체 · 팔 자세를 넣는다 (Isaac Sim 실행 중에만 import).

월드(task_world.usda)는 만들 때의 로봇 USD(261022_2119)를 참조한다. 월드를 다시 만들면 무작위 배치가 바뀔 수 있어
월드 파일은 그대로 두고, 렌더할 때 참조만 기준 모델(261027_1711, 2026-10-27 저녁)로 바꾼다.
새 모델 261028(2026-10-31): USD 뿌리 = base_link(base_footprint 는 그 자식, z −0.00675), 고정 관절은 USD 에서 합쳐져
link2 가 base_link 바로 아래(원점 = arm_base_joint 0.096 · 0 · 0.131)에 있다. 그래서 링크 변환은 USD 부모 링크 기준으로 다시 계산한다.
"""
from pathlib import Path

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom

from urdf_chain import fixed_transform, joint_locals, tree_root

TOP = Path(__file__).resolve().parents[2]
MODEL = "261027_1711"
ROBOT_USD = TOP / f"{MODEL}/usd/waffle_4wd_{MODEL}/waffle_4wd/waffle_4wd.usda"
URDF = TOP / f"{MODEL}/urdf/waffle_4wd_description_{MODEL}/urdf/waffle_4wd.urdf"
ROBOT = "/World/waffle_4wd"
ARM_PREFIX = ("RMX52_", "gripper_", "joint")   # 이전 모델 RMX52_* · 새 모델 joint1~5, 둘 다 gripper_*


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


def root_from_footprint(urdf):
    """base_footprint → USD 뿌리 링크 변환. 이전 모델(뿌리 = base_footprint)은 단위 행렬."""
    top = tree_root(str(urdf))
    return np.eye(4) if top == "base_footprint" else fixed_transform(str(urdf), top, root="base_footprint")


def arm_link_locals(stage, urdf, q):
    """팔 · 집게 관절 자식 링크마다 USD 부모 링크 기준 변환 {링크: 4 × 4}. 사이에 합쳐진 고정 관절(새 모델 arm_base_joint)도 넣는다."""
    out = {}
    for link, (jname, T) in joint_locals(str(urdf), q).items():
        if not jname.startswith(ARM_PREFIX):
            continue
        prim = find_link(stage, link)
        parent = prim.GetParent().GetName()
        out[link] = (prim, fixed_transform(str(urdf), link, root=parent, q=q))
    return out


def pose_robot(stage, urdf, T_w_bf, q):
    """차체 base_footprint 를 T_w_bf 에, 팔 · 집게 관절을 q 로 (q 에 없는 관절 0)."""
    set_xform(stage.GetPrimAtPath(ROBOT), np.asarray(T_w_bf, float) @ root_from_footprint(urdf))
    for prim, T in arm_link_locals(stage, urdf, q).values():
        set_xform(prim, T)
