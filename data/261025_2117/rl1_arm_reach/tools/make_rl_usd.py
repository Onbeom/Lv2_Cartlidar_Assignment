"""RL1용 USD 복사본 만들기(원본 261022_2119 USD는 바꾸지 않는다).

왜 복사본이 필요한가: 차체를 바닥에 고정해야 한다. Isaac Lab의 fix_root_link는 articulation 루트를 base_link에서
부모 prim(base_footprint, 강체가 아닌 Xform)으로 옮기는데, (1) 원본에서는 base_link의 newton 속성이 payload 레이어에
있어 지울 수 없어 실패하고, (2) 평탄화한 복사본에서 같은 이동을 하면 팔 관절이 강한 점성 저항을 받은 것처럼
움직인다(1 N·m에 yaw 0.135 rad/s로 일정, 힘을 끄면 즉시 멈춤. 원본은 같은 토크로 고르게 가속해 4.5 rad/s).
진단 기록: 계획_RL1_팔도달.md 10장.

이 복사본(v2 방식):
  1. Physics variant 'physx'를 골라 한 파일로 평탄화한다. articulation 루트는 base_link 그대로.
  2. base_link를 로봇 최상위 prim(/waffle_4wd)에 묶는 FixedJoint를 추가한다(articulation에서 제외, maximal 관절).
     body0을 월드가 아닌 로봇 최상위 prim으로 두어, Isaac Lab이 환경마다 복제해도 각 환경 원점에 고정된다.
     v12 물리 시험의 차체 고정(D1, 월드 고정 관절)과 같은 종류다.
그 밖의 질량·관절·drive·충돌 형상은 그대로다.

사용(PC): ~/isaacsim-venv/bin/python -I make_rl_usd.py --out ../results/usd/waffle_4wd_rl1_fixed_v2.usd
"""
import argparse
import json
from pathlib import Path

from pxr import Gf, Sdf, Usd, UsdPhysics

SRC = "/home/pa4/git/lv2_next_module/261022_2119/usd/waffle_4wd_261022_2119/waffle_4wd/waffle_4wd.usda"
TOP = "/waffle_4wd"
LINK = "/waffle_4wd/Geometry/base_footprint/base_link"
BASE_LINK_Z = 0.0067499816754000011   # URDF base_footprint_to_base_link z (m), 로봇 원점 기준 base_link 위치


def summary(stage):
    bodies = [p for p in stage.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)]
    joints = [p for p in stage.Traverse() if p.IsA(UsdPhysics.Joint)]
    mass = sum((UsdPhysics.MassAPI(p).GetMassAttr().Get() or 0.0) for p in bodies)
    roots = [str(p.GetPath()) for p in stage.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
    return {"bodies": len(bodies), "joints": len(joints), "mass_kg": round(mass, 7), "articulation_roots": roots}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    a = ap.parse_args()

    src = Usd.Stage.Open(SRC)
    src.GetDefaultPrim().GetVariantSets().GetVariantSet("Physics").SetVariantSelection("physx")
    before = summary(src)
    stage = Usd.Stage.Open(src.Flatten())

    fj = UsdPhysics.FixedJoint.Define(stage, TOP + "/Physics/base_link_to_robot_origin")
    fj.CreateBody0Rel().SetTargets([Sdf.Path(TOP)])
    fj.CreateBody1Rel().SetTargets([Sdf.Path(LINK)])
    fj.CreateLocalPos0Attr(Gf.Vec3f(0.0, 0.0, BASE_LINK_Z))
    fj.CreateLocalRot0Attr(Gf.Quatf(1.0))
    fj.CreateLocalPos1Attr(Gf.Vec3f(0.0))
    fj.CreateLocalRot1Attr(Gf.Quatf(1.0))
    fj.CreateExcludeFromArticulationAttr(True)
    fj.CreateBreakForceAttr(3.40282347e38)
    fj.CreateBreakTorqueAttr(3.40282347e38)

    out = Path(a.out).resolve()
    out.parent.mkdir(parents=True, exist_ok=True)
    stage.GetRootLayer().Export(str(out))
    after = summary(Usd.Stage.Open(str(out)))
    info = {"source": SRC, "variant": "physx", "method": "평탄화 + base_link→로봇 원점 FixedJoint(articulation 제외)",
            "before": before, "after": after}
    out.with_suffix(".json").write_text(json.dumps(info, ensure_ascii=False, indent=1))
    print(json.dumps(info, ensure_ascii=False, indent=1))


if __name__ == "__main__":
    main()
