"""월드에 USD 로 써 둔 물리 수치(v12)를 PhysX 가 실제로 읽는지 확인한다. 모의 · 판정 시험 아님.

물리 시험(261022_2119/phys_tests/run_phys_test.py)은 armature 를 실행 중 API(set_dof_armatures)로 넣었고,
이 월드는 USD 속성(physxJoint:armature)으로 넣었다. 재생을 시작해 PhysX 관절 묶음에서 다시 읽어 같은지 본다.
실행 (PC): ~/isaacsim-venv/bin/python Cartlidar/sim_world/check_physics_v12.py --world Cartlidar/sim_world/worlds/b_dice/seed_1 \
    --/crashreporter/url=
"""
import argparse
import json
import math
from pathlib import Path

ap = argparse.ArgumentParser()
ap.add_argument("--world", type=Path, required=True)
args, _ = ap.parse_known_args()

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True})

import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.experimental.prims import Articulation  # noqa: E402
from pxr import PhysxSchema, Usd, UsdPhysics  # noqa: E402

lay = json.loads((args.world / "layout.json").read_text())
A = lay["assumptions"]
ctx = omni.usd.get_context()
ctx.open_stage(str((args.world / "task_world.usda").resolve()))
for _ in range(30):
    app.update()
stage = ctx.get_stage()
roots = [str(p.GetPath()) for p in Usd.PrimRange(stage.GetPrimAtPath("/World/waffle_4wd"))
         if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
art = Articulation(roots[0])
omni.timeline.get_timeline_interface().play()
for _ in range(5):
    app.update()
dof = list(art.dof_names)
arm = art.get_dof_armatures().numpy()[0]
kp, kd = (g.numpy()[0] for g in art.get_dof_gains())
expect = {"Wheel_": A["armature"]["wheel"], "RMX52_": A["armature"]["arm"], "gripper_": A["armature"]["gripper"]}
rows, ok = {}, True
for n, a, p, d in zip(dof, arm, kp, kd):
    e = next(v for k, v in expect.items() if n.startswith(k))
    good = abs(float(a) - e) < 1e-6
    ok &= good
    rows[n] = {"armature": round(float(a), 6), "expected": e, "ok": good, "kp": round(float(p), 4), "kd": round(float(d), 4)}
arm_kd = [rows[n]["kd"] for n in dof if n.startswith("RMX52_")]
mat = stage.GetPrimAtPath("/World/Looks/floor_contact")
pm = PhysxSchema.PhysxMaterialAPI(mat)
um = UsdPhysics.MaterialAPI(mat)
ccd = {str(p.GetPath()).rsplit("/", 1)[-1]: bool(PhysxSchema.PhysxRigidBodyAPI(p).GetEnableSpeculativeCCDAttr().Get())
       for p in Usd.PrimRange(stage.GetPrimAtPath("/World/waffle_4wd"))
       if p.HasAPI(UsdPhysics.RigidBodyAPI) and p.GetName().startswith(("arm_link_", "gripper_"))}
out = {"world": str(args.world), "articulation_root": roots[0], "armature_ok": ok, "dofs": rows,
       "arm_kd_si": [round(k, 3) for k in arm_kd], "arm_kd_expected": A["arm_drive"]["kd"],
       "arm_kd_ok": all(abs(k - A["arm_drive"]["kd"]) < 1e-3 for k in arm_kd),
       "friction": [um.GetStaticFrictionAttr().Get(), um.GetDynamicFrictionAttr().Get()],
       "friction_combine": pm.GetFrictionCombineModeAttr().Get(), "speculative_ccd": ccd,
       "note": "art.get_dof_gains 단위는 SI(rad) 기준으로 보고됨 — 기대값과 다르면 단위부터 확인"}
print("CHECK_PHYSICS", json.dumps(out, ensure_ascii=False))
app.close()
