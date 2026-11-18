"""대조 진단: Isaac Lab 없이 Isaac Sim만으로 원본 261022_2119 USD(physx variant)의 yaw에 순수 토크를 준다.

v12 시험처럼 새 장면 + 바닥 + 로봇 참조 + 차체 고정(FixedJoint). 팔 drive 강성·감쇠 0, armature 0.045.
yaw에 EFFORT N·m를 0.4 s 준 뒤 0을 0.2 s. 물리 스텝마다 yaw 위치·속도 기록.
사용: ~/isaacsim-venv/bin/python sim_only_torque.py --effort 1.0 [--armature 0.045] [--no-fix]
"""
import argparse
import json

ap = argparse.ArgumentParser()
ap.add_argument("--effort", type=float, default=1.0)
ap.add_argument("--armature", type=float, default=0.045)
ap.add_argument("--no-fix", action="store_true")
ap.add_argument("--offset", type=float, nargs=2, default=[0.0, 0.0])
ap.add_argument("--usd", default="/home/pa4/git/lv2_next_module/261022_2119/usd/waffle_4wd_261022_2119/waffle_4wd/waffle_4wd.usda")
a = ap.parse_args()

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True, "width": 640, "height": 360, "anti_aliasing": 0, "multi_gpu": False})
import numpy as np  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.experimental.prims import Articulation  # noqa: E402
from omni.physx import get_physx_interface  # noqa: E402
from omni.physx.scripts import physicsUtils  # noqa: E402
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics  # noqa: E402

ROBOT = "/World/waffle_4wd"
ROOT = ROBOT + "/Geometry/base_footprint/base_link"
DT = 1.0 / 120.0
omni.usd.get_context().new_stage()
app.update()
stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
UsdGeom.Xform.Define(stage, "/World")
scene = UsdPhysics.Scene.Define(stage, "/World/physicsScene")
scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
scene.CreateGravityMagnitudeAttr(9.81)
PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim()).CreateTimeStepsPerSecondAttr(120)
physicsUtils.add_ground_plane(stage, "/World/ground", "Z", 25.0, Gf.Vec3f(0, 0, 0), Gf.Vec3f(0.5))
rp = stage.DefinePrim(ROBOT, "Xform")
rp.GetReferences().AddReference(a.usd)
UsdGeom.Xformable(rp).AddTranslateOp().Set(Gf.Vec3d(a.offset[0], a.offset[1], 0.0))
if rp.GetVariantSets().HasVariantSet("Physics"):
    rp.GetVariantSets().GetVariantSet("Physics").SetVariantSelection("physx")
app.update()
if not a.no_fix:
    fj = UsdPhysics.FixedJoint.Define(stage, "/World/fix_base_to_world")
    fj.CreateBody1Rel().SetTargets([Sdf.Path(ROOT)])
    pos = UsdGeom.Xformable(stage.GetPrimAtPath(ROOT)).ComputeLocalToWorldTransform(0).ExtractTranslation()
    fj.CreateLocalPos0Attr(Gf.Vec3f(*pos))
    fj.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
for p in Usd.PrimRange(stage.GetPrimAtPath(ROBOT)):
    if p.IsA(UsdPhysics.RevoluteJoint) and p.GetName().startswith("RMX52_"):
        d = UsdPhysics.DriveAPI.Get(p, "angular")
        d.CreateStiffnessAttr(0.0)
        d.CreateDampingAttr(0.0)
art = Articulation(ROOT if stage.GetPrimAtPath(ROOT).HasAPI(UsdPhysics.ArticulationRootAPI) else ROBOT + "/Geometry/base_footprint")
rec, state = [], {"k": 0}
YAW = None


def on_step(dt):
    global YAW
    if YAW is None:
        return
    k = state["k"]
    eff = np.zeros((1, len(dof)), np.float32)
    if k < 48:
        eff[0, YAW] = a.effort
    art.set_dof_efforts(eff)
    q = art.get_dof_positions().numpy()[0][YAW]
    if k in (0, 71):
        state["base%d" % k] = [round(float(x), 5) for x in art.get_world_poses()[0].numpy()[0]]
    v = art.get_dof_velocities().numpy()[0][YAW]
    rec.append([round(k * DT, 4), round(float(q), 5), round(float(v), 4)])
    state["k"] = k + 1


sub = get_physx_interface().subscribe_physics_step_events(on_step)
tl = omni.timeline.get_timeline_interface()
tl.play()
for _ in range(3):
    app.update()
dof = list(art.dof_names)
arm = [i for i, n in enumerate(dof) if n.startswith("RMX52_")]
art.set_dof_armatures([a.armature], dof_indices=arm)
YAW = dof.index("RMX52_Base_Yaw")
while state["k"] < 72:
    app.update()
q = [r[1] for r in rec]
print("RES", json.dumps({"usd": a.usd.rsplit("/", 1)[-1], "effort": a.effort, "armature": a.armature, "fixed": not a.no_fix,
                         "mean_speed_0_0.4s": round((q[47] - q[0]) / (47 * DT), 3),
                         "max_v": max(r[2] for r in rec), "base_start_end": [state.get("base0"), state.get("base71")], "v_after_release_0.1s": rec[60][2],
                         "every4": rec[::4]}), flush=True)
app.close()
