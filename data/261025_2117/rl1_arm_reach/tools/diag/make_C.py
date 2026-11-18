from pxr import Gf, Sdf, Usd, UsdPhysics
SRC = "/home/pa4/git/lv2_next_module/261022_2119/usd/waffle_4wd_261022_2119/waffle_4wd/waffle_4wd.usda"
D = "/tmp/claude-1000/-home-pa4-git-lv2-next-module/dbd27a7d-3fc5-4796-9855-b04b54b49994/scratchpad/rl1diag/"
src = Usd.Stage.Open(SRC); src.GetDefaultPrim().GetVariantSets().GetVariantSet("Physics").SetVariantSelection("physx")
st = Usd.Stage.Open(src.Flatten())
fj = UsdPhysics.FixedJoint.Define(st, "/waffle_4wd/Physics/base_link_to_robot_origin")
fj.CreateBody0Rel().SetTargets([Sdf.Path("/waffle_4wd")])
fj.CreateBody1Rel().SetTargets([Sdf.Path("/waffle_4wd/Geometry/base_footprint/base_link")])
fj.CreateLocalPos0Attr(Gf.Vec3f(0.0, 0.0, 0.0067499816754000011)); fj.CreateLocalRot0Attr(Gf.Quatf(1.0))
fj.CreateLocalPos1Attr(Gf.Vec3f(0.0)); fj.CreateLocalRot1Attr(Gf.Quatf(1.0))
fj.CreateExcludeFromArticulationAttr(True)
st.GetRootLayer().Export(D + "C_anchor.usd"); print("ok")
