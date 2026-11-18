"""진단용 USD 변형: A = 평탄화만(루트·고정 그대로 없음), B = 평탄화 + 루트 이동(고정 관절 없음)."""
from pxr import Sdf, Usd, UsdPhysics
SRC = "/home/pa4/git/lv2_next_module/261022_2119/usd/waffle_4wd_261022_2119/waffle_4wd/waffle_4wd.usda"
LINK = "/waffle_4wd/Geometry/base_footprint/base_link"; PARENT = "/waffle_4wd/Geometry/base_footprint"
D = "/tmp/claude-1000/-home-pa4-git-lv2-next-module/dbd27a7d-3fc5-4796-9855-b04b54b49994/scratchpad/rl1diag/"
src = Usd.Stage.Open(SRC); src.GetDefaultPrim().GetVariantSets().GetVariantSet("Physics").SetVariantSelection("physx")
flat = src.Flatten()
flat.Export(D + "A_flat.usd")
st = Usd.Stage.Open(flat)
link, parent = st.GetPrimAtPath(LINK), st.GetPrimAtPath(PARENT)
p = link.GetProperty("newton:selfCollisionEnabled"); p.FlattenTo(parent); link.RemoveProperty("newton:selfCollisionEnabled")
rs = ["PhysicsArticulationRootAPI", "NewtonArticulationRootAPI"]
items = list(link.GetMetadata("apiSchemas").GetAddedOrExplicitItems())
link.SetMetadata("apiSchemas", Sdf.TokenListOp.CreateExplicit([i for i in items if i not in rs]))
parent.SetMetadata("apiSchemas", Sdf.TokenListOp.CreateExplicit(rs))
st.GetRootLayer().Export(D + "B_rootmoved.usd")
print("ok")
