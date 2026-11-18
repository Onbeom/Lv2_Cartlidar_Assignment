"""URDF 패키지·USD 폴더를 다른 경로로 복사한 뒤에도 참조가 깨지지 않는지 확인한다(읽기만, 파일 수정 없음).

사용 (PC, Isaac Sim venv의 pxr 사용):
  ~/isaacsim-venv/bin/python -I relocation_check.py --pkg <URDF 패키지 폴더> --usd <waffle_4wd.usda>
판정: URDF 메시 미해결 0, USD 합성 오류 0, 미해결 참조 0, 모든 레이어가 USD 폴더 안에서 열림.
"""
import argparse
import json
import os
import sys
import xml.etree.ElementTree as ET

parser = argparse.ArgumentParser()
parser.add_argument('--pkg', required=True)
parser.add_argument('--usd', required=True)
parser.add_argument('--package-name', default='waffle_4wd_description_261022')
args = parser.parse_args()
pkg = os.path.realpath(args.pkg)
usd = os.path.realpath(args.usd)
usd_root = os.path.dirname(usd)
report = {'pkg': pkg, 'usd': usd}

# URDF: package:// 경로를 주어진 패키지 폴더 기준으로 풀어서 파일이 있는지 본다
urdf = os.path.join(pkg, 'urdf', 'waffle_4wd.urdf')
prefix = f'package://{args.package_name}/'
uris = [m.get('filename') for m in ET.parse(urdf).getroot().iter('mesh')]
missing = [u for u in uris if not (u.startswith(prefix) and os.path.isfile(os.path.join(pkg, u[len(prefix):])))]
report['urdf_check'] = {'mesh_refs': len(uris), 'missing': len(missing), 'missing_examples': missing[:3]}

from pxr import Usd, UsdPhysics, UsdUtils  # noqa: E402

layers, assets, unresolved = UsdUtils.ComputeAllDependencies(usd)
outside = [l.realPath for l in layers if not os.path.realpath(l.realPath).startswith(usd_root)]
stage = Usd.Stage.Open(usd)
robot = stage.GetDefaultPrim()
stage.SetEditTarget(stage.GetSessionLayer())   # variant 선택은 세션 레이어에만 한다(파일 미수정)
robot.GetVariantSets().GetVariantSet('Physics').SetVariantSelection('physx')
prims = list(stage.Traverse())
report['usd_check'] = {
    'layers': len(layers), 'assets': len(assets), 'unresolved': [str(u) for u in unresolved],
    'layers_outside_usd_folder': outside, 'composition_errors': len(stage.GetCompositionErrors()),
    'rigid_bodies': sum(p.HasAPI(UsdPhysics.RigidBodyAPI) for p in prims),
    'joints': sum(p.IsA(UsdPhysics.Joint) for p in prims),
    'colliders': sum(p.HasAPI(UsdPhysics.CollisionAPI) for p in prims),
}
ok = (not missing and not unresolved and not outside and report['usd_check']['composition_errors'] == 0)
report['ok'] = ok
print(json.dumps(report, ensure_ascii=False, indent=2))
sys.exit(0 if ok else 1)
