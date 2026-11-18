"""Waffle 4WD URDF를 Isaac Sim 6.1 USD로 가져온다(창 없이).

가져오기 설정(2단계에서 합의한 값):
  - merge_fixed_joints=False   : 카메라·라이다 프레임 9개를 합치지 않고 보존한다
  - collision_from_visuals=False: URDF의 충돌 형상(box·cylinder·convex STL)을 그대로 쓴다
  - fix_base=False             : 이동 로봇이므로 바닥에 고정하지 않는다
  - allow_self_collision=True  : 팔-팔, 팔-차체 충돌로 움직임이 막히게 한다
  - 관절 목표: 바퀴 = 속도, 팔·집게 = 위치 (강성·감쇠는 바꾸지 않는다. 3단계에서 정한다)

사용 (PC):
  ~/isaacsim-venv/bin/python import_waffle_4wd.py \
      --urdf <패키지>/urdf/waffle_4wd.urdf --package-dir <패키지 폴더> --out <USD 저장 폴더> --/crashreporter/url=
결과: <USD 저장 폴더>/waffle_4wd/waffle_4wd.usda (같은 이름 폴더가 있으면 importer가 새 이름을 만든다)
"""
import argparse
import os
import sys

parser = argparse.ArgumentParser()
parser.add_argument('--urdf', required=True, help='waffle_4wd.urdf 경로')
parser.add_argument('--package-dir', required=True, help='URDF 패키지 폴더(package:// 경로 해석용)')
parser.add_argument('--package-name', help='package:// 이름. 주지 않으면 패키지 폴더의 package.xml <name>을 쓴다')
parser.add_argument('--out', required=True, help='USD를 저장할 폴더')
args, _ = parser.parse_known_args()   # 나머지(--/crashreporter/url= 등)는 Kit 설정으로 넘긴다
if not args.package_name:
    import xml.etree.ElementTree as ET
    args.package_name = ET.parse(os.path.join(args.package_dir, 'package.xml')).getroot().findtext('name').strip()

from isaacsim import SimulationApp

simulation_app = SimulationApp({'headless': True})

import isaacsim.core.experimental.utils.app as app_utils

app_utils.enable_extension('isaacsim.asset.importer.urdf')
simulation_app.update()

from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig

config = URDFImporterConfig(
    urdf_path=os.path.abspath(args.urdf),
    usd_path=os.path.abspath(args.out),
    merge_fixed_joints=False,
    collision_from_visuals=False,
    allow_self_collision=True,
    ros_package_paths=[{'name': args.package_name, 'path': os.path.abspath(args.package_dir)}],
    fix_base=False,
    joint_drive_type='force',
    joint_target_type={'Wheel_.*': 'velocity', 'RMX52_.*': 'position', 'gripper_.*': 'position'},
)
os.makedirs(config.usd_path, exist_ok=True)
exit_code = 0
try:
    usd_file = URDFImporter(config).import_urdf()
    print(f'[import] USD 생성: {usd_file} (존재: {os.path.isfile(usd_file)})', flush=True)
    if not os.path.isfile(usd_file):
        exit_code = 1
except Exception as error:   # 실패를 성공처럼 끝내지 않는다
    import traceback
    traceback.print_exc()
    print(f'[import] 실패: {error}', flush=True)
    exit_code = 1
finally:
    sys.stdout.flush()
    sys.stderr.flush()
    simulation_app.close()
    os._exit(exit_code)
