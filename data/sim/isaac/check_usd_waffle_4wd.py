"""Waffle 4WD USD를 새 프로세스에서 다시 열어 URDF 값과 비교한다(2단계 판정 ②~⑩).

USD 파일은 바꾸지 않는다. Physics variant 선택은 세션 레이어에만 하고 저장하지 않는다.

사용 (PC):
  ~/isaacsim-venv/bin/python -I check_usd_waffle_4wd.py \
      --urdf <패키지>/urdf/waffle_4wd.urdf --usd <USD 폴더>/waffle_4wd/waffle_4wd.usda [--json 결과.json]
종료 코드: 모든 판정이 성공이면 0, 아니면 1
"""
import argparse
import json
import math
import sys
import xml.etree.ElementTree as ET

import numpy as np
from pxr import Sdf, Usd, UsdGeom, UsdPhysics, UsdUtils

# 판정 허용 오차
TOL_TOTAL_MASS_KG = 1e-6
TOL_LINK_MASS_REL = 1e-6        # 링크 질량 상대 오차(USD는 float32로 저장)
TOL_COM_M = 1e-6
TOL_INERTIA_REL = 1e-5          # 주관성 모멘트 상대 오차
TOL_LIMIT_DEG = 1e-3
TOL_LIMIT_M = 1e-6
TOL_EFFORT_REL = 1e-6
TOL_VELOCITY_REL = 1e-5
TOL_AXIS_DOT = 1e-6             # 1 − (축 내적) 허용값
TOL_POS_M = 1e-4
TOL_ROT_DEG = 0.01

# 기대 개수(URDF에서 다시 세어 함께 비교한다)
EXPECTED = {
    'mass_links': 14,
    'total_mass_kg': 4.220692,  # 261027_1711 리비전(261027_1530 5.785029, 261022_2119까지 4.891194)
    'movable_joints': 11,       # continuous 4 + revolute 5 + prismatic 2
    'collision': {'Cube': 56, 'Cylinder': 4, 'Mesh': 489},   # 261027_1711 리비전(무게추 컵 상자 −1, rail 볼록 64 → 63. 261027_1530 Cube 57 · Mesh 490, 261022_2119까지 Cube 48)
    'visual_meshes': 14,
}
SENSOR_FRAMES = ['camera_optical_frame', 'top_camera_front_optical_frame',
                 'top_camera_rear_optical_frame', 'lidar_front_scan', 'lidar_rear_scan']


# ---------- 수학 도구 ----------
def rpy_to_matrix(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr]])


def quat_to_matrix(q):
    """Gf.Quat(f|d) → 3×3 회전 행렬."""
    w = q.GetReal()
    x, y, z = q.GetImaginary()
    n = math.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def homogeneous(rotation, translation):
    m = np.eye(4)
    m[:3, :3] = rotation
    m[:3, 3] = translation
    return m


def rotation_angle_deg(ra, rb):
    c = (np.trace(ra.T @ rb) - 1.0) / 2.0
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def gf_to_numpy(matrix):
    """Gf.Matrix4d(행 벡터 규약) → 열 벡터 규약 4×4."""
    return np.array([[matrix[i][j] for j in range(4)] for i in range(4)]).T


def vec(text, default='0 0 0'):
    return np.array([float(v) for v in (text or default).split()])


# ---------- URDF ----------
def load_urdf(path):
    root = ET.parse(path).getroot()
    links = {}
    for link in root.findall('link'):
        info = {'name': link.get('name'), 'inertial': None, 'visuals': 0, 'collisions': {}}
        inertial = link.find('inertial')
        if inertial is not None:
            origin = inertial.find('origin')
            i = inertial.find('inertia').attrib
            tensor = np.array([
                [float(i['ixx']), float(i['ixy']), float(i['ixz'])],
                [float(i['ixy']), float(i['iyy']), float(i['iyz'])],
                [float(i['ixz']), float(i['iyz']), float(i['izz'])]])
            rot = rpy_to_matrix(vec(origin.get('rpy') if origin is not None else None))
            info['inertial'] = {
                'mass': float(inertial.find('mass').get('value')),
                'com': vec(origin.get('xyz') if origin is not None else None),
                'tensor_link': rot @ tensor @ rot.T,
            }
        info['visuals'] = len(link.findall('visual'))
        for col in link.findall('collision'):
            tag = col.find('geometry')[0].tag
            info['collisions'][tag] = info['collisions'].get(tag, 0) + 1
        links[info['name']] = info
    joints = {}
    for joint in root.findall('joint'):
        origin = joint.find('origin')
        limit = joint.find('limit')
        mimic = joint.find('mimic')
        joints[joint.get('name')] = {
            'type': joint.get('type'),
            'parent': joint.find('parent').get('link'),
            'child': joint.find('child').get('link'),
            'xyz': vec(origin.get('xyz') if origin is not None else None),
            'rot': rpy_to_matrix(vec(origin.get('rpy') if origin is not None else None)),
            'axis': vec(joint.find('axis').get('xyz') if joint.find('axis') is not None else None, '1 0 0'),
            'limit': None if limit is None else {k: float(v) for k, v in limit.attrib.items()},
            'mimic': None if mimic is None else {
                'joint': mimic.get('joint'),
                'multiplier': float(mimic.get('multiplier', '1')),
                'offset': float(mimic.get('offset', '0'))},
        }
    children = {j['child'] for j in joints.values()}
    roots = [name for name in links if name not in children]
    assert len(roots) == 1, roots
    # 관절 0 자세에서 root 기준 각 링크 자세와 root에서의 경로
    poses, paths = {roots[0]: np.eye(4)}, {roots[0]: [roots[0]]}
    pending = list(joints.values())
    while pending:
        rest = []
        for j in pending:
            if j['parent'] in poses:
                poses[j['child']] = poses[j['parent']] @ homogeneous(j['rot'], j['xyz'])
                paths[j['child']] = paths[j['parent']] + [j['child']]
            else:
                rest.append(j)
        assert len(rest) < len(pending)
        pending = rest
    return {'root': roots[0], 'links': links, 'joints': joints, 'poses': poses, 'paths': paths}


# ---------- 판정 기록 ----------
class Report:
    def __init__(self):
        self.items = {}

    def add(self, key, ok, detail):
        self.items.setdefault(key, {'ok': True, 'details': []})
        self.items[key]['ok'] = self.items[key]['ok'] and bool(ok)
        self.items[key]['details'].append(('성공 ' if ok else '실패 ') + detail)

    def note(self, key, detail):
        self.items.setdefault(key, {'ok': True, 'details': []})
        self.items[key]['details'].append('참고 ' + detail)

    def condition(self, key, detail):
        """정적 검사로는 확정할 수 없어 동작 시험에서 확인할 조건을 적는다."""
        self.items.setdefault(key, {'ok': True, 'details': []})
        self.items[key].setdefault('conditions', []).append(detail)
        self.items[key]['details'].append('조건 ' + detail)


def api_schemas(prim):
    """합성된 apiSchemas 메타데이터. pxr 단독 환경에는 Newton·PhysX 스키마가 등록돼 있지 않아
    GetAppliedSchemas()에 나오지 않으므로 메타데이터를 직접 읽는다."""
    op = prim.GetMetadata('apiSchemas')
    return [str(t) for t in op.GetAddedOrExplicitItems()] if op else []


def attr(prim, name):
    a = prim.GetAttribute(name)
    return a.Get() if a and a.HasAuthoredValue() else None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--urdf', required=True)
    parser.add_argument('--usd', required=True)
    parser.add_argument('--variant', default='physx', help='판정에 쓸 Physics variant')
    parser.add_argument('--json', help='판정 결과를 저장할 JSON 경로')
    args = parser.parse_args()

    urdf = load_urdf(args.urdf)
    rep = Report()

    # ② 다시 열기: 합성 오류, 참조 파일, 기본 variant
    stage = Usd.Stage.Open(args.usd, Usd.Stage.LoadAll)
    rep.add('2', stage is not None, f'Usd.Stage.Open 성공 ({args.usd})')
    errors = stage.GetCompositionErrors() if hasattr(stage, 'GetCompositionErrors') else []
    rep.add('2', len(errors) == 0, f'합성 오류 {len(errors)}개')
    for e in errors[:10]:
        rep.note('2', f'합성 오류: {e}')
    _, _, unresolved = UsdUtils.ComputeAllDependencies(Sdf.AssetPath(args.usd))
    rep.add('2', len(unresolved) == 0, f'찾지 못한 참조 파일 {len(unresolved)}개 {list(unresolved)[:5]}')
    robot = stage.GetDefaultPrim()
    vset = robot.GetVariantSets().GetVariantSet('Physics')
    default_selection = vset.GetVariantSelection()
    joints_without_variant = sum(1 for p in Usd.PrimRange(robot) if p.IsA(UsdPhysics.Joint))
    rep.note('2', f'파일에 저장된 Physics variant 선택: "{default_selection}" (선택지 {vset.GetVariantNames()}), '
                  f'이 상태의 물리 관절 수 {joints_without_variant}')
    # 판정은 세션 레이어에서 variant를 골라서 한다(파일은 바꾸지 않는다)
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        vset.SetVariantSelection(args.variant)
    rep.note('2', f'판정용 variant: "{vset.GetVariantSelection()}" (세션 레이어에만 설정, 저장 안 함)')

    # ③ 단위·축
    mpu = UsdGeom.GetStageMetersPerUnit(stage)
    up = UsdGeom.GetStageUpAxis(stage)
    rep.add('3', mpu == 1.0, f'metersPerUnit = {mpu}')
    rep.add('3', up == UsdGeom.Tokens.z, f'upAxis = {up}')
    rep.note('3', f'kilogramsPerUnit = {UsdPhysics.GetStageKilogramsPerUnit(stage)}')

    # 링크 prim 찾기: /<robot>/Geometry/<URDF 경로>
    geometry_root = robot.GetPath().AppendChild('Geometry')
    link_prims = {}
    for name, path in urdf['paths'].items():
        prim_path = geometry_root
        for part in path:
            prim_path = prim_path.AppendChild(part)
        prim = stage.GetPrimAtPath(prim_path)
        link_prims[name] = prim if prim.IsValid() else None
    missing = [n for n, p in link_prims.items() if p is None]
    rep.add('2', not missing, f'URDF 링크 {len(urdf["links"])}개 중 USD prim 없음 {len(missing)}개 {missing}')

    # ④ 질량·CoM·주관성 모멘트
    mass_prims = [p for p in Usd.PrimRange(robot) if p.HasAPI(UsdPhysics.MassAPI)]
    urdf_mass_links = [n for n, l in urdf['links'].items() if l['inertial'] and l['inertial']['mass'] > 0]
    rep.add('4', len(mass_prims) == EXPECTED['mass_links'] == len(urdf_mass_links),
            f'질량 있는 링크: USD {len(mass_prims)}개, URDF {len(urdf_mass_links)}개, 기대 {EXPECTED["mass_links"]}개')
    total = 0.0
    worst = {'mass_rel': 0.0, 'com_m': 0.0, 'inertia_rel': 0.0, 'tensor_rel': 0.0}
    for name in urdf_mass_links:
        prim = link_prims.get(name)
        ref = urdf['links'][name]['inertial']
        if prim is None or not prim.HasAPI(UsdPhysics.MassAPI):
            rep.add('4', False, f'{name}: USD에 MassAPI 없음')
            continue
        mass = attr(prim, 'physics:mass')
        com = np.array(attr(prim, 'physics:centerOfMass'))
        diag = np.array(attr(prim, 'physics:diagonalInertia'))
        axes = quat_to_matrix(attr(prim, 'physics:principalAxes'))
        total += mass
        mass_rel = abs(mass - ref['mass']) / ref['mass']
        com_err = float(np.max(np.abs(com - ref['com'])))
        eig_ref = np.sort(np.linalg.eigvalsh(ref['tensor_link']))
        inertia_rel = float(np.max(np.abs(np.sort(diag) - eig_ref) / eig_ref))
        tensor_usd = axes @ np.diag(diag) @ axes.T
        tensor_rel = float(np.max(np.abs(tensor_usd - ref['tensor_link'])) / np.max(eig_ref))
        worst = {k: max(worst[k], v) for k, v in
                 zip(worst, (mass_rel, com_err, inertia_rel, tensor_rel))}
        ok = mass_rel <= TOL_LINK_MASS_REL and com_err <= TOL_COM_M and inertia_rel <= TOL_INERTIA_REL
        if not ok:
            rep.add('4', False, f'{name}: 질량 상대오차 {mass_rel:.2e}, CoM {com_err:.2e} m, 주관성 상대오차 {inertia_rel:.2e}')
    total_urdf = sum(urdf['links'][n]['inertial']['mass'] for n in urdf_mass_links)
    rep.add('4', abs(total - EXPECTED['total_mass_kg']) <= TOL_TOTAL_MASS_KG,
            f'총질량 USD {total:.7f} kg, URDF {total_urdf:.7f} kg, 기대 {EXPECTED["total_mass_kg"]} kg')
    rep.add('4', worst['mass_rel'] <= TOL_LINK_MASS_REL, f'링크 질량 최대 상대오차 {worst["mass_rel"]:.2e} (허용 {TOL_LINK_MASS_REL:g})')
    rep.add('4', worst['com_m'] <= TOL_COM_M, f'CoM 최대 오차 {worst["com_m"]:.2e} m (허용 {TOL_COM_M:g})')
    rep.add('4', worst['inertia_rel'] <= TOL_INERTIA_REL,
            f'주관성 모멘트 최대 상대오차 {worst["inertia_rel"]:.2e} (허용 {TOL_INERTIA_REL:g})')
    rep.note('4', f'링크 좌표계 관성 텐서(주축 회전 포함) 최대 오차 / 최대 주관성 = {worst["tensor_rel"]:.2e}')

    # ⑤ 관절
    joint_prims = {p.GetName(): p for p in Usd.PrimRange(robot) if p.IsA(UsdPhysics.Joint)}
    movable_urdf = {n: j for n, j in urdf['joints'].items() if j['type'] != 'fixed'}
    movable_usd = {n: p for n, p in joint_prims.items() if not p.IsA(UsdPhysics.FixedJoint)}
    rep.add('5', len(movable_usd) == len(movable_urdf) == EXPECTED['movable_joints'],
            f'가동 관절: USD {len(movable_usd)}개, URDF {len(movable_urdf)}개, 기대 {EXPECTED["movable_joints"]}개')
    # 질량 없는 프레임(카메라·scan 등)은 강체가 아니므로 물리 고정 관절 없이 Xform 계층으로만 들어간다.
    # 물리 고정 관절은 양쪽 링크가 모두 질량을 가진 URDF 고정 관절만 기대한다(프레임 자세는 ⑦에서 본다).
    def has_mass(link):
        inertial = urdf['links'][link]['inertial']
        return bool(inertial and inertial['mass'] > 0)
    fixed_rigid_urdf = sorted(n for n, j in urdf['joints'].items()
                              if j['type'] == 'fixed' and has_mass(j['parent']) and has_mass(j['child']))
    fixed_usd = sorted(n for n, p in joint_prims.items() if p.IsA(UsdPhysics.FixedJoint))
    rep.add('5', fixed_usd == fixed_rigid_urdf,
            f'물리 고정 관절: USD {fixed_usd}, URDF(양쪽 질량 있음) {fixed_rigid_urdf}')
    frame_only = sorted(n for n, j in urdf['joints'].items() if j['type'] == 'fixed' and n not in fixed_rigid_urdf)
    rep.note('5', f'질량 없는 프레임으로 이어진 URDF 고정 관절 {len(frame_only)}개는 물리 관절 없이 Xform 계층: {frame_only}')
    type_map = {'revolute': UsdPhysics.RevoluteJoint, 'continuous': UsdPhysics.RevoluteJoint,
                'prismatic': UsdPhysics.PrismaticJoint}
    unit_axes = {'X': np.array([1.0, 0, 0]), 'Y': np.array([0, 1.0, 0]), 'Z': np.array([0, 0, 1.0])}
    physx_velocity_authored = 0
    for name, ref in movable_urdf.items():
        prim = movable_usd.get(name)
        if prim is None:
            rep.add('5', False, f'{name}: USD 관절 없음')
            continue
        problems = []
        if not prim.IsA(type_map[ref['type']]):
            problems.append(f'형식 {prim.GetTypeName()}')
        body0 = prim.GetRelationship('physics:body0').GetTargets()
        body1 = prim.GetRelationship('physics:body1').GetTargets()
        if [b.name for b in body0] != [ref['parent']] or [b.name for b in body1] != [ref['child']]:
            problems.append(f'연결 {body0}→{body1}')
        pos0 = np.array(attr(prim, 'physics:localPos0'))
        rot0 = quat_to_matrix(attr(prim, 'physics:localRot0'))
        axis_usd = rot0 @ unit_axes[attr(prim, 'physics:axis')]
        axis_ref = ref['rot'] @ (ref['axis'] / np.linalg.norm(ref['axis']))
        axis_dot = float(axis_usd @ axis_ref)
        if 1.0 - axis_dot > TOL_AXIS_DOT:
            problems.append(f'축 내적 {axis_dot:.8f}')
        if np.max(np.abs(pos0 - ref['xyz'])) > TOL_POS_M:
            problems.append(f'관절 위치 {pos0} ≠ {ref["xyz"]}')
        lower, upper = attr(prim, 'physics:lowerLimit'), attr(prim, 'physics:upperLimit')
        kind = 'angular' if ref['type'] != 'prismatic' else 'linear'
        limit_text = ''
        if ref['type'] == 'continuous':
            unlimited = (lower is None or lower <= -1e30) and (upper is None or upper >= 1e30)
            if not unlimited:
                problems.append(f'continuous인데 한계 {lower}~{upper}')
            limit_text = f'한계 없음(USD {lower}~{upper})'
        elif ref['type'] == 'revolute':
            lo, hi = math.degrees(ref['limit']['lower']), math.degrees(ref['limit']['upper'])
            err = max(abs(lower - lo), abs(upper - hi))
            if err > TOL_LIMIT_DEG:
                problems.append(f'한계 오차 {err:.2e}°')
            limit_text = f'한계 {lower:.4f}~{upper:.4f}° (오차 {err:.1e}°)'
        else:
            lo, hi = ref['limit']['lower'], ref['limit']['upper']
            err = max(abs(lower - lo), abs(upper - hi))
            if err > TOL_LIMIT_M:
                problems.append(f'한계 오차 {err:.2e} m')
            limit_text = f'한계 {lower:.7f}~{upper:.7f} m (오차 {err:.1e} m)'
        effort = attr(prim, f'drive:{kind}:physics:maxForce')
        if effort is None or abs(effort - ref['limit']['effort']) > TOL_EFFORT_REL * ref['limit']['effort']:
            problems.append(f'effort {effort} ≠ {ref["limit"]["effort"]}')
        velocity = attr(prim, 'newton:velocityLimit')
        ref_velocity = ref['limit']['velocity']
        ref_velocity_usd = math.degrees(ref_velocity) if kind == 'angular' else ref_velocity
        if velocity is None or abs(velocity - ref_velocity_usd) > TOL_VELOCITY_REL * ref_velocity_usd:
            problems.append(f'newton:velocityLimit {velocity} ≠ {ref_velocity_usd}')
        physx_velocity = attr(prim, 'physxJoint:maxJointVelocity')
        if physx_velocity is not None:
            physx_velocity_authored += 1
        drive_type = attr(prim, f'drive:{kind}:physics:type')
        detail = (f'{name}: {ref["type"]}, 축 내적 {axis_dot:.8f}, {limit_text}, effort {effort}, '
                  f'속도 newton {velocity} ({"deg/s" if kind == "angular" else "m/s"}), '
                  f'physxJoint:maxJointVelocity {physx_velocity}, drive {drive_type}')
        rep.add('5', not problems, detail + (f' — 문제: {problems}' if problems else ''))
    rep.note('5', f'PhysX 관절 속도 한계(physxJoint:maxJointVelocity)가 적힌 관절 {physx_velocity_authored}개 / {len(movable_urdf)}개')
    if physx_velocity_authored < len(movable_urdf):
        rep.condition('5', '속도 한계는 newton:velocityLimit으로만 판정했다. PhysX가 이 값을 적용하는지는 동작 시험에서 확인한다')

    # ⑥ mimic
    for name, ref in urdf['joints'].items():
        if not ref['mimic']:
            continue
        prim = joint_prims.get(name)
        targets = prim.GetRelationship('newton:mimicJoint').GetTargets() if prim else []
        coef1 = attr(prim, 'newton:mimicCoef1') if prim else None
        coef0 = attr(prim, 'newton:mimicCoef0') if prim else None
        ok = ([t.name for t in targets] == [ref['mimic']['joint']] and coef1 == ref['mimic']['multiplier']
              and (coef0 or 0.0) == ref['mimic']['offset'])
        schemas = api_schemas(prim) if prim else []
        rep.add('6', ok, f'{name}: 대상 {[str(t) for t in targets]}, 계수 {coef1}, 오프셋 {coef0} '
                         f'(URDF 대상 {ref["mimic"]["joint"]}, 계수 {ref["mimic"]["multiplier"]}, 오프셋 {ref["mimic"]["offset"]})')
        rep.note('6', f'{name} 적용 스키마 {schemas}. PhysX mimic(PhysxMimicJointAPI) 적용 여부: '
                      f'{any("PhysxMimicJointAPI" in s for s in schemas)}')

    # ⑦ 프레임 자세(관절 0 자세, root 링크 기준)
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    root_world = gf_to_numpy(cache.GetLocalToWorldTransform(link_prims[urdf['root']]))
    worst_pos, worst_rot = 0.0, 0.0
    for name, prim in link_prims.items():
        if prim is None:
            continue
        rel = np.linalg.inv(root_world) @ gf_to_numpy(cache.GetLocalToWorldTransform(prim))
        ref = urdf['poses'][name]
        scale = np.linalg.norm(rel[:3, :3], axis=0)
        rot = rel[:3, :3] / scale
        pos_err = float(np.max(np.abs(rel[:3, 3] - ref[:3, 3])))
        rot_err = rotation_angle_deg(rot, ref[:3, :3])
        worst_pos, worst_rot = max(worst_pos, pos_err), max(worst_rot, rot_err)
        if name in SENSOR_FRAMES:
            rep.add('7', pos_err <= TOL_POS_M and rot_err <= TOL_ROT_DEG,
                    f'{name}: 위치 {np.round(rel[:3, 3], 6).tolist()} m, 위치 오차 {pos_err:.1e} m, 회전 오차 {rot_err:.1e}°')
        elif pos_err > TOL_POS_M or rot_err > TOL_ROT_DEG:
            rep.add('7', False, f'{name}(센서 외 링크): 위치 오차 {pos_err:.1e} m, 회전 오차 {rot_err:.1e}°')
        if np.max(np.abs(scale - 1.0)) > 1e-6:
            rep.note('7', f'{name}: 배율 {scale}')
    rep.note('7', f'링크 {len(link_prims)}개 전체 최대 오차: 위치 {worst_pos:.1e} m, 회전 {worst_rot:.1e}°')

    # ⑧ 충돌 형상, ⑨ 표시 메시 (인스턴스 내부까지 센다)
    collision = {}
    visual_mesh_prims = 0
    for prim in Usd.PrimRange(robot, Usd.TraverseInstanceProxies()):
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            key = prim.GetTypeName()
            if key == 'Mesh':
                key = f'Mesh({attr(prim, "physics:approximation")})'
            collision[key] = collision.get(key, 0) + 1
        elif prim.IsA(UsdGeom.Mesh) and UsdGeom.Imageable(prim).ComputePurpose() != UsdGeom.Tokens.guide:
            visual_mesh_prims += 1
    urdf_collision = {}
    for link in urdf['links'].values():
        for tag, n in link['collisions'].items():
            urdf_collision[tag] = urdf_collision.get(tag, 0) + n
    usd_counts = {'Cube': collision.get('Cube', 0), 'Cylinder': collision.get('Cylinder', 0),
                  'Mesh': sum(v for k, v in collision.items() if k.startswith('Mesh'))}
    rep.add('8', usd_counts == EXPECTED['collision'],
            f'USD 충돌 형상 {collision} 합계 {sum(collision.values())}, 기대 {EXPECTED["collision"]}, URDF {urdf_collision}')
    rep.add('8', collision.get('Mesh(convexHull)', 0) == EXPECTED['collision']['Mesh'],
            f'convexHull 근사 메시 {collision.get("Mesh(convexHull)", 0)}개')
    # URDF visual 하나는 링크 아래 instanceable Xform(이름 = 링크 이름) 하나가 된다
    visual_groups = [p for p in Usd.PrimRange(robot) if p.IsInstance() and p.GetName() in urdf['links']]
    urdf_visuals = sum(l['visuals'] for l in urdf['links'].values())
    rep.add('9', len(visual_groups) == urdf_visuals == EXPECTED['visual_meshes'],
            f'표시 메시 묶음 USD {len(visual_groups)}개, URDF visual {urdf_visuals}개, 기대 {EXPECTED["visual_meshes"]}개')
    rep.note('9', f'표시용 Mesh prim(guide 제외, 하위 메시 포함) {visual_mesh_prims}개')

    # ⑩ 자기충돌
    roots = [p for p in Usd.PrimRange(robot) if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
    rep.add('10', len(roots) == 1, f'ArticulationRootAPI prim {len(roots)}개 {[str(p.GetPath()) for p in roots]}')
    for prim in roots:
        newton = attr(prim, 'newton:selfCollisionEnabled')
        physx = attr(prim, 'physxArticulation:enabledSelfCollisions')
        physx_api = 'PhysxArticulationAPI' in api_schemas(prim)
        rep.add('10', newton is True or physx is True,
                f'{prim.GetName()}: newton:selfCollisionEnabled = {newton}, '
                f'physxArticulation:enabledSelfCollisions = {physx} (PhysxArticulationAPI 적용 {physx_api})')
        if physx is None:
            rep.condition('10', '자기충돌은 newton:selfCollisionEnabled로만 판정했다. PhysX가 이 값을 적용하는지는 동작 시험에서 확인한다')
    filtered = sum(1 for p in Usd.PrimRange(robot) if p.HasAPI(UsdPhysics.FilteredPairsAPI))
    rep.note('10', f'FilteredPairsAPI(충돌 제외 쌍) prim {filtered}개')

    # 출력
    all_ok = True
    for key in sorted(rep.items, key=int):
        item = rep.items[key]
        all_ok = all_ok and item['ok']
        status = '실패' if not item['ok'] else ('조건부 성공' if item.get('conditions') else '성공')
        print(f'[{key}] {status}')
        for line in item['details']:
            print(f'    {line}')
    conditional = [k for k, v in rep.items.items() if v['ok'] and v.get('conditions')]
    print(f'[전체] {"실패 있음" if not all_ok else ("성공(조건부 " + ", ".join(sorted(conditional, key=int)) + ")" if conditional else "성공")}')
    if args.json:
        with open(args.json, 'w', encoding='utf-8') as f:
            json.dump(rep.items, f, ensure_ascii=False, indent=2)
    return 0 if all_ok else 1


if __name__ == '__main__':
    sys.exit(main())
