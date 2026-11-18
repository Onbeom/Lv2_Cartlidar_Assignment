import json, math, os, time, traceback
import adsk.core, adsk.fusion

HERE = os.path.dirname(os.path.abspath(__file__))
MOTION = os.path.join(os.path.dirname(HERE), 'Waffle_Assembly_Build', 'Oct19', 'LidarMotion')
NAMES = ['Base_Yaw', 'Shoulder', 'Elbow', 'Wrist_Pitch', 'Wrist_Roll_Reversed']

def report(data):
    with open(os.path.join(HERE, 'player_status.json'), 'w', encoding='utf-8') as f:
        json.dump(data, f, ensure_ascii=False, indent=2)

def prepare(app):
    design = adsk.fusion.Design.cast(app.activeProduct)
    if not design:
        raise RuntimeError('로봇 assembly 탭을 먼저 여세요.')
    root = design.rootComponent
    if not any('magazine_G' in o.component.name for o in root.occurrences):
        raise RuntimeError('원래 assembly 탭을 여세요. URDF 전용 설계는 재생 대상이 아닙니다.')
    lookup = {j.name: j for j in root.asBuiltJoints}
    joints = []
    for name in NAMES:
        joint = lookup.get('RMX52_' + name)
        if not joint or joint.isSuppressed:
            raise RuntimeError('활성 팔 관절이 필요합니다: RMX52_' + name)
        joints.append(joint)
    for occurrence in root.occurrences:
        if occurrence.component.name in ['UC684_Camera_On_Gripper', 'UC684_Camera_Bracket_PLA']:
            attachment = lookup.get(occurrence.component.name + '_Fixed_To_GripperMotor')
            if not attachment or attachment.isSuppressed:
                raise RuntimeError('카메라의 그리퍼 고정 관절을 확인하세요: ' + occurrence.component.name)
    with open(os.path.join(MOTION, 'motion_check.json'), encoding='utf-8') as f:
        if json.load(f).get('status') not in ['clear','preview_only']:
            raise RuntimeError('동작 경로의 기존 충돌 검사 결과를 확인해야 합니다.')
    with open(os.path.join(MOTION, 'motion_frames.json'), encoding='utf-8') as f:
        frames = json.load(f)
    if not frames:
        raise RuntimeError('동작 경로가 비어 있습니다.')
    for row in frames:
        if len(row['q']) != 5 or len(row['block']) != 3 or not all(
                math.isfinite(v) for v in row['q'] + row['block']):
            raise RuntimeError('동작 경로의 좌표가 올바르지 않습니다.')
        if not 16.8<=row.get('aperture_mm',40)<=72:
            raise RuntimeError('집게 벌림 범위는 16.8~72mm여야 합니다.')
    return design, joints, frames

def play(app, design, joints, frames):
    angles = [j.jointMotion.rotationValue for j in joints]
    finger=next((j for j in design.rootComponent.asBuiltJoints if j.name=='gripper_left_joint'),None)
    if finger is None:
        finger=next((j for j in design.rootComponent.asBuiltJoints if 'Gripper' in j.name and 'Left' in j.name and isinstance(j.jointMotion,adsk.fusion.SliderJointMotion)),None)
    finger_home=finger.jointMotion.slideValue if finger else None
    camera = app.activeViewport.camera
    document = app.activeDocument
    mounted = [(o, o.transform2.asArray()) for o in design.rootComponent.occurrences
               if o.component.name in ['UC684_Camera_On_Gripper', 'UC684_Camera_Bracket_PLA']]
    group = progress = None
    result = dict(version='2.1.0', frames=len(frames), played=0, status='playing', attached_camera_components=len(mounted))
    cleanup_errors = []
    try:
        group = design.rootComponent.customGraphicsGroups.add()
        group.isSelectable = False
        box = adsk.core.OrientedBoundingBox3D.create(
            adsk.core.Point3D.create(0,0,0), adsk.core.Vector3D.create(1,0,0),
            adsk.core.Vector3D.create(0,1,0), 3,3,6)
        body = adsk.fusion.TemporaryBRepManager.get().createBox(box)
        graphic = group.addBRepBody(body)
        graphic.color = adsk.fusion.CustomGraphicsSolidColorEffect.create(
            adsk.core.Color.create(255,190,30,255))
        progress = app.userInterface.createProgressDialog()
        progress.isCancelButtonShown = True
        progress.show('WaffleExport 2 — 투입·회수', '%v / %m 프레임', 0, len(frames), 0)
        start = time.monotonic()
        for index, row in enumerate(frames):
            if app.activeDocument != document or progress.wasCancelled:
                result['status'] = 'cancelled'
                break
            for joint, value in zip(joints, row['q']):
                joint.jointMotion.rotationValue = value
            if finger:
                finger.jointMotion.slideValue=(row.get('aperture_mm',40)-19.62773936566)/20
            transform = adsk.core.Matrix3D.create()
            transform.translation = adsk.core.Vector3D.create(*row['block'])
            group.transform = transform
            progress.progressValue = index + 1
            app.activeViewport.refresh()
            adsk.doEvents()
            result['played'] = index + 1
            delay = start + (index + 1)/12 - time.monotonic()
            if delay > 0:
                time.sleep(delay)
        else:
            result['status'] = 'complete'
    except Exception:
        result['status'] = 'error'
        raise
    finally:
        if finger:
            try:finger.jointMotion.slideValue=finger_home
            except Exception as error:cleanup_errors.append(str(error))
        for joint, value in zip(joints, angles):
            try:
                joint.jointMotion.rotationValue = value
                if abs(joint.jointMotion.rotationValue-value) > 1e-8:
                    cleanup_errors.append(joint.name)
            except Exception as error:
                cleanup_errors.append(str(error))
        try:
            if group and group.isValid:
                group.deleteMe()
        except Exception as error:
            cleanup_errors.append(str(error))
        if progress:
            progress.hide()
        if app.activeDocument == document:
            app.activeViewport.camera = camera
            app.activeViewport.refresh()
            adsk.doEvents()
            for occurrence, initial in mounted:
                if max(abs(a-b) for a,b in zip(initial, occurrence.transform2.asArray())) > 1e-5:
                    cleanup_errors.append('카메라 배치 복원: ' + occurrence.name)
        result['restored_all_joints'] = not cleanup_errors
        result['cleanup_errors'] = cleanup_errors
        result['temporary_graphics_removed'] = group is None or not group.isValid
        report(result)
        if cleanup_errors:
            raise RuntimeError('시작 자세 복원 확인 필요: ' + '; '.join(cleanup_errors))

def run(app, action=None):
    if action is None:
        value, cancelled = app.userInterface.inputBox(
            '1: G 저장공간 투입·회수 동작 재생 (402프레임)\n2: 재생 준비 상태 확인\n\n'
            '재생 중 취소 버튼으로 중단할 수 있습니다.\n'
            '수정된 경로의 CAD 미리보기입니다. 실제 모터 한계와 연속 충돌 검증은 완료되지 않았습니다.',
            'WaffleExport 2', '1')
        if cancelled:
            return
        action = {'1':'play', '2':'health'}.get(value.strip())
        if action is None:
            raise RuntimeError('1 또는 2를 입력하세요.')
    design, joints, frames = prepare(app)
    if action == 'health':
        report(dict(version='2.1.0', status='ready', frames=len(frames),
                    joints=[j.name for j in joints], reference_component_required=False))
        app.userInterface.messageBox('재생 준비 완료: 관절 5개, ' + str(len(frames)) +
                                    ' 프레임. 삭제한 시연용 부품은 필요하지 않습니다.')
    elif action == 'play':
        play(app, design, joints, frames)
    else:
        raise RuntimeError('지원하지 않는 동작: ' + str(action))
