import adsk.core, adsk.fusion
import os, json, traceback, datetime, math

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(os.path.dirname(HERE), 'Waffle_Assembly_Build')

def bbox(box):
    return {'min':[box.minPoint.x,box.minPoint.y,box.minPoint.z],
            'max':[box.maxPoint.x,box.maxPoint.y,box.maxPoint.z]}

def body_info(body):
    cylinders=[]
    for face in body.faces:
        geom=face.geometry
        if isinstance(geom,adsk.core.Cylinder):
            cylinders.append({'origin':[geom.origin.x,geom.origin.y,geom.origin.z],
                'axis':[geom.axis.x,geom.axis.y,geom.axis.z], 'radius':geom.radius})
    return {'name':body.name,'bounds':bbox(body.boundingBox),
        'appearance':body.appearance.name if body.appearance else None,
        'solid':body.isSolid,'cylinders':cylinders}

def inspect_bracket(app):
    destination=app.activeDocument
    options=app.importManager.createSTEPImportOptions(os.path.join(os.path.expanduser('~'),'Downloads','braket.step'))
    doc=app.importManager.importToNewDocument(options)
    design=adsk.fusion.Design.cast(app.activeProduct)
    bodies=list(design.rootComponent.bRepBodies)
    for occ in design.rootComponent.allOccurrences:
        bodies.extend(list(occ.bRepBodies))
    report=[body_info(b) for b in bodies]
    with open(os.path.join(OUT,'bracket_inspection.json'),'w',encoding='utf-8') as f:
        json.dump(report,f,ensure_ascii=False,indent=2)
    app.userInterface.activeSelections.clear()
    for index,target in enumerate(bodies):
        for body in bodies:body.isLightBulbOn=body==target
        adsk.doEvents()
        camera=app.activeViewport.camera
        camera.viewOrientation=adsk.core.ViewOrientations.IsoTopRightViewOrientation
        camera.isFitView=True
        camera.isSmoothTransition=False
        app.activeViewport.camera=camera
        app.activeViewport.refresh()
        adsk.doEvents()
        app.activeViewport.saveAsImageFile(os.path.join(OUT,'bracket_'+str(index)+'.png'),1000,750)
    for body in bodies:body.isLightBulbOn=True
    destination.activate()

def inspect_parts(app,design):
    occurrences=list(design.rootComponent.occurrences)
    visibility=[o.isLightBulbOn for o in occurrences]
    app.userInterface.activeSelections.clear()
    camera0=app.activeViewport.camera
    try:
        for name in ['housing:1','magazine_B:1','rail_hi:1','rail_lo:1']:
            for o in occurrences:o.isLightBulbOn=o.name==name
            adsk.doEvents()
            camera=app.activeViewport.camera
            camera.viewOrientation=adsk.core.ViewOrientations.IsoTopRightViewOrientation
            camera.isFitView=True
            camera.isSmoothTransition=False
            app.activeViewport.camera=camera
            app.activeViewport.refresh()
            adsk.doEvents()
            app.activeViewport.saveAsImageFile(os.path.join(OUT,name.replace(':','_')+'.png'),1000,750)
    finally:
        for o,v in zip(occurrences,visibility):o.isLightBulbOn=v
        app.activeViewport.camera=camera0
        app.activeViewport.refresh()

def matrix(rotation=None,translation=(0,0,0)):
    rotation=rotation or [[1,0,0],[0,1,0],[0,0,1]]
    result=adsk.core.Matrix3D.create()
    result.setWithArray([rotation[0][0],rotation[0][1],rotation[0][2],translation[0],
        rotation[1][0],rotation[1][1],rotation[1][2],translation[1],
        rotation[2][0],rotation[2][1],rotation[2][2],translation[2],0,0,0,1])
    return result

def multiply(a,b):
    av,bv=a.asArray(),b.asArray()
    result=adsk.core.Matrix3D.create()
    result.setWithArray([sum(av[i*4+k]*bv[k*4+j] for k in range(4)) for i in range(4) for j in range(4)])
    return result

def color_body(design,body,name,rgb):
    appearance=design.appearances.itemByName(name)
    if not appearance:
        appearance=design.appearances.addByCopy(body.appearance,name)
        for prop in appearance.appearanceProperties:
            cp=adsk.core.ColorProperty.cast(prop)
            if cp:
                try:cp.value=adsk.core.Color.create(*rgb,255)
                except:pass
    body.appearance=appearance

def all_bodies(occ):
    result=list(occ.bRepBodies)
    for child in occ.component.allOccurrences:
        proxy=child.createForAssemblyContext(occ) if not child.assemblyContext else child
        result.extend(list(proxy.bRepBodies))
    return result

def phase_one(app,design):
    receipt=os.path.join(OUT,'phase_one.json')
    if os.path.exists(receipt):raise RuntimeError('Phase one already executed; inspect its receipt before repeating.')
    root=design.rootComponent
    byname={o.name:o for o in root.occurrences}
    required=['Motor_Left:1','Motor_Right:1','Wheel_Left:1','Wheel_Right:1',
        'Floor_1_Joined:1','Floor_3_Joined:1','LDS_Complete:1','housing:1','housing:2',
        'magazine_B:1','magazine_G:1','magazine_R:1','rail_hi:1','rail_lo:1']
    for name in required:
        if name not in byname:raise RuntimeError('Missing expected source component '+name)
    log={'status':'working','placements':[],'created':[],'warnings':[]}
    def record():
        with open(receipt,'w',encoding='utf-8') as f:json.dump(log,f,ensure_ascii=False,indent=2)
    def move(occ,delta):
        occ.transform2=multiply(delta,occ.transform2)
        log['placements'].append({'component':occ.component.name,'transform':occ.transform2.asArray()})
        record()
    record()
    for side in ['Left','Right']:
        for kind in ['Motor','Wheel']:
            original=byname[kind+'_'+side+':1']
            copied=root.occurrences.addNewComponentCopy(original.component,
                multiply(matrix(translation=(-12.8,0,.3)),original.transform2))
            copied.component.name=kind+'_Rear_'+side
            log['created'].append(copied.name)
            original.component.name=kind+'_Front_'+side
            move(original,matrix(translation=(0,0,.3)))
    move(byname['Floor_1_Joined:1'],matrix(translation=(0,0,.3)))
    move(byname['Floor_3_Joined:1'],matrix(translation=(0,0,.75)))
    lds=byname['LDS_Complete:1']
    rear=root.occurrences.addNewComponentCopy(lds.component,lds.transform2)
    rear.component.name='LDS_Rear_Underside'
    lds.component.name='LDS_Front_Underside'
    # +X is forward. Both lidars hang below the upper deck on opposite diagonal corners.
    move(lds,matrix([[-1,0,0],[0,1,0],[0,0,-1]],(7.5,7.5,18.20882121226992)))
    move(rear,matrix([[1,0,0],[0,-1,0],[0,0,-1]],(-7.5,-7.5,18.20882121226992)))
    log['created'].append(rear.name)
    red_candidates=[o for o in root.occurrences if o.name.startswith('Open CASCADE STEP translator')]
    if len(red_candidates)!=1:raise RuntimeError('Cannot uniquely identify the third housing.')
    storage=[('Blue',byname['housing:1'],byname['magazine_B:1'],8,(50,105,220)),
        ('Green',byname['housing:2'],byname['magazine_G:1'],0,(65,175,90)),
        ('Red',red_candidates[0],byname['magazine_R:1'],-8,(220,65,65))]
    rotation=[[0,-1,0],[1,0,0],[0,0,1]]
    for name,housing,magazine,y,rgb in storage:
        for kind,occ in [('Housing',housing),('Magazine',magazine)]:
            box=occ.boundingBox
            delta=matrix(rotation,(-9.3+box.maxPoint.y,y-(box.minPoint.x+box.maxPoint.x)/2,22-box.minPoint.z))
            move(occ,delta)
            # A duplicated housing shares its component with housing:1, so do not rename it here.
            if kind=='Magazine':
                for body in all_bodies(occ):color_body(design,body,'Storage_'+name,rgb)
        group=root.rigidGroups.add(adsk.core.ObjectCollection.createWithArray([housing,magazine]),True)
        group.name='Storage_'+name+'_Housing_and_Magazine'
    move(byname['rail_lo:1'],matrix(translation=(-11.8,-12.5,22)))
    move(byname['rail_hi:1'],matrix(translation=(-11.8,-12.5,14.1)))
    log['status']='complete'
    log['notes']=['Deck lower underside 3 mm; upper underside 90 mm.',
        'Wheelbase 128 mm; existing front wheel X=64 mm retained.',
        'Storage lower elevation 220 mm is a provisional layout setting.',
        'Long bracket direction is pending confirmation; no bracket placement yet.']
    record()
    app.userInterface.activeSelections.clear()
    app.activeViewport.fit()
    app.activeViewport.refresh()
    design.exportManager.execute(design.exportManager.createFusionArchiveExportOptions(os.path.join(OUT,'Stage_1_4WD_and_storage.f3d')))

def capture(design):
    if design.snapshots.hasPendingSnapshot:design.snapshots.add()

def phase_two(app,design):
    root=design.rootComponent
    report={'status':'working','created':[],'constraints':[],'warnings':[]}
    path=os.path.join(OUT,'phase_two.json')
    def record():
        with open(path,'w',encoding='utf-8') as f:json.dump(report,f,ensure_ascii=False,indent=2)
    def occurrence(name):
        matches=[o for o in root.occurrences if o.name==name]
        if len(matches)!=1:raise RuntimeError('Expected one occurrence: '+name)
        return matches[0]
    def set_transform(occ,transform):
        if occ.isGrounded:occ.isGrounded=False
        occ.transform2=transform
    record()
    # Commit all placements before creating timeline features. Superseded preliminary groups stay suppressed.
    for group in root.rigidGroups:
        if group.name.startswith('Storage_'):group.isSuppressed=True
    for end,x in [('Front',0),('Rear',-12.8)]:
        for side in ['Left','Right']:
            for kind in ['Motor','Wheel']:
                set_transform(occurrence(kind+'_'+end+'_'+side+':1'),matrix(translation=(x,0,.3)))
    set_transform(occurrence('Floor_1_Joined:1'),matrix(translation=(0,0,.3)))
    set_transform(occurrence('Floor_3_Joined:1'),matrix(translation=(0,0,.75)))
    set_transform(occurrence('LDS_Front_Underside:1'),matrix([[-1,0,0],[0,1,0],[0,0,-1]],(7.5,7.5,18.20882121226992)))
    set_transform(occurrence('LDS_Rear_Underside:1'),matrix([[1,0,0],[0,-1,0],[0,0,-1]],(-7.5,-7.5,18.20882121226992)))
    with open(os.path.join(OUT,'phase_one.json'),encoding='utf-8') as f:stage=json.load(f)
    placements=stage['placements']
    targets=['housing:1','magazine_B:1','housing:2','magazine_G:1',
        next(o.name for o in root.occurrences if o.name.startswith('Open CASCADE STEP translator')),
        'magazine_R:1','rail_lo:1','rail_hi:1']
    for name,entry in zip(targets,placements[8:]):
        transform=adsk.core.Matrix3D.create();transform.setWithArray(entry['transform'])
        set_transform(occurrence(name),transform)
    capture(design)
    design.computeAll()
    report['committed_placements']=[{'name':o.name,'bounds':bbox(o.boundingBox),'transform':o.transform2.asArray()}
        for o in root.occurrences]
    record()

    source_name='Bracket_STEP_Source'
    source=next((o for o in root.occurrences if o.component.name==source_name),None)
    if not source:
        source=root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        source.component.name=source_name
        options=app.importManager.createSTEPImportOptions(os.path.join(os.path.expanduser('~'),'Downloads','braket.step'))
        options.isViewFit=False
        if not app.importManager.importToTarget(options,source.component):raise RuntimeError('Bracket STEP import failed')
    source_bodies=all_bodies(source)
    if len(source_bodies)!=6:raise RuntimeError('Expected six source bracket bodies')
    manager=adsk.fusion.TemporaryBRepManager.get()
    def part(name,body,transform,rgb):
        existing=next((o for o in root.occurrences if o.component.name==name),None)
        if existing:return existing
        occ=root.occurrences.addNewComponent(transform)
        occ.component.name=name
        feature=occ.component.features.baseFeatures.add()
        feature.startEdit()
        created=occ.component.bRepBodies.add(manager.copy(body.nativeObject or body),feature)
        feature.finishEdit()
        created.name=name
        color_body(design,created,name+'_Appearance',rgb)
        capture(design)
        report['created'].append(name);record()
        return occ
    rotate=[[0,-1,0],[1,0,0],[0,0,1]]
    left_matrix=matrix(rotate,(14.05,14.2,0))
    right_matrix=matrix(rotate,(14.05,-3.8,0))
    bracket_left=part('Chassis_Bracket_Left',source_bodies[1],left_matrix,(190,45,75))
    bracket_right=part('Chassis_Bracket_Right',source_bodies[3],right_matrix,(190,45,75))
    socket_left=part('Profile_Socket_Rear_Left',source_bodies[0],left_matrix,(190,45,75))
    socket_right=part('Profile_Socket_Rear_Right',source_bodies[2],right_matrix,(190,45,75))
    part('Manipulator_Support_Cyan',source_bodies[4],matrix(translation=(-2.2,-1.35,9.9)),(80,185,220))
    part('Manipulator_Support_Coral',source_bodies[5],matrix(translation=(-4.625,-.075,9.9)),(235,120,110))
    source.isLightBulbOn=False
    occurrence('braket:1').isLightBulbOn=False

    for side,y in [('Left',12.95),('Right',-12.95)]:
        name='Profile_2020_450mm_Rear_'+side+'_Envelope'
        if any(o.component.name==name for o in root.occurrences):continue
        box=adsk.core.OrientedBoundingBox3D.create(adsk.core.Point3D.create(-12.8,y,23.2),
            adsk.core.Vector3D.create(1,0,0),adsk.core.Vector3D.create(0,1,0),2,2,45)
        profile=manager.createBox(box)
        part(name,profile,matrix(),(185,190,195))
    capture(design)
    # Hide the unassigned cup bracket; keep the source geometry available in the browser.
    occurrence('RW-01_roll_cup_bracket:1').isLightBulbOn=False
    fixed=[]
    for o in root.occurrences:
        if o.component.name.startswith(('Motor_','Floor_','LDS_','rail_','housing','magazine_',
            'Chassis_Bracket_','Profile_Socket_','Manipulator_Support_','Profile_2020_')) or o.name==targets[4]:
            fixed.append(o)
    if not any(g.name=='Waffle_4WD_Static_Frame' for g in root.rigidGroups):
        frame=root.rigidGroups.add(adsk.core.ObjectCollection.createWithArray(fixed),True)
        frame.name='Waffle_4WD_Static_Frame'
        report['constraints'].append(frame.name)
    occurrence('Floor_1_Joined:1').isGrounded=True
    capture(design)
    joint_sketch=root.sketches.add(root.xYConstructionPlane)
    joint_sketch.name='Wheel_Axle_Centers'
    for end,x in [('Front',6.4),('Rear',-6.4)]:
        for side,y in [('Left',14.0),('Right',-14.0)]:
            name='Wheel_Revolute_'+end+'_'+side
            if any(j.name==name for j in root.asBuiltJoints):continue
            point=joint_sketch.sketchPoints.add(adsk.core.Point3D.create(x,y,2.625))
            geom=adsk.fusion.JointGeometry.createByPoint(point)
            joint_input=root.asBuiltJoints.createInput(occurrence('Motor_'+end+'_'+side+':1'),
                occurrence('Wheel_'+end+'_'+side+':1'),geom)
            joint_input.setAsRevoluteJointMotion(adsk.fusion.JointDirections.YAxisJointDirection)
            joint=root.asBuiltJoints.add(joint_input)
            joint.name=name
            report['constraints'].append(name);record()
    joint_sketch.isVisible=False
    capture(design)
    design.computeAll()
    app.userInterface.activeSelections.clear()
    camera=app.activeViewport.camera
    camera.viewOrientation=adsk.core.ViewOrientations.IsoTopRightViewOrientation
    camera.isFitView=True
    camera.isSmoothTransition=False
    app.activeViewport.camera=camera
    app.activeViewport.refresh()
    report['parts']=[{'name':o.name,'bounds':bbox(o.boundingBox),'visible':o.isLightBulbOn,
        'transform':o.transform2.asArray()} for o in root.occurrences]
    report['warnings']=['2020 profiles are 20x20x450 mm envelopes; slot cross-section is not modeled.',
        'Storage elevation and rail spacing are provisional until mating clearances are verified.',
        'Side bracket/wheel and profile/deck interference checks are pending.']
    archive=os.path.join(OUT,'Waffle_4WD_Assembly_Layout.f3d')
    if not design.exportManager.execute(design.exportManager.createFusionArchiveExportOptions(archive)):
        raise RuntimeError('Could not export assembly archive')
    app.activeViewport.saveAsImageFile(os.path.join(OUT,'assembly_layout.png'),1600,1200)
    report['archive']=archive
    report['status']='complete'
    record()

def final_check(app,design):
    root=design.rootComponent
    def find(name):return next(o for o in root.occurrences if o.name==name)
    report={'status':'working','checks':[]}
    receipt=os.path.join(OUT,'final_check.json')
    def record():
        with open(receipt,'w',encoding='utf-8') as f:json.dump(report,f,ensure_ascii=False,indent=2)
    frame=next(g for g in root.rigidGroups if g.name=='Waffle_4WD_Static_Frame')
    frame.isSuppressed=True
    frame.timelineObject.rollTo(True)
    red=next(o for o in root.occurrences if o.name.startswith('Open CASCADE STEP translator'))
    report['red_before']={'grounded':red.isGrounded,'ground_to_parent':red.isGroundToParent,'bounds':bbox(red.boundingBox)}
    red.isGroundToParent=False
    red.isGrounded=False
    red.transform2=matrix([[0,-1,0],[1,0,0],[0,0,1]],(-4.799,-9.95,22.001))
    rotate=[[0,-1,0],[1,0,0],[0,0,1]]
    for side,y,delta in [('Left',13.3,-.9),('Right',-2.9,.9)]:
        for prefix in ['Chassis_Bracket_','Profile_Socket_Rear_']:
            target=find(prefix+side+':1')
            target.isGroundToParent=False
            target.transform2=matrix(rotate,(14.05,y,0))
        profile=find('Profile_2020_450mm_Rear_'+side+'_Envelope:1')
        profile.isGroundToParent=False
        profile.transform2=matrix(translation=(0,delta,0))
    capture(design)
    design.computeAll()
    report['red_after']=bbox(red.boundingBox)
    fixed=[o for o in root.occurrences if o.component.name.startswith(('Motor_','Floor_','LDS_','rail_','housing','magazine_',
        'Chassis_Bracket_','Profile_Socket_','Manipulator_Support_','Profile_2020_')) or o==red]
    record()
    frame.setOccurrences(adsk.core.ObjectCollection.createWithArray(fixed),True)
    design.timeline.moveToEnd()
    frame.isSuppressed=False
    for o in root.occurrences:
        if o.component.name.startswith(('Chassis_Bracket_','Profile_Socket_','Manipulator_Support_')):
            bodies=list(o.component.bRepBodies)
            if bodies:o.appearance=bodies[0].appearance
    for joint in root.asBuiltJoints:joint.isLightBulbOn=False
    capture(design)
    design.computeAll()
    report['parts']=[{'name':o.name,'bounds':bbox(o.boundingBox),'visible':o.isLightBulbOn} for o in root.occurrences]
    report['wheel_joints']=[j.name for j in root.asBuiltJoints]
    record()
    app.userInterface.activeSelections.clear()
    camera=app.activeViewport.camera
    camera.viewOrientation=adsk.core.ViewOrientations.IsoTopRightViewOrientation
    camera.isFitView=True;camera.isSmoothTransition=False
    app.activeViewport.camera=camera
    app.activeViewport.refresh();adsk.doEvents()
    design.exportManager.execute(design.exportManager.createFusionArchiveExportOptions(os.path.join(OUT,'Waffle_4WD_Assembly_Layout.f3d')))
    app.activeViewport.saveAsImageFile(os.path.join(OUT,'assembly_layout.png'),1600,1200)
    pairs=[]
    for side in ['Left','Right']:
        for end in ['Front','Rear']:
            pairs.append(('Chassis_Bracket_'+side+':1','Wheel_'+end+'_'+side+':1'))
            pairs.append(('Chassis_Bracket_'+side+':1','Motor_'+end+'_'+side+':1'))
        pairs.append(('Profile_2020_450mm_Rear_'+side+'_Envelope:1','Floor_3_Joined:1'))
    for a,b in pairs:
        row={'a':a,'b':b}
        report['checks'].append(row);record()
        try:
            inp=design.createInterferenceInput(adsk.core.ObjectCollection.createWithArray([find(a),find(b)]))
            inp.areCoincidentFacesIncluded=False
            results=design.analyzeInterference(inp)
            def entity_name(e):
                context=getattr(e,'assemblyContext',None)
                return context.fullPathName if context else getattr(e,'fullPathName',getattr(e,'name','unknown'))
            row['overlaps']=[{'one':entity_name(r.entityOne),'two':entity_name(r.entityTwo),
                'volume_mm3':r.interferenceBody.volume*1000,'bounds':bbox(r.interferenceBody.boundingBox)} for r in results] if results else []
        except:row['error']=traceback.format_exc()
        record();adsk.doEvents()
    report['status']='complete';record()


def correction_apply(app,design):
    root=design.rootComponent
    def find(name):return next(o for o in root.occurrences if o.name==name)
    receipt=os.path.join(OUT,'correction_result.json')
    report={'status':'working','placements':[],'hole_pitch_mm':16,'mount_checks':[]}
    def record():
        with open(receipt,'w',encoding='utf-8') as f:json.dump(report,f,ensure_ascii=False,indent=2)
    record()
    backup=os.path.join(OUT,'Before_user_corrections_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S')+'.f3d')
    design.exportManager.execute(design.exportManager.createFusionArchiveExportOptions(backup))
    report['backup']=backup;record()
    for j in root.asBuiltJoints:j.isSuppressed=True
    for g in root.rigidGroups:g.isSuppressed=True
    frame=next(g for g in root.rigidGroups if g.name=='Waffle_4WD_Static_Frame')
    frame.timelineObject.rollTo(True)
    def place(occ,t):
        occ.isGroundToParent=False
        if occ.isGrounded:occ.isGrounded=False
        occ.transform2=t
        report['placements'].append({'name':occ.name,'transform':t.asArray()});record()
    manager=adsk.fusion.TemporaryBRepManager.get()
    for side,y,cx in [('Left',13.3,-1.14),('Right',-2.9,-9.26)]:
        old=find('Chassis_Bracket_'+side+':1')
        source=old.component.bRepBodies.item(0)
        modified=manager.copy(source)
        def cylinder(x,y,z0,z1,r):
            return manager.createCylinderOrCone(adsk.core.Point3D.create(x,y,z0),r,
                adsk.core.Point3D.create(x,y,z1),r)
        for z in [0,8.7]:
            for middle in [4.8,12.8,20.8]:
                for offset in [-.4,.4]:
                    plug=cylinder(cx,middle+offset,z,z+.3,.103)
                    if not manager.booleanOperation(modified,plug,adsk.fusion.BooleanTypes.UnionBooleanType):
                        raise RuntimeError('Cannot fill old mounting hole')
                for offset in [-.8,.8]:
                    cut=cylinder(cx,middle+offset,z-.002,z+.302,.1025)
                    if not manager.booleanOperation(modified,cut,adsk.fusion.BooleanTypes.DifferenceBooleanType):
                        raise RuntimeError('Cannot cut 16 mm mounting pattern')
        new=root.occurrences.addNewComponent(matrix([[0,-1,0],[1,0,0],[0,0,1]],(14.05,y,1.2)))
        new.component.name='Chassis_Bracket_'+side+'_16mm'
        base=new.component.features.baseFeatures.add();base.startEdit()
        body=new.component.bRepBodies.add(modified,base);base.finishEdit()
        body.name='Bracket_16mm_pitch'
        color_body(design,body,'Chassis_Red_Corrected',(190,45,75))
        new.appearance=body.appearance
        old.isLightBulbOn=False
        place(find('Profile_Socket_Rear_'+side+':1'),matrix([[0,-1,0],[1,0,0],[0,0,1]],(14.05,y,1.2)))
        dy=-.9 if side=='Left' else .9
        place(find('Profile_2020_450mm_Rear_'+side+'_Envelope:1'),matrix(translation=(0,dy,1.2)))
    place(find('Floor_3_Joined:1'),matrix(translation=(0,0,1.95)))
    place(find('LDS_Front_Underside:1'),matrix([[-1,0,0],[0,1,0],[0,0,-1]],(7.5,7.5,19.40882121226992)))
    place(find('LDS_Rear_Underside:1'),matrix([[1,0,0],[0,-1,0],[0,0,-1]],(-7.5,-7.5,19.40882121226992)))
    place(find('Manipulator_Support_Cyan:1'),matrix(translation=(-2.2,-1.35,11.1)))
    place(find('Manipulator_Support_Coral:1'),matrix(translation=(-4.625,-.075,11.1)))
    for end,x in [('Front',9.25),('Rear',-6.75)]:
        for side,dy in [('Left',-.04),('Right',.04)]:
            transform=matrix([[0,0,1],[0,1,0],[-1,0,0]],(x-2.325,dy,9.025))
            for kind in ['Motor','Wheel']:place(find(kind+'_'+end+'_'+side+':1'),transform)
            report['mount_checks'].append({'motor':end+'_'+side,'bracket_holes_x_mm':[(x-.8)*10,(x+.8)*10],
                'motor_holes_x_mm':[(x-.8)*10,(x+.8)*10],'motor_base_z_mm':15,'bracket_top_z_mm':15})
    red=next(o for o in root.occurrences if o.name.startswith('Open CASCADE STEP translator'))
    # Exact outer clamp bore directions from both front ends of the STEP bore axes.
    theta=math.atan2(.657127,1.4588)
    sets=[('B',find('housing:1'),-theta,1.1852965,20.2994),
          ('G',find('housing:2'),0,1.3,12.5),
          ('R',red,theta,1.1852965,4.700585)]
    report['outer_rotation_deg']=math.degrees(theta)
    for letter,housing,angle,sx,sy in sets:
        a=math.pi/2+angle;c=math.cos(a);s=math.sin(a)
        rotation=[[c,-s,0],[s,c,0],[0,0,1]]
        # Mating boss centers (native X=1.95,Y=4.5) sit on the rail's two bore-end centers.
        tx=-11.8+sx-(c*1.95-s*4.5)
        ty=-12.5+sy-(s*1.95+c*4.5)
        place(housing,matrix(rotation,(tx,ty,22)))
        place(find('magazine_'+letter+':1'),matrix(rotation,(tx,ty,33)))
    place(find('rail_lo:1'),matrix(translation=(-11.8,-12.5,22)))
    place(find('rail_hi:1'),matrix(translation=(-11.8,-12.5,16.4)))
    capture(design)
    fixed=[o for o in root.occurrences if o.isLightBulbOn and (o.component.name.startswith(('Motor_','Floor_','LDS_',
        'rail_','housing','magazine_','Chassis_Bracket_','Profile_Socket_','Manipulator_Support_','Profile_2020_')) or o==red)]
    frame.setOccurrences(adsk.core.ObjectCollection.createWithArray(fixed),True)
    design.timeline.moveToEnd();frame.isSuppressed=False
    find('Floor_1_Joined:1').isGrounded=True
    capture(design);design.computeAll()
    sketch=root.sketches.add(root.xYConstructionPlane);sketch.name='Corrected_Wheel_Axle_Centers'
    for end,x in [('Front',9.25),('Rear',-6.75)]:
        for side,y in [('Left',13.96),('Right',-13.96)]:
            point=sketch.sketchPoints.add(adsk.core.Point3D.create(x,y,2.625))
            ji=root.asBuiltJoints.createInput(find('Motor_'+end+'_'+side+':1'),find('Wheel_'+end+'_'+side+':1'),adsk.fusion.JointGeometry.createByPoint(point))
            ji.setAsRevoluteJointMotion(adsk.fusion.JointDirections.YAxisJointDirection)
            joint=root.asBuiltJoints.add(ji);joint.name='Corrected_Wheel_Revolute_'+end+'_'+side;joint.isLightBulbOn=False
    sketch.isVisible=False
    capture(design);design.computeAll();adsk.doEvents()
    app.userInterface.activeSelections.clear()
    cam=app.activeViewport.camera;cam.viewOrientation=adsk.core.ViewOrientations.IsoTopRightViewOrientation
    cam.isFitView=True;cam.isSmoothTransition=False;app.activeViewport.camera=cam;app.activeViewport.refresh();adsk.doEvents()
    report['parts']=[{'name':o.name,'bounds':bbox(o.boundingBox),'visible':o.isLightBulbOn,'transform':o.transform2.asArray()} for o in root.occurrences]
    report['archive']=os.path.join(OUT,'Waffle_4WD_Corrected.f3d')
    design.exportManager.execute(design.exportManager.createFusionArchiveExportOptions(report['archive']))
    app.activeViewport.saveAsImageFile(os.path.join(OUT,'assembly_corrected.png'),1600,1200)
    report['status']='complete';record()


def correction_finish(app,design):
    root=design.rootComponent
    def find(name):return next(o for o in root.occurrences if o.name==name)
    path=os.path.join(OUT,'correction_result.json')
    with open(path,encoding='utf-8') as f:report=json.load(f)
    joints=list(root.asBuiltJoints)
    joints[0].timelineObject.rollTo(True)
    sketch=root.sketches.add(root.xYConstructionPlane);sketch.name='Corrected_Axle_References'
    points={}
    for end,x in [('Front',9.25),('Rear',-6.75)]:
        for side,y in [('Left',13.96),('Right',-13.96)]:
            points[end+'_'+side]=sketch.sketchPoints.add(adsk.core.Point3D.create(x,y,2.625))
    for joint in joints:
        key=joint.name.replace('Wheel_Revolute_','')
        if key not in points:continue
        joint.timelineObject.rollTo(True)
        joint.setAsRevoluteJointMotion(adsk.fusion.JointDirections.YAxisJointDirection,
            adsk.fusion.JointGeometry.createByPoint(points[key]))
        joint.isSuppressed=False;joint.isLightBulbOn=False
    design.timeline.moveToEnd();sketch.isVisible=False
    for sk in root.sketches:sk.isVisible=False
    capture(design);design.computeAll();adsk.doEvents()
    app.userInterface.activeSelections.clear()
    cam=app.activeViewport.camera;cam.viewOrientation=adsk.core.ViewOrientations.IsoTopRightViewOrientation
    cam.isFitView=True;cam.isSmoothTransition=False;app.activeViewport.camera=cam;app.activeViewport.refresh();adsk.doEvents()
    report['parts']=[{'name':o.name,'bounds':bbox(o.boundingBox),'visible':o.isLightBulbOn,'transform':o.transform2.asArray()} for o in root.occurrences]
    report['joints']=[{'name':j.name,'suppressed':j.isSuppressed} for j in root.asBuiltJoints]
    report['archive']=os.path.join(OUT,'Waffle_4WD_Corrected.f3d')
    design.exportManager.execute(design.exportManager.createFusionArchiveExportOptions(report['archive']))
    app.activeViewport.saveAsImageFile(os.path.join(OUT,'assembly_corrected.png'),1600,1200)
    report['status']='complete'
    with open(path,'w',encoding='utf-8') as f:json.dump(report,f,ensure_ascii=False,indent=2)


def storage_lower_and_audit(app,design):
    root=design.rootComponent
    def find(name):return next(o for o in root.occurrences if o.name==name)
    frame=next(g for g in root.rigidGroups if g.name=='Waffle_4WD_Static_Frame')
    frame.isSuppressed=True;frame.timelineObject.rollTo(True)
    red=next(o for o in root.occurrences if o.name.startswith('Open CASCADE STEP translator'))
    storage=[find('housing:1'),find('housing:2'),red]+[find('magazine_'+c+':1') for c in 'BGR']
    for o in storage:
        t=o.transform2
        values=list(t.asArray());values[11]-=11;t.setWithArray(values)
        o.isGroundToParent=False;o.transform2=t
    find('rail_lo:1').transform2=matrix(translation=(-11.8,-12.5,11))
    find('rail_hi:1').transform2=matrix(translation=(-11.8,-12.5,5.4))
    capture(design)
    frame.setOccurrences(adsk.core.ObjectCollection.createWithArray(list(frame.occurrences)),True)
    design.timeline.moveToEnd();frame.isSuppressed=False
    capture(design);design.computeAll();adsk.doEvents()
    app.userInterface.activeSelections.clear()
    cam=app.activeViewport.camera;cam.viewOrientation=adsk.core.ViewOrientations.IsoTopRightViewOrientation
    cam.isFitView=True;cam.isSmoothTransition=False;app.activeViewport.camera=cam;app.activeViewport.refresh();adsk.doEvents()
    report={'status':'working','parts':[{'name':o.name,'bounds':bbox(o.boundingBox),'visible':o.isLightBulbOn,
        'transform':o.transform2.asArray()} for o in root.occurrences],'checks':[],'hole_tests':[]}
    path=os.path.join(OUT,'corrected_validation.json')
    def record():
        with open(path,'w',encoding='utf-8') as f:json.dump(report,f,ensure_ascii=False,indent=2)
    record()
    for side,y in [('Left',12.16),('Right',-12.16)]:
        bracket=find('Chassis_Bracket_'+side+'_16mm:1')
        body=bracket.bRepBodies.item(0)
        for middle in [9.25,1.25,-6.75]:
            for offset in [-.8,.8,-.4,.4]:
                containment=body.pointContainment(adsk.core.Point3D.create(middle+offset,y,1.35))
                report['hole_tests'].append({'side':side,'x_mm':(middle+offset)*10,'y_mm':y*10,
                    'expected':'open' if abs(offset)==.8 else 'filled','containment':int(containment)})
        design.exportManager.execute(design.exportManager.createSTEPExportOptions(os.path.join(OUT,'Chassis_Bracket_'+side+'_16mm.step'),bracket.component))
    pairs=[]
    for side in ['Left','Right']:
        bracket='Chassis_Bracket_'+side+'_16mm:1'
        pairs.extend([(bracket,'Floor_1_Joined:1'),(bracket,'Floor_3_Joined:1')])
        for end in ['Front','Rear']:pairs.extend([(bracket,'Motor_'+end+'_'+side+':1'),(bracket,'Wheel_'+end+'_'+side+':1')])
    for housing,letter in [(find('housing:1'),'B'),(find('housing:2'),'G'),(red,'R')]:
        pairs.extend([(housing.name,'magazine_'+letter+':1'),(housing.name,'Floor_3_Joined:1'),
            (housing.name,'rail_lo:1'),('magazine_'+letter+':1','rail_hi:1')])
    for a,b in pairs:
        row={'a':a,'b':b,'overlap_mm3':0};report['checks'].append(row)
        for ba in all_bodies(find(a)):
            if not ba.isSolid:continue
            for bb in all_bodies(find(b)):
                if not bb.isSolid:continue
                inp=design.createInterferenceInput(adsk.core.ObjectCollection.createWithArray([ba,bb]))
                results=design.analyzeInterference(inp)
                if results:row['overlap_mm3']+=sum(r.interferenceBody.volume*1000 for r in results)
        record();adsk.doEvents()
    design.exportManager.execute(design.exportManager.createFusionArchiveExportOptions(os.path.join(OUT,'Waffle_4WD_Corrected.f3d')))
    app.activeViewport.saveAsImageFile(os.path.join(OUT,'assembly_corrected.png'),1600,1200)
    report['status']='complete';record()


def manip_inspect(app):
    destination=app.activeDocument
    options=app.importManager.createSTEPImportOptions(os.path.join(os.path.expanduser('~'),'Downloads','rm-x52.stp'))
    doc=app.importManager.importToNewDocument(options)
    design=adsk.fusion.Design.cast(app.activeProduct)
    root=design.rootComponent
    rows=[]
    for occ in root.allOccurrences:
        rows.append({'name':occ.name,'path':occ.fullPathName,'transform':occ.transform2.asArray(),
            'bounds':bbox(occ.boundingBox),'bodies':[body_info(b) for b in occ.bRepBodies]})
    with open(os.path.join(OUT,'manip_inspection.json'),'w',encoding='utf-8') as f:
        json.dump({'document':doc.name,'root_bodies':[body_info(b) for b in root.bRepBodies],'occurrences':rows},f,ensure_ascii=False,indent=2)
    for label,ori in [('iso',adsk.core.ViewOrientations.IsoTopRightViewOrientation),('front',adsk.core.ViewOrientations.FrontViewOrientation),
        ('right',adsk.core.ViewOrientations.RightViewOrientation),('top',adsk.core.ViewOrientations.TopViewOrientation)]:
        cam=app.activeViewport.camera;cam.viewOrientation=ori;cam.isFitView=True;cam.isSmoothTransition=False
        app.activeViewport.camera=cam;app.activeViewport.refresh();adsk.doEvents()
        app.activeViewport.saveAsImageFile(os.path.join(OUT,'manip_'+label+'.png'),1400,1100)
    destination.activate()


def manip_detail(app,design):
    root=design.rootComponent
    arm=next((o for o in root.occurrences if o.component.name=='RM_X52_Manipulator'),None)
    if not arm:
        arm=root.occurrences.addNewComponent(matrix());arm.component.name='RM_X52_Manipulator'
        options=app.importManager.createSTEPImportOptions(os.path.join(os.path.expanduser('~'),'Downloads','rm-x52.stp'))
        options.isViewFit=False
        app.importManager.importToTarget(options,arm.component)
    rw=next(o for o in root.occurrences if o.component.name=='RW-01_roll_cup_bracket')
    occs=list(root.occurrences);visibility=[o.isLightBulbOn for o in occs]
    child=list(arm.component.allOccurrences);childvisibility=[o.isLightBulbOn for o in child]
    camera=app.activeViewport.camera
    report={'rw_transform':rw.transform2.asArray(),'rw_bodies':[],
        'arm_tree':[{'name':o.name,'path':o.fullPathName,'bodies':o.component.bRepBodies.count,'bounds':bbox(o.boundingBox),
            'transform':o.transform2.asArray()} for o in child]}
    for proxy in all_bodies(rw):
        b=proxy.nativeObject or proxy
        report['rw_bodies'].append({'bounds':bbox(b.boundingBox),'vertices':[[v.geometry.x,v.geometry.y,v.geometry.z] for v in b.vertices],
            'faces':[{'area':f.area,'normal':[f.geometry.normal.x,f.geometry.normal.y,f.geometry.normal.z],
                'center':[f.centroid.x,f.centroid.y,f.centroid.z]} for f in b.faces if isinstance(f.geometry,adsk.core.Plane)]})
    with open(os.path.join(OUT,'manip_detail.json'),'w',encoding='utf-8') as f:json.dump(report,f)
    try:
        groups=[('cup',[rw.name],[]),('cup_and_gripper_motor',[rw.name,arm.name],['DC12_STD:2','FR12_H104:1']),
            ('storage_contact',['housing:2','rail_lo:1'],[])]
        for label,names,children in groups:
            for o in occs:o.isLightBulbOn=o.name in names
            if children:
                for o in child:o.isLightBulbOn=o.component.bRepBodies.count==0 or any(o.name.split(':')[0].startswith(n.split(':')[0]) and o.name.endswith(':'+n.split(':')[-1]) for n in children)
            for view,ori in [('iso',adsk.core.ViewOrientations.IsoTopRightViewOrientation),('top',adsk.core.ViewOrientations.TopViewOrientation),
                ('right',adsk.core.ViewOrientations.RightViewOrientation)]:
                cam=app.activeViewport.camera;cam.viewOrientation=ori;cam.isFitView=True;cam.isSmoothTransition=False
                app.activeViewport.camera=cam;app.activeViewport.refresh();adsk.doEvents()
                app.activeViewport.saveAsImageFile(os.path.join(OUT,label+'_'+view+'.png'),1300,1000)
    finally:
        for o,v in zip(occs,visibility):o.isLightBulbOn=v
        for o,v in zip(child,childvisibility):o.isLightBulbOn=v
        arm.isLightBulbOn=False
        app.activeViewport.camera=camera;app.activeViewport.refresh()


def manip_build(app,design):
    root=design.rootComponent
    if any(o.component.name.startswith('RMX52_') for o in root.occurrences):
        raise RuntimeError('Manipulator parts already exist; inspect progress before repeating.')
    log={'status':'working','created':[],'joints':[],'groups':[]}
    path=os.path.join(OUT,'manip_build.json')
    def record():
        with open(path,'w',encoding='utf-8') as f:json.dump(log,f,ensure_ascii=False,indent=2)
    record()
    def find(name):return next(o for o in root.occurrences if o.name==name)
    backup=os.path.join(OUT,'Before_wrist_build_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S')+'.f3d')
    design.exportManager.execute(design.exportManager.createFusionArchiveExportOptions(backup))
    log['backup']=backup;record()
    # Raise the storage to a working height, retaining the approved outer clamp orientations.
    frame=next(g for g in root.rigidGroups if g.name=='Waffle_4WD_Static_Frame')
    frame.isSuppressed=True;frame.timelineObject.rollTo(True)
    with open(os.path.join(OUT,'corrected_validation.json'),encoding='utf-8') as f:baseline=json.load(f)
    poses={r['name']:r['transform'] for r in baseline['parts']}
    red=next(o for o in root.occurrences if o.name.startswith('Open CASCADE STEP translator'))
    for name in ['housing:1','housing:2',red.name]+['magazine_'+c+':1' for c in 'BGR']:
        values=list(poses[name]);values[3]+=.92*values[4];values[7]-=.92*values[0]
        values[11]=14 if name.startswith('housing') or name==red.name else 25
        t=adsk.core.Matrix3D.create();t.setWithArray(values)
        occ=find(name);occ.isGroundToParent=False;occ.transform2=t
    find('rail_lo:1').transform2=matrix(translation=(-11.8,-12.5,14))
    find('rail_hi:1').transform2=matrix(translation=(-11.8,-12.5,8.4))
    capture(design)
    frame.setOccurrences(adsk.core.ObjectCollection.createWithArray(list(frame.occurrences)),True)
    design.timeline.moveToEnd();frame.isSuppressed=False
    capture(design)
    source=next(o for o in root.occurrences if o.component.name=='RM_X52_Manipulator')
    source.isLightBulbOn=False
    leaves=[o for o in source.component.allOccurrences if o.component.bRepBodies.count]
    mount=matrix([[1,0,0],[0,0,-1],[0,1,0]],(8.7,0,10.8))
    groups=[[],[],[],[],[],[]]
    assignment={'DC12_STD:1':0,'FR12_S102:1':1,'DC12_W-IDLE_ALT1:1':1,
        'FR12_H101:1':2,'ILA-A01_OPEN:1':2,'FR12_S101:1':2,'DC12_W-IDLE_ALT1:2':2,
        'FR12_H101:2':3,'ILA-A02_OPEN:1':3,'FR12_S102:2':3,'DC12_W-IDLE_ALT1:3':3,
        'FR12_H104:1':4}
    parts={}
    for leaf in leaves:
        group=assignment.get(leaf.name,5)
        local=leaf.transform2
        if group==5:local=multiply(matrix(translation=(4.74,0,0)),local)
        new=root.occurrences.addNewComponentCopy(leaf.component,multiply(mount,local))
        new.component.name='RMX52_'+leaf.name.replace(':','_')
        new.isGroundToParent=False
        parts[leaf.name]=new;groups[group].append(new)
        log['created'].append({'source':leaf.name,'name':new.name,'group':group});record()
    # Keep the gripper servo and mechanism intact downstream; add an identical DC12_STD servo upstream.
    original=next(o for o in leaves if o.name=='DC12_STD:2')
    added=root.occurrences.addNewComponentCopy(original.component,multiply(mount,original.transform2))
    added.component.name='RMX52_Added_Wrist_Roll_DC12_STD';added.isGroundToParent=False
    groups[4].append(added);log['added_motor']=added.name;record()
    rw=find('RW-01_roll_cup_bracket:1');rw.isLightBulbOn=False
    manager=adsk.fusion.TemporaryBRepManager.get()
    cup=manager.copy((all_bodies(rw)[0].nativeObject or all_bodies(rw)[0]))
    manager.transform(cup,matrix([[-1,0,0],[0,1,0],[0,0,-1]],(43.34,0,0)))
    # Four diagonal M2 clearance bores couple the carrier to four existing holes on the servo horn.
    for angle in [45,135,225,315]:
        a=math.radians(angle);y=21.05+.8*math.cos(a);z=.8*math.sin(a)
        drill=manager.createCylinderOrCone(adsk.core.Point3D.create(21.63,y,z),.11,
            adsk.core.Point3D.create(22.15,y,z),.11)
        if not manager.booleanOperation(cup,drill,adsk.fusion.BooleanTypes.DifferenceBooleanType):
            raise RuntimeError('Could not create wrist horn fastening clearance')
    drill=manager.createCylinderOrCone(adsk.core.Point3D.create(21.63,21.05,0),.35,
        adsk.core.Point3D.create(22.15,21.05,0),.35)
    manager.booleanOperation(cup,drill,adsk.fusion.BooleanTypes.DifferenceBooleanType)
    carrier=root.occurrences.addNewComponent(mount);carrier.component.name='RMX52_RW01_Roll_Carrier_M2'
    carrier.isGroundToParent=False
    bf=carrier.component.features.baseFeatures.add();bf.startEdit();body=carrier.component.bRepBodies.add(cup,bf);bf.finishEdit()
    body.name='RW01_Carrier_With_Horn_Mount'
    color_body(design,body,'Wrist_Carrier_Orange',(220,135,60));carrier.appearance=body.appearance
    groups[5].append(carrier)
    capture(design)
    for i,group in enumerate(groups):
        if len(group)>1:
            rigid=root.rigidGroups.add(adsk.core.ObjectCollection.createWithArray(group),True)
            rigid.name='RMX52_Link_'+str(i);log['groups'].append(rigid.name)
    find('Floor_1_Joined:1').isGrounded=True
    capture(design)
    ji=root.asBuiltJoints.createInput(find('Floor_3_Joined:1'),parts['DC12_STD:1'],None)
    ji.setAsRigidJointMotion();j=root.asBuiltJoints.add(ji);j.name='RMX52_Base_Fixed_To_Deck';j.isLightBulbOn=False
    # Reference points are at actual shaft axes from the source CAD geometry.
    sketch=root.sketches.add(root.xYConstructionPlane);sketch.name='RMX52_Joint_Axes'
    definitions=[('Base_Yaw',parts['DC12_STD:1'],parts['FR12_S102:1'],(8.7,0,15.235),adsk.fusion.JointDirections.ZAxisJointDirection),
        ('Shoulder',parts['DC12_W-IDLE_ALT1:1'],parts['FR12_H101:1'],(8.7,0,19.05),adsk.fusion.JointDirections.YAxisJointDirection),
        ('Elbow',parts['DC12_W-IDLE_ALT1:2'],parts['FR12_H101:2'],(11.1,0,31.85),adsk.fusion.JointDirections.YAxisJointDirection),
        ('Wrist_Pitch',parts['DC12_W-IDLE_ALT1:3'],parts['FR12_H104:1'],(23.5,0,31.85),adsk.fusion.JointDirections.YAxisJointDirection),
        ('Wrist_Roll',added,carrier,(30.34,0,31.85),adsk.fusion.JointDirections.XAxisJointDirection)]
    for name,one,two,point,axis in definitions:
        p=sketch.sketchPoints.add(adsk.core.Point3D.create(*point))
        ji=root.asBuiltJoints.createInput(one,two,adsk.fusion.JointGeometry.createByPoint(p));ji.setAsRevoluteJointMotion(axis)
        j=root.asBuiltJoints.add(ji);j.name='RMX52_'+name;j.isLightBulbOn=False;log['joints'].append(j.name);record()
    sketch.isVisible=False;capture(design);design.computeAll();adsk.doEvents()
    log['rest_transforms']={o.name:o.transform2.asArray() for group in groups for o in group}
    log['link_members']=[[o.name for o in group] for group in groups]
    log['storage_housing_bottom_mm']=140;log['magazine_mouth_z_mm']=360
    app.userInterface.activeSelections.clear()
    cam=app.activeViewport.camera;cam.viewOrientation=adsk.core.ViewOrientations.IsoTopRightViewOrientation
    cam.isFitView=True;cam.isSmoothTransition=False;app.activeViewport.camera=cam;app.activeViewport.refresh();adsk.doEvents()
    design.exportManager.execute(design.exportManager.createFusionArchiveExportOptions(os.path.join(OUT,'Waffle_4WD_Wrist_Manipulator.f3d')))
    app.activeViewport.saveAsImageFile(os.path.join(OUT,'robot_with_manipulator.png'),1800,1300)
    design.exportManager.execute(design.exportManager.createSTEPExportOptions(os.path.join(OUT,'RW01_Roll_Carrier_M2.step'),carrier.component))
    log['status']='complete';record()


def manip_repair(app,design):
    root=design.rootComponent
    def find(name):return next(o for o in root.occurrences if o.name==name)
    path=os.path.join(OUT,'manip_build.json')
    with open(path,encoding='utf-8') as f:log=json.load(f)
    source=next(o for o in root.occurrences if o.component.name=='RM_X52_Manipulator')
    leaves={o.name:o for o in source.component.allOccurrences if o.component.bRepBodies.count}
    rigid=[g for g in root.rigidGroups if g.name.startswith('RMX52_Link_')]
    rigid[0].timelineObject.rollTo(True)
    mount=matrix([[1,0,0],[0,0,-1],[0,1,0]],(8.7,0,10.8))
    for entry in log['created']:
        o=find(entry['name']);o.isGroundToParent=False
        local=leaves[entry['source']].transform2
        if entry['group']==5:local=multiply(matrix(translation=(4.74,0,0)),local)
        o.transform2=multiply(mount,local);o.isLightBulbOn=True
        for body in o.component.bRepBodies:body.isLightBulbOn=True
    added=find(log['added_motor']);added.isGroundToParent=False
    added.transform2=multiply(mount,leaves['DC12_STD:2'].transform2);added.isLightBulbOn=True
    for b in added.component.bRepBodies:b.isLightBulbOn=True
    carrier=find('RMX52_RW01_Roll_Carrier_M2:1');carrier.transform2=mount
    capture(design)
    for group in rigid:
        group.timelineObject.rollTo(True)
        group.setOccurrences(adsk.core.ObjectCollection.createWithArray(list(group.occurrences)),True)
    fixed=next(j for j in root.asBuiltJoints if j.name=='RMX52_Base_Fixed_To_Deck')
    fixed.timelineObject.rollTo(True);fixed.setAsRigidJointMotion()
    for j in root.asBuiltJoints:
        if j.name.startswith('RMX52_') and j!=fixed:
            j.timelineObject.rollTo(True)
            axis=adsk.fusion.JointDirections.ZAxisJointDirection if j.name.endswith('Base_Yaw') else (
                adsk.fusion.JointDirections.XAxisJointDirection if j.name.endswith('Wrist_Roll') else adsk.fusion.JointDirections.YAxisJointDirection)
            j.setAsRevoluteJointMotion(axis,j.geometry)
    design.timeline.moveToEnd();capture(design);design.computeAll();adsk.doEvents()
    log['rest_transforms']={o.name:o.transform2.asArray() for o in root.occurrences if o.component.name.startswith('RMX52_')}
    log['repair']='placed_after_unlocking_ground_to_parent'
    with open(path,'w',encoding='utf-8') as f:json.dump(log,f,ensure_ascii=False,indent=2)
    cam=app.activeViewport.camera;cam.viewOrientation=adsk.core.ViewOrientations.IsoTopRightViewOrientation
    cam.isFitView=True;cam.isSmoothTransition=False;app.activeViewport.camera=cam;app.activeViewport.refresh();adsk.doEvents()
    app.activeViewport.saveAsImageFile(os.path.join(OUT,'robot_with_manipulator.png'),1800,1300)
    design.exportManager.execute(design.exportManager.createFusionArchiveExportOptions(os.path.join(OUT,'Waffle_4WD_Wrist_Manipulator.f3d')))


def manip_audit(app,design):
    root=design.rootComponent
    def find(name):return next(o for o in root.occurrences if o.name==name)
    path=os.path.join(OUT,'manip_build.json')
    with open(path,encoding='utf-8') as f:log=json.load(f)
    added=next(o for o in root.occurrences if o.name in [log['added_motor'],'RMX52_DYNAMIXEL_Wrist_Roll:1']);old=log['added_motor']
    added.component.name='RMX52_DYNAMIXEL_Wrist_Roll'
    log['added_motor']=added.name
    log['link_members']=[[added.name if n==old else n for n in g] for g in log['link_members']]
    source=find('RM_X52_Manipulator:1')
    src=next(o for o in source.component.allOccurrences if o.name=='IGP-A01_OPEN:1')
    inv=src.transform2;inv.invert()
    local=adsk.core.Point3D.create(27,21.05,0);local.transformBy(inv)
    tip=find('RMX52_IGP-A01_OPEN_1:1')
    def pos():
        p=local.copy();p.transformBy(tip.transform2);return [p.x,p.y,p.z]
    names=['Base_Yaw','Shoulder','Elbow','Wrist_Pitch','Wrist_Roll']
    joints=[next(j for j in root.asBuiltJoints if j.name=='RMX52_'+n) for n in names]
    for j in joints:j.jointMotion.rotationValue=0
    design.computeAll();adsk.doEvents()
    result={'home_cm':pos(),'joint_tests':[]}
    for j in joints:
        j.jointMotion.rotationValue=.15
        design.computeAll();adsk.doEvents()
        result['joint_tests'].append({'name':j.name,'tip_cm':pos(),'value':j.jointMotion.rotationValue})
        j.jointMotion.rotationValue=0
        design.computeAll();adsk.doEvents()
    result['parts']=[{'name':o.name,'bounds':bbox(o.boundingBox),'transform':o.transform2.asArray()} for o in root.occurrences if o.isLightBulbOn]
    with open(os.path.join(OUT,'manip_audit.json'),'w',encoding='utf-8') as f:json.dump(result,f,ensure_ascii=False,indent=2)
    with open(path,'w',encoding='utf-8') as f:json.dump(log,f,ensure_ascii=False,indent=2)
    capture(design)
    app.activeViewport.refresh()


def manip_poses(app,design):
    root=design.rootComponent
    def find(name):return next(o for o in root.occurrences if o.name==name)
    names=['Base_Yaw','Shoulder','Elbow','Wrist_Pitch','Wrist_Roll_Reversed']
    joints=[next(j for j in root.asBuiltJoints if j.name=='RMX52_'+n) for n in names]
    source=find('RM_X52_Manipulator:1')
    src=next(o for o in source.component.allOccurrences if o.name=='IGP-A01_OPEN:1')
    inv=src.transform2;inv.invert()
    local=adsk.core.Point3D.create(27,21.05,0);local.transformBy(inv)
    tip=find('RMX52_IGP-A01_OPEN_1:1')
    def pos():
        p=local.copy();p.transformBy(tip.transform2);return [p.x,p.y,p.z]
    with open(os.path.join(OUT,'reach_plan.json'),encoding='utf-8') as f:plan=json.load(f)
    report={'poses':[],'checks':[]}
    def record():
        with open(os.path.join(OUT,'pose_validation.json'),'w',encoding='utf-8') as f:json.dump(report,f,ensure_ascii=False,indent=2)
    for pose in plan['poses']:
        for j,v in zip(joints,pose['joint_radians']):j.jointMotion.rotationValue=v
        adsk.doEvents()
        p=pos();row=dict(pose);row['actual_cm']=p;row['values']=[j.jointMotion.rotationValue for j in joints];row['error_mm']=10*math.dist(p,pose['target_cm']);report['poses'].append(row);record()
        if pose['name'].startswith('G_'):
            app.activeViewport.refresh();adsk.doEvents()
            app.activeViewport.saveAsImageFile(os.path.join(OUT,'pose_'+pose['name']+'.png'),1600,1200)
    for j in joints:j.jointMotion.rotationValue=0
    adsk.doEvents();capture(design)
    pairs=[('RMX52_DYNAMIXEL_Wrist_Roll:1','RMX52_RW01_Reversed_Motor_Mount:1'),('RMX52_DC12_STD_2:1','RMX52_Wrist_Horn_Adapter_3mm:1'),('RMX52_Wrist_Horn_Adapter_3mm:1','RMX52_RW01_Reversed_Motor_Mount:1')]
    red=next(o.name for o in root.occurrences if o.name.startswith('Open CASCADE STEP translator'))
    for h,c in [('housing:1','B'),('housing:2','G'),(red,'R')]:
        pairs.extend([(h,'Floor_3_Joined:1'),(h,'rail_lo:1'),('magazine_'+c+':1','rail_hi:1'),(h,'magazine_'+c+':1')])
    for a,b in pairs:
        row={'a':a,'b':b,'overlap_mm3':0}
        for ba in all_bodies(find(a)):
            if not ba.isSolid:continue
            for bb in all_bodies(find(b)):
                if not bb.isSolid:continue
                qa,qb=ba.boundingBox,bb.boundingBox
                if any(getattr(qa.maxPoint,k)<=getattr(qb.minPoint,k)+1e-7 or getattr(qb.maxPoint,k)<=getattr(qa.minPoint,k)+1e-7 for k in ['x','y','z']):continue
                results=design.analyzeInterference(design.createInterferenceInput(adsk.core.ObjectCollection.createWithArray([ba,bb])))
                if results:row['overlap_mm3']+=sum(r.interferenceBody.volume*1000 for r in results)
        report['checks'].append(row);record();adsk.doEvents()
    report['status']='complete';record()
    design.exportManager.execute(design.exportManager.createFusionArchiveExportOptions(os.path.join(OUT,'Waffle_4WD_Wrist_Manipulator.f3d')))
    app.activeViewport.refresh();adsk.doEvents()
    app.activeViewport.saveAsImageFile(os.path.join(OUT,'robot_with_manipulator.png'),1800,1300)


def wrist_reverse(app,design):
    root=design.rootComponent
    def find(n):return next(o for o in root.occurrences if o.name==n)
    with open(os.path.join(OUT,'manip_build.json'),encoding='utf-8') as f:log=json.load(f)
    groups=[next(g for g in root.rigidGroups if g.name=='RMX52_Link_'+str(i)) for i in [4,5]]
    oldjoint=next(j for j in root.asBuiltJoints if j.name=='RMX52_Wrist_Roll')
    oldjoint.isSuppressed=True
    oldcup=find('RMX52_RW01_Roll_Carrier_M2:1');oldcup.isLightBulbOn=False
    groups[0].timelineObject.rollTo(True)
    mount=matrix([[1,0,0],[0,0,-1],[0,1,0]],(8.7,0,10.8))
    source=find('RM_X52_Manipulator:1')
    leaves={o.name:o for o in source.component.allOccurrences if o.component.bRepBodies.count}
    for e in log['created']:
        if e['group']==5:
            o=find(e['name']);o.isGroundToParent=False
            o.transform2=multiply(mount,multiply(matrix(translation=(4.45,0,0)),leaves[e['source']].transform2))
    capture(design)
    groups[1].timelineObject.rollTo(True)
    members=[find(e['name']) for e in log['created'] if e['group']==5]
    groups[1].setOccurrences(adsk.core.ObjectCollection.createWithArray(members),True)
    design.timeline.moveToEnd()
    manager=adsk.fusion.TemporaryBRepManager.get()
    cup=manager.copy(all_bodies(find('RW-01_roll_cup_bracket:1'))[0])
    cutter=manager.createCylinderOrCone(adsk.core.Point3D.create(21.19,21.05,0),1.025,adsk.core.Point3D.create(21.71,21.05,0),1.025)
    manager.booleanOperation(cup,cutter,adsk.fusion.BooleanTypes.DifferenceBooleanType)
    def make(name,body,rgb):
        o=root.occurrences.addNewComponent(adsk.core.Matrix3D.create());o.component.name=name;o.isGroundToParent=False;o.transform2=mount
        capture(design)
        bf=o.component.features.baseFeatures.add();bf.startEdit();b=o.component.bRepBodies.add(body,bf);bf.finishEdit()
        color_body(design,b,name+'_Color',rgb);o.appearance=b.appearance
        return o
    newcup=make('RMX52_RW01_Reversed_Motor_Mount',cup,(220,135,60))
    adapter=manager.createCylinderOrCone(adsk.core.Point3D.create(21.75,21.05,0),1.4,adsk.core.Point3D.create(22.05,21.05,0),1.4)
    holes=[(0,0,.35)]+[(.8*math.cos(math.radians(a)),.8*math.sin(math.radians(a)),.11) for a in [45,135,225,315]]+[(y,z,.11) for y in [-.8,.8] for z in [-.8,.8]]
    for y,z,r in holes:
        drill=manager.createCylinderOrCone(adsk.core.Point3D.create(21.74,21.05+y,z),r,adsk.core.Point3D.create(22.06,21.05+y,z),r)
        manager.booleanOperation(adapter,drill,adsk.fusion.BooleanTypes.DifferenceBooleanType)
    plate=make('RMX52_Wrist_Horn_Adapter_3mm',adapter,(100,160,210))
    capture(design)
    fixed=root.rigidGroups.add(adsk.core.ObjectCollection.createWithArray([find('RMX52_DYNAMIXEL_Wrist_Roll:1'),newcup]),True);fixed.name='RMX52_Wrist_Case_Mount'
    moving=root.rigidGroups.add(adsk.core.ObjectCollection.createWithArray([members[0],plate]),True);moving.name='RMX52_Gripper_Horn_Adapter'
    sk=root.sketches.add(root.xYConstructionPlane);sk.name='RMX52_Reversed_Wrist_Axis'
    p=sk.sketchPoints.add(adsk.core.Point3D.create(30.34,0,31.85))
    ji=root.asBuiltJoints.createInput(find('RMX52_DYNAMIXEL_Wrist_Roll:1'),plate,adsk.fusion.JointGeometry.createByPoint(p));ji.setAsRevoluteJointMotion(adsk.fusion.JointDirections.XAxisJointDirection)
    j=root.asBuiltJoints.add(ji);j.name='RMX52_Wrist_Roll_Reversed';j.isLightBulbOn=False;sk.isVisible=False
    capture(design);adsk.doEvents()
    log['wrist_revision']={'cup':newcup.name,'adapter':plate.name,'gripper_offset_cm':4.45,'old_joint_suppressed':True}
    with open(os.path.join(OUT,'manip_build.json'),'w',encoding='utf-8') as f:json.dump(log,f,ensure_ascii=False,indent=2)
    app.activeViewport.refresh();adsk.doEvents();app.activeViewport.saveAsImageFile(os.path.join(OUT,'wrist_reversed.png'),1800,1300)
    design.exportManager.execute(design.exportManager.createFusionArchiveExportOptions(os.path.join(OUT,'Waffle_4WD_Wrist_Manipulator.f3d')))


def finish_and_stl(app,design):
    root=design.rootComponent;document=app.activeDocument
    def find(n):return next(o for o in root.occurrences if o.name==n)
    report={'collision_checks':[]}
    with open(os.path.join(OUT,'manip_build.json'),encoding='utf-8') as f:log=json.load(f)
    with open(os.path.join(OUT,'reach_plan.json'),encoding='utf-8') as f:plan=json.load(f)
    joints=[next(j for j in root.asBuiltJoints if j.name=='RMX52_'+n) for n in ['Base_Yaw','Shoulder','Elbow','Wrist_Pitch','Wrist_Roll_Reversed']]
    arm=[find(e['name']) for e in log['created'] if e['group']>0 and (e['source'].startswith(('DC12','FR12','ILA','IGP','IRB','ICA','IBL','ILR')))]
    arm.extend([find('RMX52_DYNAMIXEL_Wrist_Roll:1'),find('RMX52_RW01_Reversed_Motor_Mount:1'),find('RMX52_Wrist_Horn_Adapter_3mm:1')])
    red=next(o.name for o in root.occurrences if o.name.startswith('Open CASCADE STEP translator'))
    stationary=[find(n) for n in ['Floor_3_Joined:1','housing:1','housing:2',red,'magazine_B:1','magazine_G:1','magazine_R:1','rail_hi:1','rail_lo:1']]
    def overlap(ba,bb):
        qa,qb=ba.boundingBox,bb.boundingBox
        if any(getattr(qa.maxPoint,k)<=getattr(qb.minPoint,k)+1e-6 or getattr(qb.maxPoint,k)<=getattr(qa.minPoint,k)+1e-6 for k in ['x','y','z']):return 0
        rs=design.analyzeInterference(design.createInterferenceInput(adsk.core.ObjectCollection.createWithArray([ba,bb])))
        return sum(r.interferenceBody.volume*1000 for r in rs) if rs else 0
    def record():
        with open(os.path.join(OUT,'final_manip_validation.json'),'w',encoding='utf-8') as f:json.dump(report,f,ensure_ascii=False,indent=2)
    for pose in plan['poses']:
        for j,v in zip(joints,pose['joint_radians']):j.jointMotion.rotationValue=v
        adsk.doEvents()
        hits=[]
        for a in arm:
            for b in stationary:
                total=0
                for ba in a.bRepBodies:
                    if not ba.isSolid:continue
                    for bb in all_bodies(b):
                        if bb.isSolid:total+=overlap(ba,bb)
                if total>.01:hits.append({'a':a.name,'b':b.name,'overlap_mm3':total})
        report['collision_checks'].append({'pose':pose['name'],'hits':hits});record();adsk.doEvents()
    for j in joints:j.jointMotion.rotationValue=0
    adsk.doEvents()
    joints[-1].jointMotion.rotationValue=math.pi/4;adsk.doEvents()
    report['wrist_roll_45deg_actual']=joints[-1].jointMotion.rotationValue
    app.activeViewport.refresh();adsk.doEvents();app.activeViewport.saveAsImageFile(os.path.join(OUT,'wrist_roll_45deg.png'),1600,1200)
    joints[-1].jointMotion.rotationValue=0;adsk.doEvents();capture(design)
    design.exportManager.execute(design.exportManager.createFusionArchiveExportOptions(os.path.join(OUT,'Waffle_4WD_Wrist_Manipulator.f3d')))
    app.activeViewport.refresh();adsk.doEvents();app.activeViewport.saveAsImageFile(os.path.join(OUT,'robot_with_manipulator.png'),1800,1300)
    for name in ['RMX52_RW01_Reversed_Motor_Mount:1','RMX52_Wrist_Horn_Adapter_3mm:1']:
        o=find(name);design.exportManager.execute(design.exportManager.createSTEPExportOptions(os.path.join(OUT,o.component.name+'.step'),o.component))
    manager=adsk.fusion.TemporaryBRepManager.get();parts=[]
    for side,dx in [('Left',4.2),('Right',15.4)]:
        o=find('Chassis_Bracket_'+side+'_16mm:1');b=manager.copy(o.component.bRepBodies.item(0));manager.transform(b,matrix(translation=(dx,0,0)));parts.append(b)
    temp=app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    td=adsk.fusion.Design.cast(app.activeProduct);td.designType=adsk.fusion.DesignTypes.DirectDesignType
    for b in parts:td.rootComponent.bRepBodies.add(b)
    filename=os.path.join(os.path.expanduser('~'),'Downloads','floor_braket.stl')
    options=td.exportManager.createSTLExportOptions(td.rootComponent,filename)
    options.unitType=adsk.fusion.DistanceUnits.MillimeterDistanceUnits;options.meshRefinement=adsk.fusion.MeshRefinementSettings.MeshRefinementHigh;options.isBinaryFormat=True
    report['stl_exported']=td.exportManager.execute(options);report['stl_file']=filename;report['stl_bodies']=2
    temp.close(False);document.activate();report['status']='complete';record()


def correction_inspect(app,design):
    root=design.rootComponent
    report={}
    targets=['Chassis_Bracket_Left:1','rail_lo:1','Motor_Front_Left:1','magazine_B:1','housing:1']
    for name in targets:
        occ=next(o for o in root.occurrences if o.name==name)
        report[name]=[]
        for proxy in all_bodies(occ):
            body=proxy.nativeObject or proxy
            vertices=[[v.geometry.x,v.geometry.y,v.geometry.z] for v in body.vertices]
            faces=[]
            for f in body.faces:
                if isinstance(f.geometry,adsk.core.Plane):
                    n=f.geometry.normal;c=f.centroid
                    faces.append({'area':f.area,'normal':[n.x,n.y,n.z],'center':[c.x,c.y,c.z]})
            report[name].append({'vertices':vertices,'faces':sorted(faces,key=lambda r:-r['area'])[:100]})
    with open(os.path.join(OUT,'correction_geometry.json'),'w',encoding='utf-8') as f:json.dump(report,f)
    occurrences=list(root.occurrences);visibility=[o.isLightBulbOn for o in occurrences]
    original=app.activeViewport.camera
    try:
        for name in targets[:3]:
            for o in occurrences:o.isLightBulbOn=o.name==name
            for label,orientation in [('top',adsk.core.ViewOrientations.TopViewOrientation),('iso',adsk.core.ViewOrientations.IsoTopRightViewOrientation)]:
                cam=app.activeViewport.camera;cam.viewOrientation=orientation;cam.isFitView=True;cam.isSmoothTransition=False
                app.activeViewport.camera=cam;app.activeViewport.refresh();adsk.doEvents()
                app.activeViewport.saveAsImageFile(os.path.join(OUT,'detail_'+name.replace(':','_')+'_'+label+'.png'),1400,1000)
    finally:
        for o,v in zip(occurrences,visibility):o.isLightBulbOn=v
        app.activeViewport.camera=original;app.activeViewport.refresh()


def inspect(app, design):
    os.makedirs(OUT, exist_ok=True)
    rows=[]
    for occ in design.rootComponent.allOccurrences:
        row={'path':occ.fullPathName,'name':occ.component.name,'transform':occ.transform2.asArray(),
             'bounds':bbox(occ.boundingBox),'bodies':[]}
        for body in occ.bRepBodies:
            cylinders=[]
            for face in body.faces:
                geom=face.geometry
                if isinstance(geom, adsk.core.Cylinder):
                    cylinders.append({'origin':[geom.origin.x,geom.origin.y,geom.origin.z],
                        'axis':[geom.axis.x,geom.axis.y,geom.axis.z], 'radius':geom.radius})
            row['bodies'].append({'name':body.name,'bounds':bbox(body.boundingBox),
                'appearance':body.appearance.name if body.appearance else None,
                'solid':body.isSolid,'cylinders':cylinders})
        rows.append(row)
    report={'document':app.activeDocument.name,'design_type':design.designType,
            'root_occurrences':[o.name for o in design.rootComponent.occurrences], 'occurrences':rows}
    with open(os.path.join(OUT,'inspection.json'),'w',encoding='utf-8') as f:
        json.dump(report,f,ensure_ascii=False,indent=2)
    backup=os.path.join(OUT,'Before_assembly_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S')+'.f3d')
    if not design.exportManager.execute(design.exportManager.createFusionArchiveExportOptions(backup)):
        raise RuntimeError('Assembly backup failed')

def run(context):
    app=adsk.core.Application.get()
    try:
        design=adsk.fusion.Design.cast(app.activeProduct)
        if not design: raise RuntimeError('Activate the destination assembly.')
        with open(os.path.join(HERE,'assembly_request.json'),encoding='utf-8') as f:
            request=json.load(f)
        if request['phase']=='assembly_urdf_1019':
            import importlib.util
            spec=importlib.util.spec_from_file_location('AssemblyURDF1019',os.path.join(HERE,'AssemblyURDF1019.py'))
            module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
            module.run(app,design,request,globals())
        elif request['phase'] in ['bracket_revise_1019','bracket_finish_1019']:
            import importlib.util
            spec=importlib.util.spec_from_file_location('BracketRevise1019',os.path.join(HERE,'BracketRevise1019.py'))
            module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
            if request['phase']=='bracket_finish_1019':module.finish(app,design)
            else:module.run(app,design)
        elif request['phase'] in ['bracket_audit_1019','bracket_fit_1019']:
            import importlib.util
            spec=importlib.util.spec_from_file_location('BracketAudit1019',os.path.join(HERE,'BracketAudit1019.py'))
            module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
            if request['phase']=='bracket_fit_1019':module.fit(app,design)
            else:module.run(app)
        elif request['phase']=='oct19':
            import importlib.util
            spec=importlib.util.spec_from_file_location('WaffleOct19',os.path.join(HERE,'Oct19.py'))
            module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
            module.run(app,design,request,globals())
        elif request['phase']=='inspect': inspect(app,design)
        elif request['phase']=='inspect_bracket': inspect_bracket(app)
        elif request['phase']=='inspect_parts': inspect_parts(app,design)
        elif request['phase']=='phase_one': phase_one(app,design)
        elif request['phase']=='phase_two': phase_two(app,design)
        elif request['phase']=='final_check': final_check(app,design)
        elif request['phase']=='correction_inspect': correction_inspect(app,design)
        elif request['phase']=='correction_apply': correction_apply(app,design)
        elif request['phase']=='correction_finish': correction_finish(app,design)
        elif request['phase']=='storage_lower_and_audit': storage_lower_and_audit(app,design)
        elif request['phase']=='manip_inspect': manip_inspect(app)
        elif request['phase']=='manip_detail': manip_detail(app,design)
        elif request['phase']=='manip_build': manip_build(app,design)
        elif request['phase']=='manip_repair': manip_repair(app,design)
        elif request['phase']=='manip_audit': manip_audit(app,design)
        elif request['phase']=='manip_poses': manip_poses(app,design)
        elif request['phase']=='wrist_reverse': wrist_reverse(app,design)
        elif request['phase']=='finish_and_stl': finish_and_stl(app,design)
        elif request['phase']=='resume_inspect':
            storage_lower_and_audit(app,design)
            manip_inspect(app)
    except:
        os.makedirs(OUT,exist_ok=True)
        with open(os.path.join(OUT,'error.txt'),'w',encoding='utf-8') as f:f.write(traceback.format_exc())
        app.userInterface.messageBox(traceback.format_exc())
