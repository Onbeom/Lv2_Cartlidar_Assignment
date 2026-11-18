import adsk.core, adsk.fusion, os, json

def run(app,api):
    out=api['OUT'];guard=os.path.join(out,'modular_bracket_committed.json')
    if os.path.exists(guard):raise RuntimeError('Modular bracket is already committed; use validation instead')
    design=api['source'](app);doc=app.activeDocument;root=design.rootComponent;mgr=adsk.fusion.TemporaryBRepManager.get()
    designs=[]
    for d in app.documents:
        dd=adsk.fusion.Design.cast(d.products.itemByProductType('DesignProductType'))
        if dd and dd.rootComponent.attributes.itemByName('WaffleModular','revision'):designs.append(dd)
    new=designs[-1]
    backup=os.path.join(out,'Assembly_Before_Modular_261021.f3d')
    if not os.path.exists(backup) and not design.exportManager.execute(design.exportManager.createFusionArchiveExportOptions(backup)):raise RuntimeError('Backup failed')
    names={'housing:1','housing:2','Open CASCADE STEP translator 8.0 1:2','Camera_D435_Pair_Source:1','Camera_Top_Crossbar_2020_261mm:1'}
    moving=[o for o in root.occurrences if o.name in names or o.name.startswith(('Motor_Front_','Motor_Rear_','Wheel_Front_','Wheel_Rear_','Chassis_Bracket_','Profile_2020_450mm_','magazine_','rail_hi:','rail_lo:'))]
    prior=os.path.join(out,'modular_bracket_commit_preflight.json')
    old=json.load(open(prior,encoding='utf8'))['old_transforms_cm'] if os.path.exists(prior) else {o.name:o.transform2.asArray() for o in moving}
    poses={}
    for name,values in old.items():v=list(values);v[3]-=1.25;poses[name]=v
    report={'document':doc.name,'shift_mm':[-12.5,0,0],'old_transforms_cm':old,'new_transforms_cm':poses,'parts':[]}
    api['write']('modular_bracket_commit_preflight',report)
    frame=next(g for g in root.rigidGroups if g.name=='Waffle_4WD_Static_Frame')
    members=list({o.fullPathName:o for o in frame.occurrences}.values());wheel_joints=[j for j in root.asBuiltJoints if 'Wheel' in j.name or 'wheel' in j.name]
    state=[False for j in wheel_joints]
    for j in wheel_joints:j.isSuppressed=True
    design.timeline.moveToEnd();frame.isSuppressed=True
    # Some selected components were created after the original group. Rolling
    # before that group invalidates their occurrence paths. Recreate the group
    # at the end using the same member occurrences and the new captured poses.
    if not frame.deleteMe():raise RuntimeError('Unable to replace original static frame')
    for o in moving:
        o.isGroundToParent=False
        t=adsk.core.Matrix3D.create();t.setWithArray(poses[o.name]);o.transform2=t
    if design.snapshots.hasPendingSnapshot:design.snapshots.add()
    frame=root.rigidGroups.add(adsk.core.ObjectCollection.createWithArray(members),False);frame.name='Waffle_4WD_Static_Frame'
    for j,s in zip(wheel_joints,state):j.isSuppressed=s
    if design.snapshots.hasPendingSnapshot:design.snapshots.add()
    design.computeAll();adsk.doEvents()
    errors={o.name:max(abs(a-b)for a,b in zip(o.transform2.asArray(),poses[o.name]))for o in moving}
    report['transform_errors']=errors;api['write']('modular_bracket_commit_progress',report)
    if max(errors.values())>1e-6:raise RuntimeError('Moved frame constraints changed placement; see transform errors')
    material=design.materials.itemByName('Waffle_PLA_Printed_1240_1240.0')
    for side in ['Left','Right']:
        occ=next(o for o in root.occurrences if o.name=='Chassis_Bracket_'+side+'_16mm:1');c=occ.component
        old_bodies=list(c.bRepBodies)
        for b in old_bodies:b.isLightBulbOn=False
        bf=c.features.baseFeatures.add();bf.name='Modular_Floor_Bracket_261021';bf.startEdit()
        try:
            for no in new.rootComponent.occurrences:
                if '_'+side+'_' not in no.name:continue
                temp=mgr.copy(no.component.bRepBodies.item(0));mgr.transform(temp,api['mat'](t=(-114.700031280518,282.699927706055,0)))
                b=c.bRepBodies.add(temp,bf);b.name=no.component.name;b.material=material
        finally:bf.finishEdit()
        c.material=material
        for b in bf.sourceBodies:b.material=material
        for b in bf.bodies:b.material=material;b.isLightBulbOn=True
        for b in old_bodies:b.isLightBulbOn=False
        report['parts'].append({'occurrence':occ.name,'new_body_names':[b.name for b in bf.bodies]})
    # Restore the physical model visibility from the previous mass model, not hidden history.
    raw=json.load(open(os.path.join(out,'body_physics_geometry.json'),encoding='utf8'))
    actual={r['path'].split('+')[0]for r in raw}
    for o in root.occurrences:
        if o.name in actual:o.isLightBulbOn=True
    design.computeAll();adsk.doEvents()
    newrows=[]
    for side in ['Left','Right']:
        o=next(o for o in root.occurrences if o.name=='Chassis_Bracket_'+side+'_16mm:1')
        for b in o.component.bRepBodies:
            if not b.isLightBulbOn:continue
            prop=b.getPhysicalProperties(adsk.fusion.CalculationAccuracy.MediumCalculationAccuracy)
            ok,ix,iy,iz,ixy,iyz,ixz=prop.getXYZMomentsOfInertia()
            temp=mgr.copy(b);mgr.transform(temp,o.transform2)
            newrows.append({'path':o.name,'body':b.name,'entity_token':b.entityToken,'bounds':api['bb'](temp),'volume_mm3':prop.volume*1000,'default_mass_g':prop.mass*1000,'com_component_mm':api['xyz'](prop.centerOfMass),'transform_cm':o.transform2.asArray(),'inertia_component_origin_g_mm2':[v*100000 for v in [ix,iy,iz,ixy,ixz,iyz]],'source_material':b.material.name,'valid_inertia':ok})
    report['new_bracket_physics']=newrows
    report['final_component_transforms_cm']={o.name:o.transform2.asArray()for o in root.occurrences}
    report['arm_names']=[o.name for o in root.occurrences if o.name.startswith(('arm_link_','gripper_','base_link__'))]
    base=os.path.join(out,'Assembly_Modular_261021')
    if not design.exportManager.execute(design.exportManager.createFusionArchiveExportOptions(base+'.f3d')):raise RuntimeError('Assembly F3D export failed')
    api['write']('modular_bracket_committed',report)
    cam=app.activeViewport.camera;cam.viewOrientation=adsk.core.ViewOrientations.IsoTopRightViewOrientation;cam.isFitView=True;cam.isSmoothTransition=False;app.activeViewport.camera=cam;app.activeViewport.refresh()
    app.activeViewport.saveAsImageFile(base+'.png',1800,1300)
