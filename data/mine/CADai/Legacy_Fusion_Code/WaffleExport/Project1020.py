import adsk.core, adsk.fusion
import json, os, math
HERE=os.path.dirname(__file__)
OUT=os.path.join(os.path.dirname(HERE),'Waffle_Handoff_20261020')
CAD_OUT=r'C:\Users\qjaqj\OneDrive\Desktop\Project2\261020\braket'
os.makedirs(OUT,exist_ok=True)
def write(n,d):
    with open(os.path.join(OUT,n+'.json'),'w',encoding='utf-8') as f:json.dump(d,f,ensure_ascii=False,indent=2)
def xyz(p):return [p.x*10,p.y*10,p.z*10]
def bb(e):
    b=e.boundingBox
    return dict(min=xyz(b.minPoint),max=xyz(b.maxPoint),size=xyz(b.minPoint.vectorTo(b.maxPoint)))
def geometry(b):
    d=dict(name=b.name,bounds=bb(b),solid=b.isSolid,faces=b.faces.count,planes=[],cylinders=[],circles=[])
    for f in b.faces:
        g=f.geometry
        if isinstance(g,adsk.core.Plane) and f.area>.03:
            d['planes'].append(dict(origin=xyz(g.origin),normal=g.normal.asArray(),area_mm2=f.area*100,bounds=bb(f)))
        elif isinstance(g,adsk.core.Cylinder):
            d['cylinders'].append(dict(origin=xyz(g.origin),axis=g.axis.asArray(),radius_mm=g.radius*10,bounds=bb(f)))
    for e in b.edges:
        g=e.geometry
        if isinstance(g,adsk.core.Circle3D):d['circles'].append(dict(center=xyz(g.center),normal=g.normal.asArray(),radius_mm=g.radius*10))
    return d
def inspect(app):
    source=app.activeDocument
    design=adsk.fusion.Design.cast(app.activeProduct)
    write('source_before',{'document':source.name,'occurrences':[dict(name=o.name,path=o.fullPathName,component=o.component.name,bounds=bb(o),transform=o.transform2.asArray()) for o in design.rootComponent.allOccurrences],
        'joints':[dict(name=j.name,one=j.occurrenceOne.fullPathName,two=j.occurrenceTwo.fullPathName,suppressed=j.isSuppressed) for j in design.rootComponent.asBuiltJoints]})
    mgr=adsk.fusion.TemporaryBRepManager.get();arm=[]
    for o in design.rootComponent.allOccurrences:
        if not o.fullPathName.startswith(('RMX52','RM_X52')):continue
        for b in o.component.bRepBodies:
            t=mgr.copy(b);mgr.transform(t,o.transform2);arm.append(dict(path=o.fullPathName,body=geometry(t)))
    write('arm_geometry_before',arm)
    imported=[]
    for name in ['rail_hi','rail_lo','UC-684_B0201']:
        path=os.path.join(OUT,'inputs',name+'.step')
        options=app.importManager.createSTEPImportOptions(path)
        doc=app.importManager.importToNewDocument(options)
        if not doc:raise RuntimeError('Import failed: '+name)
        dd=adsk.fusion.Design.cast(app.activeProduct);root=dd.rootComponent
        rows=[dict(path='',body=geometry(b)) for b in root.bRepBodies]
        for o in root.allOccurrences:
            for b in o.component.bRepBodies:
                t=mgr.copy(b);mgr.transform(t,o.transform2);rows.append(dict(path=o.fullPathName,body=geometry(t)))
        write(name+'_input_geometry',dict(document=doc.name,bounds=bb(root),bodies=rows))
        cam=app.activeViewport.camera;cam.viewOrientation=adsk.core.ViewOrientations.IsoTopRightViewOrientation;cam.isFitView=True;cam.isSmoothTransition=False;app.activeViewport.camera=cam;app.activeViewport.refresh();adsk.doEvents()
        app.activeViewport.saveAsImageFile(os.path.join(OUT,name+'_input.png'),1400,1000)
        imported.append(doc.name)
    source.activate()
    write('inspect_done',dict(imported_documents=imported))
def run(app,request):
    if request['action'].startswith('modular_bracket_'):
        import importlib.util
        spec=importlib.util.spec_from_file_location('ModularBracket',os.path.join(HERE,'ModularBracket.py'))
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        module.run(app,request['action'],globals())
    elif request['action'].startswith('floor_revision_'):
        import importlib.util
        spec=importlib.util.spec_from_file_location('FloorRevision',os.path.join(HERE,'FloorRevision.py'))
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        module.run(app,request['action'],globals())
    elif request['action']=='inspect':inspect(app)
    elif request['action']=='replace_rails':replace_rails(app)
    elif request['action']=='camera_preview':camera_preview(app)
    elif request['action']=='camera_build':camera_build(app)
    elif request['action']=='camera_diagnose':camera_diagnose(app)
    elif request['action']=='finish_camera_and_fit':
        finish_camera(app)
        rail_collision_details(app)
        physics_inspect(app)
    elif request['action']=='compare_and_materials':
        compare_rails(app)
        apply_materials(app)
    elif request['action']=='apply_materials':apply_materials(app)
    elif request['action']=='persist_and_validate':
        persist_materials(app)
        validate_player(app)
    elif request['action']=='portable_export':portable_export(app)
    elif request['action']=='verify_portable':verify_portable(app)
    elif request['action']=='physics_inspect':physics_inspect(app)
    elif request['action']=='fit_trial':fit_trial(app)
    elif request['action']=='commit_fit':
        commit_fit(app)
        apply_materials(app)
    elif request['action']=='advance':
        physics_inspect(app)
        if not os.path.exists(os.path.join(OUT,'rails_replaced.json')):replace_rails(app)
        camera_preview(app)

def mat(r=None,t=(0,0,0)):
    r=r or [[1,0,0],[0,1,0],[0,0,1]]
    m=adsk.core.Matrix3D.create()
    m.setWithArray([r[i][j] if j<3 else t[i]/10 for i in range(3) for j in range(4)]+[0,0,0,1])
    return m
def mul(a,b):
    x,y=a.asArray(),b.asArray();m=adsk.core.Matrix3D.create()
    m.setWithArray([sum(x[i*4+k]*y[k*4+j] for k in range(4)) for i in range(4) for j in range(4)])
    return m
def source(app):
    candidates=[]
    for d in app.documents:
        dd=adsk.fusion.Design.cast(d.products.itemByProductType('DesignProductType'))
        if dd and not d.name.startswith('Waffle_URDF_Export_') and any(j.name=='RMX52_Shoulder' for j in dd.rootComponent.asBuiltJoints):candidates.append(d)
    if not candidates:
        filename=os.path.join(CAD_OUT,'Assembly_Modular_261021.f3d')
        app.importManager.importToNewDocument(app.importManager.createFusionArchiveImportOptions(filename))
        dd=adsk.fusion.Design.cast(app.activeProduct)
        if not dd or not any(j.name=='RMX52_Shoulder' for j in dd.rootComponent.asBuiltJoints):raise RuntimeError('복구한 어셈블리에 팔 관절이 없습니다.')
        candidates=[app.activeDocument]
    doc=next((d for d in candidates if d.name=='assembly'),candidates[-1])
    doc.activate()
    return adsk.fusion.Design.cast(app.activeProduct)
def input_design(app,name):
    for doc in app.documents:
        dd=adsk.fusion.Design.cast(doc.products.itemByProductType('DesignProductType'))
        if not dd or doc.name=='assembly':continue
        bodies=dd.rootComponent.bRepBodies
        if name=='UC-684_B0201' and bodies.count==138:return dd
        if name in ['rail_hi','rail_lo'] and bodies.count==1:
            z=bodies.item(0).boundingBox.minPoint.z*10
            if abs(z-(162 if name=='rail_hi' else -4))<.01:return dd
    raise RuntimeError('Input document not open: '+name)
def world_bodies(root):
    mgr=adsk.fusion.TemporaryBRepManager.get();result=[]
    def walk(comp,t,path):
        for b in comp.bRepBodies:
            if not b.isLightBulbOn:continue
            temp=mgr.copy(b);mgr.transform(temp,t);result.append((path,temp,b))
        for o in comp.occurrences:
            if o.isLightBulbOn:walk(o.component,mul(t,o.transform2),path+'+'+o.name if path else o.name)
    walk(root,mat(),'')
    return result
def intersects(a,b):
    aa,ab=a.boundingBox,b.boundingBox
    if any(getattr(aa.maxPoint,k)<=getattr(ab.minPoint,k)+1e-7 or getattr(ab.maxPoint,k)<=getattr(aa.minPoint,k)+1e-7 for k in ['x','y','z']):return 0
    mgr=adsk.fusion.TemporaryBRepManager.get();test=mgr.copy(a)
    if not mgr.booleanOperation(test,b,adsk.fusion.BooleanTypes.IntersectionBooleanType):raise RuntimeError('Intersection failed')
    return test.volume*1000 if test.isSolid else 0
def replace_rails(app):
    design=source(app);mgr=adsk.fusion.TemporaryBRepManager.get();root=design.rootComponent
    if os.path.exists(os.path.join(OUT,'rails_replaced.json')):raise RuntimeError('Rails already replaced')
    prepared=[];report={'parts':[],'collisions':[]}
    allg=world_bodies(root)
    for name,zshift in [('rail_hi',124),('rail_lo',180)]:
        target=next(o for o in root.occurrences if o.component.name==name)
        leaf=next(o for o in target.component.occurrences if o.component.bRepBodies.count)
        inp=input_design(app,name).rootComponent.bRepBodies.item(0)
        temp=mgr.copy(inp);mgr.transform(temp,mat(t=(-118,-125,zshift)))
        for path,other,native in allg:
            if path.startswith(name+':') or not other.isSolid:continue
            volume=intersects(temp,other)
            if volume>.1:report['collisions'].append(dict(rail=name,other=path,volume_mm3=volume))
        world=bb(temp);inv=mul(target.transform2,leaf.transform2);inv.invert();mgr.transform(temp,inv)
        prepared.append((leaf.component,temp))
        report['parts'].append(dict(name=name,bounds=world,source_file=name+'.step'))
    write('rails_preflight',report)
    # Preserve the supplied STEP exactly. Existing storage fit issues remain explicit.
    report['fit_status']='requires_storage_fit_review' if report['collisions'] else 'clear'
    archive=os.path.join(OUT,'Assembly_Before_1020.f3d')
    if not design.exportManager.execute(design.exportManager.createFusionArchiveExportOptions(archive)):raise RuntimeError('Backup failed')
    for comp,temp in prepared:
        old=next(b for b in comp.bRepBodies if b.isLightBulbOn)
        feature=comp.features.baseFeatures.item(0) if comp.features.baseFeatures.count==1 and comp.features.baseFeatures.item(0).bodies.count==1 else None
        if feature:
            feature.startEdit()
            try:
                if not feature.updateBody(feature.bodies.item(0),temp):raise RuntimeError('Update body failed')
            finally:feature.finishEdit()
        else:
            feature=comp.features.baseFeatures.add();feature.name='User_STEP_Rail_20261020';feature.startEdit()
            new=comp.bRepBodies.add(temp,feature);feature.finishEdit();new.appearance=old.appearance;old.isLightBulbOn=False
    design.computeAll();adsk.doEvents();write('rails_replaced',report)

def box(x0,y0,z0,x1,y1,z1):
    return adsk.fusion.TemporaryBRepManager.get().createBox(adsk.core.OrientedBoundingBox3D.create(
        adsk.core.Point3D.create((x0+x1)/20,(y0+y1)/20,(z0+z1)/20),
        adsk.core.Vector3D.create(1,0,0),adsk.core.Vector3D.create(0,1,0),(x1-x0)/10,(y1-y0)/10,(z1-z0)/10))
def cylinder(x,y,z0,z1,r):
    return adsk.fusion.TemporaryBRepManager.get().createCylinderOrCone(adsk.core.Point3D.create(x/10,y/10,z0/10),r/10,adsk.core.Point3D.create(x/10,y/10,z1/10),r/10)
def boolean(a,b,union=True):
    kind=adsk.fusion.BooleanTypes.UnionBooleanType if union else adsk.fusion.BooleanTypes.DifferenceBooleanType
    if not adsk.fusion.TemporaryBRepManager.get().booleanOperation(a,b,kind):raise RuntimeError('Bracket boolean failed')
def camera_parts(app):
    mgr=adsk.fusion.TemporaryBRepManager.get()
    # Native optical axis is +Z; board back datum and four holes measured from the STEP.
    pcb=[-20,14.9455379345955,5.75637747137313]
    target=[405.8,0,318.5];center=[330,0,384]
    dx,dz=target[0]-center[0],target[2]-center[2];dist=math.hypot(dx,dz);a,b=dx/dist,dz/dist
    r=[[0,-b,a],[1,0,0],[0,a,b]]
    t=[center[i]-sum(r[i][j]*pcb[j] for j in range(3)) for i in range(3)]
    transform=mat(r,t)
    cam=[]
    for body in input_design(app,'UC-684_B0201').rootComponent.bRepBodies:
        temp=mgr.copy(body);mgr.transform(temp,transform);cam.append((temp,body))
    # Back frame: 8 mm electronics clearance, M3 through-holes, 4 mm walls.
    z=pcb[2];frame=box(pcb[0]-20.5,pcb[1]-20.5,z-12,pcb[0]+20.5,pcb[1]+20.5,z-8)
    boolean(frame,box(pcb[0]-12,pcb[1]-12,z-12.1,pcb[0]+12,pcb[1]+12,z-7.9),False)
    for u in [-14,14]:
        for v in [-14,14]:
            x,y=pcb[0]+u,pcb[1]+v
            boolean(frame,cylinder(x,y,z-8.01,z-.1,3))
            boolean(frame,cylinder(x,y,z-12.1,z+.1,1.7),False)
    mgr.transform(frame,transform)
    # Existing gripper cap is 2.55 mm above the motor top. Keep its screws clear.
    base=box(326.5,-12,356.31,336,12,360.31)
    boolean(base,box(326.5,-20.5,360.1,334,20.5,364.1))
    for y in [-18.5,18.5]:
        boolean(base,box(305,y-2,360.1,334,y+2,364.1))
        boolean(base,box(305,y-2,363.9,311,y+2,384))
    for y in [-8,8]:
        boolean(base,cylinder(330.5,y,356.2,364.2,1.4),False)
        boolean(base,cylinder(330.5,y,361,364.2,2.8),False)
    boolean(base,frame)
    if not base.isSolid or base.lumps.count!=1:raise RuntimeError('Bracket disconnected')
    return cam,base,transform,dict(pcb_back_center_world_mm=center,target_world_mm=target,optical_axis=[a,0,b],
        downward_angle_deg=math.degrees(math.atan2(-b,a)),motor_mount_pattern_mm=[0,16],motor_clearance_hole_mm=2.8,motor_fasteners='2 x M2.5 (existing tapped motor holes), counterbore 5.6 mm',camera_pattern_mm=[28,28],camera_hole_mm=3.4,electronics_standoff_mm=8)
def camera_preview(app):
    write('camera_progress',{'stage':'constructing bracket'})
    design=source(app);cam,bracket,transform,report=camera_parts(app)
    write('camera_progress',{'stage':'copying source geometry'})
    allg=world_bodies(design.rootComponent)
    write('camera_progress',{'stage':'checking interference'})
    report['collisions']=[]
    # Camera pose is unchanged from the completed previous check. Test the revised bracket.
    for label,body in [('bracket',bracket)]:
        write('camera_progress',{'stage':'checking interference','part':label})
        adsk.doEvents()
        for path,other,native in allg:
            if not other.isSolid:continue
            v=intersects(body,other)
            if v>.1:report['collisions'].append(dict(part=label,other=path,volume_mm3=v))
    for i,(body,native) in enumerate(cam):
        v=intersects(bracket,body)
        if v>.1:report['collisions'].append(dict(part='bracket',other='camera_'+str(i),volume_mm3=v))
    report['bracket_bounds']=bb(bracket);report['bracket_volume_mm3']=bracket.volume*1000
    write('camera_preview',report)
    # Compare unchanged central features to align supplied STEP bodies exactly.
    vertices={}
    mgr=adsk.fusion.TemporaryBRepManager.get()
    for label in ['rail_hi','rail_lo']:
        old=next(b for path,b,n in allg if path.startswith(label+':'))
        new=mgr.copy(input_design(app,label).rootComponent.bRepBodies.item(0))
        mgr.transform(new,mat(t=(-118,-125,124 if label=='rail_hi' else 180)))
        vertices[label]={'old':[xyz(v.geometry) for v in old.vertices],'new':[xyz(v.geometry) for v in new.vertices]}
    write('rail_alignment_vertices',vertices)

def physics_inspect(app):
    design=source(app);rows=[]
    transforms={}
    def walk(comp,t,path):
        transforms[path]=t.asArray()
        for o in comp.occurrences:
            if o.isLightBulbOn:walk(o.component,mul(t,o.transform2),path+'+'+o.name if path else o.name)
    walk(design.rootComponent,mat(),'')
    props=[]
    for p in design.rootComponent.occurrences.item(0).component.material.materialProperties:
        if 'dens' in p.id.lower() or 'dens' in p.name.lower():
            props.append(dict(id=p.id,name=p.name,type=p.objectType,value=getattr(p,'value',None),units=getattr(p,'units',None)))
    write('density_property',props)
    for i,(path,temp,native) in enumerate(world_bodies(design.rootComponent)):
        p=native.getPhysicalProperties(adsk.fusion.CalculationAccuracy.MediumCalculationAccuracy)
        ok,ix,iy,iz,ixy,iyz,ixz=p.getXYZMomentsOfInertia()
        rows.append(dict(path=path,body=native.name,entity_token=native.entityToken,bounds=bb(temp),volume_mm3=p.volume*1000,
            default_mass_g=p.mass*1000,com_component_mm=xyz(p.centerOfMass),transform_cm=transforms[path],
            inertia_component_origin_g_mm2=[v*100000 for v in [ix,iy,iz,ixy,ixz,iyz]],
            source_material=native.material.name,valid_inertia=ok))
        if i%25==0:write('physics_progress',dict(copied=i+1));adsk.doEvents()
    write('body_physics_geometry',rows)
    mapping={
      'RMX52_DC12_STD_1':'RMX52_Motor_J1_Base_Yaw',
      'RMX52_DC12_W-IDLE_ALT1_1':'RMX52_Motor_J2_Shoulder',
      'RMX52_DC12_W-IDLE_ALT1_2':'RMX52_Motor_J3_Elbow',
      'RMX52_DC12_W-IDLE_ALT1_3':'RMX52_Motor_J4_Wrist_Pitch',
      'RMX52_DYNAMIXEL_Wrist_Roll':'RMX52_Motor_J5_Wrist_Roll',
      'RMX52_DC12_STD_2':'RMX52_Motor_J6_Gripper_Open_Close'}
    changed=[]
    for o in design.rootComponent.occurrences:
        if o.component.name in mapping:
            old=o.component.name;new=mapping[old];o.component.attributes.add('Waffle','original_name',old)
            o.component.name=new;changed.append(dict(old=old,new=new))
    design.unitsManager.distanceDisplayUnits=adsk.fusion.DistanceUnits.MillimeterDistanceUnits
    if changed:write('motor_rename_map',changed)

def camera_diagnose(app):
    design=source(app);root=design.rootComponent
    names=['RMX52_Motor_J6_Gripper_Open_Close','UC684_Camera_On_Gripper','UC684_Camera_Bracket_PLA']
    occs=[next(o for o in root.occurrences if o.component.name==n) for n in names]
    report={'occurrences':[dict(name=o.name,grounded=o.isGrounded,ground_to_parent=o.isGroundToParent,transform=o.transform2.asArray()) for o in occs],
       'groups':[dict(name=g.name,suppressed=g.isSuppressed,members=[o.name for o in g.occurrences]) for g in root.rigidGroups],
       'joints':[dict(name=j.name,suppressed=j.isSuppressed) for j in root.asBuiltJoints]}
    j=next(j for j in root.asBuiltJoints if j.name=='RMX52_Wrist_Roll_Reversed');q=j.jointMotion.rotationValue
    try:
        j.jointMotion.rotationValue=q+.05;app.activeViewport.refresh();adsk.doEvents()
        report['after_refresh']=[dict(name=o.name,transform=o.transform2.asArray()) for o in occs]
    finally:
        j.jointMotion.rotationValue=q;app.activeViewport.refresh();adsk.doEvents()
    write('camera_attachment_diagnostic',report)

def finish_camera(app):
    design=source(app);root=design.rootComponent
    added=[next(o for o in root.occurrences if o.component.name==n) for n in ['UC684_Camera_On_Gripper','UC684_Camera_Bracket_PLA']]
    j=next(j for j in root.asBuiltJoints if j.name=='RMX52_Wrist_Roll_Reversed')
    q=j.jointMotion.rotationValue;t0=added[0].transform2.asArray()
    try:
        j.jointMotion.rotationValue=q+.05;app.activeViewport.refresh();adsk.doEvents()
        delta=max(abs(a-b) for a,b in zip(t0,added[0].transform2.asArray()))
        if delta<1e-5:raise RuntimeError('Camera does not follow wrist roll')
    finally:
        j.jointMotion.rotationValue=q;app.activeViewport.refresh();adsk.doEvents()
    _,_,_,params=camera_parts(app)
    params.update(added_components=[o.component.name for o in added],attached_to='RMX52_Motor_J6_Gripper_Open_Close',wrist_follow_test_rad=.05,follow_transform_delta=delta)
    write('camera_installed',params)
    partdir=os.path.join(OUT,'parts');os.makedirs(partdir,exist_ok=True)
    comp=added[1].component
    if not design.exportManager.execute(design.exportManager.createSTEPExportOptions(os.path.join(partdir,'UC684_Camera_Bracket_PLA.step'),comp)):raise RuntimeError('Bracket STEP export failed')
    stl=design.exportManager.createSTLExportOptions(comp,os.path.join(partdir,'UC684_Camera_Bracket_PLA_mm.stl'))
    stl.unitType=adsk.fusion.DistanceUnits.MillimeterDistanceUnits
    if not design.exportManager.execute(stl):raise RuntimeError('Bracket STL export failed')
    camera=app.activeViewport.camera;camera.eye=adsk.core.Point3D.create(48,-25,45);camera.target=adsk.core.Point3D.create(35,0,35)
    camera.upVector=adsk.core.Vector3D.create(0,0,1);camera.isFitView=False;camera.viewExtents=20;camera.isSmoothTransition=False
    app.activeViewport.camera=camera;app.activeViewport.refresh();adsk.doEvents()
    app.activeViewport.saveAsImageFile(os.path.join(OUT,'UC684_Mount_Closeup.png'),1600,1100)

def rail_collision_details(app):
    root=source(app).rootComponent;allg=world_bodies(root);mgr=adsk.fusion.TemporaryBRepManager.get();report=[]
    for path,rail,n in allg:
        if not path.startswith(('rail_hi:','rail_lo:')):continue
        for other,b,nb in allg:
            if not other.startswith(('housing:','magazine_','Open CASCADE STEP translator')):continue
            if intersects(rail,b)<.1:continue
            temp=mgr.copy(rail)
            if mgr.booleanOperation(temp,b,adsk.fusion.BooleanTypes.IntersectionBooleanType):
                report.append(dict(rail=path,other=other,volume_mm3=temp.volume*1000,bounds=bb(temp)))
    write('rail_collision_details',report)

def compare_rails(app):
    design=source(app);doc=app.activeDocument;allg=world_bodies(design.rootComponent);mgr=adsk.fusion.TemporaryBRepManager.get();report=[]
    with open(os.path.join(OUT,'source_before.json'),encoding='utf-8') as f:before=json.load(f)
    for label in ['rail_hi','rail_lo']:
        filename=os.path.join(os.path.dirname(OUT),'Waffle_Assembly_Build','AssemblyURDF1019',label+'_outward9mm.step')
        inp=app.importManager.importToNewDocument(app.importManager.createSTEPImportOptions(filename))
        dd=adsk.fusion.Design.cast(app.activeProduct)
        candidates=world_bodies(dd.rootComponent)
        if not candidates:raise RuntimeError('Old rail import has no body')
        old=candidates[0][1];old_bounds=bb(old)
        prior=next(o for o in before['occurrences'] if o['path'].startswith(label+':') and '+' in o['path'])['bounds']
        shift=[(prior['min'][i]+prior['max'][i]-old_bounds['min'][i]-old_bounds['max'][i])/2 for i in range(3)]
        mgr.transform(old,mat(t=shift));doc.activate()
        new=next(b for p,b,n in allg if p.startswith(label+':'))
        row=dict(rail=label,old_bounds=bb(old),new_bounds=bb(new),old_collisions=[],new_collisions=[],added_lumps=[],removed_lumps=[])
        for p,b,n in allg:
            if not p.startswith(('housing:','magazine_','Open CASCADE STEP translator')):continue
            for key,rail in [('old_collisions',old),('new_collisions',new)]:
                v=intersects(rail,b)
                if v>.1:row[key].append(dict(other=p,volume_mm3=v))
        for key,a,b in [('added_lumps',new,old),('removed_lumps',old,new)]:
            delta=mgr.copy(a)
            if mgr.booleanOperation(delta,b,adsk.fusion.BooleanTypes.DifferenceBooleanType):
                row[key]=[dict(bounds=bb(l),volume_mm3=l.volume*1000) for l in delta.lumps] if hasattr(delta.lumps.item(0) if delta.lumps.count else None,'volume') else [dict(bounds=bb(delta),volume_mm3=delta.volume*1000)]
        report.append(row);write('rail_old_new_comparison',report)

def fit_trial(app):
    design=source(app);allg=world_bodies(design.rootComponent);mgr=adsk.fusion.TemporaryBRepManager.get()
    rails=[b for p,b,n in allg if p.startswith(('rail_hi:','rail_lo:'))]
    selected=[(p,b) for p,b,n in allg if p.startswith(('housing:1','magazine_B:','magazine_R:','Open CASCADE STEP translator 8.0 1:2'))]
    report=[]
    for dx,dy in [(0,0)]+[(x,0) for x in [-.1,.1,-.2,.2,-.3,.3,-.5,.5]]+[(0,y) for y in [-.1,.1,-.2,.2,-.3,.3,-.5,.5]]:
        vals=[]
        for p,b in selected:
            t=mgr.copy(b);mgr.transform(t,mat(t=(dx,dy*(1 if 'housing:1' in p or 'magazine_B' in p else -1),0)))
            vals.append(dict(part=p,volume_mm3=sum(intersects(t,r) for r in rails)))
        report.append(dict(dx_mm=dx,outward_dy_mm=dy,collisions=vals))
        adsk.doEvents();write('rail_fit_trials',report)
    a0=math.atan2(-2.464,5.4706);a1=math.atan2(-2.26,5.5583);angle=a1-a0
    for offset in [0,.05,.1,.2,.3,.5]:
        vals=[];transforms={}
        for p,b in selected:
            sign=1 if 'housing:1' in p or 'magazine_B' in p else -1
            theta=sign*angle;c,s=math.cos(theta),math.sin(theta)
            old=(-98.6311,sign*58.157);new=(-97.7099,sign*58.893)
            t=(new[0]-c*old[0]+s*old[1]+offset,new[1]-s*old[0]-c*old[1],0)
            transform=mat([[c,-s,0],[s,c,0],[0,0,1]],t)
            temp=mgr.copy(b);mgr.transform(temp,transform)
            vals.append(dict(part=p,volume_mm3=sum(intersects(temp,r) for r in rails)))
            transforms[p]=transform.asArray()
        report.append(dict(rotation_deg=math.degrees(angle),forward_offset_mm=offset,collisions=vals,transforms_cm=transforms))
        adsk.doEvents();write('rail_fit_trials',report)
    apply_materials(app)

def commit_fit(app):
    if os.path.exists(os.path.join(OUT,'rail_fit_committed.json')):return
    design=source(app);root=design.rootComponent
    with open(os.path.join(OUT,'rail_fit_trials.json'),encoding='utf-8') as f:trials=json.load(f)
    trial=next(t for t in trials if 'rotation_deg' in t and t['forward_offset_mm']==0)
    if any(c['volume_mm3']>.01 for c in trial['collisions']):raise RuntimeError('Candidate fit not clear')
    poses={};oldposes={}
    for path,values in trial['transforms_cm'].items():
        name=path.split('+')[0];occ=next(o for o in root.occurrences if o.name==name)
        delta=adsk.core.Matrix3D.create();delta.setWithArray(values)
        oldposes[name]=occ.transform2.asArray();poses[name]=mul(delta,occ.transform2).asArray()
    frame=next(g for g in root.rigidGroups if g.name=='Waffle_4WD_Static_Frame')
    frame.isSuppressed=True;frame.timelineObject.rollTo(True)
    try:
        for name,values in poses.items():
            occ=next(o for o in root.occurrences if o.name==name);occ.isGroundToParent=False
            transform=adsk.core.Matrix3D.create();transform.setWithArray(values);occ.transform2=transform
        if design.snapshots.hasPendingSnapshot:design.snapshots.add()
        frame.setOccurrences(adsk.core.ObjectCollection.createWithArray(list(frame.occurrences)),True)
    finally:
        design.timeline.moveToEnd();frame.isSuppressed=False
    if design.snapshots.hasPendingSnapshot:design.snapshots.add()
    app.activeViewport.refresh();adsk.doEvents()
    errors={name:max(abs(a-b) for a,b in zip(values,next(o for o in root.occurrences if o.name==name).transform2.asArray())) for name,values in poses.items()}
    write('rail_fit_committed',dict(rotation_deg_B=trial['rotation_deg'],rotation_deg_R=-trial['rotation_deg'],old_transforms_cm=oldposes,new_transforms_cm=poses,transform_errors=errors))
    rail_collision_details(app)
    with open(os.path.join(OUT,'rail_collision_details.json'),encoding='utf-8') as f:hits=json.load(f)
    if hits or max(errors.values())>1e-6:raise RuntimeError('Committed rail fit requires review')

def camera_build(app):
    design=source(app);root=design.rootComponent
    if any(o.component.name=='UC684_Camera_On_Gripper' for o in root.occurrences):raise RuntimeError('Camera already installed')
    camera_preview(app)
    with open(os.path.join(OUT,'camera_preview.json'),encoding='utf-8') as f:report=json.load(f)
    if report['collisions']:raise RuntimeError('Camera bracket requires fit correction; see camera_preview.json')
    cam,bracket,transform,params=camera_parts(app)
    added=[]
    for name,parts in [('UC684_Camera_On_Gripper',cam),('UC684_Camera_Bracket_PLA',[(bracket,None)])]:
        occ=root.occurrences.addNewComponent(mat());occ.component.name=name;occ.isGroundToParent=False
        feature=occ.component.features.baseFeatures.add();feature.name='UC684_mount_20261020';feature.startEdit()
        try:
            for temp,original in parts:
                body=occ.component.bRepBodies.add(temp,feature)
                if original:
                    body.name=original.name
                    try:body.appearance=original.appearance
                    except:pass
        finally:feature.finishEdit()
        motor=next(o for o in root.occurrences if o.component.name=='RMX52_Motor_J6_Gripper_Open_Close')
        ji=root.asBuiltJoints.createInput(motor,occ,None);ji.setAsRigidJointMotion()
        joint=root.asBuiltJoints.add(ji);joint.name=name+'_Fixed_To_GripperMotor';joint.isLightBulbOn=False
        added.append(occ)
    design.computeAll();adsk.doEvents()
    # Confirm attachment follows wrist roll, then restore its exact initial value.
    j=next(j for j in root.asBuiltJoints if j.name=='RMX52_Wrist_Roll_Reversed')
    q=j.jointMotion.rotationValue;t0=added[0].transform2.asArray()
    try:
        j.jointMotion.rotationValue=q+.05;design.computeAll();adsk.doEvents()
        moved=added[0].transform2.asArray()
        if max(abs(a-b) for a,b in zip(t0,moved))<1e-5:raise RuntimeError('Camera does not follow wrist roll')
    finally:j.jointMotion.rotationValue=q;design.computeAll();adsk.doEvents()
    params.update(added_components=[o.component.name for o in added],attached_to='RMX52_Motor_J6_Gripper_Open_Close',wrist_follow_test_rad=.05)
    write('camera_installed',params)
    partdir=os.path.join(OUT,'parts');os.makedirs(partdir,exist_ok=True)
    comp=added[1].component
    if not design.exportManager.execute(design.exportManager.createSTEPExportOptions(os.path.join(partdir,'UC684_Camera_Bracket_PLA.step'),comp)):raise RuntimeError('Bracket STEP export failed')
    stl=design.exportManager.createSTLExportOptions(comp,os.path.join(partdir,'UC684_Camera_Bracket_PLA_mm.stl'))
    stl.unitType=adsk.fusion.DistanceUnits.MillimeterDistanceUnits
    if not design.exportManager.execute(stl):raise RuntimeError('Bracket STL export failed')
    camera=app.activeViewport.camera;camera.eye=adsk.core.Point3D.create(48,-25,45);camera.target=adsk.core.Point3D.create(35,0,35)
    camera.upVector=adsk.core.Vector3D.create(0,0,1);camera.isFitView=False;camera.viewExtents=20;camera.isSmoothTransition=False
    app.activeViewport.camera=camera;app.activeViewport.refresh();adsk.doEvents()
    app.activeViewport.saveAsImageFile(os.path.join(OUT,'UC684_Mount_Closeup.png'),1600,1100)
    physics_inspect(app)

def apply_materials(app):
    design=source(app)
    with open(os.path.join(OUT,'material_assignment_plan.json'),encoding='utf-8') as f:plan=json.load(f)
    results=[];done={};materials={};live={}
    for path,temp,native in world_bodies(design.rootComponent):live.setdefault(path,[]).append(native)
    for row in plan:
        token=row['entity_token'];density=row['density_kg_per_m3'];key=(row['name'],round(density,5))
        if token in done:
            if abs(done[token]-density)>.01:raise RuntimeError('Conflicting mass assumptions for shared body')
            continue
        candidates=[b for b in live.get(row['path'],[]) if b.name==row['body_name']]
        if len(candidates)!=1:raise RuntimeError('Body identity ambiguous: '+row['path']+' / '+row['body_name'])
        body=candidates[0];token=body.entityToken
        if not body:raise RuntimeError('Token is not a body')
        current=body.getPhysicalProperties(adsk.fusion.CalculationAccuracy.MediumCalculationAccuracy).mass*1000
        if body.material.name.startswith('Waffle_') and abs(current-row['expected_body_mass_g'])<=max(.001,row['expected_body_mass_g']*.001):
            results.append(dict(path=row['path'],material=body.material.name,mass_g=current,error_g=abs(current-row['expected_body_mass_g']),already_assigned=True))
            done[token]=density
            continue
        material=materials.get(key)
        if not material:
            name='Waffle_'+row['name']+'_'+str(round(density,3))
            material=design.materials.itemByName(name)
            if not material:material=design.materials.addByCopy(body.material,name)
            for prop in material.materialProperties:
                if prop.id in ['structural_Density','thermal_Density']:
                    fp=adsk.core.FloatProperty.cast(prop);fp.value=density
            materials[key]=material
        body.material=material
        mass=body.getPhysicalProperties(adsk.fusion.CalculationAccuracy.MediumCalculationAccuracy).mass*1000
        error=abs(mass-row['expected_body_mass_g'])
        results.append(dict(path=row['path'],material=material.name,mass_g=mass,error_g=error))
        if error>max(.001,row['expected_body_mass_g']*.001):raise RuntimeError('Assigned mass mismatch: '+row['path'])
        done[token]=density
        if len(results)%20==0:write('materials_progress',dict(assigned=len(results)));adsk.doEvents()
    design.computeAll();adsk.doEvents();write('materials_applied',dict(bodies=len(results),materials=len(materials),checks=results))
    physics_inspect(app)

def persist_materials(app):
    design=source(app)
    write('material_persistence_progress',dict(stage='starting'))
    with open(os.path.join(OUT,'material_assignment_plan.json'),encoding='utf-8') as f:plan=json.load(f)
    live={};jobs={};feature_lookup={}
    for path,temp,native in world_bodies(design.rootComponent):live[(path,native.name)]=native
    for row_index,row in enumerate(plan):
        body=live[(row['path'],row['body_name'])]
        material=design.materials.itemByName('Waffle_'+row['name']+'_'+str(round(row['density_kg_per_m3'],3)))
        if not material:raise RuntimeError('Equivalent material missing')
        comp=body.parentComponent;ckey=comp.entityToken
        if ckey not in feature_lookup:
            lookup={}
            for bf in comp.features.baseFeatures:
                for index,b in enumerate(bf.bodies):lookup[b.name]=(bf,index)
            feature_lookup[ckey]=lookup
        pair=feature_lookup[ckey].get(body.name)
        if pair:
            bf,index=pair;key=bf.entityToken
            if key not in jobs:jobs[key]=(bf,{})
            jobs[key][1][index]=material
        if row_index%25==0:
            write('material_persistence_progress',dict(stage='mapping_source_bodies',mapped=row_index,total=len(plan)))
            adsk.doEvents()
    changed=[]
    for bf,assignments in jobs.values():
        write('material_persistence_progress',dict(stage='editing_source_material',feature=bf.name,completed=len(changed),total=len(jobs)))
        # Material is non-geometric metadata. Modify the owned source bodies
        # directly, avoiding a full timeline edit of every imported feature.
        sources=bf.sourceBodies
        for index,material in assignments.items():sources[index].material=material
        changed.append(dict(feature=bf.name,bodies=len(assignments)));adsk.doEvents()
    design.computeAll();app.activeViewport.refresh();adsk.doEvents()
    physics_inspect(app)
    with open(os.path.join(OUT,'body_physics_geometry.json'),encoding='utf8') as f:actual=json.load(f)
    expected={(r['path'],r['body_name']):r['expected_body_mass_g'] for r in plan}
    errors=[dict(path=r['path'],body=r['body'],actual_mass_g=r['default_mass_g'],expected_mass_g=expected[(r['path'],r['body'])]) for r in actual if abs(r['default_mass_g']-expected.get((r['path'],r['body']),0))>max(.002,expected.get((r['path'],r['body']),0)*.001)]
    write('material_persistence_verified',dict(base_features=changed,total_visible_mass_g=sum(r['default_mass_g'] for r in actual),mass_errors=errors))
    if errors:raise RuntimeError('Material persistence requires review; see material_persistence_verified.json')

def validate_player(app):
    import importlib.util
    design=source(app)
    spec=importlib.util.spec_from_file_location('WafflePlayerCurrent',os.path.join(HERE,'WafflePlayerV2.py'))
    player=importlib.util.module_from_spec(spec);spec.loader.exec_module(player)
    design,joints,frames=player.prepare(app)
    camera=next(o for o in design.rootComponent.occurrences if o.component.name=='UC684_Camera_On_Gripper')
    initial=camera.transform2.asArray();saved=[j.jointMotion.rotationValue for j in joints]
    checks=[]
    try:
        # Coarse sampled interference checks for the added camera/mount against
        # storage, rails, floors and profile structure. Not a swept-volume proof.
        for index in [0,18,72,126,180,240,300,360,401]:
            for j,q in zip(joints,frames[index]['q']):j.jointMotion.rotationValue=q
            app.activeViewport.refresh();adsk.doEvents()
            allg=world_bodies(design.rootComponent)
            added=[(p,b) for p,b,n in allg if p.startswith('UC684_')]
            obstacles=[(p,b) for p,b,n in allg if p.startswith(('housing:','magazine_','Open CASCADE STEP translator','rail_','Floor_','Profile_2020_','Camera_Top_'))]
            hits=[]
            for p,b in added:
                for o,other in obstacles:
                    v=intersects(b,other)
                    if v>.1:hits.append(dict(part=p,obstacle=o,volume_mm3=v))
            checks.append(dict(frame=index,hits=hits));write('new_camera_motion_samples',checks)
    finally:
        for j,q in zip(joints,saved):j.jointMotion.rotationValue=q
        app.activeViewport.refresh();adsk.doEvents()
    player.play(app,design,joints,frames)
    error=max(abs(a-b) for a,b in zip(initial,camera.transform2.asArray()))
    write('final_playback_verification',dict(camera_transform_restoration_error=error,checked_camera_motion_frames=len(checks),sample_hits=sum(len(c['hits']) for c in checks)))
    filename=os.path.join(OUT,'Assembly_Working_20261020.f3d')
    if not design.exportManager.execute(design.exportManager.createFusionArchiveExportOptions(filename)):raise RuntimeError('Working archive export failed')
    view=app.activeViewport.camera;view.viewOrientation=adsk.core.ViewOrientations.IsoTopRightViewOrientation;view.isFitView=True;view.isSmoothTransition=False;app.activeViewport.camera=view
    app.activeViewport.refresh();adsk.doEvents();app.activeViewport.saveAsImageFile(os.path.join(OUT,'Assembly_Current_20261020.png'),1800,1300)

def portable_export(app):
    source_design=source(app);source_doc=app.activeDocument;root=source_design.rootComponent
    for prior in list(app.documents):
        if prior.name.startswith('Assembly_261020_Geometry_Only'):prior.close(False)
    source_doc.activate()
    if not source_design.exportManager.execute(source_design.exportManager.createFusionArchiveExportOptions(os.path.join(OUT,'Assembly_Working_20261020.f3d'))):raise RuntimeError('Latest working backup failed')
    geometry=world_bodies(root);mgr=adsk.fusion.TemporaryBRepManager.get()
    transforms={o.name:o.transform2.copy() for o in root.occurrences}
    def public_matrix(t):
        values=list(t.asArray())
        for i in [3,7,11]:values[i]*=10
        return values
    metadata=dict(document=source_doc.name,units='mm',components=[dict(name=o.name,transform_mm=public_matrix(o.transform2)) for o in root.occurrences if o.isVisible],
       rigid_groups=[dict(name=g.name,suppressed=g.isSuppressed,members=[o.fullPathName for o in g.occurrences]) for g in root.rigidGroups],
       joints=[dict(name=j.name,suppressed=j.isSuppressed,one=j.occurrenceOne.fullPathName,two=j.occurrenceTwo.fullPathName,type=j.jointMotion.objectType,rotation_rad=getattr(j.jointMotion,'rotationValue',None),slide_mm=(getattr(j.jointMotion,'slideValue',0)*10) if 'Slider' in j.jointMotion.objectType else None) for j in root.asBuiltJoints],
       transform_storage_note='Row-major 4x4 matrices; translation entries 3,7,11 are mm. Rotations are dimensionless.')
    write('native_assembly_manifest',metadata)
    doc=app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType);doc.name='Assembly_261020_Geometry_Only'
    design=adsk.fusion.Design.cast(app.activeProduct);newroot=design.rootComponent
    groups={}
    for path,temp,native in geometry:groups.setdefault(path.split('+')[0],[]).append((path,temp,native))
    body_count=0;material_copies={};part_mapping=[]
    for name,bodies in groups.items():
        transform=transforms.get(name,mat());inverse=transform.copy();inverse.invert()
        occ=newroot.occurrences.addNewComponent(transform);occ.component.name=name.replace(':','__');occ.isGroundToParent=False
        part_mapping.append(dict(source_occurrence=name,portable_component=occ.component.name))
        bf=occ.component.features.baseFeatures.add();bf.name='Visible_source_solids_20261020';bf.startEdit()
        try:
            for path,temp,native in bodies:
                mgr.transform(temp,inverse);body=occ.component.bRepBodies.add(temp,bf)
                body.name=native.name
                key=native.material.name
                if key not in material_copies:material_copies[key]=design.materials.addByCopy(native.material,'WafflePortable_'+str(len(material_copies)))
                body.material=material_copies[key]
                try:body.appearance=native.appearance
                except:pass
                body_count+=1
        finally:bf.finishEdit()
        write('portable_export_progress',dict(parts=len(list(newroot.occurrences)),total_parts=len(groups),bodies=body_count));adsk.doEvents()
    design.unitsManager.distanceDisplayUnits=adsk.fusion.DistanceUnits.MillimeterDistanceUnits
    archive=os.path.join(OUT,'Assembly_261020_Geometry_Only.f3d');step=os.path.join(OUT,'Assembly_261020_Visible.step')
    if not design.exportManager.execute(design.exportManager.createFusionArchiveExportOptions(archive)):raise RuntimeError('Portable F3D export failed')
    if not design.exportManager.execute(design.exportManager.createSTEPExportOptions(step)):raise RuntimeError('Portable STEP export failed')
    # Confirm the neutral export contains real source solids, including nested
    # components. An earlier component-only export produced an empty STEP.
    verify_portable(app)

def verify_portable(app):
    source_design=source(app);source_doc=app.activeDocument
    with open(os.path.join(OUT,'body_physics_geometry.json'),encoding='utf8') as f:expected=json.load(f)
    step=os.path.join(OUT,'Assembly_261020_Visible.step');archive=os.path.join(OUT,'Assembly_261020_Geometry_Only.f3d')
    check_doc=app.importManager.importToNewDocument(app.importManager.createSTEPImportOptions(step))
    design=adsk.fusion.Design.cast(app.activeProduct);root=design.rootComponent
    bodies=list(root.bRepBodies)
    for o in root.allOccurrences:bodies.extend(list(o.component.bRepBodies))
    volumes=[]
    for i,b in enumerate(bodies):
        volumes.append(b.getPhysicalProperties(adsk.fusion.CalculationAccuracy.MediumCalculationAccuracy).volume*1000)
        if i%25==0:write('portable_verify_progress',dict(checked=i+1,total=len(bodies)));adsk.doEvents()
    expected_volume=sum(b['volume_mm3'] for b in expected);actual_volume=sum(volumes)
    validation=dict(source_body_count=len(expected),step_body_count=len(bodies),source_volume_mm3=expected_volume,step_volume_mm3=actual_volume,accuracy='MediumCalculationAccuracy',bounding_box_mm=bb(root))
    write('portable_step_validation',validation)
    if len(bodies)!=len(expected) or abs(actual_volume-expected_volume)>max(.1,expected_volume*.001):raise RuntimeError('STEP body count or medium-accuracy volume mismatch')
    check_doc.close(False)
    names=sorted(set(r['path'].split('+')[0] for r in expected))
    write('portable_export_complete',dict(archive=archive,step=step,components=len(names),bodies=len(bodies),part_mapping=[dict(source_occurrence=n,portable_component=n.replace(':','__')) for n in names],limitations='Visible solid geometry and placements only. Joints/history remain in working archive and manifest, not in this neutral geometry document.'))
    source_doc.activate()
