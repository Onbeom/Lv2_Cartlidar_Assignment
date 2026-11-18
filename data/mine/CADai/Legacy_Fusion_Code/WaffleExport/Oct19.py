import adsk.core,adsk.fusion,os,json,math,datetime
HERE=os.path.dirname(__file__)
OUT=os.path.join(os.path.dirname(HERE),'Waffle_Assembly_Build','Oct19')
os.makedirs(OUT,exist_ok=True)

def save(name,data):
    with open(os.path.join(OUT,name+'.json'),'w',encoding='utf-8') as f:json.dump(data,f,ensure_ascii=False,indent=2)

def inspect(app,design,h):
    root=design.rootComponent
    archive=os.path.join(OUT,'Before_height_camera_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S')+'.f3d')
    design.exportManager.execute(design.exportManager.createFusionArchiveExportOptions(archive))
    report={'document':app.activeDocument.name,'backup':archive,'parts':[],
      'joints':[{'name':j.name,'suppressed':j.isSuppressed,'motion':j.jointMotion.objectType} for j in root.asBuiltJoints],
      'groups':[{'name':g.name,'suppressed':g.isSuppressed,'members':[o.name for o in g.occurrences]} for g in root.rigidGroups]}
    for o in root.occurrences:
        report['parts'].append({'name':o.name,'visible':o.isLightBulbOn,'transform':o.transform2.asArray(),
           'bodies':[{'name':b.name,'solid':b.isSolid,'bbox':h['bbox'](b.boundingBox),'volume':b.volume if b.isSolid else 0} for b in h['all_bodies'](o)]})
    save('current',report)
    cam=next((o for o in root.occurrences if o.component.name=='Camera_D435_Pair_Source'),None)
    if not cam:
        cam=root.occurrences.addNewComponent(adsk.core.Matrix3D.create());cam.component.name='Camera_D435_Pair_Source'
        options=app.importManager.createSTEPImportOptions(os.path.join(os.path.expanduser('~'),'Downloads','cam_bracket_v5_with_D435_F.step'))
        options.isViewFit=False;app.importManager.importToTarget(options,cam.component)
    h['capture'](design)
    rows=[]
    for o in cam.component.allOccurrences:
        rows.append({'name':o.name,'path':o.fullPathName,'transform':o.transform2.asArray(),'bodies':[h['body_info'](b) for b in o.bRepBodies]})
    save('camera',{'root_bodies':[h['body_info'](b) for b in cam.component.bRepBodies],'parts':rows})
    originals=[(o,o.isLightBulbOn) for o in root.occurrences];camera=app.activeViewport.camera
    for o in root.occurrences:o.isLightBulbOn=o==cam
    for label,ori in [('iso',adsk.core.ViewOrientations.IsoTopRightViewOrientation),('front',adsk.core.ViewOrientations.FrontViewOrientation),('top',adsk.core.ViewOrientations.TopViewOrientation)]:
        c=app.activeViewport.camera;c.viewOrientation=ori;c.isFitView=True;c.isSmoothTransition=False;app.activeViewport.camera=c;app.activeViewport.refresh();adsk.doEvents()
        app.activeViewport.saveAsImageFile(os.path.join(OUT,'camera_'+label+'.png'),1500,1100)
    for o,v in originals:o.isLightBulbOn=v
    cam.isLightBulbOn=False;app.activeViewport.camera=camera;app.activeViewport.refresh()

def rot(axis,angle,point):
    m=adsk.core.Matrix3D.create();m.setToRotation(angle,adsk.core.Vector3D.create(*axis),adsk.core.Point3D.create(*point));return m

def solve(x,y,z,pitch,grasp=28.5,branch=-1):
    l1=math.hypot(2.4,12.8);l2=12.4;tool=grasp+4.45-14.8
    r=math.hypot(x-8.7,y);p=math.radians(pitch)
    rw=r-tool*math.cos(p);zw=z-19.05-tool*math.sin(p)
    c=(rw*rw+zw*zw-l1*l1-l2*l2)/(2*l1*l2)
    if abs(c)>1:return None
    e=branch*math.acos(c);s=math.atan2(zw,rw)-math.atan2(l2*math.sin(e),l1+l2*math.cos(e));home=math.atan2(12.8,2.4)
    return [-math.atan2(y,x-8.7),s-home,e+home,p-s-e,0]

def transforms(q,h):
    mul=h['multiply'];m=[adsk.core.Matrix3D.create()]
    for angle,axis,point in zip(q,[(0,0,1),(0,1,0),(0,1,0),(0,1,0),(1,0,0)],[(8.7,0,15.235),(8.7,0,19.05),(11.1,0,31.85),(23.5,0,31.85),(30.34,0,31.85)]):
        m.append(mul(m[-1],rot(axis,-angle,point)))
    return m

def prepare_geometry(design,h):
    root=design.rootComponent;manager=adsk.fusion.TemporaryBRepManager.get()
    def find(n):return next(o for o in root.occurrences if o.name==n)
    with open(os.path.join(os.path.dirname(OUT),'manip_build.json'),encoding='utf-8') as f:log=json.load(f)
    arm=[]
    for e in log['created']:
        if e['group']<1:continue
        o=find(e['name'])
        for b in o.bRepBodies:
            if b.isSolid and b.volume>.015:arm.append((o.name,e['group'],world_copy(b)))
    for n,g in [('RMX52_DYNAMIXEL_Wrist_Roll:1',4),('RMX52_RW01_Reversed_Motor_Mount:1',4),('RMX52_Wrist_Horn_Adapter_3mm:1',5)]:
        for b in find(n).bRepBodies:
            if b.isSolid and b.volume>.015:arm.append((n,g,world_copy(b)))
    red=next(o.name for o in root.occurrences if o.name.startswith('Open CASCADE STEP translator'))
    names=['housing:1','housing:2',red,'magazine_B:1','magazine_G:1','magazine_R:1','rail_hi:1','rail_lo:1']
    storage=[(n,world_copy(b)) for n in names for b in h['all_bodies'](find(n)) if b.isSolid]
    fixed=[(n,world_copy(b)) for n in ['Floor_3_Joined:1','Profile_2020_450mm_Rear_Left_Envelope:1','Profile_2020_450mm_Rear_Right_Envelope:1'] for b in h['all_bodies'](find(n)) if b.isSolid]
    for name in ['Camera_D435_Pair_Source:1','Camera_Top_Crossbar_2020_261mm:1']:
        o=next((o for o in root.occurrences if o.name==name),None)
        if o:fixed.extend([(name,b) for b in occurrence_solids(o,h)])
    return arm,storage,fixed

def world_copy(b):
    manager=adsk.fusion.TemporaryBRepManager.get();c=manager.copy(b.nativeObject or b)
    if b.assemblyContext:manager.transform(c,b.assemblyContext.transform2)
    return c

def occurrence_solids(occ,h):
    manager=adsk.fusion.TemporaryBRepManager.get();result=[]
    def walk(comp,transform):
        for b in comp.bRepBodies:
            if b.isSolid:
                c=manager.copy(b);manager.transform(c,transform);result.append(c)
        for child in comp.occurrences:walk(child.component,h['multiply'](transform,child.transform2))
    walk(occ.component,occ.transform2)
    return result

def bounds6(body):
    b=body.boundingBox
    return (b.minPoint.x,b.minPoint.y,b.minPoint.z,b.maxPoint.x,b.maxPoint.y,b.maxPoint.z)

def bounds_meet(a,b):return all(a[k+3]>b[k]+.0001 and b[k+3]>a[k]+.0001 for k in range(3))

def transformed_bounds(box,transform):
    m=transform.asArray();pts=[]
    for x in [box[0],box[3]]:
        for y in [box[1],box[4]]:
            for z in [box[2],box[5]]:pts.append([m[4*k]*x+m[4*k+1]*y+m[4*k+2]*z+m[4*k+3] for k in range(3)])
    return tuple(min(p[k] for p in pts) for k in range(3))+tuple(max(p[k] for p in pts) for k in range(3))

def path_check(app,design,h):
    root=design.rootComponent;manager=adsk.fusion.TemporaryBRepManager.get()
    arm,storage,fixed=prepare_geometry(design,h)
    arm.sort(key=lambda e:-e[1])
    height=18;grasp=28.5
    stat=[]
    for n,b in storage:
        c=manager.copy(b);manager.transform(c,h['matrix'](translation=(0,0,4)));stat.append((n,c))
    stat.extend(fixed)
    stat_boxes=[(n,b,bounds6(b)) for n,b in stat]
    arm_boxes=[(n,g,b,bounds6(b)) for n,g,b in arm]
    red=next(o.name for o in root.occurrences if o.name.startswith('Open CASCADE STEP translator'))
    oldpath=os.path.join(OUT,'path_check.json')
    if os.path.exists(oldpath):
        with open(oldpath,encoding='utf-8') as f:report=json.load(f)
        report.pop('status',None)
    else:report={'height_cm':height,'grasp_source_x':grasp,'stages':[],'copy_checks':[]}
    completed={(s['storage'],s['stage']) for s in report['stages']}
    for o in root.occurrences:
        if o.name in ['housing:2','RMX52_IGP-A01_OPEN_1:1','Camera_D435_Pair_Source:1']:
            for b in h['all_bodies'](o):
                if b.isSolid:report['copy_checks'].append({'part':o.name,'proxy':h['bbox'](b.boundingBox),'implicit_copy':h['bbox'](manager.copy(b).boundingBox),'explicit_copy':h['bbox'](world_copy(b).boundingBox)})
    save('path_check',report)
    for letter,name in [('G','housing:2'),('B','housing:1'),('R',red)]:
        o=next(o for o in root.occurrences if o.name==name);m=o.transform2.asArray()
        x=m[0]*1.95+m[1]*1.95+m[3];y=m[4]*1.95+m[5]*1.95+m[7];nx,ny=m[4],-m[0]
        stages=[('retrieve_'+str(d),x+d*nx,y+d*ny,height+5,-15,True,d) for d in [0,.5,1,2,3,4,6]]
        stages.extend([('outside_lift_'+str(d),x+6*nx,y+6*ny,height+5+d,-15,True,6) for d in [1,2,4]])
        stages.extend([('insert_'+str(d),x+d*nx,y+d*ny,height+25.5,0,False,d) for d in [3,2,1,0]])
        for label,tx,ty,tz,pitch,retrieve,distance in stages:
            if (letter,label) in completed:continue
            q=solve(tx,ty,tz,pitch,grasp,-1)
            row={'storage':letter,'stage':label,'target_cm':[tx,ty,tz],'pitch_deg':pitch,'joint_radians':q,'hits':[]}
            if q:
                ts=transforms(q,h)
                for n,g,b,bb in arm_boxes:
                    moved_box=transformed_bounds(bb,ts[g]);c=None
                    for sn,sb,sbbox in stat_boxes:
                        if not bounds_meet(moved_box,sbbox):continue
                        if c is None:c=manager.copy(b);manager.transform(c,ts[g])
                        v=overlap(c,sb)
                        if v>.05:row['hits'].append({'part':n,'obstacle':sn,'overlap_mm3':v})
                    if row['hits']:break
                block_z=tz
                box=adsk.core.OrientedBoundingBox3D.create(adsk.core.Point3D.create(tx,ty,block_z),adsk.core.Vector3D.create(m[0],m[4],0),adsk.core.Vector3D.create(m[1],m[5],0),3,3,6)
                cube=manager.createBox(box)
                for sn,sb in stat:
                    v=overlap(cube,sb)
                    if v>.05:row['hits'].append({'part':'Block_30x30x60','obstacle':sn,'overlap_mm3':v})
            else:row['hits'].append({'unreachable':True})
            report['stages'].append(row);save('path_check',report);adsk.doEvents()
    report['status']='complete';save('path_check',report)

def apply_height(app,design,h):
    root=design.rootComponent;manager=adsk.fusion.TemporaryBRepManager.get()
    with open(os.path.join(OUT,'path_check.json'),encoding='utf-8') as f:path=json.load(f)
    if path.get('status')!='complete':raise RuntimeError('Path checks are not finished')
    bad=[s for s in path['stages'] if any(r.get('part')!='Block_30x30x60' for r in s['hits'])]
    if bad:save('path_blockers',bad);raise RuntimeError('Arm path still has collisions; height not committed')
    arm,storage,fixed=prepare_geometry(design,h)
    arm=[(n,g,b) for n,g,b in arm if b.volume>5 or n.startswith(('RMX52_IGP','RMX52_IRB','RMX52_ICA','RMX52_IBL','RMX52_ILR','RMX52_FR12','RMX52_ILA','RMX52_RW01','RMX52_Wrist_Horn'))]
    shifted=[]
    for n,b in storage:
        c=manager.copy(b);manager.transform(c,h['matrix'](translation=(0,0,4)));shifted.append((n,c))
    camera=next(o for o in root.occurrences if o.component.name=='Camera_D435_Pair_Source')
    cameras=occurrence_solids(camera,h)
    bars=occurrence_solids(next(o for o in root.occurrences if o.component.name=='Camera_Top_Crossbar_2020_261mm'),h)
    camera_boxes=[(b,bounds6(b)) for b in cameras+bars]
    arm_boxes=[(n,g,b,bounds6(b)) for n,g,b in arm]
    static_boxes=[(n,b,bounds6(b)) for n,b in shifted]+[(camera.name,b,bb) for b,bb in camera_boxes]
    report={'height_cm':18,'object_support_z_relative_cm':2,'object_center_z_relative_cm':5,'checks':[],'camera_bar_checks':[]}
    for b in cameras:report['camera_bar_checks'].append(overlap(b,bars[0]))
    for stage in path['stages']:
        row={'storage':stage['storage'],'stage':stage['stage'],'hits':[]}
        q=stage['joint_radians'];ts=transforms(q,h)
        target=stage['target_cm']
        # The 30 mm block bridges the two 20 mm-high ledges, rather than falling into the 23 mm bottom gap.
        box=adsk.core.OrientedBoundingBox3D.create(adsk.core.Point3D.create(*target),adsk.core.Vector3D.create(1,0,0),adsk.core.Vector3D.create(0,1,0),3,3,6)
        cube=manager.createBox(box)
        name='housing:2' if stage['storage']=='G' else ('housing:1' if stage['storage']=='B' else next(o.name for o in root.occurrences if o.name.startswith('Open CASCADE STEP translator')))
        o=next(o for o in root.occurrences if o.name==name);a=o.transform2.asArray()
        # Rotate the square block to the storage's slot direction.
        theta=math.atan2(a[4],a[0]);manager.transform(cube,rot((0,0,1),theta,target))
        cube_box=bounds6(cube)
        for sn,sb,sbbox in static_boxes:
            v=overlap(cube,sb) if bounds_meet(cube_box,sbbox) else 0
            if v>.05:row['hits'].append({'part':'30x30x60_Block','obstacle':sn,'mm3':v})
        for n,g,b,bb in arm_boxes:
            mb=transformed_bounds(bb,ts[g]);c=None
            for cb,cbbox in camera_boxes:
                if not bounds_meet(mb,cbbox):continue
                if c is None:c=manager.copy(b);manager.transform(c,ts[g])
                v=overlap(c,cb)
                if v>.05:row['hits'].append({'part':n,'obstacle':'Top_Camera_Assembly','mm3':v})
        report['checks'].append(row);save('height_final_checks',report);adsk.doEvents()
    if any(s['hits'] for s in report['checks']):raise RuntimeError('Block/camera path needs correction; height not committed')
    names=[n for n,b in storage];poses={n:next(o for o in root.occurrences if o.name==n).transform2.asArray() for n in set(names)}
    frame=next(g for g in root.rigidGroups if g.name=='Waffle_4WD_Static_Frame')
    frame.isSuppressed=True;frame.timelineObject.rollTo(True)
    for name,values in poses.items():
        o=next(o for o in root.occurrences if o.name==name);o.isGroundToParent=False
        values=list(values);values[11]+=4;t=adsk.core.Matrix3D.create();t.setWithArray(values);o.transform2=t
    h['capture'](design);frame.setOccurrences(adsk.core.ObjectCollection.createWithArray(list(frame.occurrences)),True)
    design.timeline.moveToEnd();frame.isSuppressed=False;h['capture'](design);adsk.doEvents()
    report['committed']=True;report['storage_bounds']=[{'name':n,'transform':next(o for o in root.occurrences if o.name==n).transform2.asArray()} for n in poses]
    guide=root.occurrences.addNewComponent(adsk.core.Matrix3D.create());guide.component.name='REFERENCE_Block_30x30x60mm';guide.isGroundToParent=False
    bf=guide.component.features.baseFeatures.add();bf.startEdit()
    box=adsk.core.OrientedBoundingBox3D.create(adsk.core.Point3D.create(0,0,0),adsk.core.Vector3D.create(1,0,0),adsk.core.Vector3D.create(0,1,0),3,3,6)
    b=guide.component.bRepBodies.add(manager.createBox(box),bf);bf.finishEdit();h['color_body'](design,b,'Reference_Object_Amber',(245,180,40));guide.appearance=b.appearance
    guide.transform2=h['matrix'](translation=(-7.03,0,23));h['capture'](design)
    # Verify actual Fusion joints against one center storage pose after the frame edit.
    joints=[next(j for j in root.asBuiltJoints if j.name=='RMX52_'+n) for n in ['Base_Yaw','Shoulder','Elbow','Wrist_Pitch','Wrist_Roll_Reversed']]
    example=next(s for s in path['stages'] if s['storage']=='G' and s['stage']=='retrieve_0')
    for j,v in zip(joints,example['joint_radians']):j.jointMotion.rotationValue=v
    adsk.doEvents()
    source=next(o for o in root.occurrences if o.name=='RM_X52_Manipulator:1')
    src=next(o for o in source.component.allOccurrences if o.name=='IGP-A01_OPEN:1');inv=src.transform2;inv.invert()
    p=adsk.core.Point3D.create(28.5,21.05,0);p.transformBy(inv)
    actual=next(o for o in root.occurrences if o.name=='RMX52_IGP-A01_OPEN_1:1');p.transformBy(actual.transform2)
    report['actual_grip_cm']=[p.x,p.y,p.z];report['actual_grip_error_mm']=10*math.dist(report['actual_grip_cm'],example['target_cm'])
    app.activeViewport.refresh();adsk.doEvents();app.activeViewport.saveAsImageFile(os.path.join(OUT,'G_retrieve_verified.png'),1800,1300)
    insert=next(s for s in path['stages'] if s['storage']=='G' and s['stage']=='insert_0')
    guide.transform2=h['matrix'](translation=tuple(insert['target_cm']))
    for j,v in zip(joints,insert['joint_radians']):j.jointMotion.rotationValue=v
    adsk.doEvents();app.activeViewport.refresh();adsk.doEvents();app.activeViewport.saveAsImageFile(os.path.join(OUT,'G_insert_verified.png'),1800,1300)
    for j in joints:j.jointMotion.rotationValue=0
    guide.isLightBulbOn=False;adsk.doEvents();h['capture'](design)
    report['status']='complete';save('height_final_checks',report)
    design.exportManager.execute(design.exportManager.createFusionArchiveExportOptions(os.path.join(OUT,'Assembly_Height_Cameras_Completed.f3d')))
    app.activeViewport.refresh();adsk.doEvents();app.activeViewport.saveAsImageFile(os.path.join(OUT,'assembly_completed.png'),1800,1300)

def overlap(a,b):
    qa,qb=a.boundingBox,b.boundingBox
    if any(getattr(qa.maxPoint,k)<=getattr(qb.minPoint,k)+.0001 or getattr(qb.maxPoint,k)<=getattr(qa.minPoint,k)+.0001 for k in ['x','y','z']):return 0
    manager=adsk.fusion.TemporaryBRepManager.get();a=manager.copy(a)
    ok=manager.booleanOperation(a,b,adsk.fusion.BooleanTypes.IntersectionBooleanType)
    if not ok:raise RuntimeError('Temporary intersection failed')
    return a.volume*1000 if a.faces.count else 0

def height_search(app,design,h,request=None):
    request=request or {}
    root=design.rootComponent;manager=adsk.fusion.TemporaryBRepManager.get()
    arm,storage,fixed=prepare_geometry(design,h)
    report={'trials':[],'geometry_counts':[len(arm),len(storage),len(fixed)]}
    # Keep precise geometry for the front-end and larger links; omit source fastener internals.
    major=[]
    for n,g,b in arm:
        if n.startswith(('RMX52_IGP','RMX52_IRB','RMX52_ICA','RMX52_IBL','RMX52_ILR','RMX52_FR12','RMX52_ILA','RMX52_RW01','RMX52_Wrist_Horn')) or b.volume>5:
            major.append((n,g,b))
    report['tested_bodies']=len(major)
    major.sort(key=lambda e:-e[1])
    x,y=-7.03,0
    for height in request.get('heights',[14,18,22,24,26]):
        shift=h['matrix'](translation=(0,0,height-14));stat=[]
        for n,b in storage:
            c=manager.copy(b);manager.transform(c,shift);stat.append((n,c))
        stat.extend(fixed)
        for grasp in request.get('grasps',[28.5,27]):
            for pitch in request.get('pitches',[0,-15,-30,-45,15]):
                for branch in [-1,1]:
                    q=solve(x,y,height+request.get('grip_height',3.3),pitch,grasp,branch)
                    if not q:continue
                    row={'height_cm':height,'grasp_source_x':grasp,'pitch_deg':pitch,'branch':branch,'joint_radians':q,'hits':[]}
                    ts=transforms(q,h)
                    moved=[]
                    for n,g,b in major:
                        c=manager.copy(b);manager.transform(c,ts[g]);moved.append((n,g,c))
                    for n,g,b in moved:
                        for sn,sb in stat:
                            v=overlap(b,sb)
                            if v>.05:row['hits'].append({'arm':n,'static':sn,'mm3':v});break
                        if row['hits']:break
                    if not row['hits']:
                        # Nonadjacent links must also avoid each other.
                        for i,(n,g,b) in enumerate(moved):
                            for nn,gg,bb in moved[i+1:]:
                                if abs(g-gg)<2:continue
                                v=overlap(b,bb)
                                if v>1:row['hits'].append({'arm':n,'other_arm':nn,'mm3':v});break
                            if row['hits']:break
                    report['trials'].append(row);save(request.get('report','height_search'),report);adsk.doEvents()
                    if not row['hits'] and request.get('stop_clear'):
                        report['status']='clear_candidate';save(request.get('report','height_search'),report);return
    # Native mount surfaces and finger geometry support the next detailed review.
    detail={}
    for name in ['RMX52_IGP-A01_OPEN_1:1','RMX52_IGP-A01_OPEN_2:1']:
        o=next(o for o in root.occurrences if o.name==name)
        detail[name]=[{'vertices':[[v.geometry.x,v.geometry.y,v.geometry.z] for v in b.vertices]} for b in o.component.bRepBodies if b.isSolid]
    cam=next(o for o in root.occurrences if o.component.name=='Camera_D435_Pair_Source')
    for o in cam.component.allOccurrences:
        if o.name=='bracket_v5:1':
            detail['camera_bracket']=[{'vertices':[[v.geometry.x,v.geometry.y,v.geometry.z] for v in b.vertices],
              'planes':[{'area':f.area,'normal':[f.geometry.normal.x,f.geometry.normal.y,f.geometry.normal.z],
                'point':[f.centroid.x,f.centroid.y,f.centroid.z]} for f in b.faces if isinstance(f.geometry,adsk.core.Plane)]} for b in o.bRepBodies]
    save('details',detail);report['status']='complete';save(request.get('report','height_search'),report)

def mount_camera(app,design,h):
    root=design.rootComponent;manager=adsk.fusion.TemporaryBRepManager.get()
    if any(o.component.name=='Camera_Top_Crossbar_2020_261mm' for o in root.occurrences):raise RuntimeError('Crossbar is already present')
    frame=next(g for g in root.rigidGroups if g.name=='Waffle_4WD_Static_Frame')
    frame.isSuppressed=True;frame.timelineObject.rollTo(True)
    left=next(o for o in root.occurrences if o.name=='Profile_2020_450mm_Rear_Left_Envelope:1')
    t=left.transform2;v=list(t.asArray());v[11]=1.2;t.setWithArray(v);left.isGroundToParent=False;left.transform2=t
    h['capture'](design);frame.setOccurrences(adsk.core.ObjectCollection.createWithArray(list(frame.occurrences)),True)
    design.timeline.moveToEnd();frame.isSuppressed=False
    camera=next(o for o in root.occurrences if o.component.name=='Camera_D435_Pair_Source')
    camera.isGroundToParent=False
    camera.transform2=h['matrix']([[0,0,1],[1,0,0],[0,1,0]],(-12.8,-.252,48.15));camera.isLightBulbOn=True
    for o in camera.component.allOccurrences:o.isLightBulbOn=True
    h['capture'](design)
    bar=root.occurrences.addNewComponent(adsk.core.Matrix3D.create());bar.component.name='Camera_Top_Crossbar_2020_261mm';bar.isGroundToParent=False
    bf=bar.component.features.baseFeatures.add();bf.startEdit()
    box=adsk.core.OrientedBoundingBox3D.create(adsk.core.Point3D.create(-12.8,0,47.9),adsk.core.Vector3D.create(1,0,0),adsk.core.Vector3D.create(0,1,0),2,26.1,2)
    body=bar.component.bRepBodies.add(manager.createBox(box),bf);bf.finishEdit();body.name='2020_261mm_Outer_Envelope'
    h['color_body'](design,body,'Camera_Crossbar_Aluminum',(185,190,195));bar.appearance=body.appearance
    h['capture'](design)
    floor=next(o for o in root.occurrences if o.name=='Floor_3_Joined:1')
    group=root.rigidGroups.add(adsk.core.ObjectCollection.createWithArray([floor,bar,camera]),True);group.name='Camera_Top_Fixed_Assembly'
    h['capture'](design);adsk.doEvents()
    report={'bar_length_mm':261,'bar_section_mm':[20,20],'bar_center_cm':[-12.8,0,47.9],'camera_transform':camera.transform2.asArray(),'checks':[]}
    for b in h['all_bodies'](camera):
        if b.isSolid:report['checks'].append({'camera_body':b.name,'crossbar_overlap_mm3':overlap(manager.copy(b),manager.copy(body))})
    save('camera_mounted',report)
    cam=app.activeViewport.camera;cam.viewOrientation=adsk.core.ViewOrientations.IsoTopRightViewOrientation;cam.isFitView=True;cam.isSmoothTransition=False
    app.activeViewport.camera=cam;app.activeViewport.refresh();adsk.doEvents();app.activeViewport.saveAsImageFile(os.path.join(OUT,'camera_mounted.png'),1800,1300)
    design.exportManager.execute(design.exportManager.createFusionArchiveExportOptions(os.path.join(OUT,'Assembly_With_Top_Cameras.f3d')))

def run(app,design,request,h):
    if request['action'].startswith('lds_'):
        import importlib.util
        spec=importlib.util.spec_from_file_location('LidarMotion19',os.path.join(HERE,'LidarMotion19.py'))
        mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod);mod.run(app,design,h,globals(),request);return
    if request['action']=='inspect':inspect(app,design,h)
    elif request['action']=='height_search':height_search(app,design,h,request)
    elif request['action']=='mount_camera':mount_camera(app,design,h)
    elif request['action']=='path_check':path_check(app,design,h)
    elif request['action']=='apply_height':apply_height(app,design,h)
    elif request['action']=='finalize':
        root=design.rootComponent
        app.activeViewport.visualStyle=adsk.core.VisualStyles.ShadedVisualStyle
        original=[(o,o.isLightBulbOn) for o in root.occurrences]
        visible={'Floor_1_Joined:1','OpenCR_Floor1_A:1','OpenCR_Floor1_A:2','Jetson_Orin_Nano_DevKit_Envelope:1','ROBOTIS_LB012_Battery_Left_Envelope:1','ROBOTIS_LB012_Battery_Right_Envelope:1'}
        for o in root.occurrences:o.isLightBulbOn=o.name in visible
        c=app.activeViewport.camera;c.viewOrientation=adsk.core.ViewOrientations.TopViewOrientation;c.isFitView=True;c.isSmoothTransition=False;app.activeViewport.camera=c;app.activeViewport.refresh();adsk.doEvents()
        app.activeViewport.saveAsImageFile(os.path.join(OUT,'floor1_electronics_clear.png'),1600,1200)
        for o,v in original:o.isLightBulbOn=v
        c=app.activeViewport.camera;c.viewOrientation=adsk.core.ViewOrientations.IsoTopRightViewOrientation;c.isFitView=True;c.isSmoothTransition=False;app.activeViewport.camera=c;app.activeViewport.refresh();adsk.doEvents()
        app.activeViewport.saveAsImageFile(os.path.join(OUT,'assembly_final.png'),1800,1300)
        h['capture'](design)
        saved=app.activeDocument.save('Housing and rails +40 mm; top camera crossbar; first-floor electronics layout; lidar wall clearance') if app.activeDocument.dataFile else False
        design.exportManager.execute(design.exportManager.createFusionArchiveExportOptions(os.path.join(OUT,'Waffle_Assembly_20261019_Final.f3d')))
        save('final_saved',{'cloud_save_return':saved,'document':app.activeDocument.name,'archive':'Waffle_Assembly_20261019_Final.f3d'})
    elif request['action'] in ['electronics_inspect','electronics_layout']:
        import importlib.util
        spec=importlib.util.spec_from_file_location('Electronics19',os.path.join(HERE,'Electronics19.py'))
        mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
        if request['action']=='electronics_inspect':mod.inspect(app,design,h,globals())
        else:mod.layout(app,design,h,globals())
