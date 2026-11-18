import adsk.core,adsk.fusion,os,json,math
HERE=os.path.dirname(__file__);OUT=os.path.join(os.path.dirname(HERE),'Waffle_Assembly_Build','Oct19')

def save(name,data):
    with open(os.path.join(OUT,name+'.json'),'w',encoding='utf-8') as f:json.dump(data,f,ensure_ascii=False,indent=2)

def union_bounds(bodies):
    boxes=[b.boundingBox for b in bodies]
    return [[min(getattr(b.minPoint,k) for b in boxes) for k in ['x','y','z']],
            [max(getattr(b.maxPoint,k) for b in boxes) for k in ['x','y','z']]]

def all_geometry(occ,h):
    manager=adsk.fusion.TemporaryBRepManager.get();bs=[]
    def walk(comp,t):
        for b in comp.bRepBodies:
            c=manager.copy(b);manager.transform(c,t);bs.append(c)
        for o in comp.occurrences:walk(o.component,h['multiply'](t,o.transform2))
    walk(occ.component,occ.transform2);return bs

def inspect(app,design,h,extra):
    root=design.rootComponent
    o=next((o for o in root.occurrences if o.component.name=='OpenCR_Floor1_A'),None)
    if not o:
        o=root.occurrences.addNewComponent(adsk.core.Matrix3D.create());o.component.name='OpenCR_Floor1_A'
        options=app.importManager.createSTEPImportOptions(os.path.join(os.path.expanduser('~'),'Downloads','OPENCR.step'))
        options.isViewFit=False;app.importManager.importToTarget(options,o.component)
    o.isGroundToParent=False;h['capture'](design)
    bodies=all_geometry(o,h);bounds=union_bounds(bodies)
    report={'opencr':{'name':o.name,'bounds_cm':bounds,'size_mm':[(b-a)*10 for a,b in zip(*bounds)],'solid_count':len(bodies)},'lidars':[]}
    for name in ['LDS_Front_Underside:1','LDS_Rear_Underside:1']:
        lo=next(v for v in root.occurrences if v.name==name);bs=all_geometry(lo,h)
        report['lidars'].append({'name':name,'transform':lo.transform2.asArray(),'bounds_cm':union_bounds(bs)})
    save('electronics_inspection',report)
    original=[(v,v.isLightBulbOn) for v in root.occurrences];cam=app.activeViewport.camera
    for v in root.occurrences:v.isLightBulbOn=v==o
    c=app.activeViewport.camera;c.viewOrientation=adsk.core.ViewOrientations.IsoTopRightViewOrientation;c.isFitView=True;c.isSmoothTransition=False
    app.activeViewport.camera=c;app.activeViewport.refresh();adsk.doEvents();app.activeViewport.saveAsImageFile(os.path.join(OUT,'opencr_inspected.png'),1500,1100)
    for v,b in original:v.isLightBulbOn=b
    o.isLightBulbOn=False;app.activeViewport.camera=cam;app.activeViewport.refresh()

def layout(app,design,h,extra):
    root=design.rootComponent;manager=adsk.fusion.TemporaryBRepManager.get()
    def find(n):return next(o for o in root.occurrences if o.name==n)
    if any(o.component.name=='Jetson_Orin_Nano_DevKit_Envelope' for o in root.occurrences):raise RuntimeError('Electronics already installed; inspect before rerunning')
    report={'status':'working','parts':[],'hits':[],'lidars':[],'model_limits':['Battery and Jetson are manufacturer dimension envelopes, not detailed connector CAD.','Mounts and cable routing not designed.','Lidar scan is partly obscured by chassis sidewalls.']}
    frame=next(g for g in root.rigidGroups if g.name=='Waffle_4WD_Static_Frame')
    frame.isSuppressed=True;frame.timelineObject.rollTo(True)
    for name,y in [('LDS_Front_Underside:1',5),('LDS_Rear_Underside:1',-5)]:
        o=find(name);v=list(o.transform2.asArray());v[7]=y;o.isGroundToParent=False
        t=adsk.core.Matrix3D.create();t.setWithArray(v);o.transform2=t
    h['capture'](design);frame.setOccurrences(adsk.core.ObjectCollection.createWithArray(list(frame.occurrences)),True)
    design.timeline.moveToEnd();frame.isSuppressed=False;h['capture'](design)
    # STEP includes connector projections: 109.22 x 75 x 19 mm.
    a=find('OpenCR_Floor1_A:1');a.isGroundToParent=False;a.transform2=adsk.core.Matrix3D.create()
    bb=union_bounds(all_geometry(a,h));cx=(bb[0][0]+bb[1][0])/2;cy=(bb[0][1]+bb[1][1])/2
    a.transform2=h['matrix'](translation=(5.8-cx,4.1-cy,1.7-bb[0][2]));a.isLightBulbOn=True
    b=next((o for o in root.occurrences if o.component==a.component and o!=a),None)
    if not b:b=root.occurrences.addExistingComponent(a.component,adsk.core.Matrix3D.create())
    b.isGroundToParent=False
    b.transform2=h['matrix'](translation=(5.8-cx,-4.1-cy,1.7-bb[0][2]));b.isLightBulbOn=True
    for o in a.component.allOccurrences:o.isLightBulbOn=True
    h['capture'](design)
    installed=[a,b]
    def envelope(name,size,center,bottom,rgb):
        o=root.occurrences.addNewComponent(adsk.core.Matrix3D.create());o.component.name=name;o.isGroundToParent=False
        bf=o.component.features.baseFeatures.add();bf.startEdit()
        box=adsk.core.OrientedBoundingBox3D.create(adsk.core.Point3D.create(center[0],center[1],bottom+size[2]/2),adsk.core.Vector3D.create(1,0,0),adsk.core.Vector3D.create(0,1,0),*size)
        body=o.component.bRepBodies.add(manager.createBox(box),bf);bf.finishEdit();body.name=name
        h['color_body'](design,body,name+'_Color',rgb);o.appearance=body.appearance;return o
    installed.append(envelope('Jetson_Orin_Nano_DevKit_Envelope',(10.3,9.05,3.477),(-6,0),1.3,(60,145,75)))
    for side,y in [('Left',6.7),('Right',-6.7)]:installed.append(envelope('ROBOTIS_LB012_Battery_'+side+'_Envelope',(8.8,3.5,2.6),(-6,y),1.3,(50,85,185)))
    # Existing USB interface was below the lower floor. Place it below the upper deck.
    usb=find('USB2LSD:4');v=list(usb.transform2.asArray());v[11]+=8;usb.isGroundToParent=False
    t=adsk.core.Matrix3D.create();t.setWithArray(v);usb.transform2=t;h['capture'](design)
    for o in installed:
        bounds=union_bounds(all_geometry(o,h));report['parts'].append({'name':o.name,'bounds_cm':bounds,'size_mm':[(b-a)*10 for a,b in zip(*bounds)]})
    for n in ['LDS_Front_Underside:1','LDS_Rear_Underside:1']:
        bb=union_bounds(all_geometry(find(n),h));report['lidars'].append({'name':n,'bounds_cm':bb,'wall_gap_mm':min(9.1-bb[1][1],bb[0][1]+9.1)*10})
    save('electronics_layout',report)
    test=installed+[usb,find('LDS_Front_Underside:1'),find('LDS_Rear_Underside:1')]
    # Conservative whole-component boxes include the non-solid LDS housing.
    boxes={o.name:union_bounds(all_geometry(o,h)) for o in test}
    def meet(a,b):return all(min(a[1][k],b[1][k])-max(a[0][k],b[0][k])>.0001 for k in range(3))
    for i,o in enumerate(test):
        for p in test[i+1:]:
            if meet(boxes[o.name],boxes[p.name]):report['hits'].append({'part':o.name,'other':p.name,'type':'envelope'})
    installed_names={o.name for o in test}
    others=[o for o in root.occurrences if o.isLightBulbOn and o.name not in installed_names]
    for o in test:
        bb=boxes[o.name]
        for p in others:
            for solid in extra['occurrence_solids'](p,h):
                sb=union_bounds([solid])
                if not meet(bb,sb):continue
                # Component envelope is deliberately conservative, especially for the LDS surface shell.
                box=adsk.core.OrientedBoundingBox3D.create(adsk.core.Point3D.create(*[(x+y)/2 for x,y in zip(*bb)]),adsk.core.Vector3D.create(1,0,0),adsk.core.Vector3D.create(0,1,0),*[y-x for x,y in zip(*bb)])
                volume=extra['overlap'](manager.createBox(box),solid)
                if volume>.1:report['hits'].append({'part':o.name,'other':p.name,'mm3':volume})
        save('electronics_layout',report);adsk.doEvents()
    if report['hits']:report['status']='needs_review';save('electronics_layout',report);return
    group=root.rigidGroups.add(adsk.core.ObjectCollection.createWithArray([find('Floor_1_Joined:1')]+installed+[usb]),True);group.name='Floor1_Electronics_Fixed_Layout';h['capture'](design)
    for c in [root]+[o.component for o in root.occurrences]:
        try:c.isOriginFolderLightBulbOn=False
        except:pass
        for sk in c.sketches:sk.isLightBulbOn=False
        for cp in c.constructionPlanes:cp.isLightBulbOn=False
    original=[(o,o.isLightBulbOn) for o in root.occurrences]
    # Clear view of all first-floor components and the overhead lidar pair.
    visible={o.name for o in test}|{'Floor_1_Joined:1','Chassis_Bracket_Left_16mm:1','Chassis_Bracket_Right_16mm:1'}
    for o in root.occurrences:o.isLightBulbOn=o.name in visible
    for label,ori in [('floor1_top',adsk.core.ViewOrientations.TopViewOrientation),('floor1_iso',adsk.core.ViewOrientations.IsoTopRightViewOrientation)]:
        cam=app.activeViewport.camera;cam.viewOrientation=ori;cam.isFitView=True;cam.isSmoothTransition=False;app.activeViewport.camera=cam;app.activeViewport.refresh();adsk.doEvents();app.activeViewport.saveAsImageFile(os.path.join(OUT,label+'.png'),1800,1300)
    for o,v in original:o.isLightBulbOn=v
    cam=app.activeViewport.camera;cam.viewOrientation=adsk.core.ViewOrientations.IsoTopRightViewOrientation;cam.isFitView=True;cam.isSmoothTransition=False;app.activeViewport.camera=cam;app.activeViewport.refresh();adsk.doEvents()
    app.activeViewport.saveAsImageFile(os.path.join(OUT,'assembly_electronics_completed.png'),1800,1300)
    report['status']='complete';report['archive']=os.path.join(OUT,'Waffle_Assembly_20261019.f3d');save('electronics_layout',report)
    design.exportManager.execute(design.exportManager.createFusionArchiveExportOptions(report['archive']))
