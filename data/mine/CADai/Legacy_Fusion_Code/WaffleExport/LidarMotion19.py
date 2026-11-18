import adsk.core,adsk.fusion,os,json,math,time,threading,builtins,traceback
HERE=os.path.dirname(__file__);OUT=os.path.join(os.path.dirname(HERE),'Waffle_Assembly_Build','Oct19','LidarMotion')
os.makedirs(OUT,exist_ok=True)
def save(n,d):
    with open(os.path.join(OUT,n+'.json'),'w',encoding='utf-8') as f:json.dump(d,f,indent=2,ensure_ascii=False)
def inspect(app,design,h,e):
    root=design.rootComponent;report={}
    o=next(o for o in root.occurrences if o.name=='LDS_Front_Underside:1')
    def walk(comp,t):
        for b in comp.bRepBodies:
            body=adsk.fusion.TemporaryBRepManager.get().copy(b);adsk.fusion.TemporaryBRepManager.get().transform(body,t)
            rows=[]
            for f in body.faces:
                g=f.geometry
                if isinstance(g,adsk.core.Cylinder):rows.append({'type':'cylinder','radius':g.radius,'origin':g.origin.asArray(),'axis':g.axis.asArray(),'bbox':h['bbox'](f.boundingBox),'area':f.area})
                if isinstance(g,adsk.core.Plane) and abs(g.normal.z)>.99 and f.area>.5:rows.append({'type':'plane','normal':g.normal.asArray(),'center':f.centroid.asArray(),'area':f.area,'bbox':h['bbox'](f.boundingBox)})
            report.setdefault('lidar',[]).append({'name':b.name,'solid':b.isSolid,'faces':rows,'bounds':h['bbox'](body.boundingBox)})
        for child in comp.occurrences:walk(child.component,h['multiply'](t,child.transform2))
    walk(o.component,o.transform2)
    floor=next(o for o in root.occurrences if o.name=='Floor_3_Joined:1')
    report['floor_circles']=[]
    for b in e['occurrence_solids'](floor,h):
        for ed in b.edges:
            g=ed.geometry
            if isinstance(g,adsk.core.Circle3D) and abs(g.normal.z)>.99 and abs(g.center.x)>9 and abs(g.center.y)<5:
                report['floor_circles'].append({'center':g.center.asArray(),'radius':g.radius})
    save('inspect',report)
def run(app,design,h,e,request):
    if request['action']=='lds_inspect':inspect(app,design,h,e)
    elif request['action']=='lds_build':build(app,design,h,e)
    elif request['action']=='lds_motion_prepare':prepare_motion(app,design,h,e)
    elif request['action']=='lds_motion_arm':arm_recording(app,design,h,e)
    elif request['action']=='lds_motion_play':arm_recording(app,design,h,e,True)
    elif request['action']=='lds_finalize':finalize(app,design,h,e)

def build(app,design,h,e):
    root=design.rootComponent;mgr=adsk.fusion.TemporaryBRepManager.get()
    def find(n):return next(o for o in root.occurrences if o.name==n)
    if any(o.component.name=='LDS_Front_Sliding_Carrier' for o in root.occurrences):raise RuntimeError('LDS brackets already exist')
    design.exportManager.execute(design.exportManager.createFusionArchiveExportOptions(os.path.join(OUT,'Before_LDS_Brackets.f3d')))
    app.userInterface.activeSelections.clear()
    frame=next(g for g in root.rigidGroups if g.name=='Waffle_4WD_Static_Frame')
    frame.isSuppressed=True;frame.timelineObject.rollTo(True)
    kept=[o for o in frame.occurrences if not o.fullPathName.startswith(('LDS_Front_Underside:1','LDS_Rear_Underside:1'))]
    frame.setOccurrences(adsk.core.ObjectCollection.createWithArray(kept),True)
    for name,sign in [('LDS_Front_Underside:1',1),('LDS_Rear_Underside:1',-1)]:
        o=find(name);v=list(o.transform2.asArray());v[3]=sign*18.55;v[7]=sign*5;v[11]=18.60882121226992
        o.isGroundToParent=False;t=adsk.core.Matrix3D.create();t.setWithArray(v);o.transform2=t
    h['capture'](design);design.timeline.moveToEnd();frame.isSuppressed=False;h['capture'](design)
    def box(lo,hi):
        return mgr.createBox(adsk.core.OrientedBoundingBox3D.create(adsk.core.Point3D.create(*[(a+b)/2 for a,b in zip(lo,hi)]),adsk.core.Vector3D.create(1,0,0),adsk.core.Vector3D.create(0,1,0),*[b-a for a,b in zip(lo,hi)]))
    def cylinder(x,y,z1,z2,r):return mgr.createCylinderOrCone(adsk.core.Point3D.create(x,y,z1),r,adsk.core.Point3D.create(x,y,z2),r)
    def boolean(body,tool,kind):
        if not mgr.booleanOperation(body,tool,kind):raise RuntimeError('Bracket Boolean failed')
    def drill(body,x,y,r,z1=5,z2=11):boolean(body,cylinder(x,y,z1,z2,r),adsk.fusion.BooleanTypes.DifferenceBooleanType)
    def slot(body,x1,x2,y,r,z1,z2):
        for tool in [box((x1,y-r,z1),(x2,y+r,z2)),cylinder(x1,y,z1,z2,r),cylinder(x2,y,z1,z2,r)]:boolean(body,tool,adsk.fusion.BooleanTypes.DifferenceBooleanType)
    def part(name,temp,sign,rgb):
        o=root.occurrences.addNewComponent(adsk.core.Matrix3D.create());o.component.name=name;o.isGroundToParent=False
        if sign<0:mgr.transform(temp,h['matrix']([[-1,0,0],[0,-1,0],[0,0,1]]))
        bf=o.component.features.baseFeatures.add();bf.startEdit();body=o.component.bRepBodies.add(temp,bf);bf.finishEdit();body.name=name
        h['color_body'](design,body,name+'_Color',rgb);o.appearance=body.appearance;h['capture'](design);return o
    report={'status':'building','adjustment_travel_mm':40,'current_scan_head_gap_mm':15,'parts':[],'mounts':[],'fov':[]}
    sketch=root.sketches.add(root.xYConstructionPlane);sketch.name='LDS_Adjustment_Axes'
    for label,sign in [('Front',1),('Rear',-1)]:
        base=box((9.2,.6,9.9),(14.5,8.5,10.2))
        # 40 mm between slot end centres; 3.4 mm neck, 6.4 mm captive-nut channel.
        for y in [2,8]:
            slot(base,10,14,y,.17,9.89,10.21);slot(base,10,14,y,.32,10.0,10.21)
        deck_holes=[(12,1.1),(10.3,1.8),(10.3,3.6),(11.9,3.6)]
        for x,y in deck_holes:drill(base,x,y,.135,9.89,10.21)
        bo=part('LDS_'+label+'_Deck_Fixed_Slotted_Base',base,sign,(130,145,155))
        carrier=box((9.2,1.3,9.6),(21.6,8.2,9.9))
        # Through bores align with the source LDS mounting triangle. The side spacers
        # stop above the counterbore lips; the front spacer seats on the PCB flange.
        mounts=[(17.65,2.55,7.075294932814519),(17.65,7.45,7.075294932814519),(21.1,4.97,6.758821212269919)]
        for x,y,z in mounts:
            boolean(carrier,cylinder(x,y,z,9.7,.32),adsk.fusion.BooleanTypes.UnionBooleanType)
            drill(carrier,x,y,.115,z-.01,10)
        for y in [2,8]:drill(carrier,11.5,y,.17,9.59,9.91)
        # Clear the centre above the rotating scanner and reduce the overhung mass.
        boolean(carrier,box((15.8,3.1,9.59),(20.6,6.8,9.91)),adsk.fusion.BooleanTypes.DifferenceBooleanType)
        co=part('LDS_'+label+'_Sliding_Carrier',carrier,sign,(225,155,45))
        floor=find('Floor_3_Joined:1');sensor=find('LDS_'+label+'_Underside:1')
        g=root.rigidGroups.add(adsk.core.ObjectCollection.createWithArray([floor,bo]),True);g.name='LDS_'+label+'_Base_On_Deck_Underside'
        g=root.rigidGroups.add(adsk.core.ObjectCollection.createWithArray([sensor,co]),True);g.name='LDS_'+label+'_Sensor_On_Carrier'
        h['capture'](design)
        p=sketch.sketchPoints.add(adsk.core.Point3D.create(sign*11.5,sign*5,9.9))
        ji=root.asBuiltJoints.createInput(bo,co,adsk.fusion.JointGeometry.createByPoint(p));ji.setAsSliderJointMotion(adsk.fusion.JointDirections.XAxisJointDirection)
        j=root.asBuiltJoints.add(ji);j.name='LDS_'+label+'_40mm_Adjustment';j.isLightBulbOn=False
        original_x=sensor.transform2.translation.x;j.jointMotion.slideValue=.1;adsk.doEvents();delta=sensor.transform2.translation.x-original_x;j.jointMotion.slideValue=0;adsk.doEvents()
        sense=1 if delta*sign>0 else -1
        lim=j.jointMotion.slideLimits;lim.minimumValue=min(-1.5*sense,2.5*sense);lim.maximumValue=max(-1.5*sense,2.5*sense);lim.isMinimumValueEnabled=True;lim.isMaximumValueEnabled=True
        j.attributes.add('Waffle','NominalGap_mm','15');j.attributes.add('Waffle','Travel_mm','40')
        report['parts'] += [bo.name,co.name]
        report['mounts'].append({'side':label,'deck_holes_cm':[[sign*x,sign*y,10.2] for x,y in deck_holes],'sensor_bores_cm':[[sign*x,sign*y,z] for x,y,z in mounts],'slide_direction_sense':sense,'limits_cm':[lim.minimumValue,lim.maximumValue]})
        save('brackets',report)
    sketch.isLightBulbOn=False;h['capture'](design);adsk.doEvents()
    # Conservative body bounds certify an outward half-space throughout the optical
    # opening below the PCB, not only one laser ray. The sensor's own optics are excluded.
    for side,sign in [('Front',1),('Rear',-1)]:
        rows=[];origin=sign*18.55
        for o in root.occurrences:
            if not o.isLightBulbOn or o.name=='LDS_'+side+'_Underside:1':continue
            for body in e['occurrence_solids'](o,h):
                b=body.boundingBox
                if b.maxPoint.z<5.47 or b.minPoint.z>6.70:continue
                extreme=b.maxPoint.x if sign>0 else -b.minPoint.x
                if extreme>18.55+.001:rows.append({'part':o.name,'bounds':h['bbox'](b)})
        report['fov'].append({'side':side,'outward_clear_degrees':180 if not rows else None,'optical_clear_band_z_mm':[54.7,67.0],'blocking_parts':rows,'scan_axis_xy_cm':[origin,sign*5]})
    # Measure plate/deck and all bracket solids against other solid components.
    report['interferences']=[]
    targets=[o for o in root.occurrences if o.name in report['parts']]
    for o in targets:
        for a in e['occurrence_solids'](o,h):
            for p in root.occurrences:
                if not p.isLightBulbOn or p==o:continue
                for b in e['occurrence_solids'](p,h):
                    v=e['overlap'](a,b)
                    if v>.1:report['interferences'].append({'part':o.name,'other':p.name,'mm3':v})
        save('brackets',report);adsk.doEvents()
    report['status']='review' if report['interferences'] or any(f['blocking_parts'] for f in report['fov']) else 'complete';save('brackets',report)
    cam=app.activeViewport.camera;cam.viewOrientation=adsk.core.ViewOrientations.IsoTopRightViewOrientation;cam.isFitView=True;cam.isSmoothTransition=False;app.activeViewport.camera=cam;app.activeViewport.refresh();adsk.doEvents()
    app.activeViewport.saveAsImageFile(os.path.join(OUT,'LDS_Brackets_Assembly.png'),1800,1300)
    design.exportManager.execute(design.exportManager.createFusionArchiveExportOptions(os.path.join(OUT,'Waffle_Adjustable_LDS.f3d')))

def sequence(e):
    frames=[];current=(-4.03,0,43.5,0)
    def segment(end,n,label,carry=False,block=None):
        nonlocal current
        for i in range(n):
            a=(i+1)/n;a=a*a*(3-2*a);p=[x+(y-x)*a for x,y in zip(current,end)]
            q=e['solve'](*p[:3],p[3],28.5,-1)
            if not q:raise RuntimeError('Unreachable '+label+str(p))
            frames.append({'label':label,'target':p,'q':q,'block':p[:3] if carry else (block or [-7.03,0,23])})
        current=end
    segment(current,18,'INSERT_APPROACH',True)
    segment((-7.03,0,43.5,0),48,'INSERT',True)
    segment(current,12,'RELEASE',True)
    for i in range(18):
        a=(i+1)/18;frames.append({'label':'OBJECT_DROPS_INTO_MAGAZINE','target':list(current),'q':e['solve'](*current[:3],current[3]),'block':[-7.03,0,43.5-20.5*a*a]})
    segment((-4.03,0,43.5,0),36,'WITHDRAW_FROM_MOUTH')
    segment((-1.03,0,38,-15),24,'CLEAR_MOUTH')
    segment((-1.03,0,27,-15),60,'MOVE_TO_RETRIEVAL_HEIGHT')
    segment((-1.03,0,23,-15),24,'RETRIEVAL_APPROACH')
    segment((-7.03,0,23,-15),48,'RETRIEVE')
    segment(current,18,'GRASP')
    segment((-1.03,0,23,-15),48,'EXTRACT',True)
    segment((-1.03,0,27,-15),24,'LIFT_CLEAR',True)
    segment(current,24,'DONE',True)
    return frames

def prepare_motion(app,design,h,e):
    root=design.rootComponent;mgr=adsk.fusion.TemporaryBRepManager.get();frames=sequence(e)
    arm,storage,fixed=e['prepare_geometry'](design,h)
    for o in root.occurrences:
        if o.name.startswith(('LDS_Front_','LDS_Rear_')):fixed.extend([(o.name,b) for b in e['occurrence_solids'](o,h)])
    stat=[(n,b,e['bounds6'](b)) for n,b in storage+fixed]
    armb=[(n,g,b,e['bounds6'](b)) for n,g,b in arm]
    report={'status':'checking','frame_count':len(frames),'nominal_fps':12,'samples':[],'hits':[]}
    labels={}
    for i,f in enumerate(frames):labels.setdefault(f['label'],[]).append(i)
    indices=set(range(0,len(frames),8))|{i for indices in labels.values() for i in [indices[0],indices[-1]]}
    for idx in sorted(indices):
        f=frames[idx];ts=e['transforms'](f['q'],h);moved=[];row={'frame':idx,'label':f['label'],'hits':[]}
        for name,g,b,bb in armb:
            mb=e['transformed_bounds'](bb,ts[g]);c=None
            for sn,sb,sbb in stat:
                if not e['bounds_meet'](mb,sbb):continue
                if c is None:c=mgr.copy(b);mgr.transform(c,ts[g])
                v=e['overlap'](c,sb)
                if v>.1:row['hits'].append({'arm':name,'obstacle':sn,'mm3':v})
            moved.append((name,g,b,mb,ts[g]))
        # Source object follows the hand during carried segments and gravity during release.
        center=f['block'];cube=mgr.createBox(adsk.core.OrientedBoundingBox3D.create(adsk.core.Point3D.create(*center),adsk.core.Vector3D.create(1,0,0),adsk.core.Vector3D.create(0,1,0),3,3,6));cb=e['bounds6'](cube)
        for sn,sb,sbb in stat:
            if not e['bounds_meet'](cb,sbb):continue
            v=e['overlap'](cube,sb)
            if v>.1:row['hits'].append({'object':'30x30x60','obstacle':sn,'mm3':v})
        # Non-neighbouring rigid links: broad-phase then actual BRep intersection.
        for i,(n,g,b,bb,t) in enumerate(moved):
            for nn,gg,bbb,bb2,t2 in moved[i+1:]:
                if abs(g-gg)<2 or not e['bounds_meet'](bb,bb2):continue
                c=mgr.copy(b);mgr.transform(c,t);cc=mgr.copy(bbb);mgr.transform(cc,t2)
                v=e['overlap'](c,cc)
                if v>1:row['hits'].append({'arm':n,'other_arm':nn,'mm3':v})
        report['samples'].append(row)
        if row['hits']:report['hits'].extend(row['hits'])
        save('motion_check',report);adsk.doEvents()
    report['status']='clear' if not report['hits'] else 'needs_review';save('motion_check',report);save('motion_frames',frames)
    if report['hits']:return
    # Create a neutral recording pose; the actual moving joints remain in the saved home pose.
    if app.activeDocument.dataFile:app.activeDocument.save('LDS: under-deck slotted brackets, 40 mm travel, 15 mm scanner clearance')
    save('motion_ready',{'ready':True,'frames':len(frames),'nominal_seconds':len(frames)/12})

def arm_recording(app,design,h,e,play=False):
    old=getattr(builtins,'_waffle_recording_state',None)
    if old:
        old['finished']=True
        try:old['event'].remove(old['handler'])
        except:pass
    root=design.rootComponent
    with open(os.path.join(OUT,'motion_check.json'),encoding='utf-8') as f:check=json.load(f)
    if check['status']!='clear':raise RuntimeError('Motion validation has unresolved collisions')
    with open(os.path.join(OUT,'motion_frames.json'),encoding='utf-8') as f:frames=json.load(f)
    guide=next(o for o in root.occurrences if o.component.name=='REFERENCE_Block_30x30x60mm')
    joints=[next(j for j in root.asBuiltJoints if j.name=='RMX52_'+n) for n in ['Base_Yaw','Shoulder','Elbow','Wrist_Pitch','Wrist_Roll_Reversed']]
    seen=set();original_opacity=[]
    for o in root.occurrences:
        if o.name not in ['housing:2','magazine_G:1']:continue
        for proxy in h['all_bodies'](o):
            b=proxy.nativeObject or proxy
            if b.entityToken in seen:continue
            seen.add(b.entityToken);original_opacity.append((b,1.0));b.opacity=.35
    guide.isLightBulbOn=True;guide.isGroundToParent=False
    def frame(row):
        for j,q in zip(joints,row['q']):j.jointMotion.rotationValue=q
        guide.transform2=h['matrix'](translation=tuple(row['block']));app.activeViewport.refresh()
    frame(frames[0]);adsk.doEvents()
    cam=app.activeViewport.camera;cam.eye=adsk.core.Point3D.create(65,-90,48);cam.target=adsk.core.Point3D.create(0,0,27);cam.upVector=adsk.core.Vector3D.create(0,0,1);cam.isFitView=True;cam.isSmoothTransition=False;app.activeViewport.camera=cam;app.activeViewport.refresh();adsk.doEvents()
    app.activeViewport.saveAsImageFile(os.path.join(OUT,'Recording_Start.png'),1600,1100)
    if play:
        start=time.monotonic();save('recording_started',{'local_time':time.strftime('%Y-%m-%d %H:%M:%S'),'epoch':time.time(),'frames':len(frames)})
        for i,row in enumerate(frames):
            frame(row);adsk.doEvents()
            save('recording_progress',{'frame':i+1,'total':len(frames),'stage':row['label'],'elapsed_seconds':time.monotonic()-start})
            time.sleep(1/12)
        guide.isLightBulbOn=False
        for j in joints:j.jointMotion.rotationValue=0
        for o,v in original_opacity:o.opacity=v
        adsk.doEvents();app.activeViewport.refresh()
        save('recording_done',{'frames':len(frames),'elapsed_seconds':time.monotonic()-start,'end_epoch':time.time()})
        return
    trigger=os.path.join(OUT,'recording_go.flag');token=str(time.time_ns());event_id='WaffleMotion_'+token
    state={'index':0,'finished':False,'event':None,'handler':None,'start':None}
    class Handler(adsk.core.CustomEventHandler):
        def __init__(self):super().__init__()
        def notify(self,args):
            try:
                if state['start'] is None:state['start']=time.monotonic()
                if state['index']<len(frames):
                    frame(frames[state['index']]);state['index']+=1
                    save('recording_progress',{'frame':state['index'],'total':len(frames),'stage':frames[state['index']-1]['label'],'elapsed_seconds':time.monotonic()-state['start']})
                else:
                    guide.isLightBulbOn=False
                    for j in joints:j.jointMotion.rotationValue=0
                    for o,v in original_opacity:o.opacity=v
                    app.activeViewport.refresh();state['finished']=True
                    save('recording_done',{'frames':state['index'],'elapsed_seconds':time.monotonic()-state['start']})
            except:
                state['finished']=True;save('recording_error',{'error':traceback.format_exc()})
    event=app.registerCustomEvent(event_id);handler=Handler();event.add(handler);state['event']=event;state['handler']=handler
    builtins._waffle_recording_state=state
    # Worker never reads or changes CAD data; it only schedules main-thread events.
    def worker():
        while not os.path.exists(trigger):time.sleep(.2)
        last=-1
        while not state['finished']:
            if state['index']!=last:
                last=state['index'];app.fireCustomEvent(event_id)
            time.sleep(1/12)
    thread=threading.Thread(target=worker,daemon=True);state['thread']=thread;thread.start()
    save('recording_armed',{'trigger_file':trigger,'frames':len(frames)})

def finalize(app,design,h,e):
    root=design.rootComponent
    destination=os.path.join(os.path.expanduser('~'),'Downloads')
    for j in root.asBuiltJoints:
        if j.name.startswith('RMX52_') and not j.isSuppressed:
            try:j.jointMotion.rotationValue=0
            except:pass
    guide=next(o for o in root.occurrences if o.component.name=='REFERENCE_Block_30x30x60mm');guide.isLightBulbOn=False
    app.userInterface.activeSelections.clear();h['capture'](design)
    exported=[]
    for o in root.occurrences:
        if o.component.name.startswith(('LDS_Front_Deck_Fixed','LDS_Rear_Deck_Fixed','LDS_Front_Sliding','LDS_Rear_Sliding')):
            path=os.path.join(destination,o.component.name+'.step');design.exportManager.execute(design.exportManager.createSTEPExportOptions(path,o.component));exported.append(path)
    cam=app.activeViewport.camera;cam.viewOrientation=adsk.core.ViewOrientations.IsoTopRightViewOrientation;cam.isFitView=True;cam.isSmoothTransition=False;app.activeViewport.camera=cam;app.activeViewport.refresh();adsk.doEvents()
    app.activeViewport.saveAsImageFile(os.path.join(OUT,'Final_Adjustable_LDS_Assembly.png'),1800,1300)
    saved=app.activeDocument.save('Adjustable front/rear LDS brackets and checked puck insertion/retrieval motion') if app.activeDocument.dataFile else False
    archive=os.path.join(destination,'Waffle_Adjustable_LDS_Final.f3d');design.exportManager.execute(design.exportManager.createFusionArchiveExportOptions(archive))
    save('final_saved',{'document_saved':saved,'archive':archive,'bracket_step_files':exported})
