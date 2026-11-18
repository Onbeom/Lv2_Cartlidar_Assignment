import adsk.core,adsk.fusion,os,json,importlib.util,math

def run(app,api):
    design=api['source'](app);root=design.rootComponent;mgr=adsk.fusion.TemporaryBRepManager.get();out=api['OUT']
    spec=importlib.util.spec_from_file_location('MotionMath',os.path.join(os.path.dirname(__file__),'Oct19.py'));mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
    mapping=json.load(open(os.path.join(out,'assembly_link_name_map.json'),encoding='utf8'));links={r['new']:r['link']for r in mapping}
    arm=[];obstacles=[];fixed_bounds=[]
    for o in root.occurrences:
        link=links.get(o.component.name,'arm_link_5' if o.component.name.startswith('UC684_') else 'base_link')
        g=int(link[-1]) if link.startswith('arm_link_') else 5 if link.startswith('gripper_') else 0
        if g or o.name.startswith(('housing:','magazine_','Open CASCADE STEP translator','rail_','Floor_3_','Profile_2020_','Camera_','Chassis_Bracket_','LDS_')):
            for path,b,native in api['world_bodies'](o.component):
                mgr.transform(b,o.transform2)
                if not b.isSolid:continue
                if g:arm.append((o.name,g,b))
                else:obstacles.append((o.name,b))
        if not g:
            for path,b,native in api['world_bodies'](o.component):mgr.transform(b,o.transform2);fixed_bounds.append(api['bb'](b))
        api['write']('modular_fold_progress',{'collecting':o.name,'arm_bodies':len(arm)})
        adsk.doEvents()
    names=['Base_Yaw','Shoulder','Elbow','Wrist_Pitch','Wrist_Roll_Reversed'];lookup={j.name:j for j in root.asBuiltJoints};joints=[lookup['RMX52_'+n]for n in names];initial=[j.jointMotion.rotationValue for j in joints]
    if any(abs(v)>1e-6 for v in initial):raise RuntimeError('Fold measurement requires the unchanged zero posture')
    candidates=json.load(open(os.path.join(out,'modular_fold_candidates.json'),encoding='utf8'));report={'trials':[],'status':'checking','scope':'Discrete static folded pose; stock reference angle ranges only, actual hard stops not verified.'};chosen=None
    static_boxes=[(n,b,api['bb'](b))for n,b in obstacles]
    def meet(a,b):return all(a['max'][k]>b['min'][k]+.001 and b['max'][k]>a['min'][k]+.001 for k in range(3))
    for candidate in candidates[:8]:
        q=[math.radians(round(math.degrees(v)*10)/10)for v in candidate['q']];ts=mod.transforms(q,{'multiply':api['mul']});moved=[];hits=[]
        for n,g,b in arm:
            t=mgr.copy(b);mgr.transform(t,ts[g]);moved.append((n,g,t));box=api['bb'](t)
            for sn,sb,static_box in static_boxes:
                if not meet(box,static_box):continue
                v=api['intersects'](t,sb)
                if v>.1:hits.append({'part':n,'other':sn,'mm3':v})
        moving_boxes=[(n,g,b,api['bb'](b))for n,g,b in moved]
        for i,(n,g,b,box) in enumerate(moving_boxes):
            for sn,sg,sb,sbox in moving_boxes[i+1:]:
                if abs(g-sg)<2:continue
                if not meet(box,sbox):continue
                v=api['intersects'](b,sb)
                if v>1:hits.append({'part':n,'other':sn,'mm3':v})
        row={'q_rad':q,'hits':hits};report['trials'].append(row);api['write']('modular_fold_measurement',report);adsk.doEvents()
        if hits:continue
        boxes=fixed_bounds+[api['bb'](b)for n,g,b in moved];lo=[min(r['min'][k]for r in boxes)for k in range(3)];hi=[max(r['max'][k]for r in boxes)for k in range(3)];report.update(status='sampled_static_clear',chosen_q_rad=q,bounds_mm={'min':lo,'max':hi,'size':[b-a for a,b in zip(lo,hi)]});chosen=q;break
    if chosen is None:report['status']='no_clear_candidate';api['write']('modular_fold_measurement',report);return
    try:
        for j,v in zip(joints,chosen):j.jointMotion.rotationValue=v
        app.activeViewport.refresh();adsk.doEvents()
        actual=[api['bb'](body)for path,body,native in api['world_bodies'](root)]
        lo=[min(r['min'][k]for r in actual)for k in range(3)];hi=[max(r['max'][k]for r in actual)for k in range(3)]
        report['native_occurrence_bounds_mm']={'min':lo,'max':hi,'size':[b-a for a,b in zip(lo,hi)]}
        cam=app.activeViewport.camera;cam.viewOrientation=adsk.core.ViewOrientations.IsoTopRightViewOrientation;cam.isFitView=True;cam.isSmoothTransition=False;app.activeViewport.camera=cam;app.activeViewport.refresh()
        app.activeViewport.saveAsImageFile(os.path.join(out,'Assembly_Modular_261021_Folded.png'),1500,1100)
    finally:
        for j,v in zip(joints,initial):j.jointMotion.rotationValue=v
        adsk.doEvents();report['restored_original_pose']=all(abs(j.jointMotion.rotationValue-v)<1e-8 for j,v in zip(joints,initial));api['write']('modular_fold_measurement',report)
