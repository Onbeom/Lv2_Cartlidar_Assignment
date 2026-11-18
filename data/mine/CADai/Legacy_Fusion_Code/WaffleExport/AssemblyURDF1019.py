import adsk.core, adsk.fusion, os, json, sys, types, importlib, datetime, traceback

HERE=os.path.dirname(__file__)
OUT=os.path.join(os.path.dirname(HERE),'Waffle_Assembly_Build','AssemblyURDF1019')
EXPORTER=os.path.join(os.path.expanduser('~'),'Downloads','fusion2URDF-main','fusion2URDF-main')
def write(name,data):
    os.makedirs(OUT,exist_ok=True)
    with open(os.path.join(OUT,name+'.json'),'w',encoding='utf-8') as f:json.dump(data,f,ensure_ascii=False,indent=2)
def pipeline():
    if 'WaffleF2U' not in sys.modules:
        p=types.ModuleType('WaffleF2U');p.__path__=[EXPORTER];sys.modules['WaffleF2U']=p
    return {n:importlib.import_module('WaffleF2U.'+n) for n in ['utils','core.fusion_extractor','core.robot_model','core.data_types','core.package_generator','core.snapshot_report']}
def inspect(app,design):
    root=design.rootComponent
    report={'document':app.activeDocument.name,'occurrences':[],'groups':[],'joints':[],'open_designs':[]}
    for doc in app.documents:
        dd=adsk.fusion.Design.cast(doc.products.itemByProductType('DesignProductType'))
        if dd and doc.name.startswith('braket_10'):report['open_designs'].append({'name':doc.name,'root_occurrences':[{'name':o.name,'bodies':[{'name':b.name,'volume':b.volume} for b in o.component.bRepBodies]} for o in dd.rootComponent.occurrences]})
    for o in root.allOccurrences:
        report['occurrences'].append({'name':o.name,'path':o.fullPathName,'component':o.component.name,'visible':o.isVisible,'bulb':o.isLightBulbOn,'body_count':o.component.bRepBodies.count,'grounded':o.isGrounded,'transform':o.transform2.asArray(),'bounds':[[o.boundingBox.minPoint.x,o.boundingBox.minPoint.y,o.boundingBox.minPoint.z],[o.boundingBox.maxPoint.x,o.boundingBox.maxPoint.y,o.boundingBox.maxPoint.z]]})
    for g in root.rigidGroups:
        report['groups'].append({'name':g.name,'suppressed':g.isSuppressed,'members':[o.fullPathName for o in g.occurrences]})
    for j in root.asBuiltJoints:
        report['joints'].append({'name':j.name,'suppressed':j.isSuppressed,'one':j.occurrenceOne.fullPathName,'two':j.occurrenceTwo.fullPathName,'motion':j.jointMotion.objectType})
    write('inspect',report)
    p=pipeline();log=p['utils'].Logger(timestamps=True)
    snapshot=p['core.fusion_extractor'].extract(design,log)
    p['utils'].snapshot_to_file(snapshot,os.path.join(OUT,'snapshot.json'))
    model=p['core.robot_model'].build_model(snapshot,log)
    write('model_initial',{'errors':model.errors,'warnings':model.warnings})
    log.save_markdown(os.path.join(OUT,'initial_log.md'))
def run(app,design,request,h):
    if request['action']=='inspect':inspect(app,design)
    elif request['action']=='apply_brackets':apply_brackets(app,design,h)
    elif request['action']=='prepare_export':prepare_export(app,design,h)
    elif request['action']=='export_package':export_package(app,design)
    elif request['action']=='show_interference':show_interference(app,design,h)
    elif request['action']=='search_offsets':search_offsets(app,design,h)
    elif request['action']=='move_sockets':move_sockets(app,design,h)
    elif request['action']=='compare_offsets':compare_offsets(app,design,h)
    elif request['action']=='sync_bracket_file':sync_bracket_file(app,h)
    elif request['action']=='finalize_source':finalize_source(app)
    elif request['action'] in ['audit_live','export_audit_parts']:
        spec=importlib.util.spec_from_file_location('LiveAssemblyAudit',os.path.join(HERE,'LiveAssemblyAudit.py'))
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        if request['action']=='export_audit_parts':module.exportparts(app,design)
        else:module.run(app,design,h)

def visible_geometry(root,h):
    mgr=adsk.fusion.TemporaryBRepManager.get();result=[]
    def walk(comp,t,path):
        for b in comp.bRepBodies:
            if not b.isLightBulbOn:continue
            bb=mgr.copy(b);mgr.transform(bb,t)
            result.append((path,b,bb))
        for o in comp.occurrences:
            if not o.isLightBulbOn:continue
            walk(o.component,h['multiply'](t,o.transform2),path+'+'+o.name)
    for o in root.occurrences:
        if o.isVisible:walk(o.component,o.transform2,o.fullPathName)
    return result

def sync_bracket_file(app,h):
    source_doc=next(d for d in app.documents if d.name=='assembly')
    sd=adsk.fusion.Design.cast(source_doc.products.itemByProductType('DesignProductType'))
    target_doc=next(d for d in app.documents if d.name=='braket_1019')
    target_doc.activate()
    td=adsk.fusion.Design.cast(target_doc.products.itemByProductType('DesignProductType'))
    mgr=adsk.fusion.TemporaryBRepManager.get();report={'changes':[]}
    backup=os.path.join(OUT,'braket_1019_before_outward9mm.f3d')
    td.exportManager.execute(td.exportManager.createFusionArchiveExportOptions(backup))
    for side in ['Left','Right']:
        so=next(o for o in sd.rootComponent.occurrences if o.component.name=='Chassis_Bracket_'+side+'_16mm')
        to=next(o for o in td.rootComponent.occurrences if side in o.component.name)
        b=next(b for b in so.component.bRepBodies if b.isLightBulbOn)
        temp=mgr.copy(b)
        f=to.component.features.baseFeatures.add();f.name='Sockets_outward_9mm';f.startEdit()
        old=list(to.component.bRepBodies)
        new=to.component.bRepBodies.add(temp,f);f.finishEdit()
        new.name='FloorBracket_'+side+'_Final_16mm_4mm_Outward9mm'
        new.appearance=b.appearance
        for prior in old:prior.isLightBulbOn=False
        report['changes'].append({'side':side,'volume_mm3':new.volume*1000,'source_volume_mm3':b.volume*1000,'solid':new.isSolid})
    # Export the revised long brackets and preserve other visible source brackets.
    clean=app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    clean.name='braket_261019';cd=adsk.fusion.Design.cast(app.activeProduct)
    for to in td.rootComponent.occurrences:
        if not any(side in to.component.name for side in ['Left','Right']):continue
        co=cd.rootComponent.occurrences.addNewComponent(to.transform2);co.component.name=to.component.name
        bf=co.component.features.baseFeatures.add();bf.startEdit()
        source=next(b for b in to.component.bRepBodies if b.isLightBulbOn)
        body=co.component.bRepBodies.add(mgr.copy(source),bf);bf.finishEdit();body.name=source.name;body.appearance=source.appearance
    archive=os.path.join(OUT,'braket_261019.f3d');step=os.path.join(OUT,'braket_261019.step')
    stl=os.path.join(OUT,'floor_braket.stl')
    so=cd.exportManager.createSTLExportOptions(cd.rootComponent,stl)
    so.unitType=adsk.fusion.DistanceUnits.MillimeterDistanceUnits
    so.meshRefinement=adsk.fusion.MeshRefinementSettings.MeshRefinementHigh;so.isBinaryFormat=True
    assert cd.exportManager.execute(so)
    report['preserved_brackets']=[]
    for source in td.rootComponent.bRepBodies:
        if not source.isLightBulbOn:continue
        co=cd.rootComponent.occurrences.addNewComponent(adsk.core.Matrix3D.create());co.component.name=source.name
        bf=co.component.features.baseFeatures.add();bf.startEdit()
        body=co.component.bRepBodies.add(mgr.copy(source),bf);bf.finishEdit();body.name=source.name;body.appearance=source.appearance
        report['preserved_brackets'].append({'name':source.name,'volume_mm3':source.volume*1000,'bounds':h['bbox'](source.boundingBox)})
    assert cd.exportManager.execute(cd.exportManager.createFusionArchiveExportOptions(archive))
    assert cd.exportManager.execute(cd.exportManager.createSTEPExportOptions(step))
    report.update({'archive':archive,'step':step,'stl':stl,'backup':backup})
    imported=app.importManager.importToNewDocument(app.importManager.createSTEPImportOptions(step))
    check=adsk.fusion.Design.cast(app.activeProduct)
    solids=list(check.rootComponent.bRepBodies)
    for o in check.rootComponent.allOccurrences:solids.extend(list(o.component.bRepBodies))
    report['reimport']={'body_count':len(solids),'closed_solid_count':sum(b.isSolid for b in solids),'total_volume_mm3':sum(b.volume*1000 for b in solids)}
    expected=sum(c['source_volume_mm3'] for c in report['changes'])+sum(c['volume_mm3'] for c in report['preserved_brackets'])
    assert len(solids)==2+len(report['preserved_brackets']) and all(b.isSolid for b in solids)
    assert abs(report['reimport']['total_volume_mm3']-expected)<.1
    imported.close(False)
    write('bracket_file_updated',report)
    source_doc.activate()

def show_interference(app,design,h):
    source_doc=app.activeDocument;mgr=adsk.fusion.TemporaryBRepManager.get()
    chosen=[]
    for path,b,temp in visible_geometry(design.rootComponent,h):
        if path.startswith(('Chassis_Bracket_Left_16mm:1','Chassis_Bracket_Right_16mm:1','Floor_3_Joined:1','Profile_2020_')):chosen.append((path,temp))
    floor=next(t for path,t in chosen if path.startswith('Floor_3_Joined'))
    doc=app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType);dd=adsk.fusion.Design.cast(app.activeProduct);root=dd.rootComponent;doc.name='Socket_Deck_Interference_Explanation'
    for path,temp in chosen:
        # Show the upper sockets and nearby upper deck without the lower chassis.
        if path.startswith('Chassis_Bracket_'):
            clip=mgr.createBox(adsk.core.OrientedBoundingBox3D.create(adsk.core.Point3D.create(-12.8,0,11.0),adsk.core.Vector3D.create(1,0,0),adsk.core.Vector3D.create(0,1,0),6,32,5))
            temp=mgr.copy(temp);mgr.booleanOperation(temp,clip,adsk.fusion.BooleanTypes.IntersectionBooleanType)
        o=root.occurrences.addNewComponent(adsk.core.Matrix3D.create());o.component.name=path.split('+')[0]
        bf=o.component.features.baseFeatures.add();bf.startEdit();body=o.component.bRepBodies.add(temp,bf);bf.finishEdit()
        h['color_body'](dd,body,'Context_'+str(len(list(root.occurrences))),(100,150,210) if path.startswith('Floor_3') else (170,170,170))
        if path.startswith('Floor_3'):body.opacity=.35
        elif path.startswith('Chassis_Bracket_'):
            body.opacity=.65
            overlap=mgr.copy(temp)
            if mgr.booleanOperation(overlap,floor,adsk.fusion.BooleanTypes.IntersectionBooleanType) and overlap.volume>0:
                io=root.occurrences.addNewComponent(adsk.core.Matrix3D.create());io.component.name='RED_SOLID_OVERLAP'
                f=io.component.features.baseFeatures.add();f.startEdit();ib=io.component.bRepBodies.add(overlap,f);f.finishEdit();h['color_body'](dd,ib,'Overlap_Red',(255,30,20))
    c=app.activeViewport.camera;c.eye=adsk.core.Point3D.create(-35,-45,29);c.target=adsk.core.Point3D.create(-12.5,0,10.9);c.upVector=adsk.core.Vector3D.create(0,0,1);c.isFitView=True;c.isSmoothTransition=False;app.activeViewport.camera=c;app.activeViewport.refresh();adsk.doEvents()
    app.activeViewport.saveAsImageFile(os.path.join(OUT,'Socket_Deck_Interference_Overview.png'),1800,1200)
    # Hide the opposite corner and zoom on the right-hand socket, in native model units (cm).
    for o in root.occurrences:
        if o.component.bRepBodies.count:
            bb=o.boundingBox
            if bb.minPoint.y>0:o.isLightBulbOn=False
    c=app.activeViewport.camera;c.eye=adsk.core.Point3D.create(-24,-28,23);c.target=adsk.core.Point3D.create(-12.8,-12.05,10.8);c.upVector=adsk.core.Vector3D.create(0,0,1);c.isFitView=False;c.viewExtents=10;c.isSmoothTransition=False;app.activeViewport.camera=c;app.activeViewport.refresh();adsk.doEvents()
    image_path=os.path.join(OUT,'Socket_Deck_Interference_Closeup.png');app.activeViewport.saveAsImageFile(image_path,1600,1100)
    archive=os.path.join(OUT,'Socket_Deck_Interference_Explanation.f3d');dd.exportManager.execute(dd.exportManager.createFusionArchiveExportOptions(archive))
    write('interference_explanation',{'image':image_path,'archive':archive})
    source_doc.activate()

def search_offsets(app,design,h):
    if app.activeDocument.name!='assembly':raise RuntimeError('Activate assembly')
    mgr=adsk.fusion.TemporaryBRepManager.get();root=design.rootComponent
    allg=visible_geometry(root,h);floor=next(t for path,b,t in allg if path.startswith('Floor_3_Joined'))
    report={'search':[],'rail_geometry':[]}
    for side,sign in [('Left',1),('Right',-1)]:
        for shift_mm in range(0,26):
            # 0.5mm clearance around the full 28.4mm square socket envelope.
            y=sign*(12.05+shift_mm/10)
            tool=mgr.createBox(adsk.core.OrientedBoundingBox3D.create(adsk.core.Point3D.create(-12.8,y,10.65),adsk.core.Vector3D.create(1,0,0),adsk.core.Vector3D.create(0,1,0),2.94,2.94,2.5))
            test=mgr.copy(floor);ok=mgr.booleanOperation(test,tool,adsk.fusion.BooleanTypes.IntersectionBooleanType)
            v=test.volume*1000 if ok else None
            report['search'].append({'side':side,'outward_mm':shift_mm,'expanded_envelope_overlap_mm3':v})
            if ok and v<.001:break
            adsk.doEvents()
    for path,b,temp in allg:
        if path.startswith(('rail_hi:1','rail_lo:1','Camera_Top_Crossbar_')):
            report['rail_geometry'].append({'path':path,'bounds':h['bbox'](temp.boundingBox),'vertices':[[v.geometry.x,v.geometry.y,v.geometry.z] for v in temp.vertices]})
    write('offset_search',report)

def compare_offsets(app,design,h):
    if app.activeDocument.name!='assembly':raise RuntimeError('Activate assembly')
    mgr=adsk.fusion.TemporaryBRepManager.get();allg=visible_geometry(design.rootComponent,h)
    floor=next(t for path,b,t in allg if path.startswith('Floor_3_Joined'))
    def box(cx,cy,width,z0,z1):
        return mgr.createBox(adsk.core.OrientedBoundingBox3D.create(adsk.core.Point3D.create(cx,cy,(z0+z1)/2),adsk.core.Vector3D.create(1,0,0),adsk.core.Vector3D.create(0,1,0),width,width,z1-z0))
    report={'side':'Left (symmetric counterpart)','tests':[],'wheel_bounds':[]}
    for path,b,temp in allg:
        if path.startswith('Wheel_'):report['wheel_bounds'].append({'path':path,'bounds':h['bbox'](temp.boundingBox)})
    for direction in ['outward','rearward']:
        for shift in range(0,21):
            x=-12.8-(shift/10 if direction=='rearward' else 0);y=12.05+(shift/10 if direction=='outward' else 0)
            ring=box(x,y,2.84,9.9,12.2)
            mgr.booleanOperation(ring,box(x,y,2.04,9.899,12.201),adsk.fusion.BooleanTypes.DifferenceBooleanType)
            test=mgr.copy(floor);mgr.booleanOperation(test,ring,adsk.fusion.BooleanTypes.IntersectionBooleanType);wall=test.volume*1000
            test=mgr.copy(floor);mgr.booleanOperation(test,box(x,y,2,1.9,46.9),adsk.fusion.BooleanTypes.IntersectionBooleanType);post=test.volume*1000
            test=mgr.copy(floor);mgr.booleanOperation(test,box(x,y,2.94,9.85,12.25),adsk.fusion.BooleanTypes.IntersectionBooleanType);expanded=test.volume*1000
            report['tests'].append({'direction':direction,'shift_mm':shift,'actual_box_wall_overlap_mm3':wall,'actual_profile_overlap_mm3':post,'half_mm_expanded_envelope_overlap_mm3':expanded})
            write('offset_comparison',report);adsk.doEvents()
            if expanded<.001:break

def move_sockets(app,design,h):
    if app.activeDocument.name!='assembly':raise RuntimeError('Activate assembly')
    root=design.rootComponent;mgr=adsk.fusion.TemporaryBRepManager.get()
    if os.path.exists(os.path.join(OUT,'sockets_moved.json')):raise RuntimeError('Sockets already moved')
    report={'stage':'preparing','outward_mm':9,'changes':[],'interferences':[]}
    def save(stage):report['stage']=stage;write('sockets_move_progress',report);adsk.doEvents()
    def box(x0,y0,z0,x1,y1,z1):
        return mgr.createBox(adsk.core.OrientedBoundingBox3D.create(adsk.core.Point3D.create((x0+x1)/2,(y0+y1)/2,(z0+z1)/2),adsk.core.Vector3D.create(1,0,0),adsk.core.Vector3D.create(0,1,0),x1-x0,y1-y0,z1-z0))
    def cy(a,b,r):return mgr.createCylinderOrCone(adsk.core.Point3D.create(*a),r,adsk.core.Point3D.create(*b),r)
    def op(a,b,t):
        if not mgr.booleanOperation(a,b,t):raise RuntimeError('Socket move boolean failed')
    union=adsk.fusion.BooleanTypes.UnionBooleanType;diff=adsk.fusion.BooleanTypes.DifferenceBooleanType;inter=adsk.fusion.BooleanTypes.IntersectionBooleanType
    def find(name):return next(o for o in root.occurrences if o.component.name==name)
    def update(comp,temp):
        old=next(b for b in comp.bRepBodies if b.isLightBulbOn)
        if comp.features.baseFeatures.count==1 and comp.features.baseFeatures.item(0).bodies.count==1:
            f=comp.features.baseFeatures.item(0);f.startEdit()
            try:
                if not f.updateBody(f.bodies.item(0),temp):raise RuntimeError('Body update failed')
            finally:f.finishEdit()
        else:
            f=comp.features.baseFeatures.add();f.name='Outward_profile_mount_revision';f.startEdit();new=comp.bRepBodies.add(temp,f);f.finishEdit()
            new.appearance=old.appearance
            try:new.material=old.material
            except:pass
            old.isLightBulbOn=False
    prepared=[]
    for side,sgn,oldcx,hx in [('Left',1,-1.2500007,-1.14),('Right',-1,-9.1500006,-9.26)]:
        save('rebuilding '+side+' bracket')
        target=find('Chassis_Bracket_'+side+'_16mm');native=mgr.copy(find('Chassis_Bracket_'+side).component.bRepBodies.item(0))
        # Restore the unchanged chassis, then build the sockets in their new positions.
        for z in [0,8.7]:
            for mid in [4.8,12.8,20.8]:
                for off in [-.4,.4]:op(native,cy((hx,mid+off,z),(hx,mid+off,z+.3),.104),union)
                for off in [-.8,.8]:op(native,cy((hx,mid+off,z-.002),(hx,mid+off,z+.302),.1025),diff)
        # Close the unused previous bottom attachment bore in the chassis flange.
        op(native,cy((oldcx,26.8500004,0),(oldcx,26.8500004,.3),.6),union)
        cx=oldcx+sgn*.9;y=26.8500004
        op(native,box(cx-1.42,y-1.42,0,cx+1.42,y+1.42,.7),union)
        for lo,hi in [(.3,2.3),(8.7,11)]:
            ring=box(cx-1.42,y-1.42,lo,cx+1.42,y+1.42,hi)
            op(ring,box(cx-1.02,y-1.02,lo-.001,cx+1.02,y+1.02,hi+.001),diff)
            op(native,ring,union)
        # Open the new upper pass-through in the bracket flange itself.
        op(native,box(cx-1.02,y-1.02,8.699,cx+1.02,y+1.02,11.002),diff)
        for z in [1.5,10.2]:
            op(native,cy((cx+sgn*.99,y,z),(cx+sgn*1.44,y,z),.275),diff)
            op(native,cy((cx,y+.99,z),(cx,y+1.44,z),.275),diff)
        op(native,cy((cx,y,-.002),(cx,y,.702),.275),diff)
        if not native.isSolid or native.lumps.count!=1:raise RuntimeError('Moved bracket must be a connected solid')
        test=mgr.copy(native);op(test,box(cx-1,y-1,.7,cx+1,y+1,45.7),inter)
        if test.volume*1000>.001:raise RuntimeError('Profile passage blocked')
        prepared.append((target.component,native))
        report['changes'].append({'part':target.name,'socket_center_native_mm':[cx*10,y*10],'profile_clearance_overlap_mm3':test.volume*1000})
    # Move profile geometry without changing occurrences used by rigid groups.
    for side,sgn in [('Left',1),('Right',-1)]:
        o=find('Profile_2020_450mm_Rear_'+side+'_Envelope');b=mgr.copy(o.component.bRepBodies.item(0));mgr.transform(b,h['matrix'](translation=(0,sgn*.9,0)));prepared.append((o.component,b))
    bar=find('Camera_Top_Crossbar_2020_261mm');world=h['all_bodies'](bar)[0].boundingBox
    newbar=box(world.minPoint.x,world.minPoint.y-.9,world.minPoint.z,world.maxPoint.x,world.maxPoint.y+.9,world.maxPoint.z)
    inv=bar.transform2.copy();inv.invert();mgr.transform(newbar,inv);prepared.append((bar.component,newbar))
    report['changes'].append({'part':bar.name,'length_mm':279})
    # Extend only the two end zones; keep all storage attachment geometry in the middle unchanged.
    for label in ['rail_hi','rail_lo']:
        o=find(label);leaf=next(c for c in o.component.occurrences if c.component.bRepBodies.count)
        transform=h['multiply'](o.transform2,leaf.transform2)
        original=mgr.copy(leaf.component.bRepBodies.item(0));mgr.transform(original,transform)
        bb=original.boundingBox
        center=mgr.copy(original);op(center,box(-100,-11,-100,100,11,100),inter)
        for sgn in [-1,1]:
            end=mgr.copy(original)
            op(end,box(-100,11 if sgn>0 else -100,-100,100,100 if sgn>0 else -11,100),inter)
            mgr.transform(end,h['matrix'](translation=(0,sgn*.9,0)))
            section=mgr.copy(original);op(section,box(-100,sgn*11-.001,-100,100,sgn*11+.001,100),inter)
            sb=section.boundingBox
            bridge=box(sb.minPoint.x,min(sgn*11,sgn*11.9)-.002,sb.minPoint.z,sb.maxPoint.x,max(sgn*11,sgn*11.9)+.002,sb.maxPoint.z)
            op(center,bridge,union);op(center,end,union)
        if not center.isSolid or center.lumps.count!=1:raise RuntimeError('Extended rail disconnected')
        inv=transform.copy();inv.invert();mgr.transform(center,inv);prepared.append((leaf.component,center))
        report['changes'].append({'part':label,'end_extension_each_mm':9})
    for comp,temp in prepared:update(comp,temp);save('updated '+comp.name)
    design.computeAll();adsk.doEvents()
    allg=visible_geometry(root,h)
    relevant=[g for g in allg if g[0].startswith(('Chassis_Bracket_Left_16mm:1','Chassis_Bracket_Right_16mm:1','rail_hi:1','rail_lo:1'))]
    for path,b,a in relevant:
        for otherpath,otherb,other in allg:
            if otherpath==path or not other.isSolid:continue
            aa=a.boundingBox;bb=other.boundingBox
            if any(getattr(aa.maxPoint,k)<=getattr(bb.minPoint,k)+1e-5 or getattr(bb.maxPoint,k)<=getattr(aa.minPoint,k)+1e-5 for k in ['x','y','z']):continue
            test=mgr.copy(a)
            if mgr.booleanOperation(test,other,inter) and test.volume*1000>.1:report['interferences'].append({'part':path,'other':otherpath,'volume_mm3':test.volume*1000})
        save('checking '+path)
    # Save standalone replacement brackets as well as the complete revised assembly.
    for side in ['Left','Right']:
        o=find('Chassis_Bracket_'+side+'_16mm')
        step=os.path.join(OUT,'floor_bracket_'+side.lower()+'_outward9mm.step');design.exportManager.execute(design.exportManager.createSTEPExportOptions(step,o.component))
        stl=design.exportManager.createSTLExportOptions(o.component,os.path.join(OUT,'floor_bracket_'+side.lower()+'_outward9mm.stl'));stl.unitType=adsk.fusion.DistanceUnits.MillimeterDistanceUnits;stl.meshRefinement=adsk.fusion.MeshRefinementSettings.MeshRefinementHigh;design.exportManager.execute(stl)
    archive=os.path.join(OUT,'Assembly_Brackets_Outward9mm.f3d');design.exportManager.execute(design.exportManager.createFusionArchiveExportOptions(archive))
    c=app.activeViewport.camera;c.viewOrientation=adsk.core.ViewOrientations.IsoTopRightViewOrientation;c.isFitView=True;c.isSmoothTransition=False;app.activeViewport.camera=c;app.activeViewport.refresh();adsk.doEvents();app.activeViewport.saveAsImageFile(os.path.join(OUT,'Assembly_Outward9mm.png'),1800,1300)
    report['archive']=archive;report['stage']='complete';write('sockets_moved',report)

def apply_brackets(app,design,h):
    if app.activeDocument.name!='assembly':raise RuntimeError('Activate assembly')
    os.makedirs(OUT,exist_ok=True)
    if os.path.exists(os.path.join(OUT,'brackets_applied.json')):raise RuntimeError('Brackets already applied; inspect before repeating')
    backup=os.path.join(OUT,'Assembly_Before_Bracket_URDF_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S')+'.f3d')
    if not design.exportManager.execute(design.exportManager.createFusionArchiveExportOptions(backup)):raise RuntimeError('Assembly backup failed')
    src=next((adsk.fusion.Design.cast(d.products.itemByProductType('DesignProductType')) for d in app.documents if d.name.startswith('braket_10') and any(o.component.name.startswith('FloorBracket_') for o in adsk.fusion.Design.cast(d.products.itemByProductType('DesignProductType')).rootComponent.occurrences)),None)
    if not src:raise RuntimeError('Open the revised bracket design')
    root=design.rootComponent;mgr=adsk.fusion.TemporaryBRepManager.get()
    report={'backup':backup,'parts':[],'interferences':[]}
    for side in ['Left','Right']:
        source=next(o for o in src.rootComponent.occurrences if o.component.name.startswith('FloorBracket_'+side+'_'))
        target=next(o for o in root.occurrences if o.component.name=='Chassis_Bracket_'+side+'_16mm')
        temp=mgr.copy(source.component.bRepBodies.item(0))
        bf=target.component.features.baseFeatures.item(0);bf.startEdit()
        try:
            if not bf.updateBody(bf.bodies.item(0),temp):raise RuntimeError('Bracket body replacement failed')
        finally:bf.finishEdit()
        target.isLightBulbOn=True
        next(o for o in root.occurrences if o.component.name=='Profile_Socket_Rear_'+side).isLightBulbOn=False
        target.component.bRepBodies.item(0).name='Bracket_16mm_2020_20p4mm_Clearance_4mm_Wall'
        report['parts'].append({'side':side,'occurrence':target.name,'source_volume_mm3':temp.volume*1000,'transform':target.transform2.asArray()})
    # Retire the superseded wrist joint while preserving the working reversed wrist.
    for j in root.asBuiltJoints:
        if j.name=='RMX52_Wrist_Roll':j.isSuppressed=True
    for o in root.occurrences:
        if o.component.name=='REFERENCE_Block_30x30x60mm':o.isLightBulbOn=False
    design.computeAll();adsk.doEvents()
    geoms=visible_geometry(root,h)
    for side in ['Left','Right']:
        target=next(t for path,b,t in geoms if path=='Chassis_Bracket_'+side+'_16mm:1')
        for path,b,other in geoms:
            if not other.isSolid or path=='Chassis_Bracket_'+side+'_16mm:1':continue
            aa=target.boundingBox;bb=other.boundingBox
            if any(getattr(aa.maxPoint,k)<=getattr(bb.minPoint,k)+1e-5 or getattr(bb.maxPoint,k)<=getattr(aa.minPoint,k)+1e-5 for k in ['x','y','z']):continue
            test=mgr.copy(target)
            if mgr.booleanOperation(test,other,adsk.fusion.BooleanTypes.IntersectionBooleanType):
                v=test.volume*1000
                if v>.1:report['interferences'].append({'side':side,'other':path,'volume_mm3':v,'bounds':h['bbox'](test.boundingBox)})
        write('brackets_progress',report);adsk.doEvents()
    c=app.activeViewport.camera;c.viewOrientation=adsk.core.ViewOrientations.IsoTopRightViewOrientation;c.isFitView=True;c.isSmoothTransition=False;app.activeViewport.camera=c;app.activeViewport.refresh();adsk.doEvents()
    app.activeViewport.saveAsImageFile(os.path.join(OUT,'Assembly_Revised_Brackets.png'),1800,1300)
    archive=os.path.join(OUT,'Assembly_Revised_Brackets.f3d')
    if not design.exportManager.execute(design.exportManager.createFusionArchiveExportOptions(archive)):raise RuntimeError('Updated assembly archive failed')
    report['archive']=archive;write('brackets_applied',report)

def prepare_export(app,design,h):
    if app.activeDocument.name!='assembly':raise RuntimeError('Activate source assembly')
    source_doc=app.activeDocument;root=design.rootComponent;mgr=adsk.fusion.TemporaryBRepManager.get()
    oldj={j.name:j for j in root.asBuiltJoints}
    assignments={};link_names=['base_link']
    # Union overlapping rigid groups into each physical moving link, rather than letting
    # separate overlapping Fusion groups overwrite exporter membership redirects.
    for i in range(1,6):
        label='arm_link_'+str(i);link_names.append(label)
        names=['RMX52_Link_'+str(i)]
        if i==4:names.append('RMX52_Wrist_Case_Mount')
        if i==5:names.append('RMX52_Gripper_Horn_Adapter')
        for g in root.rigidGroups:
            if not g.isSuppressed and g.name in names:
                for o in g.occurrences:assignments[o.fullPathName.split('+')[0]]=label
    for loc in ['Front_Left','Front_Right','Rear_Left','Rear_Right']:
        label='wheel_'+loc.lower();link_names.append(label);assignments['Wheel_'+loc+':1']=label
    for loc in ['Front','Rear']:
        label='lidar_'+loc.lower();link_names.append(label)
        assignments['LDS_'+loc+'_Underside:1']=label;assignments['LDS_'+loc+'_Sliding_Carrier:1']=label
    geometry=visible_geometry(root,h)
    geometry=[g for g in geometry if not g[0].startswith('REFERENCE_Block_')]
    write('export_preparation',{'stage':'copy geometry','bodies':len(geometry),'links':link_names})
    doc=app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    dd=adsk.fusion.Design.cast(app.activeProduct);nr=dd.rootComponent;doc.name='waffle_4wd'
    links={};features={};counts={n:0 for n in link_names};markers={}
    for name in link_names:
        o=nr.occurrences.addNewComponent(adsk.core.Matrix3D.create());o.component.name=name;o.isGroundToParent=False
        links[name]=o
    mapping=[]
    geometry.sort(key=lambda g: assignments.get(g[0].split('+')[0],'base_link'))
    active_bf=None
    for index,(path,source,temp) in enumerate(geometry):
        name=assignments.get(path.split('+')[0],'base_link');o=links[name]
        bf=features.get(name)
        if not bf:
            if active_bf:active_bf.finishEdit()
            bf=o.component.features.baseFeatures.add();bf.name='Source_assembly_geometry';bf.startEdit();features[name]=bf;active_bf=bf
        new=o.component.bRepBodies.add(temp,bf);new.name=source.name
        try:new.appearance=source.appearance
        except:pass
        try:new.material=source.material
        except:pass
        counts[name]+=1;mapping.append({'source':path,'body':source.name,'link':name})
        if name not in markers:
            bb=temp.boundingBox;markers[name]=[bb.maxPoint.x,bb.maxPoint.y,bb.maxPoint.z]
        if index%20==0:write('export_preparation',{'stage':'copy geometry','copied':index+1,'total':len(geometry)});adsk.doEvents()
    if active_bf:active_bf.finishEdit()
    links['base_link'].isGrounded=True
    sk=nr.sketches.add(nr.xYConstructionPlane);sk.name='URDF_Joint_Axes'
    # Source Fusion joints store occurrenceOne=physical parent. Exporter expects
    # occurrenceTwo=parent. Recreate that ordering explicitly in this derived design.
    defs=[]
    for loc in ['Front_Left','Front_Right','Rear_Left','Rear_Right']:
        defs.append(('Wheel_Revolute_'+loc,'base_link','wheel_'+loc.lower()))
    for i,n in enumerate(['Base_Yaw','Shoulder','Elbow','Wrist_Pitch','Wrist_Roll_Reversed'],1):
        defs.append(('RMX52_'+n,'base_link' if i==1 else 'arm_link_'+str(i-1),'arm_link_'+str(i)))
    with open(os.path.join(OUT,'snapshot.json'),encoding='utf-8') as f:snap=json.load(f)
    js={j['name']:j for j in snap['joints'].values()};newj={};axis_lines={}
    for name,parent,child in defs:
        data=js[name];p=[v*100 for v in data['origin_global_m']];axis=data['axis_vector']
        sp=sk.sketchPoints.add(adsk.core.Point3D.create(*p))
        lines=[]
        for sign in [-1,1]:
            lines.append(sk.sketchCurves.sketchLines.addByTwoPoints(adsk.core.Point3D.create(*p),adsk.core.Point3D.create(*[v+sign*a for v,a in zip(p,axis)])))
        ji=nr.asBuiltJoints.createInput(links[child],links[parent],adsk.fusion.JointGeometry.createByPoint(sp))
        ji.setAsRevoluteJointMotion(adsk.fusion.JointDirections.CustomJointDirection,lines[0])
        joint=nr.asBuiltJoints.add(ji);joint.name=name;joint.isLightBulbOn=False
        newj[name]=joint;axis_lines[name]=lines
    for loc in ['front','rear']:
        ji=nr.asBuiltJoints.createInput(links['lidar_'+loc],links['base_link'],None);ji.setAsRigidJointMotion()
        j=nr.asBuiltJoints.add(ji);j.name='lidar_'+loc+'_fixed';j.isLightBulbOn=False
    sk.isVisible=False;dd.computeAll();adsk.doEvents()
    tests=[]
    def transformed(point,t):
        q=adsk.core.Point3D.create(*point);q.transformBy(t);return [q.x,q.y,q.z]
    for name,parent,child in defs:
        old=oldj[name];moving=old.occurrenceTwo;world=markers[child]
        inv=moving.transform2.copy();inv.invert();local=transformed(world,inv)
        q=old.jointMotion.rotationValue
        source_doc.activate()
        try:
            old.jointMotion.rotationValue=q+.01;design.computeAll();adsk.doEvents()
            source_end=transformed(local,moving.transform2)
        finally:old.jointMotion.rotationValue=q;design.computeAll();adsk.doEvents()
        doc.activate();new=newj[name]
        new.jointMotion.rotationValue=.01;dd.computeAll();adsk.doEvents();end=transformed(world,links[child].transform2)
        err=sum((a-b)**2 for a,b in zip(end,source_end))**.5*10
        new.jointMotion.rotationValue=0;dd.computeAll();adsk.doEvents()
        sign=-1
        if err>.01:
            new.setAsRevoluteJointMotion(adsk.fusion.JointDirections.CustomJointDirection,axis_lines[name][1])
            new.jointMotion.rotationValue=.01;dd.computeAll();adsk.doEvents();end=transformed(world,links[child].transform2)
            err=sum((a-b)**2 for a,b in zip(end,source_end))**.5*10
            new.jointMotion.rotationValue=0;dd.computeAll();adsk.doEvents();sign=1
        tests.append({'joint':name,'angle_rad':.01,'marker_error_mm':err,'axis_sign_relative_to_source_axis':sign})
        write('motion_checks',tests)
        if err>.01:raise RuntimeError('Derived joint motion differs from source: '+name+' '+str(err)+' mm')
    c=app.activeViewport.camera;c.viewOrientation=adsk.core.ViewOrientations.IsoTopRightViewOrientation;c.isFitView=True;c.isSmoothTransition=False;app.activeViewport.camera=c;app.activeViewport.refresh();adsk.doEvents()
    app.activeViewport.saveAsImageFile(os.path.join(OUT,'URDF_Ready_Assembly.png'),1800,1300)
    archive=os.path.join(OUT,'Waffle_URDF_Ready.f3d')
    if not dd.exportManager.execute(dd.exportManager.createFusionArchiveExportOptions(archive)):raise RuntimeError('URDF design archive failed')
    write('export_preparation',{'stage':'complete','body_counts':counts,'mapping':mapping,'joints':len(defs)+2,'archive':archive,'motion_checks':tests})
    p=pipeline();log=p['utils'].Logger(timestamps=True);snapshot=p['core.fusion_extractor'].extract(dd,log)
    p['utils'].snapshot_to_file(snapshot,os.path.join(OUT,'ready_snapshot.json'))
    model=p['core.robot_model'].build_model(snapshot,log)
    write('ready_model',{'errors':model.errors,'warnings':model.warnings,'links':len(model.links),'joints':len(model.joints),'root':model.root_link})
    log.save_markdown(os.path.join(OUT,'ready_log.md'))
    if model.errors:raise RuntimeError('URDF-ready model validation failed: '+str(model.errors))

def finalize_source(app):
    doc=next(d for d in app.documents if d.name=='assembly');doc.activate()
    design=adsk.fusion.Design.cast(app.activeProduct);root=design.rootComponent
    report={'components':[]}
    for name in ['rail_hi','rail_lo','Camera_Top_Crossbar_2020_261mm']:
        o=next(o for o in root.occurrences if o.component.name==name)
        output_name='Camera_Top_Crossbar_2020_279mm' if name.startswith('Camera_') else name+'_outward9mm'
        path=os.path.join(OUT,output_name+'.step')
        clean=app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType);clean.name=output_name
        cd=adsk.fusion.Design.cast(app.activeProduct)
        co=cd.rootComponent.occurrences.addNewComponent(adsk.core.Matrix3D.create());co.component.name=output_name
        bf=co.component.features.baseFeatures.add();bf.startEdit()
        for b in o.component.bRepBodies:
            if b.isLightBulbOn:co.component.bRepBodies.add(adsk.fusion.TemporaryBRepManager.get().copy(b),bf)
        bf.finishEdit()
        assert cd.exportManager.execute(cd.exportManager.createSTEPExportOptions(path))
        clean.close(False);doc.activate()
        report['components'].append({'component':name,'file':path})
    c=app.activeViewport.camera;c.isFitView=True;c.isSmoothTransition=False;app.activeViewport.camera=c;app.activeViewport.refresh();adsk.doEvents()
    app.activeViewport.saveAsImageFile(os.path.join(OUT,'Assembly_Final_Lateral9mm.png'),1800,1300)
    archive=os.path.join(OUT,'Assembly_Final_Lateral9mm.f3d')
    assert design.exportManager.execute(design.exportManager.createFusionArchiveExportOptions(archive))
    report['archive']=archive
    if doc.dataFile:
        report['saved']=doc.save('Bracket sockets outward 9 mm per side, rail ends extended, 16 mm motor hole spacing.')
    write('source_final',report)

def export_package(app,design):
    if not app.activeDocument.name.startswith('waffle_4wd'):
        doc=next(d for d in app.documents if d.name.startswith('waffle_4wd'))
        doc.activate();design=adsk.fusion.Design.cast(app.activeProduct)
    with open(os.path.join(OUT,'snapshot.json'),encoding='utf-8') as f:source_snapshot=json.load(f)
    source_joints={j['name']:j for j in source_snapshot['joints'].values()}
    directions=[adsk.fusion.JointDirections.XAxisJointDirection,adsk.fusion.JointDirections.YAxisJointDirection,adsk.fusion.JointDirections.ZAxisJointDirection]
    for j in design.rootComponent.asBuiltJoints:
        if j.name not in source_joints:continue
        axis=source_joints[j.name]['axis_vector'];index=max(range(3),key=lambda i:abs(axis[i]))
        j.timelineObject.rollTo(True)
        if not j.setAsRevoluteJointMotion(directions[index],j.geometry):raise RuntimeError('Axis repair failed: '+j.name)
    design.timeline.moveToEnd();design.computeAll();adsk.doEvents()
    audit=[]
    for j in design.rootComponent.asBuiltJoints:
        m=j.jointMotion;row={'name':j.name,'type':m.objectType,'suppressed':j.isSuppressed}
        for label,motion in [('raw',m),('cast',adsk.fusion.RevoluteJointMotion.cast(m))]:
            try:
                v=motion.rotationAxisVector;row[label+'_axis']=[v.x,v.y,v.z] if v else None
                row[label+'_value']=motion.rotationValue
            except Exception as e:row[label+'_error']=str(e)
        audit.append(row)
    write('ready_joint_api_audit',audit)
    for row in audit:
        if row['name'] not in source_joints:continue
        axis=source_joints[row['name']]['axis_vector']
        if abs(sum(a*b for a,b in zip(axis,row['raw_axis'])))<.999999:raise RuntimeError('Export axis differs: '+row['name'])
    archive=os.path.join(OUT,'Waffle_URDF_Ready.f3d')
    assert design.exportManager.execute(design.exportManager.createFusionArchiveExportOptions(archive))
    p=pipeline();log=p['utils'].Logger(timestamps=True)
    snapshot=p['core.fusion_extractor'].extract(design,log)
    snapshot.design_name='waffle_4wd';snapshot.design_name_clean='waffle_4wd'
    model=p['core.robot_model'].build_model(snapshot,log)
    if model.errors:raise RuntimeError(str(model.errors))
    cfg=p['core.data_types'].ExportConfig();cfg.package_name='waffle_4wd_description';cfg.output_dir=OUT
    cfg.visual_format='dae';cfg.collision_auto_method='visual_reuse';cfg.mesh_refinement='medium';cfg.include_screenshot=True
    pkg=os.path.join(OUT,cfg.package_name)
    write('export_status',{'stage':'meshes','links':len(model.links),'joints':len(model.joints)})
    p['core.fusion_extractor'].export_meshes(model,snapshot,pkg,cfg,log)
    p['core.fusion_extractor'].capture_screenshot(pkg,log=log)
    write('export_status',{'stage':'generate package'})
    p['core.package_generator'].generate_package(model,cfg,log)
    p['core.package_generator'].generate_validation_report(model,cfg,OUT)
    log.save_markdown(os.path.join(OUT,'export_log.md'))
    p['utils'].snapshot_to_file(snapshot,os.path.join(OUT,'exported_snapshot.json'))
    write('export_status',{'stage':'complete','package':pkg,'links':len(model.links),'joints':len(model.joints),'root':model.root_link,'warnings':model.warnings,'errors':model.errors})



