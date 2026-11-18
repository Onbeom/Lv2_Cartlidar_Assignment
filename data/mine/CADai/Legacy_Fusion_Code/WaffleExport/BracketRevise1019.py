import adsk.core, adsk.fusion, os, json, datetime, traceback

OUT=os.path.join(os.path.dirname(os.path.dirname(__file__)),'Waffle_Assembly_Build','BracketRevised1019')

def run(app,design):
    if app.activeDocument.name!='braket_1016':raise RuntimeError('Activate braket_1016 before editing.')
    root=design.rootComponent
    if any(o.component.name.startswith('FloorBracket_') for o in root.occurrences):raise RuntimeError('Revised brackets already exist; do not run twice.')
    os.makedirs(OUT,exist_ok=True)
    log={'status':'working','stage':'backup','changes':[],'outputs':[]}
    def save(stage):
        log['stage']=stage
        with open(os.path.join(OUT,'result.json'),'w',encoding='utf-8') as f:json.dump(log,f,ensure_ascii=False,indent=2)
        adsk.doEvents()
    stamp=datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    backup=os.path.join(OUT,'braket_1016_before_'+stamp+'.f3d')
    if not design.exportManager.execute(design.exportManager.createFusionArchiveExportOptions(backup)):raise RuntimeError('Backup failed')
    log['backup']=backup;save('backup complete')
    mgr=adsk.fusion.TemporaryBRepManager.get()
    def box(x0,y0,z0,x1,y1,z1):
        ob=adsk.core.OrientedBoundingBox3D.create(adsk.core.Point3D.create((x0+x1)/2,(y0+y1)/2,(z0+z1)/2),adsk.core.Vector3D.create(1,0,0),adsk.core.Vector3D.create(0,1,0),x1-x0,y1-y0,z1-z0)
        return mgr.createBox(ob)
    def cyl(a,b,r):return mgr.createCylinderOrCone(adsk.core.Point3D.create(*a),r,adsk.core.Point3D.create(*b),r)
    def boolean(target,tool,kind,label):
        if not mgr.booleanOperation(target,tool,kind):raise RuntimeError('Boolean failed: '+label)
    union=adsk.fusion.BooleanTypes.UnionBooleanType
    diff=adsk.fusion.BooleanTypes.DifferenceBooleanType
    intersect=adsk.fusion.BooleanTypes.IntersectionBooleanType
    sources=[b for b in root.bRepBodies if b.boundingBox.maxPoint.z>10]
    if len(sources)!=2:raise RuntimeError('Expected two long source brackets')
    prepared=[]
    for source in sources:
        right=source.boundingBox.minPoint.x<-5
        side='Right' if right else 'Left'
        cx=-9.1500006 if right else -1.2500007
        cy=26.8500004
        hx=-9.26 if right else -1.14
        body=mgr.copy(source)
        row={'side':side,'source':source.name,'socket_center_mm':[cx*10,cy*10],'inner_mm':20.4,'wall_mm':4,'outer_mm':28.4,'holes':[]}
        log['changes'].append(row)
        for z in [0,8.7]:
            save(side+' fill old motor holes at '+str(z))
            for middle in [4.8,12.8,20.8]:
                for offset in [-.4,.4]:boolean(body,cyl((hx,middle+offset,z-.001),(hx,middle+offset,z+.301),.104),union,'old hole plug')
                for offset in [-.8,.8]:
                    boolean(body,cyl((hx,middle+offset,z-.002),(hx,middle+offset,z+.302),.1025),diff,'new motor bore')
                    row['holes'].append({'type':'motor','center_mm':[hx*10,(middle+offset)*10,(z+.15)*10],'diameter_mm':2.05})
        save(side+' replace socket walls')
        # Remove both old 24mm sockets including countersinks and the old bottom bore.
        # The rest of the chassis flange/web remains unchanged.
        for lo,hi in [(-.002,2.302),(8.699,11.002)]:
            boolean(body,box(cx-1.205,cy-1.205,lo,cx+1.205,cy+1.205,hi),diff,'remove old socket')
        # Seven-mm bottom consists of the 3mm chassis flange plus 4mm socket floor.
        floor=box(cx-1.42,cy-1.42,0,cx+1.42,cy+1.42,.7)
        boolean(body,floor,union,'new bottom')
        for lo,hi in [(.3,2.3),(8.7,11.0)]:
            ring=box(cx-1.42,cy-1.42,lo,cx+1.42,cy+1.42,hi)
            boolean(ring,box(cx-1.02,cy-1.02,lo-.001,cx+1.02,cy+1.02,hi+.001),diff,'open ring')
            boolean(body,ring,union,'attach reinforced ring')
        # Keep the original two side access directions at each level, with flat outer bearing faces.
        for z in [1.5,10.2]:
            sign=-1 if right else 1
            boolean(body,cyl((cx+sign*.99,cy,z),(cx+sign*1.44,cy,z),.275),diff,'M5 side X')
            boolean(body,cyl((cx,cy+.99,z),(cx,cy+1.44,z),.275),diff,'M5 side Y')
            row['holes'].extend([{'type':'side_X','z_mm':z*10,'diameter_mm':5.5},{'type':'side_Y','z_mm':z*10,'diameter_mm':5.5}])
        boolean(body,cyl((cx,cy,-.002),(cx,cy,.702),.275),diff,'centred M5 bottom bore')
        row['holes'].append({'type':'bottom','center_mm':[cx*10,cy*10],'diameter_mm':5.5})
        if not body.isSolid or body.lumps.count!=1:raise RuntimeError(side+' bracket is not a single connected solid')
        test=mgr.copy(body)
        boolean(test,box(cx-1,cy-1,.7,cx+1,cy+1,45.7),intersect,'nominal profile clearance')
        row['profile_overlap_mm3']=test.volume*1000
        if row['profile_overlap_mm3']>.001:raise RuntimeError('Nominal profile interferes')
        # Test the designed 0.2mm/side opening, excluding coincident boundary tolerance.
        test=mgr.copy(body)
        boolean(test,box(cx-1.0199,cy-1.0199,.7001,cx+1.0199,cy+1.0199,45.7),intersect,'clearance envelope')
        row['clearance_envelope_overlap_mm3']=test.volume*1000
        if row['clearance_envelope_overlap_mm3']>.001:raise RuntimeError('Clearance envelope interferes')
        prepared.append((source,body,side,row))
        save(side+' temporary geometry verified')
    # Commit only after BOTH temporary bodies have passed the geometric checks.
    created=[]
    for source,body,side,row in prepared:
        occ=root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        occ.component.name='FloorBracket_'+side+'_16mm_2020_Clearance_4mm'
        bf=occ.component.features.baseFeatures.add();bf.name='Reinforced_socket_and_16mm_motor_pitch';bf.startEdit()
        new=occ.component.bRepBodies.add(body,bf);bf.finishEdit();new.name=occ.component.name
        if source.appearance:new.appearance=source.appearance
        source.isLightBulbOn=False
        row['solid']=new.isSolid;row['lumps']=new.lumps.count
        actual=[]
        for face in new.faces:
            g=face.geometry
            if isinstance(g,adsk.core.Cylinder):actual.append({'origin_mm':[g.origin.x*10,g.origin.y*10,g.origin.z*10],'axis':[g.axis.x,g.axis.y,g.axis.z],'diameter_mm':g.radius*20})
        row['actual_cylinders']=actual
        created.append((occ,new,side));save(side+' committed')
    for occ,body,side in created:
        name='floor_bracket_'+side.lower()+'_16mm_2020_4mm'
        step=os.path.join(OUT,name+'.step')
        if not design.exportManager.execute(design.exportManager.createSTEPExportOptions(step,occ.component)):raise RuntimeError('STEP export failed')
        stl=os.path.join(OUT,name+'.stl');op=design.exportManager.createSTLExportOptions(occ.component,stl)
        op.unitType=adsk.fusion.DistanceUnits.MillimeterDistanceUnits;op.meshRefinement=adsk.fusion.MeshRefinementSettings.MeshRefinementHigh;op.isBinaryFormat=True
        if not design.exportManager.execute(op):raise RuntimeError('STL export failed')
        log['outputs'].extend([step,stl]);save(side+' exported')
    cam=app.activeViewport.camera;cam.viewOrientation=adsk.core.ViewOrientations.IsoTopRightViewOrientation;cam.isFitView=True;cam.isSmoothTransition=False
    app.activeViewport.camera=cam;app.activeViewport.refresh();adsk.doEvents()
    shot=os.path.join(OUT,'brackets_revised.png');app.activeViewport.saveAsImageFile(shot,1600,1100);log['screenshot']=shot
    final=os.path.join(OUT,'braket_1016_revised_16mm_2020_4mm.f3d')
    if not design.exportManager.execute(design.exportManager.createFusionArchiveExportOptions(final)):raise RuntimeError('Final archive failed')
    log['outputs'].append(final);log['status']='complete';save('complete')

def finish(app,design):
    if app.activeDocument.name!='braket_1016':raise RuntimeError('Activate braket_1016')
    mgr=adsk.fusion.TemporaryBRepManager.get()
    with open(os.path.join(OUT,'result.json'),encoding='utf-8') as f:log=json.load(f)
    log['status']='finishing'
    def save():
        with open(os.path.join(OUT,'result.json'),'w',encoding='utf-8') as f:json.dump(log,f,ensure_ascii=False,indent=2)
    def box(x0,y0,z0,x1,y1,z1):
        return mgr.createBox(adsk.core.OrientedBoundingBox3D.create(adsk.core.Point3D.create((x0+x1)/2,(y0+y1)/2,(z0+z1)/2),adsk.core.Vector3D.create(1,0,0),adsk.core.Vector3D.create(0,1,0),x1-x0,y1-y0,z1-z0))
    rows=[]
    for occ in design.rootComponent.occurrences:
        if not occ.component.name.startswith('FloorBracket_'):continue
        side='Right' if '_Right_' in occ.component.name else 'Left';hx=-9.26 if side=='Right' else -1.14
        body=mgr.copy(occ.component.bRepBodies.item(0))
        # Remove the 0.01mm boolean overlap lips, restoring the original flat flange planes.
        for z0,z1 in [(-.002,0),(.3,.302),(8.698,8.7),(9,9.002)]:
            for middle in [4.8,12.8,20.8]:
                for offset in [-.4,.4]:
                    if not mgr.booleanOperation(body,box(hx-.12,middle+offset-.12,z0,hx+.12,middle+offset+.12,z1),adsk.fusion.BooleanTypes.DifferenceBooleanType):raise RuntimeError('Flange cleanup failed')
        bf=occ.component.features.baseFeatures.item(0)
        bf.startEdit()
        try:
            if not bf.updateBody(bf.bodies.item(0),body):raise RuntimeError('Body update failed')
        finally:bf.finishEdit()
        b=occ.component.bRepBodies.item(0)
        points=[]
        for z in [.15,8.85]:
            for middle in [4.8,12.8,20.8]:
                for off in [-.4,.4]:
                    state=int(b.pointContainment(adsk.core.Point3D.create(hx,middle+off,z)))
                    points.append({'type':'old_hole_filled','y_mm':(middle+off)*10,'z_mm':z*10,'state':state})
                    if state!=0:raise RuntimeError('Old motor hole is not filled')
                for off in [-.8,.8]:
                    state=int(b.pointContainment(adsk.core.Point3D.create(hx,middle+off,z)))
                    points.append({'type':'new_hole_open','y_mm':(middle+off)*10,'z_mm':z*10,'state':state})
                    if state!=2:raise RuntimeError('New motor hole is blocked')
        cylinders=[]
        for face in b.faces:
            g=face.geometry
            if isinstance(g,adsk.core.Cylinder):cylinders.append({'origin_mm':[g.origin.x*10,g.origin.y*10,g.origin.z*10],'axis':[g.axis.x,g.axis.y,g.axis.z],'diameter_mm':g.radius*20,'z_extent_mm':(face.boundingBox.maxPoint.z-face.boundingBox.minPoint.z)*10})
        row=next(r for r in log['changes'] if r['side']==side)
        row['actual_cylinders']=cylinders;row['motor_bore_point_checks']=points;row['flanges_flattened']=True
        if not b.isSolid or b.lumps.count!=1:raise RuntimeError('Bracket solidity failed')
        name='floor_bracket_'+side.lower()+'_16mm_2020_4mm'
        if not design.exportManager.execute(design.exportManager.createSTEPExportOptions(os.path.join(OUT,name+'.step'),occ.component)):raise RuntimeError('STEP export failed')
        op=design.exportManager.createSTLExportOptions(occ.component,os.path.join(OUT,name+'.stl'))
        op.unitType=adsk.fusion.DistanceUnits.MillimeterDistanceUnits;op.meshRefinement=adsk.fusion.MeshRefinementSettings.MeshRefinementHigh;op.isBinaryFormat=True
        if not design.exportManager.execute(op):raise RuntimeError('STL export failed')
        save();adsk.doEvents()
    app.userInterface.activeSelections.clear()
    app.activeViewport.refresh();adsk.doEvents()
    app.activeViewport.saveAsImageFile(os.path.join(OUT,'brackets_revised.png'),1600,1100)
    final=os.path.join(OUT,'braket_1016_revised_16mm_2020_4mm.f3d')
    if not design.exportManager.execute(design.exportManager.createFusionArchiveExportOptions(final)):raise RuntimeError('Final archive failed')
    log['status']='complete';log['stage']='verified and exported';save()
