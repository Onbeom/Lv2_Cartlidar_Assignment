import adsk.core, adsk.fusion, os, json, math

def build(app, api):
    out=api['OUT'];source=api['source'](app);source_doc=app.activeDocument
    source_mat=source.materials.itemByName('Waffle_PLA_Printed_1240_1240.0')
    doc=app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    doc.name='Floor_Bracket_Modular_261021_R2'
    design=adsk.fusion.Design.cast(app.activeProduct);root=design.rootComponent
    root.attributes.add('WaffleModular','revision','261021R2')
    try:material=design.materials.addByCopy(source_mat,'PLA_1240kg_m3')
    except:material=source_mat
    mgr=adsk.fusion.TemporaryBRepManager.get()
    box0=api['box'];boolean=api['boolean'];mirror_center=62.700023
    fits=json.load(open(os.path.join(out,'modular_bracket_all_fits.json'),encoding='utf8'))
    right_rings=fits[0]['rings'];parts=[];report={'design':doc.name,'parts':[],'assembly_shift_mm':[-12.5,0,0],'floor_hole_width_offset_mm':5,'joint_clearance_per_face_mm':.3,'joint_bolt':'M5 x 20, washers both sides, nut; four per side','source_preserved':True}
    panels=json.load(open(os.path.join(out,'modular_bracket_panel_holes.json'),encoding='utf8'))
    top_holes=list({tuple(h['center_mm'][:2]) for p in panels['parts'] if p['part']=='Floor_3_Joined:1' for h in p['holes'] if h['radius_mm']<2.1})
    for side in ['Right','Left']:
        mirror=side=='Left'
        X=lambda x:2*mirror_center-x if mirror else x
        def box(x0,y0,z0,x1,y1,z1):
            a,b=sorted([X(x0),X(x1)])
            return box0(a,y0,z0,b,y1,z1)
        def cyl(p,q,r):
            p=(X(p[0]),p[1],p[2]);q=(X(q[0]),q[1],q[2])
            return mgr.createCylinderOrCone(adsk.core.Point3D.create(*[v/10 for v in p]),r/10,adsk.core.Point3D.create(*[v/10 for v in q]),r/10)
        def join(a,b):boolean(a,b,True);return a
        def cut(a,b):boolean(a,b,False);return a
        main=box(47.700023,-249.949947,0,52.700023,-32.3,90)
        for z0,z1 in [(0,3),(87,90)]:join(main,box(10.700025,-249.949947,z0,52.700023,-32.3,z1))
        join(main,box(45.3001,-249.949947,3,47.700023,-32.3,6))
        join(main,box(44.700023,-249.949947,84,47.700023,-32.3,87))
        hole_rows=[]
        for ring in right_rings:
            x,y,z=ring['center_mm'];r=ring['radius_mm']
            if z!=0 or y>-35:continue
            if abs(r-2)<.01:x+=5;kind='panel_lower'
            elif abs(r-1.025)<.01:
                if min(abs(y-v)for v in [-242.699946,-226.699947,-162.699948,-146.699945,-82.699944,-66.699944])>.001:continue
                kind='motor';r=1.4
            else:continue
            cut(main,cyl((x,y,-1),(x,y,4 if kind=='panel_lower' else 91),r))
            hole_rows.append({'kind':kind,'center_input_mm':[X(x),y],'radius_mm':r})
            if kind=='panel_lower':
                ty=133 if side=='Left' else -29
                world=[-154.699927706055-y,ty+X(x)-114.700031280518]
                target=min(top_holes,key=lambda p:(p[0]-world[0])**2+(p[1]-world[1])**2)
                ux=X(target[1]-ty+114.700031280518);uy=-154.699927706055-target[0]
                cut(main,cyl((ux,uy,86),(ux,uy,91),r))
                hole_rows.append({'kind':'panel_upper','center_input_mm':[X(ux),uy],'radius_mm':r,'target_world_xy_mm':list(target)})
        # Rounded cable openings. The front opening leaves 10mm of end web.
        for cy in [-74.699944,-154.699944,-229.949947]:
            w=h=20.;r=4.;cz=30
            tool=box(47,cy-w/2+r,cz-h/2,54,cy+w/2-r,cz+h/2)
            join(tool,box(47,cy-w/2,cz-h/2+r,54,cy+w/2,cz+h/2-r))
            for y in [cy-w/2+r,cy+w/2-r]:
                for z in [cz-h/2+r,cz+h/2-r]:join(tool,cyl((47,y,z),(54,y,z),r))
            cut(main,tool)
        carrier=box(0,-4,0,52.700023,0,110)
        for z0,z1 in [(0,23),(87,110)]:join(carrier,box(0,-28.4,z0,28.4,0,z1))
        for z0,z1 in [(0,6),(84,90)]:join(carrier,box(24.4,-28.4,z0,52.700023,0,z1))
        cut(carrier,box(4,-24.4,7,24.4,-4,111))
        for z in [15,102]:
            cut(carrier,cyl((-1,-14.2,z),(4.01,-14.2,z),2.75))
            cut(carrier,cyl((14.2,-4.01,z),(14.2,1,z),2.75))
        cut(carrier,cyl((14.2,-14.2,-1),(14.2,-14.2,7.01),2.75))
        for z0,z1 in [(8,40),(50,82)]:join(carrier,box(41.700023,-58,z0,47.700023,-3,z1))
        for z0,z1 in [(28,36),(54,62)]:
            cut(main,box(47.6,-46,z0,52.8,-38,z1))
            join(carrier,box(47.700023,-45.7,z0+.3,52.700023,-38.3,z1-.3))
        for z in [18,72]:
            for y in [-51,-39]:
                cut(main,cyl((41,y,z),(54,y,z),2.75))
                cut(carrier,cyl((41,y,z),(49,y,z),2.75))
        for label,temp in [('Main',main),('Profile_Carrier',carrier)]:
            o=root.occurrences.addNewComponent(adsk.core.Matrix3D.create());c=o.component;c.name='Floor_Bracket_'+side+'_'+label
            bf=c.features.baseFeatures.add();bf.name='Analytic_Modular_Bracket_261021';bf.startEdit()
            try:b=c.bRepBodies.add(temp,bf);b.name=c.name;b.material=material
            finally:bf.finishEdit()
            c.material=material
            row={'name':c.name,'side':side,'role':label,'bounds_input_mm':api['bb'](b),'volume_mm3':b.volume*1000,'mass_g':b.volume*1.24,'solid':b.isSolid,'faces':b.faces.count}
            if label=='Main':row['holes']=hole_rows
            report['parts'].append(row);parts.append(o);api['write']('modular_bracket_build_progress',report);adsk.doEvents()
        # This pair is a keyed lap joint: contact is intentional, overlap is not.
        overlap=mgr.copy(main)
        if not mgr.booleanOperation(overlap,carrier,adsk.fusion.BooleanTypes.IntersectionBooleanType):raise RuntimeError('Joint overlap check failed')
        report.setdefault('joint_overlap_mm3',{})[side]=overlap.volume*1000 if overlap.isSolid else 0
    report['total_bracket_mass_g']=sum(p['mass_g']for p in report['parts'])
    base=os.path.join(api['CAD_OUT'],'Floor_Bracket_Modular_261021')
    for ext,opts in [('f3d',design.exportManager.createFusionArchiveExportOptions(base+'.f3d')),('step',design.exportManager.createSTEPExportOptions(base+'.step'))]:
        if not design.exportManager.execute(opts):raise RuntimeError('Export failed '+ext)
    cam=app.activeViewport.camera;cam.viewOrientation=adsk.core.ViewOrientations.IsoTopRightViewOrientation;cam.isFitView=True;cam.isSmoothTransition=False;app.activeViewport.camera=cam;app.activeViewport.refresh()
    app.activeViewport.saveAsImageFile(base+'.png',1500,1100)
    report['exports']=[base+'.f3d',base+'.step'];api['write']('modular_bracket_built',report)
    return report
