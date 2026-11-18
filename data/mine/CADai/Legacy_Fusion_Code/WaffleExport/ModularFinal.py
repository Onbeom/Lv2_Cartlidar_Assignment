import adsk.core,adsk.fusion,os,json,importlib.util,math

def run(app,api):
    source=api['source'](app);source_doc=app.activeDocument;out=api['OUT'];mgr=adsk.fusion.TemporaryBRepManager.get()
    candidates=[adsk.fusion.Design.cast(d.products.itemByProductType('DesignProductType'))for d in reversed(list(app.documents)) if adsk.fusion.Design.cast(d.products.itemByProductType('DesignProductType')) and adsk.fusion.Design.cast(d.products.itemByProductType('DesignProductType')).rootComponent.attributes.itemByName('WaffleModular','revision')]
    if not candidates:
        app.importManager.importToNewDocument(app.importManager.createFusionArchiveImportOptions(os.path.join(api['CAD_OUT'],'Floor_Bracket_Modular_261021.f3d')))
        design=adsk.fusion.Design.cast(app.activeProduct)
    else:design=candidates[0]
    design.parentDocument.activate();print_dir=os.path.join(api['CAD_OUT'],'Printable_Modular_261021');os.makedirs(print_dir,exist_ok=True);report={'parts':[]}
    pd=app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType);pd.name='Floor_Bracket_Print_Layout_261021';dd=adsk.fusion.Design.cast(app.activeProduct);dd.designType=adsk.fusion.DesignTypes.DirectDesignType
    for o in design.rootComponent.occurrences:
        b=o.component.bRepBodies.item(0);temp=mgr.copy(b);a=b.boundingBox.minPoint;c=b.boundingBox.maxPoint
        if '_Main' in o.name:
            if '_Right_' in o.name:arr=[0,-1,0,c.y,0,0,1,-a.z,-1,0,0,c.x,0,0,0,1]
            else:arr=[0,-1,0,c.y,0,0,-1,c.z,1,0,0,-a.x,0,0,0,1]
        else:arr=[1,0,0,-a.x,0,0,1,-a.z,0,-1,0,c.y,0,0,0,1]
        m=adsk.core.Matrix3D.create();m.setWithArray(arr);mgr.transform(temp,m)
        no=dd.rootComponent.occurrences.addNewComponent(adsk.core.Matrix3D.create());no.component.name=o.component.name;nb=no.component.bRepBodies.add(temp);nb.name=o.component.name
        material=dd.materials.itemByName(b.material.name)
        if not material:material=dd.materials.addByCopy(b.material,b.material.name)
        nb.material=material
        base=os.path.join(print_dir,o.component.name)
        opts=dd.exportManager.createSTLExportOptions(no.component,base+'.stl');opts.unitType=adsk.fusion.DistanceUnits.MillimeterDistanceUnits;opts.isBinaryFormat=True
        if not dd.exportManager.execute(opts):raise RuntimeError('STL export failed')
        if not dd.exportManager.execute(dd.exportManager.createSTEPExportOptions(base+'.step',no.component)):raise RuntimeError('Part STEP export failed')
        report['parts'].append(dict(name=o.component.name,bounds_mm=api['bb'](nb),solid=nb.isSolid,volume_mm3=nb.volume*1000,material='PLA',stl=base+'.stl',step=base+'.step'))
        if '_Main' in o.name:
            bores=[];obsolete=[]
            for face in b.faces:
                geo=face.geometry
                if isinstance(geo,adsk.core.Cylinder) and abs(geo.axis.z)>.999:
                    if abs(geo.radius-.14)<.00001:bores.append(api['xyz'](geo.origin))
                    if abs(geo.radius-.1025)<.00001:obsolete.append(api['xyz'](geo.origin))
            report['parts'][-1].update(motor_bore_diameter_mm=2.8,motor_bore_surfaces=len(bores),unique_motor_axes=len(set((round(p[0],4),round(p[1],4))for p in bores)),obsolete_2p05_diameter_surfaces=len(obsolete))
        layout=adsk.core.Matrix3D.create()
        if '_Main' in o.name:
            layout.translation=adsk.core.Vector3D.create(0,0 if '_Right_' in o.name else 9.6,0)
        else:
            layout.setToRotation(math.pi/2,adsk.core.Vector3D.create(0,0,1),adsk.core.Point3D.create(0,0,0))
            layout.translation=adsk.core.Vector3D.create(11 if '_Right_' in o.name else 22.6,19.2,0)
        no.transform2=layout
        adsk.doEvents()
        report['parts'][-1]['layout_transform_cm']=no.transform2.asArray()
        report['parts'][-1]['layout_bounds_mm']=api['bb'](no)
    boxes=[p['layout_bounds_mm']for p in report['parts']]
    low=[min(b['min'][k]for b in boxes)for k in range(3)];high=[max(b['max'][k]for b in boxes)for k in range(3)]
    report['combined_layout_bounds_mm']={'min':low,'max':high,'size':[high[k]-low[k]for k in range(3)]}
    if not dd.exportManager.execute(dd.exportManager.createFusionArchiveExportOptions(os.path.join(print_dir,'Floor_Bracket_Print_Layout_261021.f3d'))):raise RuntimeError('Print layout export failed')
    report['bolt']='M5x20 button head, washer both sides, nut; four per side, eight total';report['support_notes']='Main: broad web on bed. Carrier: rear plate on bed; local support under socket bridges and keys may be needed. Slicer and actual print not validated.'
    api['write']('modular_print_exports',report)
    source_doc.activate()
    token_rows=[]
    def tokens(c,path):
        for body in c.bRepBodies:
            if body.isLightBulbOn:token_rows.append(dict(path=path,body=body.name,entity_token=body.entityToken))
        for occ in c.occurrences:
            if occ.isLightBulbOn:tokens(occ.component,path+'+'+occ.name if path else occ.name)
    tokens(source.rootComponent,'');api['write']('modular_entity_tokens',token_rows)
    if api.get('print_only'):return
    # Discrete CAD collision sampling of the updated single-puck demo.
    spec=importlib.util.spec_from_file_location('MotionMath',os.path.join(os.path.dirname(__file__),'Oct19.py'));motion=importlib.util.module_from_spec(spec);spec.loader.exec_module(motion)
    frames=json.load(open(os.path.join(os.path.dirname(out),'Waffle_Assembly_Build','Oct19','LidarMotion','motion_frames.json'),encoding='utf8'))
    names=json.load(open(os.path.join(out,'assembly_link_name_map.json'),encoding='utf8'));links={r['new']:r['link'] for r in names}
    arm=[];fixed=[]
    for o in source.rootComponent.occurrences:
        link=links.get(o.component.name,'arm_link_5' if o.component.name.startswith('UC684_')else 'base_link')
        group=int(link.rsplit('_',1)[-1]) if link.startswith('arm_link_') else (5 if link.startswith('gripper_') else 0)
        if group:
            for path,b,orig in api['world_bodies'](o.component):
                mgr.transform(b,o.transform2)
                if b.isSolid:arm.append((o.name,group,b))
        elif o.name.startswith(('housing:','magazine_','Open CASCADE STEP translator','rail_','Floor_3_','Profile_2020_','Camera_','Chassis_Bracket_','LDS_')):
            for path,b,orig in api['world_bodies'](o.component):
                mgr.transform(b,o.transform2)
                if b.isSolid:fixed.append((o.name,b))
    lookup={j.name:j for j in source.rootComponent.asBuiltJoints};home={n:lookup['RMX52_'+n].jointMotion.rotationValue for n in ['Base_Yaw','Shoulder','Elbow','Wrist_Pitch','Wrist_Roll_Reversed']}
    if any(abs(x)>1e-6 for x in home.values()):raise RuntimeError('Motion geometry must be sampled at arm zero')
    fixed_boxes=[(n,b,api['bb'](b))for n,b in fixed]
    arm_boxes=[(n,g,b,api['bb'](b))for n,g,b in arm]
    def meet(a,b):return all(a['max'][k]>b['min'][k]+.001 and b['max'][k]>a['min'][k]+.001 for k in range(3))
    def transformed_box(box,m,dy):
        points=[]
        for x in [box['min'][0],box['max'][0]]:
            for y in [box['min'][1]+dy,box['max'][1]+dy]:
                for z in [box['min'][2],box['max'][2]]:
                    points.append([m[4*k]*x+m[4*k+1]*y+m[4*k+2]*z+m[4*k+3]*10 for k in range(3)])
        return {'min':[min(p[k]for p in points)for k in range(3)],'max':[max(p[k]for p in points)for k in range(3)]}
    indices=set(range(len(frames)))
    for i in range(1,len(frames)):
        if frames[i]['label']!=frames[i-1]['label']:indices.update([i-1,i])
    audit=dict(status='checking',revision='Modular_261021',sample_count=len(indices),total_frames=len(frames),sample_stride=1,hits=[],checked=[],excludes=['continuous swept volume','full arm self collision','internal gripper linkage','actuator load and actual limits','all twelve pucks during grasp'])
    progress=app.userInterface.createProgressDialog();progress.isCancelButtonShown=True;progress.show('수정 경로의 이산 프레임 검사','%v / %m',0,len(frames),0)
    for i in sorted(indices):
        if progress.wasCancelled:
            audit['status']='cancelled';api['write']('modular_motion_collision',audit);progress.hide();return
        f=frames[i];applied=[math.radians(round(math.degrees(q)*10)/10)for q in f['q']];ts=motion.transforms(applied,{'multiply':api['mul']})
        matrices=[m.asArray()for m in ts]
        for name,g,b,home_box in arm_boxes:
            dy=0
            if name.startswith(('gripper_left_link__','gripper_right_link__')):
                direction=1 if name.startswith('gripper_left_link__')else -1
                dy=direction*(f.get('aperture_mm',40)-19.62773936566)/2
            moving_box=transformed_box(home_box,matrices[g],dy)
            candidates=[(sn,sb)for sn,sb,static_box in fixed_boxes if meet(moving_box,static_box)]
            if not candidates:continue
            moving=mgr.copy(b)
            if dy:mgr.transform(moving,api['mat'](t=(0,dy,0)))
            mgr.transform(moving,ts[g])
            for sn,sb in candidates:
                v=api['intersects'](moving,sb)
                if v>.1:audit['hits'].append(dict(frame=i,label=f['label'],moving=name,other=sn,overlap_mm3=v))
        cube=api['box'](*[v*10-d for v,d in zip(f['block'],[15,15,30])],*[v*10+d for v,d in zip(f['block'],[15,15,30])])
        cube_box=api['bb'](cube)
        for sn,sb,static_box in fixed_boxes:
            if not meet(cube_box,static_box):continue
            v=api['intersects'](cube,sb)
            if v>.1:audit['hits'].append(dict(frame=i,label=f['label'],moving='Puck',other=sn,overlap_mm3=v))
        audit['checked'].append(i);api['write']('modular_motion_collision',audit);progress.progressValue=i+1;adsk.doEvents()
    progress.hide()
    audit['status']='sampled_clear' if not audit['hits'] else 'collision_found';api['write']('modular_motion_collision',audit)
    path=os.path.join(os.path.dirname(out),'Waffle_Assembly_Build','Oct19','LidarMotion','motion_check.json');check=json.load(open(path,encoding='utf8'));check.update(status='preview_only',cad_sample_status=audit['status'],samples=len(indices),limitation='Discrete single-puck CAD samples; motor operating limits and continuous collision not certified.');json.dump(check,open(path,'w',encoding='utf8'),indent=2)
    source_doc.activate()
