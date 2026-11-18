import adsk.core, adsk.fusion, os, json, math

def run(app,api):
    design=api['source'](app);doc=app.activeDocument;root=design.rootComponent;out=api['OUT'];mgr=adsk.fusion.TemporaryBRepManager.get()
    report={'status':'checking','units':{'length':'mm','mass':'g','inertia':'g*mm^2'},'fits':[],'mass_rows':[],'motor_holes':[],'hardware_clearance':[]}
    lookup={o.name:o for o in root.occurrences}
    guard_path=os.path.join(out,'modular_bracket_committed.json')
    prior=json.load(open(guard_path,encoding='utf8'))
    report['moved_component_pose_errors']=[dict(part=n,max_matrix_error=max(abs(a-b)for a,b in zip(lookup[n].transform2.asArray(),values)))for n,values in prior['new_transforms_cm'].items()]
    newparts=[];newrows=[]
    for side in ['Left','Right']:
        o=lookup['Chassis_Bracket_'+side+'_16mm:1'];c=o.component
        bf=next(f for f in c.features.baseFeatures if f.name=='Modular_Floor_Bracket_261021')
        for b in bf.sourceBodies:
            b.name='Floor_Bracket_'+side+('_Main' if b.volume*1000>100000 else '_Profile_Carrier')
        for b in c.bRepBodies:
            if not b.isLightBulbOn:continue
            b.name='Floor_Bracket_'+side+('_Main' if b.volume*1000>100000 else '_Profile_Carrier')
            temp=mgr.copy(b);mgr.transform(temp,o.transform2);newparts.append((b.name,temp))
            p=b.getPhysicalProperties(adsk.fusion.CalculationAccuracy.MediumCalculationAccuracy);ok,ix,iy,iz,ixy,iyz,ixz=p.getXYZMomentsOfInertia()
            newrows.append(dict(path=o.name,body=b.name,entity_token=b.entityToken,bounds=api['bb'](temp),volume_mm3=p.volume*1000,default_mass_g=p.mass*1000,com_component_mm=api['xyz'](p.centerOfMass),transform_cm=o.transform2.asArray(),inertia_component_origin_g_mm2=[v*100000 for v in [ix,iy,iz,ixy,ixz,iyz]],source_material=b.material.name,valid_inertia=ok))
    report['new_bracket_physics']=newrows
    def masswalk(c,path):
        for b in c.bRepBodies:
            if b.isLightBulbOn:
                p=b.getPhysicalProperties(adsk.fusion.CalculationAccuracy.MediumCalculationAccuracy)
                report['mass_rows'].append({'path':path,'body':b.name,'mass_g':p.mass*1000})
        for o in c.occurrences:
            if o.isLightBulbOn:masswalk(o.component,path+'+'+o.name if path else o.name)
    masswalk(root,'');report['native_total_mass_g']=sum(r['mass_g']for r in report['mass_rows']);report['native_body_count']=len(report['mass_rows']);api['write']('modular_bracket_validation_progress',report);adsk.doEvents()
    # Motor mount centers, actual native cylindrical surfaces after translation.
    for side in ['Left','Right']:
        bracket=lookup['Chassis_Bracket_'+side+'_16mm:1'];ix=103.30003 if side=='Left' else 22.10003
        transform=api['mul'](bracket.transform2,api['mat'](t=(-114.700031280518,282.699927706055,0)))
        for end,ys in [('Front',[-242.699946,-226.699947]),('Rear',[-82.699944,-66.699944])]:
            name='Motor_'+end+'_'+side+':1';o=lookup[name];candidates=[]
            for path,b,orig in api['world_bodies'](o.component):
                mgr.transform(b,o.transform2)
                for f in b.faces:
                    g=f.geometry
                    if isinstance(g,adsk.core.Cylinder) and abs(g.axis.z)>.99 and .07<g.radius<.22:candidates.append(api['xyz'](g.origin))
            for y in ys:
                p=adsk.core.Point3D.create(ix/10,y/10,.3);p.transformBy(transform);v=api['xyz'](p)
                error=min((math.hypot(a[0]-v[0],a[1]-v[1])for a in candidates),default=None)
                report['motor_holes'].append({'motor':name,'bracket_center_world_mm':v,'nearest_motor_axis_error_mm':error})
    # Body interference with translated motors/wheels/profiles and stationary LDS mounts.
    for o in root.occurrences:
        if not o.name.startswith(('Motor_Front_','Motor_Rear_','Wheel_Front_','Wheel_Rear_','Profile_2020_450mm_','LDS_')):continue
        for path,b,orig in api['world_bodies'](o.component):
            mgr.transform(b,o.transform2)
            for n,part in newparts:
                volume=api['intersects'](b,part)
                if volume>.05:report['fits'].append({'bracket':n,'other':o.name+'+'+path,'overlap_mm3':volume})
        api['write']('modular_bracket_validation_progress',report);adsk.doEvents()
    # Find a support surface under the established green magazine center.
    g=lookup['housing:2'];green_bodies=[]
    for path,b,orig in api['world_bodies'](g.component):mgr.transform(b,g.transform2);green_bodies.append(b)
    footprint=api['box'](-97.8,-15,150,-67.8,15,229.9);support=[]
    for b in green_bodies:
        test=mgr.copy(footprint)
        if mgr.booleanOperation(test,b,adsk.fusion.BooleanTypes.IntersectionBooleanType) and test.isSolid and test.volume>1e-9:support.append(test.boundingBox.maxPoint.z*10)
    support_z=max(support) if support else None
    report['storage_support_z_mm']=support_z
    center_z=support_z+30+.02 if support_z is not None and support_z<229 else 230
    report['puck_support_status']='CAD support footprint upper surface' if center_z!=230 else 'support unresolved; prior demonstration center used'
    point=adsk.core.Point3D.create(-8.28,0,center_z/10);inverse=g.transform2.copy();inverse.invert();point.transformBy(inverse)
    channels=[('B','housing:1','magazine_B:1'),('G','housing:2','magazine_G:1'),('R','Open CASCADE STEP translator 8.0 1:2','magazine_R:1')]
    payload=[]
    for channel,house,magazine in channels:
        ho=lookup[house];obstacles=[]
        for name in [house,magazine,'rail_hi:1','rail_lo:1']:
            oo=lookup[name]
            for path,b,orig in api['world_bodies'](oo.component):mgr.transform(b,oo.transform2);obstacles.append((name,b))
        for index in range(4):
            p=point.copy();p.z+=6*index
            cube=api['box'](p.x*10-15,p.y*10-15,p.z*10-30,p.x*10+15,p.y*10+15,p.z*10+30);mgr.transform(cube,ho.transform2)
            world=p.copy();world.transformBy(ho.transform2);hits=[]
            for name,b in obstacles:
                volume=api['intersects'](cube,b)
                if volume>.05:hits.append({'other':name,'overlap_mm3':volume})
            payload.append({'channel':channel,'index':index+1,'center_world_mm':api['xyz'](world),'mass_g':66.96,'hits':hits})
        api['write']('modular_bracket_validation_progress',report);adsk.doEvents()
    report['payload_12']=payload;report['max_payload_g']=803.52;report['fully_loaded_mass_g']=report['native_total_mass_g']+803.52
    report['additional_fasteners']='Eight M5x20 button-head screws, nuts and washers are not in the CAD mass; estimate about 50g until weighed.'
    report['status']='complete';api['write']('modular_bracket_validation',report)
    guard=os.path.join(out,'modular_bracket_committed.json');data=json.load(open(guard,encoding='utf8'));data['new_bracket_physics']=newrows;api['write']('modular_bracket_committed',data)
    # Save updated source, neutral STEP, and the current native joint manifest.
    base=os.path.join(api['CAD_OUT'],'Assembly_Modular_261021')
    if not design.exportManager.execute(design.exportManager.createFusionArchiveExportOptions(base+'.f3d')):raise RuntimeError('Assembly archive export failed')
    metadata=dict(document=doc.name,units='mm',components=[dict(name=o.name,transform_mm=[v*10 if i in [3,7,11]else v for i,v in enumerate(o.transform2.asArray())])for o in root.occurrences if o.isVisible],rigid_groups=[dict(name=gr.name,suppressed=gr.isSuppressed,members=[o.fullPathName for o in gr.occurrences])for gr in root.rigidGroups],joints=[dict(name=j.name,suppressed=j.isSuppressed,one=j.occurrenceOne.fullPathName,two=j.occurrenceTwo.fullPathName,type=j.jointMotion.objectType)for j in root.asBuiltJoints])
    api['write']('native_assembly_manifest',metadata)
    # Export an exchange document built solely from the current visible bodies;
    # hidden bracket revisions in source history must not enter the STEP.
    geometry=api['world_bodies'](root)
    for prior in list(app.documents):
        if prior.name.startswith('Assembly_Modular_261021_Exchange'):prior.close(False)
    exchange_doc=app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType);exchange_doc.name='Assembly_Modular_261021_Exchange'
    exchange=adsk.fusion.Design.cast(app.activeProduct);exchange.designType=adsk.fusion.DesignTypes.DirectDesignType
    targets={}
    for index,(path,temp,native) in enumerate(geometry):
        name=path.split('+')[0]
        if name not in targets:
            occurrence=exchange.rootComponent.occurrences.addNewComponent(adsk.core.Matrix3D.create());occurrence.component.name=name.rsplit(':',1)[0];targets[name]=occurrence.component
        body=targets[name].bRepBodies.add(temp);body.name=native.name
        material=exchange.materials.itemByName(native.material.name)
        if not material:material=exchange.materials.addByCopy(native.material,native.material.name)
        body.material=material
        if index%50==0:api['write']('modular_exchange_progress',{'copied':index+1,'total':len(geometry)});adsk.doEvents()
    if not exchange.exportManager.execute(exchange.exportManager.createSTEPExportOptions(base+'.step')):raise RuntimeError('Assembly STEP export failed')
    if not exchange.exportManager.execute(exchange.exportManager.createFusionArchiveExportOptions(base+'_Geometry_Only.f3d')):raise RuntimeError('Assembly geometry archive failed')
    api['write']('modular_exchange_verified',{'body_count':len(geometry),'components':len(targets),'STEP':base+'.step','geometry_f3d':base+'_Geometry_Only.f3d'})
    doc.activate()
