import adsk.core,adsk.fusion,os,json,sys,types,importlib,math

def run(app,api):
    api['write']('modular_urdf_progress',dict(stage='select_source'))
    source=api['source'](app);source_doc=app.activeDocument;H=api['OUT'];out=os.path.join(api['CAD_OUT'],'submit','urdf');os.makedirs(out,exist_ok=True)
    read=lambda n:json.load(open(os.path.join(H,n+'.json'),encoding='utf8'))
    review=read('ROBOT_REVIEW_PENDING');physics=read('PHYSICS_ESTIMATE_G_MM');mapping={(r['path'],r['body']):r['link']for r in physics['bodies']}
    rows=api['world_bodies'](source.rootComponent);grouped={l['name']:[]for l in review['links']};missing=[]
    for path,temp,native in rows:
        link=mapping.get((path,native.name))
        if link is None:missing.append([path,native.name])
        else:grouped[link].append((temp,native))
    if missing:raise RuntimeError('Source differs from reviewed model: '+str(missing[:6]))
    api['write']('modular_urdf_progress',dict(stage='cache_bounds'))
    reference_bounds={n:[(api['bb'](t)['min']+api['bb'](t)['max'],b.material.name)for t,b in bodies]for n,bodies in grouped.items()}
    def restore_materials():
        for name,refs in reference_bounds.items():
            remaining=list(refs)
            for target in occ[name].component.bRepBodies:
                bounds=api['bb'](target);coords=bounds['min']+bounds['max']
                best=min(range(len(remaining)),key=lambda i:max(abs(a-b)for a,b in zip(coords,remaining[i][0])))
                bounds,material=remaining.pop(best)
                if max(abs(a-b)for a,b in zip(coords,bounds))>.002:raise RuntimeError('Material geometry matching failed '+name)
                if target.material.name!=material:target.material=d.materials.itemByName(material)
    existing=next((x for x in app.documents if x.name.startswith('Waffle_URDF_Export_261021')),None)
    if existing:
        doc=existing;doc.activate();d=adsk.fusion.Design.cast(app.activeProduct);root=d.rootComponent
        occ={o.component.name:o for o in root.occurrences};count=sum(o.component.bRepBodies.count for o in occ.values())
        api['write']('modular_urdf_progress',dict(stage='restore_materials'))
        d.timeline.moveToEnd();restore_materials()
        mass=sum(b.getPhysicalProperties(adsk.fusion.CalculationAccuracy.MediumCalculationAccuracy).mass*1000 for p,t,b in api['world_bodies'](root))
        if count!=669 or abs(mass-review['total_mass_g'])>.001:raise RuntimeError('Resume copy geometry/mass guard: bodies='+str(count)+' mass_g='+str(mass))
        frames={l['name']:l['link_frame_world_mm']for l in review['links']}
        axes={j['name']:[sum(frames[j['child']][k][i]*j['axis_child'][i]for i in range(3))for k in range(3)]for j in review['joints']if j['current_type']!='fixed'}
    else:
        for prior in list(app.documents):
            if prior.name.startswith('Waffle_URDF_Export_261021'):prior.close(False)
        source_doc.activate()
        doc=app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType);doc.name='Waffle_URDF_Export_261021';d=adsk.fusion.Design.cast(app.activeProduct);root=d.rootComponent;occ={};count=0
        for name,bodies in grouped.items():
            o=root.occurrences.addNewComponent(adsk.core.Matrix3D.create());o.component.name=name;occ[name]=o
            bf=o.component.features.baseFeatures.add();bf.name='Reviewed_visible_geometry';bf.startEdit()
            for temp,native in bodies:
                b=o.component.bRepBodies.add(temp,bf);b.name=native.name
                material=d.materials.itemByName(native.material.name)
                if not material:material=d.materials.addByCopy(native.material,native.material.name)
                b.material=material
                try:b.appearance=native.appearance
                except:pass
                count+=1
            bf.finishEdit()
            # Apply on result bodies after finishing the base feature; source-body
            # assignments made inside the edit are not always retained by Fusion.
            targets=list(o.component.bRepBodies)
            if len(targets)!=len(bodies):raise RuntimeError('Base feature body count changed')
            for target,(temp,native)in zip(targets,bodies):target.material=d.materials.itemByName(native.material.name)
            api['write']('modular_urdf_progress',dict(stage='copy_geometry',copied=count,total=len(rows)));adsk.doEvents()
        occ['base_link'].isGrounded=True
        frames={l['name']:l['link_frame_world_mm']for l in review['links']};sk=root.sketches.add(root.xYConstructionPlane);sk.name='URDF_export_joint_geometry';sk.is3D=True
        axes={}
        for j in review['joints']:
            if j['current_type']=='fixed':inp=root.asBuiltJoints.createInput(occ[j['child']],occ[j['parent']],None);inp.setAsRigidJointMotion()
            else:
                F=frames[j['child']];p=[F[k][3]/10 for k in range(3)];a=[sum(F[k][i]*j['axis_child'][i]for i in range(3))for k in range(3)]
                a=[0 if abs(x)<1e-6 else (1 if abs(x-1)<1e-6 else -1 if abs(x+1)<1e-6 else x)for x in a]
                sp=sk.sketchPoints.add(adsk.core.Point3D.create(*p));line=sk.sketchCurves.sketchLines.addByTwoPoints(adsk.core.Point3D.create(*p),adsk.core.Point3D.create(*[p[k]+a[k]for k in range(3)]))
                inp=root.asBuiltJoints.createInput(occ[j['child']],occ[j['parent']],adsk.fusion.JointGeometry.createByPoint(sp))
                if j['current_type']=='prismatic':inp.setAsSliderJointMotion(adsk.fusion.JointDirections.CustomJointDirection,line)
                else:inp.setAsRevoluteJointMotion(adsk.fusion.JointDirections.CustomJointDirection,line)
                axes[j['name']]=a
            joint=root.asBuiltJoints.add(inp);joint.name=j['name'];joint.isLightBulbOn=False
            if j['current_type']=='prismatic':
                lim=joint.jointMotion.slideLimits;lim.minimumValue=j['travel_limits_mm'][0]/10;lim.maximumValue=j['travel_limits_mm'][1]/10;lim.isMinimumValueEnabled=True;lim.isMaximumValueEnabled=True
        sk.isVisible=False;d.computeAll();adsk.doEvents()
        restore_materials()
        mass=sum(b.getPhysicalProperties(adsk.fusion.CalculationAccuracy.MediumCalculationAccuracy).mass*1000 for p,t,b in api['world_bodies'](root))
        api['write']('modular_urdf_mass_check',dict(bodies=count,mass_g=mass,expected_g=review['total_mass_g'],links=[dict(name=n,mass_g=sum(b.getPhysicalProperties(adsk.fusion.CalculationAccuracy.MediumCalculationAccuracy).mass*1000 for b in o.component.bRepBodies))for n,o in occ.items()]))
        if count!=669 or abs(mass-review['total_mass_g'])>.001:raise RuntimeError('Derived geometry or mass differs from reviewed assembly: '+str(mass))
    exporter=os.path.join(os.path.expanduser('~'),'Downloads','fusion2URDF-main','fusion2URDF-main')
    package='WaffleF2U261021'
    if package not in sys.modules:mod=types.ModuleType(package);mod.__path__=[exporter];sys.modules[package]=mod
    modules={n:importlib.import_module(package+'.'+n)for n in ['utils','core.fusion_extractor','core.robot_model','core.data_types','core.package_generator']}
    api['write']('modular_urdf_progress',dict(stage='repair_axes'))
    sk=next(x for x in root.sketches if x.name=='URDF_export_joint_geometry')
    moving=[j for j in review['joints']if j['current_type']!='fixed']
    repair_needed=False
    for j in root.asBuiltJoints:
        if j.name not in axes:continue
        m=adsk.fusion.RevoluteJointMotion.cast(j.jointMotion)or adsk.fusion.SliderJointMotion.cast(j.jointMotion)
        v=m.rotationAxisVector if hasattr(m,'rotationAxisVector')else m.slideDirectionVector
        if v is None or max(abs(x-y)for x,y in zip(v.asArray(),axes[j.name]))>1e-5 or j.occurrenceOne.isGroundToParent:repair_needed=True
    if repair_needed:
        for index,jdef in enumerate(moving):
            j=next(x for x in root.asBuiltJoints if x.name==jdef['name']);line=sk.sketchCurves.sketchLines.item(index)
            j.timelineObject.rollTo(True)
            geometry=adsk.fusion.JointGeometry.createByCurve(line,adsk.fusion.JointKeyPointTypes.StartKeyPoint)
            direction=adsk.fusion.JointDirections.ZAxisJointDirection
            if jdef['current_type']=='prismatic':ok=j.setAsSliderJointMotion(direction,geometry)
            else:ok=j.setAsRevoluteJointMotion(direction,geometry)
            if not ok:raise RuntimeError('Principal-axis repair failed '+j.name)
        d.timeline.moveToEnd()
        for n,o in occ.items():
            if n!='base_link':o.isGroundToParent=False;o.isGrounded=False
        d.computeAll();adsk.doEvents();restore_materials()
    audit=[]
    for j in root.asBuiltJoints:
        m=j.jointMotion;row=dict(name=j.name,type=m.objectType,grounded=j.occurrenceOne.isGrounded,ground_to_parent=j.occurrenceOne.isGroundToParent,geometry_axis=list(j.geometry.primaryAxisVector.asArray())if j.geometry else None)
        for cls,prop in [(adsk.fusion.RevoluteJointMotion,'rotationAxisVector'),(adsk.fusion.SliderJointMotion,'slideDirectionVector')]:
            cast=cls.cast(m)
            if cast:
                v=getattr(cast,prop);row['cast_axis']=list(v.asArray())if v else None
        audit.append(row)
    api['write']('modular_urdf_native_axes',audit)
    api['write']('modular_urdf_progress',dict(stage='probe_motion'))
    probes=[]
    for j in root.asBuiltJoints:
        if j.name not in axes:continue
        o=j.occurrenceOne;before=o.transform2.copy();inv=before.copy();inv.invert()
        m=j.jointMotion;slider=adsk.fusion.SliderJointMotion.cast(m);rev=adsk.fusion.RevoluteJointMotion.cast(m)
        if slider:q=slider.slideValue;slider.slideValue=q+.001
        else:q=rev.rotationValue;rev.rotationValue=q+.01
        adsk.doEvents();after=o.transform2.copy();delta=api['mul'](after,inv)
        qread=slider.slideValue if slider else rev.rotationValue
        dq=qread-q
        if slider:actual=[delta.getCell(k,3)/dq for k in range(3)]
        else:actual=[(delta.getCell(2,1)-delta.getCell(1,2))/(2*math.sin(dq)),(delta.getCell(0,2)-delta.getCell(2,0))/(2*math.sin(dq)),(delta.getCell(1,0)-delta.getCell(0,1))/(2*math.sin(dq))]
        if slider:slider.slideValue=q
        else:rev.rotationValue=q
        adsk.doEvents()
        probes.append(dict(name=j.name,value_before=q,value_after_set=qread,measured_world_axis=actual,expected=axes[j.name],error=max(abs(x-y)for x,y in zip(actual,axes[j.name]))))
        api['write']('modular_urdf_native_motion_probe',probes)
    restore_materials()
    native_motion_passed=all(x['error']<1e-5 for x in probes)
    log=modules['utils'].Logger(timestamps=True);snapshot=modules['core.fusion_extractor'].extract(d,log)
    snapshot.design_name='waffle_4wd';snapshot.design_name_clean='waffle_4wd'
    modules['utils'].snapshot_to_file(snapshot,os.path.join(out,'snapshot.json'))
    model=modules['core.robot_model'].build_model(snapshot,log);log.save_markdown(os.path.join(out,'export_log.md'))
    result=dict(stage='model_validation',native_motion_passed=native_motion_passed,source_document=source_doc.name,derived_archive=os.path.join(out,'Waffle_URDF_Export_261021.f3d'),bodies=count,mass_g=mass,links=len(model.links),joints=len(model.joints),root=model.root_link,errors=model.errors,warnings=model.warnings)
    api['write']('modular_urdf_export',result)
    if model.errors or len(model.links)!=14 or len(model.joints)!=13 or model.root_link!='base_link':raise RuntimeError('Derived export validation: '+str(result))
    # Confirm the converter reads the intended physical axes and current origins.
    axis_errors=[]
    for name,expected in axes.items():
        fj=snapshot.joints[name];actual=fj.axis_vector
        if max(abs(x-y)for x,y in zip(actual,expected))>1e-5:axis_errors.append(dict(joint=name,expected=expected,actual=actual))
    result['axis_errors']=axis_errors
    if axis_errors:api['write']('modular_urdf_export',result);raise RuntimeError('Converter axis extraction requires correction: '+str(axis_errors))
    if not d.exportManager.execute(d.exportManager.createFusionArchiveExportOptions(result['derived_archive'])):raise RuntimeError('URDF-ready archive export failed')
    cfg=modules['core.data_types'].ExportConfig();cfg.output_dir=out;cfg.package_name='waffle_4wd_description_261021';cfg.visual_format='dae';cfg.collision_auto_method='visual_reuse';cfg.mesh_refinement='medium';cfg.include_screenshot=False;cfg.include_ros2_control=False
    pkg=os.path.join(out,cfg.package_name);api['write']('modular_urdf_progress',dict(stage='export_meshes',links=len(model.links)))
    modules['core.fusion_extractor'].export_meshes(model,snapshot,pkg,cfg,log)
    modules['core.package_generator'].generate_package(model,cfg,log)
    modules['core.package_generator'].generate_validation_report(model,cfg,out)
    # The two equivalent fingers share one actuator. The generic exporter does
    # not extract Fusion MotionLinks, so add the reviewed mimic relationship.
    import xml.etree.ElementTree as ET
    patched=[]
    for folder,dirs,files in os.walk(os.path.join(pkg,'urdf')):
        for name in files:
            if not name.endswith(('.urdf','.xacro')):continue
            path=os.path.join(folder,name);tree=ET.parse(path);changed=False
            for node in tree.getroot().iter('joint'):
                if node.get('name')=='gripper_right_joint':
                    for old in list(node.findall('mimic')):node.remove(old)
                    ET.SubElement(node,'mimic',joint='gripper_left_joint',multiplier='-1',offset='0');changed=True
            if changed:tree.write(path,encoding='utf-8',xml_declaration=True);patched.append(path)
    log.save_markdown(os.path.join(out,'export_log.md'));result.update(stage='complete',package=pkg,mimic_patched_files=patched,scope='CAD/visualization export candidate. Actual joint limits, convex collision, ROS runtime and hardware control not validated. Original assembly unchanged.')
    api['write']('modular_urdf_export',result)
    with open(os.path.join(out,'export_status.json'),'w',encoding='utf8')as f:json.dump(result,f,ensure_ascii=False,indent=2)
    source_doc.activate()

def reopen(app,api):
    out=os.path.join(api['CAD_OUT'],'submit','urdf');path=os.path.join(out,'Waffle_URDF_Export_261021.f3d');prior=app.activeDocument
    doc=app.importManager.importToNewDocument(app.importManager.createFusionArchiveImportOptions(path));d=adsk.fusion.Design.cast(app.activeProduct);root=d.rootComponent
    mass=sum(b.getPhysicalProperties(adsk.fusion.CalculationAccuracy.MediumCalculationAccuracy).mass*1000 for p,t,b in api['world_bodies'](root))
    result=dict(file=path,links=root.occurrences.count,joints=root.asBuiltJoints.count,bodies=sum(o.component.bRepBodies.count for o in root.occurrences),mass_g=mass,axes=[])
    for j in root.asBuiltJoints:
        motion=adsk.fusion.RevoluteJointMotion.cast(j.jointMotion)or adsk.fusion.SliderJointMotion.cast(j.jointMotion)
        if motion:
            v=motion.rotationAxisVector if hasattr(motion,'rotationAxisVector')else motion.slideDirectionVector
            result['axes'].append(dict(joint=j.name,axis=list(v.asArray()),ground_to_parent=j.occurrenceOne.isGroundToParent))
    result['passed']=result['links']==14 and result['joints']==13 and result['bodies']==669 and abs(mass-4891.194067702904)<.002 and all(not r['ground_to_parent']for r in result['axes'])
    api['write']('modular_urdf_archive_reopen',result)
    with open(os.path.join(out,'native_archive_reopen.json'),'w',encoding='utf8')as f:json.dump(result,f,indent=2)
    doc.close(False);prior.activate()
    if not result['passed']:raise RuntimeError('Saved native export archive differs from validated copy')
