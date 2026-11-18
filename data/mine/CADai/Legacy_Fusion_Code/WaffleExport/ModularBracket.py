import adsk.core, adsk.fusion, os, json

def run(app, action, api):
    if action=='modular_bracket_sim_collision_sources':
        import importlib.util
        spec=importlib.util.spec_from_file_location('SimCollisionSource',os.path.join(os.path.dirname(__file__),'SimCollisionSource.py'))
        mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod);mod.run(app,api)
        return
    if action=='modular_bracket_urdf_reopen':
        import importlib.util
        spec=importlib.util.spec_from_file_location('ModularURDFReady',os.path.join(os.path.dirname(__file__),'ModularURDFReady.py'))
        mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod);mod.reopen(app,api)
        return
    if action=='modular_bracket_urdf_ready':
        import importlib.util
        spec=importlib.util.spec_from_file_location('ModularURDFReady',os.path.join(os.path.dirname(__file__),'ModularURDFReady.py'))
        mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod);mod.run(app,api)
        return
    if action=='modular_bracket_repair_derived_materials':
        import importlib.util
        spec=importlib.util.spec_from_file_location('ModularMaterialRepair',os.path.join(os.path.dirname(__file__),'ModularMaterialRepair.py'))
        mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod);mod.run(app,api)
        return
    if action=='modular_bracket_review2':
        import importlib.util
        spec=importlib.util.spec_from_file_location('ModularReview2',os.path.join(os.path.dirname(__file__),'ModularReview2.py'))
        mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod);mod.run(app,api)
        return
    if action=='modular_bracket_stop_fix':
        import importlib.util
        for name in ['ModularStopFix','ModularJointProbe','ModularFinal']:
            spec=importlib.util.spec_from_file_location(name,os.path.join(os.path.dirname(__file__),name+'.py'))
            mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod);mod.run(app,dict(api,print_only=True))
        return
    if action=='modular_bracket_joint_probe':
        import importlib.util
        for name in ['ModularJointProbe','ModularFinal']:
            spec=importlib.util.spec_from_file_location(name,os.path.join(os.path.dirname(__file__),name+'.py'))
            mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod);mod.run(app,dict(api,print_only=True))
        return
    if action=='modular_bracket_review_stage1':
        import importlib.util
        for name in ['ModularJointProbe','ModularFold','ModularFinal']:
            spec=importlib.util.spec_from_file_location(name,os.path.join(os.path.dirname(__file__),name+'.py'))
            mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod);mod.run(app,api)
        collision=json.load(open(os.path.join(api['OUT'],'modular_motion_collision.json'),encoding='utf8'))
        if collision['status']!='sampled_clear':
            api['write']('modular_stage1_runtime',{'status':'motion_requires_correction','collision_status':collision['status']})
            return
        design=api['source'](app);doc=app.activeDocument
        spec=importlib.util.spec_from_file_location('WafflePlayerFinal',os.path.join(os.path.dirname(__file__),'WafflePlayerV2.py'))
        mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod);mod.run(app,'play')
        filename=os.path.join(api['CAD_OUT'],'Assembly_Modular_261021.f3d')
        if not design.exportManager.execute(design.exportManager.createFusionArchiveExportOptions(filename)):raise RuntimeError('Final archive save failed')
        before=sorted(j.name for j in design.rootComponent.asBuiltJoints)
        verify_doc=app.importManager.importToNewDocument(app.importManager.createFusionArchiveImportOptions(filename));reopened=adsk.fusion.Design.cast(app.activeProduct)
        result=dict(joint_names_match=before==sorted(j.name for j in reopened.rootComponent.asBuiltJoints),remaining_external_references=[o.fullPathName for o in reopened.rootComponent.allOccurrences if o.isReferencedComponent],top_occurrences=reopened.rootComponent.occurrences.count)
        api['write']('modular_final_reopen_verified',result)
        if not result['joint_names_match']or result['remaining_external_references']:raise RuntimeError('Final native round-trip check failed')
        verify_doc.close(False);doc.activate()
        return
    if action=='modular_bracket_frame_fix':
        import importlib.util
        spec=importlib.util.spec_from_file_location('ModularFrameFix',os.path.join(os.path.dirname(__file__),'ModularFrameFix.py'))
        mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod);mod.run(app,api)
        return
    if action=='modular_bracket_mount_fix':
        import importlib.util
        spec=importlib.util.spec_from_file_location('ModularMountFix',os.path.join(os.path.dirname(__file__),'ModularMountFix.py'))
        mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod);mod.run(app,api)
        return
    if action=='modular_bracket_fold_measure':
        import importlib.util
        spec=importlib.util.spec_from_file_location('ModularFold',os.path.join(os.path.dirname(__file__),'ModularFold.py'))
        mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod);mod.run(app,api)
        return
    if action=='modular_bracket_finalize':
        import importlib.util
        spec=importlib.util.spec_from_file_location('ModularFinal',os.path.join(os.path.dirname(__file__),'ModularFinal.py'))
        mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod);mod.run(app,api)
        return
    if action=='modular_bracket_motor_relief':
        import importlib.util
        spec=importlib.util.spec_from_file_location('ModularRelief',os.path.join(os.path.dirname(__file__),'ModularRelief.py'))
        mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod);mod.run(app,api)
        return
    if action=='modular_bracket_motor_relief_inspect':
        design=api['source'](app);root=design.rootComponent;mgr=adsk.fusion.TemporaryBRepManager.get();report=[]
        for side in ['Left','Right']:
            bracket=next(o for o in root.occurrences if o.name=='Chassis_Bracket_'+side+'_16mm:1')
            body=next(b for b in bracket.component.bRepBodies if b.isLightBulbOn and b.volume*1000>100000)
            main=mgr.copy(body);mgr.transform(main,bracket.transform2)
            inverse=api['mul'](bracket.transform2,api['mat'](t=(-114.700031280518,282.699927706055,0)));inverse.invert()
            for end in ['Front','Rear']:
                motor=next(o for o in root.occurrences if o.name=='Motor_'+end+'_'+side+':1')
                for path,b,orig in api['world_bodies'](motor.component):
                    mgr.transform(b,motor.transform2);test=mgr.copy(main)
                    if not mgr.booleanOperation(test,b,adsk.fusion.BooleanTypes.IntersectionBooleanType) or not test.isSolid or test.volume*1000<.05:continue
                    row={'side':side,'motor':motor.name,'body':orig.name,'volume_mm3':test.volume*1000,'world_bounds':api['bb'](test)}
                    mgr.transform(test,inverse);row['input_bounds']=api['bb'](test);row['input_vertices']=[api['xyz'](v.geometry)for v in test.vertices]
                    report.append(row);api['write']('modular_motor_relief_diagnostic',report);adsk.doEvents()
        return
    if action=='modular_bracket_validate':
        import importlib.util
        spec=importlib.util.spec_from_file_location('ModularValidate',os.path.join(os.path.dirname(__file__),'ModularValidate.py'))
        mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod);mod.run(app,api)
        return
    if action=='modular_bracket_commit':
        import importlib.util
        spec=importlib.util.spec_from_file_location('ModularCommit',os.path.join(os.path.dirname(__file__),'ModularCommit.py'))
        mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod);mod.run(app,api)
        return
    if action=='modular_bracket_probe':
        design=api['source'](app);root=design.rootComponent;mgr=adsk.fusion.TemporaryBRepManager.get()
        candidates=[]
        for d in app.documents:
            dd=adsk.fusion.Design.cast(d.products.itemByProductType('DesignProductType'))
            if dd and dd.rootComponent.attributes.itemByName('WaffleModular','revision'):candidates.append(dd)
        new=candidates[-1];report={'parts':[]}
        floor=[]
        for fo in root.occurrences:
            if fo.name not in ['Floor_1_Joined:1','Floor_3_Joined:1']:continue
            for path,b,orig in api['world_bodies'](fo.component):
                mgr.transform(b,fo.transform2);floor.append((fo.name,b))
        for o in new.rootComponent.occurrences:
            side='Left' if '_Left_' in o.name else 'Right'
            old=next(q for q in root.occurrences if q.name=='Chassis_Bracket_'+side+'_16mm:1')
            t=old.transform2.copy();v=t.translation;v.x-=1.25;t.translation=v
            m=api['mul'](t,api['mat'](t=(-114.700031280518,282.699927706055,0)))
            b=mgr.copy(o.component.bRepBodies.item(0));mgr.transform(b,m)
            row={'name':o.name,'world_bounds':api['bb'](b),'interferences':[]}
            for n,f in floor:
                v=api['intersects'](b,f)
                row['interferences'].append({'other':n,'volume_mm3':v})
                api['write']('modular_bracket_probe_progress',{'part':o.name,'other':n,'volume_mm3':v});adsk.doEvents()
            report['parts'].append(row);api['write']('modular_bracket_probe',report)
        return
    if action=='modular_bracket_build_preview':
        import importlib.util
        spec=importlib.util.spec_from_file_location('ModularDesign',os.path.join(os.path.dirname(__file__),'ModularDesign.py'))
        mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod);mod.build(app,api)
        return
    if action in ['modular_bracket_mount_vertices','modular_bracket_all_vertices']:
        doc=next(d for d in app.documents if d.name.startswith('floor_braket_modify'))
        doc.activate();design=adsk.fusion.Design.cast(app.activeProduct)
        report=[]
        for b in design.rootComponent.bRepBodies:
            points=[]
            for v in b.vertices:
                p=v.geometry
                if action=='modular_bracket_all_vertices' or min(abs(p.z),abs(p.z-9))<.0001:points.append(api['xyz'](p))
            report.append({'body':b.name,'points_mm':points})
            api['write']('modular_bracket_all_vertices' if action=='modular_bracket_all_vertices' else 'modular_bracket_mount_vertices',report);adsk.doEvents()
        return
    if action=='modular_bracket_mount_loops':
        doc=next(d for d in app.documents if d.name.startswith('floor_braket_modify'))
        doc.activate();design=adsk.fusion.Design.cast(app.activeProduct)
        report=[]
        for b in design.rootComponent.bRepBodies:
            row={'body':b.name,'loops':[]}
            for f in b.faces:
                g=f.geometry
                if not isinstance(g,adsk.core.Plane) or abs(g.normal.z)<.999 or min(abs(g.origin.z),abs(g.origin.z-9))>.0001:continue
                for loop in f.loops:
                    if loop.isOuter:continue
                    points=[api['xyz'](edge.startVertex.geometry) for edge in loop.edges]
                    row['loops'].append({'z_mm':g.origin.z*10,'points_mm':points})
            report.append(row);api['write']('modular_bracket_mount_loops',report);adsk.doEvents()
        return
    if action=='modular_bracket_panel_holes':
        design=api['source'](app);root=design.rootComponent
        report={'document':app.activeDocument.name,'parts':[]}
        for o in root.occurrences:
            if o.name not in ['Floor_1_Joined:1','Floor_3_Joined:1']:continue
            for path,b,original in api['world_bodies'](o.component):
                holes=[]
                for f in b.faces:
                    g=f.geometry
                    if isinstance(g,adsk.core.Cylinder) and abs(g.axis.z)>.99:
                        p=g.origin.copy();p.transformBy(o.transform2)
                        holes.append({'center_mm':api['xyz'](p),'radius_mm':g.radius*10})
                report['parts'].append({'part':o.name,'path':path,'faces':b.faces.count,'holes':holes})
                api['write']('modular_bracket_panel_holes',report);adsk.doEvents()
        return
    report={'active_document':app.activeDocument.name,'documents':[]}
    mgr=adsk.fusion.TemporaryBRepManager.get()
    docs=list(app.documents)
    docs.sort(key=lambda d:0 if d==app.activeDocument else 1)
    for doc in docs:
        design=adsk.fusion.Design.cast(doc.products.itemByProductType('DesignProductType'))
        if not design:continue
        row={'name':doc.name,'is_modified':doc.isModified,'root':design.rootComponent.name,'bodies':[],'components':[]}
        try:row['data_file']={'name':doc.dataFile.name,'id':doc.dataFile.id,'version':doc.dataFile.versionNumber}
        except:row['data_file']=None
        for b in (design.rootComponent.bRepBodies if doc.name.startswith('floor_braket_modify') else []):
            g=api['geometry'](b);g['volume_mm3']=b.volume*1000
            row['bodies'].append(g)
        for o in design.rootComponent.occurrences:
            part={'name':o.name,'visible':o.isVisible,'transform_cm':o.transform2.asArray(),'bounds':api['bb'](o)}
            if any(s in o.name.lower() for s in ['bracket','floor','plate','housing','magazine']):
                part['bodies']=[]
                for b in o.component.bRepBodies:
                    if not b.isLightBulbOn:continue
                    t=mgr.copy(b);mgr.transform(t,o.transform2)
                    part['bodies'].append(api['geometry'](t))
            row['components'].append(part)
            api['write']('modular_bracket_inspection_progress',{'document':doc.name,'component':o.name})
            adsk.doEvents()
        report['documents'].append(row)
        api['write']('modular_bracket_inspection',report)
    api['write']('modular_bracket_inspection',report)
