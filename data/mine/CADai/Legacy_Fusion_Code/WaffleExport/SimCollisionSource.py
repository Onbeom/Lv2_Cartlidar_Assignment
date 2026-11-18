import adsk.core,adsk.fusion,json,os
def run(app,api):
    source=api['source'](app);prior=app.activeDocument
    workspace=os.path.dirname(os.path.dirname(__file__));out=os.path.join(workspace,'sim_work','native_sources');os.makedirs(out,exist_ok=True)
    meta=json.load(open(os.path.join(workspace,'sim_work','mesh_components.json'),encoding='utf8'))
    urdf=os.path.join(api['CAD_OUT'],'submit','urdf','waffle_4wd_description_261021','urdf','waffle_4wd.urdf')
    import importlib.util
    spec=importlib.util.spec_from_file_location('portable_fk',os.path.join(workspace,'CADai_preparation','portable_fk.py'));fk=importlib.util.module_from_spec(spec);spec.loader.exec_module(fk);frames=fk.Robot(urdf).fk()
    selected={}
    for link in meta:
        for p in link['components']:
            path=p['nearest_path'];vol=p['volume_mm3']
            if link['link']=='base_link'and vol>10000 and any(x in path for x in ['housing:','magazine_','Chassis_Bracket_','rail_','translator 8.0 1:2'])or link['link'].startswith('gripper_')and vol>1000:
                selected[(path,p['nearest_body'])]=(link['link'],p)
    bodies=[]
    for path,temp,native in api['world_bodies'](source.rootComponent):
        if (path,native.name)in selected:bodies.append((path,temp,native,selected[(path,native.name)]))
    if len(bodies)!=len(selected):raise RuntimeError('Native collision body correspondence missing')
    doc=app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType);d=adsk.fusion.Design.cast(app.activeProduct);root=d.rootComponent;report=[]
    for path,temp,native,(link,p)in bodies:
        F=frames[link];matrix=api['mat'](t=[F[i][3]for i in range(3)],r=[row[:3]for row in F[:3]]);matrix.invert();adsk.fusion.TemporaryBRepManager.get().transform(temp,matrix)
        o=root.occurrences.addNewComponent(adsk.core.Matrix3D.create());bf=o.component.features.baseFeatures.add();bf.startEdit();body=o.component.bRepBodies.add(temp,bf);bf.finishEdit()
        name=link+'_part'+str(p['index']).zfill(3);filename=os.path.join(out,name+'.stl');options=d.exportManager.createSTLExportOptions(o.component,filename);options.unitType=adsk.fusion.DistanceUnits.MillimeterDistanceUnits
        options.meshRefinement=adsk.fusion.MeshRefinementSettings.MeshRefinementMedium
        if not d.exportManager.execute(options):raise RuntimeError('Collision source STL export failed')
        report.append(dict(link=link,component_index=p['index'],path=path,body=native.name,file=filename,bounds_local_mm=api['bb'](body)));adsk.doEvents()
    with open(os.path.join(out,'manifest.json'),'w',encoding='utf8')as f:json.dump(report,f,ensure_ascii=False,indent=2)
    api['write']('simulation_native_collision_sources',dict(count=len(report),files=report))
    doc.close(False);prior.activate()
