import adsk.core,adsk.fusion,os,json,importlib.util

def run(app,api):
    source=api['source'](app);source_doc=app.activeDocument;mgr=adsk.fusion.TemporaryBRepManager.get();out=api['OUT'];cad=api['CAD_OUT']
    refs=[]
    for o in list(source.rootComponent.occurrences):
        if o.isReferencedComponent:
            name=o.name
            if not o.breakLink():raise RuntimeError('External link could not be localized: '+name)
            refs.append(name)
    source.computeAll();adsk.doEvents();api['write']('modular_external_links_localized',{'localized':refs,'remaining':[o.fullPathName for o in source.rootComponent.allOccurrences if o.isReferencedComponent]})
    choices=[]
    for d in app.documents:
        dd=adsk.fusion.Design.cast(d.products.itemByProductType('DesignProductType'))
        if dd and dd.rootComponent.attributes.itemByName('WaffleModular','revision'):choices.append(dd)
    if not choices:
        app.importManager.importToNewDocument(app.importManager.createFusionArchiveImportOptions(os.path.join(cad,'Floor_Bracket_Modular_261021.f3d')))
        new=adsk.fusion.Design.cast(app.activeProduct)
    else:new=choices[-1]
    new.parentDocument.activate();new.rootComponent.attributes.add('WaffleModular','motor_mount_revision','M2p5_clearance_2p8_16mm_only')
    old=[-238.699945,-230.699949,-158.699945,-150.699943,-78.699942,-70.699931]
    ys=[-242.699946,-226.699947,-162.699948,-146.699945,-82.699944,-66.699944]
    report=[]
    for side in ['Left','Right']:
        no=next(o for o in new.rootComponent.occurrences if o.component.name=='Floor_Bracket_'+side+'_Main');new.parentDocument.activate();temp=mgr.copy(no.component.bRepBodies.item(0));x=103.300016 if side=='Left' else 22.10003
        for y in old:
            for lo,hi in [(0,3),(87,90)]:
                tool=mgr.createCylinderOrCone(adsk.core.Point3D.create(x/10,y/10,lo/10),.1027,adsk.core.Point3D.create(x/10,y/10,hi/10),.1027)
                api['boolean'](temp,tool,True)
        for y in ys:
            tool=mgr.createCylinderOrCone(adsk.core.Point3D.create(x/10,y/10,-.01),.14,adsk.core.Point3D.create(x/10,y/10,9.01),.14)
            api['boolean'](temp,tool,False)
        bf=no.component.features.baseFeatures.item(0);bf.startEdit()
        try:
            if not bf.updateBody(bf.sourceBodies[0],temp):raise RuntimeError('Main mount update failed')
        finally:bf.finishEdit()
        source_doc.activate();o=next(o for o in source.rootComponent.occurrences if o.name=='Chassis_Bracket_'+side+'_16mm:1');aligned=mgr.copy(temp);mgr.transform(aligned,api['mat'](t=(-114.700031280518,282.699927706055,0)));bf=next(f for f in o.component.features.baseFeatures if f.name=='Modular_Floor_Bracket_261021');bf.startEdit()
        try:
            oldbody=next(b for b in bf.sourceBodies if b.volume*1000>100000)
            if not bf.updateBody(oldbody,aligned):raise RuntimeError('Assembly mount update failed')
        finally:bf.finishEdit()
        report.append(dict(side=side,hole_diameter_mm=2.8,motor_pair_spacing_mm=16,motor_pairs_per_side=3,obsolete_8mm_pairs_removed=True))
    new.parentDocument.activate();base=os.path.join(cad,'Floor_Bracket_Modular_261021');new.exportManager.execute(new.exportManager.createFusionArchiveExportOptions(base+'.f3d'));new.exportManager.execute(new.exportManager.createSTEPExportOptions(base+'.step'));api['write']('modular_motor_mount_corrected',report)
    source_doc.activate();source.computeAll();adsk.doEvents()
    spec=importlib.util.spec_from_file_location('ModularValidate',os.path.join(os.path.dirname(__file__),'ModularValidate.py'));m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);m.run(app,api)
    before=sorted(j.name for j in source.rootComponent.asBuiltJoints)
    imported=app.importManager.importToNewDocument(app.importManager.createFusionArchiveImportOptions(os.path.join(cad,'Assembly_Modular_261021.f3d')))
    checked=adsk.fusion.Design.cast(app.activeProduct);after=sorted(j.name for j in checked.rootComponent.asBuiltJoints)
    verification={'status':'reopened','same_joint_names':before==after,'joints':after,'top_occurrences':checked.rootComponent.occurrences.count,'remaining_external_references':[o.fullPathName for o in checked.rootComponent.allOccurrences if o.isReferencedComponent]}
    api['write']('modular_native_reopen_verified',verification)
    if before!=after or verification['remaining_external_references']:raise RuntimeError('Native reopened joint/reference check failed')
    imported.close(False);source_doc.activate()
