import adsk.core,adsk.fusion,os,json,importlib.util
def run(app,api):
    source=api['source'](app);source_doc=app.activeDocument;mgr=adsk.fusion.TemporaryBRepManager.get();out=api['OUT'];new=[]
    for d in app.documents:
        dd=adsk.fusion.Design.cast(d.products.itemByProductType('DesignProductType'))
        if dd and dd.rootComponent.attributes.itemByName('WaffleModular','revision'):new.append(dd)
    new=new[-1];report=[]
    for side in ['Left','Right']:
        no=next(o for o in new.rootComponent.occurrences if o.component.name=='Floor_Bracket_'+side+'_Main')
        new.parentDocument.activate();temp=mgr.copy(no.component.bRepBodies.item(0))
        x0,x1=(44.700023,45.3001) if side=='Right' else (125.400046-45.3001,125.400046-44.700023)
        api['boolean'](temp,api['box'](x0,-249.9501,3,x1,-32.2999,6),False)
        bf=no.component.features.baseFeatures.item(0);bf.startEdit()
        try:
            if not bf.updateBody(bf.sourceBodies[0],temp):raise RuntimeError('Standalone main body update failed')
        finally:bf.finishEdit()
        source_doc.activate();o=next(o for o in source.rootComponent.occurrences if o.name=='Chassis_Bracket_'+side+'_16mm:1')
        aligned=mgr.copy(temp);mgr.transform(aligned,api['mat'](t=(-114.700031280518,282.699927706055,0)))
        bf=next(f for f in o.component.features.baseFeatures if f.name=='Modular_Floor_Bracket_261021');bf.startEdit()
        try:
            old=next(b for b in bf.sourceBodies if b.volume*1000>100000)
            if not bf.updateBody(old,aligned):raise RuntimeError('Assembly main body update failed')
        finally:bf.finishEdit()
        report.append({'side':side,'motor_body_clearance_mm':.20007,'remaining_reinforcement_width_mm':2.399923,'wall_mm':5,'flange_mm':3})
    source.computeAll();adsk.doEvents();api['write']('modular_motor_relief_applied',report)
    new.parentDocument.activate();base=os.path.join(api['CAD_OUT'],'Floor_Bracket_Modular_261021')
    new.exportManager.execute(new.exportManager.createFusionArchiveExportOptions(base+'.f3d'));new.exportManager.execute(new.exportManager.createSTEPExportOptions(base+'.step'))
    source_doc.activate()
    spec=importlib.util.spec_from_file_location('ModularValidate',os.path.join(os.path.dirname(__file__),'ModularValidate.py'));mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod);mod.run(app,api)
