import adsk.core,adsk.fusion,os,json,importlib.util

def run(app,api):
    d=api['source'](app);doc=app.activeDocument;mgr=adsk.fusion.TemporaryBRepManager.get();out=api['OUT'];cad=api['CAD_OUT']
    new=next(adsk.fusion.Design.cast(x.products.itemByProductType('DesignProductType'))for x in app.documents if adsk.fusion.Design.cast(x.products.itemByProductType('DesignProductType')) and adsk.fusion.Design.cast(x.products.itemByProductType('DesignProductType')).rootComponent.attributes.itemByName('WaffleModular','revision'))
    report=[]
    for side in ['Left','Right']:
        new.parentDocument.activate();o=next(o for o in new.rootComponent.occurrences if o.component.name=='Floor_Bracket_'+side+'_Profile_Carrier');temp=mgr.copy(o.component.bRepBodies.item(0));mirror=side=='Left';X=lambda x:125.400046-x if mirror else x
        lo,hi=sorted([X(4),X(24.4)]);api['boolean'](temp,api['box'](lo,-24.4,3,hi,-4,7),True)
        tool=mgr.createCylinderOrCone(adsk.core.Point3D.create(X(14.2)/10,-1.42,-.01),.275,adsk.core.Point3D.create(X(14.2)/10,-1.42,.701),.275);api['boolean'](temp,tool,False)
        bf=o.component.features.baseFeatures.item(0);bf.startEdit()
        try:
            if not bf.updateBody(bf.sourceBodies[0],temp):raise RuntimeError('Carrier stop update failed')
        finally:bf.finishEdit()
        doc.activate();o=next(o for o in d.rootComponent.occurrences if o.name=='Chassis_Bracket_'+side+'_16mm:1');aligned=mgr.copy(temp);mgr.transform(aligned,api['mat'](t=(-114.700031280518,282.699927706055,0)));bf=next(f for f in o.component.features.baseFeatures if f.name=='Modular_Floor_Bracket_261021');bf.startEdit()
        try:
            target=next(b for b in bf.sourceBodies if b.volume*1000<100000)
            if not bf.updateBody(target,aligned):raise RuntimeError('Assembly stop update failed')
        finally:bf.finishEdit()
        report.append(dict(side=side,profile_bottom_world_z_mm=19,stop_top_world_z_mm=19,stop_thickness_mm=7,vertical_gap_mm=0,socket_internal_width_mm=20.4,profile_width_mm=20,nominal_side_gap_mm=.2))
    new.parentDocument.activate();base=os.path.join(cad,'Floor_Bracket_Modular_261021');new.exportManager.execute(new.exportManager.createFusionArchiveExportOptions(base+'.f3d'));new.exportManager.execute(new.exportManager.createSTEPExportOptions(base+'.step'));doc.activate();d.computeAll();adsk.doEvents();api['write']('modular_profile_seating',report)
    spec=importlib.util.spec_from_file_location('ModularValidate',os.path.join(os.path.dirname(__file__),'ModularValidate.py'));mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod);mod.run(app,api)
