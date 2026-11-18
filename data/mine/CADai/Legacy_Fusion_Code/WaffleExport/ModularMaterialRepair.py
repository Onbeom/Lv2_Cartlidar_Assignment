import adsk.core,adsk.fusion,os,importlib.util

def run(app,api):
    source=api['source'](app);source_doc=app.activeDocument;report=[]
    def key(temp):
        box=api['bb'](temp)
        return tuple(round(x,3)for x in box['min']+box['max'])+(round(temp.volume*1000,2)if temp.isSolid else 0,)
    def copy_material(d,body,material):
        existing=d.materials.itemByName(material.name)
        if not existing:existing=d.materials.addByCopy(material,material.name)
        body.material=existing
    catalog={}
    for path,temp,native in api['world_bodies'](source.rootComponent):catalog.setdefault(key(temp),[]).append(native)
    expected_robot_mass=sum(b.getPhysicalProperties(adsk.fusion.CalculationAccuracy.MediumCalculationAccuracy).mass*1000 for bodies in catalog.values()for b in bodies)
    path=os.path.join(api['CAD_OUT'],'Assembly_Modular_261021_Geometry_Only.f3d')
    doc=app.importManager.importToNewDocument(app.importManager.createFusionArchiveImportOptions(path));d=adsk.fusion.Design.cast(app.activeProduct);count=0
    for p,temp,body in api['world_bodies'](d.rootComponent):
        candidates=catalog.get(key(temp),[])
        if not candidates:raise RuntimeError('No source geometry/material match: '+p+' '+body.name)
        masses=[b.getPhysicalProperties(adsk.fusion.CalculationAccuracy.MediumCalculationAccuracy).mass for b in candidates]
        if max(masses)-min(masses)>1e-7:raise RuntimeError('Ambiguous mass match: '+p)
        copy_material(d,body,candidates[0].material);count+=1
        if count%50==0:adsk.doEvents()
    mass=sum(b.getPhysicalProperties(adsk.fusion.CalculationAccuracy.MediumCalculationAccuracy).mass*1000 for p,t,b in api['world_bodies'](d.rootComponent))
    if abs(mass-expected_robot_mass)>.001:raise RuntimeError('Geometry mass mismatch '+str(mass))
    if not d.exportManager.execute(d.exportManager.createFusionArchiveExportOptions(path)):raise RuntimeError('Geometry archive save failed')
    report.append(dict(file=path,bodies=count,mass_g=mass));doc.close(False);source_doc.activate()
    bracket_path=os.path.join(api['CAD_OUT'],'Floor_Bracket_Modular_261021.f3d');bracket_doc=app.importManager.importToNewDocument(app.importManager.createFusionArchiveImportOptions(bracket_path));bd=adsk.fusion.Design.cast(app.activeProduct)
    materials={o.component.name:o.component.bRepBodies.item(0).material for o in bd.rootComponent.occurrences}
    expected_bracket_mass=sum(b.getPhysicalProperties(adsk.fusion.CalculationAccuracy.MediumCalculationAccuracy).mass*1000 for p,t,b in api['world_bodies'](bd.rootComponent))
    path=os.path.join(api['CAD_OUT'],'Printable_Modular_261021','Floor_Bracket_Print_Layout_261021.f3d');doc=app.importManager.importToNewDocument(app.importManager.createFusionArchiveImportOptions(path));d=adsk.fusion.Design.cast(app.activeProduct)
    for o in d.rootComponent.occurrences:
        for body in o.component.bRepBodies:copy_material(d,body,materials[o.component.name])
    mass=sum(b.getPhysicalProperties(adsk.fusion.CalculationAccuracy.MediumCalculationAccuracy).mass*1000 for p,t,b in api['world_bodies'](d.rootComponent))
    if abs(mass-expected_bracket_mass)>.001:raise RuntimeError('Print layout mass mismatch '+str(mass))
    if not d.exportManager.execute(d.exportManager.createFusionArchiveExportOptions(path)):raise RuntimeError('Print layout archive save failed')
    report.append(dict(file=path,bodies=4,mass_g=mass));doc.close(False);bracket_doc.close(False);source_doc.activate();api['write']('modular_derived_materials_repaired',report)
    spec=importlib.util.spec_from_file_location('ModularReview2',os.path.join(os.path.dirname(__file__),'ModularReview2.py'));mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod);mod.run(app,api)
