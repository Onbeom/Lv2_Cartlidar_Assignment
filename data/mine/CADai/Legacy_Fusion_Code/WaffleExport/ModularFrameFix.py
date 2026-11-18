import adsk.core,adsk.fusion,os,json,importlib.util,math

def run(app,api):
    d=api['source'](app);root=d.rootComponent;out=api['OUT'];lookup={o.name:o for o in root.occurrences}
    old=next(x for x in json.load(open(os.path.join(out,'modular_bracket_inspection.json'),encoding='utf8'))['documents']if x['name']=='assembly')
    initial={c['name']:c['transform_cm']for c in old['components']};change=json.load(open(os.path.join(out,'modular_bracket_committed.json'),encoding='utf8'));desired=dict(initial);desired.update(change['new_transforms_cm'])
    frame=next(g for g in root.rigidGroups if g.name=='Waffle_4WD_Static_Frame');camera=next(g for g in root.rigidGroups if g.name=='Camera_Top_Fixed_Assembly')
    members=sorted(set(o.fullPathName.split('+')[0]for o in frame.occurrences));frame.isSuppressed=True;camera.isSuppressed=True
    floor=lookup['Floor_3_Joined:1'];m=adsk.core.Matrix3D.create();m.setWithArray(initial[floor.name]);floor.transform2=m
    if d.snapshots.hasPendingSnapshot:d.snapshots.add()
    adsk.doEvents();frame.deleteMe();camera.deleteMe()
    g=root.rigidGroups.add(adsk.core.ObjectCollection.createWithArray([lookup[n]for n in members]),True);g.name='Waffle_4WD_Static_Frame'
    # The crossbar attaches to the profiles, rather than imposing its old
    # relation to the top deck during a separate frame translation.
    names=['Profile_2020_450mm_Rear_Left_Envelope:1','Profile_2020_450mm_Rear_Right_Envelope:1','Camera_Top_Crossbar_2020_261mm:1','Camera_D435_Pair_Source:1']
    g=root.rigidGroups.add(adsk.core.ObjectCollection.createWithArray([lookup[n]for n in names]),True);g.name='Camera_Top_Fixed_Assembly'
    if d.snapshots.hasPendingSnapshot:d.snapshots.add()
    d.computeAll();adsk.doEvents();errors=[]
    for o in root.occurrences:
        expected=desired.get(o.name,desired.get('USB2LSD:4')if o.name=='USB2LSD:1'else None)
        if expected is None:continue
        error=max(abs(a-b)for a,b in zip(o.transform2.asArray(),expected))
        if error>1e-6:errors.append(dict(part=o.name,error=error))
    api['write']('modular_full_pose_restored',dict(errors=errors,checked=len(desired),camera_group_top_members=names,top_deck_restored=True))
    if errors:raise RuntimeError('Unexpected component pose remains: '+str(errors[:3]))
    # Restore the agreed USB2LDS equivalent mass after localizing its reference.
    usb=next(o for o in root.occurrences if o.name.startswith('USB2LSD:'));c=usb.component
    volume=sum(b.getPhysicalProperties(adsk.fusion.CalculationAccuracy.MediumCalculationAccuracy).volume for b in c.bRepBodies)
    material=d.materials.itemByName('Waffle_USB2LDS_Local_8g')
    if not material:material=d.materials.addByCopy(c.bRepBodies.item(0).material,'Waffle_USB2LDS_Local_8g')
    for p in material.materialProperties:
        if p.id in ['structural_Density','thermal_Density']:adsk.core.FloatProperty.cast(p).value=8000/volume
    c.material=material
    for bf in c.features.baseFeatures:
        for b in bf.sourceBodies:b.material=material
    for b in c.bRepBodies:b.material=material
    d.computeAll();adsk.doEvents();mass=sum(b.getPhysicalProperties(adsk.fusion.CalculationAccuracy.MediumCalculationAccuracy).mass*1000 for b in c.bRepBodies)
    api['write']('modular_usb_mass_restored',dict(mass_g=mass,target_g=8))
    if abs(mass-8)>.001:raise RuntimeError('USB material did not persist')
    spec=importlib.util.spec_from_file_location('ModularValidate',os.path.join(os.path.dirname(__file__),'ModularValidate.py'));mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod);mod.run(app,api)
