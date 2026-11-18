import adsk.core, adsk.fusion, os, json
INPUT=os.path.join(os.path.dirname(os.path.dirname(__file__)),'Waffle_Handoff_20261020','inputs','floor_braket_modify.step')

def run(app,action,api):
    globals().update({k:api[k] for k in ['source','write','geometry','world_bodies','mat','mul','bb','OUT','xyz']})
    design=source(app);doc=app.activeDocument;root=design.rootComponent
    if action=='floor_revision_inspect':
        report={'old_brackets':[], 'gripper':[], 'rigid_groups':[]}
        mgr=adsk.fusion.TemporaryBRepManager.get()
        for o in root.occurrences:
            if 'Chassis_Bracket' in o.name:
                rows=[]
                for p,b,n in world_bodies(o.component):rows.append({'path':p,'geometry':geometry(b)})
                report['old_brackets'].append({'name':o.name,'transform_cm':o.transform2.asArray(),'bodies':rows})
            if 'RMX52_' in o.name and any(s in o.name for s in ['IGP','PR33','J6','WBH_M3X10_2']):
                report['gripper'].append({'name':o.name,'transform_cm':o.transform2.asArray(),'bounds':bb(o),'grounded':o.isGrounded,'ground_to_parent':o.isGroundToParent})
        report['rigid_groups']=[{'name':g.name,'suppressed':g.isSuppressed,'members':[o.name for o in g.occurrences]} for g in root.rigidGroups]
        imported=app.importManager.importToNewDocument(app.importManager.createSTEPImportOptions(INPUT))
        dd=adsk.fusion.Design.cast(app.activeProduct)
        report['input']=[{'path':p,'geometry':geometry(b)} for p,b,n in world_bodies(dd.rootComponent)]
        report['input_document']=imported.name
        write('floor_revision_inspection',report)
        doc.activate()
    elif action=='floor_revision_preflight':
        candidates=[]
        for d in app.documents:
            dd=adsk.fusion.Design.cast(d.products.itemByProductType('DesignProductType'))
            if dd and dd.rootComponent.bRepBodies.count==2 and all(abs(b.boundingBox.maxPoint.y-b.boundingBox.minPoint.y-24.995)<.001 for b in dd.rootComponent.bRepBodies):candidates.append(dd)
        if len(candidates)!=1:raise RuntimeError('Expected one open 249.95mm bracket STEP document')
        dd=candidates[0]
        mgr=adsk.fusion.TemporaryBRepManager.get();inputs=sorted(list(dd.rootComponent.bRepBodies),key=lambda b:-b.boundingBox.minPoint.x)
        report={'parts':[]}
        for i,b in enumerate(inputs):
            name=['Chassis_Bracket_Left_16mm','Chassis_Bracket_Right_16mm'][i]
            o=next(o for o in root.occurrences if o.component.name==name);old=o.component.bRepBodies.item(0)
            temp=mgr.copy(b);mgr.transform(temp,mat(t=(-114.700031280518,282.699927706055,0)))
            loops=[]
            for f in temp.faces:
                g=f.geometry
                if not isinstance(g,adsk.core.Plane) or abs(g.normal.z)<.999 or abs(g.origin.z*10-90)>.01:continue
                for loop in f.loops:
                    if loop.isOuter:continue
                    loops.append([xyz(v.geometry) for e in loop.edges for v in [e.startVertex]])
            p=b.getPhysicalProperties(adsk.fusion.CalculationAccuracy.MediumCalculationAccuracy)
            report['parts'].append(dict(name=name,input_index=i,local_bounds=bb(temp),volume_mm3=p.volume*1000,old_volume_mm3=old.getPhysicalProperties(adsk.fusion.CalculationAccuracy.MediumCalculationAccuracy).volume*1000,top_hole_loops_mm=loops))
            write('floor_revision_preflight',report);adsk.doEvents()
        doc.activate()
    elif action=='floor_revision_commit':
        if os.path.exists(os.path.join(OUT,'floor_revision_committed.json')):raise RuntimeError('Floor revision already committed')
        candidates=[]
        for d in app.documents:
            dd=adsk.fusion.Design.cast(d.products.itemByProductType('DesignProductType'))
            if dd and dd.rootComponent.bRepBodies.count==2 and all(abs(b.boundingBox.maxPoint.y-b.boundingBox.minPoint.y-24.995)<.001 for b in dd.rootComponent.bRepBodies):candidates.append(dd)
        if len(candidates)!=1:raise RuntimeError('Exact user STEP document missing')
        inputs=sorted(list(candidates[0].rootComponent.bRepBodies),key=lambda b:-b.boundingBox.minPoint.x)
        mgr=adsk.fusion.TemporaryBRepManager.get();prepared=[];report={'parts':[]}
        alignment=mat(t=(-114.700031280518,282.699927706055,0))
        for i,b in enumerate(inputs):
            o=next(o for o in root.occurrences if o.component.name==['Chassis_Bracket_Left_16mm','Chassis_Bracket_Right_16mm'][i])
            old=next(b for b in o.component.bRepBodies if b.isLightBulbOn)
            temp=mgr.copy(b);mgr.transform(temp,alignment)
            vertices=list({tuple(round(v,5) for v in xyz(p.geometry)) for p in temp.vertices if abs(p.geometry.z*10-90)<.001})
            write('floor_revision_top_vertices_'+str(i),vertices)
            holes=[]
            for f in old.faces:
                g=f.geometry
                if not isinstance(g,adsk.core.Cylinder) or abs(g.axis.z)<.99 or g.radius*10>2 or g.origin.z*10<80:continue
                x,y=g.origin.x*10,g.origin.y*10;r=g.radius*10
                ring=[p for p in vertices if abs(((p[0]-x)**2+(p[1]-y)**2)**.5-r)<.015]
                holes.append(dict(expected_center_mm=[x,y,90],radius_mm=r,ring_points_mm=ring))
                if len(ring)<12:
                    write('floor_revision_hole_mismatch',dict(part=o.name,center_mm=[x,y,90],matched=len(ring),top_vertices_mm=vertices))
                    raise RuntimeError('Motor hole mismatch in supplied STEP: '+o.name+' '+str([x,y]))
            if len(holes)!=6:raise RuntimeError('Expected six upper motor holes')
            report['parts'].append(dict(name=o.name,bounds_local_mm=bb(temp),motor_holes=holes,transform_cm=o.transform2.asArray()))
            prepared.append((o,old,temp))
        write('floor_revision_alignment_verified',report)
        backup=os.path.join(OUT,'Assembly_Before_Floor_Revision.f3d')
        if not os.path.exists(backup) and not design.exportManager.execute(design.exportManager.createFusionArchiveExportOptions(backup)):raise RuntimeError('Backup failed')
        for o,old,temp in prepared:
            comp=o.component;appearance=old.appearance;material=old.material
            bf=next((f for f in comp.features.baseFeatures if f.name=='User_Floor_Bracket_249p95mm_20261020'),None)
            if not bf:
                bf=comp.features.baseFeatures.add();bf.name='User_Floor_Bracket_249p95mm_20261020';bf.startEdit()
                try:
                    new=comp.bRepBodies.add(temp,bf);new.name='Floor_Bracket_249p95mm';new.material=material;new.appearance=appearance
                finally:bf.finishEdit()
            bf.sourceBodies[0].material=material;old.isLightBulbOn=False
        design.computeAll();adsk.doEvents()
        write('floor_revision_committed',report)
        with open(os.path.join(OUT,'assembly_link_name_map.json'),encoding='utf8') as f:mapping=json.load(f)
        changed=[]
        lookup={r['old']:r for r in mapping}
        for o in root.occurrences:
            row=lookup.get(o.component.name)
            if not row:continue
            o.component.attributes.add('Waffle','canonical_name',row['canonical'])
            o.component.attributes.add('Waffle','urdf_link',row['link'])
            if row['new']!=o.component.name:
                changed.append(dict(old=o.component.name,new=row['new'],link=row['link']));o.component.name=row['new']
        for g in root.rigidGroups:
            if g.name.startswith('RMX52_Link_'):g.name='arm_link_'+g.name.split('_')[-1]+'_Rigid_Group'
        write('assembly_link_names_applied',changed)
        api['physics_inspect'](app)
    elif action=='floor_revision_gripper':
        if os.path.exists(os.path.join(OUT,'gripper_equivalent_joints.json')):raise RuntimeError('Gripper joints already created')
        with open(os.path.join(OUT,'assembly_link_name_map.json'),encoding='utf8') as f:mapping=json.load(f)
        lookup={r['canonical']:next(o for o in root.occurrences if o.component.name==r['new']) for r in mapping if r['canonical'].startswith('RMX52_')}
        motor=lookup['RMX52_DC12_STD_2']
        sides=[['RMX52_IGP-A01_OPEN_1','RMX52_PR33_SPD-A01_1','RMX52_WBH_M3X10_3'],['RMX52_IGP-A01_OPEN_2','RMX52_PR33_SPD-A01_2','RMX52_WBH_M3X10_4']]
        moving=[lookup[n] for names in sides for n in names];tokens={o.fullPathName for o in moving}
        old=next(g for g in root.rigidGroups if g.name=='arm_link_5_Rigid_Group')
        for ml in list(root.motionLinks):
            if ml.name=='Gripper_Symmetric_Opening_1to1':ml.deleteMe()
        for j in list(root.asBuiltJoints):
            if j.name in ['gripper_left_joint','gripper_right_joint']:j.deleteMe()
        for g in list(root.rigidGroups):
            if g.name in ['arm_link_5_Gripper_Body_Rigid_Group','gripper_left_link_Rigid_Group','gripper_right_link_Rigid_Group']:g.deleteMe()
        fixed=[o for o in old.occurrences if not any(o.component==m.component for m in moving)]
        write('gripper_group_preflight',dict(original=len(list(old.occurrences)),fixed=len(fixed),moving=[o.name for o in moving],fixed_members=[o.name for o in fixed]))
        if len(fixed)!=52:raise RuntimeError('Finger group membership needs review')
        old.isSuppressed=False
        replacement=adsk.core.ObjectCollection.create()
        for o in fixed:replacement.add(o)
        old.timelineObject.rollTo(True)
        try:
            if not old.setOccurrences(replacement,False):raise RuntimeError('Cannot remove fingers from original rigid group')
        finally:design.timeline.moveToEnd()
        design.computeAll();adsk.doEvents()
        def group(name,members):
            c=adsk.core.ObjectCollection.create()
            for o in members:c.add(o)
            g=root.rigidGroups.add(c,False);g.name=name
            return g
        joints=[];origins=[]
        for index,names in enumerate(sides):
            members=[lookup[n] for n in names];finger=members[0]
            group(['gripper_left_link','gripper_right_link'][index]+'_Rigid_Group',members)
            point=finger.transform2.translation
            vertex=finger.component.originConstructionPoint.createForAssemblyContext(finger)
            ji=root.asBuiltJoints.createInput(motor,finger,adsk.fusion.JointGeometry.createByPoint(vertex))
            ji.setAsSliderJointMotion(adsk.fusion.JointDirections.CustomJointDirection,root.yConstructionAxis)
            j=root.asBuiltJoints.add(ji);j.name=['gripper_left_joint','gripper_right_joint'][index];j.isLightBulbOn=False
            joints.append(j);origins.append(xyz(adsk.core.Point3D.create(point.x,point.y,point.z)))
        mgr=adsk.fusion.TemporaryBRepManager.get();pads=[]
        for name in ['RMX52_PR33_SPD-A01_1','RMX52_PR33_SPD-A01_2']:
            o=lookup[name];b=mgr.copy(o.component.bRepBodies.item(0));mgr.transform(b,o.transform2);pads.append(b.boundingBox)
        gap=(pads[0].minPoint.y-pads[1].maxPoint.y)*10
        low=(16.8-gap)/2;high=(72.0-gap)/2
        for index,j in enumerate(joints):
            limit=j.jointMotion.slideLimits
            limit.minimumValue=low/10;limit.maximumValue=high/10
            limit.isMinimumValueEnabled=True;limit.isMaximumValueEnabled=True
        li=root.motionLinks.createInput(joints[0],joints[1]);li.valueOne=adsk.core.ValueInput.createByString('1 mm');li.valueTwo=adsk.core.ValueInput.createByString('1 mm');li.isReversed=False
        ml=root.motionLinks.add(li);ml.name='Gripper_Symmetric_Opening_1to1'
        initial=[o.transform2.asArray() for o in moving];q=[j.jointMotion.slideValue for j in joints]
        tests=[]
        try:
            joints[0].jointMotion.slideValue=q[0]+.1;app.activeViewport.refresh();adsk.doEvents()
            tests=[dict(name=o.name,delta_mm=[(o.transform2.asArray()[k]-t[k])*10 for k in [3,7,11]]) for o,t in zip(moving,initial)]
            if not all(abs(t['delta_mm'][1]-(1 if i<3 else -1))<.01 and abs(t['delta_mm'][0])<.01 and abs(t['delta_mm'][2])<.01 for i,t in enumerate(tests)):raise RuntimeError('Gripper slider directions need correction: '+str(tests))
        finally:
            joints[0].jointMotion.slideValue=q[0];app.activeViewport.refresh();adsk.doEvents()
        error=max(abs(a-b) for o,t in zip(moving,initial) for a,b in zip(o.transform2.asArray(),t))
        if error>1e-5:raise RuntimeError('Gripper home restoration failed')
        write('gripper_equivalent_joints',dict(status='equivalent_parallel_finger_pair_not_internal_linkage_simulation',joints=[j.name for j in joints],origin_world_mm=origins,axis_world=[0,1,0],mimic_multiplier=-1,tests=tests,restoration_error=error,aperture_limits_mm=[16.8,72.0],aperture_at_home_mm=gap,slide_limits_mm=[[low,high],[-high,-low]],limits_status='user-specified aperture bounds applied; servo-angle-to-aperture transmission remains unverified',motor='RMX52_Motor_J6_Gripper_Open_Close',limitation='Internal crank/linkage parts remain in the gripper body; only the two fingers, pads and end pivot screws articulate. This is an equivalent URDF-style parallel-gripper model, not a reconstructed closed-loop mechanism.'))
        api['physics_inspect'](app)
    elif action=='floor_revision_axischeck':
        roll=next(j for j in root.asBuiltJoints if j.name=='RMX52_Wrist_Roll_Reversed')
        left=next(j for j in root.asBuiltJoints if j.name=='gripper_left_joint')
        motor=next(o for o in root.occurrences if 'RMX52_Motor_J6_Gripper_Open_Close' in o.component.name)
        fingers=[next(o for o in root.occurrences if 'IGP-A01_OPEN_'+str(i) in o.component.name) for i in [1,2]]
        saved_roll=roll.jointMotion.rotationValue;saved_slide=left.jointMotion.slideValue
        original=[o.transform2.asArray() for o in fingers];initial_motor=motor.transform2.copy();inv=initial_motor.copy();inv.invert()
        direction=adsk.core.Vector3D.create(0,1,0);direction.transformBy(inv)
        result={}
        try:
            roll.jointMotion.rotationValue=saved_roll+.1;app.activeViewport.refresh();adsk.doEvents()
            before=[o.transform2.asArray() for o in fingers]
            expected=direction.copy();expected.transformBy(motor.transform2)
            left.jointMotion.slideValue=saved_slide+.1;app.activeViewport.refresh();adsk.doEvents()
            delta=[[(o.transform2.asArray()[k]-t[k])*10 for k in [3,7,11]] for o,t in zip(fingers,before)]
            error=max(abs(delta[i][j]-expected.asArray()[j]*(1 if i==0 else -1)) for i in range(2) for j in range(3))
            result.update(expected_direction_world=expected.asArray(),actual_displacement_mm=delta,maximum_error_mm=error)
            if error>.01:raise RuntimeError('Gripper axis does not follow wrist: '+str(result))
        finally:
            left.jointMotion.slideValue=saved_slide;roll.jointMotion.rotationValue=saved_roll;app.activeViewport.refresh();adsk.doEvents()
            result['restoration_error']=max(abs(a-b)for o,t in zip(fingers,original)for a,b in zip(o.transform2.asArray(),t))
            write('gripper_parent_axis_validation',result)
    elif action=='floor_revision_finish':
        # A newly added second base feature can inherit the component's steel
        # default on recompute. Set the printed component and its owned source,
        # then the result bodies, rather than leaving a result-only override.
        material=design.materials.itemByName('Waffle_PLA_Printed_1240_1240.0')
        if not material:raise RuntimeError('Printed PLA material missing')
        repaired=[]
        for o in root.occurrences:
            if not o.component.name.startswith('Chassis_Bracket_'):continue
            comp=o.component;comp.material=material
            bf=next(f for f in comp.features.baseFeatures if f.name=='User_Floor_Bracket_249p95mm_20261020')
            bf.startEdit()
            try:
                for b in bf.sourceBodies:b.material=material
            finally:bf.finishEdit()
            for b in comp.bRepBodies:
                if b.isLightBulbOn:b.material=material
            repaired.append(dict(component=comp.name,component_material=comp.material.name,source_materials=[b.material.name for b in bf.sourceBodies],result_materials=[b.material.name for b in bf.bodies]))
        write('floor_revision_material_repair',repaired)
        api['persist_materials'](app)
        with open(os.path.join(OUT,'body_physics_geometry.json'),encoding='utf8') as f:actual=json.load(f)
        with open(os.path.join(OUT,'material_assignment_plan.json'),encoding='utf8') as f:plan=json.load(f)
        expected={(r['path'],r['body_name']):r['expected_body_mass_g'] for r in plan}
        errors=[]
        for r in actual:
            target=expected.get((r['path'],r['body']))
            if target is None and r['volume_mm3']>1e-8:errors.append(dict(path=r['path'],reason='missing target'))
            elif target is not None and abs(r['default_mass_g']-target)>max(.002,target*.001):errors.append(dict(path=r['path'],actual_mass_g=r['default_mass_g'],expected_mass_g=target))
        write('material_persistence_verified',dict(revision='Floor249p95',total_visible_mass_g=sum(r['default_mass_g'] for r in actual),mass_errors=errors))
        if errors:raise RuntimeError('Updated native mass consistency failed')
        import importlib.util
        spec=importlib.util.spec_from_file_location('FloorRevisionPlayer',os.path.join(api['HERE'],'WafflePlayerV2.py'))
        player=importlib.util.module_from_spec(spec);spec.loader.exec_module(player)
        d,joints,frames=player.prepare(app)
        player.play(app,d,joints,frames)
        api['portable_export'](app)
        source(app);app.activeViewport.saveAsImageFile(os.path.join(OUT,'Assembly_Current_20261020.png'),1400,1000)
        with open(os.path.join(api['HERE'],'player_status.json'),encoding='utf8') as f:play=json.load(f)
        with open(os.path.join(OUT,'portable_step_validation.json'),encoding='utf8') as f:step=json.load(f)
        write('floor_revision_final_validation',dict(revision='Floor249p95',native_mass_g=sum(r['default_mass_g'] for r in actual),native_material_mass_errors=errors,playback=play,step=step,limitations='Sampled camera collision checks belong to the previous unchanged camera/rail geometry. No new swept-volume or real hardware grasp validation was performed.'))
    else:raise RuntimeError('Unknown floor revision action: '+action)

