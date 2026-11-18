import adsk.core,adsk.fusion,os,json,importlib.util,math

def run(app,api):
    d=api['source'](app);root=d.rootComponent;mgr=adsk.fusion.TemporaryBRepManager.get();lookup={o.name:o for o in root.occurrences};joints={j.name:j for j in root.asBuiltJoints};report={'wheels':[],'arm':[]}
    def largest_com(o):
        rows=[]
        def walk(c,m):
            for b in c.bRepBodies:
                p=b.getPhysicalProperties(adsk.fusion.CalculationAccuracy.MediumCalculationAccuracy)
                if p.volume<=0:continue
                pos=p.centerOfMass;pos.transformBy(m);rows.append((p.volume,pos))
            for child in c.occurrences:walk(child.component,api['mul'](m,child.transform2))
        walk(o.component,o.transform2);return max(rows,key=lambda r:r[0])[1]
    for end in ['Front','Rear']:
        for side in ['Left','Right']:
            o=lookup['Wheel_'+end+'_'+side+':1'];j=joints['Wheel_Revolute_'+end+'_'+side];saved=j.jointMotion.rotationValue;start=largest_com(o)
            try:
                j.jointMotion.rotationValue=saved+.2;adsk.doEvents();endp=largest_com(o)
                report['wheels'].append(dict(name=o.name,axle_center_drift_mm=start.distanceTo(endp)*10))
            finally:j.jointMotion.rotationValue=saved;adsk.doEvents()
    spec=importlib.util.spec_from_file_location('MotionMath',os.path.join(os.path.dirname(__file__),'Oct19.py'));motion=importlib.util.module_from_spec(spec);spec.loader.exec_module(motion)
    names=['Base_Yaw','Shoulder','Elbow','Wrist_Pitch','Wrist_Roll_Reversed'];arm=[joints['RMX52_'+n]for n in names];saved=[j.jointMotion.rotationValue for j in arm]
    report['native_joint_frames']=[dict(name=j.name,transform_cm=j.transform.asArray(),axis_vector=j.jointMotion.rotationAxisVector.asArray())for j in arm]
    tool=next(o for o in root.occurrences if o.name.startswith('arm_link_5__RMX52_Motor_J6_'));home=tool.transform2.copy()
    try:
        for q in [[.2,.3,-.2,.4,0],[0,0,0,0,.2]]:
            for j,v in zip(arm,q):j.jointMotion.rotationValue=v
            adsk.doEvents();applied=[j.jointMotion.rotationValue for j in arm];expected=api['mul'](motion.transforms(applied,{'multiply':api['mul']})[5],home).asArray();actual=tool.transform2.asArray()
            report['arm'].append(dict(q_rad=q,actual_joint_values_rad=[j.jointMotion.rotationValue for j in arm],home_transform_cm=home.asArray(),actual_transform_cm=actual,position_error_mm=max(abs(actual[i]-expected[i])*10 for i in [3,7,11]),rotation_matrix_error=max(abs(actual[i]-expected[i])for i in [0,1,2,4,5,6,8,9,10])))
    finally:
        for j,v in zip(arm,saved):j.jointMotion.rotationValue=v
        adsk.doEvents()
    report['all_pass']=all(x['axle_center_drift_mm']<.05 for x in report['wheels'])and all(x['position_error_mm']<.01 and x['rotation_matrix_error']<1e-5 for x in report['arm'])
    report['comparison']='Analytic FK compared with actual applied native joint values; Fusion rounds requested angles to 0.1 degree.'
    api['write']('modular_joint_kinematics',report)
