import adsk.core, adsk.fusion
import collections, datetime, hashlib, json, math, os

OUT=os.path.join(os.path.dirname(os.path.dirname(__file__)),'Waffle_Assembly_Build','LiveAudit1019')
REFERENCE=os.path.join(os.path.dirname(OUT),'AssemblyURDF1019','Waffle_URDF_Ready.f3d')

def write(name,value):
    os.makedirs(OUT,exist_ok=True)
    with open(os.path.join(OUT,name+'.json'),'w',encoding='utf-8') as f:json.dump(value,f,ensure_ascii=False,indent=2)

def xyz(p):return [p.x*10,p.y*10,p.z*10]

def bounds(b):
    bb=b.boundingBox
    lo,hi=xyz(bb.minPoint),xyz(bb.maxPoint)
    return {'min_mm':lo,'max_mm':hi,'size_mm':[b-a for a,b in zip(lo,hi)]}

def geometry(design,h,label):
    mgr=adsk.fusion.TemporaryBRepManager.get();rows=[];bodies=[]
    def walk(comp,t,path):
        for index,b in enumerate(comp.bRepBodies):
            if not b.isLightBulbOn:continue
            temp=mgr.copy(b);mgr.transform(temp,t);bb=bounds(temp)
            try:volume=temp.volume*1000 if temp.isSolid else None
            except:volume=None
            try:area=temp.area*100
            except:area=None
            row={'path':path,'top':path.split('+')[0],'body':b.name,'index':index,'bounds':bb,
                 'solid':b.isSolid,'faces':b.faces.count,'edges':b.edges.count,'vertices':b.vertices.count,
                 'volume_mm3':volume,'area_mm2':area}
            signature=[row['solid'],row['faces'],row['edges'],row['vertices'],
                       [round(v,4) for v in bb['min_mm']+bb['max_mm']],round(volume,3) if volume is not None else None,round(area,3) if area is not None else None]
            row['signature']=hashlib.sha256(json.dumps(signature).encode()).hexdigest()
            rows.append(row);bodies.append((row,temp,b))
        for o in comp.occurrences:
            if o.isLightBulbOn:walk(o.component,h['multiply'](t,o.transform2),path+'+'+o.name if path else o.name)
    walk(design.rootComponent,adsk.core.Matrix3D.create(),'')
    write(label+'_bodies',rows)
    components=[]
    for top in sorted(set(r['top'] for r in rows)):
        rs=[r for r in rows if r['top']==top]
        lo=[min(r['bounds']['min_mm'][i] for r in rs) for i in range(3)]
        hi=[max(r['bounds']['max_mm'][i] for r in rs) for i in range(3)]
        components.append({'name':top,'body_count':len(rs),'min_mm':lo,'max_mm':hi,'size_mm':[b-a for a,b in zip(lo,hi)],
                           'center_mm':[(a+b)/2 for a,b in zip(lo,hi)]})
    joints=[]
    for j in design.rootComponent.asBuiltJoints:
        m=j.jointMotion;row={'name':j.name,'suppressed':j.isSuppressed,'type':m.objectType,
                            'one':j.occurrenceOne.fullPathName,'two':j.occurrenceTwo.fullPathName}
        if isinstance(m,adsk.fusion.RevoluteJointMotion):
            row['value_rad']=m.rotationValue;v=m.rotationAxisVector;row['axis']=[v.x,v.y,v.z]
        elif isinstance(m,adsk.fusion.SliderJointMotion):row['value_mm']=m.slideValue*10
        joints.append(row)
    result={'document':design.parentDocument.name,'bodies':rows,'components':components,'joints':joints,
            'root_occurrences':[{'name':o.name,'visible':o.isVisible,'bulb':o.isLightBulbOn} for o in design.rootComponent.occurrences]}
    write(label+'_inventory',result)
    return result,bodies

def sockets(design):
    results=[]
    for side in ['Left','Right']:
        o=next(o for o in design.rootComponent.occurrences if o.component.name=='Chassis_Bracket_'+side+'_16mm')
        b=next(b for b in o.component.bRepBodies if b.isLightBulbOn)
        cylinders=[];planes=[]
        for f in b.faces:
            g=f.geometry
            if isinstance(g,adsk.core.Cylinder):
                cylinders.append({'diameter_mm':g.radius*20,'origin_mm':xyz(g.origin),'axis':[g.axis.x,g.axis.y,g.axis.z],'bounds':bounds(f)})
            elif isinstance(g,adsk.core.Plane):
                bb=f.boundingBox
                if bb.maxPoint.y<25 or bb.maxPoint.z<.3:continue
                planes.append({'origin_mm':xyz(g.origin),'normal':[g.normal.x,g.normal.y,g.normal.z],'bounds':bounds(f)})
        motors=[c for c in cylinders if abs(c['diameter_mm']-2.05)<.01]
        ys=sorted(set(round(c['origin_mm'][1],4) for c in motors))
        pitch=[ys[i+1]-ys[i] for i in range(0,len(ys)-1,2)]
        # Extract inner/outer faces of each rectangular socket from the actual boundary faces.
        levels=[]
        for label,zlo,zhi in [('lower',3,23),('upper',87,110)]:
            axes={0:[],1:[]}
            for p in planes:
                bb=p['bounds'];z0,z1=bb['min_mm'][2],bb['max_mm'][2]
                if z0<zlo-.02 or z1>zhi+.02 or z1-z0<10:continue
                for i in [0,1]:
                    if abs(p['normal'][i])>.999 and abs(bb['size_mm'][i])<.01:
                        value=(bb['min_mm'][i]+bb['max_mm'][i])/2
                        if i==0 or value>250:axes[i].append(round(value,4))
            values={str(i):sorted(set(v)) for i,v in axes.items()}
            levels.append({'name':label,'planar_wall_coordinates_mm':values})
        results.append({'side':side,'motor_hole_count':len(motors),'motor_hole_diameter_mm':2.05,
                        'motor_hole_y_mm':ys,'pair_pitch_mm':pitch,'m5_holes':[c for c in cylinders if abs(c['diameter_mm']-5.5)<.01],
                        'all_cylinders':cylinders,'socket_levels':levels,'native_bounds':bounds(b)})
    return results

def exportparts(app,design):
    if app.activeDocument.name!='assembly':raise RuntimeError('Activate working assembly')
    rows=[]
    for name in ['Chassis_Bracket_Left_16mm','Chassis_Bracket_Right_16mm','Manipulator_Support_Cyan','Manipulator_Support_Coral']:
        o=next(o for o in design.rootComponent.occurrences if o.component.name==name)
        b=next(b for b in o.component.bRepBodies if b.isLightBulbOn)
        path=os.path.join(OUT,name+'.stl')
        options=design.exportManager.createSTLExportOptions(b,path)
        options.unitType=adsk.fusion.DistanceUnits.MillimeterDistanceUnits
        options.meshRefinement=adsk.fusion.MeshRefinementSettings.MeshRefinementHigh;options.isBinaryFormat=True
        if not design.exportManager.execute(options):raise RuntimeError('STL export failed: '+name)
        rows.append({'component':name,'file':path,'native_bounds':bounds(b),'volume_mm3':b.volume*1000})
    write('comparison_parts',rows)

def run(app,design,h):
    if app.activeDocument.name!='assembly':raise RuntimeError('Activate the working assembly for audit')
    doc=app.activeDocument;original_camera=app.activeViewport.camera
    report={'timestamp':datetime.datetime.now().isoformat(),'reference':REFERENCE,'stage':'reading current geometry'}
    write('progress',report)
    live,live_bodies=geometry(design,h,'current');report['stage']='reading archived final geometry';write('progress',report)
    reference_doc=None
    try:
        reference_doc=app.importManager.importToNewDocument(app.importManager.createFusionArchiveImportOptions(REFERENCE))
        rd=adsk.fusion.Design.cast(app.activeProduct)
        ref,_=geometry(rd,h,'reference')
    finally:
        if reference_doc:reference_doc.close(False)
        doc.activate();app.activeViewport.camera=original_camera
    # The independent export-ready archive contains the same visible geometry,
    # regrouped into twelve links; compare world-space geometry, not occurrence names.
    a=collections.Counter(r['signature'] for r in live['bodies'])
    b=collections.Counter(r['signature'] for r in ref['bodies'])
    missing=list((b-a).elements());extra=list((a-b).elements())
    report.update({'current_visible_body_count':len(live['bodies']),'reference_visible_body_count':len(ref['bodies']),
                   'matching_visible_geometry':not missing and not extra,'missing_signatures':missing,'extra_signatures':extra,
                   'components':live['components'],'brackets':sockets(design),'current_joints':live['joints'],'reference_joints':ref['joints']})
    write('dimensions',report)
    report['stage']='checking bracket, profile, rail, and deck interference';write('progress',report)
    mgr=adsk.fusion.TemporaryBRepManager.get();hits=[];tested=0
    relevant_prefixes=('Chassis_Bracket_Left_16mm','Chassis_Bracket_Right_16mm','Profile_2020_','rail_hi','rail_lo','LDS_Front_Underside','LDS_Rear_Underside')
    chosen=[i for i,(row,t,source) in enumerate(live_bodies) if row['top'].startswith(relevant_prefixes)]
    pairs=set()
    for i in chosen:
        row,a,source=live_bodies[i]
        for k,(other,b,other_source) in enumerate(live_bodies):
            if row['top']==other['top'] or not a.isSolid or not b.isSolid:continue
            key=tuple(sorted((i,k)))
            if key in pairs:continue
            pairs.add(key)
            aa=a.boundingBox;bb=b.boundingBox
            if any(getattr(aa.maxPoint,v)<=getattr(bb.minPoint,v)+1e-5 or getattr(bb.maxPoint,v)<=getattr(aa.minPoint,v)+1e-5 for v in ['x','y','z']):continue
            test=mgr.copy(a);ok=mgr.booleanOperation(test,b,adsk.fusion.BooleanTypes.IntersectionBooleanType);tested+=1
            if not ok:raise RuntimeError('Interference boolean failed for '+row['path']+' / '+other['path'])
            if test.volume*1000>.1:hits.append({'part':row['path'],'other':other['path'],'volume_mm3':test.volume*1000})
        report['tested_candidate_pairs']=tested;report['interferences']=hits;write('progress',report);adsk.doEvents()
    report['stage']='complete';write('audit',report);write('progress',report)
