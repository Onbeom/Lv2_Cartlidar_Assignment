import adsk.core, adsk.fusion, os, json, traceback

OUT=os.path.join(os.path.dirname(os.path.dirname(__file__)), 'Waffle_Assembly_Build','BracketAudit1019')
def p(v): return [round(v.x*10,6),round(v.y*10,6),round(v.z*10,6)]
def box(b):return [p(b.minPoint),p(b.maxPoint)]
def info(b,detail):
    row={'name':b.name,'visible':b.isVisible,'solid':b.isSolid,'bounds_mm':box(b.boundingBox),'volume_mm3':b.volume*1000,'area_mm2':b.area*100,'faces':b.faces.count,'edges':b.edges.count,'cylinders':[],'planes':[]}
    for f in b.faces:
        g=f.geometry
        if isinstance(g,adsk.core.Cylinder):row['cylinders'].append({'origin_mm':p(g.origin),'axis':[g.axis.x,g.axis.y,g.axis.z],'radius_mm':g.radius*10,'bounds_mm':box(f.boundingBox)})
        if detail and isinstance(g,adsk.core.Plane):
            loops=[]
            for loop in f.loops:
                loops.append({'outer':loop.isOuter,'edges':[{'type':e.geometry.objectType,'length_mm':e.length*10,'bounds_mm':box(e.boundingBox),'vertices':[p(v.geometry) for v in [e.startVertex,e.endVertex] if v]} for e in loop.edges]})
            row['planes'].append({'normal':[g.normal.x,g.normal.y,g.normal.z],'center_mm':p(f.centroid),'area_mm2':f.area*100,'bounds_mm':box(f.boundingBox),'loops':loops})
    return row
def run(app):
    os.makedirs(OUT,exist_ok=True)
    report={'active_document':app.activeDocument.name,'documents':[]}
    for doc in app.documents:
        design=adsk.fusion.Design.cast(doc.products.itemByProductType('DesignProductType'))
        if not design:continue
        detail='braket' in doc.name.lower()
        rd={'name':doc.name,'bodies':[]}
        for b in design.rootComponent.bRepBodies:rd['bodies'].append(dict(path='ROOT',**info(b,detail)))
        for occ in design.rootComponent.allOccurrences:
            if not detail and not any(s in occ.fullPathName.lower() for s in ['bracket','braket','socket','support']):continue
            for b in occ.component.bRepBodies:rd['bodies'].append(dict(path=occ.fullPathName,**info(b,detail)))
        report['documents'].append(rd)
    with open(os.path.join(OUT,'audit.json'),'w',encoding='utf-8') as f:json.dump(report,f,ensure_ascii=False,indent=2)

def fit(app,design):
    if 'braket_1016' not in app.activeDocument.name:raise RuntimeError('Activate braket_1016')
    manager=adsk.fusion.TemporaryBRepManager.get()
    rows=[]
    for body in design.rootComponent.bRepBodies:
        if body.boundingBox.maxPoint.z<10:continue
        # Centres read from the opposed inner planar walls, in cm.
        cx=-9.1500006 if body.boundingBox.minPoint.x<-5 else -1.2500007
        cy=26.8500004
        row={'body':body.name,'center_mm':[cx*10,cy*10],'sections':[],'point_grid':[]}
        row['full_length_tests']=[]
        for width in [20,20.4]:
            ob=adsk.core.OrientedBoundingBox3D.create(adsk.core.Point3D.create(cx,cy,23.2),adsk.core.Vector3D.create(1,0,0),adsk.core.Vector3D.create(0,1,0),width/10,width/10,45)
            prism=manager.createBox(ob);cut=manager.copy(body)
            ok=manager.booleanOperation(cut,prism,adsk.fusion.BooleanTypes.IntersectionBooleanType)
            row['full_length_tests'].append({'width_mm':width,'bottom_z_mm':7,'length_mm':450,'success':ok,'overlap_mm3':cut.volume*1000 if ok else None})
        for z in [3,4,5,6,7,10,22,24,85,87,88,89,90,91,92,93,94,95,100,109]:
            ob=adsk.core.OrientedBoundingBox3D.create(adsk.core.Point3D.create(cx,cy,(z+.05)/10),adsk.core.Vector3D.create(1,0,0),adsk.core.Vector3D.create(0,1,0),2,2,.01)
            prism=manager.createBox(ob)
            cut=manager.copy(body)
            ok=manager.booleanOperation(cut,prism,adsk.fusion.BooleanTypes.IntersectionBooleanType)
            row['sections'].append({'z_mm':z,'slice_thickness_mm':.1,'success':ok,'overlap_mm3':cut.volume*1000 if ok else None})
            adsk.doEvents()
        for z in [3.5,5,6.5,7.5,87.5,89.5,90.5,92,93.5,94.5]:
            grid=[]
            for y in [-9.9,-5,0,5,9.9]:
                grid.append([int(body.pointContainment(adsk.core.Point3D.create(cx+dx/10,cy+y/10,z/10))) for dx in [-9.9,-5,0,5,9.9]])
            row['point_grid'].append({'z':z,'grid':grid})
        rows.append(row)
    with open(os.path.join(OUT,'fit.json'),'w',encoding='utf-8') as f:json.dump(rows,f,ensure_ascii=False,indent=2)

