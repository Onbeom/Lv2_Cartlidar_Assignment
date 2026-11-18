"""Read-only dimensions of rail mounting faces in world coordinates (mm)."""
import adsk.core, adsk.fusion
import json, os

def xyz(p): return [p.x*10,p.y*10,p.z*10]
def bounds(e):
    b=e.boundingBox
    return {'min':xyz(b.minPoint),'max':xyz(b.maxPoint)}
def face(f):
    g=f.geometry
    d={'type':g.objectType,'bounds':bounds(f),'area':f.area*100,'edges':[]}
    if isinstance(g,adsk.core.Cylinder):
        d.update(origin=xyz(g.origin),axis=[g.axis.x,g.axis.y,g.axis.z],radius=g.radius*10)
    if isinstance(g,adsk.core.Plane):
        d.update(origin=xyz(g.origin),normal=[g.normal.x,g.normal.y,g.normal.z])
    for e in f.edges:
        c=e.geometry;row={'type':c.objectType,'bounds':bounds(e)}
        if isinstance(c,(adsk.core.Circle3D,adsk.core.Arc3D)):
            row.update(center=xyz(c.center),radius=c.radius*10)
        d['edges'].append(row)
    return d

def run(app):
    design=adsk.fusion.Design.cast(app.activeProduct)
    result={'document':app.activeDocument.name,'selected':[],'parts':[]}
    for s in app.userInterface.activeSelections:
        e=s.entity
        result['selected'].append(face(e) if isinstance(e,adsk.fusion.BRepFace) else {'type':e.objectType})
    mgr=adsk.fusion.TemporaryBRepManager.get()
    for o in design.rootComponent.allOccurrences:
        if not o.fullPathName.startswith(('rail_hi:','rail_lo:','Profile_2020_450mm_Rear_')):continue
        for b in o.component.bRepBodies:
            temp=mgr.copy(b);mgr.transform(temp,o.transform2)
            result['parts'].append({'path':o.fullPathName,'bounds':bounds(temp),'faces':[face(f) for f in temp.faces]})
    with open(os.path.join(os.path.dirname(__file__),'rail_fit_audit.json'),'w',encoding='utf-8') as f:
        json.dump(result,f,ensure_ascii=False,indent=2)
