import adsk.core,adsk.fusion,os,math

def run(app,api):
    source=api['source'](app);source_doc=app.activeDocument;report={'files':[]}
    for name,expected in [('Assembly_Modular_261021',669),('Assembly_Modular_261021_Geometry_Only',669),('Floor_Bracket_Modular_261021',4),('Printable_Modular_261021/Floor_Bracket_Print_Layout_261021',4)]:
        path=os.path.join(api['CAD_OUT'],name+'.f3d');doc=None
        try:
            doc=app.importManager.importToNewDocument(app.importManager.createFusionArchiveImportOptions(path));d=adsk.fusion.Design.cast(app.activeProduct)
            rows=[]
            for p,temp,native in api['world_bodies'](d.rootComponent):
                prop=native.getPhysicalProperties(adsk.fusion.CalculationAccuracy.MediumCalculationAccuracy)
                rows.append(dict(path=p,body=native.name,solid=native.isSolid,bounds_mm=api['bb'](temp),volume_mm3=prop.volume*1000,mass_g=prop.mass*1000,material=native.material.name))
            report['files'].append(dict(file=path,reopened=True,body_count=len(rows),expected_body_count=expected,count_matches=len(rows)==expected,top_components=d.rootComponent.occurrences.count,joints=d.rootComponent.asBuiltJoints.count,external_refs=[o.fullPathName for o in d.rootComponent.allOccurrences if o.isReferencedComponent],mass_g=sum(r['mass_g']for r in rows),bodies=rows))
            if 'Print_Layout' in name:
                lo=[min(r['bounds_mm']['min'][i]for r in rows)for i in range(3)];hi=[max(r['bounds_mm']['max'][i]for r in rows)for i in range(3)]
                report['files'][-1].update(layout_size_mm=[hi[i]-lo[i]for i in range(3)],bed_256_fits=all(hi[i]-lo[i]<256 for i in [0,1]))
        except Exception as error:report['files'].append(dict(file=path,reopened=False,error=str(error)))
        finally:
            if doc:doc.close(False)
            source_doc.activate()
            api['write']('modular_review2_reopen',report);adsk.doEvents()
