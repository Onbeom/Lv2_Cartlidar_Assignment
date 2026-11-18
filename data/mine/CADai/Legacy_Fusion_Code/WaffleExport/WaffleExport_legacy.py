import adsk.core, adsk.fusion
import json, os, traceback

HERE = os.path.dirname(os.path.abspath(__file__))

def render_parts(app):
    with open(os.path.join(HERE, 'export_complete.json'), encoding='utf-8') as f:
        names = {e['name'] for e in json.load(f)}
    candidates = []
    for doc in app.documents:
        design = adsk.fusion.Design.cast(doc.products.itemByProductType('DesignProductType'))
        if design and {o.component.name for o in design.rootComponent.occurrences} == names:
            candidates.append((doc,design))
    if len(candidates) != 1:
        raise RuntimeError('Could not identify the seven-part export document.')
    doc, design = candidates[0]
    doc.activate()
    app.userInterface.activeSelections.clear()
    output = os.path.join(os.path.dirname(HERE), 'Waffle_STEP_20261016', 'images')
    os.makedirs(output, exist_ok=True)
    occurrences = list(design.rootComponent.occurrences)
    original_visibility = [o.isLightBulbOn for o in occurrences]
    original_camera = app.activeViewport.camera
    try:
        for target in occurrences:
            for occurrence in occurrences:
                occurrence.isLightBulbOn = occurrence == target
            adsk.doEvents()
            camera = app.activeViewport.camera
            camera.viewOrientation = adsk.core.ViewOrientations.IsoTopRightViewOrientation
            camera.isFitView = True
            camera.isSmoothTransition = False
            app.activeViewport.camera = camera
            app.activeViewport.refresh()
            adsk.doEvents()
            path = os.path.join(output, target.component.name + '.png')
            if not app.activeViewport.saveAsImageFile(path, 1200, 900):
                raise RuntimeError('Could not save ' + path)
    finally:
        for occurrence, visible in zip(occurrences, original_visibility):
            occurrence.isLightBulbOn = visible
        app.activeViewport.camera = original_camera
        app.activeViewport.refresh()

def safe_volume(body):
    try:
        return body.volume
    except:
        return None

def validate_exports(app):
    with open(os.path.join(HERE, 'export_complete.json'), encoding='utf-8') as f:
        exports = json.load(f)
    return_doc = app.activeDocument
    checks = []
    for entry in exports:
        options = app.importManager.createSTEPImportOptions(entry['file'])
        imported = app.importManager.importToNewDocument(options)
        if not imported:
            raise RuntimeError('Could not reimport ' + entry['name'])
        design = adsk.fusion.Design.cast(app.activeProduct)
        bodies = list(design.rootComponent.bRepBodies)
        for occurrence in design.rootComponent.allOccurrences:
            bodies.extend(list(occurrence.bRepBodies))
        solids = sum(b.isSolid for b in bodies)
        box = design.rootComponent.boundingBox
        checks.append(dict(name=entry['name'], imported_bodies=len(bodies),
            imported_solids=solids,
            imported_surfaces=len(bodies)-solids,
            min_cm=[box.minPoint.x,box.minPoint.y,box.minPoint.z],
            max_cm=[box.maxPoint.x,box.maxPoint.y,box.maxPoint.z],
            volumes_cm3=[safe_volume(b) for b in bodies]))
        if entry['name'].startswith('Floor_') and (len(bodies)!=1 or solids!=1):
            raise RuntimeError('Floor reimport is not a single solid')
        imported.close(False)
    return_doc.activate()
    with open(os.path.join(HERE, 'validation.json'), 'w', encoding='utf-8') as f:
        json.dump(checks, f, ensure_ascii=False, indent=2)

def export_requested(app, source):
    config_path = os.path.join(HERE, 'export_config.json')
    if not os.path.exists(config_path):
        return
    with open(config_path, encoding='utf-8') as f:
        config = json.load(f)
    if os.path.exists(os.path.join(HERE, 'export_complete.json')):
        raise RuntimeError('Export already completed; refusing to overwrite files.')
    source_doc = app.activeDocument
    lookup = {o.fullPathName:o for o in source.rootComponent.allOccurrences}
    manager = adsk.fusion.TemporaryBRepManager.get()
    groups = []
    for group in config['groups']:
        bodies = []
        for path in group['paths']:
            occurrence = lookup[path]
            for body in occurrence.component.bRepBodies:
                temporary = manager.copy(body)
                if not manager.transform(temporary, occurrence.transform2):
                    raise RuntimeError('Could not position ' + path)
                bodies.append(temporary)
        if not bodies:
            raise RuntimeError('Empty group: ' + group['name'])
        volumes = [safe_volume(b) for b in bodies]
        input_volume = sum(v for v in volumes if v is not None)
        if group['join']:
            target = bodies[0]
            for body in bodies[1:]:
                if not manager.booleanOperation(target, body, adsk.fusion.BooleanTypes.UnionBooleanType):
                    raise RuntimeError('Floor union failed: ' + group['name'])
            bodies = [target]
            if target.lumps.count != 1:
                raise RuntimeError('Floor is not one connected solid: ' + group['name'] + ', lumps=' + str(target.lumps.count))
        groups.append((group, bodies, input_volume))
    output = config['output']
    os.makedirs(output, exist_ok=True)
    doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    result_design = adsk.fusion.Design.cast(app.activeProduct)
    result_design.designType = adsk.fusion.DesignTypes.DirectDesignType
    root = result_design.rootComponent
    results = []
    for group, bodies, input_volume in groups:
        occurrence = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        component = occurrence.component
        component.name = group['name']
        for index, body in enumerate(bodies):
            created = component.bRepBodies.add(body)
            created.name = group['name'] + '_' + str(index+1)
        filename = os.path.join(output, group['name'] + '.step')
        if os.path.exists(filename):
            raise RuntimeError('File already exists: ' + filename)
        options = result_design.exportManager.createSTEPExportOptions(filename, component)
        if not result_design.exportManager.execute(options):
            raise RuntimeError('STEP export failed: ' + filename)
        results.append(dict(name=group['name'], paths=group['paths'], file=filename,
            source_body_count=sum(lookup[p].component.bRepBodies.count for p in group['paths']),
            exported_bodies=component.bRepBodies.count,
            solid_lumps=sum(b.lumps.count for b in component.bRepBodies),
            input_volume_cm3=input_volume, volumes_cm3=[safe_volume(b) for b in component.bRepBodies],
            all_solid=all(b.isSolid for b in component.bRepBodies)))
    archive = os.path.join(output, 'Waffle_Requested_Parts.f3d')
    if not result_design.exportManager.execute(result_design.exportManager.createFusionArchiveExportOptions(archive)):
        raise RuntimeError('Archive export failed')
    app.activeViewport.fit()
    with open(os.path.join(HERE, 'export_complete.json'), 'w', encoding='utf-8') as f:
        json.dump(results, f, ensure_ascii=False, indent=2)

def run(context):
    app = adsk.core.Application.get()
    try:
        if os.path.exists(os.path.join(HERE, 'assembly_request.json')):
            import importlib.util
            spec = importlib.util.spec_from_file_location('WaffleAssemblyBuild', os.path.join(HERE, 'AssemblyBuild.py'))
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            module.run(context)
            return
        if os.path.exists(os.path.join(HERE, 'insert_request.json')):
            import importlib.util
            spec = importlib.util.spec_from_file_location('WaffleBatchInsert', os.path.join(HERE, 'BatchInsert.py'))
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            module.run(context)
            return
        if os.path.exists(os.path.join(HERE, 'validation.json')):
            render_parts(app)
            return
        if os.path.exists(os.path.join(HERE, 'export_complete.json')):
            validate_exports(app)
            return
        if os.path.exists(os.path.join(HERE, 'export_config.json')):
            candidates = [d for d in app.documents if d.name == 'WAFFLE_PI_ASSY (2)']
            if len(candidates) != 1:
                raise RuntimeError('Expected exactly one original Waffle document.')
            candidates[0].activate()
        design = adsk.fusion.Design.cast(app.activeProduct)
        if not design:
            raise RuntimeError('Open the Waffle assembly first.')
        records = []
        for occ in design.rootComponent.allOccurrences:
            box = occ.boundingBox
            records.append(dict(path=occ.fullPathName, name=occ.component.name,
                bodies=occ.component.bRepBodies.count,
                min=[box.minPoint.x,box.minPoint.y,box.minPoint.z],
                max=[box.maxPoint.x,box.maxPoint.y,box.maxPoint.z],
                matrix=occ.transform2.asArray()))
        with open(os.path.join(HERE, 'inspection.json'), 'w', encoding='utf-8') as f:
            json.dump(dict(document=app.activeDocument.name, occurrences=records), f, ensure_ascii=False, indent=2)
        export_requested(app, design)
    except:
        with open(os.path.join(HERE, 'error.txt'), 'w', encoding='utf-8') as f:
            f.write(traceback.format_exc())
        app.userInterface.messageBox(traceback.format_exc())
