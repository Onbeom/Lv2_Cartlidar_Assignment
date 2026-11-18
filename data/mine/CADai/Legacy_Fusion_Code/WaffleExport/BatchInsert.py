"""Insert the explicitly requested local STEP files into the active Fusion design."""
import adsk.core
import adsk.fusion
import datetime
import hashlib
import json
import os
import traceback

HERE = os.path.dirname(os.path.abspath(__file__))
GROUP = 'CodexLocalSTEPImport'

def run(context):
    app = adsk.core.Application.get()
    log = {'status': 'starting', 'inserted': [], 'skipped': [], 'errors': []}
    log_path = os.path.join(HERE, 'insert_progress.json')

    def record():
        with open(log_path, 'w', encoding='utf-8') as stream:
            json.dump(log, stream, ensure_ascii=False, indent=2)

    try:
        with open(os.path.join(HERE, 'insert_request.json'), encoding='utf-8') as stream:
            request = json.load(stream)
        design = adsk.fusion.Design.cast(app.activeProduct)
        if not design:
            raise RuntimeError('Activate the destination Fusion assembly before running WaffleExport.')
        destination_doc = app.activeDocument
        root = design.rootComponent
        log['document'] = destination_doc.name
        existing = {o.component.name: o for o in root.occurrences}
        log['existing_components'] = list(existing)
        record()

        # Preflight every requested source before modifying the assembly.
        sources = []
        hashes = set()
        for item in request['files']:
            path = item['path']
            if os.path.splitext(path)[1].lower() not in ('.step', '.stp'):
                raise RuntimeError('Only STEP files are allowed: ' + path)
            with open(path, 'rb') as stream:
                fingerprint = hashlib.sha256(stream.read()).hexdigest()
            if fingerprint in hashes:
                log['skipped'].append({'file':path, 'reason':'duplicate STEP content'})
                continue
            hashes.add(fingerprint)
            sources.append((item, fingerprint))

        timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        output = request['output']
        os.makedirs(output, exist_ok=True)
        backup = os.path.join(output, 'Assembly_before_insert_' + timestamp + '.f3d')
        if not design.exportManager.execute(design.exportManager.createFusionArchiveExportOptions(backup)):
            raise RuntimeError('Could not create the pre-insert local backup.')
        log['backup'] = backup
        log['status'] = 'importing'
        record()
        app.userInterface.activeSelections.clear()

        for item, fingerprint in sources:
            path, name = item['path'], item['name']
            tagged = next((o for o in root.occurrences
                if o.component.attributes.itemByName(GROUP, fingerprint)), None)
            if tagged:
                log['skipped'].append({'file':path, 'reason':'previously imported', 'component':tagged.component.name})
                record()
                continue
            # The seven Waffle components may already be present in the active export document.
            if item['waffle_part'] and name in existing:
                log['skipped'].append({'file':path, 'reason':'Waffle component already exists', 'component':name})
                record()
                continue
            occurrence = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
            component = occurrence.component
            component.name = name
            options = app.importManager.createSTEPImportOptions(path)
            options.isViewFit = False
            log['current_file'] = path
            record()
            if not app.importManager.importToTarget(options, component):
                raise RuntimeError('Import failed: ' + path)
            count = component.bRepBodies.count
            for child in component.allOccurrences:
                count += child.component.bRepBodies.count
            if count == 0:
                raise RuntimeError('Imported file contains no BRep bodies: ' + path)
            component.attributes.add(GROUP, fingerprint, path)
            log['inserted'].append({'file':path, 'component':component.name, 'bodies':count})
            record()
            adsk.doEvents()

        destination_doc.activate()
        app.activeViewport.fit()
        app.activeViewport.refresh()
        archive = os.path.join(output, 'Assembly_with_local_STEP_' + timestamp + '.f3d')
        if not design.exportManager.execute(design.exportManager.createFusionArchiveExportOptions(archive)):
            raise RuntimeError('Parts were inserted but the final local archive could not be saved.')
        log['archive'] = archive
        log['status'] = 'complete'
        record()
        app.userInterface.messageBox('STEP import complete.\nAdded: %d\nSkipped: %d\nSTL: 0\n\nLocal backup:\n%s' %
            (len(log['inserted']), len(log['skipped']), archive))
    except:
        log['status'] = 'error'
        log['errors'].append(traceback.format_exc())
        record()
        app.userInterface.messageBox(traceback.format_exc())
