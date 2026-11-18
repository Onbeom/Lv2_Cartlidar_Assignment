"""Explicit playback entry point; old maintenance requests are opt-in."""
import importlib.util, json, os, traceback
import adsk.core
HERE = os.path.dirname(os.path.abspath(__file__))

def run(context):
    app = adsk.core.Application.get()
    try:
        request_path = os.path.join(HERE, 'assembly_request.json')
        request = {}
        if os.path.exists(request_path):
            with open(request_path, encoding='utf-8') as f:
                request = json.load(f)
        with open(os.path.join(HERE, 'last_run_status.json'), 'w', encoding='utf-8') as f:
            json.dump({'status':'running','phase':request.get('phase','player_v2'),'action':request.get('action')}, f)
        if request.get('phase') == 'project_1020':
            with open(request_path, 'w', encoding='utf-8') as f:
                json.dump({'phase':'player_v2', 'action':None}, f)
            spec = importlib.util.spec_from_file_location('Project1020', os.path.join(HERE, 'Project1020.py'))
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            module.run(app, request)
            with open(os.path.join(HERE, 'last_run_status.json'), 'w', encoding='utf-8') as f:
                json.dump({'status':'complete','phase':'project_1020','action':request.get('action')}, f)
            return
        if request.get('phase') == 'rail_fit_audit':
            with open(request_path, 'w', encoding='utf-8') as f:
                json.dump({'phase':'player_v2', 'action':None}, f)
            spec = importlib.util.spec_from_file_location('RailFitAudit', os.path.join(HERE, 'RailFitAudit.py'))
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            module.run(app)
            return
        legacy = request.get('maintenance') is True
        filename = 'WaffleExport_legacy.py' if legacy else 'WafflePlayerV2.py'
        spec = importlib.util.spec_from_file_location('WaffleEntryV2', os.path.join(HERE, filename))
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        if legacy:
            module.run(context)
        else:
            action = request.get('action') if request.get('phase') == 'player_v2' else None
            if action:
                with open(request_path, 'w', encoding='utf-8') as f:
                    json.dump({'phase':'player_v2', 'action':None}, f)
            module.run(app, action)
        with open(os.path.join(HERE, 'last_run_status.json'), 'w', encoding='utf-8') as f:
            json.dump({'status':'complete','phase':'player_v2','action':action if not legacy else 'maintenance'}, f)
    except Exception as error:
        with open(os.path.join(HERE, 'last_run_status.json'), 'w', encoding='utf-8') as f:
            json.dump({'status':'error','message':str(error)}, f, ensure_ascii=False)
        with open(os.path.join(HERE, 'player_error.txt'), 'w', encoding='utf-8') as f:
            f.write(traceback.format_exc())
        app.userInterface.messageBox('WaffleExport 2 실행 실패:\n' + str(error))
