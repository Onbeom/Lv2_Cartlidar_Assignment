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
    except Exception as error:
        with open(os.path.join(HERE, 'player_error.txt'), 'w', encoding='utf-8') as f:
            f.write(traceback.format_exc())
        app.userInterface.messageBox('WaffleExport 2 실행 실패:\n' + str(error))
