"""E3 가중치(261104_1618_E3/runs/E3/weights/last.pt)를 D435용 480×640 · Arducam용 384×640 정적 ONNX 로 내보낸다(11-04). 설정은 export_d.py 와 같음. PC CPU."""
import hashlib, os, shutil
from pathlib import Path
os.environ['YOLO_OFFLINE'] = 'true'
import onnx, torch
from ultralytics import YOLO
HERE = Path(__file__).resolve().parent; src = Path('/home/pa4/git/lv2_next_module/261104_1618_E3/runs/E3/weights/last.pt')
SHA = '4b0d809b'
torch.set_num_threads(2)
for h in (480, 384):
    pt = HERE / f'yolo11n_task4_E3_{h}.pt'; shutil.copy2(src, pt)
    assert hashlib.sha256(pt.read_bytes()).hexdigest().startswith(SHA)
    YOLO(str(pt)).export(format='onnx', imgsz=[h, 640], batch=1, dynamic=False, simplify=False, opset=17, nms=False, device='cpu', half=False)
    o = pt.with_suffix('.onnx'); g = onnx.load(str(o)); onnx.checker.check_model(g)
    dims = lambda t: [d.dim_value for d in t.type.tensor_type.shape.dim]
    print(o.name, '입력', dims(g.graph.input[0]), '출력', dims(g.graph.output[0]), 'sha256', hashlib.sha256(o.read_bytes()).hexdigest())
    pt.unlink()
