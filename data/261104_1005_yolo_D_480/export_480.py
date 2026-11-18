"""D 가중치(yolo11n_task4_D.pt, sha 35f7af1a…)를 입력 480×640(높이×폭) 정적 ONNX 로 내보낸다(11-04). 그 밖의 설정은 export_d.py 와 같음. PC CPU."""
import hashlib, os, shutil
from pathlib import Path
os.environ['YOLO_OFFLINE'] = 'true'
import onnx, torch
from ultralytics import YOLO
HERE = Path(__file__).resolve().parent; src = Path('/home/pa4/git/lv2_next_module/261102_1634_yolo_D/yolo_export/yolo11n_task4_D.pt')
pt = HERE / 'yolo11n_task4_D_480.pt'; shutil.copy2(src, pt)
assert hashlib.sha256(pt.read_bytes()).hexdigest().startswith('35f7af1a')
torch.set_num_threads(2)
YOLO(str(pt)).export(format='onnx', imgsz=[480, 640], batch=1, dynamic=False, simplify=False, opset=17, nms=False, device='cpu', half=False)
o = pt.with_suffix('.onnx'); g = onnx.load(str(o)); onnx.checker.check_model(g)
dims = lambda t: [d.dim_value for d in t.type.tensor_type.shape.dim]
print('입력', g.graph.input[0].name, dims(g.graph.input[0]), '출력', g.graph.output[0].name, dims(g.graph.output[0]))
print('meta', {p.key: p.value[:60] for p in g.metadata_props})
print('onnx sha256', hashlib.sha256(o.read_bytes()).hexdigest())
pt.unlink()
