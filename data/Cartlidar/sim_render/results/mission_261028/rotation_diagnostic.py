"""도구 확인 전용: 회전 후보를 훑으면 기존 zbar 읽기가 회복되는지 확인. 공식 R3′ 판정에 섞지 않음."""
from pathlib import Path
import json
import cv2
import numpy as np
root=Path(__file__).resolve().parent
source=root.parents[1]/'proto_r3/zbar_check.py'
ns={};exec(source.read_text().split('root = Path(sys.argv[1])')[0],ns)
render=root/'render_r3_gpu'
meta=json.loads((render/'render_r3.json').read_text())
rows=[]
for p in meta['poses']:
    g=ns['LUT'][cv2.cvtColor(cv2.imread(str(render/p['images']['color'])),cv2.COLOR_BGR2GRAY)]
    hits=[]
    for angle in range(-45,46,5):
        matrix=cv2.getRotationMatrix2D((g.shape[1]/2,g.shape[0]/2),angle,1)
        rotated=cv2.warpAffine(g,matrix,(g.shape[1],g.shape[0]),borderValue=255)
        syms=ns['read'](rotated)
        if syms:hits.append({'angle_deg':angle,'symbols':syms})
    rows.append({'name':p['name'],'truth':p['text'],'roll_actual_deg':p['roll_actual_deg'],'hits':hits})
(root/'rotation_diagnostic.json').write_text(json.dumps(rows,indent=1,ensure_ascii=False))
print(json.dumps(rows,indent=1,ensure_ascii=False))
