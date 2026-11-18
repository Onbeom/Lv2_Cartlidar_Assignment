"""배포 ONNX (robot_policy config/models/lieupright_r2g2.onnx) 에 r2g2 기록 관측을 넣어 기록 action_raw 와 비교 (도구)."""
import json, sys
from pathlib import Path
import numpy as np, onnxruntime as ort
m = sys.argv[1]
s = ort.InferenceSession(m, providers=["CPUExecutionProvider"])
i, o = s.get_inputs()[0], s.get_outputs()[0]
print("입력", i.name, i.shape, "출력", o.name, o.shape)
worst = 0.0
for f in sorted(Path("/home/pa4/git/lv2_next_module/261110_2010_lie_rl/handoff/r2g2_parity").glob("ep*.json")):
    st = json.loads(f.read_text())["episode"]["steps"]
    for k, x in enumerate(st):
        a = s.run(None, {i.name: np.array([x["obs"]], dtype=np.float32)})[0][0]
        worst = max(worst, float(np.abs(a - np.array(x["action_raw"])).max()))
    print(f.name, len(st), "스텝")
print("action_raw 최대 차이", worst)
