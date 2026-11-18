"""truth.json → puck_tool 입력 목록 (K · 카메라 자세만, 정답 퍽은 넣지 않음)"""
import json, sys
from pathlib import Path
d = Path(sys.argv[1]).resolve(); J = json.loads((d / 'truth.json').read_text()); K = J['K']
for r in J['images']:
    v = [x for row in r['T_bf_cam'][:3] for x in row]
    print(r['id'], d / r['image'], K[0][0], K[1][1], K[0][2], K[1][2], *v)
