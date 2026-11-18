"""R3 판정: restand false 인 새 mission 과 #247 빌드 (startbox worktree) 의 같은 mock 기록이 같은지.

  python3 check_r3.py <새 폴더> <#247 폴더>
  같은 열: navigate · align · drive_skill · pose · pick · skill · set_mode · check_dice · place_box, 최종 상태.
"""
import json
import os
import sys

new, base = (json.load(open(os.path.join(d, "mock_out.json"), encoding="utf-8")) for d in sys.argv[1:3])
keys = ["navigate", "align", "drive_skill", "pose", "pick", "skill", "set_mode", "check_dice", "place_box"]
res = {k: (new[k] == base[k], len(new[k]), len(base[k])) for k in keys}
res["final"] = (new["final"] == base["final"], new["final"].get("state"), base["final"].get("state"))
print(json.dumps(dict(same=all(v[0] for v in res.values()), detail=res), ensure_ascii=False))
