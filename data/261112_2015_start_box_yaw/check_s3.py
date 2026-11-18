"""S3 판정: start_box.enabled false 인 mock 기록 (새 mission) 과 바꾸기 전 mission (HEAD archive 빌드) 기록이 같은지.

  python3 check_s3.py <s3_new 폴더> <s3_base 폴더>
  같은 열: navigate · align · drive_skill · pose · pick · skill · set_mode · check_dice · place_box, 최종 상태.
  덧붙여 계획 줄 (planner mission_lines.txt) 의 skill 이름 열과 보낸 이름 열도 비교 (place_<L>_* -> place_at 은 #73 기존 동작이라 따로 셈).
"""
import glob
import json
import os
import sys

new, base = (json.load(open(os.path.join(d, "mock_out.json"), encoding="utf-8")) for d in sys.argv[1:3])
keys = ["navigate", "align", "drive_skill", "pose", "pick", "skill", "set_mode", "check_dice", "place_box"]
res = {k: (new[k] == base[k], len(new[k]), len(base[k])) for k in keys}
res["final"] = (new["final"] == base["final"], new["final"].get("state"), base["final"].get("state"))
lines = glob.glob(os.path.join(sys.argv[1], "log", "planner_run", "mission_lines.txt")) or glob.glob(
    os.path.join(sys.argv[1], "log", "*", "planner", "mission_lines.txt"))
plan = []
if lines:
    for l in open(lines[0], encoding="utf-8"):
        t = l.split()
        if len(t) >= 3 and t[0] == "skill":
            plan.append(t[2])
sent = [e["name"] for e in new["events"] if e["kind"] in ("skill", "drive_skill")]
sent_plan = [n for n in sent if not n.startswith(("barcode_look:", "box_look:"))]
same = [a == b or (a.startswith("place_L") and b.startswith("place_at:")) for a, b in zip(plan, sent_plan)]
print(json.dumps(dict(same_as_base=all(v[0] for v in res.values()), detail=res, plan_lines=len(plan), sent_from_plan=len(sent_plan),
                      plan_vs_sent_equal_except_place_at=len(plan) == len(sent_plan) and all(same),
                      place_at=sum(1 for a, b in zip(plan, sent_plan) if a != b)), ensure_ascii=False))
