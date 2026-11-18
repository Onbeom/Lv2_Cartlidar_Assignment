"""261027_1711 재검사 결과 집계: 판정 범위(검증계획_261027_1711.md 3장)에 따라 판정 / 기록으로 나눠 센다(모의).
사용(PC): python3 tools/tally.py
"""
import glob
import json
import re
from collections import defaultdict
from pathlib import Path

W = Path(__file__).resolve().parents[1]
RECORD_TESTS = {"V1", "V1b", "V6", "V6b", "V7", "P7"}


def scope(test, name):
    """판정이면 True, 기록이면 False."""
    if test in RECORD_TESTS:
        if test == "V6" and "_mu" in name:      # V9 안의 V6: 간격만 판정(아래에서 따로 봄)
            return True
        return False
    if test in ("V4", "V6cb", "V8") and re.search(r"_r35", name):
        return False
    return True


tab = defaultdict(lambda: {"판정 성공": 0, "판정 실패": [], "기록 성공": 0, "기록 실패": []})
for f in sorted(glob.glob(str(W / "phys_tests/logs/v*_*.txt"))):
    for line in Path(f).read_text().splitlines():
        m = re.match(r"(MuJoCo|Isaac Sim) (\S+) (성공|실패)\s*(.*?)\s*(\S+)$", line.strip())
        if not m:
            continue
        eng, test, verdict, why, name = m.groups()
        key = f"{test} ({'M' if eng == 'MuJoCo' else 'I'})"
        judged = scope(test, name)
        if test == "V6" and "_mu" in name and verdict == "실패" and "간격" not in why:
            judged = False                       # V9의 V6: 간격 말고 다른 항목 실패는 기록
        k = ("판정 " if judged else "기록 ") + verdict
        if verdict == "성공":
            tab[key][k] += 1
        else:
            tab[key][k].append(f"{name}: {why}")
# P1~P8 (Isaac)
for f in sorted(glob.glob(str(W / "phys_tests/results/P*_v12_*/verdict.json"))):
    name = Path(f).parent.name
    test = name.split("_")[0]
    d = json.loads(Path(f).read_text())
    items = [("P6", d["P6"]["verdict"]), ("P8", d["P8"]["verdict"])] if "P6" in d else [(test, d["verdict"])]
    for t, v in items:
        judged = t != "P7"
        k = ("판정 " if judged else "기록 ") + v
        if v == "성공":
            tab[f"{t} (I)"][k] += 1
        else:
            tab[f"{t} (I)"][k].append(name)
for k in sorted(tab):
    v = tab[k]
    print(f"{k:12s} 판정 성공 {v['판정 성공']:3d} 실패 {len(v['판정 실패']):2d} | 기록 성공 {v['기록 성공']:3d} 실패 {len(v['기록 실패']):2d}"
          + (f"  판정 실패: {v['판정 실패']}" if v["판정 실패"] else ""))
(W / "tally.json").write_text(json.dumps(tab, ensure_ascii=False, indent=1))
