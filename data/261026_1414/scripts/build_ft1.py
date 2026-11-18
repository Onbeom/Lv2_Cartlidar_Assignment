"""실물 미세 조정 시험 1(ft1) 자료 묶기 (PC, 사용자 승인 2026-10-26).

묶음 단위 나눔: 학습 r01 · r02 · r05 · r07(실물 58장 × 4 복사), 검증 r03, 평가 r04 · r06.
학습에는 시뮬레이션 학습 자료(data/images/train 3600장) 중 464장을 seed 0 으로 무작위로 골라 섞는다(실물 232 : 시뮬레이션 464 = 1 : 2).
파일은 복사하지 않고 심볼릭 링크로 만든다. 결과: 261026_1414/data_ft1/{images,labels}/{train,val,eval}, split.json

실행: ~/yolo-venv/bin/python 261026_1414/scripts/build_ft1.py
"""
import json
import random
from pathlib import Path

B = Path(__file__).resolve().parents[1]
REAL, SIM, OUT = B / "real_labels", B / "data", B / "data_ft1"
SPLIT = {"train": ["r01", "r02", "r05", "r07"], "val": ["r03"], "eval": ["r04", "r06"]}
REAL_REPEAT, SIM_N, SEED = 4, 464, 0

if OUT.exists():
    raise SystemExit(f"{OUT} 이 이미 있음")
entries = json.loads((REAL / "auto_label.json").read_text())
if not all(e.get("checked") for e in entries):
    raise SystemExit("아직 사람이 다 보지 않은 장이 있음")


def link(src_img, src_lbl, split, name):
    for kind, src, ext in (("images", src_img, src_img.suffix), ("labels", src_lbl, ".txt")):
        d = OUT / kind / split
        d.mkdir(parents=True, exist_ok=True)
        (d / f"{name}{ext}").symlink_to(src)


record = {k: [] for k in SPLIT}
for e in entries:
    split = next((k for k, bs in SPLIT.items() if e["batch"][:3] in bs), None)
    if split is None:
        continue
    img = REAL / "images" / e["image"]
    lbl = REAL / "labels" / (img.stem + ".txt")
    reps = REAL_REPEAT if split == "train" else 1
    for r in range(reps):
        link(img, lbl, split, f"real_{img.stem}" + (f"_rep{r}" if reps > 1 else ""))
    record[split].append(e["image"])

sims = sorted((SIM / "images" / "train").glob("*.png"))
random.Random(SEED).shuffle(sims)
for img in sims[:SIM_N]:
    link(img, SIM / "labels" / "train" / (img.stem + ".txt"), "train", f"sim_{img.stem}")
record["sim_train"] = [p.name for p in sims[:SIM_N]]
record["setting"] = {"split": SPLIT, "real_repeat": REAL_REPEAT, "sim_n": SIM_N, "seed": SEED}
(OUT / "split.json").write_text(json.dumps(record, ensure_ascii=False, indent=1))
for k in ("train", "val", "eval"):
    print(k, len(list((OUT / "images" / k).iterdir())), "장 (실물", len(record[k]), "장)")
