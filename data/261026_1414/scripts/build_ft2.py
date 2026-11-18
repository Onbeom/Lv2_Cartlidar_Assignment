"""실물 미세 조정 ft2(4클래스) 자료 묶기 (PC, 사용자 승인 2026-10-27: ft1 과 같은 방식, 4클래스, ft1 계열 가중치에서 시작).

- 실물 라벨 폴더 여러 개(--real, 각 폴더에 auto_label.json · images/ · labels/ · classes.txt)를 4클래스로 맞춰 묶는다.
  6클래스 라벨은 puck_classes.REMAP_6_TO_4 로 바꿔 새 파일로 쓰고(원본 그대로), 4클래스 라벨은 링크한다. 사진은 링크.
- 나눔(묶음 단위, 같은 묶음은 한 곳에만):
  - 시작 가중치(ft1b)가 이미 배운 묶음이 평가 · 검증에 들어가지 않게, --keep 의 split.json(기본 data_ft1)에 있는 묶음은 그 나눔을 그대로 쓴다.
  - 나머지(새 사진) 묶음은 seed 순서로 섞어 평가 ≥ 20 % · 검증 ≥ 10 %(장 수 기준)가 될 때까지 채우고 나머지는 학습.
    학습 · 평가 · 검증에 빨강 · 초록 라벨 퍽이 모두 들어가는 첫 seed(0 부터)를 쓴다. 100 번 안에 없으면 멈춘다.
- 학습 = 실물 학습 장 × --repeat + 시뮬레이션 학습 자료(4클래스로 바꿈)를 실물의 --sim-ratio 배(seed 0 무작위).
- 묶음 이름이 폴더끼리 겹치거나, 사람이 다 보지 않은 장이 있으면 멈춘다. 새 사진은 auto_label.py --batch 를 r08 부터 이어 붙인다.
결과: <out>/{images,labels}/{train,val,eval}, <out>/classes.txt(4클래스), <out>/split.json

실행: ~/yolo-venv/bin/python 261026_1414/scripts/build_ft2.py --real 261026_1414/real_labels --real <새 4클래스 라벨 폴더> --out 261026_1414/data_ft2
"""
import argparse
import json
import random
import sys
from collections import Counter
from pathlib import Path

B = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(B.parents[0] / "Cartlidar" / "yolo"))
import puck_classes as pc  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--real", type=Path, action="append", required=True)
ap.add_argument("--out", type=Path, default=B / "data_ft2")
ap.add_argument("--keep", type=Path, default=B / "data_ft1" / "split.json", help="나눔을 그대로 둘 묶음(ft1b 가 배운 자료)")
ap.add_argument("--sim", type=Path, default=B / "data")
ap.add_argument("--repeat", type=int, default=4)
ap.add_argument("--sim-ratio", type=float, default=2.0)
ap.add_argument("--eval-frac", type=float, default=0.2)
ap.add_argument("--val-frac", type=float, default=0.1)
ap.add_argument("--seed", type=int, default=0, help="시뮬레이션 고르기")
ap.add_argument("--dry", action="store_true", help="나눔만 보여 주고 파일은 안 만듦(도구 확인용)")
args = ap.parse_args()

if args.out.exists() and not args.dry:
    raise SystemExit(f"{args.out} 이 이미 있음")


def load_labels(path, names):
    lines = [ln for ln in path.read_text().split("\n") if ln.strip()]
    return pc.remap_lines(lines) if names == pc.NAMES6 else lines


# 실물 장 모으기: (묶음, 사진 경로, 4클래스 라벨 줄, 원본 라벨 경로 또는 None)
items, owner = [], {}
for d in args.real:
    names = pc.read_names(d / "classes.txt")
    if names not in (pc.NAMES4, pc.NAMES6):
        raise SystemExit(f"{d}/classes.txt 가 4 · 6클래스 이름과 다름: {names}")
    entries = json.loads((d / "auto_label.json").read_text())
    if not all(e.get("checked") for e in entries):
        raise SystemExit(f"{d}: 아직 사람이 다 보지 않은 장이 있음")
    for e in entries:
        b = e["batch"][:3]
        if owner.setdefault(b, d) != d:
            raise SystemExit(f"묶음 {b} 이 {owner[b]} 와 {d} 에 모두 있음 → 새 사진 묶음 번호를 바꿀 것")
        img = d / "images" / e["image"]
        lbl = d / "labels" / (img.stem + ".txt")
        items.append((b, img, load_labels(lbl, names), lbl if names == pc.NAMES4 else None))

batches = sorted({b for b, *_ in items})
n_img = Counter(b for b, *_ in items)
colors = {b: Counter() for b in batches}
for b, _, lines, _ in items:
    for ln in lines:
        colors[b][pc.split(pc.NAMES4[int(ln.split()[0])])[0]] += 1

keep = json.loads(args.keep.read_text())["setting"]["split"] if args.keep and args.keep.exists() else {}
fixed = {b: s for s, bs in keep.items() for b in bs if b in n_img}
new = [b for b in batches if b not in fixed]


def assign(seed):
    order = sorted(new)
    random.Random(seed).shuffle(order)
    total = sum(n_img[b] for b in new)
    split, got = dict(fixed), Counter()
    for b in order:
        if got["eval"] < args.eval_frac * total:
            s = "eval"
        elif got["val"] < args.val_frac * total:
            s = "val"
        else:
            s = "train"
        split[b] = s
        got[s] += n_img[b]
    return split


def has_both(split, s):
    c = sum((colors[b] for b in batches if split[b] == s), Counter())
    return c["red"] > 0 and c["green"] > 0


if new:
    split_seed = next((s for s in range(100) if (sp := assign(s)) and has_both(sp, "train") and has_both(sp, "eval") and has_both(sp, "val")), None)
    if split_seed is None:
        raise SystemExit(f"학습 · 평가 · 검증에 빨강 · 초록이 모두 들어가는 나눔이 없음(새 묶음 {new}) → 묶음을 더 나눠 찍거나 사용자와 정함")
    split = assign(split_seed)
else:
    split_seed, split = None, dict(fixed)

summary = {s: {"batches": [b for b in batches if split[b] == s], "images": sum(n_img[b] for b in batches if split[b] == s),
               "pucks": dict(sum((colors[b] for b in batches if split[b] == s), Counter()))} for s in ("train", "val", "eval")}
real_train = summary["train"]["images"] * args.repeat
sim_n = round(real_train * args.sim_ratio)
print(json.dumps({"fixed_from_keep": fixed, "new": new, "split_seed": split_seed, **summary,
                  "train_total": {"real_x_repeat": real_train, "sim": sim_n}}, ensure_ascii=False, indent=1))
if args.dry:
    raise SystemExit(0)


def put(kind, split_name, name, src=None, text=None):
    d = args.out / kind / split_name
    d.mkdir(parents=True, exist_ok=True)
    if text is not None:
        (d / name).write_text("\n".join(text) + ("\n" if text else ""))
    else:
        (d / name).symlink_to(src.resolve())


record = {s: [] for s in ("train", "val", "eval")}
for b, img, lines, lbl in items:
    s = split[b]
    reps = args.repeat if s == "train" else 1
    for r in range(reps):
        stem = f"real_{img.stem}" + (f"_rep{r}" if reps > 1 else "")
        put("images", s, stem + img.suffix, src=img)
        if lbl is not None:
            put("labels", s, stem + ".txt", src=lbl)
        else:
            put("labels", s, stem + ".txt", text=lines)
    record[s].append(img.name)

sims = sorted((args.sim / "images" / "train").glob("*.png"))
random.Random(args.seed).shuffle(sims)
if sim_n > len(sims):
    raise SystemExit(f"시뮬레이션 학습 자료 {len(sims)}장 < 필요 {sim_n}장")
for img in sims[:sim_n]:
    put("images", "train", f"sim_{img.stem}.png", src=img)
    put("labels", "train", f"sim_{img.stem}.txt",
        text=pc.remap_lines([ln for ln in (args.sim / "labels" / "train" / (img.stem + ".txt")).read_text().split("\n")]))
(args.out / "classes.txt").write_text("\n".join(pc.NAMES4) + "\n")
record["sim_train"] = [p.name for p in sims[:sim_n]]
record["setting"] = {"split": {s: summary[s]["batches"] for s in ("train", "val", "eval")}, "fixed_from_keep": fixed,
                     "split_seed": split_seed, "real_dirs": [str(d) for d in args.real], "real_repeat": args.repeat,
                     "sim_ratio": args.sim_ratio, "sim_n": sim_n, "seed": args.seed, "pucks": {s: summary[s]["pucks"] for s in summary}}
(args.out / "split.json").write_text(json.dumps(record, ensure_ascii=False, indent=1))
for k in ("train", "val", "eval"):
    print(k, len(list((args.out / "images" / k).iterdir())), "장 (실물", len(record[k]), "장)")
