"""Ultralytics 학습 기록(results.csv)을 TensorBoard 기록으로 바꾼다. 다시 학습하지 않는다.
사용: csv_to_tb.py <출력 logdir> <이름=results.csv 경로>...
예:   csv_to_tb.py logs E3=../261104_1618_E3/runs/E3/results.csv E2=../261103_1724_E2/runs/E2/results.csv
값은 epoch 별이고, 그래프 가로축(step)은 epoch 번호다. 같은 이름의 폴더가 있으면 지우고 다시 만든다."""
import csv, shutil, sys
from pathlib import Path
from torch.utils.tensorboard import SummaryWriter

out = Path(sys.argv[1])
for arg in sys.argv[2:]:
    name, path = arg.split('=', 1)
    rows = list(csv.DictReader(open(path)))
    d = out / name
    if d.exists():
        shutil.rmtree(d)
    w = SummaryWriter(str(d))
    for r in rows:
        r = {k.strip(): v.strip() for k, v in r.items()}
        ep = int(float(r['epoch']))
        for k, v in r.items():
            if k in ('epoch', 'time') or not v:
                continue
            w.add_scalar(k, float(v), ep)  # 이름 예: metrics/recall(B), train/cls_loss, val/box_loss, lr/pg0
    w.close()
    print(f'{name}: epoch {len(rows)} → {d}')
