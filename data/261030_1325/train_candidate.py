"""고정 PC 후보 학습. 실행 중 이 파일을 변경하지 않는다."""
import argparse
import json
import os
from pathlib import Path

os.environ['YOLO_OFFLINE']='true'
os.environ['MPLCONFIGDIR']='/tmp/lv2_plot_config'
from ultralytics import YOLO
import torch

p=argparse.ArgumentParser()
p.add_argument('variant',choices=['A_positive','B_negative'])
a=p.parse_args()
root=Path('/home/pa4/git/lv2_next_module')
here=Path(__file__).resolve().parent
out=root/'Cartlidar/sim_render/results/followup_261028_1457/improvement_261030_1325'
assert torch.cuda.is_available(), 'PC GPU 접근 불가'
model=YOLO(str(root/'261029_1408/models/ft1b_legacy6.pt'))
model.train(data=str(here/a.variant/'data.yaml'),epochs=80,batch=16,imgsz=640,
            patience=0,workers=2,seed=0,deterministic=True,device='0',
            amp=False,project=str(out/'runs'),name=a.variant,exist_ok=False,plots=True)
(out/(a.variant+'_finished.json')).write_text(json.dumps({'variant':a.variant,'epochs':80,
    'checkpoint_policy':'last.pt 고정, eval 결과로 선택 안함','amp':False},ensure_ascii=False,indent=2))
