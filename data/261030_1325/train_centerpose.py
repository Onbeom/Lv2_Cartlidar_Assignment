"""TAO 공식 구성요소를 별도 PyTorch 루프로 학습. 기존 환경·원본 소스 불변."""
import argparse
import hashlib
import json
import random
import time
from pathlib import Path
import numpy as np
import torch
from torch.utils.data import DataLoader,Subset
from tao_dla_adapter import make_model,dataset_config,loss_config,ObjectPoseDataset,ObjectPoseLoss

p=argparse.ArgumentParser();p.add_argument('category',choices=['cylinder','cuboid']);a=p.parse_args()
HERE=Path(__file__).resolve().parent
BASE=HERE.parent/'Cartlidar/sim_render/results/followup_261028_1457/improvement_261030_1325'
OUT=BASE/('centerpose_train_'+a.category);OUT.mkdir(exist_ok=False)
torch.manual_seed(0);np.random.seed(0);random.seed(0);torch.set_num_threads(4)
torch.backends.cudnn.benchmark=False
torch.use_deterministic_algorithms(True,warn_only=True)
symmetry=12 if a.category=='cylinder' else 4
config=dataset_config(a.category,symmetry)
train=ObjectPoseDataset(str(BASE/'centerpose_data'/a.category/'train'),config,'train')
val=ObjectPoseDataset(str(BASE/'centerpose_data'/a.category/'val'),config,'val')
indices=random.Random(0).sample(range(len(train)),min(480,len(train)))
val_indices=random.Random(1).sample(range(len(val)),min(80,len(val)))
loader=DataLoader(Subset(train,indices),batch_size=2,shuffle=True,num_workers=2,pin_memory=True,
                  generator=torch.Generator().manual_seed(0))
validation=DataLoader(Subset(val,val_indices),batch_size=2,shuffle=False,num_workers=2)
model,metadata=make_model();model.cuda();criterion=ObjectPoseLoss(loss_config()).cuda()
optimizer=torch.optim.Adam(model.parameters(),lr=6e-5)
scaler=torch.amp.GradScaler('cuda')
metadata.update({'category':a.category,'epochs':30,'batch':2,'learning_rate':6e-5,'symmetry':symmetry,
                 'train_indices':indices,'val_indices':val_indices,'train_files':[train.images[i][0] for i in indices],
                 'validation_files':[val.images[i][0] for i in val_indices],
                 'script_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest()})
(OUT/'manifest.json').write_text(json.dumps(metadata,ensure_ascii=False,indent=2))
start=time.perf_counter();rows=[]
for epoch in range(30):
    model.train();losses=[]
    for batch in loader:
        batch={k:v.cuda(non_blocking=True) for k,v in batch.items() if isinstance(v,torch.Tensor)}
        optimizer.zero_grad(set_to_none=True)
        with torch.autocast('cuda',dtype=torch.float16):loss=criterion(model(batch['input']),batch,'train')
        if not torch.isfinite(loss):raise RuntimeError('비유한 학습 loss')
        scaler.scale(loss).backward();scaler.unscale_(optimizer)
        torch.nn.utils.clip_grad_norm_(model.parameters(),100.)
        scaler.step(optimizer);scaler.update();losses.append(float(loss.detach()))
    model.eval();vlosses=[]
    with torch.no_grad():
        for batch in validation:
            batch={k:v.cuda() for k,v in batch.items() if isinstance(v,torch.Tensor)}
            with torch.autocast('cuda',dtype=torch.float16):loss=criterion(model(batch['input']),batch,'val')
            if not torch.isfinite(loss):raise RuntimeError('비유한 검증 loss')
            vlosses.append(float(loss))
    row={'epoch':epoch+1,'train_loss':float(np.mean(losses)),'val_loss':float(np.mean(vlosses)),
         'elapsed_s':time.perf_counter()-start,'peak_allocated_bytes':torch.cuda.max_memory_allocated()}
    rows.append(row);(OUT/'learning.json').write_text(json.dumps(rows,ensure_ascii=False,indent=2))
    torch.save({'state_dict':model.state_dict(),'epoch':epoch+1,'metadata':metadata},OUT/'last.pt')
    print(json.dumps(row),flush=True)
(OUT/'finished.json').write_text(json.dumps({'epochs':30,'elapsed_s':time.perf_counter()-start},indent=2))
