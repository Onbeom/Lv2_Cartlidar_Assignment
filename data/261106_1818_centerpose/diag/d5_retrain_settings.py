"""D5 첫 시도와 같은 전체 화면 512 입력 · 같은 480장으로 학습 설정만 바꿈(lr 5e-4, batch 8). 키포인트가 배워지는지 본다."""
import json, random, sys, time
from pathlib import Path
import numpy as np, torch
from torch.utils.data import DataLoader,Subset
ROOT=Path('/home/pa4/git/lv2_next_module');HERE=ROOT/'261030_1325';sys.path.insert(0,str(HERE));sys.path.insert(0,str(Path(__file__).resolve().parent))
from tao_dla_adapter import make_model,dataset_config,loss_config,ObjectPoseDataset,ObjectPoseLoss
import importlib.util
BASE=ROOT/'Cartlidar/sim_render/results/followup_261028_1457/improvement_261030_1325'
OUT=Path(__file__).resolve().parent/'d5';OUT.mkdir(exist_ok=True)
cat,sym=sys.argv[1],int(sys.argv[2]);lr=float(sys.argv[3]);bs=int(sys.argv[4]);ep=int(sys.argv[5])
torch.manual_seed(0);np.random.seed(0);random.seed(0);torch.set_num_threads(4)
src=(Path(__file__).resolve().parent/'d3_first_model.py').read_text().split("res={}")[0]
ns={'__file__':str(Path(__file__).resolve().parent/'d3_first_model.py')};exec(compile(src,'d3','exec'),ns)
cfg=dataset_config(cat,sym);train=ObjectPoseDataset(str(BASE/'centerpose_data'/cat/'train'),cfg,'train')
idx=random.Random(0).sample(range(len(train)),min(480,len(train)))
dl=DataLoader(Subset(train,idx),batch_size=bs,shuffle=True,num_workers=2,pin_memory=True,drop_last=True,generator=torch.Generator().manual_seed(0))
model,_=make_model();model.cuda();crit=ObjectPoseLoss(loss_config()).cuda();opt=torch.optim.Adam(model.parameters(),lr=lr);sc=torch.amp.GradScaler('cuda')
ev=sorted((BASE/'centerpose_data'/cat/'eval').glob('*.png'))[::2];tr=[Path(train.images[i][0]) for i in idx[:120]]
rows=[];t0=time.perf_counter()
for e in range(1,ep+1):
    model.train();L=[]
    for b in dl:
        b={k:v.cuda(non_blocking=True) for k,v in b.items() if isinstance(v,torch.Tensor)}
        opt.zero_grad(set_to_none=True)
        with torch.autocast('cuda',dtype=torch.float16):l=crit(model(b['input']),b,'train')
        sc.scale(l).backward();sc.unscale_(opt);torch.nn.utils.clip_grad_norm_(model.parameters(),100.);sc.step(opt);sc.update();L.append(float(l.detach()))
    r=dict(epoch=e,train_loss=float(np.mean(L)),elapsed_s=time.perf_counter()-t0)
    if e%10==0 or e==ep:
        model.eval();r['eval']=ns['summ'](ns['run'](model,cfg,ev,sym));r['train120']=ns['summ'](ns['run'](model,cfg,tr,sym))
    rows.append(r);print(json.dumps(r,ensure_ascii=False),flush=True);(OUT/f'{cat}_lr{lr}_b{bs}.json').write_text(json.dumps(rows,indent=1))
