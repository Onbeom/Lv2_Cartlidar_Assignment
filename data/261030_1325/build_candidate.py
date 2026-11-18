"""최종 ft2와 분리한 PC 비교자료. 원본 라벨/이미지는 수정하지 않는다."""
import hashlib
import json
import random
from collections import Counter
from pathlib import Path

import cv2
import numpy as np

ROOT=Path('/home/pa4/git/lv2_next_module')
HERE=Path(__file__).resolve().parent
OUT=ROOT/'Cartlidar/sim_render/results/followup_261028_1457/improvement_261030_1325'
NAMES=['red_cylinder','green_cylinder','red_cuboid','green_cuboid']
REMAP={0:0,1:1,3:2,4:3}
keep=json.loads((ROOT/'261026_1414/data_ft1/split.json').read_text())['setting']['split']
split={b:s for s,bs in keep.items() for b in bs}
split.update({b:'train' for b in ['r11','r13','r15','r16','r17']})
split.update({'r14':'eval','r18':'val','r19':'eval'})
records=[]
seen={}
for folder in ['real_labels','real_labels_261027']:
    base=ROOT/'261026_1414'/folder
    names=(base/'classes.txt').read_text().splitlines()
    for row in json.loads((base/'auto_label.json').read_text()):
        assert row.get('checked'), '미확인 라벨 사용 금지'
        b=row['batch'].split('_')[0]
        src=base/'images'/row['image']
        raw=(base/'labels'/(src.stem+'.txt')).read_text().splitlines()
        lines=[]
        for line in raw:
            if not line.strip():continue
            cid,*coords=line.split()
            cid=REMAP[int(cid)] if len(names)==6 else int(cid)
            assert 0<=cid<4
            lines.append(' '.join([str(cid),*coords]))
        h=hashlib.sha256(src.read_bytes()).hexdigest()
        assert seen.setdefault(h,split[b])==split[b], '내용해시 split 누출'
        records.append({'src':src,'split':split[b],'batch':b,'labels':lines,'sha256':h})
assert len(records)==391
OUT.mkdir(parents=True,exist_ok=True)
for variant in ['A_positive','B_negative']:
    dest=HERE/variant
    dest.mkdir(exist_ok=False)
    counts=Counter()
    index=[]
    def put(name,s,labels,src=None,pixels=None):
        image=dest/'images'/s/name
        label=dest/'labels'/s/(Path(name).stem+'.txt')
        image.parent.mkdir(parents=True,exist_ok=True)
        label.parent.mkdir(parents=True,exist_ok=True)
        if pixels is None:image.symlink_to(src.resolve())
        else:assert cv2.imwrite(str(image),pixels)
        label.write_text('\n'.join(labels)+ ('\n' if labels else ''))
        counts[s]+=1
    for row in records:
        if row['split']=='train' and not row['labels'] and variant=='A_positive':continue
        repeats=4 if row['split']=='train' and row['labels'] else 2 if row['split']=='train' else 1
        for r in range(repeats):
            put(f"{row['src'].stem}_rep{r}.png",row['split'],row['labels'],src=row['src'])
        index.append({**row,'src':str(row['src']),'repeat':repeats})
    sim=ROOT/'261026_1414/data'
    rng=random.Random(0)
    train=sorted((sim/'images/train').glob('*.png'))
    rng.shuffle(train)
    train=train[:720]
    for s in ['train','val','eval']:
        paths=train if s=='train' else sorted((sim/'images'/s).glob('*.png'))
        target=s if s!='eval' else 'sim_eval'
        for i,src in enumerate(paths):
            labels=[]
            for line in (sim/'labels'/s/(src.stem+'.txt')).read_text().splitlines():
                if line.strip():
                    cid,*coords=line.split()
                    labels.append(' '.join([str(REMAP[int(cid)]),*coords]))
            put('sim_'+src.name,target,labels,src=src)
            if s=='train':
                image=cv2.imread(str(src))
                gamma=[.55,1.65][i%2];gain=[.65,1.20][(i//2)%2];sigma=[2.,5.][(i//4)%2]
                arr=(np.power(image.astype(np.float32)/255.,gamma)*255.*gain)
                noise=np.random.default_rng(20261030+i).normal(0,sigma,image.shape)
                arr=np.clip(arr+noise,0,255).astype(np.uint8)
                if i%3==0:arr=cv2.GaussianBlur(arr,(3,3),0)
                put('aug_'+src.name,target,labels,pixels=arr)
    (dest/'classes.txt').write_text('\n'.join(NAMES)+'\n')
    (dest/'data.yaml').write_text(f'path: {dest}\ntrain: images/train\nval: images/val\nnames:\n'+''.join(f'  {i}: {n}\n' for i,n in enumerate(NAMES)))
    (dest/'split.json').write_text(json.dumps({'purpose':'제한된 PC 비교, 실물3분할 최종검증 아님','real':index,'split':split,'counts':dict(counts),'sim_train':[p.name for p in train]},ensure_ascii=False,indent=2))
    print(variant,dict(counts),flush=True)
(OUT/'dataset_summary.json').write_text(json.dumps({'fixed_split':split,'real_checked':391,'excluded_unchecked':111,'builder_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest()},ensure_ascii=False,indent=2))
