"""동일 실물 평가사진의 기존/A/B 검출 비교. 학습에 평가사진을 사용하지 않는다."""
import json
from pathlib import Path
import cv2
import numpy as np
from PIL import Image,ImageDraw,ImageFont

ROOT=Path('/home/pa4/git/lv2_next_module');BASE=ROOT/'Cartlidar/sim_render/results/followup_261028_1457/improvement_261030_1325/yolo_evaluation'
names=['legacy6','A_positive','B_negative']
data={n:{v['image']:v for v in json.loads((BASE/(n+'_eval_rows.json')).read_text())} for n in names}
def overlaps(a,b):
    a=np.array(a);b=np.array(b);wh=np.maximum(0,np.minimum(a[2:],b[2:])-np.maximum(a[:2],b[:2]));i=wh.prod()
    return i/((a[2:]-a[:2]).prod()+(b[2:]-b[:2]).prod()-i+1e-9)
def counts(row):
    got=set();tp=fp=0
    for d in sorted(row['detections'],key=lambda v:-v['score']):
        if d['score']<.25:continue
        candidates=[(overlaps(d['box'],g['box']),j) for j,g in enumerate(row['gt']) if j not in got and g['class']==d['class']]
        overlap,j=max(candidates,default=(0,-1))
        if overlap>=.5:tp+=1;got.add(j)
        else:fp+=1
    return tp,fp,len(row['gt'])-tp
keys=list(data['B_negative'])
negative=max((k for k in keys if not data['B_negative'][k]['gt']),key=lambda k:counts(data['legacy6'][k])[1])
improved=max((k for k in keys if data['B_negative'][k]['gt']),key=lambda k:counts(data['B_negative'][k])[0]-counts(data['legacy6'][k])[0])
missed=max((k for k in keys if data['B_negative'][k]['gt']),key=lambda k:counts(data['B_negative'][k])[2])
chosen=[negative,improved,missed]
font_path='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
font=ImageFont.truetype(font_path,20);small=ImageFont.truetype(font_path,16)
canvas=Image.new('RGB',(1920,1210),'#f7f7f7');draw=ImageDraw.Draw(canvas)
draw.text((20,10),'동일 실물 평가사진: 기존 / 양성·합성 후보 A / 음성 추가 후보 B',font=font,fill='black')
colors=['#ee5555','#38b76b','#f9a33b','#3f9edf'];labels=['빨강 원기둥','초록 원기둥','빨강 직육면체','초록 직육면체']
for row_index,key in enumerate(chosen):
    image=cv2.cvtColor(cv2.imread(key),cv2.COLOR_BGR2RGB)
    h,w=image.shape[:2];scale=min(630/w,300/h);nw,nh=round(w*scale),round(h*scale)
    for col,n in enumerate(names):
        x=col*640+5;y=55+row_index*350
        draw.text((x,y),['기존 6클래스','후보 A','후보 B'][col],font=font,fill='black')
        image_pil=Image.fromarray(image).resize((nw,nh));ox=x+(630-nw)//2;oy=y+30
        canvas.paste(image_pil,(ox,oy));row=data[n][key]
        for g in row['gt']:
            box=[ox+g['box'][0]*scale,oy+g['box'][1]*scale,ox+g['box'][2]*scale,oy+g['box'][3]*scale]
            draw.rectangle(box,outline='white',width=1)
        for d in row['detections']:
            if d['score']<.25 or d['class']<0:continue
            box=[ox+d['box'][0]*scale,oy+d['box'][1]*scale,ox+d['box'][2]*scale,oy+d['box'][3]*scale]
            draw.rectangle(box,outline=colors[d['class']],width=2)
            txt=f"{labels[d['class']]} {d['score']:.2f}"
            tx=box[0];ty=max(oy,box[1]-18);bound=draw.textbbox((tx,ty),txt,font=small)
            draw.rectangle(bound,fill='#111111');draw.text((tx,ty),txt,font=small,fill=colors[d['class']])
        tp,fp,fn=counts(row);draw.text((x,y+325),f'올바른 검출 {tp} · 오검출 {fp} · 누락 {fn}',font=small,fill='black')
draw.text((20,1110),'전체 203장 / 정답 378개: 기존 TP325·FP56 → B TP366·FP11. B에도 초록 원기둥 누락 12개가 남음.',font=font,fill='black')
draw.text((20,1140),'흰 선=정답 / 색 선=검출 · conf≥0.25, IoU≥0.5 · Ultralytics PC 평가. 실물 주행·Jetson·TensorRT 시험 아님.',font=small,fill='black')
target=ROOT/'photo/261030_YOLO_학습후_실물사진_비교.png';canvas.save(target)
(BASE/'photo_examples.json').write_text(json.dumps({'images':chosen,'selection':'오검출/개선/잔여누락 사례. 대표성 무작위표본 아님'},ensure_ascii=False,indent=2))
print(target)
