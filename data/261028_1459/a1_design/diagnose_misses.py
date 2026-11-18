import json,sys,math,collections
from pathlib import Path
import cv2,numpy as np
sys.path.insert(0,str(Path(__file__).parent))
from a1_tagdet import CFG,detect,find_labels
root=Path('Cartlidar/sim_render/results/render_a1_1711_design_261027');meta=json.loads((root/'render_a1.json').read_text());cfg={**CFG,'dark_win':31,'dark_c':20,'crop_factor':0.};stats=collections.Counter();examples=[]
for sc in meta['scenes']:
 info={p['name']:p for p in sc['pucks']}
 for f in sc['arm_frames']:
  targets=[(n,p) for n,p in f['pucks'].items() if info[n]['shape']=='cylinder' and info[n]['standing'] and p['in_frac']==1 and p['vis']>=.9]
  if not targets:continue
  image=cv2.imread(str(root/f['image']));dets=detect(image,cfg);hsv=cv2.cvtColor(cv2.GaussianBlur(image,(3,3),0),cv2.COLOR_BGR2HSV)
  for name,p in targets:
   cls=0 if info[name]['color']=='red' else 1
   near=sorted((math.hypot(d[0]-p['box'][0],d[1]-p['box'][1]),d) for d in dets if d[3]==cls)
   if near and near[0][0]<=.05:continue
   labels=find_labels(hsv,cfg,cv2.cvtColor(image,cv2.COLOR_BGR2GRAY));tx=(p['box'][0]+1)*320;ty=(p['box'][1]+1)*240
   labels=sorted(labels,key=lambda L:math.hypot(L['c'][0]-tx,L['c'][1]-ty))
   if not labels or math.hypot(labels[0]['c'][0]-tx,labels[0]['c'][1]-ty)>30:reason='no_near_label'
   else:reason='label_but_center_miss'
   stats[reason]+=1
   if len(examples)<16:examples.append({'scene':sc['name'],'k':f['k'],'target':name,'reason':reason,'box':p['box'],'nearest_det':near[0] if near else None,'label':{'center':list(map(float,labels[0]['c'])),'side':float(labels[0]['side'])} if labels else None,'image':str(root/f['image'])})
out=Path('Cartlidar/sim_render/results/a1_code_261028');(out/'miss_diagnostic.json').write_text(json.dumps({'counts':dict(stats),'examples':examples},indent=2));print(stats)
