"""S1/S2: 발제 배치는 정답 생성기에만 사용. 등록부에는 관측과 발제 박스 중심만 전달."""
import sys,json,math,copy,random,tempfile
from pathlib import Path
sys.path.insert(0,str(Path(__file__).parent/'src/perception_survey'))
from perception_survey.core import Survey
TOP=Path(__file__).resolve().parents[1];OUT=TOP/'Cartlidar/sim_render/results/puck_survey_261028'
rng=random.Random(20261028)

def obs(p,noise=0):
 return {'x':p['center'][0]+rng.uniform(-noise,noise),'y':p['center'][1]+rng.uniform(-noise,noise),
  'id':p['tag_id'],'color':p['color'],'shape':p['shape'],'pose':'standing' if p['standing'] else 'lying',
  'yaw_deg':0.,'camera':'arm','confidence':.95}

def fill(s,ps,change=None):
 for k in range(3):
  for i,p in enumerate(ps):
   o=obs(p)
   if change:change(o,i)
   s.observe(o,10.+k)
 return s

worlds=[]
for seed in range(1,7):
 lay=json.loads((TOP/f'Cartlidar/sim_world/worlds/dice_mission/seed_{seed}/layout.json').read_text());s=Survey()
 # 서로 다른 로봇 위치/방향에서 얻은 상대 관측을 지도 좌표로 되돌림(각축3mm 잡음).
 for k in range(4):
  bx,by,yaw=rng.uniform(0,4),rng.uniform(0,6),rng.uniform(-math.pi,math.pi)
  cs,sn=math.cos(yaw),math.sin(yaw)
  for p in rng.sample(lay['pucks'],12):
   o=obs(p,.003);dx,dy=o['x']-bx,o['y']-by
   lx,ly=cs*dx+sn*dy,-sn*dx+cs*dy
   o['x'],o['y']=bx+cs*lx-sn*ly,by+sn*lx+cs*ly
   if k<2:o['id']=-1;o['camera']='front_d435'
   assert s.observe(o,100.+k)
 r=s.snapshot();assert r['complete'],r['errors'];maxerr=0.
 for p in lay['pucks']:
  t=next(t for t in r['slots'] if t['box']==p['box'] and t['side']==p['side'])
  assert (t['id'],t['color'],t['shape'],t['pose'])==(p['tag_id'],p['color'],p['shape'],'standing' if p['standing'] else 'lying')
  err=math.hypot(t['x_mm']-1000*p['center'][0],t['y_mm']-1000*p['center'][1]);assert err<=10;maxerr=max(maxerr,err)
 path=OUT/f'seed_{seed}/puck_survey.json';f=s.finish(path);assert json.loads(path.read_text())==f
 assert not s.observe(obs(lay['pucks'][0]),105.)
 worlds.append({'seed':seed,'correct':12,'slots':12,'max_position_error_mm':maxerr,'yaw_measured':sum(b['yaw_measured'] for b in r['boxes'])})
ps=json.loads((TOP/'Cartlidar/sim_world/worlds/dice_mission/seed_1/layout.json').read_text())['pucks'];cases=[]

def reject(name,s):
 try:s.finish(OUT/'invalid_should_not_exist.json')
 except ValueError:cases.append({'case':name,'refused':True});return
 raise AssertionError(name)
reject('missing_puck',fill(Survey(),ps[:-1]))
reject('duplicate_id',fill(Survey(),ps,lambda o,i:o.update(id=ps[0]['tag_id']) if i==1 else None))
s=fill(Survey(),ps);o=obs(ps[0]);o['color']='green' if o['color']=='red' else 'red';s.observe(o,20.);reject('color_conflict',s)
reject('unknown_shape',fill(Survey(),ps,lambda o,i:o.update(shape=None)))
reject('unknown_pose',fill(Survey(),ps,lambda o,i:o.update(pose=None)))
s=Survey();o=obs(ps[0]);assert s.observe(o,10.);assert not s.observe(o,9.);reject('out_of_order',s)
s=Survey();assert s.observe(obs(ps[0]),10.);assert not s.observe(obs(ps[0]),10.);reject('duplicate_frame',s)
s=Survey();o=obs(ps[0]);o['x']=float('nan');assert not s.observe(o,10.);reject('nonfinite_position',s)
s=Survey();o=obs(ps[0]);o['color']='blue';assert not s.observe(o,10.);reject('blue_obstacle',s)
s=Survey();o=obs(ps[0]);o['id']=12;assert not s.observe(o,10.);reject('place_tag_not_puck',s)
s=Survey();o=obs(ps[0]);o['x']=0.;o['y']=0.;assert not s.observe(o,10.);reject('outside_slot',s)
reject('empty_reset',Survey())
result={'S1':{'pass':True,'correct':72,'total':72,'worlds':worlds},'S2':{'pass':True,'refused':len(cases),'total':len(cases),'cases':cases}}
(OUT/'core_summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2));print(json.dumps(result,ensure_ascii=False))
