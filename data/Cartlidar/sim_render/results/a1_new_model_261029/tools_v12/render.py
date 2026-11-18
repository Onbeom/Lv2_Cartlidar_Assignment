from common import *
import argparse
ap=argparse.ArgumentParser();ap.add_argument('--plan',type=Path,required=True);ap.add_argument('--out',type=Path,required=True);ap.add_argument('--world-seed',type=int,required=True);args,_=ap.parse_known_args();plan=json.loads(args.plan.read_text())['plan'];todo=[s for s in plan if s['seed']==args.world_seed];assert len(todo)==1;args.out.mkdir(exist_ok=False,parents=True)
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'width':1280,'height':720,'anti_aliasing':0,'multi_gpu':False})
import omni.usd,omni.replicator.core as rep
from pxr import UsdGeom,Gf
import cv2
sys.path.insert(0,str(ROOT/'Cartlidar/sim_render'));from robot_usd import use_robot_usd,set_xform,ROBOT
ctx=omni.usd.get_context();sc=todo[0];lay=json.loads((Path(sc['world'])/'layout.json').read_text());pucks={p['name']:p for p in lay['pucks']};ctx.open_stage(str(Path(sc['world'])/'task_world.usda'))
def updates(n):
 for _ in range(n):app.update()
updates(60);stage=ctx.get_stage();use_robot_usd(stage,USD);updates(10)
Tbase=np.array(sc['T_world_base_link']);set_xform(stage.GetPrimAtPath(ROBOT),Tbase)
def pose(q):
 p=FRAMES.poses(q)
 for link,path in sorted(SENSORS['moving_bodies'].items(),key=lambda a:a[1].count('/')):
  prim=stage.GetPrimAtPath(ROBOT+'/'+path);parent=prim.GetParent().GetName()
  if not prim.IsValid():raise RuntimeError(('USD프레임없음',path))
  set_xform(prim,np.linalg.inv(p[parent])@p[link])
def setcam(camera,T,c):
 w,h,K=c['width'],c['height'],c['K'];aperture=20.955;f=K['fx']*aperture/w;camera.CreateHorizontalApertureAttr(aperture);camera.CreateVerticalApertureAttr(f*h/K['fy']);camera.CreateFocalLengthAttr(f);camera.CreateClippingRangeAttr(Gf.Vec2f(c['near'],10. if c is CAM['d435'] else 5.));camera.CreateHorizontalApertureOffsetAttr(-(K['cx']-w/2)/w*aperture);camera.CreateVerticalApertureOffsetAttr((K['cy']-h/2)/h*(f*h/K['fy']));A=T.copy();A[:3,1:3]*=-1;set_xform(camera.GetPrim(),A)
cams={}
for kind,c in CAM.items():
 camera=UsdGeom.Camera.Define(stage,'/World/A1New/'+kind);setcam(camera,np.array(sc['T_world_d435']),c);rp=rep.create.render_product(str(camera.GetPath()),(c['width'],c['height']));rgb=rep.AnnotatorRegistry.get_annotator('rgb');dep=rep.AnnotatorRegistry.get_annotator('distance_to_image_plane');rgb.attach(rp);dep.attach(rp);cams[kind]=(camera,rp,rgb,dep)
def grab(kind):
 for _ in range(2):rep.orchestrator.step(rt_subframes=8)
 camera,rp,rgb,dep=cams[kind];return np.asarray(rgb.get_data())[:,:,:3].astype(np.uint8),np.asarray(dep.get_data()).astype(np.float32)
def truths(T,depth,kind):
 c=CAM[kind];g=c['geometry'];out={}
 for n,p in pucks.items():
  surf=g['puck_surface'](p);inside=g['in_frac'](*surf,T,c['K'])
  if inside==0:continue
  vis,box=g['truth_box'](*surf,T,c['K'],depth);out[n]=dict(in_frac=round(inside,3),vis=round(vis,3),box=box)
 return out
report=dict(urdf=str(URDF),robot_usd=str(USD),model='261028',root='base_link',arm_q=Q,width=640,height=480,d435={k:v for k,v in CAM['d435'].items() if k!='geometry'},arm={k:v for k,v in CAM['arm'].items() if k!='geometry'},K_source='model_HFOV_not_real_calibration',jitter_injected=False,noise=False,scenes=[])
for level in [0.,1.]:
 stage.GetPrimAtPath('/World/Lights/dome').GetAttribute('inputs:intensity').Set(300.+300.*level)
 for i in range(6):stage.GetPrimAtPath(f'/World/Lights/ceiling_{i}').GetAttribute('inputs:intensity').Set(20000.*level)
 name=sc['id']+f'_L{int(level)}';(args.out/name).mkdir();pose(Q);setcam(cams['d435'][0],np.array(sc['T_world_d435']),CAM['d435']);updates(6);rgb,depth=grab('d435');cv2.imwrite(str(args.out/name/'d435.png'),cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR));np.save(args.out/name/'d435_depth.npy',depth);dv=truths(np.array(sc['T_world_d435']),depth,'d435');frames=[]
 for k,f in enumerate(sc['arm']):
  pose(f['arm_q']);setcam(cams['arm'][0],np.array(f['T_world_cam']),CAM['arm']);updates(4);rgb,depth=grab('arm');fn=f'{name}/arm_{k:02d}.png';cv2.imwrite(str(args.out/fn),cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR));frames.append(dict(f,k=k,image=fn,pucks=truths(np.array(f['T_world_cam']),depth,'arm')))
 e={k:v for k,v in sc.items() if k!='arm'};e.update(name=name,light_level=level,d435_images=dict(color=f'{name}/d435.png',depth=f'{name}/d435_depth.npy'),d435_visible={n:v['vis'] for n,v in dv.items()},arm_frames=frames,pucks=[{k:p[k] for k in ['name','tag_id','color','shape','standing','location','center']} for p in lay['pucks']]);report['scenes'].append(e);print(name,'frames',len(frames),flush=True)
for camera,rp,rgb,dep in cams.values():rgb.detach();dep.detach();rp.destroy()
(args.out/'render_a1.json').write_text(json.dumps(report,indent=1)+'\n');app.close()
