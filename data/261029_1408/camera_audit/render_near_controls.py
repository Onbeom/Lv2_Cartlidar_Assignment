"""같은 URDF/USD 카메라의 near·양면·로봇숨김 대조. 원본 모델 저장/수정 없음."""
from pathlib import Path
import json,sys,hashlib
import numpy as np
root=Path(__file__).resolve().parents[2];sys.path.insert(0,str(root/'Cartlidar/sim_render'))
meta=json.loads((root/'Cartlidar/sim_render/results/mission_261028/render_r12_gpu/render_r12.json').read_text());pose=meta['poses'][0]
out=root/'Cartlidar/sim_render/results/followup_261028_1457/camera_near_controls';out.mkdir(exist_ok=False)
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'width':640,'height':480,'anti_aliasing':0,'multi_gpu':False})
import omni.usd,omni.replicator.core as rep
from pxr import Gf,Usd,UsdGeom
import cv2
from robot_usd import use_robot_usd,pose_robot,ROBOT
from urdf_chain import fixed_transform
ctx=omni.usd.get_context();ctx.open_stage(str(Path(pose['world'])/'task_world.usda'))
def updates(n):
 for _ in range(n):app.update()
updates(60);stage=ctx.get_stage();usd=use_robot_usd(stage);updates(10);pose_robot(stage,meta['urdf'],np.array(pose['T_world_base_footprint']),meta['arm_q']);updates(10)
meshes=[UsdGeom.Mesh(p) for p in Usd.PrimRange(stage.GetPrimAtPath(ROBOT)) if p.IsA(UsdGeom.Mesh)];double_before=[bool(m.GetDoubleSidedAttr().Get()) for m in meshes]
robot=UsdGeom.Imageable(stage.GetPrimAtPath(ROBOT));camera=UsdGeom.Camera.Define(stage,'/World/CameraAudit/camera')
camera.CreateHorizontalApertureAttr(20.955);camera.CreateVerticalApertureAttr(20.955*480/640)
rp=rep.create.render_product(str(camera.GetPath()),(640,480));rgb=rep.AnnotatorRegistry.get_annotator('rgb');dep=rep.AnnotatorRegistry.get_annotator('distance_to_image_plane');rgb.attach(rp);dep.attach(rp);rows=[]
for side in ('front','rear'):
 for sensor in ('color','infra1','infra2'):
  frame=f'top_camera_{side}_{sensor}_optical_frame';T=np.array(pose['T_world_base_footprint'])@fixed_transform(meta['urdf'],frame);K=meta['K'] if sensor=='color' else {'fx':385.012817382812,'fy':385.012817382812,'cx':320.859619140625,'cy':239.928024291992};f=K['fx']*20.955/640
  camera.CreateFocalLengthAttr(f);camera.CreateVerticalApertureAttr(f*480/K['fy']);camera.CreateHorizontalApertureOffsetAttr(-(K['cx']-320)/640*20.955);camera.CreateVerticalApertureOffsetAttr((K['cy']-240)/640*20.955)
  R,t=T[:3,:3],T[:3,3];xf=UsdGeom.Xformable(camera);xf.ClearXformOpOrder();xf.AddTransformOp().Set(Gf.Matrix4d(*R[:,0].tolist(),0,*(-R[:,1]).tolist(),0,*(-R[:,2]).tolist(),0,*t.tolist(),1))
  for label,near,visible,double in [('default_near10mm',.01,True,False),('default_near0p01mm',.00001,True,False),('double_near0p01mm',.00001,True,True),('hidden_near0p01mm',.00001,False,False)]:
   if visible:robot.MakeVisible()
   else:robot.MakeInvisible()
   for m,value in zip(meshes,double_before):m.GetDoubleSidedAttr().Set(True if double else value)
   camera.CreateClippingRangeAttr(Gf.Vec2f(near,50.));updates(12)
   for _ in range(3):rep.orchestrator.step(rt_subframes=8)
   image=np.asarray(rgb.get_data())[:,:,:3].astype(np.uint8);depth=np.asarray(dep.get_data()).astype(np.float32);name=frame+'_'+label;cv2.imwrite(str(out/(name+'.png')),cv2.cvtColor(image,cv2.COLOR_RGB2BGR));np.save(out/(name+'_depth.npy'),depth);finite=np.isfinite(depth)&(depth>0)
   row={'frame':frame,'case':label,'near_m':near,'visible':visible,'forced_double_sided':double,'finite_fraction':float(finite.mean()),'under10mm_fraction':float((finite&(depth<.01)).mean()),'under150mm_fraction':float((finite&(depth<.15)).mean()),'depth_percentile_m':np.percentile(depth[finite],[0,1,50,99,100]).tolist() if finite.any() else None,'center_depth_m':float(depth[240,320]) if np.isfinite(depth[240,320]) else None,'image':name+'.png','depth':name+'_depth.npy'};rows.append(row);print(json.dumps(row,ensure_ascii=False),flush=True)
robot.MakeVisible();rgb.detach();dep.detach();rp.destroy();updates(4)
(out/'summary.json').write_text(json.dumps({'urdf':meta['urdf'],'urdf_sha256':hashlib.sha256(Path(meta['urdf']).read_bytes()).hexdigest(),'usd':usd,'mesh_count':len(meshes),'authored_double_sided_true':sum(double_before),'rows':rows,'note':'정적 렌더 대조; 원본 모델 수정/저장 없음'},ensure_ascii=False,indent=2));app.close()
