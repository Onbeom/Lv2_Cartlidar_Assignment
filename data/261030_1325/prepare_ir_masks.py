"""기준시각까지의 관절값·URDF·K로 IR 투영실루엣을 만든다. GT 불사용."""
import json
import sys
import time
from pathlib import Path
import numpy as np
from scipy.spatial import ConvexHull
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

ROOT=Path('/home/pa4/git/lv2_next_module');HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'Cartlidar/sim_world'));sys.path.insert(0,str(ROOT/'261029_2329'))
from robot_geom import Robot
from filter_core import build_cache
BASE=ROOT/'Cartlidar/sim_render/results/followup_261028_1457/improvement_261030_1325'
OUT=BASE/'ir_projection_masks';OUT.mkdir(exist_ok=False)
urdf=ROOT/'261027_1711/urdf/waffle_4wd_description_261027_1711/urdf/waffle_4wd.urdf'
robot=Robot(urdf,urdf.parent.parent)
records={'bright':'m1711_bright_1_261027_203413','dark':'m1711_dark_1_261027_203822'}
reports=[]
for light,rec in records.items():
    reader=rosbag2_py.SequentialReader();reader.open(rosbag2_py.StorageOptions(uri=str(ROOT/'261027_0953/results'/rec/'bag'),storage_id='mcap'),rosbag2_py.ConverterOptions('cdr','cdr'))
    topics=reader.get_all_topics_and_types();types={t.name:t.type for t in topics};reader.set_filter(rosbag2_py.StorageFilter(topics=['/joint_states','/top_camera_front/infra1/camera_info','/top_camera_front/infra2/camera_info']))
    q=None;info={};qstamp=None
    while reader.has_next():
        topic,data,_=reader.read_next();msg=deserialize_message(data,get_message(types[topic]))
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if stamp>3.:break
        if topic=='/joint_states':q=dict(zip(msg.name,msg.position));qstamp=stamp
        else:info[topic.split('/')[2]]=msg
    assert q is not None and len(info)==2
    poses=robot.link_poses(q);shapes=robot.world_shapes(q)
    for camera,msg in info.items():
        frame=msg.header.frame_id;C=poses[frame];R=C[:3,:3];t=C[:3,3]
        k={'fx':msg.k[0],'fy':msg.k[4],'cx':msg.k[2],'cy':msg.k[5]};prepared=[]
        for _,vertices in shapes:
            v=(vertices-t)@R;lo=v.min(0)-.005;hi=v.max(0)+.005
            if hi[2]<=.01:continue
            eq=ConvexHull(v).equations;b=(0,640,0,480)
            if lo[2]>0:
                corners=np.array([[x,y,z] for x in [lo[0],hi[0]] for y in [lo[1],hi[1]] for z in [lo[2],hi[2]]])
                p=corners[:,:2]/corners[:,2,None]*[k['fx'],k['fy']]+[k['cx'],k['cy']]
                a=np.floor(p.min(0)).astype(int)-1;z=np.ceil(p.max(0)).astype(int)+2
                b=(max(0,int(a[0])),min(640,int(z[0])),max(0,int(a[1])),min(480,int(z[1])))
            prepared.append((lo,hi,eq,b))
        start=time.perf_counter();chunks=build_cache(k,prepared);mask=np.zeros(640*480,bool)
        for ids,lo,hi in chunks:
            keep=(hi>=.01)&(lo<=4.)
            mask[ids[keep]]=True
        mask=mask.reshape(480,640);fraction=float(mask.mean())
        np.save(OUT/f'{light}_{camera}.npy',mask)
        report={'light':light,'camera':camera,'frame':frame,'reference_joint_stamp':qstamp,'q':q,'K':k,
                'masked_pixels':int(mask.sum()),'masked_fraction':fraction,'build_s':time.perf_counter()-start,
                'guard_pass':fraction<=.75,'urdf':str(urdf),'ground_truth_input':False}
        reports.append(report);print(json.dumps({k:v for k,v in report.items() if k!='q'}),flush=True)
(OUT/'summary.json').write_text(json.dumps({'rows':reports,'all_guard_pass':all(v['guard_pass'] for v in reports)},ensure_ascii=False,indent=2))
