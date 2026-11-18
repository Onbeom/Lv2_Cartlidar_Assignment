"""CAD visual 실루엣만 IR에서 가림. GT/환경깊이는 읽지 않는다."""
import json
import os
from collections import Counter
from pathlib import Path
import numpy as np
import rclpy
from sensor_msgs.msg import Image,JointState,CameraInfo
from rclpy.qos import qos_profile_sensor_data

ROOT=Path('/home/pa4/git/lv2_next_module');BASE=ROOT/'Cartlidar/sim_render/results/followup_261028_1457/improvement_261030_1325'
light=os.environ['MASK_LIGHT'];stats=Counter()
entries={r['camera']:r for r in json.loads((BASE/'ir_visual_masks/summary.json').read_text())['rows'] if r['light']==light and r['mode']=='two_sided'}
assert len(entries)==2 and all(r['guard_pass'] for r in entries.values())
masks={c:np.load(BASE/'ir_visual_masks'/f'{light}_{c}_two_sided.npy') for c in entries}
def sec(stamp):return stamp.sec+stamp.nanosec*1e-9
rclpy.init();node=rclpy.create_node('ir_visual_mask_relay');latest={'q':None,'stamp':None};info={}
def on_joint(msg):
    if len(msg.name)==len(msg.position):latest.update(q=dict(zip(msg.name,msg.position)),stamp=sec(msg.header.stamp))
node.create_subscription(JointState,'/joint_states',on_joint,20)
for camera in ['infra1','infra2']:
    publisher=node.create_publisher(Image,f'/top_camera_front/{camera}/image_rect_raw',qos_profile_sensor_data)
    def on_info(msg,c=camera):info[c]=msg
    node.create_subscription(CameraInfo,f'/top_camera_front/{camera}/camera_info',on_info,20)
    def on_image(msg,c=camera,pub=publisher):
        stats['input']+=1;t=sec(msg.header.stamp);reference=entries[c];q=latest['q'];i=info.get(c)
        valid=(msg.encoding=='mono8' and msg.width==640 and msg.height==480 and msg.step>=640 and len(msg.data)>=480*msg.step)
        if not valid:stats['invalid_image']+=1;return
        can_use=t>=3. and q is not None and latest['stamp'] is not None and abs(t-latest['stamp'])<=.1 and i is not None
        if can_use:
            keys=[n for n in reference['q'] if not n.startswith('Wheel_')]
            can_use=all(n in q and np.isfinite(q[n]) and abs(q[n]-reference['q'][n])<=.01 for n in keys)
            k=reference['K'];can_use=can_use and all(abs(a-b)<1e-6 for a,b in zip([i.k[0],i.k[4],i.k[2],i.k[5]],[k['fx'],k['fy'],k['cx'],k['cy']]))
        image=np.frombuffer(bytes(msg.data),np.uint8).reshape(480,msg.step).copy()
        if can_use:image[:,:640][masks[c]]=0;stats['visual_mask']+=1
        else:image[480-220:,:]=0;stats['bottom220_fallback']+=1
        msg.data=image.tobytes();pub.publish(msg);stats['output']+=1
    node.create_subscription(Image,f'/unmasked/top_camera_front/{camera}/image_rect_raw',on_image,qos_profile_sensor_data)
try:rclpy.spin(node)
finally:
    target=Path(os.environ['MASK_STATS']);target.write_text(json.dumps({'counts':dict(stats),'light':light,'input_contract':'3초 이전→bottom220, 기준자세 범위 안→visual CAD, 바퀴회전은 고정 실루엣 근사'},ensure_ascii=False,indent=2))
    node.destroy_node();rclpy.shutdown()
