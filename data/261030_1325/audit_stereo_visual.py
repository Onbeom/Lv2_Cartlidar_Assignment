"""기존 SLAM 입력의 동기화/특징 진단. 정답 궤적을 읽지 않는다."""
import json
from collections import Counter
from pathlib import Path
import cv2
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

ROOT=Path('/home/pa4/git/lv2_next_module')
OUT=ROOT/'Cartlidar/sim_render/results/followup_261028_1457/improvement_261030_1325'
names={'bright':'m1711_bright_1_261027_203413','dark':'m1711_dark_1_261027_203822'}
left='/top_camera_front/infra1/image_rect_raw';right='/top_camera_front/infra2/image_rect_raw'
reports=[]
for light,name in names.items():
    path=ROOT/'261027_0953/results'/name/'bag';reader=rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path),storage_id='mcap'),rosbag2_py.ConverterOptions('cdr','cdr'))
    types={t.name:t.type for t in reader.get_all_topics_and_types()}
    topics=[left,right,left.replace('image_rect_raw','camera_info'),right.replace('image_rect_raw','camera_info')]
    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    targets=[1,30,60,90,120,150,180,210,240,270];pending={};selected={};info={};counts=Counter();stamps={left:set(),right:set()}
    while reader.has_next():
        topic,data,_=reader.read_next();msg=deserialize_message(data,get_message(types[topic]));counts[topic]+=1
        if 'camera_info' in topic:
            if topic not in info:info[topic]={'k':list(msg.k),'p':list(msg.p),'d':list(msg.d),'frame':msg.header.frame_id,'width':msg.width,'height':msg.height}
            continue
        ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec;stamps[topic].add(ns)
        slot=next((t for t in targets if ns*1e-9>=t and ns*1e-9<t+.04 and t not in selected),None)
        if slot is None:continue
        assert msg.encoding=='mono8'
        image=np.ndarray((msg.height,msg.width),dtype=np.uint8,buffer=msg.data,strides=(msg.step,1)).copy()
        pending.setdefault(ns,{})[topic]=image
        if len(pending[ns])==2:selected[slot]=(ns,pending[ns]);pending.pop(ns)
    orb=cv2.ORB_create(nfeatures=1200);matcher=cv2.BFMatcher(cv2.NORM_HAMMING)
    rows=[]
    for stamp,(ns,images) in sorted(selected.items()):
        for mode in ['raw','mask220','visual']:
            imgs=[images[t].copy() for t in [left,right]]
            if mode=='mask220':
                for image in imgs:image[image.shape[0]-220:]=0
            if mode=='visual':
                if stamp<3.:
                    for image in imgs:image[image.shape[0]-220:]=0
                else:
                    for image,camera in zip(imgs,['infra1','infra2']):image[np.load(OUT/'ir_visual_masks'/f'{light}_{camera}_two_sided.npy')]=0
            features=[orb.detectAndCompute(image,None) for image in imgs]
            matches=[]
            if features[0][1] is not None and features[1][1] is not None:
                pairs=matcher.knnMatch(features[0][1],features[1][1],k=2)
                matches=[a for pair in pairs if len(pair)==2 for a,b in [pair] if a.distance<.75*b.distance]
            deltas=[]
            if len(matches)>=8:
                pts1=np.float32([features[0][0][m.queryIdx].pt for m in matches]);pts2=np.float32([features[1][0][m.trainIdx].pt for m in matches])
                F,inliers=cv2.findFundamentalMat(pts1,pts2,cv2.FM_RANSAC,1.,.99)
                if inliers is not None:deltas=np.abs(pts1[inliers.ravel().astype(bool),1]-pts2[inliers.ravel().astype(bool),1]).tolist()
            rows.append({'target_s':stamp,'stamp_ns':ns,'mode':mode,'left_features':len(features[0][0]),'right_features':len(features[1][0]),
                         'ratio_matches':len(matches),'ransac_inliers':len(deltas),
                         'median_abs_y_px':float(np.median(deltas)) if deltas else None,'p95_abs_y_px':float(np.percentile(deltas,95)) if deltas else None})
    reports.append({'light':light,'counts':dict(counts),'left_unique_stamps':len(stamps[left]),'right_unique_stamps':len(stamps[right]),
                    'unpaired_left':len(stamps[left]-stamps[right]),'unpaired_right':len(stamps[right]-stamps[left]),'camera_info':info,'rows':rows})
(OUT/'stereo_visual_audit.json').write_text(json.dumps({'simulation_only':True,'rows':reports,'note':'ORB/RANSAC대응진단이며정확한스테레오보정품질/실물검증은아님'},ensure_ascii=False,indent=2))
for report in reports:print(json.dumps(report,ensure_ascii=False),flush=True)
