"""기존 센서와 고정 VO를 합친 진단 bag. 정답/옛 map TF/바퀴odom은 입력하지 않음."""
import hashlib
import json
from collections import Counter, defaultdict
from pathlib import Path
import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message

ROOT=Path('/home/pa4/git/lv2_next_module')
BASE=ROOT/'Cartlidar/sim_render/results/followup_261028_1457'
OUT=BASE/'improvement_261030_1325'
old={'bright':'m1711_bright_1_261027_203413','dark':'m1711_dark_1_261027_203822'}
def read(path,topics):
    reader=rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path),storage_id='mcap'),rosbag2_py.ConverterOptions('cdr','cdr'))
    metadata={v.name:v for v in reader.get_all_topics_and_types()}
    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    while reader.has_next():
        topic,data,stamp=reader.read_next()
        yield metadata[topic],data,stamp

for light,rec in old.items():
    target=OUT/f'slam_input_{light}'
    assert not target.exists()
    rows=[];counts=Counter();edges=Counter();headers=defaultdict(list);tf_static=0
    source=ROOT/'261027_0953/results'/rec/'bag'
    vo=BASE/f'slam_coupled_{light}'/'out_bag'
    for path,topics,kind in [(source,['/scan_front','/scan_rear','/tf_static','/tf','/clock'],'sensor'),
                             (vo,['/visual_slam/tracking/odometry','/tf'],'vo')]:
        for metadata,data,stamp in read(path,topics):
            topic=metadata.name
            if topic=='/tf':
                msg=deserialize_message(data,get_message(metadata.type))
                if kind=='sensor':msg.transforms=[t for t in msg.transforms if t.header.frame_id not in ['map','odom']]
                else:msg.transforms=[t for t in msg.transforms if t.header.frame_id=='odom' and t.child_frame_id=='base_footprint']
                if not msg.transforms:continue
                for t in msg.transforms:edges[t.header.frame_id+'->'+t.child_frame_id]+=1
                data=serialize_message(msg)
            elif topic=='/tf_static':tf_static+=1
            elif topic in ['/scan_front','/scan_rear','/visual_slam/tracking/odometry']:
                msg=deserialize_message(data,get_message(metadata.type))
                headers[topic].append({'stamp':msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,'frame':msg.header.frame_id})
            rows.append((stamp,metadata,data));counts[topic]+=1
    assert edges['map->odom']==0 and edges['odom->base_footprint']>0 and tf_static>0
    writer=rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(target),storage_id='mcap'),rosbag2_py.ConverterOptions('cdr','cdr'))
    created=set()
    for stamp,metadata,data in sorted(rows,key=lambda v:v[0]):
        if metadata.name not in created:writer.create_topic(metadata);created.add(metadata.name)
        writer.write(metadata.name,data,stamp)
    del writer
    audit={'old_model':'261027_1711','light':light,'raw_sensor_bag':str(source),'frozen_vo_bag':str(vo),
           'counts':dict(counts),'tf_edges':dict(edges),'tf_static_messages':tf_static,
           'header_ranges':{k:{'min':min(v['stamp'] for v in vals),'max':max(v['stamp'] for v in vals),
                              'frames':sorted({v['frame'] for v in vals})} for k,vals in headers.items()},
           'record_ns_range':[min(v[0] for v in rows),max(v[0] for v in rows)],
           'map_tf_excluded':True,'wheel_odom_excluded':True,'truth_input':False,
           'sha256':{str(p):hashlib.sha256(p.read_bytes()).hexdigest() for d in [source,vo] for p in d.glob('*.mcap')}}
    (OUT/f'slam_input_{light}_audit.json').write_text(json.dumps(audit,ensure_ascii=False,indent=2))
    print(light,dict(counts),flush=True)
