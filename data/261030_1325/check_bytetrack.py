"""공식 Ultralytics ByteTrack을 고정 모의측정의 2D투영과 비교. GPU 사용 없음."""
import hashlib
import json
import os
import sys
import time
from collections import defaultdict
from pathlib import Path
from types import SimpleNamespace
import numpy as np
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE/'tracker_deps'));os.environ['YOLO_OFFLINE']='true'
from ultralytics.trackers.byte_tracker import BYTETracker
from ultralytics.engine.results import Boxes

BASE=HERE.parent/'Cartlidar/sim_render/results/followup_261028_1457/improvement_261030_1325'
rows=json.loads((BASE/'tracker_v2_holdout/holdout_kf_hungarian_tag_rows.json').read_text())
grouped=defaultdict(list)
for row in rows:grouped[row['stamp']].append(row)
tracker=BYTETracker(SimpleNamespace(track_high_thresh=.25,track_low_thresh=.1,new_track_thresh=.25,
                                  track_buffer=20,match_thresh=.8,fuse_score=True))
initial={};last={};switches=wrong=valid=missing=0;durations=[];outputs=[]
for stamp,items in sorted(grouped.items()):
    boxes=[]
    for item in items:
        x,y,z=item['xyz'];cx=400*x/z+320;cy=400*y/z+240;w=400*.03/z;h=400*.06/z
        boxes.append([cx-w/2,cy-h/2,cx+w/2,cy+h/2,.9,0])
    result=Boxes(np.asarray(boxes,np.float32).reshape(-1,6),(480,640))
    start=time.perf_counter();tracks=tracker.update(result);durations.append(time.perf_counter()-start)
    matched={int(t[-1]):int(t[4]) for t in tracks}
    for j,item in enumerate(items):
        identity=item['truth_for_evaluation'];tid=matched.get(j)
        if tid is None:missing+=1
        else:
            valid+=1
            if identity in last:switches+=last[identity]!=tid
            if identity in initial:wrong+=initial[identity]!=tid
            else:initial[identity]=tid
            last[identity]=tid
        outputs.append({'stamp':stamp,'truth_for_evaluation':identity,'track_id':tid})
summary={'simulation_only':True,'algorithm':'Ultralytics ByteTrack','package_version':'8.4.189',
    'input_detections':len(rows),'valid':valid,'missing':missing,'valid_ratio':valid/len(rows),
    'id_switch_events':switches,'outputs_not_initial_identity':wrong,
    'p95_update_ms':float(np.percentile(durations,95)*1000),'config':vars(tracker.args),
    'limitations':'고정모의측정2D투영·score.9가정,실제YOLO bbox/깊이/태그/저신뢰검출미검증. 3D/태그를쓰는후보와정보량이다름.',
    'script_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
(BASE/'bytetrack_summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2))
(BASE/'bytetrack_rows.json').write_text(json.dumps(outputs,ensure_ascii=False,indent=2))
print(json.dumps(summary,ensure_ascii=False))
