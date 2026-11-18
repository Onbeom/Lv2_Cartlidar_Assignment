"""설계 모의 입력: 정답은 생성/채점에만, 추적기에는 측정점·선택적 해독태그만 전달."""
import json
import time
from pathlib import Path
import numpy as np
from rgbd_tracker import RGBDTracker, Detection

root=Path('/home/pa4/git/lv2_next_module')
out=root/'Cartlidar/sim_render/results/followup_261028_1457/improvement_261030_1325/tracker_design'
out.mkdir(exist_ok=False)
rng=np.random.default_rng(20261030)
tracker=RGBDTracker()
initial={};switches=0;valid_count=0;hold_count=0;rows=[];durations=[]
for frame in range(101):
    stamp=frame*.1
    truth=[('a',np.array([-.5+.1*stamp,0,1.]),10),('b',np.array([.5-.1*stamp,0,1.]),11)]
    # a가1초가량가렸다가나옴. 실제상대외형은같고태그는처음/후반만보임.
    truth=[v for v in truth if not (v[0]=='a' and 20<=frame<32)]
    rng.shuffle(truth)
    detections=[Detection(pos+rng.normal(0,.005,3),0,tag if frame<10 or frame>=60 else None) for _,pos,tag in truth]
    start=time.perf_counter();got=tracker.update(stamp,detections);durations.append(time.perf_counter()-start)
    for (identity,_,tag),d,result in zip(truth,detections,got):
        if result['valid']:
            valid_count+=1
            if identity in initial:switches+=result['track_id']!=initial[identity]
            else:initial[identity]=result['track_id']
        else:hold_count+=1
        rows.append({'frame':frame,'truth_for_evaluation':identity,'measurement':d.xyz.tolist(),'visible_decoded_tag':d.tag_id,'output':result})

# 중첩모호입력: 두트랙동일예측위치에동일측정을제공하면보류해야함.
t=RGBDTracker()
t.update(0.,[Detection(np.array([-.01,0,1]),0,20),Detection(np.array([.01,0,1]),0,21)])
amb=t.update(.1,[Detection(np.array([0.,0,1]),0),Detection(np.array([0.,0,1]),0)])
restored=t.update(.2,[Detection(np.array([-.01,0,1]),0,20),Detection(np.array([.01,0,1]),0,21)])
duplicate=t.update(.3,[Detection(np.array([-.01,0,1]),0,20),Detection(np.array([.01,0,1]),0,20)])
before=[(v.track_id,v.x.tolist()) for v in t.tracks]
late=t.update(.2,[Detection(np.array([10.,0,1]),0,20)])
after=[(v.track_id,v.x.tolist()) for v in t.tracks]
criteria={'no_identity_switch':switches==0,'ambiguity_held':all(not v['valid'] for v in amb),
          'tag_return_restored':all(v['valid'] for v in restored) and [v['track_id'] for v in restored]==[1,2],
          'duplicate_tag_held':all(not v['valid'] for v in duplicate),
          'late_input_no_state_change':before==after and not late[0]['valid']}
summary={'simulation_only':True,'seed':20261030,'input_detections':len(rows),'valid_outputs':valid_count,
         'held_outputs':hold_count,'id_switches':switches,'criteria':criteria,'pass':all(criteria.values()),
         'chi2_gate_3d_99pct':tracker.chi2_gate,'p95_update_ms':float(np.percentile(durations,95)*1000),
         'limitation':'설계입력·기준odom3D점. 실제YOLO/태그/TF/깊이오류와ByteTrack비교는미완료.'}
(out/'rows.json').write_text(json.dumps(rows,ensure_ascii=False,indent=2))
(out/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2))
print(json.dumps(summary,ensure_ascii=False))
