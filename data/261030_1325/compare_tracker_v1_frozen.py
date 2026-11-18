"""코드 고정 후 별도 모의입력과 최근접 기준 비교. 실제 재식별 성능 아님."""
import hashlib
import json
import time
from pathlib import Path
import numpy as np
from scipy.optimize import linear_sum_assignment
from rgbd_tracker import Detection,RGBDTracker

ROOT=Path('/home/pa4/git/lv2_next_module')
OUT=ROOT/'Cartlidar/sim_render/results/followup_261028_1457/improvement_261030_1325/tracker_comparison'
OUT.mkdir(exist_ok=False)
class Nearest:
    def __init__(self):self.tracks={};self.next=1
    def update(self,stamp,detections):
        self.tracks={k:v for k,v in self.tracks.items() if stamp-v[1]<=2.}
        ids=list(self.tracks);used=set();got=[None]*len(detections)
        if ids and detections:
            cost=np.array([[np.linalg.norm(self.tracks[i][0]-d.xyz) for d in detections] for i in ids])
            ii,jj=linear_sum_assignment(cost)
            for i,j in zip(ii,jj):
                if cost[i,j]>.25:continue
                tid=ids[i];self.tracks[tid]=(detections[j].xyz,stamp);used.add(j)
                got[j]={'valid':True,'track_id':tid}
        for j,d in enumerate(detections):
            if j not in used:
                tid=self.next;self.next+=1;self.tracks[tid]=(d.xyz,stamp);got[j]={'valid':True,'track_id':tid}
        return got

def sequence(seed,variant):
    rng=np.random.default_rng(seed);rows=[]
    for frame in range(121):
        t=frame*.1
        truth=[('a',[-.6+.1*t,0,1.],10),('b',[.6-.1*t,0,1.],11)]
        if variant=='holdout':truth.append(('c',[0.,-.48+.08*t,1.],12))
        truth=[v for v in truth if not (v[0]=='a' and 20<=frame<32)]
        rng.shuffle(truth)
        observations=[]
        for identity,pos,tag in truth:
            if variant=='holdout' and 10<=frame<80 and rng.random()<.07:continue
            observations.append((identity,Detection(np.array(pos)+rng.normal(0,.01 if variant=='holdout' else .005,3),0,
                 tag if frame<10 or frame>=80 else None)))
        rows.append((t,observations))
    return rows
reports=[]
for variant,seed in [('design',20261030),('holdout',20261230)]:
    inputs=sequence(seed,variant)
    for name,tracker in [('nearest3d',Nearest()),('kf_hungarian_tag',RGBDTracker())]:
        first={};last={};switches=wrong=valid=hold=0;durations=[];raw=[]
        for stamp,observations in inputs:
            start=time.perf_counter();outputs=tracker.update(stamp,[v[1] for v in observations]);durations.append(time.perf_counter()-start)
            for (identity,d),result in zip(observations,outputs):
                if result['valid']:
                    valid+=1;tid=result['track_id']
                    if identity in last:switches+=last[identity]!=tid
                    if identity in first:wrong+=first[identity]!=tid
                    else:first[identity]=tid
                    last[identity]=tid
                else:hold+=1
                raw.append({'stamp':stamp,'truth_for_evaluation':identity,'xyz':d.xyz.tolist(),'visible_decoded_tag':d.tag_id,'output':result})
        report={'variant':variant,'seed':seed,'algorithm':name,'input_detections':valid+hold,'valid':valid,'held':hold,
                'valid_ratio':valid/(valid+hold),'id_switch_events':switches,'outputs_not_initial_identity':wrong,
                'p95_update_ms':float(np.percentile(durations,95)*1000)}
        reports.append(report);(OUT/f'{variant}_{name}_rows.json').write_text(json.dumps(raw,ensure_ascii=False,indent=2))
        print(json.dumps(report,ensure_ascii=False),flush=True)
(OUT/'summary.json').write_text(json.dumps({'simulation_only':True,'rows':reports,
    'tracker_sha256':hashlib.sha256((Path(__file__).parent/'rgbd_tracker.py').read_bytes()).hexdigest(),
    'limitations':'설계자료와별도모의난수. 실제YOLO깊이/TF/태그오류·ByteTrack·ROS연동·실물미검증. 보류율과함께해석.'},ensure_ascii=False,indent=2))
