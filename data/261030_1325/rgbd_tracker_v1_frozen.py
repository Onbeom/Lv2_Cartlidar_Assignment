"""odom 좌표 3D 추적 시제품. 영상/정답/로봇 명령을 읽지 않는다."""
from collections import Counter
from dataclasses import dataclass

import numpy as np
from scipy.optimize import linear_sum_assignment
from scipy.stats import chi2


@dataclass
class Detection:
    xyz: np.ndarray
    class_id: int
    tag_id: int | None = None  # 상위 처리의 실제 해독/기하 검증을 통과한 값


@dataclass
class Track:
    track_id: int
    class_id: int
    tag_id: int | None
    x: np.ndarray
    covariance: np.ndarray
    last_seen: float


class RGBDTracker:
    def __init__(self, sigma=.02, acceleration_sigma=1., max_distance=.25,
                 max_gap=2., ambiguity_delta=1.):
        self.sigma=sigma
        self.acceleration_sigma=acceleration_sigma
        self.max_distance=max_distance
        self.max_gap=max_gap
        self.ambiguity_delta=ambiguity_delta
        self.chi2_gate=float(chi2.ppf(.99,3))
        self.tracks=[]
        self.tag_memory={}
        self.next_id=1
        self.stamp=None

    def update(self, stamp, detections):
        if not np.isfinite(stamp) or (self.stamp is not None and stamp<=self.stamp):
            return [{'valid':False,'reason':'non_increasing_stamp'} for _ in detections]
        dt=0. if self.stamp is None else stamp-self.stamp
        self.stamp=stamp
        self.tracks=[t for t in self.tracks if stamp-t.last_seen<=self.max_gap]
        F=np.eye(6);F[:3,3:]=np.eye(3)*dt
        G=np.vstack((np.eye(3)*dt**2/2,np.eye(3)*dt))
        Q=G@G.T*self.acceleration_sigma**2
        for track in self.tracks:
            track.x=F@track.x
            track.covariance=F@track.covariance@F.T+Q
        outputs=[None]*len(detections)
        tags=Counter(d.tag_id for d in detections if d.tag_id is not None)
        valid=[]
        for j,d in enumerate(detections):
            if np.asarray(d.xyz).shape!=(3,) or not np.isfinite(d.xyz).all() or d.class_id not in range(4):
                outputs[j]={'valid':False,'reason':'invalid_detection'}
            elif d.tag_id is not None and tags[d.tag_id]>1:
                outputs[j]={'valid':False,'reason':'duplicate_tag'}
            else:valid.append(j)
        costs=np.full((len(self.tracks),len(detections)),1e9)
        R=np.eye(3)*self.sigma**2
        for i,t in enumerate(self.tracks):
            S=t.covariance[:3,:3]+R
            for j in valid:
                d=detections[j]
                if d.class_id!=t.class_id:continue
                if d.tag_id is not None and t.tag_id is not None and d.tag_id!=t.tag_id:continue
                if d.tag_id is not None and d.tag_id in self.tag_memory and self.tag_memory[d.tag_id]!=t.track_id:continue
                delta=np.asarray(d.xyz)-t.x[:3]
                distance=np.linalg.norm(delta)
                mahalanobis=float(delta@np.linalg.solve(S,delta))
                if distance<=self.max_distance and mahalanobis<=self.chi2_gate:
                    costs[i,j]=mahalanobis
        ambiguous_rows=set();ambiguous_cols=set()
        for i,row in enumerate(costs):
            ids=np.argsort(row)
            if len(ids)>=2 and row[ids[1]]<1e9 and row[ids[1]]-row[ids[0]]<=self.ambiguity_delta:
                ambiguous_rows.add(i)
                ambiguous_cols.update(int(v) for v in ids if row[v]<=row[ids[0]]+self.ambiguity_delta)
        for j in valid:
            col=costs[:,j];ids=np.argsort(col)
            if len(ids)>=2 and col[ids[1]]<1e9 and col[ids[1]]-col[ids[0]]<=self.ambiguity_delta:
                ambiguous_cols.add(j)
                ambiguous_rows.update(int(v) for v in ids if col[v]<=col[ids[0]]+self.ambiguity_delta)
        for j in ambiguous_cols:outputs[j]={'valid':False,'reason':'ambiguous_association'}
        for i in ambiguous_rows:
            for j in valid:
                if costs[i,j]<1e9 and outputs[j] is None:
                    outputs[j]={'valid':False,'reason':'ambiguous_association'}
        blocked={j for j,o in enumerate(outputs) if o is not None}
        safe=costs.copy()
        if ambiguous_rows:safe[list(ambiguous_rows),:]=1e9
        if blocked:safe[:,list(blocked)]=1e9
        used=set()
        if safe.size:
            ii,jj=linear_sum_assignment(safe)
            for i,j in zip(ii,jj):
                if safe[i,j]>=1e9:continue
                t=self.tracks[i];d=detections[j]
                S=t.covariance[:3,:3]+R
                K=np.linalg.solve(S,t.covariance[:3,:]).T
                t.x+=K@(np.asarray(d.xyz)-t.x[:3])
                H=np.zeros((3,6));H[:3,:3]=np.eye(3)
                A=np.eye(6)-K@H
                t.covariance=A@t.covariance@A.T+K@R@K.T
                t.last_seen=stamp
                if d.tag_id is not None:
                    t.tag_id=d.tag_id;self.tag_memory[d.tag_id]=t.track_id
                outputs[j]={'valid':True,'track_id':t.track_id,'tag_id':t.tag_id,'xyz':t.x[:3].tolist(),'reason':'matched'}
                used.add(j)
        for j in valid:
            if j in used or outputs[j] is not None:continue
            d=detections[j]
            if d.tag_id is not None and any(t.tag_id==d.tag_id for t in self.tracks):
                outputs[j]={'valid':False,'reason':'known_tag_outside_gate'}
                continue
            track_id=self.tag_memory.get(d.tag_id) if d.tag_id is not None else None
            if track_id is None:track_id=self.next_id;self.next_id+=1
            if d.tag_id is not None:self.tag_memory[d.tag_id]=track_id
            t=Track(track_id,d.class_id,d.tag_id,np.r_[d.xyz,np.zeros(3)],
                    np.diag([self.sigma**2]*3+[.5**2]*3),stamp)
            self.tracks.append(t)
            outputs[j]={'valid':True,'track_id':track_id,'tag_id':d.tag_id,'xyz':list(map(float,d.xyz)),'reason':'new_or_tag_return'}
        return outputs
