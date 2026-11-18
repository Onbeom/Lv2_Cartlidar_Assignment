"""차체 한 자리에서 9칸을 차례로 보는 실제 팔 보기 자세 (정적 기구학·충돌 확인)."""
import numpy as np
from arm_pose import ARM_J

def plan_views_fixed(ar,cells,top,height=0.18,aim_v=145,max_bases=48):
    order=[4,0,1,2,3,5,6,7,8]
    desired=np.eye(4);desired[:3,:3]=np.diag([1.,-1.,-1.]);desired[:3,3]=[cells[4][0],cells[4][1],top+height]
    seeds=[[0.,0.3,0.8,0.5,0.],[0.,-0.3,1.2,0.9,0.],[0.,0.6,0.3,1.,0.],[0.,0.,0.5,1.4,1.5],[0.,0.,0.5,1.4,-1.5]]
    tried=0;ar.backoff_max=0.01;best_count=0
    for base in ar.base_candidates(desired,[]):
        if ar.body_hits(base):continue
        tried+=1
        if tried>max_bases:break
        frames=[];previous=None
        for cell in order:
            target=np.array([cells[cell][0],cells[cell][1],top+0.06]);T=desired.copy();T[:2,3]=target[:2]
            found=None
            for iteration in range(5):
                found=ar.frame(base,T,([previous] if previous is not None else [])+seeds)
                if found is None:break
                q,b,actual=found
                p=np.linalg.inv(actual)@np.append(target,1.)
                v=283*p[1]/p[2]+240
                if abs(v-aim_v)<=5:break
                T=actual.copy();T[:3,3]+=actual[:3,1]*(v-aim_v)*p[2]/283
            if found is None or abs(v-aim_v)>5:continue
            previous=q
            covered=[]
            inverse=np.linalg.inv(actual)
            for k,xy in enumerate(cells):
                corners=[inverse@np.array([xy[0]+dx,xy[1]+dy,top+0.0602,1.]) for dx in (-0.008,0.008) for dy in (-0.008,0.008)]
                if all(pt[2]>0.025 and 32<=283*pt[0]/pt[2]+320<=608 and 32<=283*pt[1]/pt[2]+240<=235 for pt in corners):covered.append(k)
            frames.append({'q':dict(zip(ARM_J,map(float,q))),'backoff_m':b,'T_world_cam':actual.tolist(),'aim_v_px':float(v),'view_cell':cell,'covered_cells':covered})
        if len(frames)>best_count:
            best_count=len(frames);print("R6 고정차체 검색",tried,"시야",best_count,"/9",flush=True)
        coverage=set(k for frame in frames for k in frame['covered_cells'])
        if len(coverage)==9:return {'T_world_base_footprint':base.tolist(),'frames':frames,'tried_bases':tried}
    return None


def plan_views_moving(ar,cells,top,height=0.20,aim_v=145,max_bases=12):
    frames=[]
    for cell in [4,0,1,2,3,5,6,7,8]:
        target=np.array([cells[cell][0],cells[cell][1],top+0.06])
        T=np.eye(4);T[:3,:3]=np.diag([1.,-1.,-1.]);T[:3,3]=[target[0],target[1],top+height]
        ar.backoff_max=0.05
        real=ar.realize_aim(T,[],target,v_px=aim_v)
        if real is None:
            # 큰 중앙 받침대는 카메라를 칸 바로 위로 놓기 어려워 가장자리에서 비스듬히 본다.
            for dy in (0.06,0.12,0.18):
                trial=T.copy();trial[1,3]+=dy
                axis=target-trial[:3,3];axis/=np.linalg.norm(axis)
                x=np.array([1.,0.,0.]);x-=axis*(x@axis);x/=np.linalg.norm(x)
                trial[:3,0]=x;trial[:3,1]=np.cross(axis,x);trial[:3,2]=axis
                real=ar.realize_aim(trial,[],target,v_px=aim_v)
                if real is not None:break
        if real is None:
            print('R6 접근점검색실패',cell,flush=True);return None
        frame=dict(real['frames'][0]);frame.update({'T_world_base_footprint':real['T_world_base_footprint'],'aim_v_px':real['aim_v_px'],'view_cell':cell})
        frames.append(frame)
    return {'T_world_base_footprint':frames[0]['T_world_base_footprint'],'frames':frames,'tried_bases':None}


_CACHE={}
def plan_views(ar,cells,top,height=0.20,aim_v=145,max_bases=12):
    key=(id(ar.scene),tuple(np.asarray(cells).ravel()),height,aim_v)
    if key in _CACHE:
        cached=_CACHE[key]
        if all(not ar.body_hits(np.array(frame['T_world_base_footprint'])) and not ar.arm_hits(np.array(frame['T_world_base_footprint']),[frame['q'][j] for j in ARM_J]) for frame in cached['frames']):
            return cached
    fixed=plan_views_fixed(ar,cells,top,height,aim_v,min(max_bases,4))
    if fixed is not None:
        for frame in fixed['frames']:frame['T_world_base_footprint']=fixed['T_world_base_footprint']
        _CACHE[key]=fixed
        return fixed
    moving=plan_views_moving(ar,cells,top,height,aim_v,max_bases)
    if moving is not None:_CACHE[key]=moving
    return moving
