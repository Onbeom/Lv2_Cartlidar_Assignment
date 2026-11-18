"""링크 로컬 hull을 한 번 만든 뒤 관측시각 FK로 광학좌표 평면을 갱신한다."""
import numpy as np
from scipy.spatial import ConvexHull

def load_local(robot):
    return [(link,T,V,ConvexHull(V).equations) for link,items in robot.shapes.items() for T,V in items]

def prepare(robot,local,q,frame,k,camera_pose=None):
    poses=robot.link_poses(q);C=np.linalg.inv(poses[frame] if camera_pose is None else camera_pose)
    planes=[];lohi=[];bounds=[];offsets=[0]
    for link,T,V,eq in local:
        M=C@poses[link]@T;R=M[:3,:3];t=M[:3,3]
        v=V@R.T+t;lo=v.min(0)-.005;hi=v.max(0)+.005
        if hi[2]<=0:continue
        if lo[2]<=0:b=(0,640,0,480)
        else:
            corners=np.array([[x,y,z] for x in (lo[0],hi[0]) for y in (lo[1],hi[1]) for z in (lo[2],hi[2])])
            p=corners[:,:2]/corners[:,2,None]*[k['fx'],k['fy']]+[k['cx'],k['cy']]
            a=np.floor(p.min(0)).astype(int)-1;z=np.ceil(p.max(0)).astype(int)+2
            b=(max(0,int(a[0])),min(640,int(z[0])),max(0,int(a[1])),min(480,int(z[1])))
        if b[0]>=b[1] or b[2]>=b[3]:continue
        n=eq[:,:3]@R.T;d=eq[:,3]-n@t
        planes.extend(np.c_[n,d]);offsets.append(len(planes));lohi.append(np.r_[lo,hi]);bounds.append(b)
    return (np.asarray(planes,np.float32),np.asarray(offsets,np.int32),np.asarray(lohi,np.float32),
            np.asarray(bounds,np.int32),np.array([k['fx'],k['fy'],k['cx'],k['cy']],np.float32))
