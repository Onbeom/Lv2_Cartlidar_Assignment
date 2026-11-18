"""URDF 자기형상 필터 코어. ROS/렌더 정답 입력 없음."""
import numpy as np
def build_cache(k, prepared):
    chunks=[]
    for lo,hi,eq,(x0,x1,y0,y1) in prepared:
        if x0>=x1 or y0>=y1:
            continue
        yy,xx=np.mgrid[y0:y1,x0:x1]
        ids=(yy*640+xx).ravel()
        rays=np.stack(((xx.ravel()-k['cx'])/k['fx'],(yy.ravel()-k['cy'])/k['fy'],np.ones(xx.size)),-1)
        # 기존 판정의 확장 AABB와 hull halfspace를 모두 유지한다.
        normals=np.vstack((eq[:,:3],np.eye(3),-np.eye(3)))
        offsets=np.r_[.005-eq[:,3],hi,-lo]
        lower=np.zeros(len(ids));upper=np.full(len(ids),np.inf);possible=np.ones(len(ids),bool)
        for normal,limit in zip(normals,offsets):
            a=rays@normal
            positive=a>1e-12;negative=a < -1e-12
            upper[positive]=np.minimum(upper[positive],limit/a[positive])
            lower[negative]=np.maximum(lower[negative],limit/a[negative])
            possible &= (positive|negative|(limit>=0))
        keep=possible&(upper>=lower)&(upper>0)
        if keep.any():chunks.append((ids[keep],lower[keep],upper[keep]))
    return chunks

def apply_cache(depth,chunks):
    flat=depth.ravel();mask=np.zeros(flat.size,bool)
    for ids,lo,hi in chunks:
        values=flat[ids]
        selected=np.isfinite(values)&(values>0)&(values>=lo)&(values<=hi)
        mask[ids[selected]]=True
    return mask.reshape(depth.shape)
