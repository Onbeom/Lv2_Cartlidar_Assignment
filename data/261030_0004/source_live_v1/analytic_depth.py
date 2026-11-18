"""2D 모의 로봇의 이동 카메라로 보는 복도·동적 상자의 분석식 깊이."""
import numpy as np
W,H,FX=320,240,200.
vv,uu=np.indices((H,W))
by=-(uu-W/2)/FX
bz=-(vv-H/2)/FX
def render(x,y,yaw,elapsed):
    c,s=np.cos(yaw),np.sin(yaw);rays=np.stack((c-s*by,s+c*by,bz),-1);origin=np.array([x,y,.45]);depth=np.full((H,W),np.inf)
    with np.errstate(divide='ignore',invalid='ignore'):
        for axis,value in [(0,0.),(0,3.),(1,-.5),(1,.5),(2,0.)]:
            distance=(value-origin[axis])/rays[:,:,axis];depth=np.minimum(depth,np.where(distance>.105,distance,np.inf))
    center=None
    if 1<=elapsed<2:center=.9*(2-elapsed)
    elif 2<=elapsed<26:center=0.
    elif 26<=elapsed<27:center=-.9*(elapsed-26)
    if center is not None:
        lo=np.array([1.025,center-.25,0.]);hi=np.array([1.375,center+.25,1.8]);enter=np.full((H,W),-np.inf);leave=np.full((H,W),np.inf)
        with np.errstate(divide='ignore',invalid='ignore'):
            for axis in range(3):
                a=(lo[axis]-origin[axis])/rays[:,:,axis];b=(hi[axis]-origin[axis])/rays[:,:,axis];enter=np.maximum(enter,np.minimum(a,b));leave=np.minimum(leave,np.maximum(a,b))
        depth=np.minimum(depth,np.where((leave>=enter)&(enter>.105),enter,np.inf))
    return np.where(np.isfinite(depth)&(depth<=10),depth,0.).astype(np.float32)
