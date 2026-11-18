"""GPU ROI 점-볼록형상 검사. 정답/마스크를 입력하지 않는다."""
import numpy as np
import torch
import triton
import triton.language as tl

@triton.jit
def _inside(DEPTH,PLANES,OFFSETS,LOHI,BOUNDS,OUTPUT,
            FX:tl.constexpr,FY:tl.constexpr,CX:tl.constexpr,CY:tl.constexpr,BLOCK:tl.constexpr):
    hull=tl.program_id(0);chunk=tl.program_id(1)
    x0=tl.load(BOUNDS+hull*4);x1=tl.load(BOUNDS+hull*4+1)
    y0=tl.load(BOUNDS+hull*4+2);y1=tl.load(BOUNDS+hull*4+3)
    width=x1-x0;index=chunk*BLOCK+tl.arange(0,BLOCK)
    valid=index<width*(y1-y0)
    x=x0+index%width;y=y0+index//width
    pixel=y*640+x
    z=tl.load(DEPTH+pixel,mask=valid,other=0)
    px=(x-CX)*z/FX;py=(y-CY)*z/FY
    valid=valid&(z>0)&(z<float('inf'))
    valid=valid&(px>=tl.load(LOHI+hull*6))&(py>=tl.load(LOHI+hull*6+1))&(z>=tl.load(LOHI+hull*6+2))
    valid=valid&(px<=tl.load(LOHI+hull*6+3))&(py<=tl.load(LOHI+hull*6+4))&(z<=tl.load(LOHI+hull*6+5))
    start=tl.load(OFFSETS+hull);end=tl.load(OFFSETS+hull+1)
    plane=start
    while (plane<end)&(tl.sum(valid.to(tl.int32),0)>0):
        a=tl.load(PLANES+plane*4);b=tl.load(PLANES+plane*4+1);c=tl.load(PLANES+plane*4+2);d=tl.load(PLANES+plane*4+3)
        valid=valid&(px*a+py*b+z*c+d<=.005)
        plane+=1
    tl.atomic_or(OUTPUT+pixel,1,mask=valid)

def filter_depth(depth,planes,offsets,lohi,bounds,k):
    """CPU 배치 입력→GPU 계산→CPU 마스크. 호출자는 CUDA 동기화를 포함해 측정한다."""
    arrs=[torch.as_tensor(np.ascontiguousarray(a),device='cuda') for a in [depth.astype(np.float32),planes,offsets,lohi,bounds]]
    output=torch.zeros(depth.size,device='cuda',dtype=torch.int32)
    spans=(bounds[:,1]-bounds[:,0])*(bounds[:,3]-bounds[:,2])
    grid=(len(bounds),triton.cdiv(int(spans.max()),256))
    _inside[grid](*arrs,output,*map(float,k),256,enable_fp_fusion=False)
    return output.cpu().numpy().astype(bool).reshape(depth.shape)
