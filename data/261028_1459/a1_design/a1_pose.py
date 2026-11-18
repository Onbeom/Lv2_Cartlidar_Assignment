"""A1 개인 자세 도구: 2D 전체 윤곽 대신 실제 3D 차체·장애물 충돌로 후보 검사."""
import math
import numpy as np
from arm_pose import ArmRealizer
from urdf_chain import planar_pose

class VolumeArmRealizer(ArmRealizer):
    def base_candidates(self,T_des,floor_xy):
        point=np.asarray(T_des)[:3,3][:2];x0,x1,y0,y1=self.scene.bounds
        out=[]
        for radius in (.16,.20,.24,.28,.32,.36,.40,.44):
            for a in np.radians(np.arange(0,360,15)):
                xy=point+radius*np.array([math.cos(a),math.sin(a)])
                base=planar_pose(*xy,a+math.pi)
                body=np.vstack([v@base[:3,:3].T+base[:3,3] for _,v in self.body])
                if body[:,0].min()<x0 or body[:,0].max()>x1 or body[:,1].min()<y0 or body[:,1].max()>y1:continue
                # self.boxes에는 벽·박스·모든퍽이 높이와5mm여유를 포함해 들어 있음.
                if self.body_hits(base):continue
                out.append((radius,base))
        hint=getattr(self,'view_hint',None)
        if hint is not None:
            hint=np.asarray(hint,float);hint=hint/max(np.linalg.norm(hint),1e-9)
            out.sort(key=lambda rb:(abs(float(rb[1][:2,0]@hint)),abs(rb[0]-.24)))
        else:out.sort(key=lambda rb:abs(rb[0]-.24))
        return [base for _,base in out]

    def realize(self,*args,**kwargs):
        if getattr(self,'view_hint',None) is not None:kwargs.setdefault('max_bases',24)
        return super().realize(*args,**kwargs)
