"""A1 집기 전 접근 각도를 바꾸는 개인 도구. 관절 한계·실제 충돌 여유 유지."""
import math
import numpy as np
from scipy.optimize import least_squares
from a1_pose_heading import VolumeArmRealizer
from arm_pose import TCP,POS_TOL,AXIS_TOL

class TiltArmRealizer(VolumeArmRealizer):
    tcp_tilt=0.
    def solve_tcp(self,T_bf,p,roll,seeds):
        a=math.radians(self.tcp_tilt)
        direction=np.asarray(p)[:2]-T_bf[:2,3];direction=direction/max(np.linalg.norm(direction),1e-9)
        axis=np.r_[-math.sin(a)*direction,math.cos(a)]
        lo,hi=[l for l,h in self.lim],[h for l,h in self.lim]
        def residual(v):
            T=self.cam(T_bf,v,'arm_link_5')
            return np.r_[(T@TCP)[:3]-p,.05*(T[:3,2]-axis)]
        for seed in seeds:
            result=least_squares(residual,np.clip(seed,lo,hi),bounds=(lo,hi),xtol=1e-10,ftol=1e-10)
            T=self.cam(T_bf,result.x,'arm_link_5')
            if np.linalg.norm((T@TCP)[:3]-p)<=POS_TOL and math.acos(np.clip(T[:3,2]@axis,-1,1))<=AXIS_TOL:
                yield result.x
