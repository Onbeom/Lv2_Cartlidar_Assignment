"""45도로 쥔 퍽의 팔 카메라 화면 위치 (기하 계산, 렌더 아님). 값은 기준 URDF: gripper_roll_link 기준."""
import numpy as np
cam=np.array([0.100256,0,0.077446]); p=0.261788
ax=np.array([np.cos(p),0,-np.sin(p)]); yd=np.array([-np.sin(p),0,-np.cos(p)])  # 광축, 화면 아래 방향
tool=np.array([0.139,0,0]); s=np.sqrt(.5)
up=np.array([-s,0,s]); fw=np.array([s,0,s])   # 집게 45도 아래일 때 월드 위 · 월드 앞(수평)
c=tool-0.015*up
pts={"퍽 중심":c,"윗면 중심(태그)":c+.03*up,"윗면 앞 가장자리":c+.03*up+.015*fw,"윗면 뒤 가장자리":c+.03*up-.015*fw,
     "앞 옆면 가운데":c+.015*fw,"아랫면 앞 가장자리":c-.03*up+.015*fw,"잡는 점(tool)":tool}
res=[("640x480 f283",283,480),("1280x720 f424",424.5,720),("1280x720 f566(세로 잘림 가정)",566,720)]
print("점 | Z mm | 축 밖 deg | "+" | ".join(r[0] for r in res)+" (v px, 화면 안=O)")
for n,q in pts.items():
    d=q-cam; Z=d@ax; Y=d@yd
    print(n,"|",round(Z*1e3,1),"|",round(np.degrees(np.arctan2(Y,Z)),1),"|"," | ".join(f"{h/2+f*Y/Z:.0f} {'O' if h/2+f*Y/Z<h else 'X'}" for _,f,h in res))
print("\n집게 방향 1 mm 이동 시 dv px, 폭 30 mm 의 px, 면 법선과 시선 각")
nrm={"윗면 앞 가장자리":up,"앞 옆면 가운데":fw,"아랫면 앞 가장자리":fw,"윗면 중심(태그)":up}
for n,nm in nrm.items():
    q=pts[n]; d=q-cam; Z=d@ax; Y=d@yd; x=np.array([1e-3,0,0]); dq=d+x
    ang=np.degrees(np.arccos((-d/np.linalg.norm(d))@nm))
    print(n,"|"," | ".join(f"dv {f*((dq@yd)/(dq@ax)-Y/Z):.1f} 폭 {f*.03/Z:.0f}" for _,f,h in res),"| 법선각",round(ang,1))
