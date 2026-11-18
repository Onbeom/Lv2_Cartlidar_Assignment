"""실물 Arducam '10 cm' 태그 영상에서 검은 테두리 가장자리의 흐림 σ 를 잰다 (10-90 % 폭 / 2.56). 참고용, 거리는 사용자가 알려 준 값."""
import cv2,glob,numpy as np,os,sys
d=cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11))
for folder in sorted(glob.glob('clips/L*')):
    res=[]
    for f in sorted(glob.glob(folder+'/*.png')):
        g=cv2.imread(f,0).astype(np.float32); cs,ids,_=d.detectMarkers(g.astype(np.uint8))
        if ids is None: continue
        for c in cs:
            c=c.reshape(4,2); cen=c.mean(0)
            for i in range(4):
                a,b=c[i],c[(i+1)%4]; L=np.linalg.norm(b-a)
                if L<40: continue
                t=(b-a)/L; n=np.array([-t[1],t[0]]); n=n if (a-cen)@n>0 else -n
                ws=[]
                for s in np.linspace(.25,.75,9):
                    p0=a+(b-a)*s; prof=[]
                    for k in np.arange(-20,20.01,.5):
                        q=p0+n*k; prof.append(cv2.getRectSubPix(g,(1,1),(float(q[0]),float(q[1])))[0,0])
                    prof=np.array(prof); lo,hi=np.percentile(prof,10),np.percentile(prof,90)
                    if hi-lo<60: continue
                    x=np.arange(-20,20.01,.5); nrm=(prof-lo)/(hi-lo)
                    i10=np.argmax(nrm>.1); i90=np.argmax(nrm>.9)
                    if i90>i10: ws.append(x[i90]-x[i10])
                if ws: res.append(np.median(ws))
    if res: print(os.path.basename(folder),'변 수',len(res),'10-90 폭 px 5/50 % 분위',np.percentile(res,[5,50]).round(1),'σ≈',(np.percentile(res,[5,50])/2.56).round(1))
