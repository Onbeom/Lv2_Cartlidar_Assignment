"""color_detector.cpp 를 Python 으로 옮긴 진단용 (perception.yaml 값)."""
import cv2, numpy as np
HSV={0:[([0,80,35],[10,255,255]),([170,80,35],[179,255,255])],1:[([40,80,35],[85,255,255])]}
def detect(bgr):
    hsv=cv2.cvtColor(cv2.GaussianBlur(bgr,(3,3),0),cv2.COLOR_BGR2HSV)
    k=cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3,3)); out=[]
    for cls,rs in HSV.items():
        m=np.zeros(hsv.shape[:2],np.uint8)
        for lo,hi in rs: m|=cv2.inRange(hsv,np.array(lo),np.array(hi))
        m=cv2.morphologyEx(cv2.morphologyEx(m,cv2.MORPH_OPEN,k),cv2.MORPH_CLOSE,k)
        cs,_=cv2.findContours(m,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        for c in cs:
            if cv2.contourArea(c)<0.0004*m.size: continue
            x,y,w,h=cv2.boundingRect(c); fill=cv2.countNonZero(m[y:y+h,x:x+w])/max(1,w*h)
            if fill<0.35: continue
            sc=0.5*fill+0.5*min(w,h)/max(w,h)
            if sc>=0.3: out.append(((x+w/2-320)/320,(y+h/2-240)/240,w/640,cls,round(sc,3),round(fill,3)))
    return out
