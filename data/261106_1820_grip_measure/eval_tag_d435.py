"""앞 D435 + 윗면 태그 PnP 만으로 손 안 퍽 자세 (모의). 집게 자세는 (a) 정답 (b) 명령 관절 FK 두 가지로 본다."""
import json,math,sys,cv2,numpy as np
sys.path.insert(0,'.'); import measure as M
folder=sys.argv[1]; meta=json.load(open(folder+'/meta.json'));K=meta['K'];Km=np.array([[K['fx'],0,K['cx']],[0,K['fy'],K['cy']],[0,0,1]])
det=cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)); P=cv2.aruco.DetectorParameters(); P.cornerRefinementMethod=cv2.aruco.CORNER_REFINE_SUBPIX; det.setDetectorParameters(P)
h=.008; obj=np.array([[-h,h,0],[h,h,0],[h,-h,0],[-h,-h,0]],np.float32)
Tb=np.array(meta['T_BF_cam']); rng=np.random.default_rng(5)
def run(fr,noise,use):
    im=cv2.imread(f"{folder}/{fr['id']}.png")
    if noise: im=np.clip(im.astype(np.float32)+rng.normal(0,2,im.shape),0,255).astype(np.uint8)
    cs,ids,_=det.detectMarkers(cv2.cvtColor(im,cv2.COLOR_BGR2GRAY))
    if ids is None: return None
    TG=np.array(fr['T_bf_G']); Tcmd=np.array(fr['T_bf_G_cmd']); T_cam_G_true=np.linalg.inv(TG)@Tb
    T_use=(T_cam_G_true if use=='true' else np.linalg.inv(Tcmd)@Tb)
    TGp=np.array(fr['T_G_puck']); ax=TGp[:3,2]*np.sign(TGp[:3,2]@M.Z_P); top=TGp[:3,3]+.03*ax
    uv=M.project(top[None],K,T_cam_G_true)[0]
    dd=[np.linalg.norm(c.reshape(4,2).mean(0)-uv) for c in cs]; i=int(np.argmin(dd))
    if dd[i]>40: return None
    n,rv,tv,err=cv2.solvePnPGeneric(obj,cs[i].reshape(4,2).astype(np.float32),Km,None,flags=cv2.SOLVEPNP_IPPE_SQUARE)
    best=None
    for r,t in zip(rv,tv):
        R,_=cv2.Rodrigues(r); nG=T_use[:3,:3]@R[:,2]; cG=T_use[:3,:3]@t.ravel()+T_use[:3,3]
        s=nG@M.Z_P
        if best is None or abs(s)>best[0]: best=(abs(s),nG*np.sign(s),cG)
    _,nG,cG=best; c_e=cG-.0302*nG
    e=c_e-TGp[:3,3]; b=(c_e-.03*nG)-(TGp[:3,3]-.03*ax)
    return dict(hor=abs(e[0]+e[2])*M.S*1e3,bot=abs(b[0]+b[2])*M.S*1e3,x=abs(e[0])*1e3,z=abs(e[2])*1e3,y=abs(e[1])*1e3,th=math.degrees(math.acos(np.clip(nG@ax,-1,1))),px=np.linalg.norm(cs[i].reshape(4,2)[0]-cs[i].reshape(4,2)[1]))
out={}
for po in ('D','A'):
  for sh in('cylinder','cuboid'):
    F=[f for f in meta['frames'] if f['pose']==po and f['shape']==sh]
    for noise,use in((0,'true'),(1,'true'),(1,'cmd')):
        R=[run(f,noise,use) for f in F]; ok=[r for r in R if r]
        s={k:(round(float(np.median([r[k] for r in ok])),2),round(float(np.percentile([r[k] for r in ok],95)),2)) for k in('hor','bot','x','z','y','th','px')}
        out[f'{po}/{sh}/{"noise" if noise else "clean"}/{use}']=dict(n=len(F),found=len(ok),**s)
        print(po,sh[:3],'잡음' if noise else '깨끗','집게=정답' if use=='true' else '집게=FK','찾음',len(ok),'/',len(F),'수평',s['hor'],'바닥',s['bot'],'x',s['x'],'z',s['z'],'기울기',s['th'],'태그변 px',s['px'][0])
json.dump(out,open('dj_tag.json','w'),indent=1,ensure_ascii=False)
