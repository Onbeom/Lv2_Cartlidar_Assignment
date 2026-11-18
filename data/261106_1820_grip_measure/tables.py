import json,numpy as np,sys,cv2,math
sys.path.insert(0,'.'); import measure as M
meta=json.load(open('r_judge/meta.json'));F={f['id']:f for f in meta['frames']};K=meta['K'];T=np.array(meta['T_G_cam'])
# 윗면 앞 / 아랫면 앞 가장자리의 축 밖 각 (왜곡 0)
ang={'top':[],'bot':[]}; rows={}
for f in meta['frames']:
    TG=np.array(f['T_G_puck']);R,c=TG[:3,:3],TG[:3,3]
    top=M.top_far_points(f['shape'],R,c); bot=top-2*M.HALF_L*R[:,2]
    uvt=M.project(top,K,T); uvb=M.project(bot,K,T)
    ang['top'].append(max(math.atan((uvt[:,1].min()-360)/424.5),0)*180/math.pi); ang['bot'].append(math.atan((uvb[:,1].min()-360)/424.5)*180/math.pi)
    rows[f['id']]=(uvt[:,1].min(),uvb[:,1].min())
for k in ang: print(k,'축 밖 각 중앙',round(np.median(ang[k]),1),'최대',round(max(ang[k]),1))
print('f별: 화면 세로 반각, 아랫면 앞 가장자리가 화면 안인 장 %, 윗면 앞 %, 필요 광축 기울기(지금 15°, 최악 + 1° 여유)')
for f_ in (424.5,473,503,560,640,800):
    half=math.degrees(math.atan(360/f_)); b=np.mean(np.array(ang['bot'])<half)*100; t=np.mean(np.array(ang['top'])<half)*100
    print(f_,round(half,1),round(b),round(t),'아랫면만',round(15+max(ang['bot'])-half+1,1),'윗면까지',round(15+max(ang['top'])-half+1,1))
# 윗면 앞 화면 안 / 밖 나눔
for name in sys.argv[1:]:
    r=json.load(open(f'judge.json.{name}.rows.json'))
    out=[]
    for sh in('cylinder','cuboid'):
        a=[x for x in r if x['shape']==sh and 'fail' not in x]; i=[abs(x['ehor']) for x in a if rows[x['id']][0]<712]; o=[abs(x['ehor']) for x in a if rows[x['id']][0]>=712]
        out.append(f"{sh[:3]} 안 n{len(i)} p95 {np.percentile(i,95):.2f} 밖 n{len(o)} p95 {np.percentile(o,95):.2f}")
    print(name,*out)
