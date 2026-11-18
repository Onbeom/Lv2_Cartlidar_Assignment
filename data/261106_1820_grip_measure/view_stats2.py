"""보는 자세 넣은 평가를 다시 집계. 원본의 slip 은 월드 위 기준이라 집게를 들어 올리면 18 mm 가 저절로 생겨 'seated' 를 망친다.
그래서 칸 판정은 슬립 조건을 빼고(xy 3 mm · 높이 3 mm · 기울기 5°) 같은 식으로, 손 안 변화는 집게 좌표로 본다."""
import json,sys,numpy as np
def st(a):
    a=np.array(a,float); return f"{np.median(a):.2f}/{np.percentile(a,95):.2f}/{a.max():.1f}"
print('파일 | 집음 | 칸에 앉음 원기둥/상자 | 손에서 빠짐(>15 mm) | 넘겨받기→앞 운반 손 안 변화 mm 중앙/p95/최대 | 기울기 변화 ° | 정착 p95 s | 몸체 N | 상자 N')
for f in sys.argv[1:]:
    d=json.load(open(f)); R=[r for r in d['results'] if r.get('picked')]; v={x['env']:x for x in d['view']}
    ok=lambda r: r['xy_err_mm']<3 and abs(r['bottom_above_floor_mm'])<3 and r['tilt_deg']<5
    seat={s:sum(ok(r) for r in R if r['shape']==s) for s in('cyl','box')}
    ch=[];ti=[];lost=[]
    for r in R:
        x=v[r['env']]; a=np.array(x['handover_to_view_mm'])+np.array(x['view_to_front_mm']); ch.append(np.linalg.norm(a)); ti.append(x['tilt_change_view_deg']+x['tilt_change_front_deg'])
        if ch[-1]>15: lost.append(r['env'])
    s=d['summary']
    print(f.replace('chain_','').replace('.json',''),'|',len(R),'|',seat['cyl']+seat['box'],f"({seat['cyl']}/{seat['box']})",f"{100*(seat['cyl']+seat['box'])/len(R):.0f}%",'|',len(lost),'|',st(ch),'|',st(ti),'|',round(np.percentile([x['settle_05mm_s'] for x in d['view']],95),2),'|',s['arm_body_max_N'],'|',s['arm_box_transit_max_N'])
