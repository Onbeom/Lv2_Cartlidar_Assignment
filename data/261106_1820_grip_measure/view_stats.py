import json,sys,numpy as np
for f in sys.argv[1:]:
    d=json.load(open(f)); s=d['summary']; v=d['view']
    print(f,'q_view',d.get('q_view'),'성공',s['chain']['ok'],'/',s['chain']['n'],'단계',s['stages'],'팔-몸체',s['arm_body_max_N'],'팔-상자',s['arm_box_transit_max_N'])
    def st(a): a=np.array(a,float); return f"중앙 {np.median(a):.2f} p95 {np.percentile(a,95):.2f} 최대 {a.max():.2f}"
    print(' n',len(v),'정착 0.5mm s',st([x['settle_05mm_s'] for x in v]),'| 끝 0.3s 흔들림 mm',st([x['tail_pp_mm'] for x in v]))
    print(' 머무는 동안 손 안',st([x['hand_during_hold_mm'] for x in v]))
    print(' 넘겨받기→보기 mm',st([np.linalg.norm(x['handover_to_view_mm']) for x in v]),'| 보기→앞 운반 mm',st([np.linalg.norm(x['view_to_front_mm']) for x in v]))
    print(' 기울기 변화 ° 보기',st([x['tilt_change_view_deg'] for x in v]),'| 앞 운반',st([x['tilt_change_front_deg'] for x in v]))
    print(' 관절 오차 rad',st(np.abs([x['joint_err_view_rad'][:4] for x in v]).ravel()),'| 시간 s 보기까지',st([x['t_to_view_s'] for x in v]),'전체 넘겨받기',st([x['t_transit_s'] for x in v]))
    for sh in('cyl','box'):
        w=[x for x in v if x['shape']==sh]; print(' ',sh,len(w),'단계',{k:sum(x['stage']==k for x in w) for k in ('transit_drop','carry_slip','not_seated','seated')})
