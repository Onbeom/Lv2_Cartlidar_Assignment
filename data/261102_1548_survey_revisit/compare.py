"""기록된 관측을 옛 등록부(core_d1, PR #51)와 칸별 다시 보기 등록부(core_revisit)에 똑같이 넣어 결과가 같은지 본다. 도구 확인용."""
import importlib.util, json, sys, collections
def load(p):
    s=importlib.util.spec_from_file_location(p.split('/')[-1][:-3],p);m=importlib.util.module_from_spec(s);s.loader.exec_module(m);return m
old,new=load(sys.argv[1]),load(sys.argv[2])
NEW_SLOT=('problems','status','revisits');NEW_TOP=('revisit','unconfirmed','max_revisits','survey_id')
T=collections.Counter();status=collections.Counter()
for rp in open(sys.argv[3]).read().split():
    try:lines=open(rp+'/survey.observations.jsonl').read().splitlines()
    except FileNotFoundError:T['no_obs_file']+=1;continue
    obs=collections.OrderedDict()
    for l in lines:
        o=json.loads(l);obs.setdefault(o['survey_id'],[]).append(o)
    for sid,os_ in obs.items():
        a=old.Survey(frame='mission_map',gate_m=.035,min_observations=2,tagged_min_confidence=.4)
        b=new.Survey(frame='mission_map',gate_m=.035,min_observations=2,tagged_min_confidence=.4)
        for o in os_:
            ra=a.observe(o['observation_m'],o['stamp_ns']/1e9);rb=b.observe(o['observation_m'],o['stamp_ns']/1e9)
            T['observe_diff']+=ra!=rb
        sa,sb=a.snapshot(),b.snapshot()
        for k in NEW_TOP:sa.pop(k,None);sb.pop(k,None)
        for s in sa['slots']:
            for k in NEW_SLOT:s.pop(k,None)
        for s in sb['slots']:
            status[s['status']]+=1
            for k in NEW_SLOT:s.pop(k,None)
        T['surveys']+=1;T['same']+=sa==sb;T['complete']+=sb['complete']
        if not sb['complete']:print('미완료',rp.split('lv2_next_module/')[1],sb['errors'][:4])
print(dict(T));print('칸 상태',dict(status))
