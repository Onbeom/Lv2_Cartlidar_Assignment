"""기록된 관측(survey.observations.jsonl)을 등록부 변형에 다시 넣어 회마다 완료 · 칸 일치를 본다. 도구 확인용."""
import importlib.util, json, sys, collections
def load(path):
    spec=importlib.util.spec_from_file_location('core_'+str(abs(hash(path))),path); m=importlib.util.module_from_spec(spec); spec.loader.exec_module(m); return m
def run(core, replay):
    runs=[json.loads(l) for l in open(replay+'/b0_runs.jsonl')]
    obs=collections.OrderedDict()
    for l in open(replay+'/survey.observations.jsonl'):
        o=json.loads(l); obs.setdefault(o['survey_id'],[]).append(o)
    out=[]
    for r in runs:
        sv=core.Survey(frame='mission_map',gate_m=.035,min_observations=2,tagged_min_confidence=.4)
        for o in obs.get(r['survey_id'],[]): sv.observe(o['observation_m'],o['stamp_ns']/1e9)
        snap=sv.snapshot(); ok=0; wrong=[]
        slots={(s['box'],s['side']):s for s in snap['slots']}
        for c in r['cells']:
            s=slots[(c['box'],c['side'])]
            got={k:s[k] for k in ('id','color','shape','pose')}
            if got==c['want']: ok+=1
            elif all(v is not None for v in got.values()): wrong.append((c['name'],c['want'],got))
        wc=snap['complete'] and ok<12
        out.append({'complete':snap['complete'],'cells_ok':ok,'wrong_complete':wc,'errors':snap['errors'],'wrong_cells':wrong,
                    'same_as_recorded':snap['complete']==r['complete']})
    return out
if __name__=='__main__':
    core=load(sys.argv[1]); T=collections.Counter()
    for rp in sys.argv[2:]:
        for k,x in enumerate(run(core,rp)):
            T['runs']+=1; T['pass']+=x['complete'] and x['cells_ok']==12; T['cells_ok']+=x['cells_ok']; T['wrong_complete']+=x['wrong_complete']
            T['same_as_recorded']+=x['same_as_recorded']; T['wrong_cells_full']+=len(x['wrong_cells'])
            if not x['complete'] or x['cells_ok']<12: print(rp.split('/')[-1],k,x['errors'],x['wrong_cells'])
    print(dict(T))
