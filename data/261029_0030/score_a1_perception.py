"""A1 원본 J 판정을 보존하고 사용자 담당 인지 출력만 보조 판정. 정답은 채점에만 사용."""
import json,sys,math
from pathlib import Path
render,replay=map(Path,sys.argv[1:3]);meta=json.loads((render/'render_a1.json').read_text());scenes={s['name']:s for s in meta['scenes']};ss=[json.loads(x) for x in (replay/'a1_scenes.jsonl').read_text().splitlines()];rows=[json.loads(x) for x in (replay/'a1_rows.jsonl').read_text().splitlines()]
print(ss[0]) if '--schema' in sys.argv else None
result=dict(P1=dict(ok=0,n=0),P2=dict(ok=0,n=0),P3_other_frames=0,P3_hidden=dict(ok=0,n=0),P4=dict(ok=0,n=0),planned_selected_mismatch=[],failures=[])
for scene in ss:
 name=scene['scene'] if 'scene' in scene else scene['name'];s=scenes[name];target=scene.get('tracked_target',scene.get('tracked'));rs=[r for r in rows if r['scene']==name];frames={f['k']:f for f in s['arm_frames']}
 if not target:raise RuntimeError(('실제선택목표없음 또는 스키마불일치',scene))
 if target!=s['target']:result['planned_selected_mismatch'].append(dict(scene=name,planned=s['target'],selected=target))
 def truth(r):return frames[r['k']]['pucks'].get(target,dict(in_frac=0,vis=0,box=None))
 first=next((r for r in rs if r['offset_valid']),None);result['P1']['n']+=1;result['P1']['ok']+=bool(first and first.get('pointed')==target)
 for r in rs:
  t=truth(r);b=t['box'];offset=r.get('offset');same=r.get('pointed')==target
  if t['in_frac']==1 and t['vis']>=.5:
   result['P2']['n']+=1;err=math.hypot(offset[0]-b[0],offset[1]-b[1]) if offset and b else math.inf;ok=r['offset_valid'] and same and err<=.05;result['P2']['ok']+=ok
   if not ok:result['failures'].append(dict(scene=name,k=r['k'],check='P2',offset_valid=r['offset_valid'],error=err if math.isfinite(err) else None))
  if r['offset_valid'] and r.get('pointed')!=target:result['P3_other_frames']+=1
  if t['vis']==0:
   result['P3_hidden']['n']+=1;result['P3_hidden']['ok']+=not r['offset_valid']
 outside=False;back=None
 for r in rs:
  t=truth(r)
  if t['in_frac']==0:outside=True
  if outside and t['in_frac']==1 and t['vis']>=.5:back=r['k'];break
 if back is not None:
  result['P4']['n']+=1;ok=any(r['offset_valid'] and r.get('pointed')==target for r in rs if back<=r['k']<back+3);result['P4']['ok']+=ok
  if not ok:result['failures'].append(dict(scene=name,k=back,check='P4'))
for key in ['P1','P2','P3_hidden','P4']:
 v=result[key];v['rate']=v['ok']/v['n'] if v['n'] else None
result['pass_observed_checks']=result['P1']['rate']==1 and result['P2']['rate'] is not None and result['P2']['rate']>=.95 and result['P3_other_frames']==0 and result['P3_hidden']['ok']==result['P3_hidden']['n'] and (result['P4']['n']==0 or result['P4']['rate']>=.95)
result['P4_exercised']=result['P4']['n']>0;result['note']='인지범위 보조 판정. 기존 J판정 및 계획/선택 불일치 기록 보존. P4없음은 시험안함.'
(replay/'perception_summary.json').write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n');print(json.dumps(result,ensure_ascii=False))
