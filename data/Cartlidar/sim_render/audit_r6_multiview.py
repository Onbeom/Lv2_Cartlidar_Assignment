"""재생 완료 결과를 판정/실제 ID집합 기준으로 함께 감사한다. 관측을 수정하지 않는다."""
import argparse,json
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('directory',type=Path);p.add_argument('--expected-groups',type=int,default=72);a=p.parse_args()
rows=[json.loads(line) for line in (a.directory/'rows.jsonl').read_text().splitlines()]
keys=[(r['group'],r['light']) for r in rows]
ids=lambda r:sorted(q['tag_id'] for q in r['result']['pucks'])
summary={'groups':len(rows),'expected_groups':a.expected_groups,'unique_groups':len(set(keys)),
 'decision_match':sum(bool(r['match']) for r in rows),
 'id_set_match':sum(ids(r)==r['placed_tags'] for r in rows),
 'unexpected_ids':sum(len(set(ids(r))-set(r['placed_tags'])) for r in rows),
 'missing_frames':sum(len(r['missing_frames']) for r in rows),
 'failures':[{'group':r['group'],'light':r['light'],'decision_match':r['match'],'id_set_match':ids(r)==r['placed_tags'],'conflicts':r['result']['conflicts']} for r in rows if not r['match'] or ids(r)!=r['placed_tags']],
 'simulation_only':True}
summary['pass']=len(rows)==a.expected_groups and len(set(keys))==len(rows) and not summary['failures']
(a.directory/'audit.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2))
print(json.dumps(summary,ensure_ascii=False))
