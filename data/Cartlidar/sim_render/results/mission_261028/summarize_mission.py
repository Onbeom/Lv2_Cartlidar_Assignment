"""영속 결과에서 모의 판정 요약 생성. 진행 중 시험은 통과로 표시하지 않는다."""
from pathlib import Path
import json
ROOT=Path(__file__).resolve().parent

def read(path):
    return json.loads(path.read_text())

r1=read(ROOT/'replay_r1/summary.json')
g=list(r1['by_dist'].values())
summary={'kind':'모의 판정','source_sha':read(ROOT/'build_provenance.json')['source_sha'],
         'r1':{'complete':True,'frames':r1['frames'],'correct':sum(x['correct'] for x in g),
               'false_light':sum(x['puck_as_light_off_frames'] for x in g),'position_far':sum(x['position_far_frames'] for x in g),
               'max_position_error_m':max(x['max_pos_err_m'] for x in g),
               'pass':all(x['accuracy']>=0.98 and x['puck_as_light_off_frames']==0 and x['position_far_frames']==0 for x in g)}}
p=ROOT/'replay_r2/summary.json'
rows=[json.loads(s) for s in (ROOT/'replay_r2/r2_progress.jsonl').read_text().splitlines()]
summary['r2']={'complete':p.exists(),'planned_runs':40,'finished_runs':len(rows),'correct':sum(r['correct'] for r in rows),
               'wrong_confirmations':sum(r['wrong_confirmations'] for r in rows),
               'not_confirmed':sum(not r['confirmed'] for r in rows),'within_limit':sum(r['within_limit'] for r in rows),
               'max_confirm_s':max((r['confirm_after_cycles']*r['period'] for r in rows if r['confirmed']),default=None),
               'max_confirm_cycles':max((r['confirm_after_cycles'] for r in rows if r['confirmed']),default=None),
               'failures':[r for r in rows if not r['correct'] or not r['within_limit'] or r['wrong_confirmations']]}
summary['r2']['pass']=p.exists() and len(rows)==40 and all(r['correct'] and r['within_limit'] and not r['wrong_confirmations'] for r in rows)
render=next(ROOT/n for n in ('render_r3_full','render_r3_complete','render_r3_final','render_r3_gpu') if (ROOT/n/'zbar_summary.json').exists())
z=read(render/'zbar_summary.json');m=read(render/'render_r3.json')
summary['r3']={**z,'render':str(render),'labels':len(m['labels']),'planned_conditions':12,
               'rendered_conditions':len({(p['label'],p['tilt_deg']) for p in m['poses']}),
               'planned_frames':72, 'rendered_frames':len(m['poses']),
               'by_tilt':{str(t):{'frames':sum(p['tilt_deg']==t for p in m['poses'])} for t in (0,15)},
               'height_range_m':[min(l['center'][2] for l in m['labels']),max(l['center'][2] for l in m['labels'])],
               'pass':z['pass_visible']}
(ROOT/'summary.json').write_text(json.dumps(summary,indent=1,ensure_ascii=False))
lines=['# 발제 맵 모의 판정 요약','',f"인지 SHA `{summary['source_sha']}`",'',
       '| 시험 | 판정 | 수치 |','|---|---|---|',
       f"| R1′ | {'통과' if summary['r1']['pass'] else '실패'} | {summary['r1']['correct']}/{summary['r1']['frames']}, 켜진 등 오인{summary['r1']['false_light']}, 최대 위치 오차{summary['r1']['max_position_error_m']*1000:.2f}mm |",
       f"| R2′ | {('통과' if summary['r2']['pass'] else '실패') if summary['r2']['complete'] else '진행 중'} | 완료{len(rows)}/40, 정답{summary['r2']['correct']}, 미확정{summary['r2']['not_confirmed']}, 틀린 확정{summary['r2']['wrong_confirmations']} |",
       f"| R3′ | {'통과' if summary['r3']['pass'] else '실패'} | 보임≥0.95 {z['correct']}/{z['eligible']}, 틀린 문자열{z['wrong_strings']}, 렌더한 조건{summary['r3']['rendered_conditions']}/12 |",'',
       'R2′는 26초 주기, 3주기78초 상한. R3′는 보이는 라벨의 읽기 판정이며, 자세 확보 범위를 따로 기록한다.',
       '결과는 이 폴더의 replay_r1/, replay_r2/, render_r3*와 진단 파일들. 진단 수치는 공식 결과와 구분한다.']
(ROOT/'summary.md').write_text('\n'.join(lines)+'\n')
print(json.dumps({k:v for k,v in summary.items() if k!='r3'},ensure_ascii=False))

# 라벨·기울기별 읽기 수, 자세 검색 범위 표(판정 대상 제외를 숨기지 않음).
zrows=read(render/'zbar_rows.json')
by_name={r[0]:r for r in zrows}
coverage=[]
for label in m['labels']:
    for tilt in (0,15):
        ps=[p for p in m['poses'] if p['label']==label['id'] and p['tilt_deg']==tilt]
        correct=sum(by_name[p['name']][3]['1']=='ok' for p in ps if p['label_visible']>=0.95)
        eligible=sum(p['label_visible']>=0.95 for p in ps)
        coverage.append({'label':label['id'],'height_m':label['center'][2],'tilt_deg':tilt,'frames':len(ps),'eligible':eligible,'correct':correct,'d_min':ps[0]['d_min'] if ps else None})
(ROOT/'r3_coverage.json').write_text(json.dumps(coverage,indent=1,ensure_ascii=False))
