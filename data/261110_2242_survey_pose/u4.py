"""U4: 측량 완료 파일(예제 퍽, B4 +X 직육면체 누움) -> dump_pucks(헤더) -> 등록표 -> 두 계획기.
사용: python3 u4.py <dev mission_planner 폴더> <fix/lie-pick-order mission_planner 폴더>"""
import copy, json, os, subprocess, sys, yaml

dev, lie = sys.argv[1], sys.argv[2]
P = yaml.safe_load(open(os.path.join(lie, 'config/problem_q1_example.yaml'), encoding='utf-8'))
slots = [dict(box=r['box'], side=r['side'], id=r['id'], color=r['color'],
              shape='cuboid' if r['shape'] == 'box' else r['shape'],
              pose='lying' if (r['box'], r['side']) == (4, 1) else 'standing') for r in P['pucks']]
json.dump({'complete': True, 'slots': slots}, open('survey_complete.json', 'w'))
txt = subprocess.run(['./dump_pucks', 'survey_complete.json'], check=True, capture_output=True, text=True).stdout
open('pucks_from_survey.yaml', 'w').write(txt)
reg = yaml.safe_load(txt)['pucks']
print('등록표 pose:', {r['id']: r.get('pose') for r in reg})


def run(root, pucks, **kw):
    code = f"""
import sys, os, json, yaml
sys.path.insert(0, {root!r})
from mission_planner.planner import World, plan
from mission_planner.mission_lines import to_lines
w = World(os.path.join({root!r}, 'worlds/dice_mission.sdf'))
pr = yaml.safe_load(open({os.path.join(lie, 'config/problem_q1_example.yaml')!r}, encoding='utf-8'))
pr['pucks'] = json.loads({json.dumps(pucks)!r})
out = {{m: to_lines(w, plan(w, pr, m), **{kw!r}) for m in ('per_location', 'single_load')}}
print(json.dumps(out, ensure_ascii=False))
"""
    return json.loads(subprocess.run([sys.executable, '-c', code], check=True, capture_output=True, text=True).stdout)


no_pose = [{k: v for k, v in r.items() if k != 'pose'} for r in reg]
a, b = run(dev, reg), run(dev, no_pose)
for m in a:
    print(f'dev {m}: pose 있음 {len(a[m][0])} 줄, 없음 {len(b[m][0])} 줄, 같음 {a[m][0] == b[m][0]}')
c = run(lie, reg, lie_skill='lie_upright')
for m, (lines, notes) in c.items():
    hit = [ln for ln in lines if 'lie_upright' in ln]
    print(f'lie {m}: 누운 퍽 스킬 {len(hit)} 줄 -> {hit}')
