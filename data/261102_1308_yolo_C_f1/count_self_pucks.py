"""pucks.jsonl 에서 base_link x < 0.35 m 인 D435 퍽(차체 · 집게 자리 헛 퍽)과 그 밖 D435 퍽 수를 센다."""
import json,sys
near=far=0
for rp in sys.argv[1:]:
    for l in open(rp+'/pucks.jsonl'):
        r=json.loads(l)
        if r.get('frame')!='base_link': continue
        for p in r['pucks']:
            if p['camera']!='front_d435': continue
            if p['position'][0]<0.35: near+=1
            else: far+=1
print({'d435_x_lt_0.35':near,'d435_other':far})
