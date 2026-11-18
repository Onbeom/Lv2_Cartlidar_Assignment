import json,sys
d=json.load(open(sys.argv[1]))
print('보임',d['visibility'])
for k,v in d.items():
    if k=='visibility':continue
    print(k,*[f"{s[:3]}: n{v[s]['n']} fail{v[s]['fail']} hor {v[s]['ehor'][0]}/{v[s]['ehor'][1]} bot {v[s]['ebot'][0]}/{v[s]['ebot'][1]} x {v[s]['ex'][0]}/{v[s]['ex'][1]} up {v[s]['eup'][0]}/{v[s]['eup'][1]} th {v[s]['eth'][0]}/{v[s]['eth'][1]}" for s in v])
