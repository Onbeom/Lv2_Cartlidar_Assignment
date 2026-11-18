import json
from pathlib import Path
root=Path(__file__).resolve().parent;reports=[]
for seed in range(1,7):
 p=root/f'r6_fresh_seed{seed}_render';meta=json.loads((p/'render_r6.json').read_text())
 for row in meta['poses']:row['images']={k:str((p/v).resolve()) for k,v in row['images'].items()}
 reports.append(meta)
merged=dict(reports[0]);merged['poses']=[p for m in reports for p in m['poses']];assert len(merged['poses'])==648
out=root/'r6_fresh_render';out.mkdir(exist_ok=True);(out/'render_r6.json').write_text(json.dumps(merged,indent=1,ensure_ascii=False))
print('독립 혼합자세 영상',len(merged['poses']))
