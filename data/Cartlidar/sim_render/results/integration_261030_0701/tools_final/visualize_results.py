"""저장된 새모델 A1 판정과 실제 렌더를 사진으로 모은다. 채점/입력은 수정하지 않는다."""
import json
from pathlib import Path
from PIL import Image,ImageDraw,ImageFont
R=Path('/home/pa4/git/lv2_next_module');O=R/'Cartlidar/sim_render/results/a1_new_model_261029';P=R/'photo'
font='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
f=lambda n:ImageFont.truetype(font,n)
im=Image.new('RGB',(1600,1240),'#f4f6fa');d=ImageDraw.Draw(im)
d.text((48,30),'새 모델 A1 · 인지 정렬 검증 (모의)',font=f(36),fill='#132b46')
d.text((48,88),'같은 물체의 여러 면 = 동일 태그 ID  |  모터 명령 없음  |  실물 K/D 미적용',font=f(22),fill='#41516a')
rows=[('설계 v8','design/replay'),('설계 v9','design/replay_v9'),('설계 v10','design/replay_v10'),('설계 v12','design/replay_v12'),('기존 실패시야 v12','independent/replay_v12'),('새 별도시야 v12','validation_v12/replay_isolated')]
for i,(label,path) in enumerate(rows):
 y=150+i*76;p=O/path/'perception_summary.json'
 d.text((48,y),label,font=f(25),fill='#132b46')
 if not p.exists():d.text((310,y),'진행 중 / 결과 없음',font=f(23),fill='#6a7482');continue
 s=json.loads(p.read_text());v=s['P2'];rate=v['rate'];color='#16896d' if s['pass_observed_checks'] else '#ca6d32'
 d.rounded_rectangle((310,y+7,310+800*rate,y+39),radius=8,fill=color)
 d.text((1130,y),f"{v['ok']}/{v['n']} ({rate:.2%})",font=f(25),fill='#132b46')
 d.text((310,y+40),f"다른 퍽 {s['P3_other_frames']}건 · 최초잠금 {s['P1']['ok']}/{s['P1']['n']} · 복귀 {s['P4']['ok']}/{s['P4']['n']}",font=f(18),fill='#41516a')
d.line((310+800*.95,142,310+800*.95,610),fill='#333333',width=2)
d.text((1065,120),'기준 95%',font=f(19),fill='#333333')
d.text((48,650),'같은 설계 장면 · 실제 팔 카메라 입력과 정렬값',font=f(27),fill='#132b46')
meta=json.loads((O/'design/render_a1.json').read_text());scene=meta['scenes'][0]
rs=[json.loads(x) for x in (O/'design/replay_v11/a1_rows.jsonl').read_text().splitlines()];rs={r['k']:r for r in rs if r['scene']==scene['name']}
ks=[0,next(a['k'] for a in scene['arm_frames'] if a['pucks'].get('puck_10',{}).get('in_frac',0)==0),len(scene['arm_frames'])-1]
for i,(k,title) in enumerate(zip(ks,['목표 관측','화면 밖 · 정렬 보류','복귀 · 재획득'])):
 a=scene['arm_frames'][k];pic=Image.open(O/'design'/a['image']).convert('RGB');pd=ImageDraw.Draw(pic);w,h=pic.size;r=rs[k]
 if r['offset_valid']:
  x=(r['offset'][0]+1)*w/2;y=(r['offset'][1]+1)*h/2
  pd.ellipse((x-13,y-13,x+13,y+13),outline='#00ff9c',width=4)
 pic=pic.resize((480,270));x=48+i*512;im.paste(pic,(x,705))
 d.text((x,989),f"{title} · 프레임 {k}",font=f(23),fill='#132b46')
 d.text((x,1025),f"정렬 유효: {'예' if r['offset_valid'] else '아니요'}",font=f(21),fill='#41516a')
d.text((48,1095),'P2: 목표가 화면 안에 있고 절반 이상 보일 때, 목표 일치·정렬 유효·중심 오차 ≤ 0.05.',font=f(21),fill='#41516a')
d.text((48,1132),'원본 J 판정과 계획/실제 목표 불일치는 결과 JSON에 별도 보존. 같은 6월드의 별도 시야 시험.',font=f(21),fill='#41516a')
d.text((48,1169),'기존 실패시야 v11은 초기 목표 연결 11/12로 실패. 수정 후 자료 재검증과 새 시야 검증을 구분.',font=f(21),fill='#41516a')
P.mkdir(exist_ok=True);path=P/'261030_새모델_A1_인지검증.png';im.save(path);print(path)
