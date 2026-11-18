import json
from pathlib import Path
from PIL import Image,ImageDraw,ImageFont
R=Path('/home/pa4/git/lv2_next_module');O=R/'Cartlidar/sim_render/results/integration_261030_1319';D=json.loads((O/'probe_full_v4/summary.json').read_text());F='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc';font=lambda n:ImageFont.truetype(F,n)
im=Image.new('RGB',(1600,1240),'#f5f7fa');p=ImageDraw.Draw(im)
p.text((44,25),'기존 후보 인지 연결 검증 · PC 모의',font=font(35),fill='#12324a')
p.text((44,78),'실제 인지 노드 → 실제 PuckArray → 실제 B0 등록·저장  |  정답 메시지 주입 없음',font=font(23),fill='#40576a')
metrics=[('저장영상 부분등록',f"{sum(x['pass_'] for x in D['scenes'])}/6"),('A1 정적 목표 일치',f"{sum(len(x['offset_targets']) for x in D['scenes'])}/24"),('원시합성 완료',f"{D['fixture']['fields_matched']}/12"),('다른 투영크기 완료',f"{D['fixture_scale_holdout']['fields_matched']}/12")]
for i,(label,value) in enumerate(metrics):
 x=44+i*390;p.rounded_rectangle((x,130,x+370,250),radius=12,fill='#e1f1eb');p.text((x+18,144),label,font=font(23),fill='#155845');p.text((x+18,180),value,font=font(37),fill='#155845')
p.text((44,280),'설정·점수 의미의 연결 누락을 발견하고 대조',font=font(28),fill='#12324a')
rows=[('기본 태그 설정 / B0 기존 문턱','probe_full_v2'),('기존 R6 검증 설정 / B0 기존 문턱','probe_full_v3'),('기존 R6 설정 / 태그 점유율 문턱 명시','probe_full_v4')]
for i,(label,path) in enumerate(rows):
 d=json.loads((O/path/'summary.json').read_text())['fixture'];y=338+i*68;p.text((44,y),label,font=font(23),fill='#40576a');p.text((905,y),f"일치 {d['fields_matched']}/12",font=font(24),fill='#148568' if d['fields_matched']==12 else '#a7602b');p.text((1160,y),f"최대 {d['max_position_error_mm']:.3f} mm",font=font(24),fill='#40576a')
p.text((44,560),'아래는 연결 검사용 이상적 합성 RGB 입력 · 실제 카메라 사진/Isaac 렌더 아님',font=font(24),fill='#12324a')
for i,k in enumerate([1,2]):
 image=Image.open(O/'probe_full_v4/fixtures'/f'box_{k}.png').resize((640,480));x=92+i*760;im.paste(image,(x,610));p.text((x,1100),'원기둥 · 빨강/초록' if k==1 else '직육면체 · 빨강/초록',font=font(22),fill='#40576a')
p.text((44,1148),f"원시합성 최대 오차 {D['fixture']['max_position_error_mm']:.3f} mm / 크기변경 {D['fixture_scale_holdout']['max_position_error_mm']:.3f} mm · 연결/오류/재시작 검사 {sum(D['checks'].values())}/{len(D['checks'])}",font=font(22),fill='#40576a')
p.text((44,1187),'색상 대체 검출+태그 · 시험용 TF · 최종 YOLO 엔진/SLAM 정확도/실물 판정과 구분',font=font(22),fill='#40576a')
path=R/'photo/261030_기존후보_인지연결검증.png';im.save(path);print(path)
