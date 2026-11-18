"""옆에서 본 팔 자세 비교 한 장: 집기 직전(두 브라켓 같음) · 내려다보기 지금 URDF · 내려다보기 출력품. 결과 photo/261027_팔카메라_브라켓_팔자세.png"""
import json
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

R = Path("Cartlidar/sim_render/results")
m = json.loads((R / "armcam_bracket_261027/render_armcam_bracket.json").read_text())
F = ImageFont.truetype("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc", 17)
FS = ImageFont.truetype("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc", 15)
ITEMS = [("grasp", 40.83, "집기 직전 (두 브라켓 같은 자세: 집게 수직 아래)"),
         ("look", 40.83, "내려다보기 — 지금 URDF (광축 40.83°)"),
         ("look", 15.0, "내려다보기 — 출력품 (판-장착면 75°, 광축 약 15°)")]
w, h = 640, 480
sheet = Image.new("RGB", (3 * w + 40, h + 110), "white")
dr = ImageDraw.Draw(sheet)
for i, (pose, ang, title) in enumerate(ITEMS):
    e = next(x for x in m["renders"] if x["pose"] == pose and abs(x["angle"] - ang) < 1e-6)
    x0 = 10 + i * (w + 10)
    dr.text((x0, 6), title, fill="black", font=F)
    sheet.paste(Image.open(R / f"armcam_bracket_side_261027/side_{pose}_{ang:g}.png"), (x0, 32))
    q = e["q_deg"]
    dr.text((x0, 36 + h), f"어깨 {q['RMX52_Shoulder']}° · 팔꿈치 {q['RMX52_Elbow']}° · 손목 pitch {q['RMX52_Wrist_Pitch']}°", fill="black", font=F)
dr.text((10, h + 66), "초록 선 = 팔 카메라 광축(0.15 m), 노란 점 = 집게 가운데(TCP). 로봇 오른쪽에서 본 모의 렌더, 충돌 검사 안 함. 관절은 기준 자세(모두 0) 대비",
        fill="black", font=FS)
dr.text((10, h + 86), "내려다보기 = 카메라 광축을 수직 아래로, 퍽 윗면 10 cm 위 (READ_TAG 판정 렌더와 같은 꼴) — 수치 IK", fill="black", font=FS)
sheet.save("photo/261027_팔카메라_브라켓_팔자세.png")
