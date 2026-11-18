"""render_armcam_bracket.py 결과 6장을 한 장으로: 열 = 브라켓(지금 URDF · 출력품), 행 = 자세. 결과 photo/261027_팔카메라_브라켓비교.png"""
import json
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

R = Path(sys.argv[1] if len(sys.argv) > 1 else "Cartlidar/sim_render/results/armcam_bracket_261027")
d = json.loads((R / "render_armcam_bracket.json").read_text())
F = ImageFont.truetype("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc", 18)
FS = ImageFont.truetype("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc", 15)
COLS = [(40.83, "지금 URDF: 판-장착면 49.17° · 광축 40.83°"), (15.0, "출력품: 판-장착면 75° · 광축 약 15°")]
ROWS = [("zero", "기준 자세(관절 모두 0)"), ("grasp", "집기 직전(집게 수직 아래, TCP 가 퍽 위 3 cm) — 두 열 팔 자세 같음"),
        ("look", "내려다보기(광축 수직 아래, 퍽 윗면 10 cm 위) — 브라켓마다 팔 자세 다름")]
S = 0.75
NEAR = 0.0   # [m] 렌더에서 --near 2.5 cm 로 잘라냈으므로 빗금 없음 (0.025 로 두면 렌더 깊이로 빗금)
w, h = int(640 * S), int(480 * S)
top, lab = 34, 48
sheet = Image.new("RGB", (2 * w + 30, top + 3 * (h + lab) + 48), "white")
dr = ImageDraw.Draw(sheet)
for ci, (ang, title) in enumerate(COLS):
    dr.text((10 + ci * (w + 10), 6), title, fill="black", font=F)
for ri, (pose, title) in enumerate(ROWS):
    y = top + ri * (h + lab)
    dr.text((10, y), title, fill="black", font=F)
    for ci, (ang, _) in enumerate(COLS):
        e = next(x for x in d["renders"] if x["pose"] == pose and abs(x["angle"] - ang) < 1e-6)
        a = np.asarray(Image.open(R / e["image"]).convert("RGB")).copy()
        dep = np.load(R / e["image"].replace(".png", "_depth.npy"))
        near = dep < NEAR   # 카메라 자기 몸체: 브라켓과 같이 도는 부분이라 실제로는 이 자리에 안 보임
        yy, xx = np.mgrid[:a.shape[0], :a.shape[1]]
        a[near] = np.where((((xx + yy) // 8) % 2 == 0)[near][:, None], [150, 150, 150], [205, 205, 205])
        im = Image.fromarray(a.astype(np.uint8)).resize((w, h))
        sheet.paste(im, (10 + ci * (w + 10), y + 24))
        q = e["q_deg"]
        dr.text((10 + ci * (w + 10), y + 24 + h), f"손목 pitch {q['RMX52_Wrist_Pitch']}° · 어깨 {q['RMX52_Shoulder']}° · 팔꿈치 {q['RMX52_Elbow']}°",
                fill="black", font=FS)
dr.text((10, sheet.height - 38), "노란 십자 = 집게 가운데(TCP). 카메라에서 2.5 cm 안(렌즈 둘레 카메라 몸체 메시)은 렌더에서 잘라냄 — 출력품 열은 광학 프레임만 돌려",
        fill="black", font=FS)
dr.text((10, sheet.height - 20), "몸체가 시야에 들어오는데 실제로는 브라켓과 같이 돌아 안 보임. 모의 렌더(Isaac Sim, b_dice seed_1 선반 2), 640×480 · f 283 px 가정, 충돌 검사 안 함",
        fill="black", font=FS)
sheet.save("photo/261027_팔카메라_브라켓비교.png")
