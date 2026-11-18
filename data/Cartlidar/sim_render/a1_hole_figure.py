"""설명 그림: 팔 카메라로 위에서 본 퍽의 색 덩어리와 라벨 구멍, 채움 비율(지금 / 구멍 채움). 결과 photo/261027_팔검출_라벨구멍.png"""
import json
import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

R = 'Cartlidar/sim_render/results/render_a1_tool_261027/'
d = json.load(open(R + 'render_a1.json'))
HSV = {'red': [([0, 80, 35], [10, 255, 255]), ([170, 80, 35], [179, 255, 255])], 'green': [([40, 80, 35], [85, 255, 255])]}
FONT = ImageFont.truetype('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc', 17)


def crop(scene, k, name, col, half=34):
    sc = [s for s in d['scenes'] if s['name'] == scene][0]
    f = sc['arm_frames'][k]
    img = cv2.imread(R + f['image'])
    b = f['pucks'][name]['box']
    cx, cy = int((b[0] + 1) * 320), int((b[1] + 1) * 240)
    c = img[cy - half:cy + half, cx - half:cx + half].copy()
    hsv = cv2.cvtColor(cv2.GaussianBlur(c, (3, 3), 0), cv2.COLOR_BGR2HSV)
    m = np.zeros(c.shape[:2], np.uint8)
    for lo, hi in HSV[col]:
        m |= cv2.inRange(hsv, np.array(lo), np.array(hi))
    k3 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    m = cv2.morphologyEx(cv2.morphologyEx(m, cv2.MORPH_OPEN, k3), cv2.MORPH_CLOSE, k3)
    cs, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnt = min(cs, key=lambda q: cv2.pointPolygonTest(q, (half, half), True) * -1)   # 가운데를 감싼 덩어리
    one = np.zeros_like(m)
    cv2.drawContours(one, [cnt], -1, 255, -1)
    blob = cv2.bitwise_and(m, one)
    x, y, w, h = cv2.boundingRect(cnt)
    fill = cv2.countNonZero(blob[y:y + h, x:x + w]) / (w * h)
    fillh = cv2.countNonZero(one[y:y + h, x:x + w]) / (w * h)

    def show(mask):
        v = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        cv2.rectangle(v, (x, y), (x + w - 1, y + h - 1), (0, 200, 255), 1)
        return v
    tiles = [cv2.resize(t, (204, 204), interpolation=cv2.INTER_NEAREST) for t in (c, show(blob), show(one))]
    return tiles, ['원본 (팔 카메라)', f'지금 채움 {fill:.2f}', f'구멍 채운 뒤 {fillh:.2f}']


rows = [crop('w1_in00_L1', 0, 'puck_06', 'red'), crop('w1_out01_L1', 0, 'puck_07', 'green')]
canvas = Image.new('RGB', (3 * 214 + 10, 2 * 236 + 40), 'white')
dr = ImageDraw.Draw(canvas)
for r, (tiles, labs) in enumerate(rows):
    for i, (t, lab) in enumerate(zip(tiles, labs)):
        canvas.paste(Image.fromarray(cv2.cvtColor(t, cv2.COLOR_BGR2RGB)), (10 + i * 214, 10 + r * 236))
        dr.text((10 + i * 214, 10 + r * 236 + 206), lab, fill='black', font=FONT)
dr.text((10, 2 * 236 + 12), '흰 칸 = 색으로 잡힌 화소, 주황 선 = 외접 상자, 하한 0.35', fill='black', font=FONT)
canvas.save('photo/261027_팔검출_라벨구멍.png')
