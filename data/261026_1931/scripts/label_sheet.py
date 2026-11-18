"""육안 라벨 확인용: 사진마다 신호등 영역을 잘라 라벨과 함께 격자로 (PC)"""
import glob, json, os, sys
import cv2, numpy as np
CAM = sys.argv[1]
base = os.path.expanduser('~/git/lv2_next_module/261026_1931/scripts')
photos = os.path.expanduser('~/git/lv2_next_module/cartlidar/photo')
out = sys.argv[2]
lab = json.load(open(os.path.join(base, 'labels_eye.json')))['labels']
groups = json.load(open(os.path.join(base, 'tower_boxes.json')))['groups']
dirs = sorted(glob.glob(os.path.join(photos, '20261026_1*')))
H, W = 150, 70   # 칸 하나 (영역을 높이 H 로 맞추고 폭 W 안에)
col = {'R': (0, 0, 220), 'Y': (0, 200, 230), 'G': (0, 170, 0), 'off': (90, 90, 90)}
tiles = []
for i, d in enumerate(dirs):
    name = os.path.basename(d)
    box = next(g[2] for g in groups if g[0] <= i <= g[1])
    img = cv2.imread(os.path.join(d, CAM, 'color.png'))
    x0, y0, x1, y1 = box
    c = img[y0:y1, x0:x1]
    s = min((H - 22) / c.shape[0], (W - 4) / c.shape[1])
    c = cv2.resize(c, (max(1, int(c.shape[1] * s)), max(1, int(c.shape[0] * s))), interpolation=cv2.INTER_AREA)
    t = np.full((H, W, 3), 255, np.uint8)
    t[2:2 + c.shape[0], (W - c.shape[1]) // 2:(W - c.shape[1]) // 2 + c.shape[1]] = c
    l = lab.get(name, '?')
    cv2.putText(t, f'{i} {l}', (2, H - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.45, col.get(l, (0, 0, 0)), 1, cv2.LINE_AA)
    tiles.append(t)
per = 29
rows = [np.hstack(tiles[r:r + per] + [np.full((H, W, 3), 255, np.uint8)] * (per - len(tiles[r:r + per]))) for r in range(0, len(tiles), per)]
cv2.imwrite(out, np.vstack(rows))
print(len(tiles), 'tiles', len(lab), 'labels')
