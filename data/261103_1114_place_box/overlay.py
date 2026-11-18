"""검출(빨강) · 정답(초록) 윗면 사각형을 원본 영상에 겹쳐 그림 (도구 확인)."""
import json, math, sys, numpy as np, cv2
sys.path.insert(0, sys.path[0]); import detect_box as D
d, names, out = sys.argv[1], sys.argv[2].split(','), sys.argv[3]
meta = json.load(open(f"{d}/render_place_box.json")); K = np.array([[meta["K"]["fx"], 0, meta["K"]["cx"]], [0, meta["K"]["fy"], meta["K"]["cy"]], [0, 0, 1]])
T = np.array(meta["T_bf_cam"]); Hm = D.plane_homography(K, T, D.BOX_H); tiles = []
def sq(c, y):
    R = np.array([[math.cos(y), -math.sin(y)], [math.sin(y), math.cos(y)]])
    P = np.array([np.asarray(c) + R @ np.array(s) * D.BOX / 2 for s in ((1, 1), (-1, 1), (-1, -1), (1, -1))])
    q = (Hm @ np.c_[P, np.ones(4)].T).T; return (q[:, :2] / q[:, 2:]).astype(np.int32)
for sc in meta["scenes"]:
    if sc["image"] not in names: continue
    img = cv2.imread(f"{d}/{sc['image']}"); r = D.detect(img, K, T, (0.2553, 0.0))
    cv2.polylines(img, [sq(sc["center"], sc["yaw"])], True, (0, 255, 0), 3); cv2.polylines(img, [sq(r["center"], r["yaw"])], True, (0, 0, 255), 2)
    cv2.putText(img, sc["image"], (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 0), 3); tiles.append(cv2.resize(img, (640, 360)))
while len(tiles) % 2: tiles.append(np.zeros_like(tiles[0]))
cv2.imwrite(out, np.vstack([np.hstack(tiles[i:i + 2]) for i in range(0, len(tiles), 2)]))
