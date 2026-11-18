"""C++ place_box_check 로 렌더 자료를 돌려 정답 · Python 시제품 결과와 견줌 (도구 확인)."""
import json, math, re, subprocess, sys, numpy as np
d, exe, pylog = sys.argv[1], sys.argv[2], sys.argv[3]
meta = json.load(open(f"{d}/render_place_box.json")); K = meta["K"]; T = np.array(meta["T_bf_cam"]).reshape(-1)
py = {}
for l in open(pylog):
    m = re.match(r"(\S+): 중심 ([\d.]+) mm 요 ([+-][\d.]+)° 칸 최대 ([\d.]+)", l)
    if m: py[m.group(1)] = float(m.group(4))
def cells(c, y):
    R = np.array([[math.cos(y), -math.sin(y)], [math.sin(y), math.cos(y)]])
    return np.array([np.asarray(c) + R @ np.array([dx, dy]) * 0.06 for dx in (-1, 0, 1) for dy in (-1, 0, 1)])
rows = []
for sc in meta["scenes"]:
    out = subprocess.run([exe, f"{d}/{sc['image']}", *map(str, (K["fx"], K["fy"], K["cx"], K["cy"])), *map(str, T), "0.2553", "0.0"], capture_output=True, text=True).stdout.split()
    x, y, yaw = map(float, out[:3])
    dc = 1000 * math.hypot(x - sc["center"][0], y - sc["center"][1]); dy = math.degrees((yaw - sc["yaw"] + math.pi / 4) % (math.pi / 2) - math.pi / 4)
    de = 1000 * np.linalg.norm(cells((x, y), yaw) - cells(sc["center"], sc["yaw"]), axis=1).max()
    rows.append((dc, abs(dy), de, de - py.get(sc["image"], np.nan)))
a = np.array(rows); ok = ((a[:, 0] <= 5) & (a[:, 1] <= 1) & (a[:, 2] <= 6)).sum()
print(f"C++ 통과 {ok}/{len(a)}")
for i, k in enumerate(("중심 mm", "요 °", "칸 최대 mm")): print(f"  {k}: 중앙 {np.median(a[:, i]):.2f} p95 {np.percentile(a[:, i], 95):.2f} 최대 {a[:, i].max():.2f}")
print(f"  칸 최대 오차 C++ − Python: 중앙 {np.nanmedian(a[:, 3]):+.2f} 최대 |차| {np.nanmax(np.abs(a[:, 3])):.2f} mm")
