"""미로 배치 데이터 만들기: SDF 기둥 위치 + STL 벽 형상 -> 도면 방향의 위쪽 반 미로 (JSON).

- 기둥 83개 위치는 SDF(post_*)에서 읽는다.
- 이웃한 두 기둥(x 0.734 m, y 0.394 m 간격) 사이에 STL 벽 삼각형이 있으면 판 하나로 본다.
  SDF 충돌체에는 380 mm 판이 없어서 STL로 찾는다.
- 도면(Cartlidar/photos/기본맵사진.jpg)은 SDF/STL을 y 방향으로 뒤집은 모양이다 -> y를 뒤집는다.
- 가운데 공용 벽을 y = 0으로 두고 y >= 0 쪽(위쪽 반 + 가운데 벽)만 남긴다.
- 기둥 종류: 벽 끝·모서리·갈림 기둥(직선으로 이어지는 중간이 아닌 기둥) = AL, 나머지 = 출력 연결부.
  전체 83개 중 AL 30개로 도면 범례(주황 AL 30곳)와 같아야 한다.

실행 (PC): ~/isaacsim-venv/bin/python -I make_maze_layout.py
"""
import argparse
import hashlib
import json
import re
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
MAZE = Path.home() / "git/ros_test/waffle-lyrical-slam/src/waffle_slam_course/models/twin_closed_maze"
PITCH = {"x720": 0.734, "y380": 0.394}     # 기둥 중심 간격(도면 값, m)
PANEL_LEN = {"x720": 0.720, "y380": 0.380}  # 판 길이(도면 이름, m)


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_stl(path):
    data = Path(path).read_bytes()
    n = int(np.frombuffer(data[80:84], np.uint32)[0])
    rec = np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")])
    return np.frombuffer(data[84:84 + n * 50], dtype=rec)["v"].reshape(-1, 3, 3).astype(float)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sdf", type=Path, default=MAZE / "model.sdf")
    ap.add_argument("--stl", type=Path, default=MAZE / "meshes/maze.stl")
    ap.add_argument("--out", type=Path, default=HERE / "data/maze_upper_drawing.json")
    args = ap.parse_args()

    sdf = args.sdf.read_text()
    cols = re.findall(r'<collision name="(\w+)"><pose>([^<]+)</pose>', sdf)
    posts = [tuple(round(float(v), 3) for v in p.split()[:2]) for n, p in cols if n.startswith("post_")]
    if len(posts) != 83:
        raise SystemExit(f"SDF 기둥 수 {len(posts)} != 83")
    post_set = set(posts)

    tri = load_stl(args.stl)
    lo, hi = tri.min(1), tri.max(1)

    def covered(x, y, z=0.2, tol=0.002):
        m = ((lo[:, 0] <= x + tol) & (hi[:, 0] >= x - tol) & (lo[:, 1] <= y + tol) & (hi[:, 1] >= y - tol)
             & (lo[:, 2] <= z) & (hi[:, 2] >= z))
        return int(m.sum())

    segs = []
    for (x, y) in posts:
        for kind, (dx, dy) in (("x720", (PITCH["x720"], 0.0)), ("y380", (0.0, PITCH["y380"]))):
            q = (round(x + dx, 3), round(y + dy, 3))
            if q in post_set and min(covered(x + dx * t, y + dy * t) for t in (0.3, 0.5, 0.7)) > 0:
                segs.append({"kind": kind, "a": (x, y), "b": q})
    full = Counter(s["kind"] for s in segs)
    shared_full = sum(1 for s in segs if abs(s["a"][1]) < 1e-6 and abs(s["b"][1]) < 1e-6)

    # 기둥 종류(전체 미로 기준으로 판정해야 가운데 줄 기둥도 맞게 나온다)
    adj = defaultdict(list)
    for s in segs:
        adj[s["a"]].append(s["kind"])
        adj[s["b"]].append(s["kind"])
    ptype = {p: ("printed" if len(adj[p]) == 2 and adj[p][0] == adj[p][1] else "al") for p in posts}
    n_al = sum(1 for t in ptype.values() if t == "al")

    # 도면 방향: y 뒤집기, 위쪽 반(y >= 0)만
    def flip(p):
        return (p[0], round(-p[1], 3) + 0.0)

    up_posts = sorted({flip(p) for p in posts if -p[1] >= -1e-6}, key=lambda p: (p[1], p[0]))
    post_id = {p: i for i, p in enumerate(up_posts)}
    out_posts = [{"id": post_id[p], "x": p[0], "y": p[1],
                  "type": ptype[(p[0], round(-p[1], 3) + 0.0)], "shared": abs(p[1]) < 1e-6} for p in up_posts]
    out_panels = []
    for s in segs:
        a, b = flip(s["a"]), flip(s["b"])
        if min(a[1], b[1]) < -1e-6:
            continue
        a, b = sorted((a, b), key=lambda p: (p[1], p[0]))
        out_panels.append({"kind": s["kind"], "length": PANEL_LEN[s["kind"]], "a": post_id[a], "b": post_id[b],
                           "center": [round((a[0] + b[0]) / 2, 4), round((a[1] + b[1]) / 2, 4)],
                           "axis": "x" if s["kind"] == "x720" else "y",
                           "shared": abs(a[1]) < 1e-6 and abs(b[1]) < 1e-6})
    out_panels.sort(key=lambda d: (d["center"][1], d["center"][0], d["axis"]))
    for i, d in enumerate(out_panels):
        d["id"] = i

    xs = sorted({p[0] for p in up_posts})
    ys = sorted({p[1] for p in up_posts})
    summary = {
        "full_maze": {"panels_x720": full["x720"], "panels_y380": full["y380"], "shared_panels": shared_full,
                      "posts": len(posts), "posts_al": n_al},
        "upper": {"panels_x720": sum(1 for d in out_panels if d["kind"] == "x720" and not d["shared"]),
                  "panels_y380": sum(1 for d in out_panels if d["kind"] == "y380"),
                  "shared_panels": sum(1 for d in out_panels if d["shared"]),
                  "posts": sum(1 for d in out_posts if not d["shared"]),
                  "shared_posts": sum(1 for d in out_posts if d["shared"]),
                  "posts_al": sum(1 for d in out_posts if d["type"] == "al")},
    }
    expect = {"panels_x720": 40, "panels_y380": 40, "shared_panels": 6, "posts": 83, "posts_al": 30}
    bad = {k: (summary["full_maze"][k], v) for k, v in expect.items() if summary["full_maze"][k] != v}
    if bad:
        raise SystemExit(f"도면 범례와 다름: {bad}")

    out = {
        "description": "Twin Closed Maze 위쪽 반(도면 방향). 가운데 공용 벽 y = 0, 바깥 벽 y = 3.152 m. 단위 m.",
        "frame": "x = SDF x, y = -(SDF y). 도면 기본맵사진.jpg와 같은 방향(+x 오른쪽, P 번호 위로 증가)",
        "sources": {"sdf": str(args.sdf), "sdf_sha256": sha256(args.sdf),
                    "stl": str(args.stl), "stl_sha256": sha256(args.stl)},
        "grid": {"x": xs, "y": ys},
        "summary": summary,
        "posts": out_posts,
        "panels": out_panels,
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(out, indent=1, ensure_ascii=False))
    print(json.dumps(summary, ensure_ascii=False))
    print("wrote", args.out)


if __name__ == "__main__":
    main()
