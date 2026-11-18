"""D435 영상이 로봇 모델에 따라 달라지는지 확인 (도구 확인, 모의, 판정 아님 — 사용자 결정 2026-10-27 저녁 3번).

지난 판정 렌더(render_r4 · render_r3 결과 json)의 D435 자세를 그대로 쓰고, 월드는 만든 그대로(라벨 · 퍽 옮기지 않음),
조명 1 한 가지로 옛 모델(261022_2119) 두 번 · 새 모델(261027_1711) 한 번을 차례로 렌더해 화소를 비교한다. 팔은 접은 자세 S0.
렌더 잡음(같은 모델 두 번 사이 차이)이 있어 두 가지로 센다(2026-10-27 첫 실행에서 tol 2 는 잡음만으로 모든 장이 달랐음):
  diff_px   = 채널 중 하나라도 |차| > --tol 인 화소
  strong_px = 5 × 5 평균 뒤 채널 중 하나라도 |차| > --strong 인 화소(잡음 제거, 물체 모양 차이만 남김)
판정 보조: 옛 · 옛(잡음) 과 옛 · 새 를 같은 장끼리 비교. strong_px 가 있는 장은 세 영상과 차이 그림을 남긴다.
결과: <out>/d435_model_compare.json, <out>/diff_<id>.png
실행 (PC, GPU 잠금):
  flock /tmp/lv2_gpu.lock ~/isaacsim-venv/bin/python Cartlidar/sim_render/d435_model_compare.py \
      --renders Cartlidar/sim_render/results/render_r4_261025/render_r4.json Cartlidar/sim_render/results/render_r3_judge7_261026/render_r3.json \
      --out <폴더> --/crashreporter/url=
"""
import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
TOP = HERE.parents[1]
sys.path.insert(0, str(HERE))
from r12_poses import K, S0, W, H  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--renders", type=Path, nargs="+", required=True)
ap.add_argument("--out", type=Path, required=True)
ap.add_argument("--old", default="261022_2119")
ap.add_argument("--new", default="261027_1711")
ap.add_argument("--tol", type=int, default=2)
ap.add_argument("--strong", type=int, default=25)
ap.add_argument("--subframes", type=int, default=8)
args, _ = ap.parse_known_args()


def model(rev):
    return (TOP / f"{rev}/usd/waffle_4wd_{rev}/waffle_4wd/waffle_4wd.usda",
            TOP / f"{rev}/urdf/waffle_4wd_description_{rev}/urdf/waffle_4wd.urdf")


poses = []
for f in args.renders:
    d = json.loads(f.read_text())
    seen = set()
    for p in d["poses"]:
        if p["kind"] == "d435" and p["id"] not in seen:
            seen.add(p["id"])
            poses.append({"src": f.parent.name, "id": p["id"], "world": p["world"], "T_w_bf": p["T_world_base_footprint"],
                          "T_w_cam": p["T_world_cam"]})
print("D435 자세", len(poses), flush=True)

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True, "width": W, "height": H, "anti_aliasing": 0, "multi_gpu": False})

import cv2  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf, UsdGeom  # noqa: E402

from robot_usd import pose_robot, use_robot_usd  # noqa: E402

out = args.out
out.mkdir(parents=True, exist_ok=False)
ctx = omni.usd.get_context()


def updates(n):
    for _ in range(n):
        app.update()


def set_camera(stage, path, T_w_opt, Kc):
    """render_r4.py 와 같음."""
    cam = UsdGeom.Camera.Define(stage, path)
    ha = 20.955
    focal = Kc["fx"] * ha / W
    cam.CreateHorizontalApertureAttr(ha)
    cam.CreateVerticalApertureAttr(focal * H / Kc["fy"])
    cam.CreateFocalLengthAttr(focal)
    cam.CreateHorizontalApertureOffsetAttr(-(Kc["cx"] - W / 2) / W * ha)
    cam.CreateVerticalApertureOffsetAttr((Kc["cy"] - H / 2) / W * ha)
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.01, 50.0))
    R, e = T_w_opt[:3, :3], T_w_opt[:3, 3]
    m = Gf.Matrix4d(*R[:, 0].tolist(), 0, *(-R[:, 1]).tolist(), 0, *(-R[:, 2]).tolist(), 0, *e.tolist(), 1)
    xf = UsdGeom.Xformable(cam)
    xf.ClearXformOpOrder()
    xf.AddTransformOp().Set(m)


def set_lighting(stage, level=1.0):
    stage.GetPrimAtPath("/World/Lights/dome").GetAttribute("inputs:intensity").Set(300.0 + level * 300.0)
    for i in range(6):
        stage.GetPrimAtPath(f"/World/Lights/ceiling_{i}").GetAttribute("inputs:intensity").Set(level * 20000.0)


imgs = {}
t0 = time.time()
for tag, rev in (("old", args.old), ("old2", args.old), ("new", args.new)):
    usd, urdf = model(rev)
    for world in sorted({p["world"] for p in poses}):
        ctx.open_stage(str((TOP / world / "task_world.usda").resolve()))
        updates(60)
        stage = ctx.get_stage()
        use_robot_usd(stage, usd)
        updates(10)
        set_lighting(stage)
        todo = [p for p in poses if p["world"] == world]
        cam = "/World/CompareD435/cam"
        set_camera(stage, cam, np.array(todo[0]["T_w_cam"]), K)
        rp = rep.create.render_product(cam, (W, H))
        a_rgb = rep.AnnotatorRegistry.get_annotator("rgb")
        a_rgb.attach(rp)
        for p in todo:
            pose_robot(stage, urdf, np.array(p["T_w_bf"]), S0)
            set_camera(stage, cam, np.array(p["T_w_cam"]), K)
            updates(4)
            for _ in range(2):
                rep.orchestrator.step(rt_subframes=args.subframes)
            imgs[(tag, p["src"], p["id"])] = np.asarray(a_rgb.get_data())[:, :, :3].astype(np.uint8).copy()
        a_rgb.detach()
        rp.destroy()
        updates(4)
        print(tag, rev, world, len(todo), round(time.time() - t0, 1), flush=True)

def compare(a, b):
    d = np.abs(a.astype(int) - b.astype(int))
    m = (d > args.tol).any(2)
    sd = np.abs(cv2.blur(a.astype(np.float32), (5, 5)) - cv2.blur(b.astype(np.float32), (5, 5)))
    ms = (sd > args.strong).any(2)
    return int(m.sum()), ms


rows = []
for p in poses:
    a, a2, b = (imgs[(t, p["src"], p["id"])] for t in ("old", "old2", "new"))
    n_noise, ms_noise = compare(a, a2)
    n, ms = compare(a, b)
    row = {"src": p["src"], "id": p["id"], "noise_diff_px": n_noise, "noise_strong_px": int(ms_noise.sum()),
           "diff_px": n, "strong_px": int(ms.sum())}
    if ms.any():
        ys, xs = np.nonzero(ms)
        row["strong_bbox"] = [int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max())]
        vis = np.hstack([a, a2, b, np.dstack([ms * 255] * 3).astype(np.uint8)])
        cv2.imwrite(str(out / f"diff_{p['src']}_{p['id']}.png"), cv2.cvtColor(vis, cv2.COLOR_RGB2BGR))
    rows.append(row)
summary = {"old": args.old, "new": args.new, "tol": args.tol, "strong": args.strong, "poses": len(rows),
           "noise_poses_with_strong": sum(r["noise_strong_px"] > 0 for r in rows),
           "noise_max_strong_px": max(r["noise_strong_px"] for r in rows),
           "poses_with_strong": sum(r["strong_px"] > 0 for r in rows), "max_strong_px": max(r["strong_px"] for r in rows),
           "noise_median_diff_px": float(np.median([r["noise_diff_px"] for r in rows])),
           "median_diff_px": float(np.median([r["diff_px"] for r in rows])),
           "total_s": round(time.time() - t0, 1), "rows": rows}
(out / "d435_model_compare.json").write_text(json.dumps(summary, indent=1, ensure_ascii=False))
print(json.dumps({k: v for k, v in summary.items() if k != "rows"}, ensure_ascii=False))
app.close()
