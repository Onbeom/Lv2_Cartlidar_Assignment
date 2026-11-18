"""D435 RGB 노출 · gain 훑기 촬영: 카메라와 장면(신호등 + 퍽)을 고정해 두고, 노출 × gain 조합을 자동으로 돌며 컬러 영상을 저장한다.
신호등 판별(켜진 색이 하얗게 · 노랗게 날아가지 않는가)과 퍽 판별(퍽이 배경보다 충분히 밝고 색이 남는가)을 같은 장면에서 비교해
운용에 쓸 고정 노출 · gain 하나를 고르려는 자료다. 분석은 analyze_sweep.py.

조합 하나마다 설정을 바꾸고 --settle 장을 버린 뒤 1장 저장한다. 신호등 색은 저절로 바뀌므로 전체 조합을 --repeats 번 되풀이해
조합마다 여러 신호등 상태가 찍히게 한다. 화이트밸런스는 고정(--white-balance)해 조합끼리 색이 비교되게 한다.

실행 (PC, D435 연결): Cartlidar/.capture-venv/bin/python Cartlidar/yolo/exposure_sweep.py --serial 261822072247 --label light_on_050cm
  미리보기 창에서 장면을 맞추고 스페이스 = 훑기 시작, q = 그만. 훑는 동안 카메라 · 퍽 · 신호등을 건드리지 않는다.
결과: <out>/<YYMMDD_HHMM>_<label>/frames/*.png, frames.csv(파일 · 설정 노출 · gain · 실제 노출 · gain(메타데이터 지원 시) · 시각), sweep.json(설정 · 장치 정보)
"""
import argparse
import csv
import json
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs

ap = argparse.ArgumentParser()
ap.add_argument("--serial", required=True, help="D435 시리얼(앞 261822072247 · 뒤 261822079801)")
ap.add_argument("--label", required=True, help="장면 이름. 낮조명 on/off · 거리 등을 넣는다 (예: light_on_050cm)")
ap.add_argument("--out", type=Path, default=Path.home() / "git/lv2_next_module/Cartlidar/exposure_sweep")
ap.add_argument("--exposures", default="10,20,30,50,80,120,166,250",
                help="RGB 노출(장치 단위 100 µs, 30 fps 에서 최대 약 333)")
ap.add_argument("--gains", default="16,32,64,96,128", help="RGB gain(D435 0~128)")
ap.add_argument("--white-balance", type=float, default=4600.0, help="고정 화이트밸런스 K (기존 수동 촬영 값 4600)")
ap.add_argument("--repeats", type=int, default=10, help="전체 조합을 되풀이할 횟수")
ap.add_argument("--settle", type=int, default=6, help="설정을 바꾼 뒤 버릴 장 수")
ap.add_argument("--no-preview", action="store_true", help="미리보기 없이 바로 훑기(도구 확인용)")
args = ap.parse_args()

exposures = [float(v) for v in args.exposures.split(",")]
gains = [float(v) for v in args.gains.split(",")]

pipe, cfg = rs.pipeline(), rs.config()
cfg.enable_device(args.serial)
cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
profile = pipe.start(cfg)
dev = profile.get_device()
sensor = dev.first_color_sensor()


def put(option, value):
    r = sensor.get_option_range(option)
    if not r.min <= value <= r.max:
        raise SystemExit(f"{option} {value} 이 장치 범위 {r.min}~{r.max} 밖")
    sensor.set_option(option, value)


for e in exposures:
    r = sensor.get_option_range(rs.option.exposure)
    if not r.min <= e <= r.max:
        raise SystemExit(f"노출 {e} 이 장치 범위 {r.min}~{r.max} 밖")
for g in gains:
    r = sensor.get_option_range(rs.option.gain)
    if not r.min <= g <= r.max:
        raise SystemExit(f"gain {g} 이 장치 범위 {r.min}~{r.max} 밖")

# 미리보기: 자동 노출로 장면 맞추기
sensor.set_option(rs.option.enable_auto_exposure, 1)
sensor.set_option(rs.option.enable_auto_white_balance, 1)
win = "exposure_sweep"
print("장면을 맞추고 스페이스 = 훑기 시작, q = 그만")
while not args.no_preview:
    img = np.asanyarray(pipe.wait_for_frames().get_color_frame().get_data()).copy()
    cv2.putText(img, "preview (auto exposure)  SPACE=start  q=quit", (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                (255, 255, 255), 1)
    cv2.imshow(win, img)
    k = cv2.waitKey(1) & 0xFF
    if k == ord("q"):
        pipe.stop()
        raise SystemExit("그만둠(저장 없음)")
    if k == ord(" "):
        break

stamp = datetime.now().strftime("%y%m%d_%H%M")
out = args.out / f"{stamp}_{args.label}"
(out / "frames").mkdir(parents=True, exist_ok=False)
sensor.set_option(rs.option.enable_auto_exposure, 0)
sensor.set_option(rs.option.enable_auto_white_balance, 0)
put(rs.option.white_balance, args.white_balance)
for _ in range(30):  # 자동 노출 · 화이트밸런스를 끈 직후 1 s 는 버림(첫 장이 어둡게 찍힌 적 있음)
    pipe.wait_for_frames()
info = {k: dev.get_info(getattr(rs.camera_info, k)) for k in ("name", "serial_number", "firmware_version")}
json.dump({"label": args.label, "started": datetime.now().isoformat(timespec="seconds"), "device": info,
           "exposures": exposures, "gains": gains, "white_balance": args.white_balance, "repeats": args.repeats,
           "settle": args.settle, "auto_exposure": 0, "auto_white_balance": 0},
          open(out / "sweep.json", "w"), indent=1, ensure_ascii=False)

meta_ok = None
t0 = time.time()
n = 0
with open(out / "frames.csv", "w", newline="") as fp:
    wr = csv.writer(fp)
    wr.writerow(["file", "repeat", "exposure", "gain", "actual_exposure", "actual_gain", "frame_number", "t_s"])
    combos = [(rep, e, g) for rep in range(args.repeats) for e in exposures for g in gains]
    for rep, e, g in combos:
        put(rs.option.exposure, e)
        put(rs.option.gain, g)
        for _ in range(args.settle):
            pipe.wait_for_frames()
        for _ in range(30):  # 메타데이터가 있으면 실제 노출 · gain 이 설정값과 같은 장이 올 때까지(최대 30장 더)
            f = pipe.wait_for_frames().get_color_frame()
            if meta_ok is None:
                meta_ok = all(f.supports_frame_metadata(m) for m in (rs.frame_metadata_value.actual_exposure,
                                                                       rs.frame_metadata_value.gain_level))
            if not meta_ok:
                break
            actual = f.get_frame_metadata(rs.frame_metadata_value.actual_exposure)
            actual_gain = f.get_frame_metadata(rs.frame_metadata_value.gain_level)
            if actual == e and actual_gain == g:
                break
        else:
            print(f"경고: 노출 {e:g} · gain {g:g} 이 메타데이터에 반영 안 됨(실제 {actual} · {actual_gain})")
        img = np.asanyarray(f.get_data()).copy()
        if not meta_ok:
            actual = actual_gain = ""
        name = f"e{int(e):04d}_g{int(g):03d}_r{rep:02d}.png"
        cv2.imwrite(str(out / "frames" / name), img)
        wr.writerow([name, rep, e, g, actual, actual_gain, f.get_frame_number(), round(time.time() - t0, 3)])
        n += 1
        show = img.copy()
        cv2.putText(show, f"rep {rep + 1}/{args.repeats}  exp {e:g}  gain {g:g}  (do not touch)", (6, 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.imshow(win, show)
        if (cv2.waitKey(1) & 0xFF) == ord("q"):
            print("중간에 그만둠 — 찍은 것까지는 저장됨")
            break
pipe.stop()
cv2.destroyAllWindows()
print(f"{n}장, {time.time() - t0:.0f} s → {out}  (실제 노출 메타데이터 {'있음' if meta_ok else '없음'})")
