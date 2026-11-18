#!/usr/bin/env python3
"""Arducam(UVC, 팔 카메라) 미리보기; S 로 원본 컬러 한 장 저장, V 로 영상 녹화 시작 · 멈춤(--seconds 뒤 자동 멈춤), Q/Esc 로 끝.
영상은 <output>/video/<시각>_<label>/arm/frames/NNNNNN.jpg(JPEG 품질 95, 매 프레임) + frames.csv(번호 · 시각) + metadata.json. capture_realsense.py 와 같은 저장 꼴(시각 폴더/arm/color.png · metadata.json).
장치를 안 주면 /sys/class/video4linux 이름에 'USB 2.0 Camera' 또는 'Arducam' 이 든 첫 영상 장치를 씀."""
import argparse
import csv
from datetime import datetime
import queue
import threading
import json
from pathlib import Path
import time
from zoneinfo import ZoneInfo

V4L = Path("/sys/class/video4linux")


def devices():
    out = []
    for d in sorted(V4L.glob("video*"), key=lambda p: int(p.name[5:])):
        name = (d / "name").read_text().strip() if (d / "name").exists() else "?"
        index = (d / "index").read_text().strip() if (d / "index").exists() else "?"
        out.append((f"/dev/{d.name}", name, index))
    return out


def stripe_score(frame):
    """세로줄 점수: 열 평균 밝기의 옆 열 차이가 중앙값의 6 배를 넘는 열 수(장면 모서리도 조금 섞임 — 조합끼리 비교용)."""
    import numpy as np
    col = np.abs(np.diff(frame.astype(np.float32).mean(0), axis=0)).sum(1)
    return int((col > 6 * max(float(np.median(col)), 1e-3)).sum())


class Recorder:
    """V 로 시작한 영상 녹화: 받은 프레임을 큐로 넘겨 다른 스레드가 JPEG 로 씀(큐가 차면 그 프레임은 버리고 dropped 로 셈)."""

    def __init__(self, cv2, folder, meta):
        self.cv2, self.folder, self.q = cv2, folder, queue.Queue(maxsize=120)
        (folder / "frames").mkdir(parents=True, exist_ok=False)
        self.meta, self.n, self.dropped, self.t0 = meta, 0, 0, time.monotonic()
        self.log = open(folder / "frames.csv", "w", newline="")
        self.w = csv.writer(self.log)
        self.w.writerow(["index", "host_elapsed_s", "wall_time"])
        self.th = threading.Thread(target=self.run, daemon=True)
        self.th.start()

    def put(self, frame):
        try:
            self.q.put_nowait((self.n, time.monotonic() - self.t0, datetime.now(ZoneInfo("Asia/Seoul")).isoformat(), frame))
            self.n += 1
        except queue.Full:
            self.dropped += 1

    def run(self):
        while True:
            item = self.q.get()
            if item is None:
                break
            i, t, wall, frame = item
            self.cv2.imwrite(str(self.folder / "frames" / f"{i:06d}.jpg"), frame, [self.cv2.IMWRITE_JPEG_QUALITY, 95])
            self.w.writerow([i, f"{t:.4f}", wall])

    def stop(self):
        self.q.put(None)
        self.th.join()
        self.log.close()
        dur = time.monotonic() - self.t0
        self.meta |= {"frames": self.n, "dropped": self.dropped, "seconds": round(dur, 2), "mean_fps": round(self.n / max(dur, 1e-6), 2)}
        json.dump(self.meta, open(self.folder / "metadata.json", "w"), ensure_ascii=False, indent=1)
        return self.meta


def probe(cv2, dev, output):
    now = datetime.now(ZoneInfo("Asia/Seoul"))
    folder = output / ("probe_" + now.strftime("%Y%m%d_%H%M%S"))
    folder.mkdir(parents=True, exist_ok=False)
    rows = []
    for fourcc in ("MJPG", "YUYV"):
        for w, h in ((640, 480), (800, 600), (1280, 720), (1280, 960), (1920, 1080), (2592, 1944), (320, 240)):
            cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
            row = {"ask": f"{fourcc} {w}x{h}"}
            frames, t0 = [], None
            for _ in range(60):
                ok, f = cap.read()
                if not ok:
                    continue
                if t0 is None:
                    t0 = time.monotonic()
                frames.append(f)
            if len(frames) > 30:
                f = frames[-1]
                got = int(cap.get(cv2.CAP_PROP_FOURCC)).to_bytes(4, "little").decode(errors="replace")
                row |= {"got": f"{got} {f.shape[1]}x{f.shape[0]}", "fps": round((len(frames) - 1) / max(time.monotonic() - t0, 1e-6), 1),
                        "stripe_cols": stripe_score(f)}
                name = f"{fourcc}_{f.shape[1]}x{f.shape[0]}.png"
                if not (folder / name).exists():
                    cv2.imwrite(str(folder / name), f)
            else:
                row["got"] = "열기 실패"
            cap.release()
            rows.append(row)
            print(json.dumps(row, ensure_ascii=False), flush=True)
    json.dump({"device": dev, "rows": rows}, open(folder / "probe.json", "w"), ensure_ascii=False, indent=1)
    print(f"저장 {folder}")
    return 0


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--device", help="예: /dev/video2 (안 주면 이름으로 찾음)")
    p.add_argument("--list", action="store_true", help="영상 장치 목록만 보고 끝")
    p.add_argument("--fourcc", default="MJPG", help="MJPG 또는 YUYV")
    p.add_argument("--probe", action="store_true", help="형식 × 해상도 조합을 하나씩 열어 실제 크기 · fps · 세로줄 점수를 재고 한 장씩 저장(창 없음)")
    p.add_argument("--width", type=int, default=1280)      # 11-02 probe: 640×480 · 320×240(MJPG · YUYV 모두)는 고정 열에 색 세로줄, 1280×720 · 1920×1080 MJPG 는 깨끗
    p.add_argument("--height", type=int, default=720)
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--label", default="arm_pucks")
    p.add_argument("--seconds", type=float, default=120, help="영상 녹화 자동 멈춤(초)")
    p.add_argument("--preview-scale", type=float, default=1.0,
                   help="미리보기 창 크기 배율(저장 영상과 무관). ssh -X 로 띄우면 0.3 정도로 줄여야 녹화가 느려지지 않음(11-03: 1.0 에서 0.65 fps)")
    p.add_argument("--output", type=Path, default=Path(__file__).resolve().parent / "photo")
    a = p.parse_args()
    if a.list:
        for dev, name, index in devices():
            print(f"{dev}\tindex {index}\t{name}")
        return 0
    dev = a.device
    if dev is None:
        cand = [d for d, n, i in devices() if i == "0" and ("USB 2.0 Camera" in n or "Arducam" in n)]
        if not cand:
            raise SystemExit("Arducam 을 못 찾음 — --list 로 보고 --device 로 지정")
        dev = cand[0]
    import cv2
    if a.probe:
        return probe(cv2, dev, a.output)
    cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*a.fourcc))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, a.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, a.height)
    cap.set(cv2.CAP_PROP_FPS, a.fps)
    if not cap.isOpened():
        raise SystemExit(f"{dev} 를 열 수 없음(다른 프로그램 · ROS 노드가 쓰는 중인지 확인)")
    t0, n_saved, msg, rec = time.monotonic(), 0, "warming up", None
    window = f"Arducam {dev}"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    while True:
        ok, frame = cap.read()
        if not ok:
            msg = "frame read failed"
            time.sleep(0.05)
            continue
        ready = time.monotonic() - t0 > 2.0          # 처음 2 초는 자동 노출 안정화
        if rec:
            rec.put(frame)
            if time.monotonic() - rec.t0 > a.seconds:
                print("영상 끝(시간)", rec.stop()); rec = None
        view = frame.copy()
        state = f"REC {rec.n} drop {rec.dropped}" if rec else "V record"
        cv2.putText(view, f"{frame.shape[1]}x{frame.shape[0]} | S save | {state} | Q quit | saved {n_saved} | {msg if not ready else 'ready'}",
                    (10, view.shape[0] - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        if a.preview_scale != 1.0:
            view = cv2.resize(view, None, fx=a.preview_scale, fy=a.preview_scale, interpolation=cv2.INTER_AREA)
        cv2.imshow(window, view)
        key = cv2.waitKey(10) & 0xFF
        if key in (ord("q"), ord("Q"), 27) or cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
            break
        if key in (ord("v"), ord("V")) and ready:
            if rec:
                print("영상 끝(V)", rec.stop()); rec = None
            else:
                now = datetime.now(ZoneInfo("Asia/Seoul"))
                rec = Recorder(cv2, a.output / "video" / f"{now.strftime('%Y%m%d_%H%M%S')}_{a.label}" / "arm",
                               {"started_at": now.isoformat(), "label": a.label, "device": dev, "fourcc": a.fourcc,
                                "image_size": [frame.shape[1], frame.shape[0]], "requested_fps": a.fps, "jpeg_quality": 95})
                print(f"영상 시작 {rec.folder}")
        if key in (ord("s"), ord("S")) and ready:
            now = datetime.now(ZoneInfo("Asia/Seoul"))
            folder = a.output / now.strftime("%Y%m%d_%H%M%S_%f") / "arm"
            folder.mkdir(parents=True, exist_ok=False)
            cv2.imwrite(str(folder / "color.png"), frame)
            props = {k: cap.get(getattr(cv2, k)) for k in ("CAP_PROP_FRAME_WIDTH", "CAP_PROP_FRAME_HEIGHT", "CAP_PROP_FPS", "CAP_PROP_AUTO_EXPOSURE",
                                                           "CAP_PROP_EXPOSURE", "CAP_PROP_GAIN", "CAP_PROP_AUTO_WB", "CAP_PROP_WB_TEMPERATURE",
                                                           "CAP_PROP_FOCUS", "CAP_PROP_AUTOFOCUS")}
            json.dump({"captured_at": now.isoformat(), "label": a.label, "device": dev, "fourcc": a.fourcc, "name": dict((d, n) for d, n, _ in devices()).get(dev),
                       "image_size": [frame.shape[1], frame.shape[0]], "cv2_props": props,
                       "note": "color.png 는 미리보기 글자 없이 원본. 내부 파라미터(fx 등)는 보정 전이라 없음."},
                      open(folder / "metadata.json", "w"), ensure_ascii=False, indent=1)
            n_saved += 1
            print(f"저장 {folder}")
    if rec:
        print("영상 끝(종료)", rec.stop())
    cap.release()
    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
