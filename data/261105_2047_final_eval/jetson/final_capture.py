#!/usr/bin/env python3
"""최종 확인 영상을 창 없이, ssh 터미널 키 입력만으로 한 회차 찍는다.
저장 꼴과 녹화 코드는 record_signal.py(sequence_capture) · capture_arducam.py 그대로다.
키: Enter 녹화 시작 · q 이 회차 건너뜀(녹화 전) · x 녹화 멈추고 버림(녹화 중) · Ctrl+C 끝.
다 찍지 못한 회차는 <출력>_failed 로 옮겨 판정 폴더에 섞이지 않게 한다(지우지 않는다).
미리보기: Jetson 모니터(DISPLAY)에 창을 띄운다. 보기만 하는 창이고, 키는 터미널에서만 받는다. 창을 못 띄워도 녹화는 된다.
끝 코드: 0 저장됨 · 이미 있어 건너뜀, 1 실패, 2 사용자가 건너뜀, 130 Ctrl+C."""
import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import select
import shutil
import subprocess
import sys
import termios
import time
import tty
from zoneinfo import ZoneInfo

SERIALS = {"front": "261822072247", "rear": "261822079801"}
SUFFIX = {"front": "f", "rear": "r"}
D435_SECONDS, ARM_SECONDS = 90.0, 60.0
D435_FPS, D435_EXPOSURE, D435_GAIN = 6, 80.0, 128.0
D435_MIN_FRAMES = 500   # 90 s × 6 fps 는 약 538 장
ARM_MIN_FPS = 10.0      # 이보다 느리면 경고만 한다. 판정 전 사람이 본다
PREVIEW_WIDTH, PREVIEW_PERIOD_S = 640, 0.2   # 녹화를 늦추지 않게 작게, 초당 5 장만 그린다


class Keys:
    """터미널에서 한 글자씩 읽는다. Enter 없이 바로 들어온다."""

    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.saved = termios.tcgetattr(self.fd) if os.isatty(self.fd) else None
        if self.saved:
            tty.setcbreak(self.fd)
            termios.tcflush(self.fd, termios.TCIFLUSH)   # 앞서 눌러 둔 키로 바로 시작하지 않게
        return self

    def __exit__(self, *exc):
        if self.saved:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.saved)

    def get(self):
        if not select.select([self.fd], [], [], 0)[0]:
            return None
        data = os.read(self.fd, 1)
        return data.decode(errors="ignore").lower() if data else None


class Preview:
    """Jetson 모니터의 보기 전용 창. 저장되는 장은 건드리지 않는다."""

    def __init__(self, cv2, enabled, label):
        self.cv2, self.label, self.status, self.next = cv2, label, "ready", 0.0
        self.window = f"final capture {label}"
        self.on = enabled and self.can_open()
        if self.on:
            cv2.namedWindow(self.window, cv2.WINDOW_NORMAL)

    @staticmethod
    def can_open():
        """화면에 못 붙으면 Qt 가 프로세스를 통째로 죽이므로(잡을 수 없음) 다른 프로세스로 먼저 본다."""
        code = "import cv2; cv2.namedWindow('probe'); cv2.waitKey(1); cv2.destroyAllWindows()"
        try:
            ok = subprocess.run([sys.executable, "-c", code], capture_output=True, timeout=15).returncode == 0
        except subprocess.TimeoutExpired:
            ok = False
        if not ok:
            say(f"미리보기 창을 못 띄웠습니다(DISPLAY={os.environ.get('DISPLAY', '없음')}). 녹화는 됩니다. "
                "Jetson 모니터에 cart11 로 로그인되어 있는지 보세요.")
        return ok

    def show(self, frame):
        now = time.monotonic()
        if not self.on or now < self.next:
            return
        self.next = now + PREVIEW_PERIOD_S
        scale = PREVIEW_WIDTH / frame.shape[1]
        view = self.cv2.resize(frame, None, fx=scale, fy=scale, interpolation=self.cv2.INTER_AREA)
        self.cv2.putText(view, f"{self.label} {self.status}", (8, 22), self.cv2.FONT_HERSHEY_SIMPLEX,
                         0.55, (0, 255, 255), 1)
        self.cv2.imshow(self.window, view)
        self.cv2.waitKey(1)

    def close(self):
        if self.on:
            self.cv2.destroyAllWindows()
            self.cv2.waitKey(1)


def say(text):
    print(text, flush=True)


def line(text):
    print("\r" + text.ljust(70), end="", flush=True)


def shelve(folder, output):
    """다 찍지 못한 폴더를 판정 폴더 밖으로 옮긴다."""
    failed = output.parent / (output.name + "_failed")
    failed.mkdir(parents=True, exist_ok=True)
    target = failed / folder.name
    if folder.name == "arm":   # <시각>_<label>/arm 은 위 폴더째 옮긴다
        folder, target = folder.parent, failed / folder.parent.name
    shutil.move(str(folder), str(target))
    say(f"판정 폴더에서 뺐습니다: {target}")


def existing(cam, label, output):
    """이 회차 폴더를 (다 찍힌 것, 덜 찍힌 것) 으로 나눈다."""
    done, partial = [], []
    if cam == "arm":
        for d in sorted((output / "video").glob(f"*_{label}")):
            meta = d / "arm" / "metadata.json"
            ok = meta.exists() and json.load(open(meta)).get("seconds", 0) >= ARM_SECONDS - 1
            (done if ok else partial).append(d / "arm")
    elif output.exists():
        for d in sorted(p for p in output.iterdir() if p.is_dir() and (p / "capture.json").exists()):
            info = json.load(open(d / "capture.json"))
            if info.get("label") == label:
                (done if info.get("complete") else partial).append(d)
    return done, partial


def wait_start(keys, label, seconds, ready):
    """녹화 전 대기. 시작이면 True, 건너뜀이면 False."""
    say(f"[{label}] 배치가 끝나면 Enter: {seconds:.0f} 초 녹화 시작 · q: 이 회차 건너뜀")
    shown = ""
    while True:
        message = ready()          # 카메라가 죽으면 여기서 예외가 난다
        if message != shown:
            line(message)
            shown = message
        key = keys.get()
        if key == "q":
            print()
            return False
        if key in ("\n", "\r") and message == "준비됨":
            print()
            return True
        time.sleep(0.02)


def d435(args, keys, label):
    import cv2
    import numpy as np
    import pyrealsense2 as rs
    from sequence_capture import Recording, Stream, sensor_options
    conf = argparse.Namespace(mode="video", label=label, output=args.output, width=640, height=480, fps=D435_FPS,
                              rgb_exposure=D435_EXPOSURE, rgb_gain=D435_GAIN, rgb_white_balance=None,
                              seconds=D435_SECONDS)
    serial = SERIALS[args.cam]
    context = rs.context()
    connected = [d.get_info(rs.camera_info.serial_number) for d in context.query_devices()]
    if serial not in connected:
        raise RuntimeError(f"{args.cam} D435({serial}) 가 연결되어 있지 않습니다. 연결됨: {connected}")
    preview = Preview(cv2, not args.no_window, label)
    stream = Stream(rs, np, conf, context, serial)
    recording = None
    warm = time.monotonic() + 2

    def check():
        if stream.error:
            raise RuntimeError(stream.error)
        with stream.lock:
            frame = stream.latest
        if frame is not None:
            preview.show(frame["color"])
        return frame

    def ready():
        frame = check()
        fresh = frame is not None and time.monotonic() - frame["host_monotonic_s"] < 0.5
        return "준비됨" if fresh and time.monotonic() >= warm else "카메라 준비 중"

    try:
        if not wait_start(keys, label, D435_SECONDS, ready):
            return 2
        settings = sensor_options(rs, stream.device)
        calibration = {**stream.calibration, "aligned_depth_intrinsics": stream.calibration["color_intrinsics"]}
        recording = Recording(cv2, conf, calibration, settings)
        with stream.lock:
            stream.recording = recording
        preview.status = "REC"
        while not recording.done.is_set():
            check()
            elapsed = time.monotonic() - recording.started
            if elapsed >= D435_SECONDS:
                with stream.lock:
                    recording.stop("duration_reached")
            if keys.get() == "x":
                with stream.lock:
                    recording.stop("user_stop")
            line(f"REC {elapsed:5.1f}/{D435_SECONDS:.0f} s  저장 {recording.written}  대기 {recording.queue.qsize()}")
            time.sleep(0.1)
        print()
        with stream.lock:
            stream.recording = None
        result = recording.finish()
        say(f"저장 {result['saved_frames']} 장 · 끝난 이유 {result['stop_reason']} · 빠진 번호 "
            f"{result['color_frame_gaps']}/{result['depth_frame_gaps']} · complete {result['complete']}")
        path, recording = recording.path, None
        if not result["complete"]:
            say(f"다 찍지 못했습니다: {result['error'] or result['stop_reason']}")
            shelve(path, args.output)
            return 1
        if result["saved_frames"] < D435_MIN_FRAMES:
            say(f"주의: {result['saved_frames']} 장으로 약 538 장보다 적습니다. 영상은 남겼으니 판정 전에 봅니다.")
        say(f"완료: {path}")
        return 0
    finally:
        with stream.lock:
            stream.recording = None
        stream.close()
        preview.close()
        if recording is not None:      # 예외 · Ctrl+C 로 녹화 중 끊김
            recording.stop("capture_error")
            recording.finish()
            shelve(recording.path, args.output)


def arm(args, keys, label):
    import cv2
    from capture_arducam import Recorder, devices
    dev = args.device
    if dev is None:
        found = [d for d, n, i in devices() if i == "0" and ("USB 2.0 Camera" in n or "Arducam" in n)]
        if not found:
            raise RuntimeError("Arducam 을 못 찾았습니다")
        dev = found[0]
    cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_FPS, 30)
    if not cap.isOpened():
        raise RuntimeError(f"{dev} 를 열 수 없습니다(다른 프로그램 · ROS 노드가 쓰는 중인지 확인)")
    preview = Preview(cv2, not args.no_window, label)
    start, last_ok = time.monotonic(), time.monotonic()
    rec = frame = None

    def grab():
        nonlocal frame, last_ok
        ok, got = cap.read()
        if ok:
            if got.shape[:2] != (720, 1280):
                raise RuntimeError(f"해상도가 1280×720 이 아닙니다: {got.shape[1]}×{got.shape[0]}")
            frame, last_ok = got, time.monotonic()
            preview.show(got)
        elif time.monotonic() - last_ok > 3:
            raise RuntimeError("3 초 동안 영상이 들어오지 않았습니다")
        else:
            time.sleep(0.05)
        return ok

    def ready():
        grab()
        return "준비됨" if frame is not None and time.monotonic() - start > 2 else "카메라 준비 중"

    try:
        if not wait_start(keys, label, ARM_SECONDS, ready):
            return 2
        now = datetime.now(ZoneInfo("Asia/Seoul"))
        rec = Recorder(cv2, args.output / "video" / f"{now.strftime('%Y%m%d_%H%M%S')}_{label}" / "arm",
                       {"started_at": now.isoformat(), "label": label, "device": dev, "fourcc": "MJPG",
                        "image_size": [1280, 720], "requested_fps": 30, "jpeg_quality": 95})
        preview.status = "REC"
        shown = 0.0
        stopped = False
        while time.monotonic() - rec.t0 <= ARM_SECONDS:
            if grab():
                rec.put(frame)
            if keys.get() == "x":
                stopped = True
                break
            elapsed = time.monotonic() - rec.t0
            if elapsed - shown >= 0.5:
                shown = elapsed
                line(f"REC {elapsed:5.1f}/{ARM_SECONDS:.0f} s  받음 {rec.n}  버림 {rec.dropped}")
        print()
        meta = rec.stop()
        folder, rec = args.output / "video" / f"{now.strftime('%Y%m%d_%H%M%S')}_{label}" / "arm", None
        say(f"저장 {meta['frames']} 장 · {meta['seconds']} s · {meta['mean_fps']} fps · 버림 {meta['dropped']}")
        if stopped or not meta["frames"]:
            say("다 찍지 못했습니다")
            shelve(folder, args.output)
            return 1
        if meta["mean_fps"] < ARM_MIN_FPS:
            say(f"주의: {meta['mean_fps']} fps 로 느립니다. 영상은 남겼으니 판정 전에 봅니다.")
        say(f"완료: {folder}")
        return 0
    finally:
        cap.release()
        preview.close()
        if rec is not None:
            folder = rec.folder
            rec.stop()
            shelve(folder, args.output)


def summary(output):
    for cam, labels in (("앞", [f"final_v{v}_f" for v in (1, 2, 3)]), ("뒤", [f"final_v{v}_r" for v in (1, 2, 3)]),
                        ("팔", [f"final_arm_v{v}" for v in (1, 2, 3)])):
        kind = "arm" if cam == "팔" else "d435"
        say(f"{cam}: " + " · ".join(f"{x} {'있음' if existing(kind, x, output)[0] else '없음'}" for x in labels))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("cam", choices=("front", "rear", "arm", "summary"))
    p.add_argument("round", type=int, nargs="?", choices=(1, 2, 3))
    p.add_argument("--output", type=Path, default=Path.home() / "capture" / "video_final")
    p.add_argument("--device", help="Arducam 장치. 안 주면 이름으로 찾음")
    p.add_argument("--no-window", action="store_true", help="미리보기 창을 띄우지 않음")
    p.add_argument("--retake", action="store_true", help="이미 다 찍은 회차를 _failed 로 옮기고 다시 찍음")
    args = p.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    if args.cam == "summary":
        summary(args.output)
        return 0
    if args.round is None:
        p.error("회차(1 2 3)를 주세요")
    label = f"final_arm_v{args.round}" if args.cam == "arm" else f"final_v{args.round}_{SUFFIX[args.cam]}"
    done, partial = existing(args.cam, label, args.output)
    for folder in partial:
        shelve(folder, args.output)
    if done and not args.retake:
        say(f"[{label}] 이미 있어 건너뜁니다: {done[0]}")
        return 0
    for folder in done:
        shelve(folder, args.output)
    with Keys() as keys:
        try:
            return (arm if args.cam == "arm" else d435)(args, keys, label)
        except KeyboardInterrupt:
            print()
            say("Ctrl+C 로 멈췄습니다")
            return 130
        except Exception as exc:
            print()
            say(f"실패: {exc}")
            return 1


if __name__ == "__main__":
    raise SystemExit(main())
