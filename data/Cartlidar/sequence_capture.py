"""Shared single-camera RGB-D recorder. Preview first; S starts each recording."""
import argparse
import csv
from datetime import datetime
import json
from pathlib import Path
import queue
import sys
import threading
import time
from zoneinfo import ZoneInfo

from capture_realsense import intrinsics, set_rgb_options

ROOT = Path(__file__).resolve().parent


def parse_args(mode):
    parser = argparse.ArgumentParser(description=(
        "Preview a RealSense camera; press S to start, S again to stop, Q to quit."))
    parser.add_argument("--serial", help="Camera serial; required when multiple cameras are connected")
    parser.add_argument("--allow-one", action="store_true", help="Accepted for compatibility; one camera is the default")
    parser.add_argument("--list", action="store_true", help="List devices without opening a camera")
    parser.add_argument("--label", default="signal_sequence" if mode == "video" else "depth_static")
    parser.add_argument("--output", type=Path, default=ROOT / ("video" if mode == "video" else "depth_sequence"))
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--rgb-exposure", type=float, help="Manual RGB exposure in device units; default restores auto")
    parser.add_argument("--rgb-gain", type=float)
    parser.add_argument("--rgb-white-balance", type=float)
    if mode == "video":
        parser.add_argument("--seconds", type=float, default=120, help="Automatic stop after this many seconds (default: 120)")
    else:
        parser.add_argument("--frames", type=int, default=0,
                            help="Number of unique RGB-D pairs; 0 records until S/Q (default: 0)")
    args = parser.parse_args()
    args.mode = mode
    if min(args.width, args.height, args.fps) <= 0:
        parser.error("Resolution and FPS must be positive")
    if mode == "video" and args.seconds <= 0:
        parser.error("--seconds must be positive")
    if mode == "depth" and args.frames < 0:
        parser.error("--frames must be nonnegative (0 = continuous)")
    if args.rgb_gain is not None and args.rgb_exposure is None:
        parser.error("--rgb-gain requires --rgb-exposure")
    return args


def sensor_options(rs, device):
    options = {}
    for sensor in device.query_sensors():
        values = {}
        for key in ("enable_auto_exposure", "exposure", "gain", "enable_auto_white_balance",
                    "white_balance", "emitter_enabled", "laser_power"):
            option = getattr(rs.option, key)
            if sensor.supports(option):
                try:
                    values[key] = sensor.get_option(option)
                except RuntimeError:
                    pass
        options[sensor.get_info(rs.camera_info.name)] = values
    return options


class Recording:
    """Bounded disk writer: preserves raw samples, reports gaps and write errors."""
    FIELDS = ("index", "host_elapsed_s", "color_timestamp_ms", "depth_timestamp_ms",
              "color_frame_number", "depth_frame_number", "color_exposure", "depth_exposure",
              "color_timestamp_domain", "depth_timestamp_domain")

    def __init__(self, cv2, args, calibration, settings):
        self.cv2, self.args = cv2, args
        self.started = time.monotonic()
        now = datetime.now(ZoneInfo("Asia/Seoul"))
        self.path = args.output / now.strftime("%Y%m%d_%H%M%S_%f")
        self.path.mkdir(parents=True, exist_ok=False)
        for name in ("color", "depth_raw", "depth_aligned"):
            (self.path / name).mkdir()
        self.info = {
            "label": args.label, "mode": args.mode, "started_at": now.isoformat(),
            "requested_seconds": getattr(args, "seconds", None),
            "requested_frames": getattr(args, "frames", None), "configured_fps": args.fps,
            "calibration": calibration, "sensor_options_at_start": settings,
            "depth_filtering": "none; raw Z16 and depth aligned to color are saved",
            "color_order_in_memory": "BGR", "depth_invalid_value": 0,
            "video_note": "color.avi is MJPEG at configured FPS; use frames.csv timestamps "
                          "and lossless PNG originals for timing analysis. No frames are duplicated to fill gaps.",
            "state": "recording", "complete": False,
        }
        self.done = threading.Event()
        self.queue = queue.Queue(maxsize=60)
        self.accepted = self.written = self.color_gaps = self.depth_gaps = 0
        self.skipped_duplicate_pairs = 0
        self.reused_color_frames = self.reused_depth_frames = 0
        self.frame_number_reversed = False
        self.last_ids = None
        self.first_times = self.last_times = None
        self.error = None
        self.reason = None
        self.info_path = self.path / "capture.json"
        self.info_path.write_text(json.dumps(self.info, indent=2, ensure_ascii=False), encoding="utf-8")
        self.thread = threading.Thread(target=self.write, daemon=True)
        self.thread.start()

    def offer(self, frame):
        # Called only by the acquisition thread.
        if self.done.is_set():
            return
        if self.args.mode == "video" and time.monotonic() - self.started >= self.args.seconds:
            self.stop("duration_reached")
            return
        ids = (frame["color_frame_number"], frame["depth_frame_number"])
        if self.last_ids is not None:
            if ids[0] < self.last_ids[0] or ids[1] < self.last_ids[1]:
                self.frame_number_reversed = True
                self.error = "Frame number moved backwards; stopped to avoid mixing camera epochs"
                self.stop("frame_number_error")
                return
            if ids[0] == self.last_ids[0] or ids[1] == self.last_ids[1]:
                # A frameset may reuse one stream while waiting for its next frame.
                # Do not count it as an independent noise sample or terminate the session.
                self.skipped_duplicate_pairs += 1
                self.reused_color_frames += ids[0] == self.last_ids[0]
                self.reused_depth_frames += ids[1] == self.last_ids[1]
                return
        try:
            self.queue.put_nowait((self.accepted, frame))
        except queue.Full:
            self.error = "Disk writer could not keep up; recording stopped rather than silently dropping samples"
            self.stop("queue_overflow")
            return
        if self.last_ids is not None:
            # Compare accepted pairs only; rejected frames must not double-count gaps.
            self.color_gaps += max(0, ids[0] - self.last_ids[0] - 1)
            self.depth_gaps += max(0, ids[1] - self.last_ids[1] - 1)
        self.last_ids = ids
        times = (frame["color_timestamp_ms"], frame["depth_timestamp_ms"])
        if self.first_times is None:
            self.first_times = times
        self.last_times = times
        self.accepted += 1
        if self.args.mode == "depth" and self.args.frames > 0 and self.accepted >= self.args.frames:
            self.stop("frame_target_reached")

    def stop(self, reason):
        if not self.done.is_set():
            self.reason = reason
            self.done.set()

    def write(self):
        writer = None
        try:
            if self.args.mode == "video":
                writer = self.cv2.VideoWriter(str(self.path / "color.avi"),
                    self.cv2.VideoWriter_fourcc(*"MJPG"), self.args.fps,
                    (self.args.width, self.args.height))
                if not writer.isOpened():
                    raise OSError("MJPEG video writer could not open")
            with (self.path / "frames.csv").open("w", newline="") as log:
                csv_writer = csv.DictWriter(log, fieldnames=self.FIELDS)
                csv_writer.writeheader()
                while True:
                    try:
                        index, frame = self.queue.get(timeout=0.1)
                    except queue.Empty:
                        if self.done.is_set():
                            break
                        continue
                    for name in ("color", "depth_raw", "depth_aligned"):
                        filename = self.path / name / f"{index:06d}.png"
                        if not self.cv2.imwrite(str(filename), frame[name],
                                [self.cv2.IMWRITE_PNG_COMPRESSION, 1]):
                            raise OSError(f"Failed writing {filename}")
                    if writer is not None:
                        writer.write(frame["color"])
                    row = {k: frame.get(k) for k in self.FIELDS}
                    row["index"] = index
                    row["host_elapsed_s"] = frame["host_monotonic_s"] - self.started
                    csv_writer.writerow(row)
                    log.flush()
                    self.written += 1
        except Exception as exc:
            self.error = str(exc)
            self.stop("write_error")
        finally:
            if writer is not None:
                writer.release()

    def finish(self):
        self.stop("user_stop")
        self.thread.join()
        target_reached = self.reason in ("duration_reached", "frame_target_reached") or (
            self.args.mode == "depth" and self.args.frames == 0 and self.reason == "user_stop")
        self.info.update({
            "state": "error" if self.error else "finished", "stop_reason": self.reason,
            "error": self.error, "accepted_frames": self.accepted, "saved_frames": self.written,
            "complete": bool(target_reached and not self.error and self.accepted > 0 and self.written == self.accepted),
            "color_frame_gaps": self.color_gaps, "depth_frame_gaps": self.depth_gaps,
            "skipped_duplicate_pairs": self.skipped_duplicate_pairs,
            "reused_color_frames": self.reused_color_frames,
            "reused_depth_frames": self.reused_depth_frames,
            "frame_number_reversed": self.frame_number_reversed,
            "consecutive_frames": self.accepted > 0 and not self.color_gaps and not self.depth_gaps
                                  and not self.frame_number_reversed,
            "first_timestamps_ms": self.first_times, "last_timestamps_ms": self.last_times,
            "closed_at": datetime.now(ZoneInfo("Asia/Seoul")).isoformat(),
        })
        temp = self.path / "capture.json.tmp"
        temp.write_text(json.dumps(self.info, indent=2, ensure_ascii=False), encoding="utf-8")
        temp.replace(self.info_path)
        return self.info


class Stream:
    def __init__(self, rs, np, args, context, serial):
        self.rs, self.np, self.args = rs, np, args
        self.pipeline = rs.pipeline(context)
        config = rs.config()
        config.enable_device(serial)
        config.enable_stream(rs.stream.color, args.width, args.height, rs.format.bgr8, args.fps)
        config.enable_stream(rs.stream.depth, args.width, args.height, rs.format.z16, args.fps)
        self.profile = self.pipeline.start(config)
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.latest = self.recording = self.error = None
        self.thread = None
        try:
            self.device = self.profile.get_device()
            set_rgb_options(rs, self.device.first_color_sensor(), args)
            color = self.profile.get_stream(rs.stream.color)
            depth = self.profile.get_stream(rs.stream.depth)
            ext = depth.get_extrinsics_to(color)
            self.scale = self.device.first_depth_sensor().get_depth_scale()
            self.calibration = {
                "serial": serial, "device_name": self.device.get_info(rs.camera_info.name),
                "depth_unit_m": self.scale, "color_intrinsics": intrinsics(color),
                "raw_depth_intrinsics": intrinsics(depth),
                "depth_to_color_extrinsics": {"rotation_column_major": list(ext.rotation),
                                               "translation_m": list(ext.translation)},
            }
            self.align = rs.align(rs.stream.color)
            self.thread = threading.Thread(target=self.run, daemon=True)
            self.thread.start()
        except BaseException:
            self.pipeline.stop()
            raise

    def run(self):
        try:
            while not self.stop_event.is_set():
                frames = self.pipeline.wait_for_frames(2000)
                raw = frames.get_depth_frame()
                aligned = self.align.process(frames)
                color, depth = aligned.get_color_frame(), aligned.get_depth_frame()
                if not raw or not color or not depth:
                    continue
                def exposure(frame):
                    key = self.rs.frame_metadata_value.actual_exposure
                    return frame.get_frame_metadata(key) if frame.supports_frame_metadata(key) else None
                bundle = {
                    "color": self.np.asanyarray(color.get_data()).copy(),
                    "depth_raw": self.np.asanyarray(raw.get_data()).copy(),
                    "depth_aligned": self.np.asanyarray(depth.get_data()).copy(),
                    "host_monotonic_s": time.monotonic(),
                    "color_timestamp_ms": color.get_timestamp(), "depth_timestamp_ms": raw.get_timestamp(),
                    "color_timestamp_domain": str(color.get_frame_timestamp_domain()),
                    "depth_timestamp_domain": str(raw.get_frame_timestamp_domain()),
                    "color_frame_number": color.get_frame_number(), "depth_frame_number": raw.get_frame_number(),
                    "color_exposure": exposure(color), "depth_exposure": exposure(raw),
                }
                with self.lock:
                    self.latest = bundle
                    if self.recording is not None:
                        self.recording.offer(bundle)
        except Exception as exc:
            if not self.stop_event.is_set():
                self.error = str(exc)

    def close(self):
        self.stop_event.set()
        if self.thread:
            self.thread.join(timeout=3)
        self.pipeline.stop()


def main(mode):
    args = parse_args(mode)
    try:
        import cv2
        import numpy as np
        import pyrealsense2 as rs
    except ImportError as exc:
        print(f"Missing dependency: {exc}. Use the run_*.sh launcher.", file=sys.stderr)
        return 1
    stream = recording = None
    window = "RGB-D sequence | S start/stop | Q quit"
    try:
        context = rs.context()
        devices = sorted((d.get_info(rs.camera_info.serial_number), d.get_info(rs.camera_info.name))
                         for d in context.query_devices())
        for serial, name in devices:
            print(serial, name)
        if args.list:
            return 0
        if args.serial:
            if args.serial not in [d[0] for d in devices]:
                raise RuntimeError("Requested camera serial is not connected")
            serial = args.serial
        elif len(devices) == 1:
            serial = devices[0][0]
        else:
            raise RuntimeError("Connect one camera or select one with --serial SERIAL")
        stream = Stream(rs, np, args, context, serial)
        cv2.namedWindow(window, cv2.WINDOW_NORMAL)
        ready_at = time.monotonic() + 2
        print(f"Preview only. Press S in the image window to start. Output: {args.output}")
        if args.mode == "depth":
            print(f"Depth capture: {args.frames} unique pairs" if args.frames else
                  "Depth capture: CONTINUOUS until S/Q (no frame limit)")
        message = "Waiting for S (2 s warmup)"
        while True:
            if stream.error:
                raise RuntimeError(stream.error)
            with stream.lock:
                frame = stream.latest
            fresh = frame is not None and time.monotonic() - frame["host_monotonic_s"] < 0.5
            if frame is None:
                preview = np.zeros((args.height, args.width * 2, 3), np.uint8)
            else:
                depth_view = cv2.applyColorMap(cv2.convertScaleAbs(frame["depth_aligned"],
                    alpha=stream.scale * 255 / 3), cv2.COLORMAP_JET)
                depth_view[frame["depth_aligned"] == 0] = 0
                preview = np.hstack([frame["color"].copy(), depth_view])
            if recording is not None:
                if args.mode == "video" and time.monotonic() - recording.started >= args.seconds:
                    with stream.lock:
                        recording.stop("duration_reached")
                elapsed = time.monotonic() - recording.started
                message = (f"REC {elapsed:.1f}s saved:{recording.written} "
                           f"reused:{recording.skipped_duplicate_pairs} queued:{recording.queue.qsize()}")
                if recording.done.is_set():
                    with stream.lock:
                        stream.recording = None
                    result = recording.finish()
                    print(f"{'ERROR' if recording.error else 'Saved'}: {recording.path}\n"
                          f"{recording.written} frames | reason:{recording.reason} | "
                          f"gaps RGB/depth:{recording.color_gaps}/{recording.depth_gaps} | "
                          f"reused pairs skipped:{recording.skipped_duplicate_pairs} | "
                          f"complete:{result['complete']}")
                    if recording.error:
                        print(recording.error, file=sys.stderr)
                    message = (f"Saved {recording.written} | gaps:{recording.color_gaps}/{recording.depth_gaps}"
                               if not recording.error else f"ERROR: {recording.error}")
                    recording = None
            cv2.putText(preview, f"{serial} | RGB + aligned depth (0-3m)", (10, 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
            cv2.putText(preview, "S start/stop | Q quit | " + message, (10, preview.shape[0] - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            cv2.imshow(window, preview)
            key = cv2.waitKey(10) & 0xFF
            if key in (ord("q"), ord("Q"), 27) or cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
                break
            if key in (ord("s"), ord("S")):
                if recording is not None:
                    with stream.lock:
                        recording.stop("user_stop")
                elif not fresh or time.monotonic() < ready_at:
                    message = "Not started: wait for warmup and fresh frames"
                else:
                    settings = sensor_options(rs, stream.device)
                    calibration = {**stream.calibration,
                        "aligned_depth_intrinsics": stream.calibration["color_intrinsics"]}
                    recording = Recording(cv2, args, calibration, settings)
                    with stream.lock:
                        stream.recording = recording
                    print(f"Recording started: {recording.path}")
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f"Capture error: {exc}\nClose other camera apps; check USB and udev permissions.", file=sys.stderr)
        if recording is not None:
            recording.error = str(exc)
            recording.stop("capture_error")
        return 1
    finally:
        if stream is not None:
            with stream.lock:
                stream.recording = None
            stream.close()
        if recording is not None:
            result = recording.finish()
            print(f"Recording closed: {recording.path}; saved {result['saved_frames']} frames")
        cv2.destroyAllWindows()
    return 0
