#!/usr/bin/env python3
"""Two RealSense RGB-D previews; S saves an unmodified snapshot from each camera."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import shutil
import sys
import threading
import time
from zoneinfo import ZoneInfo

DEFAULT_OUTPUT = Path(__file__).resolve().parent / "photo"


def arguments():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    p.add_argument("--label", default="unsorted", help="Scene name stored in metadata")
    p.add_argument("--front", help="Front camera serial (requires --rear)")
    p.add_argument("--rear", help="Rear camera serial (requires --front)")
    p.add_argument("--allow-one", action="store_true", help="Allow collecting with one camera")
    p.add_argument("--list", action="store_true", help="List camera serials and exit")
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=480)
    p.add_argument("--fps", type=int, default=30)
    # Without manual overrides, restore the original automatic RGB settings.
    p.add_argument("--rgb-exposure", type=float,
                   help="RGB exposure in device units (D435 range 1-10000, default 166); turns RGB auto exposure off")
    p.add_argument("--rgb-gain", type=float, help="RGB gain (needs --rgb-exposure)")
    p.add_argument("--rgb-white-balance", type=float, help="Fixed RGB white balance in K; turns auto white balance off")
    args = p.parse_args()
    if args.rgb_gain is not None and args.rgb_exposure is None:
        p.error("--rgb-gain needs --rgb-exposure (gain is ignored while auto exposure is on)")
    if bool(args.front) != bool(args.rear):
        p.error("--front and --rear must be supplied together")
    if args.front and args.front == args.rear:
        p.error("Front and rear must have different serials")
    if min(args.width, args.height, args.fps) < 1:
        p.error("Resolution and FPS must be positive")
    return args


def intrinsics(profile):
    k = profile.as_video_stream_profile().get_intrinsics()
    return dict(width=k.width, height=k.height, fx=k.fx, fy=k.fy,
                ppx=k.ppx, ppy=k.ppy, distortion_model=str(k.model), coeffs=list(k.coeffs))


def set_rgb_options(rs, sensor, args):
    """Restore original auto RGB settings unless explicitly overridden."""
    def put(option, value):
        r = sensor.get_option_range(option)
        if not r.min <= value <= r.max:
            raise ValueError(f"{option} {value} outside device range {r.min}..{r.max}")
        sensor.set_option(option, value)
    if args.rgb_exposure is not None:
        sensor.set_option(rs.option.enable_auto_exposure, 0)
        put(rs.option.exposure, args.rgb_exposure)
        if args.rgb_gain is not None:
            put(rs.option.gain, args.rgb_gain)
    else:
        # Original capture metadata: RGB gain 64, auto exposure enabled.
        # A previous manual session can leave auto exposure disabled on the device.
        put(rs.option.gain, 64.0)
        sensor.set_option(rs.option.enable_auto_exposure, 1)
    if args.rgb_white_balance is not None:
        sensor.set_option(rs.option.enable_auto_white_balance, 0)
        put(rs.option.white_balance, args.rgb_white_balance)
    else:
        sensor.set_option(rs.option.enable_auto_white_balance, 1)


class Camera:
    def __init__(self, rs, np, context, name, serial, args):
        self.rs, self.np = rs, np
        self.name, self.serial = name, serial
        self.lock = threading.Lock()
        self.latest = None
        self.error = None
        self.stop_event = threading.Event()
        self.pipeline = rs.pipeline(context)
        self.thread = None
        self.started = False
        config = rs.config()
        config.enable_device(serial)
        config.enable_stream(rs.stream.color, args.width, args.height, rs.format.bgr8, args.fps)
        config.enable_stream(rs.stream.depth, args.width, args.height, rs.format.z16, args.fps)
        profile = self.pipeline.start(config)
        self.started = True
        try:
            device = profile.get_device()
            self.depth_scale = device.first_depth_sensor().get_depth_scale()
            set_rgb_options(rs, device.first_color_sensor(), args)
            color = profile.get_stream(rs.stream.color)
            depth = profile.get_stream(rs.stream.depth)
            ext = depth.get_extrinsics_to(color)
            self.calibration = {
                "name": name, "serial": serial,
                "device_name": device.get_info(rs.camera_info.name),
                "depth_unit_m": self.depth_scale,
                "color_intrinsics": intrinsics(color),
                "raw_depth_intrinsics": intrinsics(depth),
                "depth_to_color_extrinsics": {
                    "rotation_column_major": list(ext.rotation), "translation_m": list(ext.translation)},
            }
            self.align = rs.align(rs.stream.color)
            self.thread = threading.Thread(target=self.run, daemon=True, name=name)
            self.thread.start()
        except BaseException:
            self.pipeline.stop()
            self.started = False
            raise

    def run(self):
        try:
            while not self.stop_event.is_set():
                frames = self.pipeline.wait_for_frames(1000)
                raw = frames.get_depth_frame()
                aligned = self.align.process(frames)
                color = aligned.get_color_frame()
                depth = aligned.get_depth_frame()
                if not raw or not color or not depth:
                    continue
                sensor_options = {}
                for sensor in self.pipeline.get_active_profile().get_device().query_sensors():
                    values = {}
                    for key in ("enable_auto_exposure", "exposure", "gain",
                                "enable_auto_white_balance", "white_balance", "emitter_enabled"):
                        option = getattr(self.rs.option, key)
                        if sensor.supports(option):
                            try:
                                values[key] = sensor.get_option(option)
                            except RuntimeError:
                                pass
                    sensor_options[sensor.get_info(self.rs.camera_info.name)] = values
                snapshot = {
                    "color": self.np.asanyarray(color.get_data()).copy(),
                    "depth_raw": self.np.asanyarray(raw.get_data()).copy(),
                    "depth_aligned": self.np.asanyarray(depth.get_data()).copy(),
                    "received_monotonic_s": time.monotonic(),
                    "metadata": {
                        **self.calibration,
                        "aligned_depth_intrinsics": intrinsics(depth.profile),
                        "color_timestamp_ms": color.get_timestamp(),
                        "depth_timestamp_ms": depth.get_timestamp(),
                        "color_timestamp_domain": str(color.get_frame_timestamp_domain()),
                        "depth_timestamp_domain": str(depth.get_frame_timestamp_domain()),
                        "color_frame_number": color.get_frame_number(),
                        "depth_frame_number": depth.get_frame_number(),
                        "sensor_options_at_receive": sensor_options,
                    },
                }
                with self.lock:
                    self.latest = snapshot
        except Exception as exc:
            if not self.stop_event.is_set():
                self.error = str(exc)

    def snapshot(self):
        with self.lock:
            return self.latest

    def close(self):
        self.stop_event.set()
        if self.thread:
            self.thread.join(timeout=2)
        if self.started:
            self.pipeline.stop()
            self.started = False


def save(cv2, snapshots, output, label):
    now = datetime.now(ZoneInfo("Asia/Seoul"))
    name = now.strftime("%Y%m%d_%H%M%S_%f")
    output.mkdir(parents=True, exist_ok=True)
    temporary = output / (".saving_" + name)
    temporary.mkdir()
    try:
        for camera, frame in snapshots:
            directory = temporary / f"{camera.name}_{camera.serial}"
            directory.mkdir()
            for key in ("color", "depth_raw", "depth_aligned"):
                if not cv2.imwrite(str(directory / f"{key}.png"), frame[key]):
                    raise OSError(f"Failed writing {key}.png")
            (directory / "metadata.json").write_text(
                json.dumps(frame["metadata"], indent=2), encoding="utf-8")
        times = [f["received_monotonic_s"] for _, f in snapshots]
        (temporary / "capture.json").write_text(json.dumps({
            "captured_at": now.isoformat(), "label": label,
            "hardware_synchronized": False,
            "host_receive_spread_ms": (max(times) - min(times)) * 1000,
            "note": "Latest frames at keypress; separate camera clocks are not synchronized. "
                    "Depth PNGs are uint16; multiply by depth_unit_m for metres. "
                    "color.png is saved without preview annotations.",
        }, indent=2, ensure_ascii=False), encoding="utf-8")
        destination = output / name
        temporary.rename(destination)
        return destination
    except BaseException:
        shutil.rmtree(temporary)
        raise


def main():
    args = arguments()
    try:
        import cv2
        import numpy as np
        import pyrealsense2 as rs
    except ImportError as exc:
        print(f"Missing dependency: {exc}\nUse ./run_capture.sh", file=sys.stderr)
        return 1
    try:
        context = rs.context()
        devices = sorted((d.get_info(rs.camera_info.serial_number), d.get_info(rs.camera_info.name))
                         for d in context.query_devices())
    except RuntimeError as exc:
        print(f"Cannot query USB devices: {exc}\nRun from your normal desktop terminal; "
              "check RealSense USB/udev permissions.", file=sys.stderr)
        return 1
    for serial, name in devices:
        print(f"{serial}  {name}")
    if args.list:
        if not devices:
            print("No RealSense connected.")
        return 0
    serials = [serial for serial, _ in devices]
    if args.front:
        chosen = [("front", args.front), ("rear", args.rear)]
        if any(s not in serials for _, s in chosen):
            print("Requested serial not connected.", file=sys.stderr)
            return 1
    elif len(devices) == 2:
        chosen = list(zip(("cam1", "cam2"), serials))
        print("cam1/cam2 follow serial order. Use --front SERIAL --rear SERIAL for roles.")
    elif len(devices) == 1 and args.allow_one:
        chosen = [("cam1", serials[0])]
    else:
        print("Connect exactly two RealSense cameras, or specify --front/--rear. "
              "Use --allow-one for one camera.", file=sys.stderr)
        return 1
    cameras = []
    window = "RealSense capture | S save | Q/ESC quit"
    try:
        for name, serial in chosen:
            cameras.append(Camera(rs, np, context, name, serial, args))
        cv2.namedWindow(window, cv2.WINDOW_NORMAL)
        print(f"Output: {args.output.resolve()}\nS: save all cameras; Q/ESC: quit")
        message, saved = "Warming up (2 s)...", 0
        ready_after = time.monotonic() + 2
        while True:
            views, snapshots = [], []
            all_ready = time.monotonic() >= ready_after
            for camera in cameras:
                if camera.error:
                    raise RuntimeError(f"{camera.name}: {camera.error}")
                frame = camera.snapshot()
                fresh = frame is not None and time.monotonic() - frame["received_monotonic_s"] <= 0.5
                all_ready = all_ready and fresh
                if fresh:
                    snapshots.append((camera, frame))
                    image = frame["color"].copy()
                    depth_view = cv2.applyColorMap(cv2.convertScaleAbs(
                        frame["depth_aligned"], alpha=camera.depth_scale * 255 / 2), cv2.COLORMAP_JET)
                    depth_view[frame["depth_aligned"] == 0] = 0
                    image = np.hstack((image, depth_view))
                else:
                    image = np.zeros((args.height, args.width * 2, 3), dtype=np.uint8)
                cv2.putText(image, f"{camera.name} SN:{camera.serial} | RGB + depth (0-2m)",
                            (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
                views.append(image)
            preview = np.vstack(views)
            cv2.putText(preview, f"S save | Q quit | saved:{saved} | {message}",
                        (10, preview.shape[0] - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            cv2.imshow(window, preview)
            key = cv2.waitKey(10) & 0xFF
            if key in (ord("q"), ord("Q"), 27) or cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
                break
            if key in (ord("s"), ord("S")):
                if not all_ready:
                    message = "Not saved: wait for fresh frames from every camera"
                    print(message)
                    continue
                try:
                    destination = save(cv2, snapshots, args.output, args.label)
                    saved += 1
                    message = "Saved " + destination.name
                    print(f"Saved: {destination}")
                except OSError as exc:
                    message = "Save failed (see terminal)"
                    print(f"Save failed: {exc}", file=sys.stderr)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f"Capture stopped: {exc}\nClose RealSense Viewer/ROS camera drivers if busy. "
              "Check USB 3 cables and camera permissions.", file=sys.stderr)
        return 1
    finally:
        for camera in reversed(cameras):
            camera.close()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
