"""Restore connected RealSense RGB cameras to the original auto exposure settings."""
from types import SimpleNamespace
import pyrealsense2 as rs
from capture_realsense import set_rgb_options


def main():
    context = rs.context()
    devices = list(context.query_devices())
    if not devices:
        raise RuntimeError("No RealSense camera connected")
    args = SimpleNamespace(rgb_exposure=None, rgb_gain=None, rgb_white_balance=None)
    for device in devices:
        sensor = device.first_color_sensor()
        set_rgb_options(rs, sensor, args)
        print(device.get_info(rs.camera_info.serial_number), {
            "auto_exposure": sensor.get_option(rs.option.enable_auto_exposure),
            "gain": sensor.get_option(rs.option.gain),
            "auto_white_balance": sensor.get_option(rs.option.enable_auto_white_balance),
        })


if __name__ == "__main__":
    main()
