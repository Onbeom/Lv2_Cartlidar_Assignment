"""Isaac Sim ROS 2 브리지 검증용 최소 장면 (Isaac Sim 6.1, ~/isaacsim/python.sh로 실행).

장면에 넣은 값(카메라 크기·내부 파라미터, 벽 거리, TF·Odometry 값)을 meta.json에 남기고,
재생 → 일시정지 → 재생 순서로 돌리며 프레임마다 시각을 ticks.csv에, 받은 cmd_vel을 twist_rx.csv에 쓴다.
ROS 통신은 OmniGraph 브리지 노드만 쓴다(이 프로세스 안에서 rclpy를 쓰지 않는다).
"""

import argparse
import csv
import json
import math
import os
import time

parser = argparse.ArgumentParser()
parser.add_argument("--out", required=True, help="결과 폴더")
parser.add_argument("--duration", type=float, default=600.0, help="[s] 재생 시간(일시정지 제외, 실제 시간 기준)")
parser.add_argument("--pause-at", type=float, default=20.0, help="[s] 재생 시작 후 일시정지할 시점")
parser.add_argument("--pause-len", type=float, default=5.0, help="[s] 일시정지 길이")
parser.add_argument("--maps-at", type=float, default=5.0, help="[s] 재생 시작 후 /proc/self/maps를 저장할 시점")
args, _ = parser.parse_known_args()
os.makedirs(args.out, exist_ok=True)

# 환경은 SimulationApp 시작 전에 기록한다(python.sh가 setup_ros_env.sh로 바꾼 뒤의 값)
with open(os.path.join(args.out, "isaac_env.json"), "w") as f:
    keys = ["ROS_DISTRO", "RMW_IMPLEMENTATION", "ROS_DOMAIN_ID", "ROS_AUTOMATIC_DISCOVERY_RANGE", "LD_LIBRARY_PATH"]
    json.dump({k: os.environ.get(k) for k in keys}, f, indent=2)

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import carb
import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.prim as prim_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.graph.core as og
import omni.usd
import usdrt.Sdf
from isaacsim.core.experimental.objects import Cube
from isaacsim.core.simulation_manager import SimulationManager
from pxr import Gf, UsdGeom, UsdLux

app_utils.enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

from isaacsim.sensors.experimental.rtx import Lidar, LidarSensor

# ---- 장면에 넣는 값 (관찰 쪽이 이 값과 비교한다) ----
DT = 1.0 / 60.0                      # [s] 물리·렌더 한 단계
CAM_W, CAM_H = 640, 480              # [px]
FOCAL_MM = 1.93                      # [mm] 초점거리
H_APERTURE_MM = 3.896                # [mm] 가로 센서 크기
V_APERTURE_MM = H_APERTURE_MM * CAM_H / CAM_W   # 정사각 화소가 되게 맞춘다
BASELINE_M = 0.05                    # [m] 좌우 카메라 간격
CAM_FRAME_SKIP = 1                   # 1 = 두 단계마다 한 장 (60 Hz 단계 → 30 Hz)
SENSOR_Z = 0.5                       # [m] 카메라·LiDAR 높이
WALL_DIST_M = 2.0                    # [m] LiDAR에서 벽 앞면까지
WALL_THICK = 0.2
LIDAR_CONFIG = "Example_Rotary_2D"
LIDAR_HZ = 10.0
BASE_POS = (1.0, 2.0, 0.0)           # TF로 내보낼 base_link 위치 [m]
BASE_YAW_DEG = 90.0
ODOM_POS = (1.5, -0.75, 0.125)       # Odometry에 넣는 고정값 (이진수로 정확히 표현되는 값)
ODOM_YAW_DEG = 30.0
ODOM_LIN = (0.25, -0.125, 0.0625)    # [m/s]
ODOM_ANG = (0.5, -0.25, 0.125)       # [rad/s]

fx = CAM_W * FOCAL_MM / H_APERTURE_MM
fy = CAM_H * FOCAL_MM / V_APERTURE_MM
odom_q = (0.0, 0.0, math.sin(math.radians(ODOM_YAW_DEG) / 2), math.cos(math.radians(ODOM_YAW_DEG) / 2))  # xyzw
base_q = (0.0, 0.0, math.sin(math.radians(BASE_YAW_DEG) / 2), math.cos(math.radians(BASE_YAW_DEG) / 2))
meta = {
    "dt": DT,
    "camera": {"width": CAM_W, "height": CAM_H, "encoding": "rgb8", "fx": fx, "fy": fy,
               "cx": CAM_W / 2.0, "cy": CAM_H / 2.0, "frame_skip": CAM_FRAME_SKIP,
               "baseline_m": BASELINE_M, "frame_left": "camera_left", "frame_right": "camera_right"},
    "lidar": {"config": LIDAR_CONFIG, "hz": LIDAR_HZ, "wall_dist_m": WALL_DIST_M, "frame": "lidar_link"},
    "tf": {"parent": "world", "child": "base_link", "translation": BASE_POS, "rotation_xyzw": base_q},
    "odom": {"frame": "odom", "child": "base_link", "position": ODOM_POS, "orientation_xyzw": odom_q,
             "linear": ODOM_LIN, "angular": ODOM_ANG},
    "timeline": {"pause_at_s": args.pause_at, "pause_len_s": args.pause_len, "duration_s": args.duration},
}
with open(os.path.join(args.out, "meta.json"), "w") as f:
    json.dump(meta, f, indent=2)

stage_utils.set_stage_units(meters_per_unit=1.0)
stage = omni.usd.get_context().get_stage()

# 조명 (영상이 검게 나오지 않게)
light = UsdLux.DistantLight.Define(stage, "/World/Light")
light.CreateIntensityAttr(3000.0)

# 벽: 앞면이 x = WALL_DIST_M
Cube("/World/Wall", sizes=1.0, positions=[[WALL_DIST_M + WALL_THICK / 2, 0.0, SENSOR_Z]],
     scales=[[WALL_THICK, 4.0, 2.0]], colors=[0.6, 0.6, 0.6])

# TF로 내보낼 고정 프레임
base = UsdGeom.Xform.Define(stage, "/World/base_link")
base_api = UsdGeom.XformCommonAPI(base)
base_api.SetTranslate(Gf.Vec3d(*BASE_POS))
base_api.SetRotate((0.0, 0.0, BASE_YAW_DEG), UsdGeom.XformCommonAPI.RotationOrderXYZ)


def make_camera(path, y):
    """+x(벽)를 보는 카메라. USD 카메라는 -Z를 보고 +Y가 위라서 (90, 0, -90)으로 돌린다."""
    cam = UsdGeom.Camera.Define(stage, path)
    api = UsdGeom.XformCommonAPI(cam)
    api.SetTranslate(Gf.Vec3d(0.0, y, SENSOR_Z))
    api.SetRotate((90.0, 0.0, -90.0), UsdGeom.XformCommonAPI.RotationOrderXYZ)
    cam.GetFocalLengthAttr().Set(FOCAL_MM)
    cam.GetHorizontalApertureAttr().Set(H_APERTURE_MM)
    cam.GetVerticalApertureAttr().Set(V_APERTURE_MM)
    cam.GetProjectionAttr().Set("perspective")
    cam.GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 100.0))


make_camera("/World/camera_left", BASELINE_M / 2)
make_camera("/World/camera_right", -BASELINE_M / 2)

# 2D LiDAR (rtx_lidar.py 예제와 같은 방식)
lidar = Lidar.create(path="/World/lidar_link", config=LIDAR_CONFIG, tick_rate=LIDAR_HZ,
                     translations=[[0.0, 0.0, SENSOR_Z]])
simulation_app.update()

keys = og.Controller.Keys

# 시계·TF·Odometry·cmd_vel: 재생 중 매 단계 실행
og.Controller.edit(
    {"graph_path": "/BridgeCheck", "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ("PublishTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
            ("PublishOdom", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
            ("SubscribeTwist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
        ],
        keys.CONNECT: [
            ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "PublishTF.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "PublishOdom.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "SubscribeTwist.inputs:execIn"),
            ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
            ("ReadSimTime.outputs:simulationTime", "PublishTF.inputs:timeStamp"),
            ("ReadSimTime.outputs:simulationTime", "PublishOdom.inputs:timeStamp"),
        ],
        keys.SET_VALUES: [
            ("PublishClock.inputs:topicName", "/clock"),
            ("PublishTF.inputs:topicName", "/tf"),
            ("PublishTF.inputs:targetPrims", [usdrt.Sdf.Path("/World/base_link")]),
            ("PublishOdom.inputs:topicName", "/odom"),
            ("PublishOdom.inputs:odomFrameId", "odom"),
            ("PublishOdom.inputs:chassisFrameId", "base_link"),
            ("PublishOdom.inputs:publishRawVelocities", True),   # 넣은 속도를 회전 변환 없이 그대로 보낸다
            ("PublishOdom.inputs:position", list(ODOM_POS)),
            ("PublishOdom.inputs:orientation", list(odom_q)),
            ("PublishOdom.inputs:linearVelocity", list(ODOM_LIN)),
            ("PublishOdom.inputs:angularVelocity", list(ODOM_ANG)),
            ("SubscribeTwist.inputs:topicName", "/cmd_vel"),
        ],
    },
)

# 카메라: camera_periodic.py 예제와 같은 방식(렌더 결과를 매 프레임 내보내는 helper)
cam_graph, _, _, _ = og.Controller.edit(
    {"graph_path": "/BridgeCheckCamera", "evaluator_name": "push",
     "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND},
    {
        keys.CREATE_NODES: [
            ("OnTick", "omni.graph.action.OnTick"),
            ("RenderLeft", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
            ("RenderRight", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
            ("ImageLeft", "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ("ImageRight", "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ("CameraInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
        ],
        keys.CONNECT: [
            ("OnTick.outputs:tick", "RenderLeft.inputs:execIn"),
            ("OnTick.outputs:tick", "RenderRight.inputs:execIn"),
            ("RenderLeft.outputs:execOut", "ImageLeft.inputs:execIn"),
            ("RenderRight.outputs:execOut", "ImageRight.inputs:execIn"),
            ("RenderRight.outputs:execOut", "CameraInfo.inputs:execIn"),
            ("RenderLeft.outputs:renderProductPath", "ImageLeft.inputs:renderProductPath"),
            ("RenderRight.outputs:renderProductPath", "ImageRight.inputs:renderProductPath"),
            ("RenderLeft.outputs:renderProductPath", "CameraInfo.inputs:renderProductPath"),
            ("RenderRight.outputs:renderProductPath", "CameraInfo.inputs:renderProductPathRight"),
        ],
        keys.SET_VALUES: [
            ("RenderLeft.inputs:cameraPrim", [usdrt.Sdf.Path("/World/camera_left")]),
            ("RenderRight.inputs:cameraPrim", [usdrt.Sdf.Path("/World/camera_right")]),
            ("RenderLeft.inputs:width", CAM_W), ("RenderLeft.inputs:height", CAM_H),
            ("RenderRight.inputs:width", CAM_W), ("RenderRight.inputs:height", CAM_H),
            ("ImageLeft.inputs:topicName", "/left/image_raw"), ("ImageLeft.inputs:frameId", "camera_left"),
            ("ImageLeft.inputs:type", "rgb"), ("ImageLeft.inputs:frameSkipCount", CAM_FRAME_SKIP),
            ("ImageRight.inputs:topicName", "/right/image_raw"), ("ImageRight.inputs:frameId", "camera_right"),
            ("ImageRight.inputs:type", "rgb"), ("ImageRight.inputs:frameSkipCount", CAM_FRAME_SKIP),
            ("CameraInfo.inputs:topicName", "/left/camera_info"), ("CameraInfo.inputs:frameId", "camera_left"),
            ("CameraInfo.inputs:topicNameRight", "/right/camera_info"),
            ("CameraInfo.inputs:frameIdRight", "camera_right"),
            ("CameraInfo.inputs:frameSkipCount", CAM_FRAME_SKIP),
        ],
    },
)
og.Controller.evaluate_sync(cam_graph)
simulation_app.update()

SimulationManager.setup_simulation(dt=DT, device="cpu")
simulation_app.update()

# LiDAR → LaserScan (rtx_lidar.py 예제의 메타데이터 읽기를 그대로 따른다)
lidar_prim = prim_utils.get_prim_at_path(lidar.paths[0])
rot_hz = float(lidar_prim.GetAttribute("omni:sensor:Core:scanRateBaseHz").Get() or 0)
near = float(lidar_prim.GetAttribute("omni:sensor:Core:nearRangeM").Get() or 0)
far = float(lidar_prim.GetAttribute("omni:sensor:Core:farRangeM").Get() or 0)
firing = int(lidar_prim.GetAttribute("omni:sensor:Core:patternFiringRateHz").Get() or 0)
if rot_hz <= 0 or firing <= 0:
    raise RuntimeError("LaserScan: scanRateBaseHz 또는 patternFiringRateHz가 0")
scan_meta = {"horizontalFov": 360.0, "horizontalResolution": 360.0 * rot_hz / firing,
             "depthRange": [near, far], "rotationRate": rot_hz, "azimuthRange": [-180.0, 180.0]}
meta["lidar"].update({"scan_rate_hz": rot_hz, "near_m": near, "far_m": far,
                      "horizontal_resolution_deg": scan_meta["horizontalResolution"]})
with open(os.path.join(args.out, "meta.json"), "w") as f:
    json.dump(meta, f, indent=2)
lidar_sensor = LidarSensor(lidar, annotators=[])
lidar_sensor.attach_writer("RtxLidarROS2PublishLaserScan", topicName="/scan", frameId="lidar_link", **scan_meta)
simulation_app.update()

# ---- 실행 순서: 재생 → (pause_at) 일시정지 pause_len → 재생, 재생 시간 합이 duration이 될 때까지 ----
tick_f = open(os.path.join(args.out, "ticks.csv"), "w", newline="")
tick_w = csv.writer(tick_f)
tick_w.writerow(["wall_s", "sim_s", "state"])          # state: play / pause
twist_f = open(os.path.join(args.out, "twist_rx.csv"), "w", newline="")
twist_w = csv.writer(twist_f)
twist_w.writerow(["wall_s", "sim_s", "lx", "ly", "lz", "ax", "ay", "az"])
lin_attr = og.Controller.attribute("/BridgeCheck/SubscribeTwist.outputs:linearVelocity")
ang_attr = og.Controller.attribute("/BridgeCheck/SubscribeTwist.outputs:angularVelocity")
simtime_attr = og.Controller.attribute("/BridgeCheck/ReadSimTime.outputs:simulationTime")

app_utils.play()
play_wall0 = time.time()
played = 0.0            # [s] 실제 시간 기준 누적 재생 시간
seg_start = time.time()
paused_done = False
pause_start = None
maps_saved = False
last_twist = None
next_report = 30.0
print(f"[bridge_check] play start, duration {args.duration:.0f} s", flush=True)

while simulation_app.is_running():
    now = time.time()
    if app_utils.is_playing():
        played_now = played + (now - seg_start)
        if not maps_saved and played_now >= args.maps_at:
            with open("/proc/self/maps") as src, open(os.path.join(args.out, "proc_maps.txt"), "w") as dst:
                dst.write(src.read())
            maps_saved = True
        if not paused_done and played_now >= args.pause_at:
            played = played_now
            app_utils.pause()
            pause_start = now
            tick_w.writerow([f"{now:.6f}", "", "pause_begin"])
            print("[bridge_check] pause", flush=True)
            continue
        if played_now >= args.duration:
            break
        simulation_app.update()
        sim_s = simtime_attr.get()
        tick_w.writerow([f"{time.time():.6f}", f"{sim_s:.6f}", "play"])
        tw = tuple(lin_attr.get()) + tuple(ang_attr.get())
        if tw != last_twist:
            twist_w.writerow([f"{time.time():.6f}", f"{sim_s:.6f}"] + [repr(float(v)) for v in tw])
            last_twist = tw
        if played_now >= next_report:
            print(f"[bridge_check] played {played_now:.0f} s, sim {sim_s:.1f} s", flush=True)
            next_report += 30.0
        # 실제 시간 속도로 맞춘다: 시뮬레이션 시간이 실제 재생 시간보다 앞서면 기다린다
        ahead = sim_s - (played + (time.time() - seg_start))
        if ahead > 0:
            time.sleep(min(ahead, DT))
    else:
        simulation_app.update()              # 일시정지 중에도 앱은 돌린다
        time.sleep(DT)
        if pause_start is None:              # play() 직후 아직 재생 상태가 아닌 경우
            continue
        if not paused_done and now - pause_start >= args.pause_len:
            tick_w.writerow([f"{now:.6f}", "", "pause_end"])
            paused_done = True
            app_utils.play()
            seg_start = time.time()
            print("[bridge_check] resume", flush=True)

print(f"[bridge_check] done, wall {time.time() - play_wall0:.0f} s", flush=True)
tick_f.close()
twist_f.close()
app_utils.stop()
simulation_app.close()
