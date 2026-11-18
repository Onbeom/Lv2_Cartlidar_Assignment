"""L2 로봇 장면: Waffle 4WD(261027_1530, v12 조건) + Isaac 기본 브리지 + Isaac 안 Lyrical rclpy.

조건 A′(Lyrical source + empy·lark 링크)로 ~/isaacsim-venv/bin/python에서 실행한다(run_l2.sh).
검증계획 9장(방식 (a)):
  기본 브리지: /clock, /odom, TF odom→base_footprint, 컬러 3·깊이 2 영상
  rclpy(이 프로세스): /scan_front·/scan_rear(PhysX 레이캐스트), camera_info 3개(컬러 영상 header 복사),
                      /joint_states(50 Hz), /cmd_vel(TwistStamped 받기 → 바퀴 기구학)
판정용 참값(차체 자세·관절 값·바퀴 목표)은 truth.csv에, 장면 값은 meta.json에 쓴다. 판정은 analyze_l2.py.
"""

import argparse
import csv
import json
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(os.path.dirname(HERE))

parser = argparse.ArgumentParser()
parser.add_argument("--out", required=True, help="결과 폴더")
parser.add_argument("--duration", type=float, default=600.0, help="[s] 시뮬레이션 시간")
parser.add_argument("--usd", default=os.path.join(ROOT_DIR, "261027_1613/phys_tests/usd_copy/waffle_4wd/waffle_4wd.usda"))
parser.add_argument("--wheels", default=os.path.join(HERE, "config/wheels.yaml"))
args, _ = parser.parse_known_args()
os.makedirs(args.out, exist_ok=True)
sys.path.insert(0, HERE)
from wheel_kinematics import DiffDrive, load_config  # noqa: E402

with open(os.path.join(args.out, "isaac_env.json"), "w") as f:
    keys = ["ROS_DISTRO", "RMW_IMPLEMENTATION", "ROS_DOMAIN_ID", "ROS_AUTOMATIC_DISCOVERY_RANGE", "PYTHONPATH"]
    json.dump({k: os.environ.get(k) for k in keys}, f, indent=2)

# ---- 조건 (물리 시험 v12와 같게: 261022_2119/phys_tests/run_phys_test.py, 검증계획 15장) ----
PHYS_DT = 1.0 / 120.0
RENDER_DT = 1.0 / 30.0                # 렌더·OmniGraph tick 30 Hz → 카메라 30 Hz, /clock·/odom·TF 30 Hz
MU_STATIC, MU_DYNAMIC = 0.3, 0.225
ARM_KP, ARM_KD = 400.0, 36.0          # v12
GL_KP, GL_KD = 1000.0, 10.0
WHEEL_KP, WHEEL_KD = 0.0, 1.0
WHEEL_ARMATURE, ARM_ARMATURE = None, 0.045   # 바퀴는 바퀴 정의 파일 wheel_armature_kgm2 (아래에서 읽음)
ARM_RATE = 43 * 2 * math.pi / 60      # XM430-W350 무부하 속도, 팔 위치 목표 변화 한계 [rad/s]
ARM_POSE_STOW = [0.0, 1.4, -1.0, -1.3, 0.0]   # S0 (yaw, 어깨, 팔꿈치, 손목 pitch, roll)

# ---- 센서 (검증계획 9-1, 사용자 승인 2026-10-25) ----
W, H = 640, 480
D435_K = (605.850769, 605.67987, 324.365814, 245.139175)      # 실물 261822072247 컬러 640x480
ARM_K = (283.0, 283.0, 320.0, 240.0)                           # 인지 렌더 도구 가정값(실측 전)
CAMERAS = {  # 이름: (URDF frame, K, 컬러 토픽 앞부분, 깊이 발행 여부)
    "front": ("top_camera_front_color_optical_frame", D435_K, "/top_camera_front", True),
    "rear": ("top_camera_rear_color_optical_frame", D435_K, "/top_camera_rear", True),
    "arm": ("camera_optical_frame", ARM_K, "/arm_camera", False),
}
LIDAR_HZ, LIDAR_RES_DEG, LIDAR_MIN, LIDAR_MAX = 10.0, 1.0, 0.12, 8.0   # 임시(실물 사양 전, 사용자 승인)
LIDARS = {"/scan_front": "lidar_front_scan", "/scan_rear": "lidar_rear_scan"}
JOINT_STATE_HZ = 50.0

# ---- 확인용 물체 (world = 시작 시 base_footprint 아래 바닥, 검증계획 9-2) ----
#   (이름, 중심 xyz, 크기 xyz, 색, 충돌 여부)
OBJECTS = [
    ("wall_front", (1.5 + 0.05, 0.0, 0.3), (0.1, 3.0, 0.6), (0.6, 0.6, 0.6), True),
    ("wall_rear", (-1.0 - 0.05, 0.0, 0.3), (0.1, 3.0, 0.6), (0.6, 0.6, 0.6), True),
    ("box_left", (0.0, 0.6 + 0.05, 0.2), (0.3, 0.1, 0.4), (0.3, 0.3, 0.8), True),
    ("marker_front_red", (0.9, 0.3, 0.005), (0.15, 0.15, 0.01), (1.0, 0.0, 0.0), False),
    ("marker_rear_green", (-0.9, 0.2, 0.005), (0.15, 0.15, 0.01), (0.0, 1.0, 0.0), False),
]

ROBOT = "/World/waffle_4wd"
ROOT = ROBOT + "/Geometry/base_footprint/base_link"

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True, "width": 640, "height": 360, "anti_aliasing": 0, "multi_gpu": False})

import isaacsim.core.experimental.utils.app as app_utils  # noqa: E402

app_utils.enable_extension("isaacsim.ros2.bridge")
app.update()

import numpy as np  # noqa: E402
import omni.graph.core as og  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
import usdrt.Sdf  # noqa: E402
from isaacsim.core.experimental.prims import Articulation  # noqa: E402
from isaacsim.core.rendering_manager import RenderingManager  # noqa: E402
from isaacsim.core.simulation_manager import SimulationEvent, SimulationManager  # noqa: E402
from isaacsim.sensors.experimental.physics import Raycast, RaycastSensor  # noqa: E402
from omni.physx.scripts import physicsUtils  # noqa: E402
from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade  # noqa: E402

# 시스템 Lyrical rclpy (조건 A′에서 isaacsim.ros2.core가 불러온 것과 같은 것)
import rclpy  # noqa: E402
from builtin_interfaces.msg import Time as RosTime  # noqa: E402
from geometry_msgs.msg import TwistStamped  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.qos import QoSProfile, qos_profile_sensor_data  # noqa: E402
from sensor_msgs.msg import CameraInfo, Image, JointState, LaserScan  # noqa: E402


def deg_gain(v):
    return v * math.pi / 180.0


def ros_time(t):
    sec = int(math.floor(t))
    return RosTime(sec=sec, nanosec=int(round((t - sec) * 1e9)) % 1000000000)


# ---------------- 장면 ----------------
omni.usd.get_context().new_stage()
app.update()
stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
UsdGeom.Xform.Define(stage, "/World")
scene = UsdPhysics.Scene.Define(stage, "/World/physicsScene")
scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
scene.CreateGravityMagnitudeAttr(9.81)
PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim()).CreateTimeStepsPerSecondAttr(int(round(1.0 / PHYS_DT)))
UsdLux.DomeLight.Define(stage, "/World/DomeLight").CreateIntensityAttr(1000.0)
UsdLux.DistantLight.Define(stage, "/World/SunLight").CreateIntensityAttr(2500.0)
ground = physicsUtils.add_ground_plane(stage, "/World/ground", "Z", 25.0, Gf.Vec3f(0, 0, 0), Gf.Vec3f(0.5))

robot_prim = stage.DefinePrim(ROBOT, "Xform")
robot_prim.GetReferences().AddReference(os.path.abspath(args.usd))
robot_prim.GetVariantSets().GetVariantSet("Physics").SetVariantSelection("physx")
app.update()
if not stage.GetPrimAtPath(ROOT).IsValid():
    raise RuntimeError(f"robot root not found: {ROOT}")

mat = UsdShade.Material.Define(stage, "/World/Materials/contact")
mapi = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
mapi.CreateStaticFrictionAttr(MU_STATIC)
mapi.CreateDynamicFrictionAttr(MU_DYNAMIC)
mapi.CreateRestitutionAttr(0.0)
pmat = PhysxSchema.PhysxMaterialAPI.Apply(mat.GetPrim())
pmat.CreateFrictionCombineModeAttr("average")
pmat.CreateRestitutionCombineModeAttr("average")
for path in [ground + "/CollisionPlane"] + [f"{ROOT}/wheel_{w}/tire_cylinder"
                                            for w in ("front_left", "front_right", "rear_left", "rear_right")]:
    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid():
        raise RuntimeError(f"material target missing: {path}")
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat, UsdShade.Tokens.weakerThanDescendants, "physics")

for p in Usd.PrimRange(stage.GetPrimAtPath(ROOT)):     # 팔·집게 speculative CCD (v10~)
    if p.HasAPI(UsdPhysics.RigidBodyAPI) and p.GetName().startswith(("arm_link_", "gripper_")):
        PhysxSchema.PhysxRigidBodyAPI.Apply(p).CreateEnableSpeculativeCCDAttr(True)

for name, prim in {p.GetName(): p for p in Usd.PrimRange(stage.GetPrimAtPath(ROBOT))
                   if p.IsA(UsdPhysics.RevoluteJoint) or p.IsA(UsdPhysics.PrismaticJoint)}.items():
    if name.startswith("Wheel_"):
        kp, kd, angular = WHEEL_KP, WHEEL_KD, True
    elif name.startswith("RMX52_"):
        kp, kd, angular = ARM_KP, ARM_KD, True
    elif name == "gripper_left_joint":
        kp, kd, angular = GL_KP, GL_KD, False
    elif name == "gripper_right_joint":
        kp, kd, angular = 0.0, 0.0, False
    else:
        continue
    drive = UsdPhysics.DriveAPI.Get(prim, "angular" if angular else "linear")
    drive.CreateStiffnessAttr(deg_gain(kp) if angular else kp)
    drive.CreateDampingAttr(deg_gain(kd) if angular else kd)
    drive.CreateTargetPositionAttr(0.0)
    drive.CreateTargetVelocityAttr(0.0)

for name, center, size, color, collide in OBJECTS:
    cube = UsdGeom.Cube.Define(stage, f"/World/Objects/{name}")
    cube.CreateSizeAttr(1.0)
    cube.AddTranslateOp().Set(Gf.Vec3d(*center))
    cube.AddScaleOp().Set(Gf.Vec3f(*size))
    cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    if collide:
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())


def frame_prim(name):
    hits = [p for p in Usd.PrimRange(stage.GetPrimAtPath(ROBOT)) if p.GetName() == name]
    if len(hits) != 1:
        raise RuntimeError(f"frame prim '{name}': {len(hits)}개")
    return str(hits[0].GetPath())


# 카메라: optical frame(+Z 앞, +Y 아래) 아래에 USD 카메라(−Z 앞, +Y 위) → X축 180°
cam_paths = {}
for cname, (frame, (fx, fy, cx, cy), _, _) in CAMERAS.items():
    path = frame_prim(frame) + "/l2_camera"
    cam = UsdGeom.Camera.Define(stage, path)
    cam.AddRotateXOp().Set(180.0)
    f_mm = 1.93
    cam.GetFocalLengthAttr().Set(f_mm)
    cam.GetHorizontalApertureAttr().Set(W * f_mm / fx)
    cam.GetVerticalApertureAttr().Set(H * f_mm / fy)
    cam.GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 100.0))
    prim = cam.GetPrim()
    prim.ApplyAPI("OmniLensDistortionOpenCvPinholeAPI")       # fx≠fy·주점 위치를 렌더에 그대로 반영
    prim.GetAttribute("omni:lensdistortion:model").Set("opencvPinhole")
    prim.GetAttribute("omni:lensdistortion:opencvPinhole:imageSize").Set(Gf.Vec2i(W, H))
    for k, v in (("fx", fx), ("fy", fy), ("cx", cx), ("cy", cy)):
        prim.GetAttribute(f"omni:lensdistortion:opencvPinhole:{k}").Set(float(v))
    for k in ("k1", "k2", "p1", "p2", "k3", "k4", "k5", "k6", "s1", "s2", "s3", "s4"):
        prim.GetAttribute(f"omni:lensdistortion:opencvPinhole:{k}").Set(0.0)
    cam_paths[cname] = path

# 라이다: scan frame 아래 PhysX 레이캐스트 (scan frame의 +x가 0°, 반시계가 +)
n_rays = int(round(360.0 / LIDAR_RES_DEG))
angles = [math.radians(-180.0 + LIDAR_RES_DEG * i) for i in range(n_rays)]
lidar_sensors = {}
for topic, frame in LIDARS.items():
    rc = Raycast.create(frame_prim(frame) + "/l2_raycast", min_range=LIDAR_MIN, max_range=LIDAR_MAX,
                        ray_origins=[[0.0, 0.0, 0.0]] * n_rays,
                        ray_directions=[[math.cos(a), math.sin(a), 0.0] for a in angles])
    lidar_sensors[topic] = (frame, rc)
app.update()

# ---------------- 기본 브리지 그래프 ----------------
keys = og.Controller.Keys
og.Controller.edit(
    {"graph_path": "/L2Phys", "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [
            ("Tick", "omni.graph.action.OnPlaybackTick"),
            ("SimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ("Clock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ("Odom", "isaacsim.core.nodes.IsaacComputeOdometry"),
            ("PubOdom", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
            ("PubTF", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
        ],
        keys.CONNECT: [
            ("Tick.outputs:tick", "Clock.inputs:execIn"),
            ("Tick.outputs:tick", "Odom.inputs:execIn"),
            ("Odom.outputs:execOut", "PubOdom.inputs:execIn"),
            ("Odom.outputs:execOut", "PubTF.inputs:execIn"),
            ("SimTime.outputs:simulationTime", "Clock.inputs:timeStamp"),
            ("SimTime.outputs:simulationTime", "PubOdom.inputs:timeStamp"),
            ("SimTime.outputs:simulationTime", "PubTF.inputs:timeStamp"),
            ("Odom.outputs:position", "PubOdom.inputs:position"),
            ("Odom.outputs:orientation", "PubOdom.inputs:orientation"),
            ("Odom.outputs:linearVelocity", "PubOdom.inputs:linearVelocity"),
            ("Odom.outputs:angularVelocity", "PubOdom.inputs:angularVelocity"),
            ("Odom.outputs:position", "PubTF.inputs:translation"),
            ("Odom.outputs:orientation", "PubTF.inputs:rotation"),
        ],
        keys.SET_VALUES: [
            ("Clock.inputs:topicName", "/clock"),
            ("Odom.inputs:chassisPrim", [usdrt.Sdf.Path(ROOT)]),
            ("PubOdom.inputs:topicName", "/odom"),
            ("PubOdom.inputs:odomFrameId", "odom"),
            ("PubOdom.inputs:chassisFrameId", "base_footprint"),
            ("PubTF.inputs:topicName", "/tf"),
            ("PubTF.inputs:parentFrameId", "odom"),
            ("PubTF.inputs:childFrameId", "base_footprint"),
        ],
    },
)

cam_nodes, cam_conn, cam_vals = [("OnTick", "omni.graph.action.OnTick")], [], []
for cname, (frame, _, topic, depth) in CAMERAS.items():
    rp, rgb = f"RP_{cname}", f"RGB_{cname}"
    cam_nodes += [(rp, "isaacsim.core.nodes.IsaacCreateRenderProduct"), (rgb, "isaacsim.ros2.bridge.ROS2CameraHelper")]
    cam_conn += [("OnTick.outputs:tick", f"{rp}.inputs:execIn"), (f"{rp}.outputs:execOut", f"{rgb}.inputs:execIn"),
                 (f"{rp}.outputs:renderProductPath", f"{rgb}.inputs:renderProductPath")]
    cam_vals += [(f"{rp}.inputs:cameraPrim", [usdrt.Sdf.Path(cam_paths[cname])]),
                 (f"{rp}.inputs:width", W), (f"{rp}.inputs:height", H),
                 (f"{rgb}.inputs:topicName", f"{topic}/color/image_raw" if cname != "arm" else f"{topic}/image_raw"),
                 (f"{rgb}.inputs:frameId", frame), (f"{rgb}.inputs:type", "rgb")]
    if depth:
        dn = f"Depth_{cname}"
        cam_nodes.append((dn, "isaacsim.ros2.bridge.ROS2CameraHelper"))
        cam_conn += [(f"{rp}.outputs:execOut", f"{dn}.inputs:execIn"),
                     (f"{rp}.outputs:renderProductPath", f"{dn}.inputs:renderProductPath")]
        cam_vals += [(f"{dn}.inputs:topicName", f"{topic}/aligned_depth_to_color/image_raw"),
                     (f"{dn}.inputs:frameId", frame), (f"{dn}.inputs:type", "depth")]
cam_graph, _, _, _ = og.Controller.edit(
    {"graph_path": "/L2Camera", "evaluator_name": "push",
     "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND},
    {keys.CREATE_NODES: cam_nodes, keys.CONNECT: cam_conn, keys.SET_VALUES: cam_vals},
)
og.Controller.evaluate_sync(cam_graph)
app.update()

SimulationManager.setup_simulation(dt=PHYS_DT, device="cpu")
RenderingManager.set_dt(dt=RENDER_DT)
app.update()

# ---------------- Isaac 안 Lyrical rclpy ----------------
if not rclpy.ok():
    rclpy.init()
node = rclpy.create_node("isaac_l2_bridge")
executor = SingleThreadedExecutor()
executor.add_node(node)


def spin_pending(n=12):
    """쌓인 콜백(영상 3개·명령)을 처리한다. Lyrical rclpy 실행기에는 spin_some이 없어 spin_once를 여러 번 부른다"""
    for _ in range(n):
        executor.spin_once(timeout_sec=0.0)


REL = QoSProfile(depth=10)          # reliable: reliable·best effort 구독 모두 연결된다
scan_pubs = {t: node.create_publisher(LaserScan, t, REL) for t in LIDARS}
js_pub = node.create_publisher(JointState, "/joint_states", REL)
info_pubs, info_msgs = {}, {}
for cname, (frame, (fx, fy, cx, cy), topic, _) in CAMERAS.items():
    m = CameraInfo()
    m.width, m.height = W, H
    m.distortion_model = "plumb_bob"
    m.d = [0.0] * 5
    m.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    m.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    m.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    info_msgs[cname] = m
    info_topic = f"{topic}/color/camera_info" if cname != "arm" else f"{topic}/camera_info"
    info_pubs[cname] = node.create_publisher(CameraInfo, info_topic, REL)
    color_topic = f"{topic}/color/image_raw" if cname != "arm" else f"{topic}/image_raw"

    def on_color(img, c=cname):          # 컬러 영상 header를 그대로 복사해 stamp를 정확히 맞춘다
        info = info_msgs[c]
        info.header = img.header
        info_pubs[c].publish(info)
        state["info_sent"][c] += 1

    node.create_subscription(Image, color_topic, on_color, qos_profile_sensor_data)

_wcfg = load_config(args.wheels)
dd = DiffDrive(_wcfg)
WHEEL_ARMATURE = float(_wcfg["wheel_armature_kgm2"])
cmd = {"v": 0.0, "w": 0.0, "t": None, "n": 0}


def on_cmd(msg):
    cmd.update(v=msg.twist.linear.x, w=msg.twist.angular.z, t=state["sim_t"], n=cmd["n"] + 1,
               frame=msg.header.frame_id)
    cmd_w.writerow([f"{state['sim_t']:.6f}", repr(msg.twist.linear.x), repr(msg.twist.angular.z), msg.header.frame_id])


node.create_subscription(TwistStamped, "/cmd_vel", on_cmd, REL)

meta = {
    "phys_dt": PHYS_DT, "render_dt": RENDER_DT, "duration_s": args.duration, "usd": os.path.abspath(args.usd),
    "conditions": {"mu": [MU_STATIC, MU_DYNAMIC], "arm_kp_kd": [ARM_KP, ARM_KD], "wheel_kd": WHEEL_KD,
                   "armature_wheel_arm": [WHEEL_ARMATURE, ARM_ARMATURE], "arm_pose": ARM_POSE_STOW},
    "cameras": {c: {"frame": v[0], "K": list(v[1]), "width": W, "height": H, "topic": v[2], "depth": v[3]}
                for c, v in CAMERAS.items()},
    "lidar": {"hz": LIDAR_HZ, "res_deg": LIDAR_RES_DEG, "range": [LIDAR_MIN, LIDAR_MAX], "topics": LIDARS,
              "angle_min": angles[0], "angle_increment": math.radians(LIDAR_RES_DEG), "n": n_rays},
    "joint_state_hz": JOINT_STATE_HZ,
    "objects": [{"name": o[0], "center": o[1], "size": o[2], "collide": o[4]} for o in OBJECTS],
    "wheels_config": os.path.abspath(args.wheels),
}

truth_f = open(os.path.join(args.out, "truth.csv"), "w", newline="", buffering=1)
truth_w = csv.writer(truth_f)
cmd_f = open(os.path.join(args.out, "cmd_rx.csv"), "w", newline="", buffering=1)
cmd_w = csv.writer(cmd_f)
cmd_w.writerow(["sim_s", "v", "w", "frame_id"])
state = {"sim_t": 0.0, "next_js": 0.0, "next_scan": 0.0, "arm_sent": None, "info_sent": {c: 0 for c in CAMERAS},
         "scan_sent": {t: 0 for t in LIDARS}, "js_sent": 0, "first_scan_saved": False}

timeline = omni.timeline.get_timeline_interface()
timeline.play()
for _ in range(3):
    app.update()
art = Articulation(ROOT)
if not art.is_physics_tensor_entity_valid():
    raise RuntimeError("articulation tensor view invalid after play")
dof = list(art.dof_names)
wheel_idx = [dof.index(j) for j in dd.joints]
arm_names = [n for n in dof if n.startswith("RMX52_")]
arm_idx = [dof.index(n) for n in arm_names]
pos_idx = [i for i, n in enumerate(dof) if not n.startswith("Wheel_")]
art.set_dof_max_velocities(np.full((1, len(dof)), 1.0e3, np.float32))     # 제어기에서 한계를 준다(물리 시험 v4~)
art.set_dof_armatures([WHEEL_ARMATURE], dof_indices=wheel_idx)
art.set_dof_armatures([ARM_ARMATURE], dof_indices=arm_idx)
stow = np.zeros(len(dof), np.float32)
for n, v in zip(arm_names, ARM_POSE_STOW):
    stow[dof.index(n)] = v
meta["dof_names"] = dof
meta["wheel_joints"] = dd.joints
with open(os.path.join(args.out, "meta.json"), "w") as f:
    json.dump(meta, f, indent=2)
truth_w.writerow(["sim_s", "x", "y", "z", "qw", "qx", "qy", "qz", "vx", "vy", "wz", "cmd_v", "cmd_w", "cmd_age_s"]
                 + [f"tgt_{j}" for j in dd.joints] + [f"q_{n}" for n in dof])

sensors = {t: (frame, RaycastSensor(rc)) for t, (frame, rc) in lidar_sensors.items()}


def pre_step(dt, context):
    t = state["sim_t"]
    prev = state["arm_sent"] if state["arm_sent"] is not None else np.zeros(len(dof), np.float32)
    tgt = prev + np.clip(stow - prev, -ARM_RATE * dt, ARM_RATE * dt)          # S0로 접고 유지(제어기 속도 한계)
    state["arm_sent"] = tgt
    art.set_dof_position_targets(tgt[pos_idx].reshape(1, -1), dof_indices=pos_idx)
    rates = dd.rates_at(t, cmd["t"], cmd["v"], cmd["w"])
    state["wheel_tgt"] = rates
    art.set_dof_velocity_targets(np.array([rates], np.float32), dof_indices=wheel_idx)


def post_step(dt, context):
    # Isaac 시뮬레이션 시간을 그대로 쓴다(/clock·영상·TF stamp와 같은 출처). 직접 세면 콜백 등록 전 재생한 프레임만큼 늦어진다
    state["sim_t"] = float(SimulationManager.get_simulation_time())
    t = state["sim_t"]
    if t + 1e-9 >= state["next_js"]:
        state["next_js"] += 1.0 / JOINT_STATE_HZ
        q = art.get_dof_positions().numpy()[0]
        dq = art.get_dof_velocities().numpy()[0]
        eff = art.get_dof_projected_joint_forces().numpy()[0]
        js = JointState()
        js.header.stamp = ros_time(t)
        js.name = dof
        js.position, js.velocity, js.effort = [float(x) for x in q], [float(x) for x in dq], [float(x) for x in eff]
        js_pub.publish(js)
        state["js_sent"] += 1
        bp, bq = art.get_world_poses()
        bp, bq = bp.numpy()[0], bq.numpy()[0]
        lv, av = art.get_velocities()
        lv, av = lv.numpy()[0], av.numpy()[0]
        age = (t - cmd["t"]) if cmd["t"] is not None else -1.0
        truth_w.writerow([f"{t:.6f}"] + [repr(float(x)) for x in bp] + [repr(float(x)) for x in bq]
                         + [repr(float(lv[0])), repr(float(lv[1])), repr(float(av[2]))]
                         + [repr(cmd["v"]), repr(cmd["w"]), f"{age:.6f}"]
                         + [repr(float(x)) for x in state.get("wheel_tgt", [0.0] * len(dd.joints))]
                         + [repr(float(x)) for x in q])
    if t + 1e-9 >= state["next_scan"]:
        state["next_scan"] += 1.0 / LIDAR_HZ
        for topic, (frame, sensor) in sensors.items():
            # Isaac 6.1 RaycastSensor의 depths는 모든 빔에 첫 빔 값을 되풀이한다(2026-10-25 진단). hit_positions(센서 frame)는
            # 빔마다 맞으므로 그 거리를 쓴다. 맞지 않은 빔은 (0, 0, 0)으로 나오므로 최소 거리 미만 = 반사 없음
            hits = np.asarray(sensor.get_data()["hit_positions"], dtype=np.float64).reshape(-1, 3)
            if hits.shape[0] != n_rays:
                continue
            depths = np.linalg.norm(hits, axis=1)
            ranges = np.where((depths >= LIDAR_MIN) & (depths < LIDAR_MAX), depths, np.inf)
            s = LaserScan()
            s.header.stamp = ros_time(t)
            s.header.frame_id = frame
            s.angle_min = angles[0]
            s.angle_increment = math.radians(LIDAR_RES_DEG)
            s.angle_max = angles[-1]
            s.scan_time = 1.0 / LIDAR_HZ
            s.time_increment = 0.0
            s.range_min, s.range_max = LIDAR_MIN, LIDAR_MAX
            s.ranges = [float(x) for x in ranges]
            scan_pubs[topic].publish(s)
            state["scan_sent"][topic] += 1


SimulationManager.register_callback(pre_step, event=SimulationEvent.PHYSICS_PRE_STEP)
SimulationManager.register_callback(post_step, event=SimulationEvent.PHYSICS_POST_STEP)

ticks_f = open(os.path.join(args.out, "ticks.csv"), "w", newline="", buffering=1)
ticks_w = csv.writer(ticks_f)
ticks_w.writerow(["wall_s", "sim_s"])
wall0 = time.time()
next_report = 30.0
print(f"[l2] play, duration {args.duration:.0f} s (sim)", flush=True)
while app.is_running() and state["sim_t"] < args.duration:
    app.update()
    spin_pending()
    ticks_w.writerow([f"{time.time():.6f}", f"{state['sim_t']:.6f}"])
    if state["sim_t"] >= next_report:
        print(f"[l2] sim {state['sim_t']:.0f} s, wall {time.time() - wall0:.0f} s, js {state['js_sent']}, "
              f"scan {state['scan_sent']}, info {state['info_sent']}, cmd {cmd['n']}", flush=True)
        next_report += 30.0
for _ in range(20):                       # 마지막 영상에 대한 camera_info까지 내보낸다
    executor.spin_once(timeout_sec=0.01)
with open(os.path.join(args.out, "scene_counts.json"), "w") as f:
    json.dump({"sim_s": state["sim_t"], "wall_s": time.time() - wall0, "js_sent": state["js_sent"],
               "scan_sent": state["scan_sent"], "info_sent": state["info_sent"], "cmd_rx": cmd["n"]}, f, indent=2)
print(f"[l2] done, sim {state['sim_t']:.1f} s, wall {time.time() - wall0:.0f} s", flush=True)
truth_f.close()
cmd_f.close()
ticks_f.close()
node.destroy_node()
timeline.stop()
app.close()
