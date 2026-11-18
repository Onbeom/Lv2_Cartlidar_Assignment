"""SLAM 비교 기록 장면: 과제 미로 월드(Cartlidar/sim_world, Isaac 6.1 USD) + Waffle 4WD 261027_1711(v12 조건, 바퀴 armature 0.007244) + 정해진 경로 주행.

4단계 L2 장면(261025_0307/l2/scene_l2.py)을 바탕으로 바꾼 것(원본은 그대로 둠):
  - 장면: 월드 USD를 그대로 연다(로봇·바닥 마찰·벽 포함). 조명은 --light_level(0~1)로 월드 생성기와 같은 식으로 다시 정함.
  - 카메라: 앞 RealSense 적외선 좌(infra1)·우(infra2) 640×480 30 Hz, 실물 D435(261822072247) 적외선 K.
    기본 브리지가 rgb8로 낸 영상을 이 프로세스에서 mono8(Y8, 실물과 같은 형식)로 바꿔 header 그대로 다시 낸다.
  - 오도메트리: 바퀴 관절 속도로 계산(엔코더와 같은 방식, 미끄럼 오차 포함). /odom과 TF odom→base_footprint를 rclpy로 냄.
    L2의 Isaac 계산 오도메트리(차체 실제 움직임 적분)는 정답과 같아 비교에 쓰지 않는다.
  - 주행: route.py 경로(출발 칸 → 모든 칸 → 출발 칸)를 정답 자세로 따라감(제자리 회전 뒤 직진). 정답 자세는 주행 제어에만 쓰고
    ROS로 내지 않는다(truth.csv에만 기록 → 평가용).
  - 라이다 2개(PhysX 레이캐스트 10 Hz·1°·0.12~8 m), /joint_states 50 Hz, /clock은 L2와 같다.
"""

import argparse
import csv
import json
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
WORK_DIR = os.path.dirname(HERE)
TOP = os.path.dirname(WORK_DIR)
L2 = os.path.join(TOP, "261025_0307", "l2")                       # 기구학 코드(wheel_kinematics.py)
L2_1711 = os.path.join(TOP, "261027_1744", "l2")                # 261027_1711 바퀴 정의(y ±0.1546, armature 0.007244)

parser = argparse.ArgumentParser()
parser.add_argument("--out", required=True)
parser.add_argument("--world", default=os.path.join(WORK_DIR, "worlds_1711/seed_1"))   # make_world_1711.py 로 만든 복사본(로봇 261027_1711)
parser.add_argument("--light_level", type=float, required=True, help="0 = 가장 어두움 ~ 1 = 가장 밝음")
parser.add_argument("--max_waypoints", type=int, default=0, help="0이면 전체 경로, 아니면 앞에서부터 이 개수만(도구 확인용)")
parser.add_argument("--v", type=float, default=0.20, help="[m/s] 직진 속도")
parser.add_argument("--w", type=float, default=0.60, help="[rad/s] 제자리 회전 최대 각속도")
parser.add_argument("--max_sim_s", type=float, default=900.0)
parser.add_argument("--wheels", default=os.path.join(L2_1711, "config/wheels.yaml"))
args, _ = parser.parse_known_args()
os.makedirs(args.out, exist_ok=True)
sys.path.insert(0, L2)
sys.path.insert(0, HERE)
from route import route_waypoints  # noqa: E402
from wheel_kinematics import DiffDrive, load_config  # noqa: E402

# ---- 조건 (L2·물리 시험 v12와 같게) ----
PHYS_DT = 1.0 / 120.0
RENDER_DT = 1.0 / 30.0
ARM_KP, ARM_KD = 400.0, 36.0
GL_KP, GL_KD = 1000.0, 10.0
WHEEL_KP, WHEEL_KD = 0.0, 1.0
ARM_ARMATURE = 0.045                    # 바퀴 armature 는 --wheels 의 wheel_armature_kgm2
ARM_RATE = 43 * 2 * math.pi / 60
ARM_POSE_STOW = [0.0, 1.4, -1.0, -1.3, 0.0]
LIGHT_RANGE = {"dome": (300.0, 600.0), "rect": (0.0, 20000.0)}     # 월드 생성기(build_task_world.py)와 같음

W, H = 640, 480
IR_K = (385.012817382812, 385.012817382812, 320.859619140625, 239.928024291992)   # 실물 D435 Infrared 1·2 640x480
IR = {"infra1": "top_camera_front_infra1_optical_frame", "infra2": "top_camera_front_infra2_optical_frame"}
IR_TOPIC = "/top_camera_front/{}/image_rect_raw"
IR_INFO = "/top_camera_front/{}/camera_info"
LIDAR_HZ, LIDAR_RES_DEG, LIDAR_MIN, LIDAR_MAX = 10.0, 1.0, 0.12, 8.0
LIDARS = {"/scan_front": "lidar_front_scan", "/scan_rear": "lidar_rear_scan"}
JOINT_STATE_HZ = 50.0
ODOM_HZ = 30.0

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
from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdLux, UsdPhysics  # noqa: E402

import rclpy  # noqa: E402
from builtin_interfaces.msg import Time as RosTime  # noqa: E402
from geometry_msgs.msg import TransformStamped  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.qos import QoSProfile, qos_profile_sensor_data  # noqa: E402
from sensor_msgs.msg import CameraInfo, Image, JointState, LaserScan  # noqa: E402
from tf2_msgs.msg import TFMessage  # noqa: E402


def deg_gain(v):
    return v * math.pi / 180.0


def ros_time(t):
    sec = int(math.floor(t))
    return RosTime(sec=sec, nanosec=int(round((t - sec) * 1e9)) % 1000000000)


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


# ---------------- 장면 ----------------
world_usd = os.path.join(args.world, "task_world.usda")
omni.usd.get_context().open_stage(world_usd)
app.update()
stage = omni.usd.get_context().get_stage()
if not stage.GetPrimAtPath(ROOT).IsValid():
    raise RuntimeError(f"robot root not found: {ROOT}")
scene_prim = stage.GetPrimAtPath("/World/physicsScene")
PhysxSchema.PhysxSceneAPI.Apply(scene_prim).CreateTimeStepsPerSecondAttr(int(round(1.0 / PHYS_DT)))

level = min(max(args.light_level, 0.0), 1.0)
dome = LIGHT_RANGE["dome"][0] + level * (LIGHT_RANGE["dome"][1] - LIGHT_RANGE["dome"][0])
rect = LIGHT_RANGE["rect"][0] + level * (LIGHT_RANGE["rect"][1] - LIGHT_RANGE["rect"][0])
UsdLux.DomeLight(stage.GetPrimAtPath("/World/Lights/dome")).GetIntensityAttr().Set(dome)
n_rect = 0
for p in stage.GetPrimAtPath("/World/Lights").GetChildren():
    if p.GetName().startswith("ceiling_"):
        UsdLux.RectLight(p).GetIntensityAttr().Set(rect)
        n_rect += 1

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


def frame_prim(name):
    hits = [p for p in Usd.PrimRange(stage.GetPrimAtPath(ROBOT)) if p.GetName() == name]
    if len(hits) != 1:
        raise RuntimeError(f"frame prim '{name}': {len(hits)}개")
    return str(hits[0].GetPath())


cam_paths = {}
fx, fy, cx, cy = IR_K
for cname, frame in IR.items():
    path = frame_prim(frame) + "/slam_camera"
    cam = UsdGeom.Camera.Define(stage, path)
    cam.AddRotateXOp().Set(180.0)
    f_mm = 1.93
    cam.GetFocalLengthAttr().Set(f_mm)
    cam.GetHorizontalApertureAttr().Set(W * f_mm / fx)
    cam.GetVerticalApertureAttr().Set(H * f_mm / fy)
    cam.GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 100.0))
    prim = cam.GetPrim()
    prim.ApplyAPI("OmniLensDistortionOpenCvPinholeAPI")
    prim.GetAttribute("omni:lensdistortion:model").Set("opencvPinhole")
    prim.GetAttribute("omni:lensdistortion:opencvPinhole:imageSize").Set(Gf.Vec2i(W, H))
    for k, v in (("fx", fx), ("fy", fy), ("cx", cx), ("cy", cy)):
        prim.GetAttribute(f"omni:lensdistortion:opencvPinhole:{k}").Set(float(v))
    for k in ("k1", "k2", "p1", "p2", "k3", "k4", "k5", "k6", "s1", "s2", "s3", "s4"):
        prim.GetAttribute(f"omni:lensdistortion:opencvPinhole:{k}").Set(0.0)
    cam_paths[cname] = path

n_rays = int(round(360.0 / LIDAR_RES_DEG))
angles = [math.radians(-180.0 + LIDAR_RES_DEG * i) for i in range(n_rays)]
lidar_sensors = {}
for topic, frame in LIDARS.items():
    rc = Raycast.create(frame_prim(frame) + "/slam_raycast", min_range=LIDAR_MIN, max_range=LIDAR_MAX,
                        ray_origins=[[0.0, 0.0, 0.0]] * n_rays,
                        ray_directions=[[math.cos(a), math.sin(a), 0.0] for a in angles])
    lidar_sensors[topic] = (frame, rc)
app.update()

# ---------------- 기본 브리지: /clock, 적외선 rgb (rgb는 이 프로세스가 mono8로 바꿔 다시 냄) ----------------
keys = og.Controller.Keys
og.Controller.edit(
    {"graph_path": "/SlamClock", "evaluator_name": "execution"},
    {keys.CREATE_NODES: [("Tick", "omni.graph.action.OnPlaybackTick"),
                         ("SimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                         ("Clock", "isaacsim.ros2.bridge.ROS2PublishClock")],
     keys.CONNECT: [("Tick.outputs:tick", "Clock.inputs:execIn"),
                    ("SimTime.outputs:simulationTime", "Clock.inputs:timeStamp")],
     keys.SET_VALUES: [("Clock.inputs:topicName", "/clock")]},
)
cam_nodes, cam_conn, cam_vals = [("OnTick", "omni.graph.action.OnTick")], [], []
for cname, frame in IR.items():
    rp, rgb = f"RP_{cname}", f"RGB_{cname}"
    cam_nodes += [(rp, "isaacsim.core.nodes.IsaacCreateRenderProduct"), (rgb, "isaacsim.ros2.bridge.ROS2CameraHelper")]
    cam_conn += [("OnTick.outputs:tick", f"{rp}.inputs:execIn"), (f"{rp}.outputs:execOut", f"{rgb}.inputs:execIn"),
                 (f"{rp}.outputs:renderProductPath", f"{rgb}.inputs:renderProductPath")]
    cam_vals += [(f"{rp}.inputs:cameraPrim", [usdrt.Sdf.Path(cam_paths[cname])]),
                 (f"{rp}.inputs:width", W), (f"{rp}.inputs:height", H),
                 (f"{rgb}.inputs:topicName", f"/slam_rgb/{cname}"),
                 (f"{rgb}.inputs:frameId", frame), (f"{rgb}.inputs:type", "rgb")]
cam_graph, _, _, _ = og.Controller.edit(
    {"graph_path": "/SlamCamera", "evaluator_name": "push",
     "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND},
    {keys.CREATE_NODES: cam_nodes, keys.CONNECT: cam_conn, keys.SET_VALUES: cam_vals},
)
og.Controller.evaluate_sync(cam_graph)
app.update()
for cname in IR:     # 렌더 제품이 만들어졌는지(도구 확인용 출력)
    rpp = og.Controller.attribute(f"/SlamCamera/RP_{cname}.outputs:renderProductPath").get()
    print(f"[slam] render product {cname}: {rpp}, camera {cam_paths[cname]}", flush=True)

SimulationManager.setup_simulation(dt=PHYS_DT, device="cpu")
RenderingManager.set_dt(dt=RENDER_DT)
app.update()

# ---------------- Lyrical rclpy ----------------
if not rclpy.ok():
    rclpy.init()
node = rclpy.create_node("isaac_slam_bridge")
executor = SingleThreadedExecutor()
executor.add_node(node)
REL = QoSProfile(depth=10)
scan_pubs = {t: node.create_publisher(LaserScan, t, REL) for t in LIDARS}
js_pub = node.create_publisher(JointState, "/joint_states", REL)
odom_pub = node.create_publisher(Odometry, "/odom", REL)
tf_pub = node.create_publisher(TFMessage, "/tf", QoSProfile(depth=100))
ir_pubs, info_pubs, info_msgs = {}, {}, {}
state = {"sim_t": 0.0, "next_js": 0.0, "next_scan": 0.0, "next_odom": 0.0, "arm_sent": None,
         "ir_sent": {c: 0 for c in IR}, "ir_in": {c: 0 for c in IR}, "scan_sent": {t: 0 for t in LIDARS},
         "js_sent": 0, "odom_sent": 0, "odom": [0.0, 0.0, 0.0], "wp": 0, "phase": "turn", "done_t": None}
for cname, frame in IR.items():
    m = CameraInfo()
    m.width, m.height = W, H
    m.distortion_model = "plumb_bob"
    m.d = [0.0] * 5
    m.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    m.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    # 오른쪽(infra2) P[3] = −fx·baseline. baseline = URDF infra1→infra2 거리(실행 때 USD에서 읽어 meta에 남김)
    m.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    info_msgs[cname] = m
    ir_pubs[cname] = node.create_publisher(Image, IR_TOPIC.format(cname), qos_profile_sensor_data)
    info_pubs[cname] = node.create_publisher(CameraInfo, IR_INFO.format(cname), REL)

    def on_rgb(img, c=cname):
        state["ir_in"][c] += 1
        a = np.frombuffer(bytes(img.data), np.uint8)
        if a.size != img.height * img.width * 3:
            return
        a = a.reshape(img.height, img.width, 3).astype(np.float32)
        y = np.clip(0.299 * a[..., 0] + 0.587 * a[..., 1] + 0.114 * a[..., 2] + 0.5, 0, 255).astype(np.uint8)
        out = Image()
        out.header = img.header
        out.height, out.width, out.encoding, out.is_bigendian, out.step = img.height, img.width, "mono8", 0, img.width
        out.data = y.tobytes()
        ir_pubs[c].publish(out)
        info = info_msgs[c]
        info.header = img.header
        info_pubs[c].publish(info)
        state["ir_sent"][c] += 1

    node.create_subscription(Image, f"/slam_rgb/{cname}", on_rgb, qos_profile_sensor_data)

wcfg = load_config(args.wheels)
dd = DiffDrive(wcfg)
WHEEL_ARMATURE = float(wcfg["wheel_armature_kgm2"])
wps, wp_ids, _, _ = route_waypoints(os.path.join(args.world, "layout.json"))
if args.max_waypoints > 0:
    wps, wp_ids = wps[:args.max_waypoints], wp_ids[:args.max_waypoints]

truth_f = open(os.path.join(args.out, "truth.csv"), "w", newline="", buffering=1)
truth_w = csv.writer(truth_f)
truth_w.writerow(["sim_s", "x", "y", "z", "qw", "qx", "qy", "qz", "yaw", "odom_x", "odom_y", "odom_yaw",
                  "cmd_v", "cmd_w", "wp", "phase"])

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
art.set_dof_max_velocities(np.full((1, len(dof)), 1.0e3, np.float32))
art.set_dof_armatures([WHEEL_ARMATURE], dof_indices=wheel_idx)
art.set_dof_armatures([ARM_ARMATURE], dof_indices=arm_idx)
stow = np.zeros(len(dof), np.float32)
for n, v in zip(arm_names, ARM_POSE_STOW):
    stow[dof.index(n)] = v
state["arm_sent"] = art.get_dof_positions().numpy()[0].astype(np.float32)      # 월드의 무작위 팔 자세에서 S0로 접음

# 적외선 baseline(USD 프레임 사이 거리)과 정답 기준(odom 원점 = 시작 때 base_footprint)
xc = UsdGeom.XformCache()
p1 = xc.GetLocalToWorldTransform(stage.GetPrimAtPath(frame_prim(IR["infra1"]))).ExtractTranslation()
p2 = xc.GetLocalToWorldTransform(stage.GetPrimAtPath(frame_prim(IR["infra2"]))).ExtractTranslation()
baseline = float((p2 - p1).GetLength())
info_msgs["infra2"].p[3] = -fx * baseline
meta = {"world": os.path.abspath(args.world), "wheels": os.path.abspath(args.wheels), "wheel_armature": WHEEL_ARMATURE, "light_level": level, "dome": dome, "rect": rect, "n_rect": n_rect,
        "phys_dt": PHYS_DT, "render_dt": RENDER_DT, "ir_K": list(IR_K), "ir_baseline_m": baseline,
        "ir_topics": {c: [IR_TOPIC.format(c), IR_INFO.format(c), f] for c, f in IR.items()},
        "lidar": {"hz": LIDAR_HZ, "res_deg": LIDAR_RES_DEG, "range": [LIDAR_MIN, LIDAR_MAX], "topics": LIDARS},
        "odom": "바퀴 관절 속도(엔코더 방식), " + str(ODOM_HZ) + " Hz",
        "route": {"ids": wp_ids, "v": args.v, "w_max": args.w}, "dof_names": dof}


def odom_rates():
    q_dot = art.get_dof_velocities().numpy()[0]
    r = wcfg["radius_m"]
    left = [q_dot[dof.index(w["joint"])] * w["sign"] * r for w in wcfg["wheels"] if w["y_m"] > 0]
    right = [q_dot[dof.index(w["joint"])] * w["sign"] * r for w in wcfg["wheels"] if w["y_m"] < 0]
    track = 2.0 * abs(wcfg["wheels"][0]["y_m"])
    vl, vr = float(np.mean(left)), float(np.mean(right))
    return 0.5 * (vl + vr), (vr - vl) / track


def truth_pose():
    bp, bq = art.get_world_poses()
    bp, bq = bp.numpy()[0], bq.numpy()[0]
    w_, x_, y_, z_ = bq
    yaw = math.atan2(2 * (w_ * z_ + x_ * y_), 1 - 2 * (y_ * y_ + z_ * z_))
    return bp, bq, yaw


cmd = {"v": 0.0, "w": 0.0}


def controller(t):
    """정답 자세로 경유점을 따라감: 방향 오차 > 4°면 제자리 회전, 아니면 직진하며 방향 보정. 마지막 경유점 뒤 정지."""
    if state["wp"] >= len(wps):
        return 0.0, 0.0
    bp, _, yaw = truth_pose()
    tx, ty = wps[state["wp"]]
    dx, dy = tx - bp[0], ty - bp[1]
    dist = math.hypot(dx, dy)
    if dist < 0.03:
        state["wp"] += 1
        state["phase"] = "turn"
        if state["wp"] >= len(wps):
            state["done_t"] = t
        return 0.0, 0.0
    err = wrap(math.atan2(dy, dx) - yaw)
    if state["phase"] == "turn":
        if abs(err) < math.radians(2.0):
            state["phase"] = "go"
            return 0.0, 0.0
        return 0.0, float(np.clip(2.0 * err, -args.w, args.w))
    if abs(err) > math.radians(4.0) and dist > 0.10:
        state["phase"] = "turn"
        return 0.0, 0.0
    v = min(args.v, 1.0 * dist + 0.03)
    return v, float(np.clip(2.0 * err, -0.5, 0.5))


def pre_step(dt, context):
    t = state["sim_t"]
    prev = state["arm_sent"]
    tgt = prev + np.clip(stow - prev, -ARM_RATE * dt, ARM_RATE * dt)
    state["arm_sent"] = tgt
    art.set_dof_position_targets(tgt[pos_idx].reshape(1, -1), dof_indices=pos_idx)
    settled = float(np.max(np.abs(stow - tgt))) < 1e-4 and t > 2.0        # 팔을 다 접고 2 s 뒤 출발
    v, w = controller(t) if settled else (0.0, 0.0)
    cmd.update(v=v, w=w)
    rates = dd.rates_at(t, t, v, w)
    art.set_dof_velocity_targets(np.array([rates], np.float32), dof_indices=wheel_idx)


def post_step(dt, context):
    state["sim_t"] = float(SimulationManager.get_simulation_time())
    t = state["sim_t"]
    v, w = odom_rates()
    o = state["odom"]
    o[0] += v * math.cos(o[2] + 0.5 * w * dt) * dt
    o[1] += v * math.sin(o[2] + 0.5 * w * dt) * dt
    o[2] = wrap(o[2] + w * dt)
    if t + 1e-9 >= state["next_odom"]:
        state["next_odom"] += 1.0 / ODOM_HZ
        qz, qw = math.sin(o[2] / 2), math.cos(o[2] / 2)
        od = Odometry()
        od.header.stamp = ros_time(t)
        od.header.frame_id, od.child_frame_id = "odom", "base_footprint"
        od.pose.pose.position.x, od.pose.pose.position.y = o[0], o[1]
        od.pose.pose.orientation.z, od.pose.pose.orientation.w = qz, qw
        od.twist.twist.linear.x, od.twist.twist.angular.z = v, w
        odom_pub.publish(od)
        tf = TransformStamped()
        tf.header = od.header
        tf.child_frame_id = "base_footprint"
        tf.transform.translation.x, tf.transform.translation.y = o[0], o[1]
        tf.transform.rotation.z, tf.transform.rotation.w = qz, qw
        tf_pub.publish(TFMessage(transforms=[tf]))
        state["odom_sent"] += 1
    if t + 1e-9 >= state["next_js"]:
        state["next_js"] += 1.0 / JOINT_STATE_HZ
        q = art.get_dof_positions().numpy()[0]
        dq = art.get_dof_velocities().numpy()[0]
        js = JointState()
        js.header.stamp = ros_time(t)
        js.name = dof
        js.position, js.velocity = [float(x) for x in q], [float(x) for x in dq]
        js_pub.publish(js)
        state["js_sent"] += 1
        bp, bq, yaw = truth_pose()
        truth_w.writerow([f"{t:.6f}"] + [repr(float(x)) for x in bp] + [repr(float(x)) for x in bq]
                         + [repr(yaw), repr(o[0]), repr(o[1]), repr(o[2]), repr(cmd["v"]), repr(cmd["w"]),
                            state["wp"], state["phase"]])
    if t + 1e-9 >= state["next_scan"]:
        state["next_scan"] += 1.0 / LIDAR_HZ
        for topic, (frame, sensor) in sensors.items():
            hits = np.asarray(sensor.get_data()["hit_positions"], dtype=np.float64).reshape(-1, 3)
            if hits.shape[0] != n_rays:
                continue
            depths = np.linalg.norm(hits, axis=1)
            ranges = np.where((depths >= LIDAR_MIN) & (depths < LIDAR_MAX), depths, np.inf)
            s = LaserScan()
            s.header.stamp = ros_time(t)
            s.header.frame_id = frame
            s.angle_min, s.angle_increment, s.angle_max = angles[0], math.radians(LIDAR_RES_DEG), angles[-1]
            s.scan_time, s.time_increment = 1.0 / LIDAR_HZ, 0.0
            s.range_min, s.range_max = LIDAR_MIN, LIDAR_MAX
            s.ranges = [float(x) for x in ranges]
            scan_pubs[topic].publish(s)
            state["scan_sent"][topic] += 1


sensors = {t: (frame, RaycastSensor(rc)) for t, (frame, rc) in lidar_sensors.items()}
_, _, yaw0 = truth_pose()
meta["start_truth_yaw"] = yaw0
with open(os.path.join(args.out, "meta.json"), "w") as f:
    json.dump(meta, f, indent=2, ensure_ascii=False)
SimulationManager.register_callback(pre_step, event=SimulationEvent.PHYSICS_PRE_STEP)
SimulationManager.register_callback(post_step, event=SimulationEvent.PHYSICS_POST_STEP)

wall0 = time.time()
next_report = 30.0
print(f"[slam] play, light {level}, waypoints {len(wps)}, baseline {baseline:.6f} m", flush=True)
while app.is_running() and state["sim_t"] < args.max_sim_s:
    app.update()
    for _ in range(12):
        executor.spin_once(timeout_sec=0.0)
    if state["done_t"] is not None and state["sim_t"] > state["done_t"] + 3.0:      # 도착 뒤 3 s 정지 기록
        break
    if state["sim_t"] >= next_report:
        print(f"[slam] sim {state['sim_t']:.0f} s, wall {time.time() - wall0:.0f} s, wp {state['wp']}/{len(wps)}, "
              f"ir {state['ir_sent']}, scan {state['scan_sent']}", flush=True)
        next_report += 30.0
for _ in range(20):
    executor.spin_once(timeout_sec=0.01)
counts = {"sim_s": state["sim_t"], "wall_s": time.time() - wall0, "done_t": state["done_t"], "wp_reached": state["wp"],
          "n_wp": len(wps), "js_sent": state["js_sent"], "odom_sent": state["odom_sent"], "scan_sent": state["scan_sent"],
          "ir_in": state["ir_in"], "ir_sent": state["ir_sent"]}
with open(os.path.join(args.out, "scene_counts.json"), "w") as f:
    json.dump(counts, f, indent=2)
print(f"[slam] done {json.dumps(counts)}", flush=True)
truth_f.close()
node.destroy_node()
timeline.stop()
app.close()
