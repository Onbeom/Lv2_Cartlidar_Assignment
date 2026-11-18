"""Isaac 실시간 연결 장면(모의, 261101): 261030_1943/record_slam_lab.py 사본에 실시간 연결용 선택만 더함. 바꾼 것:
  - /odom 50 Hz(실물 팀 controllers_4wd.yaml publish_rate · sensors.yaml odom_rate_hz 와 같게, 녹화 때 30 Hz)
  - --odom_rot_scale S: 바퀴 odom 회전 증분 · 각속도를 1/S(녹화 확인 1.149 → 1.15, 시험 bag make_test_bag.py 와 같은 보정)
  - --odom_tf 0: odom→base_link TF 를 내지 않음(launch 의 EKF 가 냄 — 같은 TF 를 두 노드가 내지 않게)
  - --mode cmd_vel: 경유점 제어 대신 --cmd_topic(TwistStamped) 명령으로 바퀴 구동(--cmd_timeout s 넘게 끊기면 정지), --stop_file 이 생기면 끝
  - --start x y yaw(USD 좌표): 시작 자세
  - --imu_topic: 자이로 yaw rate 만(sensor_msgs/Imu, frame imu_link, 차체 각속도 z + 정규 잡음 σ --imu_gyro_sigma, --imu_hz). 팀 모델 sensors.yaml imu.sim 값
    (100 Hz · σ 0.0017 rad/s, 둘 다 가정)을 씀. 바이어스 · 두 IMU 평균 · 축 어긋남은 흉내 내지 않음. 자세 · 가속도는 '없음'(공분산 −1)
  - --truth_topic: 채점용 참값 PoseStamped(frame map = 발제 좌표 = USD + --truth_offset, 50 Hz). 채점기만 받고 Nav2 노드는 받지 않음
원본 설명:
SLAM 녹화 장면(모의): 새 미로 maze.zip + 새 모델 waffle_4wd 261028(Isaac Lab 3.0 가벼운 판, 시작 자세 drive).

이전 녹화(261027_0953/slam/scene_slam.py, 구모델 · 옛 미로)와 같은 토픽 · 같은 방식으로 내되, 로봇 · 센서는 새 모델 공유본
(261029_1030/urdf_261028/isaac, 생성 파일은 고치지 않고 불러 쓰기만)의 설정을 쓴다.
  - 라이다: LDS-03 x2 (sensors_cfg.lds03_cfg, 400 광선, 0.05~12 m, 10 Hz, 차체 가림 상자 포함). 맞은 점을 센서 frame(laser_front/rear)
    으로 옮겨 각도 칸(0.9°)에 넣어 LaserScan 으로 낸다. 차체 가림에 맞은 짧은 거리도 그대로 낸다(실물과 같게, 거르기는 SLAM 설정에서).
  - 카메라: 앞 D435 적외선 좌 · 우(sensors_cfg.d435_cfg 'infra1' | 'infra2', rgb 렌더 → mono8, 투사 점 무늬 없음), 30 Hz.
    CameraInfo 는 모델 HFOV 로 계산한 K(실물 보정값 아님), 오른쪽 P[3] = −fx · baseline(sensors.json 위치에서).
  - 오도메트리: 바퀴 관절 속도로 계산(간격 0.317 m, 반지름 0.033 m, 보정 배수 없음 = 엔코더 그대로). /odom, TF odom→base_link 30 Hz(새 URDF 루트, 팀 dev 규약).
  - 정답: base_link 실제 자세 → truth.csv(50 Hz)에만. 주행 제어에만 쓰고 ROS 로 내지 않는다.
  - 주행: route_maze.json 경유점(제자리 회전 뒤 직진). /clock 60 Hz, /joint_states 50 Hz.
"""

import argparse
import csv
import json
import math
import os
import time

from isaaclab.app import AppLauncher

HERE = os.path.dirname(os.path.abspath(__file__))
TOP = os.path.dirname(HERE)
MODEL = os.path.join(TOP, "261029_1030/urdf_261028/isaac")
MAZE_DIR = os.path.join(TOP, "261030_1943")
parser = argparse.ArgumentParser()
parser.add_argument("--out", required=True)
parser.add_argument("--light_level", type=float, required=True, help="0 = 가장 어두움 ~ 1 = 가장 밝음 (이전 녹화와 같은 식)")
parser.add_argument("--max_waypoints", type=int, default=0, help="0 = 전체, 아니면 앞에서부터 이 개수만(도구 확인용)")
parser.add_argument("--v", type=float, default=0.20)
parser.add_argument("--w", type=float, default=0.60)
parser.add_argument("--max_sim_s", type=float, default=1500.0)
parser.add_argument("--maze", default=os.path.join(MAZE_DIR, "maze_src/maze/maze.usda"))
parser.add_argument("--route", default=os.path.join(MAZE_DIR, "route_maze.json"))
parser.add_argument("--odom_rot_scale", type=float, default=1.15)
parser.add_argument("--odom_tf", type=int, default=0)
parser.add_argument("--mode", choices=["route", "cmd_vel"], default="route")
parser.add_argument("--cmd_topic", default="/cmd_vel_nav")
parser.add_argument("--cmd_timeout", type=float, default=0.5)
parser.add_argument("--stop_file", default="")
parser.add_argument("--start", type=float, nargs=3, default=None, help="USD x y yaw")
parser.add_argument("--truth_topic", default="")
parser.add_argument("--imu_topic", default="")
parser.add_argument("--imu_hz", type=float, default=100.0)
parser.add_argument("--imu_gyro_sigma", type=float, default=0.0017)
parser.add_argument("--imu_seed", type=int, default=20261101)
parser.add_argument("--truth_offset", type=float, nargs=2, default=[2.2345, 3.2136])
# 강제 이동(위치 잃음 시험, 2026-11-01 승인 기준 K1~K3): 모의 시각 T 뒤 차체가 still_s 동안 멈춰 있을 때 참 자세를 순간 이동(속도 0).
# --kidnap_by T dx dy dyaw: 발제 좌표 축으로 상대 이동 · --kidnap_to T x y yaw: 발제 좌표 절대 자세. 기록 <out>/kidnap.json.
parser.add_argument("--kidnap_by", type=float, nargs=4, default=None)
# 지도에 없는 물체(인지 세션 검토 A1, 2026-11-01): 배치 layout.json 의 상자(0.07 m, 라이다 아래) · 퍽(aabb) · 신호등 + 다리 기둥(지름 0.1 m, 높이 0.8 m, 고정)
# 을 정적 충돌체로 놓고 라이다 대상에 넣음. 다리 = --legs x1 y1 [x2 y2 …](발제 좌표).
parser.add_argument("--objects", default="", help="layout.json 경로(빈 값 = 벽만)")
parser.add_argument("--legs", type=float, nargs="*", default=[])
parser.add_argument("--kidnap_to", type=float, nargs=4, default=None)
parser.add_argument("--kidnap_still_s", type=float, default=1.0, help="이 시간 동안 명령 |v| < 0.005 · |w| < 0.005 이면 실행(0 = T 에 바로)")
parser.add_argument("--occluder_mode", choices=["raycaster", "analytic"], default="raycaster",
                    help="라이다 차체 가림: raycaster = 옛 방식(레이캐스터가 가림 상자 추적 — 상자가 출발 자세 자리에 굳어 허상을 남김), "
                         "analytic = 센서 frame 에서 칸마다 상자 교차로 한 번 계산(lidar_occlusion.py, 2026-11-01)")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
args.headless = True
args.enable_cameras = True
os.makedirs(args.out, exist_ok=True)
app = AppLauncher(args).app

import sys  # noqa: E402

sys.path.insert(0, MODEL)
import numpy as np  # noqa: E402
import torch  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.assets import Articulation  # noqa: E402
from isaaclab.sensors.camera import TiledCamera  # noqa: E402
from isaaclab.sensors.ray_caster import MultiMeshRayCaster  # noqa: E402
from isaaclab.sim.utils import get_current_stage  # noqa: E402
from pxr import Gf, UsdGeom, UsdLux  # noqa: E402

from waffle_4wd_isaac import robot_cfg as R  # noqa: E402
from waffle_4wd_isaac import sensors_cfg as S  # noqa: E402

import rclpy  # noqa: E402
from builtin_interfaces.msg import Time as RosTime  # noqa: E402
from geometry_msgs.msg import PoseStamped, TransformStamped, TwistStamped  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.qos import QoSProfile, qos_profile_sensor_data  # noqa: E402
from rosgraph_msgs.msg import Clock  # noqa: E402
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState, LaserScan  # noqa: E402
from tf2_msgs.msg import TFMessage  # noqa: E402

PHYS_DT = 1.0 / 120.0
CLOCK_HZ, JS_HZ, ODOM_HZ, LIDAR_HZ = 60.0, 50.0, 50.0, 10.0
LIGHT_RANGE = {"dome": (300.0, 600.0), "rect": (0.0, 20000.0)}      # 이전 녹화와 같은 식
ROBOT = "/World/Robot"
LIDARS = {"/scan_front": ("lds03_front", "laser_front"), "/scan_rear": ("lds03_rear", "laser_rear")}
IR = {"infra1": "top_camera_front_infra1_optical_frame", "infra2": "top_camera_front_infra2_optical_frame"}
N_BINS = 400
MIN_TURN_W = 0.35                       # [rad/s] 제자리 회전 최소 명령
SEP, RAD = float(R.WHEEL_SEPARATION), float(R.WHEEL_RADIUS)


def ros_time(t):
    sec = int(math.floor(t))
    return RosTime(sec=sec, nanosec=int(round((t - sec) * 1e9)) % 1000000000)


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def quat_xyzw_to_mat(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


route = json.load(open(args.route))
wps = [tuple(p) for p in route["waypoints"]]
if args.max_waypoints > 0:
    wps = wps[:args.max_waypoints]
start = wps[0]
yaw_start = math.pi                    # 이전 발제 출발 방향(−X)과 같게
if args.start is not None:
    start, yaw_start = (args.start[0], args.start[1]), args.start[2]

# ---------------- 장면 ----------------
sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=PHYS_DT, device=args.device, physics_material=R.CONTACT_MATERIAL))
g = sim_utils.GroundPlaneCfg(physics_material=R.CONTACT_MATERIAL)
g.func("/World/ground", g)
maze = sim_utils.UsdFileCfg(usd_path=args.maze)
maze.func("/World/maze", maze)
stage = get_current_stage()
level = min(max(args.light_level, 0.0), 1.0)
dome = LIGHT_RANGE["dome"][0] + level * (LIGHT_RANGE["dome"][1] - LIGHT_RANGE["dome"][0])
rect = LIGHT_RANGE["rect"][0] + level * (LIGHT_RANGE["rect"][1] - LIGHT_RANGE["rect"][0])
UsdLux.DomeLight.Define(stage, "/World/Lights/dome").CreateIntensityAttr(dome)
n_rect = 0
for ix, x in enumerate((-1.2, 1.2)):
    for iy, y in enumerate((-2.2, 0.0, 2.2)):
        lt = UsdLux.RectLight.Define(stage, f"/World/Lights/ceiling_{ix}{iy}")     # 아래(−Z)로 비춤
        lt.CreateIntensityAttr(rect)
        lt.CreateWidthAttr(1.0)
        lt.CreateHeightAttr(1.0)
        UsdGeom.Xformable(lt).AddTranslateOp().Set(Gf.Vec3d(x, y, 2.0))
        n_rect += 1
cfg = R.WAFFLE_4WD_CFG.replace(prim_path=ROBOT)
cfg.init_state.pos = (start[0], start[1], R.ROOT_HEIGHT + 0.003)
cfg.init_state.rot = (0.0, 0.0, math.sin(yaw_start / 2), math.cos(yaw_start / 2))
cfg.init_state.joint_pos = R.pose("drive")
robot = Articulation(cfg)
occ = S.spawn_lidar_occluders(ROBOT) if args.occluder_mode == "raycaster" else []
obj_rec = []
if args.objects or args.legs:
    ox_, oy_ = args.truth_offset
    col = sim_utils.CollisionPropertiesCfg()
    def put(name, cfg, x, y, z, yaw):
        cfg.func(f"/World/objects/obj_{name}", cfg, translation=(x - ox_, y - oy_, z), orientation=(0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)))   # (x, y, z, w)
        obj_rec.append({"name": name, "mission_xy": [x, y], "z": z, "yaw": yaw})
    if args.objects:
        lay = json.load(open(args.objects))
        for b in lay["boxes"]:
            sx, sy, sz = b["size"]
            put(b["name"], sim_utils.MeshCuboidCfg(size=(sx, sy, sz), collision_props=col), b["center"][0], b["center"][1], sz / 2, b.get("yaw", 0.0))
        for pk in lay["pucks"]:
            (x0, y0, z0), (x1, y1, z1) = pk["aabb"]
            put(pk["name"], sim_utils.MeshCuboidCfg(size=(x1 - x0, y1 - y0, z1 - z0), collision_props=col), (x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2, 0.0)
        sl = lay["signal_light"]
        put("signal_light", sim_utils.MeshCuboidCfg(size=(0.06, 0.06, sl["height"]), collision_props=col), sl["x"], sl["y"], sl["height"] / 2, sl.get("yaw", 0.0))
    for i in range(0, len(args.legs) - 1, 2):
        put(f"leg_{i // 2}", sim_utils.MeshCylinderCfg(radius=0.05, height=0.8, collision_props=col), args.legs[i], args.legs[i + 1], 0.4, 0.0)
    print(f"[objects] {len(obj_rec)} 개", flush=True)
targets = S.lidar_targets(["/World/maze/wall_[^/]+/collision/.*", "/World/ground"] + (["/World/objects/obj_[^/]+"] if obj_rec else []),
                          ROBOT if args.occluder_mode == "raycaster" else None)
OCC_RANGES = {}
if args.occluder_mode == "analytic":
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import lidar_occlusion  # noqa: E402
    OCC_RANGES = {nm: lidar_occlusion.occluder_ranges(S.SENSORS, nm, N_BINS) for nm, _ in LIDARS.values()}
    print("[occluder] analytic", {nm: int(np.isfinite(r).sum()) for nm, r in OCC_RANGES.items()}, flush=True)
lidars = {}
for topic, (name, frame) in LIDARS.items():
    c = S.lds03_cfg(name, ROBOT, targets)
    c.spawn.func(c.prim_path, c.spawn)
    lidars[topic] = (MultiMeshRayCaster(c), frame)
cams = {c: TiledCamera(S.d435_cfg(ROBOT, "front", c)) for c in IR}
sim.reset()
dt = sim.get_physics_dt()

sp = S.SPEC["d435"]
W, H = int(sp["width"]), int(sp["height"])
fx = (W / 2.0) / math.tan(float(sp["ir_hfov_rad"]) / 2.0)
fy, cx, cy = fx, W / 2.0, H / 2.0
p1, p2 = np.array(S.POSE["d435_front_infra1"]["pos"]), np.array(S.POSE["d435_front_infra2"]["pos"])
baseline = float(np.linalg.norm(p2 - p1))
lab_K = {}
for cname, cam in cams.items():
    try:
        k = cam.data.intrinsic_matrices
        k = k.torch if hasattr(k, "torch") else k
        lab_K[cname] = [round(float(v), 4) for v in k[0].flatten().tolist()]
    except Exception as e:  # noqa: BLE001
        lab_K[cname] = f"읽기 실패: {e}"

# ---------------- rclpy ----------------
rclpy.init()
node = rclpy.create_node("isaac_live_bridge_261101")
REL = QoSProfile(depth=10)
clock_pub = node.create_publisher(Clock, "/clock", REL)
scan_pubs = {t: node.create_publisher(LaserScan, t, REL) for t in LIDARS}
js_pub = node.create_publisher(JointState, "/joint_states", REL)
odom_pub = node.create_publisher(Odometry, "/odom", REL)
tf_pub = node.create_publisher(TFMessage, "/tf", QoSProfile(depth=100))
ir_pubs = {c: node.create_publisher(Image, f"/top_camera_front/{c}/image_rect_raw", qos_profile_sensor_data) for c in IR}
info_pubs = {c: node.create_publisher(CameraInfo, f"/top_camera_front/{c}/camera_info", REL) for c in IR}
cmd = {"v": 0.0, "w": 0.0, "t_rx": -1e9, "n": 0}


def on_cmd(m):
    cmd["v"], cmd["w"], cmd["t_rx"] = float(m.twist.linear.x), float(m.twist.angular.z), state["t"]
    cmd["n"] += 1


if args.mode == "cmd_vel":
    node.create_subscription(TwistStamped, args.cmd_topic, on_cmd, REL)
truth_pub = node.create_publisher(PoseStamped, args.truth_topic, REL) if args.truth_topic else None
imu_pub = node.create_publisher(Imu, args.imu_topic, qos_profile_sensor_data) if args.imu_topic else None
imu_rng = np.random.default_rng(args.imu_seed)
info_msgs = {}
for c, frame in IR.items():
    m = CameraInfo()
    m.width, m.height, m.distortion_model = W, H, "plumb_bob"
    m.d = [0.0] * 5
    m.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    m.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    m.p = [fx, 0.0, cx, (-fx * baseline if c == "infra2" else 0.0), 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    m.header.frame_id = frame
    info_msgs[c] = m

wl_ids, _ = robot.find_joints(R.WHEEL_JOINTS_LEFT, preserve_order=True)
wr_ids, _ = robot.find_joints(R.WHEEL_JOINTS_RIGHT, preserve_order=True)
wheels = list(wl_ids) + list(wr_ids)
hold_names = R.ARM_JOINTS + R.FINGER_JOINTS
hold, _ = robot.find_joints(hold_names, preserve_order=True)
drive_pose = R.pose("drive")
target = torch.tensor([[drive_pose[n] for n in hold_names]], dtype=torch.float32, device=sim.device)
cmd_buf = robot.actuators.target_command
names = list(robot.joint_names)

state = {"t": 0.0, "wp": 1, "phase": "turn", "done_t": None, "odom": [0.0, 0.0, 0.0], "v": 0.0, "w": 0.0,
         "next": {"clock": 0.0, "js": 0.0, "odom": 0.0, "scan": 0.0, "imu": 0.0}, "sent": {"clock": 0, "js": 0, "odom": 0, "scan": {t: 0 for t in LIDARS},
                                                                         "ir": {c: 0 for c in IR}}, "ir_last": {c: -1.0 for c in IR}}


def truth():
    d = robot.data
    p = d.root_pos_w.torch[0].tolist()
    q = d.root_quat_w.torch[0].tolist()          # xyzw
    return p, q, float(d.heading_w.torch[0])


def controller():
    if args.mode == "cmd_vel":
        rclpy.spin_once(node, timeout_sec=0.0)
        return (cmd["v"], cmd["w"]) if state["t"] - cmd["t_rx"] <= args.cmd_timeout else (0.0, 0.0)
    """이전 녹화와 같은 규칙: 방향 오차 > 4°면 제자리 회전, 아니면 직진하며 방향 보정. 마지막 경유점 뒤 정지."""
    if state["wp"] >= len(wps):
        return 0.0, 0.0
    p, _, yaw = truth()
    tx, ty = wps[state["wp"]]
    dx, dy = tx - p[0], ty - p[1]
    dist = math.hypot(dx, dy)
    if dist < 0.03:
        state["wp"] += 1
        state["phase"] = "turn"
        if state["wp"] >= len(wps):
            state["done_t"] = state["t"]
        return 0.0, 0.0
    err = wrap(math.atan2(dy, dx) - yaw)
    if state["phase"] == "turn":
        if abs(err) < math.radians(2.0):
            state["phase"] = "go"
            return 0.0, 0.0
        # 작은 회전 명령은 스키드 마찰을 못 이겨 멈춤(T1 첫 시도 0.08 rad/s 에서 정지) → 최소 회전 속도
        return 0.0, float(math.copysign(min(args.w, max(MIN_TURN_W, 2.0 * abs(err))), err))
    if abs(err) > math.radians(4.0) and dist > 0.10:
        state["phase"] = "turn"
        return 0.0, 0.0
    return min(args.v, 1.0 * dist + 0.03), float(np.clip(2.0 * err, -0.5, 0.5))


def scan_msg(sensor, name, frame, t):
    d = sensor.data
    hits = (d.ray_hits_w.torch[0] if hasattr(d.ray_hits_w, "torch") else d.ray_hits_w[0]).cpu().numpy().astype(np.float64)
    # 센서 frame = URDF laser_front/rear (base_link 정답 자세 × sensors.json 고정 자세). Lab 센서의 quat_w 는 ray_alignment='base' 라
    # 뒤집힘(roll π)이 빠져 TF 와 좌우가 반대가 됨(T1 셋째 시도에서 발견) → 쓰지 않음
    bp = robot.data.root_pos_w.torch[0].cpu().numpy().astype(np.float64)
    bq = robot.data.root_quat_w.torch[0].cpu().numpy().astype(np.float64)
    Rb = quat_xyzw_to_mat(bq)
    Rs = Rb @ quat_xyzw_to_mat(np.array(S.POSE[name]["quat_xyzw"], np.float64))
    pos = bp + Rb @ np.array(S.POSE[name]["pos"], np.float64)
    ok = np.isfinite(hits).all(axis=1)
    rel = (hits[ok] - pos) @ Rs                              # 센서 frame 으로
    rng = np.linalg.norm(rel, axis=1)
    ang = np.arctan2(rel[:, 1], rel[:, 0])
    inc = 2 * math.pi / N_BINS
    b = np.floor((ang + math.pi) / inc).astype(int) % N_BINS
    ranges = np.full(N_BINS, np.inf)
    keep = (rng >= 0.05) & (rng <= 12.0)
    np.minimum.at(ranges, b[keep], rng[keep])
    if name in OCC_RANGES:
        ranges = np.minimum(ranges, OCC_RANGES[name])
    s = LaserScan()
    s.header.stamp, s.header.frame_id = ros_time(t), frame
    s.angle_min, s.angle_increment = -math.pi, inc
    s.angle_max = -math.pi + inc * (N_BINS - 1)
    s.scan_time, s.time_increment, s.range_min, s.range_max = 1.0 / LIDAR_HZ, 0.0, 0.05, 12.0
    s.ranges = [float(x) for x in ranges]
    return s


truth_f = open(os.path.join(args.out, "truth.csv"), "w", newline="", buffering=1)
tw = csv.writer(truth_f)
tw.writerow(["sim_s", "x", "y", "z", "qw", "qx", "qy", "qz", "yaw", "odom_x", "odom_y", "odom_yaw", "cmd_v", "cmd_w", "wp", "phase"])
meta = {"model": "261029_1030/urdf_261028 (Isaac light, pose drive)", "maze": args.maze, "route": args.route, "n_waypoints": len(wps),
        "route_length_m": route["length_m"], "start": start, "start_yaw": yaw_start, "light_level": level, "dome": dome, "rect": rect,
        "n_rect": n_rect, "phys_dt": PHYS_DT, "ir_K_model": [fx, fy, cx, cy], "ir_K_isaaclab": lab_K, "ir_baseline_m": baseline,
        "wheel_separation_m": SEP, "wheel_radius_m": RAD, "odom_multiplier": 1.0, "lidar_bins": N_BINS, "lidar_occluders": len(occ), "occluder_mode": args.occluder_mode,
        "joint_names": names, "truth_body": "base_link (root)", "truth_in_ros": False,
        "live": {"mode": args.mode, "odom_hz": ODOM_HZ, "odom_rot_scale": args.odom_rot_scale, "odom_tf": bool(args.odom_tf),
                 "cmd_topic": args.cmd_topic, "cmd_timeout": args.cmd_timeout, "imu_topic": args.imu_topic, "imu_hz": args.imu_hz,
                 "imu_gyro_sigma": args.imu_gyro_sigma}}
meta["objects"] = {"layout": args.objects, "legs": args.legs, "placed": obj_rec}
json.dump(meta, open(os.path.join(args.out, "meta.json"), "w"), indent=2, ensure_ascii=False)
print(f"[slam] start {start} yaw {yaw_start:.3f}, waypoints {len(wps)}, IR K {fx:.3f}, baseline {baseline:.6f}, Lab K {lab_K}", flush=True)

wall0, next_report = time.time(), 30.0
steps = 0
kid = {"spec": args.kidnap_by or args.kidnap_to, "mode": "by" if args.kidnap_by else "to", "done": False, "still_since": None}


def maybe_kidnap(t):
    """강제 이동: 조건이 되면 root 자세를 쓰고 속도 0. 발제 좌표 = USD + truth_offset."""
    if kid["spec"] is None or kid["done"] or t < kid["spec"][0]:
        return
    still = abs(state["v"]) < 0.005 and abs(state["w"]) < 0.005
    kid["still_since"] = (kid["still_since"] if kid["still_since"] is not None else t) if still else None
    if args.kidnap_still_s > 0 and (kid["still_since"] is None or t - kid["still_since"] < args.kidnap_still_s):
        return
    p, q, yaw = truth()
    ox, oy = args.truth_offset
    if kid["mode"] == "by":
        _, dx, dy, dyaw = kid["spec"]
        nx, ny, nyaw = p[0] + dx, p[1] + dy, wrap(yaw + dyaw)
    else:
        _, mx, my, myaw = kid["spec"]
        nx, ny, nyaw = mx - ox, my - oy, wrap(myaw)
    pose = torch.tensor([[nx, ny, p[2], 0.0, 0.0, math.sin(nyaw / 2), math.cos(nyaw / 2)]], dtype=torch.float32, device=sim.device)
    robot.write_root_pose_to_sim_index(root_pose=pose)
    robot.write_root_velocity_to_sim_index(root_velocity=torch.zeros((1, 6), dtype=torch.float32, device=sim.device))
    kid["done"] = True
    rec = {"t": t, "mode": kid["mode"], "spec": kid["spec"], "before_mission": [p[0] + ox, p[1] + oy, yaw],
           "after_mission": [nx + ox, ny + oy, nyaw], "still_s": args.kidnap_still_s}
    json.dump(rec, open(os.path.join(args.out, "kidnap.json"), "w"), indent=2, ensure_ascii=False)
    print(f"[kidnap] {rec}", flush=True)
while app.is_running() and state["t"] < args.max_sim_s:
    t = state["t"]
    v, w = controller() if t > 2.0 else (0.0, 0.0)       # 2 s 가만히 둔 뒤 출발
    state["v"], state["w"] = v, w
    wl, wr = (v - w * SEP / 2) / RAD, (v + w * SEP / 2) / RAD
    lim = float(R.WHEEL_SPEED_MAX)
    k = max(1.0, abs(wl) / lim, abs(wr) / lim)              # 비율 유지 잘림
    cmd_buf.set_velocity_index(value=torch.tensor([[wl / k, wl / k, wr / k, wr / k]], dtype=torch.float32, device=sim.device), joint_ids=wheels)
    cmd_buf.set_position_index(value=target, joint_ids=hold)
    robot.write_data_to_sim()
    sim.step(render=(steps % 4 == 0))                      # 렌더 30 Hz
    robot.update(dt)
    steps += 1
    state["t"] = t = steps * dt
    maybe_kidnap(t)
    for s_, _ in lidars.values():
        s_.update(dt)
    for c_ in cams.values():
        c_.update(dt)
    jv = robot.data.joint_vel.torch[0]
    vl = float(jv[wl_ids].mean()) * RAD
    vr = float(jv[wr_ids].mean()) * RAD
    ov, ow = 0.5 * (vl + vr), (vr - vl) / SEP
    o = state["odom"]
    o[0] += ov * math.cos(o[2] + 0.5 * ow * dt) * dt
    o[1] += ov * math.sin(o[2] + 0.5 * ow * dt) * dt
    ow /= args.odom_rot_scale                                # 회전 배수 보정(실물은 제자리 회전 실측으로 정할 값)
    o[2] = wrap(o[2] + ow * dt)
    nx = state["next"]
    if t + 1e-9 >= nx["clock"]:
        nx["clock"] += 1.0 / CLOCK_HZ
        clock_pub.publish(Clock(clock=ros_time(t)))
        state["sent"]["clock"] += 1
    if t + 1e-9 >= nx["odom"]:
        nx["odom"] += 1.0 / ODOM_HZ
        qz, qw = math.sin(o[2] / 2), math.cos(o[2] / 2)
        od = Odometry()
        od.header.stamp = ros_time(t)
        od.header.frame_id, od.child_frame_id = "odom", "base_link"
        od.pose.pose.position.x, od.pose.pose.position.y = o[0], o[1]
        od.pose.pose.orientation.z, od.pose.pose.orientation.w = qz, qw
        od.twist.twist.linear.x, od.twist.twist.angular.z = ov, ow
        odom_pub.publish(od)
        tf = TransformStamped()
        tf.header = od.header
        tf.child_frame_id = "base_link"      # 새 URDF 루트 = base_link(base_footprint 는 그 아래), 팀 dev 규약과 같음
        tf.transform.translation.x, tf.transform.translation.y = o[0], o[1]
        tf.transform.rotation.z, tf.transform.rotation.w = qz, qw
        if args.odom_tf:
            tf_pub.publish(TFMessage(transforms=[tf]))
        state["sent"]["odom"] += 1
    if imu_pub is not None and t + 1e-9 >= nx["imu"]:
        nx["imu"] += 1.0 / args.imu_hz
        im = Imu()
        im.header.stamp, im.header.frame_id = ros_time(t), "imu_link"
        im.orientation_covariance[0] = -1.0
        im.linear_acceleration_covariance[0] = -1.0
        im.angular_velocity.z = float(robot.data.root_ang_vel_b.torch[0, 2]) + float(imu_rng.normal(0.0, args.imu_gyro_sigma))
        sg = args.imu_gyro_sigma ** 2
        im.angular_velocity_covariance = [sg, 0.0, 0.0, 0.0, sg, 0.0, 0.0, 0.0, sg]
        imu_pub.publish(im)
        state["sent"]["imu"] = state["sent"].get("imu", 0) + 1
    if t + 1e-9 >= nx["js"]:
        nx["js"] += 1.0 / JS_HZ
        js = JointState()
        js.header.stamp = ros_time(t)
        js.name = names
        js.position = [float(x) for x in robot.data.joint_pos.torch[0].tolist()]
        js.velocity = [float(x) for x in jv.tolist()]
        js_pub.publish(js)
        state["sent"]["js"] += 1
        p, q, yaw = truth()
        tw.writerow([f"{t:.6f}"] + [repr(float(x)) for x in p] + [repr(float(q[3])), repr(float(q[0])), repr(float(q[1])), repr(float(q[2]))]
                    + [repr(yaw), repr(o[0]), repr(o[1]), repr(o[2]), repr(v), repr(w), state["wp"], state["phase"]])
        if truth_pub is not None:
            ps = PoseStamped()
            ps.header.stamp, ps.header.frame_id = ros_time(t), "map"
            ps.pose.position.x, ps.pose.position.y = p[0] + args.truth_offset[0], p[1] + args.truth_offset[1]
            ps.pose.orientation.z, ps.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            truth_pub.publish(ps)
    if t + 1e-9 >= nx["scan"]:
        nx["scan"] += 1.0 / LIDAR_HZ
        for topic, (s_, frame) in lidars.items():
            scan_pubs[topic].publish(scan_msg(s_, LIDARS[topic][0], frame, t))
            state["sent"]["scan"][topic] += 1
    if steps % 4 == 0:
        for c, cam in cams.items():
            ts = cam.data.output["rgb"]
            a = (ts.torch[0] if hasattr(ts, "torch") else ts[0]).cpu().numpy()[..., :3].astype(np.float32)
            y = np.clip(0.299 * a[..., 0] + 0.587 * a[..., 1] + 0.114 * a[..., 2] + 0.5, 0, 255).astype(np.uint8)
            img = Image()
            img.header.stamp, img.header.frame_id = ros_time(t), IR[c]
            img.height, img.width, img.encoding, img.is_bigendian, img.step = H, W, "mono8", 0, W
            img.data = y.tobytes()
            ir_pubs[c].publish(img)
            info = info_msgs[c]
            info.header.stamp = img.header.stamp
            info_pubs[c].publish(info)
            state["sent"]["ir"][c] += 1
    if state["done_t"] is not None and t > state["done_t"] + 3.0:
        break
    # 막힘 감지: 30 s 동안 정답 위치가 2 cm 넘게 안 움직이면 기록하고 멈춤(본 녹화 v3 경로에서 1200 s 허비)
    if args.mode == "cmd_vel" and steps % 120 == 0 and args.stop_file and os.path.exists(args.stop_file):
        break
    if args.mode == "route" and t > 2.0 and steps % 120 == 0:
        p_now = robot.data.root_pos_w.torch[0, :2].cpu().numpy()
        hist = state.setdefault("hist", [])
        hist.append((t, p_now))
        while hist and hist[0][0] < t - 30.0:
            hist.pop(0)
        if t > 35.0 and state["done_t"] is None and max(np.linalg.norm(h[1] - p_now) for h in hist) < 0.02:
            state["stuck"] = {"t": t, "wp": state["wp"], "pos": p_now.tolist()}
            print(f"[slam] stuck {state['stuck']}", flush=True)
            break
    if t >= next_report:
        print(f"[slam] sim {t:.0f} s, wall {time.time() - wall0:.0f} s, wp {state['wp']}/{len(wps)}, sent {state['sent']}", flush=True)
        next_report += 30.0

counts = {"sim_s": state["t"], "wall_s": time.time() - wall0, "done_t": state["done_t"], "wp_reached": state["wp"], "n_wp": len(wps), "stuck": state.get("stuck"),
          "sent": state["sent"], "cmd_received": cmd["n"]}
json.dump(counts, open(os.path.join(args.out, "scene_counts.json"), "w"), indent=2)
print(f"[slam] done {json.dumps(counts)}", flush=True)
truth_f.close()
node.destroy_node()
rclpy.shutdown()
app.close()
