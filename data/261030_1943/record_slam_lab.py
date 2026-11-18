"""SLAM 녹화 장면(모의): 새 미로 maze.zip + 새 모델 waffle_4wd 261028(Isaac Lab 3.0 가벼운 판, 시작 자세 drive).

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
parser = argparse.ArgumentParser()
parser.add_argument("--out", required=True)
parser.add_argument("--light_level", type=float, required=True, help="0 = 가장 어두움 ~ 1 = 가장 밝음 (이전 녹화와 같은 식)")
parser.add_argument("--max_waypoints", type=int, default=0, help="0 = 전체, 아니면 앞에서부터 이 개수만(도구 확인용)")
parser.add_argument("--v", type=float, default=0.20)
parser.add_argument("--w", type=float, default=0.60)
parser.add_argument("--max_sim_s", type=float, default=1500.0)
parser.add_argument("--maze", default=os.path.join(HERE, "maze_src/maze/maze.usda"))
parser.add_argument("--route", default=os.path.join(HERE, "route_maze.json"))
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
from geometry_msgs.msg import TransformStamped  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.qos import QoSProfile, qos_profile_sensor_data  # noqa: E402
from rosgraph_msgs.msg import Clock  # noqa: E402
from sensor_msgs.msg import CameraInfo, Image, JointState, LaserScan  # noqa: E402
from tf2_msgs.msg import TFMessage  # noqa: E402

PHYS_DT = 1.0 / 120.0
CLOCK_HZ, JS_HZ, ODOM_HZ, LIDAR_HZ = 60.0, 50.0, 30.0, 10.0
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
occ = S.spawn_lidar_occluders(ROBOT)
targets = S.lidar_targets(["/World/maze/wall_[^/]+/collision/.*", "/World/ground"], ROBOT)
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
node = rclpy.create_node("isaac_slam_bridge_261028")
REL = QoSProfile(depth=10)
clock_pub = node.create_publisher(Clock, "/clock", REL)
scan_pubs = {t: node.create_publisher(LaserScan, t, REL) for t in LIDARS}
js_pub = node.create_publisher(JointState, "/joint_states", REL)
odom_pub = node.create_publisher(Odometry, "/odom", REL)
tf_pub = node.create_publisher(TFMessage, "/tf", QoSProfile(depth=100))
ir_pubs = {c: node.create_publisher(Image, f"/top_camera_front/{c}/image_rect_raw", qos_profile_sensor_data) for c in IR}
info_pubs = {c: node.create_publisher(CameraInfo, f"/top_camera_front/{c}/camera_info", REL) for c in IR}
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
         "next": {"clock": 0.0, "js": 0.0, "odom": 0.0, "scan": 0.0}, "sent": {"clock": 0, "js": 0, "odom": 0, "scan": {t: 0 for t in LIDARS},
                                                                         "ir": {c: 0 for c in IR}}, "ir_last": {c: -1.0 for c in IR}}


def truth():
    d = robot.data
    p = d.root_pos_w.torch[0].tolist()
    q = d.root_quat_w.torch[0].tolist()          # xyzw
    return p, q, float(d.heading_w.torch[0])


def controller():
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
        "wheel_separation_m": SEP, "wheel_radius_m": RAD, "odom_multiplier": 1.0, "lidar_bins": N_BINS, "lidar_occluders": len(occ),
        "joint_names": names, "truth_body": "base_link (root)", "truth_in_ros": False}
json.dump(meta, open(os.path.join(args.out, "meta.json"), "w"), indent=2, ensure_ascii=False)
print(f"[slam] start {start} yaw {yaw_start:.3f}, waypoints {len(wps)}, IR K {fx:.3f}, baseline {baseline:.6f}, Lab K {lab_K}", flush=True)

wall0, next_report = time.time(), 30.0
steps = 0
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
        tf_pub.publish(TFMessage(transforms=[tf]))
        state["sent"]["odom"] += 1
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
    if t > 2.0 and steps % 120 == 0:
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
          "sent": state["sent"]}
json.dump(counts, open(os.path.join(args.out, "scene_counts.json"), "w"), indent=2)
print(f"[slam] done {json.dumps(counts)}", flush=True)
truth_f.close()
node.destroy_node()
rclpy.shutdown()
app.close()
