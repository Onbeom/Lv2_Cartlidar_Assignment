"""노드 확인 (모의 재생): 렌더 영상을 /arm_camera 로 내고 /perception/place_box/measure 를 불러 결과를 정답과 비교. 도구 확인.
먼저 띄울 것 (같은 ROS_DOMAIN_ID): robot_state_publisher (B URDF), static TF odom → base_link (단위), perception 노드."""
import json, math, sys, time, numpy as np, cv2, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import Image, CameraInfo, JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from robot_interfaces.srv import SetPerceptionMode
d, n_scenes = sys.argv[1], int(sys.argv[2])
meta = json.load(open(f"{d}/render_place_box.json")); K = meta["K"]
rclpy.init(); nd = Node("place_box_check")
img_pub = nd.create_publisher(Image, "/arm_camera/image_raw", QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE))
info_pub = nd.create_publisher(CameraInfo, "/arm_camera/camera_info", QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE))
js_pub = nd.create_publisher(JointState, "/joint_states", 10)
status = []
nd.create_subscription(String, "/perception/place_box/status", lambda m: status.append(json.loads(m.data)),
                       QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
mode_cli = nd.create_client(SetPerceptionMode, "/perception/set_mode"); meas = nd.create_client(Trigger, "/perception/place_box/measure")
def spin(t):
    end = time.time() + t
    while time.time() < end: rclpy.spin_once(nd, timeout_sec=0.01)
def call(cli, req):
    assert cli.wait_for_service(10.0); f = cli.call_async(req)
    while not f.done(): rclpy.spin_once(nd, timeout_sec=0.01)
    return f.result()
def publish(bgr):
    st = nd.get_clock().now().to_msg()
    js = JointState(); js.header.stamp = st; js.name = list(meta["q"]) + ["gripper_roll", "gripper_left_joint", "gripper_right_joint"]
    js.position = [float(v) for v in meta["q"].values()] + [0.0, 0.0, 0.0]; js_pub.publish(js)
    ci = CameraInfo(); ci.header.stamp = st; ci.header.frame_id = "arm_camera_optical_frame"; ci.width, ci.height = meta["width"], meta["height"]
    ci.k = [K["fx"], 0.0, K["cx"], 0.0, K["fy"], K["cy"], 0.0, 0.0, 1.0]; ci.d = [0.0] * 5; ci.distortion_model = "plumb_bob"
    ci.p = [K["fx"], 0.0, K["cx"], 0.0, 0.0, K["fy"], K["cy"], 0.0, 0.0, 0.0, 1.0, 0.0]; info_pub.publish(ci)
    m = Image(); m.header = ci.header; m.height, m.width = bgr.shape[:2]; m.encoding = "bgr8"; m.step = m.width * 3; m.data = bgr.tobytes(); img_pub.publish(m)
for _ in range(20): publish(np.zeros((meta["height"], meta["width"], 3), np.uint8)); spin(0.05)
r = SetPerceptionMode.Request(); r.mode = SetPerceptionMode.Request.READ_TAG; print("모드", call(mode_cli, r).message, flush=True)
rows = []
for sc in meta["scenes"][:n_scenes]:
    bgr = cv2.imread(f"{d}/{sc['image']}"); status.clear()
    for _ in range(20): publish(bgr); spin(0.05)   # 관절 · 영상이 먼저 돌게 (앞 장면 영상이 다 빠지게)
    res = call(meas, Trigger.Request())
    t0 = time.time()
    while not status and time.time() - t0 < 30: publish(bgr); spin(0.1)
    s = status[-1] if status else {"ok": False, "why": "시간 초과"}
    if s.get("ok"):
        dc = 1000 * math.hypot(s["x"] - sc["center"][0], s["y"] - sc["center"][1]); dyaw = math.degrees((s["yaw"] - sc["yaw"] + math.pi / 4) % (math.pi / 2) - math.pi / 4)
        rows.append((dc, abs(dyaw))); print(f"{sc['image']}: 중심 {dc:.2f} mm 요 {dyaw:+.2f}° 장 {s['frames']} 흩어짐 {1000*s['spread_m']:.2f} mm 걸린 {time.time()-t0:.1f} s", flush=True)
    else:
        rows.append((np.inf, np.inf)); print(sc["image"], "실패", s, flush=True)
a = np.array(rows); print(f"통과 {int(((a[:,0] <= 5) & (a[:,1] <= 1)).sum())}/{len(a)}  중심 최대 {a[:,0].max():.2f} mm  요 최대 {a[:,1].max():.2f}°")
