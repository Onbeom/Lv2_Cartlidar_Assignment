"""#81 노드 확인 (모의 재생): 렌더 영상을 /arm_camera 로 내고 READ_TAG 의 /perception/pucks 에서 목표 퍽 면 요를 참값과 비교. 도구.
먼저 띄울 것 (같은 ROS_DOMAIN_ID): robot_state_publisher (B URDF), static TF odom → base_link, perception 노드."""
import json, math, sys, time, numpy as np, cv2, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, CameraInfo, JointState
from robot_interfaces.msg import PuckArray, Puck
from robot_interfaces.srv import SetPerceptionMode
sys.path.insert(0, sys.argv[3])   # perception_survey 소스 (face_yaw.py)
from perception_survey.face_yaw import face_yaw, face_yaw_error, fold_quarter
d, out = sys.argv[1], sys.argv[2]
meta = json.load(open(f"{d}/render_face_yaw.json")); K = meta["K"]
rclpy.init(); nd = Node("face_yaw_check")
rel = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)
img_pub = nd.create_publisher(Image, "/arm_camera/image_raw", rel); info_pub = nd.create_publisher(CameraInfo, "/arm_camera/camera_info", rel)
js_pub = nd.create_publisher(JointState, "/joint_states", 10)
got = []
nd.create_subscription(PuckArray, "/perception/pucks", lambda m: got.append(m), QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE))
mode_cli = nd.create_client(SetPerceptionMode, "/perception/set_mode")
def spin(t):
    end = time.time() + t
    while time.time() < end: rclpy.spin_once(nd, timeout_sec=0.01)
def publish(bgr, q):
    st = nd.get_clock().now().to_msg()
    js = JointState(); js.header.stamp = st; js.name = list(q) + ["gripper_roll", "gripper_left_joint", "gripper_right_joint"]
    js.position = [float(v) for v in q.values()] + [0.0, 0.0, 0.0]; js_pub.publish(js)
    ci = CameraInfo(); ci.header.stamp = st; ci.header.frame_id = "arm_camera_optical_frame"; ci.width, ci.height = meta["width"], meta["height"]
    ci.k = [K["fx"], 0.0, K["cx"], 0.0, K["fy"], K["cy"], 0.0, 0.0, 1.0]; ci.d = [0.0] * 5; ci.distortion_model = "plumb_bob"
    ci.p = [K["fx"], 0.0, K["cx"], 0.0, 0.0, K["fy"], K["cy"], 0.0, 0.0, 0.0, 1.0, 0.0]; info_pub.publish(ci)
    m = Image(); m.header = ci.header; m.height, m.width = bgr.shape[:2]; m.encoding = "bgr8"; m.step = m.width * 3; m.data = bgr.tobytes(); img_pub.publish(m)
q0 = meta["scenes"][0]["q"]
for _ in range(20): publish(np.zeros((meta["height"], meta["width"], 3), np.uint8), q0); spin(0.05)
assert mode_cli.wait_for_service(10.0)
r = SetPerceptionMode.Request(); r.mode = SetPerceptionMode.Request.READ_TAG; f = mode_cli.call_async(r)
while not f.done(): rclpy.spin_once(nd, timeout_sec=0.01)
rows = []
for sc in meta["scenes"]:
    bgr = cv2.imread(f"{d}/{sc['image']}"); tg = sc["target"]
    for _ in range(10): publish(bgr, sc["q"]); spin(0.05)   # 관절 · TF 가 먼저 돌고 앞 장면 결과가 빠지게
    got.clear(); vals = []; t0 = time.time()
    while len(vals) < 5 and time.time() - t0 < 10:
        publish(bgr, sc["q"]); spin(0.1)
        for m in got:
            for p in m.pucks:
                if p.camera == Puck.CAMERA_ARM and p.tag_id == tg["tag_id"]:
                    o = p.pose.orientation
                    vals.append({"shape": p.shape, "standing": p.standing, "valid": p.orientation_valid,
                                 "yaw": face_yaw(o.x, o.y, o.z, o.w) if p.orientation_valid else None,
                                 "xy": [p.pose.position.x, p.pose.position.y]})
        got.clear()
    truth = fold_quarter(tg["yaw"])
    ys = [v["yaw"] for v in vals if v["yaw"] is not None and v["valid"] and v["standing"] and v["shape"] == Puck.SHAPE_CUBOID]
    err = math.degrees(face_yaw_error(float(np.median(ys)), truth)) if ys else None
    row = {"image": sc["image"], "n": len(vals), "n_ok": len(ys), "truth_deg": round(math.degrees(truth), 3),
           "est_deg": round(math.degrees(float(np.median(ys))), 3) if ys else None, "err_deg": round(err, 3) if err is not None else None,
           "rel_j1_deg": round(math.degrees(face_yaw_error(tg["yaw"], sc["j1"])), 3), "shapes": sorted({v["shape"] for v in vals})}
    rows.append(row); print(row, flush=True)
json.dump(rows, open(out, "w"), ensure_ascii=False, indent=1)
e = [abs(r["err_deg"]) for r in rows if r["err_deg"] is not None]
print(f"값 있음 {len(e)}/{len(rows)}  ≤2° {sum(x <= 2 for x in e)}/{len(rows)}  중앙 {np.median(e) if e else float('nan'):.3f}°  최대 {max(e) if e else float('nan'):.3f}°")
