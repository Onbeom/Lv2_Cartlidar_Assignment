"""도구 확인(판정 아님): 녹화의 /tracked_pose(Cartographer) 연속 두 표본 사이 이동과 같은 구간 EKF(odom→base_link TF) 이동의 차이 = '점프'.
loc_monitor jump_xy · jump_yaw_deg 문턱 근거. 사용: python3 jump_stats.py <결과 폴더...>(시작 15 s 뒤부터)"""
import math, sys, bisect, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
yaw = lambda q: math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
def rel(a, b):
    c, s = math.cos(a[2]), math.sin(a[2]); dx, dy = b[0]-a[0], b[1]-a[1]
    return (c*dx+s*dy, -s*dx+c*dy, math.remainder(b[2]-a[2], 2*math.pi))
for run in sys.argv[1:]:
    r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=run+'/out_bag'), rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=['/tracked_pose', '/tf']))
    TP, OB = [], []
    while r.has_next():
        tp, data, _ = r.read_next(); m = deserialize_message(data, get_message(types[tp]))
        if tp == '/tracked_pose':
            TP.append((m.header.stamp.sec+m.header.stamp.nanosec*1e-9, m.pose.position.x, m.pose.position.y, yaw(m.pose.orientation)))
        else:
            for t in m.transforms:
                if (t.header.frame_id, t.child_frame_id) == ('odom', 'base_link'):
                    OB.append((t.header.stamp.sec+t.header.stamp.nanosec*1e-9, t.transform.translation.x, t.transform.translation.y, yaw(t.transform.rotation)))
    OB.sort(); ot = [o[0] for o in OB]
    def odom_at(t):
        i = bisect.bisect_left(ot, t); return OB[min(i, len(OB)-1)] if i < len(OB) and abs(OB[min(i, len(OB)-1)][0]-t) < 0.03 else None
    jx, jy = [], []
    for a, b in zip(TP, TP[1:]):
        if a[0] < 15: continue
        oa, ob = odom_at(a[0]), odom_at(b[0])
        if not oa or not ob: continue
        rt, ro = rel(a[1:], b[1:]), rel(oa[1:], ob[1:])
        jx.append(math.hypot(rt[0]-ro[0], rt[1]-ro[1])); jy.append(abs(math.degrees(rt[2]-ro[2])))
    q = lambda v, p: sorted(v)[int(p*(len(v)-1))]
    print(run.split('/')[-1], 'n', len(jx), 'xy p50 %.4f p99 %.4f 최대 %.4f m' % (q(jx,.5), q(jx,.99), max(jx)), '| yaw p50 %.2f p99 %.2f 최대 %.2f°' % (q(jy,.5), q(jy,.99), max(jy)))
