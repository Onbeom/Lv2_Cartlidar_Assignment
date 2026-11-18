"""회전 자리 도착 오차(판정 아님): Shim 제자리 회전이 시작될 때 회전 중심(참값 base − 0.0786)과 계획 경로의 가장 가까운 제자리 회전 자리 사이 거리 · 들어온 방향 기준 앞뒤/옆 성분."""
import sys, math, csv, bisect, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
run = sys.argv[1]
T = [r for r in csv.DictReader(open(f'{run}/truth.csv'))]
tt = [float(r['t']) for r in T]
def truth(t):
    r = T[min(bisect.bisect_left(tt, t), len(T)-1)]
    x, y, th = float(r['x']), float(r['y']), float(r['yaw'])
    return x - 0.0786*math.cos(th), y - 0.0786*math.sin(th), th
rd = rosbag2_py.SequentialReader(); rd.open(rosbag2_py.StorageOptions(uri=f'{run}/out_bag'), rosbag2_py.ConverterOptions('', ''))
types = {t.name: t.type for t in rd.get_all_topics_and_types()}
rd.set_filter(rosbag2_py.StorageFilter(topics=['/clock', '/plan', '/cmd_vel_ctrl']))
clk = 0; plan = None; prev = None; out = []
yaw = lambda q: math.atan2(2*q.w*q.z, 1-2*q.z*q.z)
while rd.has_next():
    tp, data, _ = rd.read_next()
    m = deserialize_message(data, get_message(types[tp]))
    if tp == '/clock': clk = m.clock.sec + m.clock.nanosec*1e-9; continue
    if tp == '/plan':
        P = [(p.pose.position.x, p.pose.position.y, yaw(p.pose.orientation)) for p in m.poses]
        rot = []
        for i in range(1, len(P)-1):
            if math.hypot(P[i+1][0]-P[i][0], P[i+1][1]-P[i][1]) < 1e-3 and abs(math.remainder(P[i+1][2]-P[i][2], 2*math.pi)) > 0.01:
                j = i
                while j > 0 and math.hypot(P[j][0]-P[i][0], P[j][1]-P[i][1]) < 1e-3: j -= 1
                rot.append((P[i][0], P[i][1], math.atan2(P[i][1]-P[j][1], P[i][0]-P[j][0])))
        plan = rot; continue
    tw = m.twist
    if prev is not None and abs(prev.linear.x) > 1e-3 and abs(tw.linear.x) < 1e-6 and abs(tw.angular.z) > 0.005 and plan:
        x, y, th = truth(clk)
        best = min(plan, key=lambda r: math.hypot(r[0]-x, r[1]-y))
        dx, dy = best[0]-x, best[1]-y; d = math.hypot(dx, dy)
        if d < 0.08:
            a = dx*math.cos(best[2]) + dy*math.sin(best[2]); l = -dx*math.sin(best[2]) + dy*math.cos(best[2])
            out.append((round(clk, 1), round(d, 4), round(a, 4), round(l, 4), best[:2]))
    prev = tw
for o in out: print(o)
ds = [o[1] for o in out]
if ds: print('n', len(ds), '중앙', sorted(ds)[len(ds)//2], '최대', max(ds), '≥0.025', sum(d >= 0.025 for d in ds))
