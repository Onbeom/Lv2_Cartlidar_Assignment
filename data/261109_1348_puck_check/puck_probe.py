#!/usr/bin/env python3
"""태그 퍽 확인 (#143 세 번째 항목). 인지 모드만 켜고 결과를 읽는다. 팔 · 바퀴 명령은 보내지 않는다."""
import argparse, json, math, os, statistics as st, sys, time
import rclpy, yaml
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data as SENSOR
from sensor_msgs.msg import CameraInfo, JointState
from robot_interfaces.msg import Puck, PuckArray
from robot_interfaces.srv import SetPerceptionMode as Mode

CX_ROTATED, CY_ROTATED = 584.12, 359.36   # 실측 cx 694.88 · cy 359.64 를 180° 돌린 값
CX_TOL = 1.0      # [px] yaml 소수 자리 차이만 허용
RATE_MIN = 80.0   # [%] 인지 결과 중 팔 카메라 태그 퍽이 있는 비율
STAB_MAX = 5.0    # [mm] 수평 위치 흔들림 p95


def info_topic(path):
    def walk(v):
        if isinstance(v, dict):
            t = v.get('info_topic')
            if isinstance(t, str) and 'arm' in t:
                return t
            for x in v.values():
                r = walk(x)
                if r:
                    return r
        return None
    with open(path, encoding='utf-8') as f:
        return walk(yaml.safe_load(f))


def p95(a):
    a = sorted(a)
    return a[min(len(a) - 1, int(math.ceil(.95 * len(a))) - 1)] if a else None


ap = argparse.ArgumentParser()
ap.add_argument('--seconds', type=float, default=15.0)
ap.add_argument('--service_wait', type=float, default=90.0)
ap.add_argument('--tag', type=int, default=-1, help='놓은 퍽의 태그 번호 (모르면 -1)')
ap.add_argument('--config', default=os.path.expanduser(
    '~/Lv2_Cartlidar_Assignment/workspace/ros2_ws/install/perception/share/perception/config/perception.yaml'))
ap.add_argument('--info_topic', default='')
ap.add_argument('--out', default='/tmp/puck_probe.json')
a = ap.parse_args()

topic = a.info_topic or info_topic(a.config)
rclpy.init()
n = Node('puck_probe')
s = dict(q={}, info=None, joints=0, msgs=0, hit=0, frames=set(), pts=[], tags={}, meta=None, other=0)


def on_pucks(m):
    s['msgs'] += 1
    s['frames'].add(m.header.frame_id)
    arm = [p for p in m.pucks if p.camera == Puck.CAMERA_ARM and p.tag_id >= 0]
    s['other'] += len(m.pucks) - len(arm)
    if not arm:
        return
    s['hit'] += 1
    p = arm[0]
    s['tags'][p.tag_id] = s['tags'].get(p.tag_id, 0) + 1
    s['pts'].append((p.pose.position.x, p.pose.position.y, p.pose.position.z))
    s['meta'] = dict(tag_id=p.tag_id, color=p.color, shape=p.shape, confidence=round(float(p.confidence), 3))


n.create_subscription(PuckArray, '/perception/pucks', on_pucks, 10)


def on_js(m):
    s['joints'] += 1
    s['q'] = dict(zip(m.name, [float(x) for x in m.position]))


n.create_subscription(JointState, '/joint_states', on_js, SENSOR)
n.create_subscription(CameraInfo, topic, lambda m: s.__setitem__('info', m), SENSOR)
cli = n.create_client(Mode, '/perception/set_mode')
if not cli.wait_for_service(timeout_sec=a.service_wait):
    print('실패: /perception/set_mode 가 뜨지 않음')
    sys.exit(2)


def set_mode(v):
    r = Mode.Request()
    r.mode = v
    f = cli.call_async(r)
    rclpy.spin_until_future_complete(n, f, timeout_sec=5.0)
    return f.result()


print('모드 켜기:', set_mode(4))   # READ_TAG (팔 카메라만)
t0 = last = time.monotonic()
while time.monotonic() - t0 < a.seconds:
    rclpy.spin_once(n, timeout_sec=.05)
    if time.monotonic() - last >= 1.0:
        last = time.monotonic()
        print(f"  {last - t0:4.0f} s  결과 {s['msgs']}  팔 태그 퍽 {s['hit']}  관절 {s['joints']}", flush=True)
print('모드 끄기:', set_mode(0))

i = s['info']
cx, cy = (float(i.k[2]), float(i.k[5])) if i else (None, None)
pts = s['pts']
med = [st.median(c) for c in zip(*pts)] if pts else None
dev = [1000 * math.hypot(p[0] - med[0], p[1] - med[1]) for p in pts] if pts else []
rate = 100.0 * s['hit'] / s['msgs'] if s['msgs'] else 0.0
stab = p95(dev)
tag_ok = len(s['tags']) == 1 and (a.tag < 0 or a.tag in s['tags'])
checks = {
    'camera_info 가 돌린 값': bool(i) and abs(cx - CX_ROTATED) <= CX_TOL and abs(cy - CY_ROTATED) <= CX_TOL,
    '관절값 받음': s['joints'] > 0,
    f'팔 태그 퍽 비율 {RATE_MIN:.0f} % 이상': rate >= RATE_MIN,
    'frame_id = base_link': s['frames'] == {'base_link'},
    '태그 번호 하나': tag_ok,
    f'수평 흔들림 p95 {STAB_MAX:.0f} mm 이하': stab is not None and stab <= STAB_MAX,
    '위치 유한': bool(pts) and all(map(math.isfinite, med)),
}
checks = {k: bool(v) for k, v in checks.items()}
out = dict(joints_last=s['q'], info_topic=topic, cx=cx, cy=cy, joints=s['joints'], msgs=s['msgs'], hit=s['hit'], rate_pct=round(rate, 1),
           frames=sorted(s['frames']), tags=s['tags'], meta=s['meta'], other_pucks=s['other'],
           median_xyz_m=[round(v, 4) for v in med] if med else None, stab_p95_mm=round(stab, 2) if stab is not None else None,
           checks=checks, passed=all(checks.values()))
with open(a.out, 'w', encoding='utf-8') as f:
    json.dump(out, f, ensure_ascii=False, indent=1)
print(json.dumps(out, ensure_ascii=False, indent=1))
sys.exit(0 if out['passed'] else 1)
