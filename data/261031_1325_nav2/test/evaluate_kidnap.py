"""강제 이동(위치 잃음) 판정 채점 K1~K3(기준 사용자 승인 2026-11-01, 모의).
R1 이동 뒤 3 s 안 /localization/status 가 SUSPECT 이상(Cartographer 가 그 전에 스스로 되돌리면 통과 · '스스로 되돌림' 표시, B1)
R2 LOST 동안 최종 명령(/cmd_vel_nav) 0 — LOST 0.2 s 뒤부터 표본 100 %
R3 다시 찾은 순간(LOST/RECOVERING → OK) 추정 오차 ≤ 0.05 m · ≤ 3°
R4 다시 찾은 뒤 10 s 안 추정 오차 > 0.10 m 0 번
R5 감지(첫 SUSPECT) → 다시 찾기 ≤ 60 s(K3 는 못 찾음 + 안전 정지도 통과: --k3)
추정 자세 = mission_map→map(정적) · map→odom · odom→base_link TF, 참값 = truth.csv(base_link). 시각은 모두 /clock(모의 초).
사용: python3 evaluate_kidnap.py <결과 폴더> [--k3] → <결과 폴더>/evaluate_kidnap.json"""
import bisect
import csv
import json
import math
import sys
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

OUT = Path(sys.argv[1])
K3 = '--k3' in sys.argv
kid = json.load(open(OUT / 'scene' / 'kidnap.json')) if (OUT / 'scene' / 'kidnap.json').exists() else json.load(open(OUT / 'kidnap.json'))
tk = float(kid.get('t', kid.get('t_sim', 0.0)))
yaw = lambda q: math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def comp(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2])


r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=str(OUT / 'out_bag')), rosbag2_py.ConverterOptions('', ''))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
r.set_filter(rosbag2_py.StorageFilter(topics=['/clock', '/tf', '/tf_static', '/localization/status', '/cmd_vel_nav']))
clk, MM, MO = 0.0, None, None
status, cmds, est = [], [], []
while r.has_next():
    tp, data, _ = r.read_next()
    m = deserialize_message(data, get_message(types[tp]))
    if tp == '/clock':
        clk = m.clock.sec + m.clock.nanosec * 1e-9
    elif tp == '/localization/status':
        status.append((clk, m.data))
    elif tp == '/cmd_vel_nav':
        tw = m.twist if hasattr(m, 'twist') else m
        cmds.append((clk, tw.linear.x, tw.angular.z))
    else:
        for t in m.transforms:
            k = (t.header.frame_id, t.child_frame_id)
            v = (t.transform.translation.x, t.transform.translation.y, yaw(t.transform.rotation))
            if k == ('mission_map', 'map'):
                MM = v
            elif k == ('map', 'odom'):
                MO = v
            elif k == ('odom', 'base_link') and MM and MO:
                est.append((t.header.stamp.sec + t.header.stamp.nanosec * 1e-9,) + comp(comp(MM, MO), v))
T = [(float(x['t']), float(x['x']), float(x['y']), float(x['yaw'])) for x in csv.DictReader(open(OUT / 'truth.csv'))]
tt = [x[0] for x in T]


def err_at(e):
    q = T[min(bisect.bisect_left(tt, e[0]), len(T) - 1)]
    return math.hypot(e[1] - q[1], e[2] - q[2]), abs(math.degrees(math.remainder(e[3] - q[3], 2 * math.pi)))


after = [s for s in status if s[0] >= tk]
# 메시지 순서로 찾음(같은 /clock 에 OK → LOST 가 함께 찍힐 수 있어 시각만으로는 틀림, K1 첫 채점에서 발견)
i_sus = next((i for i, x in enumerate(status) if x[0] >= tk and x[1] != 'OK'), None)
i_lost = next((i for i, x in enumerate(status) if x[0] >= tk and x[1] in ('LOST', 'RECOVERING')), None)
i_ok = next((i for i in range(i_sus + 1, len(status)) if status[i][1] == 'OK'), None) if i_sus is not None else None
t_sus = status[i_sus][0] if i_sus is not None else None
t_lost = status[i_lost][0] if i_lost is not None else None
t_ok = status[i_ok][0] if i_ok is not None else None
# 이동 직후 3 s 안 추정 오차가 0.05 m 안으로 돌아왔는지(B1 '스스로 되돌림')
e3 = [err_at(e) for e in est if tk + 0.5 <= e[0] <= tk + 3.0]
self_fix = bool(e3) and min(x[0] for x in e3) <= 0.05 and t_sus is None
rep = {'t_kidnap': tk, 'kidnap': kid, 't_suspect': t_sus, 't_lost': t_lost, 't_ok_again': t_ok,
       'status_changes': [s for i, s in enumerate(after) if i == 0 or s[1] != after[i - 1][1]][:20]}
eu = [err_at(e) for e in est if tk <= e[0] <= (t_sus if t_sus is not None else tk + 3.0)]
rep['err_until_detect'] = {'max_m': round(max(x[0] for x in eu), 4), 'max_deg': round(max(x[1] for x in eu), 2)} if eu else None   # 인지 요청: 감지 전 위치 오차
rep['R1'] = {'pass': (t_sus is not None and t_sus - tk <= 3.0) or self_fix, 'delay_s': None if t_sus is None else round(t_sus - tk, 2), 'self_fixed': self_fix}
if t_lost is not None:
    t_end = t_ok if t_ok is not None else cmds[-1][0]
    win = [c for c in cmds if t_lost + 0.2 <= c[0] <= t_end]
    nz = [c for c in win if abs(c[1]) > 1e-6 or abs(c[2]) > 1e-6]
    rep['R2'] = {'pass': len(nz) == 0, 'n': len(win), 'n_nonzero': len(nz)}
else:
    rep['R2'] = {'pass': None, 'note': 'LOST 없음'}
if t_ok is not None:
    e0 = next((e for e in est if e[0] >= t_ok), None)
    m0 = err_at(e0) if e0 else (None, None)
    rep['R3'] = {'pass': e0 is not None and m0[0] <= 0.05 and m0[1] <= 3.0, 'err_m': None if e0 is None else round(m0[0], 4), 'err_deg': None if e0 is None else round(m0[1], 2)}
    e10 = [err_at(e)[0] for e in est if t_ok <= e[0] <= t_ok + 10.0]
    rep['R4'] = {'pass': bool(e10) and max(e10) <= 0.10, 'max_err_m': round(max(e10), 4) if e10 else None}
    rep['R5'] = {'pass': t_ok - t_sus <= 60.0, 'detect_to_ok_s': round(t_ok - t_sus, 2)}
else:
    stop = [c for c in cmds if t_lost is not None and c[0] >= t_lost + 0.2]
    safe = bool(stop) and all(abs(c[1]) < 1e-6 and abs(c[2]) < 1e-6 for c in stop)
    rep['R3'] = rep['R4'] = {'pass': None, 'note': '다시 찾지 못함'}
    rep['R5'] = {'pass': K3 and safe, 'note': 'K3: 못 찾음 + 안전 정지면 통과' if K3 else '다시 찾지 못함', 'safe_stop': safe}
keys = ['R1', 'R2', 'R3', 'R4', 'R5']
rep['pass'] = all(rep[k]['pass'] in (True, None) for k in keys) and rep['R1']['pass'] is True
json.dump(rep, open(OUT / 'evaluate_kidnap.json', 'w'), indent=1, ensure_ascii=False)
print(json.dumps({k: rep[k] for k in ['t_kidnap', 't_suspect', 't_lost', 't_ok_again', 'err_until_detect'] + keys + ['pass']}, ensure_ascii=False))
