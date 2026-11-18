요청하신 측정 방법입니다. 사각지대 · odom 회전 과대 · 회전 중심을 재고 결과는 표로 남겨 주세요. 값이 나오면 보정 계층과 `robot_geometry.yaml` `base.icr_x`(지금 −0.0658)를 정할 수 있습니다.

**조건**
- 경기와 같은 바닥에서 재 주세요(바닥이 다르면 미끄러짐이 달라집니다). 팔은 주행과 같은 STOW.
- 로봇: `base_bringup`만 실행(Nav2 · drive_node · mission 끔). 스크립트는 `/cmd_vel_teleop`으로 보내므로 조이스틱 데드맨을 누르지 마세요. 끝나거나 Ctrl+C면 정지 명령을 냅니다.
- 주변 1.5 × 1.5 m를 비워 주세요. 비상 정지를 손 닿는 곳에 둡니다.
- 도구 확인은 가짜 `/odom` · `/imu/data` 노드로만 했습니다. 실물에서는 처음 돌립니다.

**1. 제자리 회전 곡선(사각지대)**
```bash
python3 skid_probe.py sweep
```
명령 ±0.1~1.5 rad/s, 약 2분. 시작 전 10초는 움직이지 않고 자이로 치우침을 잽니다.

**2. 전진 + 회전 곡선**
```bash
python3 skid_probe.py arc
```
v 0.04 · 0.08 · 0.12, w 0.1~0.8 왕복, 약 4분.

**3. IMU 각도 검증(스크립트 값이 믿을 만한지)**
바닥에 차체 방향 표시를 하고 세 바퀴 돕니다. 멈춘 뒤 표시와 각도 차를 재 주세요.
```bash
python3 skid_probe.py turn 1080 --w 0.8
```

**4. 회전 중심**
180° 돌면 차체 위 점은 회전 중심을 사이에 두고 반대편으로 갑니다. 그래서 전후 위치의 가운데가 곧 회전 중심입니다.
1. 차체 앞 · 뒤 중심선 위에 점 F, R을 정하고 base_link 기준 x(`x_F`, `x_R`)를 잽니다.
2. F, R 바로 아래 바닥을 표시합니다(F0, R0).
3. 돌립니다.
```bash
python3 skid_probe.py turn 180 --w 0.8
```
4. 멈춘 뒤 F, R 아래를 표시합니다(F1, R1).
5. 가운데 점 M_F = (F0 + F1)/2, M_R = (R0 + R1)/2. 두 점이 5 mm 안에서 같아야 하고, IMU 값이 180 ± 3° 안이어야 합니다. 아니면 다시 잽니다.
6. 회전 중심 `x_c = x_F − (F0→M_F 거리, F0→R0 방향 성분)`, 옆으로 벗어난 양은 수직 성분입니다.

`--w 0.8`과 `--w 0.4`, 시계 · 반시계(`turn -180`)로 3번씩 재 주세요.

**결과 표(예상은 모의 값, 실물 바닥에서는 다를 수 있음)**
| 항목 | 값의 의미 · 단위 | 예상 | 결과 |
|---|---|---|---|
| 사각지대 | IMU 회전이 나오기 시작하는 명령(rad/s) | 0.1까지 회전 0 | |
| 실제 / 명령 | 0.35 · 0.8 명령의 w IMU(rad/s) | 약 0.19 · 0.58~0.60 | |
| 바퀴 odom 과대(제자리) | odom/IMU | 약 1.15 | |
| 바퀴 odom 과대(전진 회전) | odom/IMU 최대 | 최대 약 1.40 | |
| IMU 각도 차 | 1080° 뒤 바닥 표시와 차(°) | 수 ° 이내 | |
| 회전 중심 x | base_link 기준(m), 평균 · 범위 | −0.0658(현재 설정), 모의 측정 −0.0786 | |
| 회전 중심 옆 치우침 | m | 0 근처 | |

CSV(`skid_sweep_*.csv`, `skid_arc_*.csv`)를 함께 올려 주시면 곡선 분석은 제가 하겠습니다.

<details><summary>skid_probe.py</summary>

```python
#!/usr/bin/env python3
"""4륜 스키드 회전 측정. /cmd_vel_teleop 로 명령, /imu/data = 실제 회전, /odom = 바퀴 회전.
  skid_probe.py sweep            제자리 회전 곡선 (약 2 분)
  skid_probe.py arc              전진 + 회전 곡선, 왕복 (약 4 분, 빈 바닥 1.5 x 1.5 m)
  skid_probe.py turn 180 --w 0.8 IMU 각도로 돌고 멈춤 (음수면 시계 방향)
base_bringup 만 띄운 상태에서 실행 (Nav2 · drive_node 끔). 끝나거나 Ctrl+C 면 정지 명령을 낸다."""
import argparse, csv, math, time
import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

class P:
    def __init__(s):
        s.n = rclpy.create_node('skid_probe'); s.pub = s.n.create_publisher(TwistStamped, '/cmd_vel_teleop', 10); s.wi = s.wo = s.vo = None; s.t_last = 0.
        s.n.create_subscription(Imu, '/imu/data', s.imu, qos_profile_sensor_data); s.n.create_subscription(Odometry, '/odom', s.odom, 10)
    def imu(s, m): s.wi = m.angular_velocity.z; s.got_i = time.monotonic()
    def odom(s, m): s.wo = m.twist.twist.angular.z; s.vo = m.twist.twist.linear.x; s.got_o = time.monotonic()
    def cmd(s, v, w):
        m = TwistStamped(); m.header.stamp = s.n.get_clock().now().to_msg(); m.header.frame_id = 'base_link'; m.twist.linear.x = float(v); m.twist.angular.z = float(w); s.pub.publish(m)
    def run(s, v, w, sec, each=None):
        """sec 초 동안 20 Hz 로 명령. each(dt, 경과) 가 True 면 일찍 끝"""
        t0 = last = time.monotonic(); nxt = t0
        while rclpy.ok() and (now := time.monotonic()) - t0 < sec:
            rclpy.spin_once(s.n, timeout_sec=.002)
            if now >= nxt: s.cmd(v, w); nxt += .05
            if each and now - last >= .01:
                stop = each(now - last, now - t0); last = now
                if stop: return
    def wait(s):
        t0 = time.monotonic()
        while rclpy.ok() and (s.wi is None or s.wo is None):
            rclpy.spin_once(s.n, timeout_sec=.05)
            if time.monotonic() - t0 > 5: raise SystemExit('/imu/data 또는 /odom 이 안 들어옴. 움직이지 않고 끝냄')
    def mean_step(s, v, w, run_, settle):
        A = []
        s.run(v, w, run_, lambda d, t: (A.append((s.wi, s.wo, s.vo)) if t >= settle else None, False)[1])
        n = max(1, len(A)); return [sum(a[i] for a in A) / n for i in range(3)]
    def bias(s, sec=10.):
        print(f'정지 {sec:.0f} 초 (자이로 치우침 재기)', flush=True); B = []
        s.run(0, 0, sec, lambda d, t: (B.append(s.wi), False)[1]); b = sum(B) / len(B); print(f'치우침 {b:+.5f} rad/s', flush=True); return b

def table(p, plan, a, name):
    b = p.bias(); rows = []
    print(f"{'v 명령':>7} {'w 명령':>7} {'w IMU':>8} {'w odom':>8} {'IMU/명령':>8} {'odom/IMU':>9} {'v odom':>7}", flush=True)
    for v, w in plan:
        wi, wo, vo = p.mean_step(v, w, a.run, a.settle); wi -= b; p.run(0, 0, a.stop); rows.append((v, w, wi, wo, vo))
        print(f'{v:7.2f} {w:7.2f} {wi:8.3f} {wo:8.3f} {wi / w:8.2f} ' + (f'{wo / wi:9.2f}' if abs(wi) > .02 else '        -') + f' {vo:7.3f}', flush=True)
    out = f"skid_{name}_{time.strftime('%H%M%S')}.csv"
    with open(out, 'w', newline='') as f: c = csv.writer(f); c.writerow(['v_cmd', 'w_cmd', 'w_imu', 'w_odom', 'v_odom']); c.writerows(rows)
    print('저장', out, flush=True)

def main():
    ap = argparse.ArgumentParser(); ap.add_argument('mode', choices=['sweep', 'arc', 'turn']); ap.add_argument('deg', nargs='?', type=float, default=180.); ap.add_argument('--w', type=float, default=.8); ap.add_argument('--run', type=float); ap.add_argument('--settle', type=float, default=1.5); ap.add_argument('--stop', type=float, default=2.); a = ap.parse_args()
    rclpy.init(); p = P()
    try:
        p.wait()
        if a.mode == 'sweep':
            a.run = a.run or 5.; table(p, [(0, s * w) for w in (.1, .2, .3, .4, .6, .8, 1., 1.5) for s in (1, -1)], a, 'sweep')
        elif a.mode == 'arc':
            a.run = a.run or 3.; a.settle = min(a.settle, 1.); a.stop = min(a.stop, 1.5)
            table(p, [x for v in (.04, .08, .12) for w in (.1, .2, .35, .8) for s in (1, -1) for x in ((v, s * w), (-v, -s * w))], a, 'arc')
        else:
            b = p.bias(5.); goal = math.radians(abs(a.deg)); sgn = math.copysign(1, a.deg); w = sgn * abs(a.w); st = dict(i=0., o=0.)
            def add(d, t): st['i'] += (p.wi - b) * d; st['o'] += p.wo * d; return False
            def go(d, t): add(d, t); return abs(st['i']) >= goal - abs(a.w) * .04
            print(f'{a.deg:.0f}° 회전 시작 ({w:+.2f} rad/s)', flush=True); p.run(0, w, goal / abs(a.w) * 4 + 5, go); p.run(0, 0, 2., add)
            print(f"끝: IMU {math.degrees(st['i']):+.1f}° · 바퀴 odom {math.degrees(st['o']):+.1f}° · odom/IMU {st['o'] / st['i']:.3f}", flush=True)
    finally:
        for _ in range(10): p.cmd(0, 0); time.sleep(.02)
        p.n.destroy_node(); rclpy.shutdown()
if __name__ == '__main__': main()
```
</details>
