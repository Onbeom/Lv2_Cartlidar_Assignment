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
