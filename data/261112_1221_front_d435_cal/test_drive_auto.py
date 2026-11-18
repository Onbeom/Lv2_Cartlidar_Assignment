import os, sys, subprocess, time, signal
J='/home/pa4/git/lv2_next_module/261106_1655_real_drive/jetson'; sys.path.insert(0, J)
sys.argv=['x']; exec(open('/home/pa4/git/lv2_next_module/261106_1655_real_drive/teleop_check/test_robot_teleop.py').read().split('fw = FakeWheel()')[0])
for args, scale, kill in ((['test'], '1', None), (['run', 'front'], '0.2', None), (['run', 'rear'], '0.2', None), (['run', 'front'], '1', 4.0)):
    fw = FakeWheel()
    env = dict(os.environ, REAL_DRIVE='/home/pa4/git/lv2_next_module/261106_1655_real_drive/jetson', WHEEL_PORT=fw.path, DRIVE_TIME_SCALE=scale)
    p = subprocess.Popen([sys.executable, '/home/pa4/git/lv2_next_module/261112_1221_front_d435_cal/jetson/drive_auto.py'] + args, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    if kill: time.sleep(kill); p.send_signal(signal.SIGINT)
    out = p.communicate(timeout=60)[0]; time.sleep(0.3)
    torque = [l for l in fw.log]
    moving = [l for l in torque if abs(l[2][0]) > 1e-6]
    vals = sorted(set(round(l[2][0], 3) for l in moving))
    last = torque[-1] if torque else None
    if not torque: print("명령 없음, 출력:", out[-600:]); fw.run=False; continue
    print(args, 'scale', scale, 'kill', kill, '| 명령 수', len(torque), '움직임 바퀴값', vals, '| 마지막 flags', last[3], '바퀴', [round(v, 3) for v in last[2][:4]], '| 출력 끝:', out.strip().splitlines()[-2:])
    fw.run = False
