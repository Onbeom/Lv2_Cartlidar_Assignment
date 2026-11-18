#!/usr/bin/env python3
"""robot_teleop.py 도구 확인 (PC, 모의). 가짜 바퀴 보드 + 팀 fake_opencr(팔) + 가짜 터미널. 실물 보드 아님."""
import json, os, pty, signal, struct, subprocess, sys, tempfile, threading, time
HERE = os.path.dirname(os.path.abspath(__file__)); J = os.path.join(HERE, '..', 'jetson'); sys.path.insert(0, J)
from teleop_common import tool
FAKE = os.path.join(J, '..', '..', '..', 'Lv2_Cartlidar_Assignment_armcam', 'workspace/tools/arm_bench/monitor/fake_opencr.py')
FAKE = os.path.expanduser('~/git/Lv2_Cartlidar_Assignment_armcam/workspace/tools/arm_bench/monitor/fake_opencr.py')
tmp = tempfile.mkdtemp(); res = []
def rec(n, ok, v=''): res.append((n, bool(ok), v)); print('통과' if ok else '실패', n, v, flush=True)

class FakeWheel:
    def __init__(s):
        s.m, sl = pty.openpty(); import tty; tty.setraw(sl); s.path = os.ttyname(sl); s.sl = sl
        s.log, s.err, s.run = [], 0, True; s.parser = tool.Parser(); s.last = 0.0
        threading.Thread(target=s.loop, daemon=True).start()
    def loop(s):
        import select
        seq = 0
        while s.run:
            r, _, _ = select.select([s.m], [], [], 0.01)
            if r:
                for b in os.read(s.m, 4096):
                    f = s.parser.push(b)
                    if f and f[0] == tool.TYPE_CMD:
                        v = struct.unpack(tool.CMD_FMT, f[1]); s.log.append((time.monotonic(), v[0], v[1:7], v[14], v[15])); s.last = time.monotonic()
            e = s.err | (tool.ERR_TIMEOUT if time.monotonic() - s.last > 0.2 else 0)
            pl = struct.pack(tool.STATE_FMT, 1, *([0.0] * 12), *([0.0] * 12), *([0.0] * 12), 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 12.0, e, 5, seq & 255); seq += 1
            os.write(s.m, tool.frame(tool.TYPE_STATE, pl))
fw = FakeWheel()
link = os.path.join(tmp, 'arm'); farm = subprocess.Popen([sys.executable, FAKE, '--link', link, '--start', '대기'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL); time.sleep(1.5)
LOG = os.path.join(tmp, 'arm.jsonl')
def start():
    m, sl = pty.openpty()
    p = subprocess.Popen([sys.executable, os.path.join(J, 'robot_teleop.py'), '--wheel_port', fw.path, '--log', LOG,
        '--arm_cmd', f'{sys.executable} -u {os.path.join(J, "arm_agent.py")} --port {link}'], stdin=sl, stdout=sl, stderr=sl, start_new_session=True)
    os.close(sl)
    def drain():
        try:
            while os.read(m, 4096): pass
        except OSError: pass
    threading.Thread(target=drain, daemon=True).start()
    return p, m
def status():
    last = None
    for line in open(LOG, encoding='utf-8'):
        d = json.loads(line)
        if 'state' in d: last = d
    return last
def wait_state(st, t=6):
    e = time.monotonic() + t
    while time.monotonic() < e:
        if os.path.exists(LOG):
            d = status()
            if d and d['state'] == st: return True
        time.sleep(0.05)
    return False
p, m = start(); key = lambda k: os.write(m, k.encode())
e = time.monotonic() + 15
while not os.path.exists(LOG) and time.monotonic() < e: time.sleep(0.1)
time.sleep(1.5); n0 = len(fw.log)
key('w'); time.sleep(0.4); rec('W1 r 전: 바퀴 보드에 명령 안 보냄, 팔도 IDLE', len(fw.log) == n0 and status()['state'] == 'IDLE', f'{len(fw.log)-n0}개')
key('y'); time.sleep(0.3); rec('A0 r 전: 팔 키 무시', status()['state'] == 'IDLE')
time.sleep(0.7); key('r'); ok = wait_state('RUN'); time.sleep(0.2)
w = fw.log[-1]; rec('W2 r: 바퀴 TORQUE_ON · 역할 1 · 속도 0, 팔 RUN', ok and w[1] == 1 and w[3] & tool.FLAG_TORQUE_ON and not any(w[2]), f'flags={w[3]}')
k = 0.05 / 0.033 / 1.5
key('w'); time.sleep(0.35); w = fw.log[-1]; rec('W3 w 단계1: 왼 · 오 모터 +1.010 rad/s', all(abs(x - k) < 0.02 for x in w[2][:4]) and w[2][4] == 0, f'{w[2][0]:.3f}')
time.sleep(0.6); w = fw.log[-1]; rec('W4 키 뗌: 약 0.6 s 뒤 0', not any(w[2]), f'{w[2][0]:.3f}')
ka = 0.25 * 0.317 / 2 / 0.033 / 1.5
key('a'); time.sleep(0.35); w = fw.log[-1]; rec('W5 a: 왼 − 오 + 0.800', abs(w[2][0] + ka) < 0.02 and abs(w[2][1] + ka) < 0.02 and abs(w[2][2] - ka) < 0.02 and abs(w[2][3] - ka) < 0.02, f'{w[2][0]:.3f} {w[2][2]:.3f}')
key(' '); time.sleep(0.08); w = fw.log[-1]; rec('W6 space: 바로 0', not any(w[2]))
key('s'); time.sleep(0.35); w = fw.log[-1]; rec('W7 s: 후진 −1.010', abs(w[2][0] + k) < 0.02, f'{w[2][0]:.3f}')
key(' '); key('d'); time.sleep(0.2); w = fw.log[-1]; rec('W8 d: 왼 + 오 −', w[2][0] > 0 and w[2][2] < 0, f'{w[2][0]:.3f} {w[2][2]:.3f}')
key(' '); time.sleep(0.1); kl, kr = (0.05 - 0.25 * 0.317 / 2) / 0.033 / 1.5, (0.05 + 0.25 * 0.317 / 2) / 0.033 / 1.5
key('z'); time.sleep(0.35); w = fw.log[-1]; rec('W8a z 왼쪽 원: 왼 +0.210 오 +1.811', all(abs(x - kl) < 0.02 for x in w[2][:2]) and all(abs(x - kr) < 0.02 for x in w[2][2:4]), f'{w[2][0]:.3f} {w[2][2]:.3f}')
key(' '); time.sleep(0.1); key('c'); time.sleep(0.35); w = fw.log[-1]; rec('W8b c 오른쪽 원: 왼 +1.811 오 +0.210', all(abs(x - kr) < 0.02 for x in w[2][:2]) and all(abs(x - kl) < 0.02 for x in w[2][2:4]), f'{w[2][0]:.3f} {w[2][2]:.3f}')
time.sleep(0.6); w = fw.log[-1]; rec('W8c 원 키 뗌: 약 0.6 s 뒤 0', not any(w[2]), f'{w[2][0]:.3f}')
key(' '); key('3'); key('w'); time.sleep(0.5); w = fw.log[-1]; k3 = 0.2 / 0.033 / 1.5; rec('W9 단계3 : +4.04, 가속 제한 뒤', abs(w[2][0] - k3) < 0.1, f'{w[2][0]:.3f}'); key(' '); key('1')
# 팔 키 전부
step = 0.05; pairs = [('y', 'h', 'J1'), ('u', 'j', 'J2'), ('i', 'k', 'J3'), ('o', 'l', 'J4'), ('p', ';', 'J5'), ('[', "'", '그리퍼')]
for i, (a, b, name) in enumerate(pairs):
    if i == 5:  # 대기 자세 그리퍼는 닫힘 한계 근처라 먼저 조금 연다
        for _ in range(4): key(b); time.sleep(0.25)
        time.sleep(0.5)
    q0 = status()['q'][:]; key(a); time.sleep(1.0); q1 = status()['q'][:]; key(b); time.sleep(1.0); q2 = status()['q'][:]
    others = max(abs(q1[j] - q0[j]) for j in range(6) if j != i)
    rec(f'A1 {name} {a}/{b}: + 쪽 한 칸, − 쪽 돌아옴, 다른 관절 그대로', abs(q1[i] - q0[i] - step) < 0.012 and abs(q2[i] - q0[i]) < 0.012 and others < 0.01, f'{q1[i]-q0[i]:+.3f} {q2[i]-q1[i]:+.3f} 다른 {others:.3f}')
# 눌러 두기(30 Hz 반복 1.5 s) 뒤 넘어감
q0 = status()['q'][0]
for _ in range(45): key('y'); time.sleep(1/30)
qr = status()['cmd'][0]; time.sleep(1.2); q2 = status()['q'][0]
rec('A2 y 1.5 s 누름: 속도 0.2 rad/s 안, 뗀 뒤 한 칸 안에서 멈춤', 0.2 < qr - q0 < 0.2 * 1.6 + step + 0.03 and abs(q2 - qr) <= step + 0.02, f'누르는 동안 {qr-q0:+.3f}, 뗀 뒤 {q2-qr:+.3f}')
for _ in range(45): key('h'); time.sleep(1/30)
time.sleep(1.0)
key('m'); q0 = status()['q'][0]; key('y'); time.sleep(1.2); q1 = status()['q'][0]; rec('A3 m: 한 번 양 0.10', abs(q1 - q0 - 0.10) < 0.012, f'{q1-q0:+.3f}')
key('h'); time.sleep(1.0); key('n'); key('n'); q0 = status()['q'][0]; key('y'); time.sleep(0.8); q1 = status()['q'][0]; rec('A3 n n: 한 번 양 0.02', abs(q1 - q0 - 0.02) < 0.012, f'{q1-q0:+.3f}'); key('m')
key('y'); time.sleep(0.05); key(' '); time.sleep(0.5); d = status(); rec('A4 space: 팔 목표 = 명령, RUN 유지', d['state'] == 'RUN')
for _ in range(8): key("["); time.sleep(0.3)
time.sleep(0.5); d = status(); rec('A5 그리퍼 닫힘 한계까지: 멈춤 없이 한계 앞에서 서고 RUN 유지', d['state'] == 'RUN' and d['q'][5] <= 0.9541, f"{d['q'][5]:.4f} {d['state']}")
key('w'); time.sleep(0.2); key('e'); ok = wait_state('STOPPED', 3); time.sleep(0.05); w = fw.log[-1]; rec('E1 e: 팔 STOPPED, 바퀴 ESTOP 비트', ok and w[3] & tool.FLAG_ESTOP and not any(w[2]), f'flags={w[3]}')
time.sleep(0.5); n0 = len(fw.log); key('w'); key('y'); time.sleep(0.4); rec('E2 e 뒤: 바퀴 송신 없음, r 없이 안 움직임', len(fw.log) == n0, f'{len(fw.log)-n0}')
key('r'); wait_state('RUN'); time.sleep(0.2); fw.err = tool.ERR_MOTOR if hasattr(tool, 'ERR_MOTOR') else 0x08; time.sleep(0.45); w = fw.log[-1]; rec('E3 바퀴 보드 오류 비트: ESTOP 비트 뒤 멈춤', w[3] & tool.FLAG_ESTOP or time.monotonic() - fw.last > 0.05, f'flags={w[3]}'); fw.err = 0
key('r'); time.sleep(0.5); os.kill(p.pid, signal.SIGHUP); t = time.monotonic()
while p.poll() is None and time.monotonic() - t < 6: time.sleep(0.05)
time.sleep(0.2); d = status(); w = fw.log[-1]
rec('Q1 SIGHUP(터미널 끊김): 끝, 바퀴 토크 끔, 팔 END', p.poll() == 0 and w[3] == 0 and d['state'] == 'END', f"rc={p.poll()} flags={w[3]} 팔={d['state']}")
# 에이전트 직접: hb 끊김 · 입력 끊김
ag = subprocess.Popen([sys.executable, '-u', os.path.join(J, 'arm_agent.py'), '--port', link], stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1)
out = []; threading.Thread(target=lambda: [out.append(json.loads(l)) for l in ag.stdout], daemon=True).start()
time.sleep(1.5); ag.stdin.write('start\n'); ag.stdin.flush()
e = time.monotonic() + 5
while time.monotonic() < e and not any(d.get('state') == 'RUN' for d in out):
    ag.stdin.write('hb\n'); ag.stdin.flush(); time.sleep(0.1)
t = time.monotonic()
while time.monotonic() - t < 3.5 and not (out and out[-1].get('state') == 'STOPPED'): time.sleep(0.05)
rec('H1 hb 끊김: 약 1.5 s 뒤 STOPPED', out[-1].get('state') == 'STOPPED' and 1.2 < time.monotonic() - t < 2.5, f'{time.monotonic()-t:.2f} s')
ag.stdin.write('start\n'); ag.stdin.flush(); time.sleep(1.0); ag.stdin.close(); t = time.monotonic()
while ag.poll() is None and time.monotonic() - t < 5: time.sleep(0.05)
rec('H2 입력 끊김(ssh 끊김): 정지 후 끝', ag.poll() == 0, f'{time.monotonic()-t:.2f} s')
farm.terminate(); fw.run = False
print('통과', sum(r[1] for r in res), '/', len(res)); sys.exit(0 if all(r[1] for r in res) else 1)
