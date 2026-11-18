#!/usr/bin/env python3
"""arm_sweep.py 도구 확인 (PC, 모의). 팀 fake_opencr(가짜 팔 보드) + 가짜 터미널 + 가짜 카메라. 실물 보드 · 실물 카메라 아님."""
import json, os, pty, select, signal, subprocess, sys, tempfile, threading, time
HERE = os.path.dirname(os.path.abspath(__file__)); J = os.path.join(HERE, '..', 'jetson'); sys.path.insert(0, J)
import arm_sweep
FAKE = os.path.expanduser('~/git/Lv2_Cartlidar_Assignment_armcam/workspace/tools/arm_bench/monitor/fake_opencr.py')
tmp = tempfile.mkdtemp(); res = []
def rec(n, ok, v=''): res.append((n, bool(ok), v)); print('통과' if ok else '실패', n, v, flush=True)

# ---- 궤적 ----
p = arm_sweep.Path(arm_sweep.sweep_points(0.5, 0.2, 3), 0.5, 1.5)
ts = [i * 0.002 for i in range(int(p.duration / 0.002) + 1)]; xs = [p.pos(t) for t in ts]
vs = [abs(b - a) / 0.002 for a, b in zip(xs, xs[1:])]
rec('P1 궤적: 시작 · 끝이 기본 자세', abs(xs[0] - 0.5) < 1e-9 and abs(p.pos(p.duration) - 0.5) < 1e-9)
rec('P2 궤적: 범위 ±0.2', abs(max(xs) - 0.7) < 1e-6 and abs(min(xs) - 0.3) < 1e-6, f'{min(xs):.4f} {max(xs):.4f}')
rec('P3 궤적: 최고 속도 0.5', abs(max(vs) - 0.5) < 0.01, f'{max(vs):.4f}')
rec('P4 궤적: 이어짐(한 칸 변화 ≤ 속도 상한 분)', max(vs) <= 0.5 + 1e-6)
short = arm_sweep.Path([0.0, 0.01], 0.5, 1.5); rec('P5 짧은 구간: 삼각형으로 끝점 도달', abs(short.pos(short.duration) - 0.01) < 1e-9)

# ---- 녹화 (가짜 카메라) ----
class FakeCap:
    def __init__(s): s.i = 0; s.released = False
    def read(s):
        time.sleep(0.01); s.i += 1
        import numpy as np
        return True, np.frombuffer(b'\xff\xd8abc' if s.i % 5 else b'bad', dtype='uint8')
    def release(s): s.released = True
c = FakeCap(); r = arm_sweep.Recorder(c, os.path.join(tmp, 'rec'), lambda: (0.1, 0.2))
time.sleep(0.1); d = r.begin('run1', {'speed': 0.1}); time.sleep(0.2); out = r.end({'result': '완료'}); n1 = len([f for f in os.listdir(d) if f.endswith('.jpg')])
time.sleep(0.1); n2 = len([f for f in os.listdir(d) if f.endswith('.jpg')]); rj = json.load(open(os.path.join(d, 'result.json'), encoding='utf-8'))
rows = open(os.path.join(d, 'frames.csv')).read().splitlines()
rec('R1 begin ~ end 사이 장만 저장, 깨진 장 제외', out[1] == n1 == n2 > 5 and rj['bad_frames'] > 0 and rj['frames'] == n1, f'{n1} 장 bad {rj["bad_frames"]}')
rec('R2 frames.csv: 장 수 + 머리줄, 각도 기록', len(rows) == n1 + 1 and rows[1].endswith(',0.1000,0.2000'), rows[1])
r.close(); rec('R3 close: 카메라 놓음', c.released)

# ---- 프로그램 전체 ----
link = os.path.join(tmp, 'arm'); farm = subprocess.Popen([sys.executable, FAKE, '--link', link, '--start', '대기'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
e = time.monotonic() + 8
while not os.path.exists(link) and time.monotonic() < e: time.sleep(0.05)
time.sleep(0.5)
LOG = os.path.join(tmp, 'arm.jsonl')
def start(*extra, log=LOG):
    m, sl = pty.openpty()
    p = subprocess.Popen([sys.executable, os.path.join(J, 'arm_sweep.py'), '--port', link, '--cam', 'none', '--log', log, *extra], stdin=sl, stdout=sl, stderr=sl, start_new_session=True)
    os.close(sl); buf = []
    def drain():
        try:
            while True:
                d = os.read(m, 4096)
                if not d: break
                buf.append(d.decode(errors='ignore'))
        except OSError: pass
    threading.Thread(target=drain, daemon=True).start()
    return p, m, buf
def rows(log=LOG):
    out = []
    for l in open(log, encoding='utf-8'):
        try: out.append(json.loads(l))
        except ValueError: pass
    return out
def status(): return [d for d in rows() if 'state' in d][-1]
def notes(): return [d['note'] for d in rows() if 'note' in d]
def wait(fn, t=10):
    e = time.monotonic() + t
    while time.monotonic() < e:
        if fn(): return True
        time.sleep(0.03)
    return False
def wait_state(s, t=6): return wait(lambda: os.path.exists(LOG) and any('state' in d for d in rows()) and status()['state'] == s, t)

p, m, buf = start('--speeds', '0.1,0.25,0.5', '--amp', '0.1', '--cycles', '1'); key = lambda k: os.write(m, k.encode())
wait_state('IDLE', 10); time.sleep(0.7)
key('2'); time.sleep(0.4); rec('A0 토크 전: 숫자 키 무시', status()['state'] == 'IDLE' and any('토크를 먼저' in n for n in notes()))
q0 = status()['q'][:]
key('t'); ok = wait_state('RUN'); time.sleep(0.3); d = status(); rec('A1 t: RUN, 기본 자세 = 켤 때 자세', ok and abs(d['base'] - q0[0]) < 0.03, f"base {d['base']} q0 {q0[0]}")
base = d['base']; lim = base
i0 = len(rows()); key('2'); time.sleep(0.25); key('t'); time.sleep(0.1); rec('A2 움직이는 중 t 무시', any('토크를 바꾸지' in n for n in notes()) and status()['state'] == 'RUN')
key('1'); time.sleep(0.05); rec('A3 움직이는 중 숫자 키 거부', any('움직이는 중입니다' in n for n in notes()))
wait(lambda: any('0.25 rad/s 끝' in n for n in notes()), 15)
seg = [d for d in rows()[i0:] if 'q' in d and d['q']]
qs = [d['q'][0] for d in seg]; cs = [d['cmd'][0] for d in seg]
others = max(abs(d['q'][k] - q0[k]) for d in seg for k in range(1, 5))
tv = [(a['t'], a['cmd'][0]) for a in seg if a['moving']]
v = max(abs(b[1] - a[1]) / (b[0] - a[0]) for a, b in zip(tv[::5], tv[5::5])) if len(tv) > 10 else 0
rec('A4 왕복: 명령 범위 기본 ±0.1', abs(max(cs) - base - 0.1) < 0.012 and abs(min(cs) - base + 0.1) < 0.012, f'{min(cs)-base:+.3f} {max(cs)-base:+.3f}')
rec('A5 왕복: 속도 상한 안(0.25 × 1.15)', 0.1 < v <= 0.25 * 1.15 + 0.03, f'{v:.3f}')
rec('A6 왕복: 다른 관절 그대로', others < 0.03, f'{others:.4f}')
time.sleep(0.2); d = status(); rec('A7 끝나면 기본 자세', abs(d['q'][0] - base) <= 0.02 and not d['moving'] and d['state'] == 'RUN', f"{d['q'][0] - base:+.4f}")
key('3'); time.sleep(0.45); key(' '); time.sleep(0.05); c1 = status()['q'][0]; time.sleep(0.5); c2 = status()['q'][0]; d = status()
rec('A8 space: 그 자리에 멈춤, RUN 유지, 기본 자세 아님', abs(c1 - c2) < 0.03 and d['state'] == 'RUN' and not d['moving'] and abs(c2 - base) > 0.03, f'{c2-base:+.3f}')
key('1'); time.sleep(0.2); rec('A9 기본 자세 밖에서 왕복 거부', any('b 로 먼저' in n for n in notes()) and not status()['moving'])
key('b'); wait(lambda: not status()['moving'] and abs(status()['q'][0] - base) <= 0.02, 8); time.sleep(0.2); d = status(); rec('A10 b: 기본 자세로', abs(d['q'][0] - base) <= 0.02, f"{d['q'][0]-base:+.4f}")
key('e'); ok = wait_state('STOPPED', 3); time.sleep(0.2); key('2'); time.sleep(0.2); rec('E1 e: STOPPED, 숫자 키 안 움직임', ok and status()['state'] == 'STOPPED' and not status()['moving'])
time.sleep(0.7); key('r'); ok = wait_state('RUN'); rec('E2 r: 다시 RUN', ok)
key('t'); ok = wait_state('IDLE', 4); rec('T1 t: 토크 끔 → IDLE', ok)
time.sleep(0.8); key('t'); ok = wait_state('RUN'); rec('T2 t: 다시 켬', ok)
time.sleep(0.3); os.kill(p.pid, signal.SIGHUP); e = time.monotonic() + 6
while p.poll() is None and time.monotonic() < e: time.sleep(0.05)
rec('Q1 SIGHUP(터미널 끊김): 끝, 복구 rc 0', p.poll() == 0, f'rc={p.poll()}')
# 한계 넘는 왕복 거부
time.sleep(1.5); L2 = os.path.join(tmp, 'b.jsonl')
sys.path.insert(0, os.path.dirname(FAKE))
from teleop_common import tool
lo, hi = tool.load_limits(); stow = [d for d in rows() if d.get('q')][-1]['q']
near = [k for k in range(5) if stow[k] - 0.35 < lo[k] + 0.01 or stow[k] + 0.35 > hi[k] - 0.01]
print('한계 근처 관절', [k + 1 for k in near], '자세', stow[:5], 'lo', lo[:5], 'hi', hi[:5])
assert near, '대기 자세에 한계 근처 관절이 없어 거부 시험 불가'
p2, m2, _ = start('--amp', '0.35', '--cycles', '1', '--joint', str(near[0] + 1), log=L2); key2 = lambda k: os.write(m2, k.encode())
wait(lambda: os.path.exists(L2) and any('state' in d for d in rows(L2)), 10); time.sleep(0.7); key2('t'); wait(lambda: any(d.get('state') == 'RUN' for d in rows(L2)), 6); time.sleep(0.3)
b2 = [d for d in rows(L2) if 'state' in d][-1]; key2('1'); time.sleep(0.5)
print('참고 기본', b2['base'])
q2 = [d for d in rows(L2) if 'state' in d][-1]
limited = any('관절 한계' in d.get('note', '') for d in rows(L2))
rec('L1 왕복 끝이 관절 한계를 넘으면 거부, 움직이지 않음', limited and not q2['moving'], f'거부={limited} 움직임={q2["moving"]}')
key2('q'); e = time.monotonic() + 6
while p2.poll() is None and time.monotonic() < e: time.sleep(0.05)
rec('Q2 q: 끝 rc 0', p2.poll() == 0, f'rc={p2.poll()}')
# 인자 거부
for args, name in ((['--speeds', '0.9'], '속도 상한'), (['--amp', '0.5'], '진폭 상한')):
    p3, m3, b3 = start(*args, log=os.path.join(tmp, 'c.jsonl')); e = time.monotonic() + 5
    while p3.poll() is None and time.monotonic() < e: time.sleep(0.05)
    rec(f'V {name} 넘으면 실행 거부', p3.poll() == 2, f'rc={p3.poll()}')
farm.terminate()
bad = [x for x in res if not x[1]]
print(f'== {len(res) - len(bad)}/{len(res)} 통과'); sys.exit(1 if bad else 0)
