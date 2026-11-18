#!/usr/bin/env python3
"""프로세스별 CPU 시간 스냅샷 · 두 스냅샷 차이 (Jetson, 읽기만: /proc).

cpu_snap.py snap <out.json>          지금 모든 프로세스의 utime+stime(틱) · 이름 저장
cpu_snap.py diff <a.json> <b.json>   두 스냅샷 사이 프로세스별 평균 CPU % (코어 하나 = 100) · 전체 합 · 6 코어 대비 %
이름: ros2 노드면 '__node:=' 값 또는 실행 파일 이름, 컨테이너면 실행 파일 이름 + pid (담긴 노드는 bringup.log 의 'Loaded node' 로).
"""
import json, os, sys, time

HZ = os.sysconf('SC_CLK_TCK')


def name_of(pid):
    try:
        args = open(f'/proc/{pid}/cmdline', 'rb').read().split(b'\0')
    except OSError:
        return None
    args = [a.decode(errors='replace') for a in args if a]
    if not args:
        return None
    for a in args:
        if a.startswith('__node:='):
            return a.split('=', 1)[1]
    exe = os.path.basename(args[0])
    if exe.startswith('python') and len(args) > 1:
        exe = os.path.basename(args[1])
    return exe.split()[0][:40]


def snap(path):
    out = {'t': time.time(), 'ncpu': os.cpu_count(), 'p': {}}
    for d in os.listdir('/proc'):
        if not d.isdigit():
            continue
        try:
            f = open(f'/proc/{d}/stat').read()
        except OSError:
            continue
        rest = f[f.rindex(')') + 2:].split()
        ticks = int(rest[11]) + int(rest[12])   # utime + stime
        n = name_of(d)
        if n:
            out['p'][d] = [n, ticks]
    json.dump(out, open(path, 'w'))


def diff(a, b):
    A, B = json.load(open(a)), json.load(open(b))
    dt = B['t'] - A['t']
    rows = []
    for pid, (n, t1) in B['p'].items():
        t0 = A['p'].get(pid, [n, 0])[1] if pid in A['p'] else 0   # 중간에 생긴 프로세스는 0 부터
        pct = (t1 - t0) / HZ / dt * 100
        if pct >= 0.5:
            label = f'{n}({pid})' if n.startswith('component_container') else n
            rows.append((pct, label))
    rows.sort(reverse=True)
    tot = sum(p for p, _ in rows)
    print(f'구간 {dt:.0f} s · 프로세스 합 {tot:.0f} (코어 {B["ncpu"]} 개 대비 {tot / B["ncpu"]:.1f} %, 0.5 미만 제외)')
    for p, n in rows:
        print(f'  {p:6.1f}  {n}')


if __name__ == '__main__':
    if sys.argv[1] == 'snap':
        snap(sys.argv[2])
    else:
        diff(sys.argv[2], sys.argv[3])
