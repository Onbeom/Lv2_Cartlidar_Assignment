#!/usr/bin/env python3
"""replay_track 결과를 판정 값으로 묶는다. analyze.py <run 폴더> <replay 출력> [...쌍 반복]

전체 검색이 찾은 태그를 정답으로 본다. 전체 검색도 못 찾은 장은 분모에서 뺀다.
등속 구간 = 명령 속도가 그 회차 속도의 90 % 이상인 장. 기준 2 번은 등속 구간 값으로 본다.
최대 간격 = 전체 검색이 계속 태그를 보는 동안, 추적 결과가 나온 장 사이의 촬영 시각 간격.
"""
import csv
import json
import os
import statistics as st
import sys


def tags(text):
    p = text.split()
    return {int(p[i]): (float(p[i + 1]), float(p[i + 2]), float(p[i + 3])) for i in range(0, len(p), 4)}


def read_replay(path):
    out = []
    for line in open(path, encoding='utf-8'):
        head, tr, base = line.rstrip('\n').split('|')
        i, t, ms = head.split()
        out.append(dict(i=int(i), t=float(t), ms=float(ms), track=tags(tr), base=tags(base)))
    return out


def cruise_flags(run):
    rows = list(csv.DictReader(open(os.path.join(run, 'frames.csv'), encoding='utf-8')))
    v = json.load(open(os.path.join(run, 'run.json'), encoding='utf-8'))['speed']
    t = [float(r['t']) for r in rows]
    c = [float(r['cmd']) for r in rows]
    flags = {}
    for k in range(1, len(rows) - 1):
        flags[k] = abs(c[k + 1] - c[k - 1]) / max(1e-6, t[k + 1] - t[k - 1]) >= 0.9 * v
    return flags, v


def pct(xs, p):
    return sorted(xs)[min(len(xs) - 1, int(p * len(xs)))] if xs else float('nan')


def analyze(run, replay):
    r = read_replay(replay)
    flags, v = cruise_flags(run)
    seen = lost = seen_c = lost_c = wrong = frames_c = 0
    moved, gaps, longs = [], [], []
    last_ok = None
    prev = None
    for f in r:
        cruise = flags.get(f['i'], False)
        frames_c += cruise
        wrong += len(set(f['track']) - set(f['base']))
        for k in f['base']:
            seen += 1
            seen_c += cruise
            longs.append(f['base'][k][2])
            if k not in f['track']:
                lost += 1
                lost_c += cruise
        if not f['base']:
            last_ok = None
        elif set(f['base']) <= set(f['track']):
            if last_ok is not None:
                gaps.append((f['t'] - last_ok) * 1000)
            last_ok = f['t']
        if prev and cruise:
            for k in set(prev['base']) & set(f['base']):
                a, b = prev['base'][k], f['base'][k]
                moved.append(((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5)
        prev = f
    ms = [f['ms'] for f in r]
    dt = [(b['t'] - a['t']) * 1000 for a, b in zip(r, r[1:])]
    return dict(run=os.path.basename(run.rstrip('/')), speed=v, processed=len(r), cruise_frames=frames_c,
                long_px=round(st.median(longs), 1) if longs else None, seen=seen, lost=lost,
                lost_pct=round(100 * lost / seen, 2) if seen else None, seen_cruise=seen_c, lost_cruise=lost_c,
                lost_cruise_pct=round(100 * lost_c / seen_c, 2) if seen_c else None, wrong_id=wrong,
                max_gap_ms=round(max(gaps), 1) if gaps else None,
                moved_px_p50=round(pct(moved, 0.5), 1), moved_px_p95=round(pct(moved, 0.95), 1),
                moved_px_max=round(max(moved), 1) if moved else None, step_ms_p50=round(pct(dt, 0.5), 1),
                step_ms_max=round(max(dt), 1) if dt else None, proc_ms_mean=round(st.mean(ms), 1), proc_ms_p99=round(pct(ms, 0.99), 1))


if __name__ == '__main__':
    a = sys.argv[1:]
    for run, rep in zip(a[::2], a[1::2]):
        print(json.dumps(analyze(run, rep), ensure_ascii=False))
