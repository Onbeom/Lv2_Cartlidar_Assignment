"""truth.csv(장면 50 Hz: t, 참값 위치 · 자세, odom 자세(회전 보정 1.0 = 엔코더 그대로), 명령 v · w)로 구간마다
  - 제자리 회전: odom 회전/참 회전(회전 배수), 회전 중심 x_icr = −(차체 옆 속도)/w 중앙값, base_link 가 그린 원 반지름
  - 직진: odom 거리/참 거리(직진 배수), 옆 밀림
구간 = 명령이 같은 칸에서 시작 1.5 s 뒤 ~ 끝 0.3 s 전. 사용: analyze.py <truth.csv> <out.json>"""
import csv, json, math, sys
import numpy as np
rows = list(csv.reader(open(sys.argv[1])))
head = rows[0] if not rows[0][0].replace('.', '').isdigit() else None
data = np.array([[float(x) for x in r[:14]] for r in rows[1 if head else 0:]])
t, px, py, yaw, oyaw, ox, oy, v, w = data[:, 0], data[:, 1], data[:, 2], data[:, 8], data[:, 11], data[:, 9], data[:, 10], data[:, 12], data[:, 13]
segs, i = [], 0
while i < len(t):
    j = i
    while j + 1 < len(t) and v[j + 1] == v[i] and w[j + 1] == w[i]: j += 1
    if abs(v[i]) > 1e-6 or abs(w[i]) > 1e-6: segs.append((i, j))
    i = j + 1
uw = lambda a: np.unwrap(a)
out = []
for i, j in segs:
    k = (t >= t[i] + 1.5) & (t <= t[j] - 0.3) & (np.arange(len(t)) >= i) & (np.arange(len(t)) <= j)
    idx = np.flatnonzero(k)
    if len(idx) < 20: continue
    T, X, Y, YW, OYW, OX, OY = t[idx], px[idx], py[idx], uw(yaw[idx]), uw(oyaw[idx]), ox[idx], oy[idx]
    dt = T[-1] - T[0]; d_true = YW[-1] - YW[0]; d_odom = OYW[-1] - OYW[0]
    r = {'cmd_v': float(v[i]), 'cmd_w': float(w[i]), 'window_s': round(dt, 2)}
    if abs(w[i]) > 1e-6:
        vx = np.gradient(X, T); vy = np.gradient(Y, T); wr = np.gradient(YW, T)
        vlat = -np.sin(YW) * vx + np.cos(YW) * vy                      # 차체 옆 속도
        ok = np.abs(wr) > 0.05
        cx, cy = X.mean(), Y.mean()
        r.update(true_w=round(d_true / dt, 4), odom_w=round(d_odom / dt, 4), rot_scale_odom_over_true=round(d_odom / d_true, 4),
                 true_over_cmd=round(d_true / dt / w[i], 4), x_icr=round(float(np.median(-vlat[ok] / wr[ok])), 4),
                 base_circle_radius=round(float(np.median(np.hypot(X - cx, Y - cy))), 4))
    else:
        dist_true = float(np.sum(np.hypot(np.diff(X), np.diff(Y)))); dist_odom = float(np.sum(np.hypot(np.diff(OX), np.diff(OY))))
        r.update(true_v=round(dist_true / dt, 4), odom_over_true=round(dist_odom / dist_true, 4), true_over_cmd=round(dist_true / dt / abs(v[i]), 4),
                 yaw_drift_rad=round(float(d_true), 4))
    out.append(r)
turn = [r for r in out if 'x_icr' in r]; st = [r for r in out if 'odom_over_true' in r]
summ = {'turn_rot_scale_median': float(np.median([r['rot_scale_odom_over_true'] for r in turn])) if turn else None,
        'x_icr_median': float(np.median([r['x_icr'] for r in turn])) if turn else None,
        'straight_scale_median': float(np.median([r['odom_over_true'] for r in st])) if st else None, 'segments': out}
json.dump(summ, open(sys.argv[2], 'w'), indent=1, ensure_ascii=False)
print(json.dumps({k: summ[k] for k in ('turn_rot_scale_median', 'x_icr_median', 'straight_scale_median')}))
for r in out: print(r)
