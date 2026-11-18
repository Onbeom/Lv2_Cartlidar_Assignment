"""도구 확인(판정 아님): plan_only.json 경로마다 제자리 회전 횟수 · 합계 각, 호(전진하며 도는) 길이, 직선 길이, 짧은 이동 구간(< 0.03 m) 수를 센다.
설계 44(사용자 2026-10-24 결정 '제자리 회전 최소화')에서 설계 43 경로와 비교용.
사용: python3 path_stats.py <plan_only.json...>"""
import json, math, sys
for f in sys.argv[1:]:
    tot = dict(rot_n=0, rot_deg=0.0, arc_m=0.0, straight_m=0.0, short=0)
    rows = []
    for leg in json.load(open(f))['legs']:
        P = leg['path_rc']
        rn, rd, am, sm, sh = 0, 0.0, 0.0, 0.0, 0
        in_rot = False; seg = 0.0
        for a, b in zip(P, P[1:]):
            dxy = math.hypot(b[0]-a[0], b[1]-a[1]); dth = abs(math.remainder(b[2]-a[2], 2*math.pi))
            if dxy < 1e-3 and dth > 1e-3:
                if not in_rot:
                    rn += 1
                    if 1e-3 < seg < 0.03: sh += 1
                    seg = 0.0
                in_rot = True; rd += math.degrees(dth)
            else:
                in_rot = False; seg += dxy
                if dth > 1e-3: am += dxy
                else: sm += dxy
        rows.append(f"{leg['i']}: 회전 {rn} 번 {rd:.0f}° · 호 {am:.2f} m · 직선 {sm:.2f} m · 짧은 구간 {sh} · 쓸림 {leg['min_clearance_sweep_m']}")
        for k, v in zip(tot, (rn, rd, am, sm, sh)): tot[k] += v
    print(f); print('  ' + '\n  '.join(rows))
    print(f"  합계: 제자리 회전 {tot['rot_n']} 번 {tot['rot_deg']:.0f}°, 호 {tot['arc_m']:.2f} m, 직선 {tot['straight_m']:.2f} m, 짧은 구간 {tot['short']}")
