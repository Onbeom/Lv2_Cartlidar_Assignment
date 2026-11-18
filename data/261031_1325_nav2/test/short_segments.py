"""도구 확인(판정 아님): plan_only.json 경로에서 제자리 회전 자리 사이 이동이 짧은(기본 < 0.03 m = Shim forward_sampling_distance) 구간을 찾는다.
이런 구간은 Shim 이 다음 회전 방향까지 한 번에 돌아 경로 처리기의 구간 끝 방향 조건(yaw 허용)을 못 맞출 수 있다(설계 42 s20261101 출발 모퉁이).
사용: python3 short_segments.py <plan_only.json...> [--max 0.03]"""
import argparse, json, math
ap = argparse.ArgumentParser(); ap.add_argument('files', nargs='+'); ap.add_argument('--max', type=float, default=0.03)
a = ap.parse_args()
for f in a.files:
    out = []
    for leg in json.load(open(f))['legs']:
        P = leg['path_rc']
        rot = [i for i in range(1, len(P)) if math.hypot(P[i][0]-P[i-1][0], P[i][1]-P[i-1][1]) < 1e-3 and abs(math.remainder(P[i][2]-P[i-1][2], 2*math.pi)) > 0.01]
        for p, q in zip(rot, rot[1:]):
            L = sum(math.hypot(P[i][0]-P[i-1][0], P[i][1]-P[i-1][1]) for i in range(p, q))
            if 1e-3 < L < a.max:
                out.append(f"구간 {leg['i']} ({P[p][0]:.3f}, {P[p][1]:.3f}) 이동 {L:.3f}")
    print(f, len(out), out)
