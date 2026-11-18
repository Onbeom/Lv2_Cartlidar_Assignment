"""도구 확인용: 메시지 stamp − (받은 시각 직전 장면 프레임의 시뮬레이션 시간). 출처가 같으면 토픽끼리 값이 비슷해야 한다.

사용: python3 stamp_offset.py <L2 결과 폴더>
"""
import bisect
import csv
import statistics
import sys

d = sys.argv[1]
ticks = [(float(r["wall_s"]), float(r["sim_s"])) for r in csv.DictReader(open(d + "/ticks.csv"))]
tw = [t[0] for t in ticks]
off = {}
for r in csv.DictReader(open(d + "/rx.csv")):
    i = bisect.bisect_right(tw, float(r["rx_wall_s"])) - 1
    if i >= 0 and r["topic"] != "/tf":
        off.setdefault(r["topic"], []).append(float(r["stamp_s"]) - ticks[i][1])
for t in sorted(off):
    v = off[t]
    print(f"{t}: 중앙 {statistics.median(v):+.4f} s, 범위 {min(v):+.4f}~{max(v):+.4f} s, {len(v)}개")
