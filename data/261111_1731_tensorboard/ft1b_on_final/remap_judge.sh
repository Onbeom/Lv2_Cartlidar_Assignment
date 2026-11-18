#!/bin/bash
# ft1b(6클래스, 파랑 포함)를 최종 판정 영상에 돌린 검출을 4클래스로 바꿔 E3 와 같은 규칙으로 판정한다(참고, 판정 아님).
# 6 → 4: 0→0 red_cyl, 1→1 green_cyl, 3→2 red_cub, 4→3 green_cub, 파랑(2 · 5)은 버림.
set -e
H=$(cd "$(dirname "$0")" && pwd); F=$H/../../261105_2047_final_eval
for n in ft1b_raw ft1b_best_raw; do
  o=${n%_raw}; o=${o/ft1b/ft1bL}; [ "$n" = ft1b_best_raw ] && o=ft1bB
  python3 - "$H/dets_$n.json" "$F/dets_$o.json" <<'EOF'
import json, sys
M = {0: 0, 1: 1, 3: 2, 4: 3}
d = json.load(open(sys.argv[1]))
json.dump({k: [[M[int(c)], *r] for c, *r in v if int(c) in M] for k, v in d.items()}, open(sys.argv[2], 'w'))
print(sys.argv[2], sum(len(v) for v in d.values()), '→', sum(1 for v in d.values() for c, *_ in v if int(c) in M))
EOF
done
cd "$F" && JUDGE_S0_FIXED=114 JUDGE_AGNOSTIC=final_rear JUDGE_CAMS=final_front,final_rear,final_arm JUDGE_OUT=judge_ft1b_ref.json ~/yolo-venv/bin/python judge.py ft1bL ft1bB E3 | grep -v '겹친\|s0'
