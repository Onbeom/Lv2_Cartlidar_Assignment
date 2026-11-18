#!/bin/bash
# YOLO 자료용 월드 만들기 (PC). 쓰는 법: build_worlds.sh <split> <첫 seed> <필요 개수>
# 만들기 실패한 seed 는 worlds/<split>_failed.txt 에 적고 다음 번호로 채운다. R3 판정용 seed 22~31 과 겹치지 않게 101~ / 201~ / 301~ 를 쓴다.
split=$1; seed=$2; need=$3
root=$HOME/git/lv2_next_module/261026_1414/worlds
bw=$HOME/git/lv2_next_module/Cartlidar/sim_world/build_task_world.py
mkdir -p "$root/$split"
ok=$(ls -d "$root/$split"/seed_* 2>/dev/null | while read d; do [ -f "$d/layout.json" ] && echo; done | wc -l)
while [ "$ok" -lt "$need" ]; do
  out="$root/$split/seed_$seed"
  if [ -f "$out/layout.json" ]; then seed=$((seed+1)); continue; fi
  if ~/isaacsim-venv/bin/python -I "$bw" --seed "$seed" --out "$out" > "$out.log" 2>&1; then
    ok=$((ok+1)); rm -f "$out.log"
  else
    echo "$seed $(tail -1 "$out.log")" >> "$root/${split}_failed.txt"; rm -rf "$out"; mv "$out.log" "$root/${split}_seed_${seed}_failed.log"
  fi
  seed=$((seed+1))
done
echo "$split: $ok 개 완료"
