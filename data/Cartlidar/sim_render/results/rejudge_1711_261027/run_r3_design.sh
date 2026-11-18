#!/usr/bin/env bash
# R3 팔 읽기 설계 자료: 월드마다 따로 계획(CPU 병렬) → 640 · 1080p 렌더(GPU 잠금) → zbar 판정 코드 (PC)
set -u
cd ~/git/lv2_next_module
R=Cartlidar/sim_render; O=$R/results/rejudge_1711_261027
for s in 1 2 3; do
  ~/isaacsim-venv/bin/python $R/render_r3.py --worlds Cartlidar/sim_world/worlds/b_dice/seed_$s --d435-scenes 0 --out /tmp/unused \
      --only-plan --save-plan $O/r3_plan_seed$s.json > $O/r3_plan_seed$s.log 2>&1 &
done
wait
W="Cartlidar/sim_world/worlds/b_dice/seed_1 Cartlidar/sim_world/worlds/b_dice/seed_2 Cartlidar/sim_world/worlds/b_dice/seed_3"
P="$O/r3_plan_seed1.json $O/r3_plan_seed2.json $O/r3_plan_seed3.json"
flock /tmp/lv2_gpu.lock ~/isaacsim-venv/bin/python $R/render_r3.py --worlds $W --d435-scenes 0 --load-plan $P \
    --out $R/results/render_r3_1711_design640_261027 --/crashreporter/url= > $O/render_r3_640.log 2>&1
flock /tmp/lv2_gpu.lock ~/isaacsim-venv/bin/python $R/render_r3.py --worlds $W --d435-scenes 0 --load-plan $P \
    --arm-size 1920 1080 --arm-f 636 --out $R/results/render_r3_1711_design1080_261027 --/crashreporter/url= > $O/render_r3_1080.log 2>&1
cd $R/proto_r3
python3 -I zbar_check.py ../results/render_r3_1711_design640_261027 > ../results/rejudge_1711_261027/r3_zbar_640.txt 2>&1
python3 -I zbar_check.py ../results/render_r3_1711_design1080_261027 > ../results/rejudge_1711_261027/r3_zbar_1080.txt 2>&1
echo 끝
