#!/usr/bin/env bash
# 261027_1711 재판정: R4 · R6 렌더(GPU 잠금) → 재생 · 판정 (PC). 저장소 밖 lv2_next_module 에서 실행.
set -eu
cd ~/git/lv2_next_module
R=Cartlidar/sim_render
U=261027_1711/urdf/waffle_4wd_description_261027_1711/urdf/waffle_4wd.urdf
SETUP=/tmp/claude-1000/-home-pa4-git-lv2-next-module/53f1d6b5-f824-4b3c-9008-e4ebfdcc61fe/scratchpad/a1_fix_build/install/setup.bash
W="Cartlidar/sim_world/worlds/b_dice/seed_1 Cartlidar/sim_world/worlds/b_dice/seed_2 Cartlidar/sim_world/worlds/b_dice/seed_3"
flock /tmp/lv2_gpu.lock ~/isaacsim-venv/bin/python $R/render_r4.py --worlds $W --out $R/results/render_r4_1711_aim_261027 --/crashreporter/url= > $R/results/rejudge_1711_261027/render_r4_aim.log 2>&1
flock /tmp/lv2_gpu.lock ~/isaacsim-venv/bin/python $R/render_r6.py --worlds $W --out $R/results/render_r6_1711_aim_261027 --/crashreporter/url= > $R/results/rejudge_1711_261027/render_r6_aim.log 2>&1
while pgrep -f run_a1_design > /dev/null; do sleep 30; done   # 같은 ROS 도메인 87 재생이 겹치지 않게
REPLAY=replay_r4.py $R/run_replay.sh $U $R/results/replay_r4_1711_aim_261027_arm $SETUP -- --render $R/results/render_r4_1711_aim_261027 arm > $R/results/rejudge_1711_261027/replay_r4_aim_arm.log 2>&1
REPLAY=replay_r6.py $R/run_replay.sh $U $R/results/replay_r6_1711_aim_261027 $SETUP -- --render $R/results/render_r6_1711_aim_261027 > $R/results/rejudge_1711_261027/replay_r6_aim.log 2>&1
echo 끝
