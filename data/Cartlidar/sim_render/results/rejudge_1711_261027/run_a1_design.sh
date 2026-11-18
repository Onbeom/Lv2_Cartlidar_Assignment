#!/usr/bin/env bash
# A1 실제 팔 자세 설계 자료: 렌더(GPU 잠금) → R4 · R6 재생이 끝나기를 기다림(같은 ROS 도메인 87) → 새 · 옛 코드 재생 → 검출률 (PC)
set -u
cd ~/git/lv2_next_module
R=Cartlidar/sim_render; O=$R/results/rejudge_1711_261027
U=261027_1711/urdf/waffle_4wd_description_261027_1711/urdf/waffle_4wd.urdf
S=/tmp/claude-1000/-home-pa4-git-lv2-next-module/53f1d6b5-f824-4b3c-9008-e4ebfdcc61fe/scratchpad
RD=$R/results/render_a1_1711_design_261027
flock /tmp/lv2_gpu.lock ~/isaacsim-venv/bin/python $R/render_a1.py --worlds Cartlidar/sim_world/worlds/b_dice/seed_{1,2,3} --scenes 6 --out $RD --/crashreporter/url= > $O/render_a1_design.log 2>&1
[ -f $RD/render_a1.json ] || { echo "A1 렌더 실패"; exit 1; }
while pgrep -f run_r4_r6.sh > /dev/null; do sleep 30; done
REPLAY=replay_a1.py $R/run_replay.sh $U $R/results/replay_a1_1711_design_new $S/a1_fix_build/install/setup.bash -- --render $RD > $O/replay_a1_design_new.log 2>&1
REPLAY=replay_a1.py $R/run_replay.sh $U $R/results/replay_a1_1711_design_old $S/a1_old_build/install/setup.bash -- --render $RD > $O/replay_a1_design_old.log 2>&1
python3 $S/a1_det_eval.py $RD label > $O/a1_design_det_label.txt 2>&1
python3 $S/a1_det_eval.py $RD old > $O/a1_design_det_old.txt 2>&1
echo 끝
