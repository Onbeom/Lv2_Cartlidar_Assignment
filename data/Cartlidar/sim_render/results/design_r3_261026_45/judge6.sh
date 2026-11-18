# 판정 6차(R3 변경 2 · 3 · 4). judge5.sh 와 경로만 다름. 팔은 render_r3.py 기본값(640×480 · f 283 · 0.04 · 0.05 · 0.06 m)
cd /home/pa4/git/lv2_next_module/Cartlidar/sim_world
OK=(); sd=28
while [ ${#OK[@]} -lt 3 ] && [ $sd -le 37 ]; do
  ~/isaacsim-venv/bin/python -I build_task_world.py --seed $sd --out worlds/b_dice_judge6/seed_$sd 2>&1 | grep -v "^\[Warning\]" | tail -3
  rc=${PIPESTATUS[0]}; echo "seed $sd 생성 종료 코드 $rc"
  if [ $rc -eq 0 ]; then OK+=($sd); else mkdir -p worlds/b_dice_judge6_rejected; mv worlds/b_dice_judge6/seed_$sd worlds/b_dice_judge6_rejected/ 2>/dev/null; echo "seed $sd 제외(생성 실패)"; fi
  sd=$((sd+1))
done
echo "쓸 seed: ${OK[*]}"
C=results/check_judge6_261026
~/isaacsim-venv/bin/python -I check_task_world.py --seeds ${OK[*]} --out $C 2>&1 | grep -v "^\[Warning\]" | tail -30
crc=${PIPESTATUS[0]}; echo "check 종료 코드 $crc"
for s in ${OK[*]}; do cmp -s <(python3 -c "import json;d=json.load(open('worlds/b_dice_judge6/seed_$s/layout.json'));d.pop('sources',None);print(json.dumps(d,sort_keys=True))") <(python3 -c "import json;d=json.load(open('$C/seed_$s/layout.json'));d.pop('sources',None);print(json.dumps(d,sort_keys=True))") && echo "seed $s 배치 같음" || { echo "seed $s 배치 다름"; crc=9; }; done
if [ $crc -ne 0 ]; then echo "검사 실패로 멈춤"; echo ALLDONE; exit 0; fi
cd /home/pa4/git/lv2_next_module
W=Cartlidar/sim_world/worlds/b_dice_judge6
# 같은 GPU(8 GB)를 다른 세션(RL 학습 · YOLO 렌더 · 학습)이 쓰면 끝날 때까지 기다림(연속 2번 GPU 계산 프로세스 0 일 때 시작). 렌더 중 GPU 메모리를 10 s 마다 기록
idle=0; while [ $idle -lt 2 ]; do if [ -z "$(nvidia-smi --query-compute-apps=pid --format=csv,noheader)" ]; then idle=$((idle+1)); else idle=0; fi; sleep 30; done; echo "렌더 시작 $(date +%T)"
exec 9>/tmp/lv2_gpu.lock; flock 9   # GPU 작업끼리 같은 잠금 파일로 차례 지킴(YOLO 세션과 약속)
mkdir -p Cartlidar/sim_render/results/judge_r3_6_261026; nvidia-smi --query-gpu=timestamp,memory.used --format=csv -l 10 > Cartlidar/sim_render/results/judge_r3_6_261026/gpu_mem_render.csv & GP=$!
~/isaacsim-venv/bin/python Cartlidar/sim_render/render_r3.py --worlds $(for s in ${OK[*]}; do echo -n "$W/seed_$s "; done) --urdf 261022_2119/urdf/waffle_4wd_description_261022_2119/urdf/waffle_4wd.urdf --out Cartlidar/sim_render/results/render_r3_judge6_261026 --d435-scenes 6 --seed 20261042 2>&1 | grep -v "Warning\|\[Info\]" | tail -5
echo "렌더 종료 코드 ${PIPESTATUS[0]}"
kill $GP 2>/dev/null; flock -u 9
[ -f Cartlidar/sim_render/results/render_r3_judge6_261026/render_r3.json ] || { echo "렌더 실패: render_r3.json 없음"; echo ALLDONE; exit 0; }
cd Cartlidar/sim_render/proto_r3
R=../results/render_r3_judge6_261026; O=../results/judge_r3_6_261026; mkdir -p $O
sha256sum locate_v2.py ev2.py zbar_check.py > $O/code_sha256.txt
python3 ev2.py $R -v dump=$O/d435_clean.json > $O/d435_clean.txt 2>&1
for s in 0 1 2; do python3 ev2.py $R noise=0.0018 corr=3 seed=$s -v dump=$O/d435_avg5_seed$s.json > $O/d435_avg5_seed$s.txt 2>&1; done
python3 -I zbar_check.py $R > $O/arm_zbar.txt 2>&1
echo ALLDONE
