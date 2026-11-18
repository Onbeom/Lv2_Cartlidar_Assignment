cd /home/pa4/git/lv2_next_module/Cartlidar/sim_world
OK=(); sd=16
while [ ${#OK[@]} -lt 3 ] && [ $sd -le 25 ]; do
  ~/isaacsim-venv/bin/python -I build_task_world.py --seed $sd --out worlds/b_dice_judge3/seed_$sd 2>&1 | grep -v "^\[Warning\]" | tail -3
  rc=${PIPESTATUS[0]}; echo "seed $sd 생성 종료 코드 $rc"
  if [ $rc -eq 0 ]; then OK+=($sd); else mkdir -p worlds/b_dice_judge3_rejected; mv worlds/b_dice_judge3/seed_$sd worlds/b_dice_judge3_rejected/ 2>/dev/null; echo "seed $sd 제외(생성 실패)"; fi
  sd=$((sd+1))
done
echo "쓸 seed: ${OK[*]}"
C=results/check_judge3_261026_0932
~/isaacsim-venv/bin/python -I check_task_world.py --seeds ${OK[*]} --out $C 2>&1 | grep -v "^\[Warning\]" | tail -30
crc=${PIPESTATUS[0]}; echo "check 종료 코드 $crc"
for s in ${OK[*]}; do cmp -s <(python3 -c "import json;d=json.load(open('worlds/b_dice_judge3/seed_$s/layout.json'));d.pop('sources',None);print(json.dumps(d,sort_keys=True))") <(python3 -c "import json;d=json.load(open('$C/seed_$s/layout.json'));d.pop('sources',None);print(json.dumps(d,sort_keys=True))") && echo "seed $s 배치 같음" || { echo "seed $s 배치 다름"; crc=9; }; done
if [ $crc -ne 0 ]; then echo "검사 실패로 멈춤"; echo ALLDONE; exit 0; fi
cd /home/pa4/git/lv2_next_module
W=Cartlidar/sim_world/worlds/b_dice_judge3
~/isaacsim-venv/bin/python Cartlidar/sim_render/render_r3.py --worlds $(for s in ${OK[*]}; do echo -n "$W/seed_$s "; done) --urdf 261022_2119/urdf/waffle_4wd_description_261022_2119/urdf/waffle_4wd.urdf --out Cartlidar/sim_render/results/render_r3_judge3_261026 --d435-scenes 6 --seed 20261042 2>&1 | grep -v "Warning\|\[Info\]" | tail -5
echo "렌더 종료 코드 ${PIPESTATUS[0]}"
cd Cartlidar/sim_render/proto_r3
R=../results/render_r3_judge3_261026; O=../results/judge_r3_3_261026; mkdir -p $O
sha256sum locate_v2.py ev2.py zbar_check.py > $O/code_sha256.txt
python3 ev2.py $R -v dump=$O/d435_clean.json > $O/d435_clean.txt 2>&1
for s in 0 1 2; do python3 ev2.py $R noise=0.0018 corr=3 seed=$s -v dump=$O/d435_avg5_seed$s.json > $O/d435_avg5_seed$s.txt 2>&1; done
python3 -I zbar_check.py $R > $O/arm_zbar.txt 2>&1
echo ALLDONE
