# 44차: 설계 자료 5벌 월드로 팔 장면만 640×480 · 0.04 · 0.05 · 0.06 m 렌더(판정 기준 R3 변경 4). D435 장면은 0.
cd /home/pa4/git/lv2_next_module
W=Cartlidar/sim_world/worlds; U=261022_2119/urdf/waffle_4wd_description_261022_2119/urdf/waffle_4wd.urdf; O=Cartlidar/sim_render/results
run() {  # 이름 seed 월드...
  n=$1; sd=$2; shift 2
  ~/isaacsim-venv/bin/python Cartlidar/sim_render/render_r3.py --worlds "$@" --urdf $U --out $O/render_r3arm640_$n --d435-scenes 0 --seed $sd 2>&1 | grep -v "Warning\|\[Info\]" | tail -3
  echo "$n 렌더 종료 코드 ${PIPESTATUS[0]}"
  [ -f $O/render_r3arm640_$n/render_r3.json ] || echo "$n 실패: render_r3.json 없음(SimulationApp 은 예외에도 0 으로 끝남)"
}
run base_261026 20261027 $W/b_dice/seed_{1,2,3}
run holdout_261026 20261041 $W/b_dice_holdout/seed_{4,6,7}
run judge1_261026 20261042 $W/b_dice_judge/seed_{8,9,11}
run judge2_261026 20261042 $W/b_dice_judge2/seed_{12,13,15}
run judge3_261026 20261042 $W/b_dice_judge3/seed_{16,20,21}
echo ALLDONE
