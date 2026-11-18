cd /home/pa4/git/lv2_next_module
W=Cartlidar/sim_world/worlds/b_dice_judge
~/isaacsim-venv/bin/python Cartlidar/sim_render/render_r3.py --worlds $W/seed_8 $W/seed_9 $W/seed_11 --urdf 261022_2119/urdf/waffle_4wd_description_261022_2119/urdf/waffle_4wd.urdf --out Cartlidar/sim_render/results/render_r3_judge_261025 --d435-scenes 6 --seed 20261042 2>&1 | grep -v "^\[Warning\]\|^Warning\|\[Info\]" | tail -40
echo "종료 코드 ${PIPESTATUS[0]}"
echo ALLDONE
