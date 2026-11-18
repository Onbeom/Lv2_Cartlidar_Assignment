cd /home/pa4/git/lv2_next_module/Cartlidar/sim_world
for sd in 8 9 10; do ~/isaacsim-venv/bin/python -I build_task_world.py --seed $sd --out worlds/b_dice_judge/seed_$sd 2>&1 | grep -v "^\[Warning\]" | tail -5; echo "seed $sd 종료 코드 ${PIPESTATUS[0]}"; done
echo ALLDONE
