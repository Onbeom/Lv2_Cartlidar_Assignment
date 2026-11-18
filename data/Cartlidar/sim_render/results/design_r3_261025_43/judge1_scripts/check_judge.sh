cd /home/pa4/git/lv2_next_module/Cartlidar/sim_world
~/isaacsim-venv/bin/python -I check_task_world.py --seeds 8 9 10 --out results/check_judge_261025_2223 2>&1 | grep -v "^\[Warning\]" | tail -25
for sd in 8 9 10; do cmp -s <(python3 -c "import json,sys;d=json.load(open('worlds/b_dice_judge/seed_$sd/layout.json'));d.pop('sources',None);print(json.dumps(d,sort_keys=True))") <(python3 -c "import json,sys;d=json.load(open('results/check_judge_261025_2223/seed_$sd/layout.json'));d.pop('sources',None);print(json.dumps(d,sort_keys=True))") && echo "seed $sd 배치 같음" || echo "seed $sd 배치 다름"; done
echo ALLDONE
