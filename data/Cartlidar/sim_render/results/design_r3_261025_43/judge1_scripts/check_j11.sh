cd /home/pa4/git/lv2_next_module/Cartlidar/sim_world
~/isaacsim-venv/bin/python -I build_task_world.py --seed 11 --out worlds/b_dice_judge/seed_11 2>&1 | grep -v "^\[Warning\]" | tail -3
~/isaacsim-venv/bin/python -I check_task_world.py --seeds 11 --out results/check_judge11_261025_2223 2>&1 | grep -v "^\[Warning\]" | tail -12
cmp -s <(python3 -c "import json;d=json.load(open('worlds/b_dice_judge/seed_11/layout.json'));d.pop('sources',None);print(json.dumps(d,sort_keys=True))") <(python3 -c "import json;d=json.load(open('results/check_judge11_261025_2223/seed_11/layout.json'));d.pop('sources',None);print(json.dumps(d,sort_keys=True))") && echo "seed 11 배치 같음" || echo "seed 11 배치 다름"
echo ALLDONE
