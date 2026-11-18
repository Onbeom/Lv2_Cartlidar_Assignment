cd /home/pa4/git/lv2_next_module/Cartlidar/sim_render/proto_r3
R=../results/render_r3_judge_261025; O=../results/judge_r3_261025; mkdir -p $O
sha256sum locate_v2.py ev2.py zbar_check.py > $O/code_sha256.txt
python3 ev2.py $R -v dump=$O/d435_clean.json > $O/d435_clean.txt 2>&1
for sd in 0 1 2; do python3 ev2.py $R noise=0.0018 corr=3 seed=$sd -v dump=$O/d435_avg5_seed$sd.json > $O/d435_avg5_seed$sd.txt 2>&1; done
python3 -I zbar_check.py $R > $O/arm_zbar.txt 2>&1
echo ALLDONE
