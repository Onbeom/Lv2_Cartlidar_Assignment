# 45차 설계: 설계 자료 6벌 × 깊이 4조건, 설정 E = con_max 1.08. off(기본값 꺼짐 = 44차와 같은지)는 judge4 만. ev2 가 안에서 12 프로세스를 쓰므로 3개씩. 사용: bash run45.sh
cd /home/pa4/git/lv2_next_module/Cartlidar/sim_render/proto_r3
O=../results/design_r3_261026_45
declare -A R=([base]=render_r3_261025 [holdout]=render_r3_holdout_261025 [judge1]=render_r3_judge_261025 [judge2]=render_r3_judge2_261026 [judge3]=render_r3_judge3_261026 [judge4]=render_r3_judge4_261026)
declare -A C=([off]="" [E]="con_max=1.08")
for n in "${!R[@]}"; do mkdir -p $O/$n; for c in E $([ $n = judge4 ] && echo off); do
  echo "python3 ev2.py ../results/${R[$n]} ${C[$c]} -v dump=$O/$n/${c}_clean.json > $O/$n/${c}_clean.txt 2>&1"
  for s in 0 1 2; do echo "python3 ev2.py ../results/${R[$n]} noise=0.0018 corr=3 seed=$s ${C[$c]} -v dump=$O/$n/${c}_avg5_seed$s.json > $O/$n/${c}_avg5_seed$s.txt 2>&1"; done
done; done | xargs -P 3 -I{} bash -c "{}"
echo ALLDONE
