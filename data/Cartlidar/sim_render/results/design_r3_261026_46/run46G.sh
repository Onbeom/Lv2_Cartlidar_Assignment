# 46차 설계(판정 6차 뒤): 설계 자료 8벌 × 깊이 4조건, 설정 G = sharp_min 끔(-99) — 헛 후보의 sharp 값 분포를 보려고. 사용: bash run46G.sh
cd /home/pa4/git/lv2_next_module/Cartlidar/sim_render/proto_r3
O=../results/design_r3_261026_46
declare -A R=([base]=render_r3_261025 [holdout]=render_r3_holdout_261025 [judge1]=render_r3_judge_261025 [judge2]=render_r3_judge2_261026 [judge3]=render_r3_judge3_261026 [judge4]=render_r3_judge4_261026 [judge5]=render_r3_judge5_261026 [judge6]=render_r3_judge6_261026)
declare -A C=([G]="sharp_min=-99")
for n in "${!R[@]}"; do mkdir -p $O/$n; for c in G; do
  echo "python3 ev2.py ../results/${R[$n]} ${C[$c]} -v dump=$O/$n/${c}_clean.json > $O/$n/${c}_clean.txt 2>&1"
  for s in 0 1 2; do echo "python3 ev2.py ../results/${R[$n]} noise=0.0018 corr=3 seed=$s ${C[$c]} -v dump=$O/$n/${c}_avg5_seed$s.json > $O/$n/${c}_avg5_seed$s.txt 2>&1"; done
done; done | xargs -P 3 -I{} bash -c "{}"
echo ALLDONE
