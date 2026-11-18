# 44차 설계: 설정 A~D × 깊이 4조건. 사용: bash run44.sh <렌더 폴더 이름> <출력 이름> [설정...]
cd /home/pa4/git/lv2_next_module/Cartlidar/sim_render/proto_r3
R=../results/$1; O=../results/design_r3_261026_44/$2; mkdir -p $O; shift 2
declare -A C=([A]="top_h=0.2" [B]="top_h=0.2 side_min_frac=0.3" [C]="top_h=0.2 side_min_frac=0.3 rescore_m=0.05" [D]="top_h=0.2 side_min_frac=0.3 rescore_m=0.05 valid_frac=0.9")
for c in "$@"; do
  python3 ev2.py $R ${C[$c]} -v dump=$O/${c}_clean.json > $O/${c}_clean.txt 2>&1
  for s in 0 1 2; do python3 ev2.py $R noise=0.0018 corr=3 seed=$s ${C[$c]} -v dump=$O/${c}_avg5_seed$s.json > $O/${c}_avg5_seed$s.txt 2>&1; done
  echo "$c 끝"
done
echo ALLDONE
