#!/bin/bash
# 평가 자료 채점. 설계 2 고정(runs/d2/best.pt). 한 번만.
cd /home/pa4/git/lv2_next_module/261106_1818_centerpose; export PYTHONDONTWRITEBYTECODE=1
exec 9>/tmp/lv2_gpu.lock; flock -w 7200 9
Y=/home/pa4/yolo-venv/bin/python; [ -e results/summary.json ] && { echo 이미 채점함; exit 1; }; mkdir -p results; sha256sum runs/d2/best.pt tagtool/puck_tool infer_cp.py score.py train_crop.py render/render_cp.py tagtool/puck_tool.cpp > results/source_sha256.txt; uptime > results/load.txt; nvidia-smi --query-compute-apps=pid,process_name,used_memory --format=csv,noheader >> results/load.txt
python3 make_list.py data/eval > results/list.txt; tagtool/puck_tool results/list.txt > results/tag.jsonl 2> results/tag.err
timeout 900 $Y infer_cp.py --data data/eval --ckpt runs/d2/best.pt --box e3 --out results/cp_e3.jsonl > results/cp_e3.log 2>&1
timeout 900 $Y infer_cp.py --data data/eval --ckpt runs/d2/best.pt --box oracle --out results/cp_oracle.jsonl > results/cp_oracle.log 2>&1
CUDA_VISIBLE_DEVICES= timeout 1200 $Y infer_cp.py --data data/eval --ckpt runs/d2/best.pt --box oracle --device cpu --limit 100 --out results/cp_oracle_cpu.jsonl > results/cp_oracle_cpu.log 2>&1
uptime >> results/load.txt
$Y score.py --data data/eval --tag results/tag.jsonl --cp results/cp_e3.jsonl results/cp_oracle.jsonl results/cp_oracle_cpu.jsonl --out results/summary.json > results/score.log 2>&1
sed -e "s#data/val#data/eval#; s#results_val/{sys.argv\[1\]}#results/{sys.argv[1]}#; s#results_val/tag#results/tag#; s#검증 자료#평가 자료#; s#평가 자료는 안 읽음##" analyze_val.py > analyze_eval.py; $Y analyze_eval.py cp_e3 > results/analyze.log 2>&1
echo 끝 > results/done.txt
