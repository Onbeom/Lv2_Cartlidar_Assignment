#!/usr/bin/env bash
# 도구 확인: F1 이 끝난 뒤 이 브랜치(0.05) · 대조(0) 각 15 s
D=/home/pa4/git/lv2_next_module/261104_0056_tf_wait
until [ -f $D/f1_done.txt ]; do sleep 20; done
bash $D/run_harness.sh tool_wait 0.05 --duration 35
bash $D/run_harness.sh tool_off 0.0 --duration 35
echo tool_done > $D/tool_done.txt
