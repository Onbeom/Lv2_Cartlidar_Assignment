#!/usr/bin/env bash
# 시나리오 전부 (README 표 3~8) 를 차례로 돌리고 check_oblique.py 로 확인. 결과 = run_<이름>/ , 요약 = summary.txt
set +u
W="$(cd "$(dirname "$0")" && pwd)"
cd "$W"
: > summary.txt
run() {   # 이름 oblique rot shift [base]
  bash "$W/run_mock.sh" "$1" "$2" "$3" "$4" "$5" > "run_$1.console.txt" 2>&1
  mv "run_$1.console.txt" "run_$1/console.txt" 2>/dev/null
}
run base 0 0 0,0 base
run off 0 0 0,0
run r0 1 0 0,0
run r20 1 20 0.03,-0.03
run r40 1 40 -0.05,0.05
run rm45 1 -45 0,0
{
  echo "### off vs base"; python3 check_oblique.py run_off --oblique 0 --base run_base | tail -15
  echo "### r0";   python3 check_oblique.py run_r0 --oblique 1 --rot 0 --shift 0,0
  echo "### r20";  python3 check_oblique.py run_r20 --oblique 1 --rot 20 --shift=0.03,-0.03
  echo "### r40";  python3 check_oblique.py run_r40 --oblique 1 --rot 40 --shift=-0.05,0.05
  echo "### rm45"; python3 check_oblique.py run_rm45 --oblique 1 --rot=-45 --shift 0,0
} >> summary.txt 2>&1
grep -E "^###|^==|^BAD" summary.txt
