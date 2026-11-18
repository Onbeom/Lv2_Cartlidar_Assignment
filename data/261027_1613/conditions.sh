# 재검사(261027_1530) 시험 실행기들이 같이 읽는 경로·조건 (PC). 실행기에서 source 한다.
TOP="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"          # ~/git/lv2_next_module
WORK="$TOP/261027_1613"
URDF="$TOP/261027_1530/urdf/waffle_4wd_description_261027_1530/urdf/waffle_4wd.urdf"
USD="$WORK/phys_tests/usd_copy/waffle_4wd/waffle_4wd.usda"      # 261027_1530 USD 사본(원본은 바꾸지 않음)
MJ_PY="$TOP/261022_2119/mujoco/.venv/bin/python"                 # MuJoCo venv는 261022_2119 것을 같이 씀
WHEELS_YAML="$WORK/l2/config/wheels.yaml"
WA=${WA:-$(python3 -c "import sys; sys.path.insert(0, '$WORK/l2'); from wheel_kinematics import load_config; print(load_config('$WHEELS_YAML')['wheel_armature_kgm2'])")}   # 환경 변수 WA를 주면 그 값(armature 비교 실험)
export WA
AA=0.045                                                         # 팔 armature (XM430-W350 식별값, 이전과 같음)
