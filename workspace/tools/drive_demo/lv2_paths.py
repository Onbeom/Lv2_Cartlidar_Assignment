"""저장소 위치 찾기 — 노트북 · Jetson 어디서든 같은 도구를 쓰도록.

  환경 변수 LV2_SRC (…/workspace/ros2_ws/src) 를 주면 그것을, 없으면 아래 후보를 차례로 찾는다.
  python3 lv2_paths.py  → 찾은 경로 출력 (run_T.sh 도 이것으로 config.h 를 찾음)
"""
import os
import sys
from pathlib import Path

_CANDIDATES = [
    str(Path(__file__).resolve().parents[3]),          # 이 파일이 든 저장소 (workspace/tools/drive_demo)
    '~/GIT/lv2_CartLidar/Lv2_Cartlidar_Assignment',   # 노트북
    '~/GIT/Lv2_Cartlidar_Assignment',
    '~/Lv2_Cartlidar_Assignment',
    '~/ws/Lv2_Cartlidar_Assignment',
]


def find_src():
    env = os.environ.get('LV2_SRC')
    if env:
        p = Path(env).expanduser()
        if (p / 'opencr_hardware' / 'scripts' / 'opencr_arm_tool.py').exists():
            return p
        sys.exit(f'LV2_SRC={env} 에 opencr_hardware/scripts/opencr_arm_tool.py 가 없음')
    for c in _CANDIDATES:
        p = Path(c).expanduser() / 'workspace' / 'ros2_ws' / 'src'
        if (p / 'opencr_hardware' / 'scripts' / 'opencr_arm_tool.py').exists():
            return p
    sys.exit('저장소를 못 찾음 — LV2_SRC=<저장소>/workspace/ros2_ws/src 로 알려 주세요')


SRC = find_src()
SCRIPTS = SRC / 'opencr_hardware' / 'scripts'
CONFIG_H = SRC / 'opencr_firmware' / 'config.h'

if __name__ == '__main__':
    print(SRC)
