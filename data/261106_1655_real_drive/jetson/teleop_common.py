"""팀 저장소 사본(team/)의 OpenCR 프로토콜 · 팔 제어를 불러온다. 없으면 pyserial 은 사본을 쓴다."""
import glob
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
TEAM = HERE / 'team'
try:
    import serial  # noqa: F401
except ImportError:
    sys.path.insert(0, str(TEAM / 'vendor'))
sys.path.insert(0, str(TEAM / 'workspace/ros2_ws/src/opencr_hardware/scripts'))
sys.path.insert(0, str(TEAM / 'workspace/tools/arm_bench/slider'))

import opencr_arm_tool as tool  # noqa: E402

ROLE_WHEEL, ROLE_ARM = 1, 2


def find_port(role, name='auto'):
    """포트를 읽기만 해서 역할이 맞는 OpenCR 을 찾는다. 쓰지 않는다."""
    if name != 'auto':
        return name
    seen = []
    for p in sorted(glob.glob('/dev/ttyACM*')):
        try:
            link = tool.Link(p)
            st = link.wait_state(1.0)
            link.ser.close()
        except Exception as e:  # noqa: BLE001
            seen.append(f'{p}: {type(e).__name__}')
            continue
        seen.append(f"{p}: role={st['role']} fw={st['fw']}" if st else f'{p}: STATE 없음')
        if st and st['role'] == role:
            return p
    sys.exit(f'역할 {role} 보드를 못 찾음 ({"; ".join(seen) or "/dev/ttyACM* 없음"})')
