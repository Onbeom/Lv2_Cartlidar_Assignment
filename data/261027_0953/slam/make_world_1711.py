"""SLAM 비교용 월드를 새 로봇 리비전(261027_1711)으로 복사한다 (PC, 원본 월드는 그대로 둠).

원본 Cartlidar/sim_world/worlds/b_dice/<seed>/task_world.usda 는 로봇 USD 261022_2119 를 참조한다.
복사본에서 바꾸는 것만:
  - 로봇 참조 → 261027_1711 USD
  - 상대 경로(텍스처)를 복사본 자리에서 같은 파일을 가리키게
  - 바퀴 관절 armature 0.0163 → 0.007244 (261027_1744/l2/config/wheels.yaml, 증속 벨트 1:1.5 반영. 장면이 실행 중에도 다시 넣음)
실행: python3 261027_0953/slam/make_world_1711.py [seed_1]
결과: 261027_0953/worlds_1711/<seed>/{task_world.usda, layout.json}
"""
import os
import re
import shutil
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
WORK = os.path.dirname(HERE)
TOP = os.path.dirname(WORK)
seed = sys.argv[1] if len(sys.argv) > 1 else "seed_1"
src = os.path.join(TOP, "Cartlidar/sim_world/worlds/b_dice", seed)
dst = os.path.join(WORK, "worlds_1711", seed)
OLD_ROBOT = "261022_2119/usd/waffle_4wd_261022_2119/waffle_4wd/waffle_4wd.usda"
NEW_ROBOT = os.path.join(TOP, "261027_1711/usd/waffle_4wd_261027_1711/waffle_4wd/waffle_4wd.usda")
OLD_ARM, NEW_ARM = "physxJoint:armature = 0.0163", "physxJoint:armature = 0.007244"

text = open(os.path.join(src, "task_world.usda")).read()


def rel(m):
    p = m.group(1)
    if p.endswith(OLD_ROBOT):
        target = NEW_ROBOT
    elif p.startswith("."):
        target = os.path.normpath(os.path.join(src, p))
    else:
        return m.group(0)
    if not os.path.exists(target):
        raise SystemExit(f"없는 파일: {target}")
    return "@" + os.path.relpath(target, dst) + "@"


os.makedirs(dst, exist_ok=True)
n_arm = text.count(OLD_ARM)
if n_arm != 4:
    raise SystemExit(f"바퀴 armature 줄이 4개가 아님: {n_arm}")
text, n_ref = re.subn(r"@([^@]+)@", rel, text.replace(OLD_ARM, NEW_ARM))
if NEW_ROBOT.split("/lv2_next_module/")[1] not in os.path.normpath(os.path.join(dst, re.search(r"@([^@]*waffle_4wd\.usda)@", text).group(1))):
    raise SystemExit("로봇 참조 바꾸기 실패")
open(os.path.join(dst, "task_world.usda"), "w").write(text)
shutil.copy2(os.path.join(src, "layout.json"), dst)
print(f"{dst}: 경로 {n_ref}개 다시 씀, 바퀴 armature {n_arm}줄")
