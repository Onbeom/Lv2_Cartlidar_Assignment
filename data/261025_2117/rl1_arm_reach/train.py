"""RL1 학습·재생 진입점. Isaac Lab 통합 학습 CLI(scripts/reinforcement_learning/train.py·play.py)를 그대로 쓰고,
그 전에 이 폴더의 과제(Waffle-RL1-ArmReach)를 gym에 등록한다. 기록은 실행한 폴더의 logs/rsl_rl/rl1_arm_reach/에 쌓인다.

사용(PC, IsaacLab-6.1 폴더에서. 기록을 이 폴더에 두려면 --log_root 대신 cd를 이 폴더로):
  uv run --frozen --extra isaacsim python <이 파일> train --task Waffle-RL1-ArmReach --num_envs 512 --seed 1
  uv run --frozen --extra isaacsim python <이 파일> play  --task Waffle-RL1-ArmReach --num_envs 16 --checkpoint <model.pt>
"""
import sys
from pathlib import Path

import warp as wp

wp.config.enable_backward = False

sys.path.insert(0, str(Path(__file__).resolve().parent))

import waffle_rl  # noqa: E402,F401  과제 등록
from isaaclab_rl.entrypoints.dispatch import run_cli  # noqa: E402

if __name__ == "__main__":
    if len(sys.argv) < 2 or sys.argv[1] not in ("train", "play"):
        sys.exit("첫 인자는 train 또는 play")
    action = sys.argv[1]
    if action == "train":
        from torch.distributed.elastic.multiprocessing.errors import record

        raise SystemExit(record(lambda: run_cli("train", sys.argv[2:]))())
    raise SystemExit(run_cli("play", sys.argv[2:]))
