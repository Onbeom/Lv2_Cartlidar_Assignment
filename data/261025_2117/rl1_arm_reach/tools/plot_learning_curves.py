"""RL1 학습 곡선 그림: 학습 중 위치 오차(Metrics/tcp_target/position_error, 학습 목표 표 기준)를 iteration별로 그린다.

TensorBoard 기록(logs/rsl_rl/rl1_arm_reach/<실행>/events.*)을 읽는다. 판정(J1~J5)은 평가 스크립트 결과이고 이 그림은 학습 중 값이다.
사용(PC): ~/pa-course/IsaacLab-6.1/.venv/bin/python plot_learning_curves.py --out <png>
"""
import argparse
import glob
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from tensorboard.backend.event_processing.event_accumulator import EventAccumulator  # noqa: E402

LOGS = Path(__file__).resolve().parents[1] / "logs" / "rsl_rl" / "rl1_arm_reach"
TAG = "Metrics/tcp_target/position_error"
# (실행 폴더 끝 이름, 표시 이름, 색) — 색은 검증한 범주 팔레트 1~6번(밝은 배경)
RUNS = [
    ("_v2_seed1", "v2 (속도 명령, 1000 it)", "#2a78d6"),
    ("_v3s1_sweep", "v3 S1 (200 it)", "#eb6834"),
    ("_v3s2_sweep", "v3 S2 (200 it)", "#1baf7a"),
    ("_v3s3_sweep", "v3 S3 (200 it)", "#eda100"),
    ("_v3s3_seed1", "v3 S3 본 학습", "#e87ba4"),
    ("_v4_seed1", "v4 본 학습", "#008300"),
]


def load(suffix):
    d = sorted(glob.glob(str(LOGS / f"*{suffix}")))
    if not d:
        return None
    ea = EventAccumulator(d[-1], size_guidance={"scalars": 0})
    ea.Reload()
    if TAG not in ea.Tags()["scalars"]:
        return None
    ev = ea.Scalars(TAG)
    return [e.step for e in ev], [e.value * 1000.0 for e in ev]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    plt.rcParams["font.family"] = "Noto Sans CJK KR"
    fig, ax = plt.subplots(figsize=(10, 5.6), dpi=130)
    fig.patch.set_facecolor("#fcfcfb")
    ax.set_facecolor("#fcfcfb")
    last = {}
    for suffix, name, color in RUNS:
        r = load(suffix)
        if r is None:
            continue
        x, y = r
        ax.plot(x, y, color=color, lw=2, label=name)
        last[name] = (x[-1], y[-1], color)
    ax.axhline(10, color="#8a8984", lw=1, ls=(0, (4, 3)))
    ax.text(1005, 10, "J1 기준 10 mm\n(평가: 3 s 시점)", va="center", fontsize=8.5, color="#5f5e5a")
    for name, (x, y, color) in last.items():
        ax.plot([x], [y], "o", ms=8, color=color, mec="#fcfcfb", mew=2)
        ax.annotate(f"{y:.0f} mm", (x, y), xytext=(6, 6), textcoords="offset points", fontsize=9, color="#2b2b29")
    ax.set_xlim(0, 1130)
    ax.set_ylim(0, 360)
    ax.set_xlabel("학습 iteration (1 iteration = 1024 환경 × 24 정책 스텝)", color="#5f5e5a")
    ax.set_ylabel("학습 중 평균 위치 오차 (mm)", color="#5f5e5a")
    ax.set_title("RL1 팔 도달 학습 곡선 (모의, Isaac Lab + Isaac Sim PhysX, 고친 USD)", loc="left", color="#2b2b29")
    ax.grid(color="#e4e3df", lw=0.8)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    for s in ("left", "bottom"):
        ax.spines[s].set_color("#c3c2b7")
    ax.tick_params(colors="#5f5e5a")
    ax.legend(loc="upper right", frameon=False, fontsize=9)
    fig.tight_layout()
    Path(a.out).parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(a.out, facecolor=fig.get_facecolor())
    print("saved", a.out, {k: round(v[1], 1) for k, v in last.items()})


if __name__ == "__main__":
    main()
