"""퍽 YOLO 클래스 번호: 도구들이 번호 대신 클래스 이름('<색>_<모양>')으로 색 · 모양을 읽게 하는 공용 모듈.

4클래스(사용자 결정 2026-10-26, 다음 학습부터): 번호 = 모양 × 2 + 색 → 0 red_cylinder · 1 green_cylinder · 2 red_cuboid · 3 green_cuboid.
6클래스(sim_v1 · ft1, render_yolo.py 출력): 번호 = 모양 × 3 + 색, 파랑(2 · 5)은 자료 없음.
색 순서는 팀 perception 과 같음(0 red · 1 green).
"""
from pathlib import Path

NAMES4 = ["red_cylinder", "green_cylinder", "red_cuboid", "green_cuboid"]
NAMES6 = ["red_cylinder", "green_cylinder", "blue_cylinder", "red_cuboid", "green_cuboid", "blue_cuboid"]
REMAP_6_TO_4 = {0: 0, 1: 1, 3: 2, 4: 3}
BGR = {"red": (0, 0, 255), "green": (0, 200, 0), "blue": (255, 0, 0)}


def split(name):
    """'red_cylinder' → ('red', 'cylinder')."""
    color, shape = name.split("_", 1)
    return color, shape


def names_of(model):
    """ultralytics 모델의 names(dict 또는 list) → 번호 순 이름 목록."""
    n = model.names
    return [n[i] for i in sorted(n)] if isinstance(n, dict) else list(n)


def read_names(path, default=NAMES6):
    """classes.txt(한 줄에 이름 하나) → 이름 목록. 없으면 default."""
    path = Path(path)
    if not path.exists():
        return list(default)
    return [ln.strip() for ln in path.read_text().split("\n") if ln.strip()]


def remap_lines(lines, mapping=REMAP_6_TO_4):
    """YOLO 라벨 줄들의 클래스 번호를 바꾼다. mapping 에 없는 번호(파랑)가 있으면 멈춘다."""
    out = []
    for ln in lines:
        if not ln.strip():
            continue
        k, rest = ln.split(" ", 1)
        if int(k) not in mapping:
            raise ValueError(f"바꿀 수 없는 클래스 번호 {k}(파랑?)")
        out.append(f"{mapping[int(k)]} {rest}")
    return out
