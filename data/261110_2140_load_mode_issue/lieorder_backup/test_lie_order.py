"""#193: 퍽 pose (standing | lying) — 같은 정차에서 서 있는 퍽 먼저 (inline 제외), 누운 직육면체 세워 놓기 문장."""
import copy
import os

import math

import pytest
import yaml

from mission_planner.allocation import MissionError, parse_pucks
from mission_planner.mission_lines import to_lines
from mission_planner.planner import _pick_tour, plan

PROB = yaml.safe_load(open(os.path.join(os.path.dirname(__file__), "..", "config", "problem_q1_example.yaml"), encoding="utf-8"))
MODES = ("per_location", "single_load")


def with_pose(lying_side=None, lying_ids=()):
    p = copy.deepcopy(PROB)
    for r in p["pucks"]:
        if r["side"] == lying_side or r["id"] in lying_ids:
            r["pose"] = "lying"
    return p


def stops_of(world, lying_side):
    """_pick_tour 로 12 퍽을 한 번에 돌면 박스마다 정차 하나에 두 퍽 -> [(접근 키, [퍽, 퍽], inline)]."""
    pucks = parse_pucks(with_pose(lying_side=lying_side)["pucks"])
    _, stops = _pick_tour(world, "HOME", pucks, None)
    return [(k, ps, bool(world.approaches[k].extra.get("inline"))) for k, ps in stops]


def test_parse_pose():
    rows = copy.deepcopy(PROB["pucks"])
    assert all(p.pose == "standing" for p in parse_pucks(rows))          # 없으면 standing
    rows[0]["pose"] = "lying"
    assert parse_pucks(rows)[0].pose == "lying"
    rows[0]["pose"] = "sideways"
    with pytest.raises(MissionError):
        parse_pucks(rows)


def test_standing_first(world):
    base = {k: [p.id for p in ps] for k, ps, _ in stops_of(world, None)}
    flipped = 0
    for side in (-1, 1):
        stops = stops_of(world, side)
        assert all(len(ps) == 2 for _, ps, _ in stops)
        for k, ps, inline in stops:
            if not inline:
                assert [p.pose for p in ps] == ["standing", "lying"]           # 서 있는 퍽 먼저
                flipped += [p.id for p in ps] != base[k]
    assert flipped >= 1                                                         # 원래 순서를 뒤집은 정차가 있음


def test_inline_keeps_distance(world):
    seen = 0
    for side in (-1, 1):
        for k, ps, inline in stops_of(world, side):
            if inline:
                d = [world.approaches[k].extra["pucks"][p.side]["dist"] for p in ps]
                assert d[0] < d[1]                                              # 앞 퍽이 누워도 가까운 퍽 먼저
                seen += 1
    assert seen == 2


@pytest.mark.parametrize("mode", MODES)
def test_slots_consistent(world, mode):
    p = plan(world, with_pose(lying_side=-1), mode)
    picks = [s for s in p["steps"] if s["type"] == "pick"]
    places = [s for s in p["steps"] if s["type"] == "place"]
    assert sorted(s["puck"] for s in picks) == list(range(12))
    assert sorted(tuple(s["slot"]) for s in picks) == sorted(tuple(s["from_slot"]) for s in places)
    slot_of = {s["puck"]: tuple(s["slot"]) for s in picks}
    assert all(slot_of[s["puck"]] == tuple(s["from_slot"]) for s in places)


@pytest.mark.parametrize("mode", MODES)
def test_lie_skill_lines(world, mode):
    pr = with_pose(lying_side=1)
    p = plan(world, pr, mode)
    pose = {s["puck"]: (s["shape"], s["pose"]) for s in p["steps"] if s["type"] == "pick"}
    n_lie_box = sum(v == ("box", "lying") for v in pose.values())
    n_lie_cyl = sum(v == ("cylinder", "lying") for v in pose.values())
    assert n_lie_box and n_lie_cyl

    lines, notes = to_lines(world, p, lie_skill="lie_upright")
    assert sum("toppick_tilt" in ln for ln in lines) == 12                      # 다시 집기는 모두 기존 toppick_tilt
    assert sum("lie_upright" in ln for ln in lines) == n_lie_box
    for k, ln in enumerate(lines):
        if "lie_upright" in ln:                                                 # target_look -> 세워 놓기 -> toppick_tilt -> pallet_load
            assert lines[k - 1].split()[2].startswith("target_look:")
            assert lines[k + 1].split()[2] == "toppick_tilt"
            assert lines[k + 2].split()[2].startswith("pallet_load_")
            # 상자 사전값: lie_upright:base_footprint:x:y:yaw, 직전 target_look 퍽은 상자 중심에서 PUCK_DX (0.04 m)
            f = ln.split()[2].split(":")
            assert f[0] == "lie_upright" and f[1] == "base_footprint" and len(f) == 5
            bx, by, byaw = float(f[2]), float(f[3]), float(f[4])
            t = lines[k - 1].split()[2].split(":")
            assert math.hypot(float(t[2]) - bx, float(t[3]) - by) == pytest.approx(0.04, abs=2e-4)
            turn = lines[k - 2].split()[2] if lines[k - 2].split()[1] == "drive" else "turn_0"
            assert (byaw == 0.0) == (turn == "turn_0")                          # 요 = −turn (inline 은 0)
            if turn != "turn_0":
                assert abs(byaw) == pytest.approx(17.06, abs=0.05) and (byaw < 0) == turn.startswith("turn_p")
    assert sum("누운 원기둥" in n for n in notes) == n_lie_cyl

    lines0, notes0 = to_lines(world, p)
    assert sum("toppick_tilt" in ln for ln in lines0) == 12
    assert len(lines0) == len(lines) - n_lie_box                                # 빈 값이면 세워 놓기 줄만 빠짐
    assert sum("lie_skill 이 비어" in n for n in notes0) == n_lie_box
