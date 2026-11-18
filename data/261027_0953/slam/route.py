"""미로 칸 연결(layout.json의 칸별 벽)로 출발 칸에서 모든 칸을 돌고 출발 칸으로 돌아오는 경로(깊이 우선 순회)를 만든다.

벽 판정: 두 칸 사이 변에 어느 한쪽이라도 벽(edges의 side)이 있으면 막힘. 이웃 순서는 오른쪽·위·왼쪽·아래(고정)라
같은 layout이면 늘 같은 경로가 나온다.
사용: python3 route.py <layout.json>  → 칸 순서와 길이를 출력
"""
import json
import sys

OPP = {"left": "right", "right": "left", "top": "bottom", "bottom": "top"}
STEP = {"right": (1, 0), "top": (0, 1), "left": (-1, 0), "bottom": (0, -1)}


def load_cells(layout_path):
    d = json.load(open(layout_path))
    cells = {}
    for c in d["cells"]:
        i, j = int(c["id"][1]), int(c["id"][2])
        cells[(i, j)] = {"id": c["id"], "center": tuple(c["center"]), "walls": {e["side"] for e in c["edges"]}}
    return d, cells


def neighbors(cells, ij):
    out = []
    for side in ("right", "top", "left", "bottom"):
        di, dj = STEP[side]
        nb = (ij[0] + di, ij[1] + dj)
        if nb in cells and side not in cells[ij]["walls"] and OPP[side] not in cells[nb]["walls"]:
            out.append(nb)
    return out


def tour(cells, start):
    """깊이 우선으로 모든 칸을 방문하고 되돌아오는 칸 순서(되돌아가는 칸 포함, 처음·끝 = start)."""
    seen, order = {start}, [start]

    def visit(ij):
        for nb in neighbors(cells, ij):
            if nb not in seen:
                seen.add(nb)
                order.append(nb)
                visit(nb)
                order.append(ij)

    visit(start)
    return order, seen


def route_waypoints(layout_path):
    d, cells = load_cells(layout_path)
    sc = d["robot"]["start_cell"]
    start = (int(sc[1]), int(sc[2]))
    order, seen = tour(cells, start)
    return [cells[ij]["center"] for ij in order], [cells[ij]["id"] for ij in order], len(seen), len(cells)


if __name__ == "__main__":
    wps, ids, n_seen, n_all = route_waypoints(sys.argv[1])
    length = sum(((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5 for a, b in zip(wps, wps[1:]))
    print(f"방문 칸 {n_seen}/{n_all}, 경유점 {len(wps)}, 길이 {length:.2f} m")
    print(" → ".join(ids))
