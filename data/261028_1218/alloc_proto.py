"""주사위 미션 할당 계산 시제품 (팀 mission 에 넣기 전 규칙 확인용).

입력: 퍽 조사 표 12행(id, color red|green, shape cylinder|cuboid), 신호 횟수 {R,Y,G}, 바코드 문자열.
출력: L1~L3 그룹 · 장소별 퍽 ID · 격자 칸, 또는 오류.
실행: python3 -I alloc_proto.py   (전수 탐색과 비교하는 자체 시험)
"""
import itertools
import random
import time

ASC = {'P2-01', 'P2-03'}
DESC = {'P2-02', 'P2-04'}
ORDER = 'RYG'   # 동률이면 이 순서 유지

# 주사위 눈 칸 (장소 중심 기준 mm, +Y = 지도 위쪽). 발제 4쪽 그림
DICE = {
    1: [(0, 0)],
    2: [(-60, 60), (60, -60)],
    3: [(-60, 60), (0, 0), (60, -60)],
    4: [(-60, 60), (60, 60), (-60, -60), (60, -60)],
    5: [(-60, 60), (60, 60), (0, 0), (-60, -60), (60, -60)],
    6: [(-60, 60), (-60, 0), (-60, -60), (60, 60), (60, 0), (60, -60)],
}


def assign_groups(counts, barcode):
    """신호 횟수 → L1, L2, L3 에 놓을 그룹 문자 (안정 정렬이라 동률은 R → Y → G)."""
    if barcode in ASC:
        return sorted(ORDER, key=lambda g: counts[g])
    if barcode in DESC:
        return sorted(ORDER, key=lambda g: -counts[g])
    raise ValueError(f'미등록 바코드 {barcode!r}')


def plan_y(pucks, counts, prefer=('cylinder', 'cuboid')):
    """Y 모양을 정하고 Y 에 쓸 빨강 · 초록 수를 닫힌 식으로 구함.
    Y 를 뺀 나머지가 R · G 에 정확히 맞아야 하므로 y_red = 빨강 전체 − R, y_green = 초록 전체 − G 로 정해진다."""
    red = [p for p in pucks if p['color'] == 'red']
    green = [p for p in pucks if p['color'] == 'green']
    y_red, y_green = len(red) - counts['R'], len(green) - counts['G']
    if y_red < 0 or y_green < 0 or y_red + y_green != counts['Y']:
        return None
    for shape in prefer:
        if sum(p['shape'] == shape for p in red) >= y_red and sum(p['shape'] == shape for p in green) >= y_green:
            return shape, y_red, y_green
    return None


def allocate(pucks, counts, barcode):
    ids = [p['id'] for p in pucks]
    if len(pucks) != 12 or len(set(ids)) != 12:
        raise ValueError('퍽 ID 중복 또는 12개 아님')
    if any(p['color'] not in ('red', 'green') or p['shape'] not in ('cylinder', 'cuboid') for p in pucks):
        raise ValueError('색 · 모양 불명 퍽 있음')
    if sum(counts.values()) != 12 or not all(1 <= counts[g] <= 6 for g in ORDER):
        raise ValueError(f'신호 횟수 오류 {counts}')
    order = assign_groups(counts, barcode)
    y = plan_y(pucks, counts)
    if y is None:
        raise ValueError('유효한 할당 없음')
    shape, y_red, y_green = y
    # 어떤 ID 를 Y 로 쓸지는 자유. 여기서는 ID 작은 순(실제로는 주행 거리로 고름)
    yr = [p for p in pucks if p['color'] == 'red' and p['shape'] == shape][:y_red]
    yg = [p for p in pucks if p['color'] == 'green' and p['shape'] == shape][:y_green]
    used = {p['id'] for p in yr + yg}
    groups = {
        'Y': yr + yg,
        'R': [p for p in pucks if p['color'] == 'red' and p['id'] not in used],
        'G': [p for p in pucks if p['color'] == 'green' and p['id'] not in used],
    }
    out = []
    for loc, g in zip(('L1', 'L2', 'L3'), order):
        ps = groups[g]
        out.append({'location': loc, 'group': g, 'y_shape': shape if g == 'Y' else None,
                    'cells': [{'id': p['id'], 'offset_mm': c} for p, c in zip(ps, DICE[len(ps)])]})
    return out


def brute_feasible(pucks, counts):
    """전수 탐색: Y 로 쓸 퍽 조합을 모두 보고 나머지가 R · G 에 맞는지."""
    for shape in ('cylinder', 'cuboid'):
        cand = [p for p in pucks if p['shape'] == shape]
        for ys in itertools.combinations(cand, counts['Y']):
            rest = [p for p in pucks if p not in ys]
            if sum(p['color'] == 'red' for p in rest) == counts['R'] and sum(p['color'] == 'green' for p in rest) == counts['G']:
                return True
    return False


def check(pucks, counts, barcode):
    """할당 결과가 규칙에 맞는지 (시험용)."""
    a = allocate(pucks, counts, barcode)
    byid = {p['id']: p for p in pucks}
    seen = [c['id'] for loc in a for c in loc['cells']]
    assert sorted(seen) == sorted(byid), '중복/누락'
    for loc in a:
        ps = [byid[c['id']] for c in loc['cells']]
        assert len(ps) == counts[loc['group']]
        if loc['group'] == 'R':
            assert all(p['color'] == 'red' for p in ps)
        if loc['group'] == 'G':
            assert all(p['color'] == 'green' for p in ps)
        if loc['group'] == 'Y':
            assert len({p['shape'] for p in ps}) == 1


def main():
    # 발제 Q1: RGRRGRGGYGYG → R4 Y2 G6, P2-01 오름차순 → Y(2) · R(4) · G(6)
    seq = 'RGRRGRGGYGYG'
    counts = {g: seq.count(g) for g in ORDER}
    print('Q1', counts, assign_groups(counts, 'P2-01'), assign_groups(counts, 'P2-02'))
    # 동률: R3 Y3 G6 → 오름 R, Y, G / 내림 G, R, Y
    print('동률', assign_groups({'R': 3, 'Y': 3, 'G': 6}, 'P2-01'), assign_groups({'R': 3, 'Y': 3, 'G': 6}, 'P2-04'))

    rng = random.Random(0)
    n = agree = feas = 0
    t0 = time.perf_counter()
    for _ in range(20000):
        pucks = [{'id': i, 'color': rng.choice(('red', 'green')), 'shape': rng.choice(('cylinder', 'cuboid'))} for i in range(12)]
        r = rng.randint(1, 6)
        y = rng.randint(1, 6)
        g = 12 - r - y
        if not 1 <= g <= 6:
            continue
        counts = {'R': r, 'Y': y, 'G': g}
        bf = brute_feasible(pucks, counts)
        cf = plan_y(pucks, counts) is not None
        n += 1
        agree += bf == cf
        feas += cf
        if cf:
            check(pucks, counts, rng.choice(sorted(ASC | DESC)))
    print(f'무작위 {n}건: 닫힌 식 = 전수 탐색 {agree}/{n}, 풀 수 있음 {feas}, {time.perf_counter() - t0:.1f} s')


if __name__ == '__main__':
    main()
