"""arm_link_5 표시 메시(DAE)에서 집게 체결 나사 세트 4개(나사 4 + 너트 4 = 8조각)를 지운다.

이 나사들은 집게 손가락 부품인데 CAD 내보내기에서 arm_link_5 메시에 합쳐져 있었다(실물 arm_link_5에는 없음).
표시 전용이라 충돌·질량·관절과는 관계없다.

조각 고르기: 같은 좌표의 점을 하나로 보고 연결된 삼각형 묶음(조각)을 나눈 뒤,
DAE 좌표에서 아래 상자 안에 완전히 들어가는 조각만 지운다. 고른 조각 수와 삼각형 수가 기대값과 다르면 아무것도 쓰지 않는다.

사용 (PC):
  python3 remove_arm_link_5_fasteners.py <원본 arm_link_5.dae> <새 arm_link_5.dae>
"""
import re
import sys

import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components

EXPECTED_COMPONENTS = 8
EXPECTED_TRIANGLES = 4724


def float_array(text, array_id):
    m = re.search(r'<float_array id="%s"[^>]*>([^<]*)<' % array_id, text)
    return np.array(m.group(1).split(), dtype=float)


def main(src, dst):
    text = open(src, encoding='utf-8').read()
    pos = float_array(text, 'arm_link_5-mesh-positions-array').reshape(-1, 3)
    nor = float_array(text, 'arm_link_5-mesh-normals-array').reshape(-1, 3)
    p = np.array(re.search(r'<p>([^<]*)</p>', text).group(1).split(), dtype=np.int64).reshape(-1, 3, 2)
    tri_pos = p[:, :, 0]

    # 같은 좌표의 점을 하나로 보고 조각을 나눈다
    _, merged = np.unique(np.round(pos, 9), axis=0, return_inverse=True)
    merged = merged.ravel()
    tri = merged[tri_pos]
    n = merged.max() + 1
    edges = np.concatenate([tri[:, [0, 1]], tri[:, [1, 2]]])
    graph = coo_matrix((np.ones(len(edges)), (edges[:, 0], edges[:, 1])), shape=(n, n))
    _, label = connected_components(graph, directed=False)
    tri_label = label[tri[:, 0]]

    # 나사 세트 4개를 감싸는 상자(DAE 좌표, m). 로봇 좌표로는 x 356~367 mm, y ±22~28 mm, z 298~352 mm 부근
    lo, hi = BOX
    remove = []
    for comp in np.unique(tri_label):
        v = pos[tri_pos[tri_label == comp].ravel()]
        if np.all(v.min(0) >= lo) and np.all(v.max(0) <= hi):
            remove.append(comp)
    drop = np.isin(tri_label, remove)
    print(f'조각 {len(np.unique(tri_label))}개 중 지울 조각 {len(remove)}개, 삼각형 {int(drop.sum())}개 / {len(p)}개')
    if len(remove) != EXPECTED_COMPONENTS or drop.sum() != EXPECTED_TRIANGLES:
        print('기대값(조각 8개, 삼각형 4724개)과 달라 파일을 쓰지 않는다')
        return 1

    # 남는 삼각형만 쓰고, 쓰이지 않는 점·법선은 빼서 번호를 다시 매긴다(범위 계산에 남지 않게)
    keep = p[~drop]
    used_pos, new_pos = np.unique(keep[:, :, 0], return_inverse=True)
    used_nor, new_nor = np.unique(keep[:, :, 1], return_inverse=True)
    keep = np.stack([new_pos.reshape(-1, 3), new_nor.reshape(-1, 3)], axis=2)

    def fmt(a):
        return ' '.join(repr(float(x)) for x in a.ravel())

    def replace_array(text, array_id, values, vectors):
        text = re.sub(r'(<float_array id="%s" count=")\d+(">)[^<]*(<)' % array_id,
                      lambda m: f'{m.group(1)}{values.size}{m.group(2)}{fmt(values)}{m.group(3)}', text, count=1)
        source = array_id.replace('-array', '')
        return re.sub(r'(<source id="%s">.*?<accessor [^>]*count=")\d+(")' % source,
                      lambda m: f'{m.group(1)}{vectors}{m.group(2)}', text, count=1, flags=re.S)

    text = replace_array(text, 'arm_link_5-mesh-positions-array', pos[used_pos], len(used_pos))
    text = replace_array(text, 'arm_link_5-mesh-normals-array', nor[used_nor], len(used_nor))
    text = re.sub(r'(<triangles [^>]*count=")\d+(")', lambda m: f'{m.group(1)}{len(keep)}{m.group(2)}', text, count=1)
    text = re.sub(r'<p>[^<]*</p>', lambda m: '<p>' + ' '.join(map(str, keep.ravel())) + '</p>', text, count=1)
    open(dst, 'w', encoding='utf-8').write(text)
    print(f'저장: {dst} (점 {len(pos)}→{len(used_pos)}, 법선 {len(nor)}→{len(used_nor)}, 삼각형 {len(p)}→{len(keep)})')
    return 0


# 나사 세트 점 범위(x 0.3561~0.3666, y ±0.028001, z 0.292~0.345 m)에 0.5 mm 여유
BOX = (np.array([0.3556, -0.0285, 0.2915]), np.array([0.3671, 0.0285, 0.3455]))

if __name__ == '__main__':
    sys.exit(main(sys.argv[1], sys.argv[2]))
