"""FDM 출력 부품의 실제 질량 비율(채움률 반영) 추정. 실측 없음 — 가정 값이다.

가정(사용자 2026-10-27 "채움률은 100 %가 아닐 거야, 적절히 유추"): 슬라이서 기본값에 가까운 설정.
  - 껍질 두께 SHELL_MM = 0.8 mm(0.4 mm 노즐 벽 2줄, 윗면·바닥 0.2 mm × 4층과 비슷), 이 부분은 100 %
  - 안쪽 채움 INFILL = 15 %(튜터 확인 2026-10-27, Bambu Studio · PrusaSlicer 기본값과도 같음)
  - 껍질 부피 ≈ 표면적 × 껍질 두께(얇은 부분은 부품 부피를 넘지 않게 자름)
비율 f = (껍질 + 채움 × (부피 − 껍질)) / 부피. 질량은 부품 안에 고르게 퍼져 있다고 보고 무게중심·관성 모양은 꽉 찬 부품과 같게 둔다.
"""
SHELL_MM = 0.8
INFILL = 0.15


def fill_fraction(mesh_mm):
    v = abs(float(mesh_mm.volume))
    shell = min(v, float(mesh_mm.area) * SHELL_MM)
    return (shell + INFILL * (v - shell)) / v
