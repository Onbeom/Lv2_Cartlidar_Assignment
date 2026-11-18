PR: #215

#192 누운 직육면체 학습 코드 PR #215(브랜치 `fix/lie-upright-train`)에 릴리스 rl-20261110 판 파일 셋의 복사본을 넣었습니다. 확인해 주시고, 문제가 있으면 고쳐 주세요.

## 넣은 복사본 (`workspace/urdf_and_usd/policy/manipulator/waffle_isaac/`)

| 복사본 | 원본(릴리스 rl-20261110) | 원본 md5 | 원본과 다른 줄 (머리말 3줄 제외) |
|---|---|---|---|
| lie_arm_selfcol.py | arm_selfcol.py | 45f55ebbc933 | 0 |
| lie_base_env_cfg.py | sidepick_env_cfg.py | b9f68b46b459 | 0 |
| lie_base_env.py | sidepick_env.py | c4b52124423c | diff 로 다른 줄 8 (원본 import 3 줄을 빼고 5 줄로 바꿈: lie_arm_selfcol · URDF_PATHS 가져오기와 URDF_FULL 정의 · lie_base_env_cfg), 머리말 3 줄 제외 |

- 왜: 누운 직육면체 환경(`lieupright_env.py`)이 릴리스 판을 상속하는데, dev 판과 달랐습니다(sidepick_env 418줄 · cfg 214줄 차이, arm_selfcol 없음). '기존 환경 파일은 그대로'라는 조건을 지키려고 새 이름으로 복사했습니다.
- 퍽 질량은 누운 직육면체 환경 설정에서만 실측값(13.5 · 17.5 g)으로 덮어썼습니다. dev `dice2_cfg.py`(45 · 55 g)는 그대로입니다.
- 이 PR 트리에서 정책 r2g2를 평가했고(Isaac 모의 512대, seed 777), 작업 트리와 같은 482/512가 나왔습니다. 물리 설정 출력도 같습니다.

## 부탁
1. 복사본 내용이 맞는지 봐 주세요.
2. 릴리스 판을 dev에 올리실 계획이 있으면 알려 주세요. 올라오면 복사본 셋을 지우고 import를 원래 이름으로 되돌리겠습니다.
3. 문제가 있으면 복사본을 고쳐 주셔도 됩니다.
