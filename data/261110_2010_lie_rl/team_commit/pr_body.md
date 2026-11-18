## 무엇
- 누운 직육면체 세워 놓기 학습 환경 `Waffle-LieUpright` 단계 B ~ B7과 평가·진단 도구, 시작 상태, 기록을 `policy/manipulator/` 아래 새 파일로 넣었습니다(#192).
- 지금 정책 r2g2는 `Waffle-LieUprightB7-Direct`로 학습했습니다. 관측을 미션 노드와 같게 했습니다: 퍽 · 긴 축 · 옆 퍽은 시작 값으로 고정, 상자 · D는 잰 값, 쥠은 손가락만. 노드 관절 속도 비례 축소와 놓음 규칙도 학습에 넣었습니다.
- 기존 파일은 고치지 않았고, `waffle_isaac/__init__.py`와 `agents/rsl_rl_ppo_cfg.py`에 등록 줄만 더했습니다(+20줄).
- .pt · ONNX는 넣지 않습니다. 배포 모델을 r2e → r2g2로 바꾸는 일은 노드 쪽(미션 파트)에서 따로 합니다.

## 왜 복사본이 있나
- 이 환경은 릴리스 rl-20261110 판 `sidepick_env.py` · `sidepick_env_cfg.py` · `arm_selfcol.py`를 상속합니다. 그런데 dev 판과 다릅니다(sidepick_env 418줄 · cfg 214줄 차이, arm_selfcol 없음).
- 그래서 `lie_base_env.py` · `lie_base_env_cfg.py` · `lie_arm_selfcol.py`라는 새 이름으로 복사했습니다. 원본 작성자는 developlcy-oss(LCY)입니다.
- 원본과의 차이(머리말 3줄 제외):

| 복사본 | 원본 md5 | 다른 줄 |
|---|---|---|
| lie_arm_selfcol.py | 45f55ebbc933 | 0 |
| lie_base_env_cfg.py | b9f68b46b459 | 0 |
| lie_base_env.py | c4b52124423c | diff 로 다른 줄 8 (원본 import 3 줄을 빼고 5 줄로 바꿈: lie_arm_selfcol · URDF_PATHS 가져오기와 URDF_FULL 정의 · lie_base_env_cfg), 머리말 3 줄 제외 |

- 퍽 질량은 이 환경 설정에서만 실측값(원기둥 13.5 g, 직육면체 17.5 g)으로 덮어씁니다. dev `dice2_cfg.py`는 45·55 g입니다. `URDF_FULL`은 팀 설정 `URDF_PATHS["full"]`을 씁니다.
- dev에 릴리스 판이 들어오면 복사본 셋을 지우고 import를 원래 이름으로 되돌립니다. 확인·수정은 Issue로 따로 요청합니다.

## 시험 (Isaac 모의, 이 PR 트리 그대로, dev b6a2ab0)
- `lieupright_eval.py --cfg B7 --policy <r2g2> --num_envs 512 --seed 777 --nb_prob 0.5 --vmax 3.0 --vmode scale`
  - 팔-상자 접촉 없는 성공 482/512 (94.1 %)
  - 작업 트리 결과와 같고, 물리 설정 출력도 줄마다 같습니다.
- 작업 트리에서 낸 r2g2 판정(3 seed × 512):
  - B7 관측: v 2.0 93.6 % · v 3.0 95.7 %
  - v 4.0은 90.3 %입니다. 노드 상한이 3.0이라 기록 항목으로 두었습니다.
  - 미션 노드 순서 모의의 정책 단계: 98.9 %

## 한계
- 모의 결과만 있습니다. 실물 손가락 대응(#201)은 아직 재지 않았습니다.
- 노드 관절 속도 상한을 3.0 rad/s 넘게 올리려면 다시 평가해야 합니다.
