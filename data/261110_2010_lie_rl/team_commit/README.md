# #192 학습 코드 올리기 (2026-11-11, RL 261111 1)

사용자 결정(11-11):
- 학습 코드만 올립니다. .pt · ONNX는 넣지 않습니다(배포는 #203).
- dev에 없는 릴리스 rl-20261110 의존 코드는 새 이름으로 복사해 올립니다.
- 복사본 확인·수정은 팔 담당(developlcy-oss)에게 Issue로 요청합니다.
- #192에 댓글은 올리지 않습니다.

옛 묶음(r2e 정책 포함, 11-11 오전)은 `old_261111/`에 둡니다.

## 묶음

- `files/`: 넣을 파일입니다. 저장소 뿌리 기준 경로이고 12개입니다.
  - 복사본 3개: `waffle_isaac/lie_base_env.py`, `lie_base_env_cfg.py`, `lie_arm_selfcol.py`
  - 환경 2개: `waffle_isaac/lieupright_env.py`, `lieupright_env_cfg.py`
  - 도구 3개: `lieupright_eval.py`, `lieupright_diag.py`, `phys_report.py`
  - 시작 상태 2개: `lieupright_starts.json`, `lieupright_starts_v2.json`
  - 기록 1개: `기록/1c_누운퍽_세우기_LieUpright.md`
- `dev_register.patch`: `waffle_isaac/__init__.py`와 `agents/rsl_rl_ppo_cfg.py`에 등록 줄만 더합니다(+20줄). origin/dev b6a2ab0에 맞음(B ~ B7 등록, +20줄).
- `copy_files.sh`: 파일을 넣고 패치를 붙입니다.
- `pr_body.md`, `issue_copy_check.md`: PR · Issue 본문입니다.
- 시험(Isaac 모의, 이 묶음을 dev b6a2ab0 트리에 얹어서):
  - r2g2 · B7 · scale 3.0, 512대 seed 777 → 접촉 없는 성공 482/512(작업 트리와 같음), 물리 설정 출력 같음
  - 그전 판(r2e, dev 3fc20fc) 496/512도 같았음

## 명령 (사용자가 직접, PC)

(필수) 최신 dev를 받습니다.

```bash
git -C /home/pa4/git/Lv2_Cartlidar_Assignment fetch origin
```

(필수) 새 작업 폴더와 브랜치를 origin/dev에서 만듭니다.

```bash
git -C /home/pa4/git/Lv2_Cartlidar_Assignment worktree add /home/pa4/git/Lv2_Cartlidar_Assignment_lietrain -b fix/lie-upright-train origin/dev
```

(필수) 파일을 넣고 등록 패치를 붙입니다.

```bash
bash /home/pa4/git/lv2_next_module/261110_2010_lie_rl/team_commit/copy_files.sh /home/pa4/git/Lv2_Cartlidar_Assignment_lietrain
```

(필수) 커밋 1 — 릴리스 판 복사본:

```bash
git -C /home/pa4/git/Lv2_Cartlidar_Assignment_lietrain add workspace/urdf_and_usd/policy/manipulator/waffle_isaac/lie_base_env.py workspace/urdf_and_usd/policy/manipulator/waffle_isaac/lie_base_env_cfg.py workspace/urdf_and_usd/policy/manipulator/waffle_isaac/lie_arm_selfcol.py
```

```bash
git -C /home/pa4/git/Lv2_Cartlidar_Assignment_lietrain commit -m "chore(manipulator RL): 릴리스 rl-20261110 판 sidepick 환경 · 자기 충돌 복사본 (#192)" -m "누운 직육면체 환경이 상속하는 릴리스 판이 dev 에 없어 새 이름 (lie_base_env · lie_base_env_cfg · lie_arm_selfcol) 으로 복사함. 원본 작성자 developlcy-oss. 원본과 다른 줄: lie_arm_selfcol · lie_base_env_cfg 0, lie_base_env diff 로 다른 줄 8 (원본 import 3 줄을 빼고 5 줄로 바꿈: lie_arm_selfcol · URDF_PATHS 가져오기와 URDF_FULL 정의 · lie_base_env_cfg), 머리말 3 줄 제외. dev 에 릴리스 판이 들어오면 지움."
```

(필수) 커밋 2 — 누운 직육면체 환경 · 도구 · 기록 · 등록:

```bash
git -C /home/pa4/git/Lv2_Cartlidar_Assignment_lietrain add workspace/urdf_and_usd/policy/manipulator/waffle_isaac/lieupright_env.py workspace/urdf_and_usd/policy/manipulator/waffle_isaac/lieupright_env_cfg.py workspace/urdf_and_usd/policy/manipulator/waffle_isaac/__init__.py workspace/urdf_and_usd/policy/manipulator/waffle_isaac/agents/rsl_rl_ppo_cfg.py workspace/urdf_and_usd/policy/manipulator/lieupright_eval.py workspace/urdf_and_usd/policy/manipulator/lieupright_diag.py workspace/urdf_and_usd/policy/manipulator/phys_report.py workspace/urdf_and_usd/policy/manipulator/lieupright_starts.json workspace/urdf_and_usd/policy/manipulator/lieupright_starts_v2.json "workspace/urdf_and_usd/policy/manipulator/기록/1c_누운퍽_세우기_LieUpright.md"
```

```bash
git -C /home/pa4/git/Lv2_Cartlidar_Assignment_lietrain commit -m "feat(manipulator RL): 누운 직육면체 세워 놓기 학습 환경 Waffle-LieUpright (#192)" -m "상자 위 누운 직육면체를 쥐고 들어 손 안에서 세운 뒤 세울 자리에 놓는 환경 (B ~ B7 단계, 정책 r2g2 는 B7: 관측을 미션 노드와 같게, 관절 속도 비례 축소 · 노드 놓음 규칙 포함) 과 평가 · 진단 도구, 시작 상태, 기록. 퍽 질량은 이 환경에서만 실측값 (13.5 · 17.5 g). Isaac 모의 r2g2: v 3.0 팔-상자 접촉 없는 성공 95.7 %, 미션 노드 순서 모의 정책 단계 98.9 %."
```

(필수) 올립니다.

```bash
git -C /home/pa4/git/Lv2_Cartlidar_Assignment_lietrain push -u origin fix/lie-upright-train
```

(필수) PR을 만듭니다.

```bash
gh pr create --repo developlcy-oss/Lv2_Cartlidar_Assignment --base dev --head fix/lie-upright-train --title "feat(manipulator RL): 누운 직육면체 세워 놓기 학습 코드 (#192)" --body-file /home/pa4/git/lv2_next_module/261110_2010_lie_rl/team_commit/pr_body.md
```

(필수) Issue 본문에 PR 번호를 넣습니다. 바로 위 `gh pr create`가 출력한 주소 끝의 번호로 `<번호>`를 바꿔서 실행합니다.

```bash
sed -i 's/PR_NUMBER/<번호>/g' /home/pa4/git/lv2_next_module/261110_2010_lie_rl/team_commit/issue_copy_check.md
```

(필수) 복사본 확인 Issue를 팔 담당에게 올립니다.

```bash
gh issue create --repo developlcy-oss/Lv2_Cartlidar_Assignment --title "릴리스 rl-20261110 판 복사본 확인 · 수정 요청 (#192 학습 코드 PR)" --assignee developlcy-oss --body-file /home/pa4/git/lv2_next_module/261110_2010_lie_rl/team_commit/issue_copy_check.md
```

(선택 · 권장) PR이 합쳐진 뒤 작업 폴더를 지웁니다. 디스크를 차지하지 않게 하려는 것입니다.

```bash
git -C /home/pa4/git/Lv2_Cartlidar_Assignment worktree remove /home/pa4/git/Lv2_Cartlidar_Assignment_lietrain
```
