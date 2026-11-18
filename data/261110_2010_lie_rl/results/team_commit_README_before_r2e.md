# 팀 저장소 반영 준비 — (a) 우리 파일만 추가 (실행하지 않음, 사용자 결정 대기)

2026-11-11 03:4x 준비. 아래 명령은 **사용자가 직접** 실행합니다 (커밋 · push · PR 은 사용자 몫). 명령은 모두 PC 에서 실행합니다.

## 먼저 알아 둘 것

- **이 (a) 묶음만으로는 dev 에서 돌지 않습니다.** 우리 환경 `WaffleLieUprightEnv` 는 팀원 릴리스 rl-20261110 (zip, `261110_1930_arm_rl_review/src_v2/`) 의 `WaffleSidePickEnv` · `WaffleSidePick45EnvCfg` 를 상속합니다. origin/dev (7cb49b0) 의 `sidepick_env.py` · `sidepick_env_cfg.py` 는 릴리스 판과 418 · 214 줄 다르고 `WaffleSidePick45EnvCfg` 가 없습니다 (`_perc_filter` 등 보조 함수도 없음).
- E1 재현 (`chain45_play.py`, n5 정책 `sidepick45_latest.pt`) 도 dev 에 없습니다.
- 그래서 (a) 는 '파일 보관 + 위치 문의' 용도입니다. 돌게 하려면 릴리스 판 반영 (팔 담당 developlcy-oss 파트) 이 먼저 필요합니다 → #192 댓글 초안 (`comment_192.md`) 으로 묻습니다.
- 패치 `dev_register.patch` 는 origin/dev 판 `waffle_isaac/__init__.py` (등록 3 줄) · `agents/rsl_rl_ppo_cfg.py` (`WaffleLieUprightPPORunnerCfg` 추가) 에 맞춰 만들었고, `patch --dry-run` 으로 맞는 것을 확인했습니다.

## 넣을 파일 (팀 저장소 `workspace/urdf_and_usd/policy/manipulator/` 아래)

| 경로 | 크기 (바이트) | 내용 |
|---|---|---|
| `waffle_isaac/lieupright_env.py` | 30 727 | P1 세우기 환경 |
| `waffle_isaac/lieupright_env_cfg.py` | 6 718 | 설정 (B2 · B3 포함) |
| `waffle_isaac/__init__.py` · `agents/rsl_rl_ppo_cfg.py` | 패치 (+16 줄) | 등록 3 개 · PPO 설정 |
| `lieupright_eval.py` | 10 160 | 512 대 평가 (`--chain_phys` · `--dump_e1`) |
| `lieupright_diag.py` | 3 953 | 쥠 판정 진단 |
| `phys_report.py` | 2 535 | 물리 설정 출력 |
| `lieupright_starts.json` | 305 572 | 시작 상태 376 개 |
| `lieupright_starts_v2.json` | 755 114 | 시작 상태 932 개 (r2c · r2d 학습에 씀) |
| `policies/lieupright_r2d.onnx` | 226 965 | 정책 (한 파일) |
| `policies/lieupright_r2d.pt` | 229 524 | 정책 TorchScript |
| `기록/1c_누운퍽_세우기_LieUpright.md` | (handoff README) | 관측 · 행동 · 끝 상태 · 결과 · 한계 |

합계 약 1.6 MB. 가장 큰 것은 시작 상태 json (0.76 MB).

## 명령 (사용자가 직접, PC)

(필수) 새 작업 폴더와 브랜치를 origin/dev 에서 만듭니다.

```bash
git -C /home/pa4/git/Lv2_Cartlidar_Assignment fetch origin
```

```bash
git -C /home/pa4/git/Lv2_Cartlidar_Assignment worktree add /home/pa4/git/Lv2_Cartlidar_Assignment_liebox -b fix/lie-box-regrasp origin/dev
```

(필수) 파일을 넣습니다.

```bash
bash /home/pa4/git/lv2_next_module/261110_2010_lie_rl/team_commit/copy_files.sh /home/pa4/git/Lv2_Cartlidar_Assignment_liebox
```

(필수) 커밋 1 — 환경 · 도구:

```bash
git -C /home/pa4/git/Lv2_Cartlidar_Assignment_liebox add workspace/urdf_and_usd/policy/manipulator/waffle_isaac/lieupright_env.py workspace/urdf_and_usd/policy/manipulator/waffle_isaac/lieupright_env_cfg.py workspace/urdf_and_usd/policy/manipulator/waffle_isaac/__init__.py workspace/urdf_and_usd/policy/manipulator/waffle_isaac/agents/rsl_rl_ppo_cfg.py workspace/urdf_and_usd/policy/manipulator/lieupright_eval.py workspace/urdf_and_usd/policy/manipulator/lieupright_diag.py workspace/urdf_and_usd/policy/manipulator/phys_report.py workspace/urdf_and_usd/policy/manipulator/lieupright_starts.json workspace/urdf_and_usd/policy/manipulator/lieupright_starts_v2.json
```

```bash
git -C /home/pa4/git/Lv2_Cartlidar_Assignment_liebox commit -m "feat(manipulator RL): 누운 직육면체 세워 놓기 환경 Waffle-LieUpright (#192)" -m "상자 위 누운 직육면체를 쥐고 들어 손 안에서 세운 뒤 세울 자리에 놓고 손을 빼는 P1 환경과 평가 · 진단 도구를 더함. 팀원 릴리스 rl-20261110 의 SidePick45 환경을 상속하므로 dev 에서는 그 판이 들어온 뒤에 돈다."
```

(필수) 커밋 2 — 정책 · 기록:

```bash
git -C /home/pa4/git/Lv2_Cartlidar_Assignment_liebox add workspace/urdf_and_usd/policy/manipulator/policies/lieupright_r2d.onnx workspace/urdf_and_usd/policy/manipulator/policies/lieupright_r2d.pt "workspace/urdf_and_usd/policy/manipulator/기록/1c_누운퍽_세우기_LieUpright.md"
```

```bash
git -C /home/pa4/git/Lv2_Cartlidar_Assignment_liebox commit -m "feat(manipulator RL): 누운 직육면체 세워 놓기 정책 r2d 와 기록 (#192)" -m "Isaac 모의 512 대: 팔-상자 접촉 없는 세워 놓기 93.0 ~ 95.3 %, 이어서 n5 다시 쥐고 들기 90.2 % (각본 85.4 %). 관측 45 · 행동 6 · 30 Hz, 실물 미확인."
```

(필수) push · PR 은 #192 댓글 답을 본 뒤에 정합니다 (`gh pr create` 명령은 그때 줌).

정리 (선택 · 권장, 이 작업 폴더를 다 쓴 뒤): `git -C /home/pa4/git/Lv2_Cartlidar_Assignment worktree remove /home/pa4/git/Lv2_Cartlidar_Assignment_liebox`
