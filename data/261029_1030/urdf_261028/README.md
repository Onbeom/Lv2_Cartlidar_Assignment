# urdf_261028 (waffle_4wd 4륜 + RM-X52 팔)

| 폴더 | 쓰는 곳 | 시작 |
|---|---|---|
| `waffle_4wd_description_261028/` | ROS 2 (RViz) · Gazebo Sim. 작업 공간 `src/` 에 넣고 `colcon build` | 그 폴더 `README.md` |
| `isaac/` (2026-10-29 추가) | Isaac Sim 6.1 / Isaac Lab 3.0. USD 2개 (가벼운 · 정밀), 로봇 · 센서 설정 | 그 폴더 `README.md`, `./run.sh check` |

두 폴더는 같은 URDF 에서 나왔다. `isaac/urdf/*.urdf` 가 `waffle_4wd_description_261028/meshes/` 를 상대 경로로 쓰므로 압축은 이 구조 그대로 풀 것.

AI 도구 (Claude Code · Codex · Gemini CLI) 를 쓸 때는 이 폴더에서 열 것: `AGENTS.md` (본문) 를 `CLAUDE.md` · `GEMINI.md` 가 가져온다. 틀리기 쉬운 사실 (바퀴 간격 0.317 = 바퀴 폭 가운데, 벨트 1:1.5, 손가락 같은 명령, D435 깊이는 받은 값 그대로 frame 기준) 이 정리돼 있다.
