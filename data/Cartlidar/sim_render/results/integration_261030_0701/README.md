# B0·A1와 YOLO/R6 후보 개인 통합

팀 폴더 반영 전 개인 소스만 통합한다. 원본 261029_1408/src/{perception,robot_interfaces}와 A1 source_v12, B0 261028_1459/src/perception_survey를 사용했다. 공통 기준 커밋0e2438a61f119a12b19f3f67a12c965613fcb2a4로 A1 변경10파일을3방향 병합했고 텍스트 충돌0. 자동 병합 성공은 운용 검증 완료를 뜻하지 않는다. 파일별 출처는 261030_0701/merge_report.json.

| 단계 | 예상(실행 전) | 기준·이유 | 결과 |
|---|---|---|---|
| 독립 빌드·기존 단위 회귀 | B0/A1와 R6/YOLO 클래스/TFmini 후보가 함께 빌드되고 단위 전체 통과 | 상대 코드의 추가 필드·메시지·정책이 누락되지 않았는지 검사 | 미실행 |
| 새 모델 A1 영상 통합 재생 | 고정 후보와 같은 P 기준 충족·다른 퍽0 | 공용 node.cpp 병합이 인지 동작을 바꾸지 않는지 확인 | 미실행 |

실물/TensorRT 엔진 운용/모터/Jetson 검증은 아니다. 팀 반영은 사용자 명시 허가 후 별도 fix/ 브랜치, commit/push/PR은 사용자 명령 제공.


| 영향 범위 회귀 · 실행 전 | 예상 | 기준·이유 |
|---|---|---|
| 기존모델 R4 D435 저장120장에 통합 후보 적용 | 크기 초과 연결부 재분리 후 기존 정상 검출 보존 | 기존 shelf≤1m 272/276·floor50/50 이상, 색100%, 대응 없는 결과≤1건(기존1). 원래 R4 기준≥95%·색≥98%도 함께 보존. 새모델 판정으로 쓰지 않는 회귀 검사 |


## 2026-10-30 인지 코드 최종 확인 · 팀 반영 승인 전

사용자 결정: 같은 물체의 모든 면에 동일 태그 ID. 변경 통보 전까지 유지. 승인 설정의 ‘나 대신 승인’과 무관하게 팀 반영은 명시 허가 필요. 실물 SSH/보정은 보류.

| 항목 | 결과 |
|---|---|
| 새모델 A1 v12 설계 회귀 | P1 12/12·P2 210/212=99.06%·다른퍽0·가림48/48무효·복귀12/12 |
| v11이 실패했던 기존 별도시야를 v12로 재검증(이제 설계자료) | P1 12/12·P2 213/216=98.61%·다른퍽0·가림40/40·복귀10/10 |
| 고정 v12의 새 별도시야 rng20261229(같은6월드) | P1 12/12·P2 210/212=99.06%·다른퍽0·가림52/52·복귀12/12·평가불가0. validation_v12/replay_isolated/perception_summary.json |
| YOLO/R6후보+B0/A1 개인 통합 | 빌드3패키지 완료·C++139/139·B0단위8/8·A1 설계 P2 210/212=99.06%, 나머지 동일 |
| 통합후 기존모델 R4 D435120장 회귀 | 선반≤1m 272/276→275/276=99.64%·바닥50/50·색100%·대응없는검출1건 유지·미응답0. 이전모델 회귀이지 새모델 R4 완료 아님 |
| TF/입력 실패 처리 | 실제0.133333s지연 P2 17/17·전체23응답·대기로그7. 10s지연은 유효정렬0/23(응답4), 깊이전부0은 유효정렬0/23(응답23). 정상성공률과 분리 |

새 별도시야 잔여2프레임: w2 어두움 k9/k16의 중심오차 .06302/.06385로.05 초과. 같은 목표이며 오인0. 통과 문턱95%를 충족하나 실패 프레임을 없애지 않는다. seed5 계획puck_01/실제선택puck_00의2장면도 P에서 제외하지 않았다. 원본 J와 계획불일치 목록은 각 summary.json에 보존. 기존 v11 별도시야의 초기목표 없음11/12 실패도 보존.

처음 지연도구의 wall/sim시각 혼용 결과 및 첫 새별도시야 domain90 충돌 실행은 판정에서 제외하고 로그를 보존했다. 수정 도구는 self.now_s 기반 지연·도메인별flock으로 격리하며 최종 별도시야는 domain92, 통합은91, R4회귀는93. 모든 시험 종료. GPU 작업은 flock/30분 상한, 재생도30분 상한 유지.

개인 통합 소스261030_0701/src/{perception,perception_survey,robot_interfaces}, 출처/병합기준 merge_report.json, 파일SHA results/integration_261030_0701/source_sha256.json. 팀HEAD 기준 반영검토37파일(인지30·B0등록부7, 메시지변경0), team_preview.patch/manifest 준비. 이 파일들은 개인 폴더의 검토 자료이며 팀 적용하지 않았다. commit/push/PR도 없음.

다음: 사용자에게 B0·통합인지코드 반영 범위와 기능별커밋/PR제목·간단본문을 제시하고 명시 승인받기. 승인 전 팀 파일/브랜치 변경 금지. 승인 후에도 commit/push/PR은 사용자가 실행할 git -C/gh 명령만 제공. 실물 K/D·Jetson TensorRT·전체 R1/R2/R3/R6 새모델 재판정·전체로봇 운용은 미완료이며 이 A1결과로 대체하지 않는다.

사진 photo/261030_새모델_A1_인지검증.png, 결과 results/a1_new_model_261029/ 및 results/integration_261030_0701/. 검토 codex/B0_A1_반영검토_2026-10-30.md.


반영 범위 최종 축소: 위37파일은 상대후보까지 포함한 통합 호환성 참고용이다. 실제 사용자에게 승인 요청할 최소 범위는 **23파일(B0 7·A1/인지16), 메시지/mission/제어0**이며 team_minimal.patch/manifest로 구분한다. B0 fix/perception-puck-survey와 A1 fix/perception-arm-target의 독립 PR·간단 본문은 codex/B0_A1_반영검토_2026-10-30.md에 준비했다. 팀 실제 적용·브랜치 생성·commit/push/PR은 아직 없다.


## 사용자 반영 승인 및 B0 적용 · 2026-10-30

사용자가 승인 요청에 **“B0와 A1 반영 승인”**으로 명시 응답했다. 이후 팀 파일 반영과 fix/ 브랜치 생성은 승인 범위 내에서 진행하며, commit/push/PR 실행은 계속 사용자가 직접 한다.

원격fetch후 origin/dev=619bd6e90348515fe64cdc0a2b519d495329dde4 확인. feat/perception-barcode PR#5는 이미 dev에 병합됨을 읽기 조회로 확인했다. 최신dev에 이전 팔추적/퍽병합 일부가 있으므로 구7d40a52 기준 패치를 통째 덮지 않는다.

- B0: 팀 주 작업폴더 브랜치를 fix/perception-puck-survey로 만들고 최신origin/dev를 기반으로9파일 추가·스테이징 완료. 빈 __init__.py·resource표식2개가 이전 파일수 집계에서 빠졌음을 수정, 기능추가 아님. core.py는.gitignore core.*에 걸려 해당파일만 포함한9파일을 정확히 강제스테이징. 빌드2패키지·단위8/8·staged whitespace검사 통과. 원본과SHA일치. 기존 미추적문서/zip4개 모두 보존(최신dev에서는ignore). 아직commit/push/PR없음.
- A1: 최신dev와고정v12를0e2438a공통기준으로개인261030_0701/team_candidate에3방향병합,충돌0. 이미반영된부분제외실제변경12파일. 독립빌드2패키지·단위127/127통과. 실행cpp는검증v12와동일하며config는최신프레임주석을보존. a1_latest_dev.patch/manifest준비,팀실제적용은B0커밋후별도fix/perception-arm-target에서진행. 승인재요청불필요.
- 최종대상은현재dev대비B0 9+A1 12=21파일. 초기23집계는구dev기준이고빈파일2개누락이있었다. 실제새기능범위확대없이완전패키지표식포함및이미병합된변경중복제외로정리했다.

B0 실제staged패치 results/integration_261030_0701/b0_team_staged.patch, SHA b0_team_applied_sha256.json. A1최신후보패치 같은폴더a1_latest_dev.patch,기준/파일목록a1_latest_dev_manifest.json. 37파일전체통합비교와초기team_minimal.patch는이제제출용이아닌과거검토자료.

다음 사람 단계: codex/B0_제출명령_2026-10-30.md의B0커밋·푸시·PR명령을사용자가실행. 완료확인후A1별도브랜치반영및커밋·푸시·PR명령제공. 실물보정은여전히보류. 새승인기다림아님,사용자직접git처리기다림이다.


### A1 실제 팀 소스 확인 · 실행 전
사용자가 B0 PR#18 생성을 알려줬고 d927a4e/원격PR OPEN·dev기준을 읽기 확인·채팅에 연결했다. B0 작업트리청결 확인 후 origin/dev=619bd6e에서 fix/perception-arm-target 생성, 승인된 A1패치12파일 적용. 후보manifest SHA12/12 일치·다른 파일변경0. 실제 팀 경로를 입력으로 새 개인빌드 폴더에서 빌드·단위127/127과 staged범위/공백 확인 예정. GPU 재렌더 없음. commit/push/PR은 사용자 실행.


## B0 PR 확인·A1 실제 팀 반영 완료 · 2026-10-30

사용자가 하단 터미널에서 B0 commit/push/PR 생성했다고 알림. 터미널/로컬/원격 읽기 확인: B0 d927a4e4d77cdeb43923074e0c07e3a1babef3aa, PR#18 OPEN·dev기준·URL https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/18 . 해당 PR을 이 채팅 첨부로 연결. B0 작업트리 청결·원격커밋일치 확인후 다음 승인범위를 진행.

현재 팀 주폴더 브랜치 **fix/perception-arm-target**, 기준 origin/dev=619bd6e90348515fe64cdc0a2b519d495329dde4. A1 패치12파일만 실제 반영·스테이징 완료. B0커밋은이독립브랜치에포함하지않는다. 후보manifest SHA12/12일치·추가변경0·미스테이징0·staged공백검사통과. 실제팀경로를입력으로개인 team_verify_a1/에서새빌드2패키지44.3초·C++단위127/127통과. 로그results/integration_261030_0701/a1_team_build.log,a1_team_gtest.txt,실제staged패치a1_team_staged.patch. GPU재렌더/실물작업없음.

다음사람단계: codex/A1_제출명령_2026-10-30.md의commit/push/PR명령을사용자가직접실행. A1현재커밋/푸시/PR없음. 추가승인질문필요없음. A1 PR생성알림후주소/커밋확인·채팅첨부·STATUS갱신. 실물보정은사용자지시로계속보류. YOLO/TFmini/R2/R3/R6별도후보전체를이승인으로이식하지않는다.
