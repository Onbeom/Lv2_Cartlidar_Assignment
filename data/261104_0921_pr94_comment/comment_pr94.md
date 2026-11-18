인지 쪽 출력 형식(survey 완료 파일 12칸 · cuboid→box, revisit 응답 JSON, SignalSequence cycles · CONFIRMED 규칙, TaskMarker 필드)과 맞는 것 확인했습니다. 몇 가지만 봐 주세요.

1. survey.on_unconfirmed: registered 일 때, 안내대로 계획기를 pucks_file:=/tmp/mission_log/pucks_survey.yaml 로 띄우면 survey.pucks_out 과 같은 파일이라 이번 측량이 실패해도 지난 실행의 측량 결과로 계획합니다(파일이 없으면 plan 실패). survey_start 때 pucks_out 을 지우거나, registered 용 등록표는 다른 경로로 두는 게 안전해 보입니다.
2. MISMATCH 를 바로 멈추지 않고 다음 묶음을 기다리는 것은 인지 규칙(같은 순서 2묶음이면 CONFIRMED, A·B·A 면 A 확정)과 맞습니다. 다만 원래 주석이 발제 '오류 기록 후 안전 정지'였는데, 반복 묶음으로 확정해도 되는지 발제 기준 확인 부탁드립니다.
3. survey.poses 가 mission_planner approach_poses.yaml stage 값을 복사한 것이라, 한쪽만 고치면 어긋납니다. 같은 파일에서 읽거나 주석에 함께 고칠 곳을 적어 두면 좋겠습니다.
4. 시험이 mission 빌드뿐이라 writePucksFromSurvey(완료 파일 → 등록표) 변환은 단위 시험 하나 있으면 좋겠습니다. dev 가 앞서 있어 병합 전 dev 반영 뒤 빌드도 한 번 부탁드립니다.
