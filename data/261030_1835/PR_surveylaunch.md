## 무엇
perception_survey의 `colcon test`가 pytest 시험을 찾게 했고, perception.launch.py가 B0 노드를 `puck_survey.yaml`(태그 문턱 0.4)로 띄웁니다(`use_survey`·`survey_output` 인자).

## 왜
`colcon test`에서 시험 0개로 실패했고, B0 노드를 띄우는 launch가 없어 운용 설정을 줄 곳이 없었습니다.

## 시험
`colcon test` 12/12. launch 실행 확인: 노드 4개 기동, 문턱 0.4·완료 경로·map 적용, 미완료 상태의 finish 거부.

## 한계
PR #21 위에 쌓았습니다(#21 먼저 병합). 0.4는 모의 검증값이며 실물 최적값이 아닙니다.
