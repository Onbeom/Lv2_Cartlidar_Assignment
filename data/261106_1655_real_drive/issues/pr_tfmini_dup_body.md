## 무엇을
`perception.launch.py` 의 단독 실행용 인자 `use_tfmini` 를 `standalone_tfmini` 로 이름을 바꿈(`requirements.txt` 의 실행 예시도 같이).

## 왜
bringup 의 `use_tfmini:=true`(control_container 에 tfmini_node)가 같은 이름이라 포함한 perception launch 로도 넘어가, perception_container 에도 tfmini_node 가 하나 더 떠서 두 노드가 같은 `/dev/tfmini` 를 열었음(Jetson 실물 11-13 · 11-14).

## 시험
- PC: launch 인자 평가로 bringup `use_tfmini:=true` 일 때 perception 쪽 tfmini_node 없음, `standalone_tfmini:=true` 일 때 있음.
- Jetson 실물(11-14, 정지 상태 bringup 3 번): `/tfmini_node` 가 control_container 에 한 번만 뜸(고치기 전 2 번), `/dev/tfmini 열림 (115200 bps)` · 출력 mm, 노드 죽음 없음.

## 한계
- perception launch 를 단독으로 TFmini 와 함께 띄우던 명령은 `standalone_tfmini:=true` 로 바꿔야 함.
