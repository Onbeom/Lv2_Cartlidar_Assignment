## 요청 (인지 파트 → Controller, `workspace/docs/interfaces.md`)

`interfaces.md` 에 인지 쪽 새 인터페이스가 빠져 있습니다(지금은 `/perception/result` · `/perception/set_mode` 등만). 표에 아래를 넣어 주세요. `robot_interfaces/names.hpp` 에는 `PERCEPTION_PUCKS = "/perception/pucks"` 가 이미 있습니다.

| 토픽 · 서비스 | 형식 | QoS | 보내는 쪽 | 받는 쪽 |
|---|---|---|---|---|
| `/perception/pucks` | robot_interfaces/PuckArray | `qos::result()`(`/perception/result` 와 같음) | perception_node | perception_survey (mission 은 필요하면) |
| `/perception/puck_survey` | std_msgs/String(JSON) | Reliable · TRANSIENT_LOCAL · depth 1 | perception_survey | mission |
| `/perception/survey/start` · `/reset` · `/finish` | std_srvs/Trigger | 서비스 | mission | perception_survey |

JSON 필드: `slots[]`(box, side, x_mm, y_mm, color, shape, pose, id …), `complete`, `errors`, `frame`(map, mm, 발제 원점). 인지 PR #21 · #24 와 함께 맞춰 주시면 됩니다.
