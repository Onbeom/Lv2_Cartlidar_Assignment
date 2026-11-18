@kanichong @developlcy-oss 위 계산대로 mission 쪽을 #247 로 올렸습니다(모의만, `start_box.enabled` 기본 꺼짐). #247 을 합치면 #229 도 함께 합쳐지므로, #229 를 정한 뒤 합쳐 주세요.

한 가지 더 알려 드립니다.
- 정사각 윗면만으로는 ±45 를 못 가르고, 틀리면 퍽 자리가 57 mm 틀립니다. target_look 의 match_distance 안인지는 확인하지 않았습니다.
- 그래서 잰 돌림이 |40°| 이상이면 mission_log 에 '부호 모호' 경고를 남깁니다(멈추지는 않음).
