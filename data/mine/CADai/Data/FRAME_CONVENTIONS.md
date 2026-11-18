# 좌표와 검증 결과를 읽는 기준

ROBOT_REVIEW_PENDING.json의 link_frame_world_mm, com_link_mm, axis_child는 원래 CAD/검산 모델의 링크 프레임이다. Fusion2URDF는 관절 축을 표현하기 위해 일부 링크 프레임을 회전시켰다. 그 원래 local 좌표를 현재 URDF local 좌표로 바로 대입하지 않는다.

현재 URDF 프레임/관성 원점/관성6값은 URDF_LINK_FRAMES_AND_INERTIALS_G_MM.json을 사용한다. JOINTS_COMPLETE_MM_RAD.json은 현재 URDF의 parent-relative origin/RPY/child-axis와 제한값이다. LINKS_G_MM.csv의 world CoM/관성은 원래 engineering base_link 월드 기준이다. base_footprint 좌표로 보면 모든 home 좌표의Z에6.7499816754mm가 추가된다. 회전 변환에 따른 관성 성분 차이를 단위 오류로 오해하지 않는다.

이식 FK와 독립 CAD 기준 FK를 비교할 때는 먼저 두 home 링크 프레임 사이 고정변환을 적용했다. 402자세/14물리 링크의 world CoM 최대차이는0.0001657994mm였다. 새 물리 모델의 실측 검증이나 실제 파지 검증을 뜻하지 않는다.

원본 시연402프레임 중159프레임이 임시 bounded 관절 범위를 벗어난다. 원본은 보존했으며 임의 clamp하지 않았다. 다음 세션은 실제 범위를 확인하거나 경로를 다시 계획해야 한다.

충돌 STL은490개 폐곡면 convex 조각이고 base_link에는424개가 있다. 부피·convex·폐곡면·정점수와 단위를 검사했다. 부동소수 STL 저장으로 생긴 퇴화 면38개는0.0001~0.01mm 격자 범위 안에서 재생성했고 실제 적용값은 convex_mesh_repairs.json에 기록했다. 저장공간의 CAD-확인된 퍽 수용 열에 convex 근사가 침범하는 부분을 collision에서만 제거했으며 재저장 후30×30×240mm 열과 양의 체적 겹침이 없는지 다시 검사했다. 이는 움직이는 집게·팔의 연속 충돌 검사나 공차 보증이 아니다.

URDF 수정 전 파일은 기존 submit/urdf/backup_before_simulation_261021에 보존했다. 일반 exporter 또는 구형 finalizer를 재실행하면 새 설정이 사라질 수 있으므로 원본과 차이를 확인하고 다시 적용해야 한다. 모터 stall effort, 임시 한계·힘·속도·감쇠, 센서 광학/스캔 기준, 실제 ROS/Isaac 런타임은06_SIMULATION_CHANGES.md와 함께 검증한다.
