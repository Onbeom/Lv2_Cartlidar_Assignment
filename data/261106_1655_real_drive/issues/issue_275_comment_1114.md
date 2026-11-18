11-14 10:20 에도 같은 증상 (실물 Jetson, 정지 상태, `use_planner:=false`, 팔 HOLD):
- 1 · 2 번째 bringup 컨트롤러 0/4 → 3 번째에 4/4.
- 실패한 두 번 모두 하드웨어는 정상(`Successful 'activate' of hardware 'OpenCRSystem'`), spawner 3 개가 `Failed to acquire lock in 20 seconds. Attempt 1 of 5 failed` (arm_controller · diff_drive_controller · imu_sensor_broadcaster).
- 기록: Jetson `~/real_drive/p7a_out/1114_1020/` (`bringup_fail_1.log` · `bringup_fail_2.log` · `summary.txt`).
