-- V4: map→odom(·odom→base_link)을 '지금 외삽 자세' 대신 마지막 스캔 시각의 자세 · 그 시각 stamp 로 냄(cartographer_ros 기본 true → false).
-- node.cpp: use_pose_extrapolator 가 true 면 tracking_to_local = ExtrapolatePose(now) 인데 published_to_tracking 은 스캔 시각 TF 라 어긋남.
options.use_pose_extrapolator = false
