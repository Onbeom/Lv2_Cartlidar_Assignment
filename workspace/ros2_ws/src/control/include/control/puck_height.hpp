// 상자 위 퍽 중심 높이 (target_look 값, manipulator_node target_look.box_z*, #279)
//
// 왜: target_look 은 팔 카메라로 잰 퍽 중심 (x · y · z) 을 집기에 넘긴다. 카메라 높이를 팔 정기구학 (URDF + 영점) 으로 내는데,
//     실물에서 그 높이가 사진으로 잰 값과 자세마다 8 ~ 22 mm 다르고 (#279), 11-12 실물 집기 실패 원인 중 하나가 높이 치우침
//     +2.5 cm 였다 (#190). 집기 정책은 퍽 높이 오차 ±5 mm 로 학습 (k8 · k9, 잡는 높이 허용 6 mm).
// 어떻게: 상자 위 퍽이면 높이를 알고 있다 (바닥 → 상자 윗면 0.07 m, 발제) → z 만 '상자 윗면 + 퍽 반 높이' 로 바꾼다.
//     세운 퍽 0.030 (높이 60 mm), 누운 직육면체 0.015 (높이 30 mm). x · y 는 잰 값 그대로.
//     잰 z 가 상자 퍽 높이에서 tolerance 넘게 떨어져 있으면 (상자 위 퍽이 아니거나 측정이 크게 틀림) 바꾸지 않고 잰 값을 쓴다.
#pragma once

#include <cmath>

namespace control
{

struct PuckZ
{
  double z;       // 쓸 퍽 중심 높이 (fixed frame)
  bool snapped;   // true = 상자 기준 높이로 바꿈
};

// measured_z: 잰 퍽 중심 높이, floor_z: 같은 frame 의 바닥 높이, box_top: 바닥 → 상자 윗면,
// center_dz: 상자 윗면 → 퍽 중심 (세운 퍽 0.030 · 누운 퍽 0.015), tolerance: 바꿀 수 있는 잰 값과의 최대 차이
inline PuckZ boxPuckZ(double measured_z, double floor_z, double box_top, double center_dz, double tolerance)
{
  const double z = floor_z + box_top + center_dz;
  if (!std::isfinite(measured_z) || !std::isfinite(z) || std::abs(measured_z - z) > tolerance) {
    return {measured_z, false};
  }
  return {z, true};
}

}  // namespace control
