// 팔 카메라 자세별 실측 보정 (ROS 의존 없음 → 단위 시험). 2026-11-15, #278.
//   팔 링크 길이 · 영점이 아직 확정되지 않아(#292 · #295) URDF FK 의 팔 카메라 자리가 자세마다 다르게 틀림
//   (보드 사진: BOXLOOK_PLACE 높이 −10 mm, BOXLOOK_TAG −23 mm). 그래서 URDF 는 그대로 두고, 사진을 찍은 자세 근처에서만
//   link5 → 카메라 장착을 실측 값으로 바꿔 쓴다. 표는 perception.yaml arm.cam_correction (보드 사진 PnP 맞춤, 인지 담당 도구 cal_pose_table.py).
//   고르는 법: 촬영 시각 base ← link5 (설계 FK) 가 표의 link5 자세와 위치 tol_m · 각 tol_rad 안인 항목 중 가장 가까운 것.
//   없으면 보정하지 않음(설계 FK 그대로).
#pragma once

#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/LinearMath/Transform.hpp"

namespace perception
{

struct ArmCamEntry
{
  std::string name;
  tf2::Transform link5;   // base ← link5 (찍은 관절각의 설계 FK)
  double pitch = 0, roll = 0, x = 0, z = 0;   // link5 기준 장착 보정 (rad · m)
};

// 보정 회전 · 이동: link5 기준 (롤 · 피치 · 요 0) + (x, 0, z)
inline tf2::Transform arm_cam_mount_delta(const ArmCamEntry & e)
{
  tf2::Quaternion q;
  q.setRPY(e.roll, e.pitch, 0.0);
  return tf2::Transform(q, tf2::Vector3(e.x, 0.0, e.z));
}

// 평면 10 개씩 [x y z 롤 피치 요 (link5) · 피치 롤 x z (보정)] → 항목. 개수가 안 맞으면 빈 목록
inline std::vector<ArmCamEntry> arm_cam_entries(const std::vector<std::string> & names, const std::vector<double> & flat)
{
  std::vector<ArmCamEntry> out;
  if (flat.size() != names.size() * 10) {return out;}
  for (size_t i = 0; i < names.size(); ++i) {
    const double * v = flat.data() + 10 * i;
    ArmCamEntry e;
    e.name = names[i];
    tf2::Quaternion q;
    q.setRPY(v[3], v[4], v[5]);
    e.link5 = tf2::Transform(q, tf2::Vector3(v[0], v[1], v[2]));
    e.pitch = v[6];
    e.roll = v[7];
    e.x = v[8];
    e.z = v[9];
    out.push_back(e);
  }
  return out;
}

// base ← link5 가 tol 안에 드는 가장 가까운 항목 (위치 차로 고름)
inline std::optional<size_t> arm_cam_select(
  const std::vector<ArmCamEntry> & es, const tf2::Transform & base_link5, double tol_m, double tol_rad)
{
  std::optional<size_t> best;
  double best_d = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < es.size(); ++i) {
    const double d = (es[i].link5.getOrigin() - base_link5.getOrigin()).length();
    const double a = (es[i].link5.getRotation().inverse() * base_link5.getRotation()).getAngleShortestPath();
    if (d <= tol_m && a <= tol_rad && d < best_d) {
      best_d = d;
      best = i;
    }
  }
  return best;
}

// 카메라 쪽에 곱할 보정 C: 보정된 (target ← 카메라) = (target ← 카메라, 설계) · C
//   link5_cam = link5 ← 카메라 (설계, TF 고정 변환). 보정된 link5 ← 카메라 = delta · link5_cam
inline tf2::Transform arm_cam_right_correction(const ArmCamEntry & e, const tf2::Transform & link5_cam)
{
  return link5_cam.inverse() * arm_cam_mount_delta(e) * link5_cam;
}

}  // namespace perception
