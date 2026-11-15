// 팔 카메라 자세별 실측 보정 (arm_cam_correction.hpp) 단위 시험
#include <gtest/gtest.h>

#include <cmath>

#include "perception/arm_cam_correction.hpp"

namespace P = perception;

namespace
{
tf2::Transform T(double x, double y, double z, double r, double p, double yw)
{
  tf2::Quaternion q;
  q.setRPY(r, p, yw);
  return tf2::Transform(q, tf2::Vector3(x, y, z));
}

void expect_near(const tf2::Transform & a, const tf2::Transform & b, double tol = 1e-6)   // 각은 acos 라 1e-8 rad 아래는 못 가름
{
  EXPECT_NEAR((a.getOrigin() - b.getOrigin()).length(), 0.0, tol);
  EXPECT_NEAR((a.getRotation().inverse() * b.getRotation()).getAngleShortestPath(), 0.0, tol);
}
}  // namespace

TEST(ArmCamCorrection, ParsesFlatTable)
{
  const auto es = P::arm_cam_entries({"A", "B"}, {0.11, 0, 0.37, 0, 0.43, -0.03, 0.08, -0.027, 0.05, 0.022,
                                                   0.15, 0, 0.40, 0, 0.49, -0.03, 0.082, -0.029, 0.05, 0.012});
  ASSERT_EQ(es.size(), 2u);
  EXPECT_EQ(es[1].name, "B");
  EXPECT_DOUBLE_EQ(es[1].z, 0.012);
  EXPECT_TRUE(P::arm_cam_entries({"A"}, {1, 2, 3}).empty());   // 개수가 안 맞으면 빈 목록 (보정 안 함)
}

TEST(ArmCamCorrection, SelectsNearestInsideTolerance)
{
  const auto es = P::arm_cam_entries({"PLACE", "TAG"}, {0.11, 0, 0.37, 0, 0.43, 0, 0.08, 0, 0.05, 0.02,
                                                        0.15, 0, 0.40, 0, 0.49, 0, 0.08, 0, 0.05, 0.01});
  auto s = P::arm_cam_select(es, T(0.112, 0.005, 0.372, 0, 0.44, 0.02), 0.03, 0.10);
  ASSERT_TRUE(s);
  EXPECT_EQ(*s, 0u);
  s = P::arm_cam_select(es, T(0.150, 0, 0.398, 0, 0.49, 0), 0.03, 0.10);
  ASSERT_TRUE(s);
  EXPECT_EQ(*s, 1u);
  EXPECT_FALSE(P::arm_cam_select(es, T(0.20, 0, 0.30, 0, 0.9, 0), 0.03, 0.10));     // 멀면 없음
  EXPECT_FALSE(P::arm_cam_select(es, T(0.11, 0, 0.37, 0, 0.43, 0.3), 0.03, 0.10));  // 위치는 같아도 각이 크면 없음
}

TEST(ArmCamCorrection, RightCorrectionEqualsCorrectedMount)
{
  // 보정된 base ← 카메라 = base ← link5 · delta · link5 ← 카메라 와 같아야 함
  P::ArmCamEntry e;
  e.pitch = 0.0797;
  e.roll = -0.027;
  e.x = 0.0507;
  e.z = 0.0222;
  const auto base_link5 = T(0.11, -0.0004, 0.369, 0.001, 0.426, -0.029);
  const auto link5_cam = T(0.088256, 0, 0.077446, -1.570796, 0.2, -1.570796);
  const auto design = base_link5 * link5_cam;
  const auto got = design * P::arm_cam_right_correction(e, link5_cam);
  const auto want = base_link5 * P::arm_cam_mount_delta(e) * link5_cam;
  expect_near(got, want);
  // 보정이 0 이면 그대로
  P::ArmCamEntry z;
  expect_near(design * P::arm_cam_right_correction(z, link5_cam), design);
}
