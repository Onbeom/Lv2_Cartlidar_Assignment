// 상자 위 퍽 중심 높이 (puck_height.hpp, manipulator_node target_look.box_z*, #279)
#include <gtest/gtest.h>

#include <limits>

#include "control/puck_height.hpp"

using control::boxPuckZ;

namespace
{
// manipulator.yaml 기본: 상자 윗면 0.07 m, 세운 퍽 반 높이 0.030 · 누운 퍽 0.015, 허용 0.04 m
constexpr double kBox = 0.07;
constexpr double kTol = 0.04;
}  // namespace

TEST(PuckHeight, CameraHeightErrorIsReplacedByBoxTop)
{
  // 실물 #279 · #190: 카메라 높이가 8 ~ 25 mm 틀리면 잰 퍽 높이가 그만큼 틀림 → 상자 기준 높이로
  for (double err : {-0.025, -0.008, 0.0, 0.010, 0.022, 0.025}) {
    const auto r = boxPuckZ(0.100 + err, 0.0, kBox, 0.030, kTol);
    EXPECT_TRUE(r.snapped) << err;
    EXPECT_DOUBLE_EQ(r.z, 0.100) << err;
  }
  // 바닥이 fixed frame 에서 0 이 아니어도 (odom z 치우침) 바닥 기준
  const auto r = boxPuckZ(0.335, 0.231, kBox, 0.030, kTol);
  EXPECT_TRUE(r.snapped);
  EXPECT_NEAR(r.z, 0.331, 1e-12);
}

TEST(PuckHeight, LyingPuckUsesHalfHeight)
{
  const auto r = boxPuckZ(0.092, 0.0, kBox, 0.015, kTol);
  EXPECT_TRUE(r.snapped);
  EXPECT_DOUBLE_EQ(r.z, 0.085);
}

TEST(PuckHeight, FarFromBoxKeepsMeasured)
{
  // 바닥 위 퍽 · 팔레트 칸 퍽 등 상자 위가 아니거나 측정이 크게 틀리면 잰 값 그대로
  for (double z : {0.030, 0.055, 0.145, 0.300}) {
    const auto r = boxPuckZ(z, 0.0, kBox, 0.030, kTol);
    EXPECT_FALSE(r.snapped) << z;
    EXPECT_DOUBLE_EQ(r.z, z) << z;
  }
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(boxPuckZ(nan, 0.0, kBox, 0.030, kTol).snapped);
  EXPECT_FALSE(boxPuckZ(0.1, nan, kBox, 0.030, kTol).snapped);
}
