// loc_core 단위 시험 (localization/test/test_loc_core.py 를 옮김): 거리장 · 잔차 · 탐색이 알려진 자세를 되찾는지
#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <fstream>

#include <opencv2/imgcodecs.hpp>

#include "perception/loc_core.hpp"

namespace L = perception::loc;

namespace
{
std::string make_map()
{
  // 0.025 m 칸, 4 m × 3 m 방 + 가운데 벽 하나(대칭 깨기)
  const auto dir = std::filesystem::temp_directory_path() / ("loc_core_test_" + std::to_string(::getpid()));
  std::filesystem::create_directories(dir);
  cv::Mat img(120, 160, CV_8U, cv::Scalar(254));
  img.row(0).setTo(0);
  img.row(119).setTo(0);
  img.col(0).setTo(0);
  img.col(159).setTo(0);
  img(cv::Range(40, 80), cv::Range(100, 101)).setTo(0);
  cv::imwrite((dir / "m.pgm").string(), img);
  std::ofstream(dir / "m.yaml") << "image: m.pgm\nresolution: 0.025\norigin: [0.0, 0.0, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n";
  return (dir / "m.yaml").string();
}

const L::DistField & field()
{
  static L::DistField f(make_map());
  return f;
}

// pose 에서 광선을 쏴 벽에 닿는 점(base_link 기준)
std::vector<L::P2> scan_from(const L::DistField & f, const L::Pose & pose, int n = 180, double r_max = 3.5)
{
  std::vector<L::P2> pts;
  for (int i = 0; i < n; ++i) {
    const double a = -M_PI + 2 * M_PI * i / n;
    for (double r = 0.05; r < r_max; r += 0.005) {
      const double x = pose[0] + r * std::cos(pose[2] + a), y = pose[1] + r * std::sin(pose[2] + a);
      if (f.lookup(x, y) < 0.0126) {pts.push_back({r * std::cos(a), r * std::sin(a)}); break;}
    }
  }
  return pts;
}
}  // namespace

TEST(LocCore, ResidualSmallAtTruthLargeWhenShifted)
{
  const L::Pose pose{1.2, 1.5, 0.3};
  const auto pts = scan_from(field(), pose);
  EXPECT_LT(L::residual_multi(field(), {{pose, pts}}, "interp_p75"), 0.02);
  EXPECT_GT(L::residual_multi(field(), {{L::Pose{1.4, 1.5, 0.3}, pts}}, "interp_p75"), 0.05);
}

TEST(LocCore, LocalSearchRecoversPose)
{
  const L::Pose pose{1.2, 1.5, 0.3};
  const auto pts = scan_from(field(), pose);
  const auto r = L::search(field(), pts, L::Pose{1.4, 1.4, 0.4}, 0.5, 30.0 * M_PI / 180.0);
  EXPECT_LT(std::hypot(r.best[0] - pose[0], r.best[1] - pose[1]), 0.03);
  EXPECT_LT(std::abs(r.best[2] - pose[2]), 2.0 * M_PI / 180.0);
}

TEST(LocCore, UpsideDownLidarMirrorsY)
{
  const auto T = L::T2d_from_tf(0.1, 0.0, 1.0, 0.0, 0.0, 0.0);   // roll π
  const auto p = L::scan_points({1.0f}, M_PI / 2, 0.0, T, 0.1, 3.5);
  ASSERT_EQ(p.size(), 1u);
  EXPECT_NEAR(p[0].x, 0.1, 1e-7);
  EXPECT_NEAR(p[0].y, -1.0, 1e-7);
}

TEST(LocCore, RearScanMovedIntoFrontFrame)
{
  // 움직이는 중: 뒤 스캔 시각 자세의 점을 odom 이동만큼 옮기면 앞 스캔 시각 자세에서 맞음 (#151)
  const L::Pose p_front{1.2, 1.5, 0.3}, p_rear{1.25, 1.52, 0.38};
  const auto pts = scan_from(field(), p_rear);
  const double raw = L::residual_multi(field(), {{p_front, pts}}, "interp_p75");
  const auto moved = L::move_points(pts, L::se2_compose(L::se2_inverse(p_front), p_rear));
  const double mv = L::residual_multi(field(), {{p_front, moved}}, "interp_p75");
  EXPECT_LT(mv, 0.02);
  EXPECT_LT(0.02, raw);
}

TEST(LocCore, ToNowCompensation)
{
  // 스캔 시각 자세 ∘ (odom 스캔 시각 → 지금) = 지금 자세 (odom 이 map 과 틀어져 있어도)
  const L::Pose map_odom{0.4, -0.2, 0.5}, o_st{1.0, 2.0, 0.1}, o_now{1.3, 2.1, 0.6};
  const auto pose_st = L::se2_compose(map_odom, o_st), pose_now = L::se2_compose(map_odom, o_now);
  const auto got = L::se2_compose(pose_st, L::se2_compose(L::se2_inverse(o_st), o_now));
  for (int i = 0; i < 3; ++i) {EXPECT_NEAR(got[i], pose_now[i], 1e-9);}
}

TEST(LocCore, PercentileLikeNumpy)
{
  EXPECT_DOUBLE_EQ(L::percentile({1, 2, 3, 4}, 75), 3.25);
  EXPECT_DOUBLE_EQ(L::percentile({1, 2, 3, 4}, 50), 2.5);
  EXPECT_DOUBLE_EQ(L::percentile({5}, 95), 5);
  EXPECT_DOUBLE_EQ(L::percentile({1, 2, 3}, 100), 3);
}
