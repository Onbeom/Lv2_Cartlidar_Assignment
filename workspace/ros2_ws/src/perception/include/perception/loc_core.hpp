// 위치 잃음 감지 · 다시 찾기 계산 (ROS 없음). localization/loc_core.py 를 C++ 로 옮김(같은 계산).
// - 지도: 점유 지도 yaml → 가장 가까운 점유 칸까지 거리장(clip m 에서 자름).
// - residual_multi: 라이다 끝점을 스캔마다 자기 자세로 놓았을 때 거리장 값의 중앙값('cell_median') 또는 보간 75 백분위('interp_p75').
// - search: 자세 후보를 격자로 훑어 거리장 평균이 가장 작은 자세(거친 → 고운 2 단)와 '다른 자리' 2 등 점수.
// Python 판과 다른 점: 점수 평균을 double 로 더함(numpy 는 float32 짝 합) → 1e-7 m 정도 차이, 같은 점수 후보의 순서는 안정 정렬.
#pragma once

#include <array>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>

namespace perception::loc
{

struct P2 {double x, y;};
using Pose = std::array<double, 3>;   // x, y, yaw
using T2 = std::array<double, 6>;     // 2×3 [a00 a01 tx; a10 a11 ty]

class DistField
{
public:
  explicit DistField(const std::string & map_yaml, double clip = 0.10);
  // 지도 밖은 clip
  float lookup(double x, double y) const;
  float lookup_interp(double x, double y) const;
  // 빈 칸(벽에서 0.15 m 넘게)을 x · y 모두 step_m 간격으로 솎음
  std::vector<P2> free_grid(double step_m) const;
  double clip() const {return clip_;}

private:
  double res_, ox_, oy_, clip_;
  int H_, W_;
  cv::Mat dist_;   // CV_32F
  std::vector<std::pair<int, int>> free_rc_;   // 행 순서
};

Pose se2_compose(const Pose & a, const Pose & b);
Pose se2_inverse(const Pose & a);
double wrap(double a);
std::vector<P2> move_points(const std::vector<P2> & pts, const Pose & d);
T2 T2d_from_tf(double tx, double ty, double qx, double qy, double qz, double qw);
// LaserScan 값 → base_link 기준 점. 고르게 n_max 개로 줄임
std::vector<P2> scan_points(const std::vector<float> & ranges, double angle_min, double angle_inc, const T2 & T,
  double r_min = 0.15, double r_max = 3.5, int n_max = 180);

// numpy percentile(linear) 와 같은 방식
double percentile(std::vector<double> v, double q);

double residual_multi(const DistField & f, const std::vector<std::pair<Pose, std::vector<P2>>> & pose_pts,
  const std::string & mode = "cell_median");

struct SearchResult
{
  Pose best{};
  double score = 0;
  std::optional<Pose> second;
  std::optional<double> second_score;
  size_t n_cand = 0;
};
// center 가 없으면 지도 전체(빈 칸 0.1 m 격자 × 5°)
SearchResult search(const DistField & f, const std::vector<P2> & pts, const std::optional<Pose> & center,
  double lin = 0.5, double ang = 0.5235987755982988, int n_refine = 8, double sep_xy = 0.3, double sep_yaw = 0.3490658503988659);

}  // namespace perception::loc
