// localization/loc_core.py 를 C++ 로 옮김 — 설명은 include/perception/loc_core.hpp
#include "perception/loc_core.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <limits>
#include <numeric>
#include <stdexcept>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <yaml-cpp/yaml.h>

namespace perception::loc
{

DistField::DistField(const std::string & map_yaml, double clip)
: clip_(clip)
{
  const YAML::Node meta = YAML::LoadFile(map_yaml);
  const std::filesystem::path img_path = std::filesystem::path(map_yaml).parent_path() / meta["image"].as<std::string>();
  const cv::Mat img = cv::imread(img_path.string(), cv::IMREAD_GRAYSCALE);
  if (img.empty()) {throw std::runtime_error("지도 그림을 못 읽음: " + img_path.string());}
  const double occ_t = meta["occupied_thresh"].as<double>(), free_t = meta["free_thresh"].as<double>();
  res_ = meta["resolution"].as<double>();
  ox_ = meta["origin"][0].as<double>();
  oy_ = meta["origin"][1].as<double>();
  H_ = img.rows;
  W_ = img.cols;
  cv::Mat not_occ(H_, W_, CV_8U);
  std::vector<char> is_free(static_cast<size_t>(H_) * W_);
  for (int r = 0; r < H_; ++r) {
    for (int c = 0; c < W_; ++c) {
      const double v = (255.0 - static_cast<double>(img.at<uint8_t>(r, c))) / 255.0;
      not_occ.at<uint8_t>(r, c) = v > occ_t ? 0 : 1;
      is_free[static_cast<size_t>(r) * W_ + c] = v < free_t;
    }
  }
  cv::Mat dt;
  cv::distanceTransform(not_occ, dt, cv::DIST_L2, 5);   // CV_32F, 칸 단위
  dist_.create(H_, W_, CV_32F);
  const float fres = static_cast<float>(res_), fclip = static_cast<float>(clip), f015 = 0.15f;   // numpy: float32 배열 × 파이썬 실수 = float32
  for (int r = 0; r < H_; ++r) {
    for (int c = 0; c < W_; ++c) {
      const float d = dt.at<float>(r, c) * fres;
      dist_.at<float>(r, c) = std::min(d, fclip);
      if (is_free[static_cast<size_t>(r) * W_ + c] && d > f015) {free_rc_.emplace_back(r, c);}
    }
  }
}

float DistField::lookup(double x, double y) const
{
  const auto c = static_cast<int64_t>(std::floor((x - ox_) / res_));
  const auto r = static_cast<int64_t>(H_ - 1) - static_cast<int64_t>(std::floor((y - oy_) / res_));
  if (c < 0 || c >= W_ || r < 0 || r >= H_) {return static_cast<float>(clip_);}
  return dist_.at<float>(static_cast<int>(r), static_cast<int>(c));
}

float DistField::lookup_interp(double x, double y) const
{
  const double fx = (x - ox_) / res_ - 0.5;
  const double fy = (H_ - 1) - ((y - oy_) / res_ - 0.5);
  const double c0d = std::floor(fx), r0d = std::floor(fy);
  const auto c0 = static_cast<int64_t>(c0d), r0 = static_cast<int64_t>(r0d);
  const double ax = fx - c0d, ay = fy - r0d;
  if (c0 < 0 || c0 + 1 >= W_ || r0 < 0 || r0 + 1 >= H_) {return static_cast<float>(clip_);}
  const int c = static_cast<int>(c0), r = static_cast<int>(r0);
  const double g00 = dist_.at<float>(r, c), g01 = dist_.at<float>(r, c + 1), g10 = dist_.at<float>(r + 1, c), g11 = dist_.at<float>(r + 1, c + 1);
  return static_cast<float>(g00 * (1 - ax) * (1 - ay) + g01 * ax * (1 - ay) + g10 * (1 - ax) * ay + g11 * ax * ay);
}

std::vector<P2> DistField::free_grid(double step_m) const
{
  const int k = std::max(1, static_cast<int>(std::nearbyint(step_m / res_)));   // 파이썬 round 와 같이 짝수 쪽 반올림
  std::vector<P2> out;
  for (const auto & [r, c] : free_rc_) {
    if (r % k == 0 && c % k == 0) {
      out.push_back({ox_ + (c + 0.5) * res_, oy_ + (H_ - 1 - r + 0.5) * res_});
    }
  }
  return out;
}

double wrap(double a) {return std::atan2(std::sin(a), std::cos(a));}

Pose se2_compose(const Pose & a, const Pose & b)
{
  const double c = std::cos(a[2]), s = std::sin(a[2]);
  return {a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], std::atan2(std::sin(a[2] + b[2]), std::cos(a[2] + b[2]))};
}

Pose se2_inverse(const Pose & a)
{
  const double c = std::cos(a[2]), s = std::sin(a[2]);
  return {-(c * a[0] + s * a[1]), -(-s * a[0] + c * a[1]), -a[2]};
}

std::vector<P2> move_points(const std::vector<P2> & pts, const Pose & d)
{
  const double c = std::cos(d[2]), s = std::sin(d[2]);
  std::vector<P2> out;
  out.reserve(pts.size());
  for (const auto & p : pts) {out.push_back({p.x * c + p.y * -s + d[0], p.x * s + p.y * c + d[1]});}
  return out;
}

T2 T2d_from_tf(double tx, double ty, double qx, double qy, double qz, double qw)
{
  return {1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), tx,
    2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), ty};
}

std::vector<P2> scan_points(const std::vector<float> & ranges, double angle_min, double angle_inc, const T2 & T,
  double r_min, double r_max, int n_max)
{
  std::vector<P2> p;
  for (size_t i = 0; i < ranges.size(); ++i) {
    const double r = ranges[i];
    if (!(std::isfinite(r) && r > r_min && r < r_max)) {continue;}
    const double a = angle_min + angle_inc * static_cast<double>(i);
    const double x = r * std::cos(a), y = r * std::sin(a);
    p.push_back({x * T[0] + y * T[1] + T[2], x * T[3] + y * T[4] + T[5]});
  }
  if (static_cast<int>(p.size()) > n_max && n_max > 0) {   // np.linspace(0, len − 1, n_max).astype(int)
    std::vector<P2> q;
    q.reserve(n_max);
    const double last = static_cast<double>(p.size() - 1);
    const double step = n_max > 1 ? last / (n_max - 1) : 0.0;
    for (int i = 0; i < n_max; ++i) {
      const double v = (i == n_max - 1 && n_max > 1) ? last : i * step;
      q.push_back(p[static_cast<size_t>(v)]);
    }
    return q;
  }
  return p;
}

double percentile(std::vector<double> v, double q)
{
  if (v.empty()) {return std::numeric_limits<double>::quiet_NaN();}
  std::sort(v.begin(), v.end());
  const double n = static_cast<double>(v.size()), qq = q / 100.0;
  const double vi = n * qq + (1.0 + qq * (1.0 - 1.0 - 1.0)) - 1.0;   // numpy _compute_virtual_index (linear: α = β = 1)
  const double lo = std::floor(vi);
  const size_t i0 = static_cast<size_t>(std::clamp(lo, 0.0, n - 1)), i1 = std::min(i0 + 1, v.size() - 1);
  const double t = vi - lo, d = v[i1] - v[i0];
  return t >= 0.5 ? v[i1] - d * (1 - t) : v[i0] + d * t;
}

namespace
{
// float32 배열의 numpy percentile(linear) — 차이(b − a)는 float32, 섞을 때는 float64 (numpy 1.26 _lerp 와 같음)
double percentile_f32(std::vector<float> v, double q)
{
  std::sort(v.begin(), v.end());
  const double n = static_cast<double>(v.size()), qq = q / 100.0;
  const double vi = n * qq + (1.0 + qq * (1.0 - 1.0 - 1.0)) - 1.0;
  const double lo = std::floor(vi);
  const size_t i0 = static_cast<size_t>(std::clamp(lo, 0.0, n - 1)), i1 = std::min(i0 + 1, v.size() - 1);
  const double t = vi - lo;
  const float d = v[i1] - v[i0];
  return t >= 0.5 ? static_cast<double>(v[i1]) - static_cast<double>(d) * (1 - t) : static_cast<double>(v[i0]) + static_cast<double>(d) * t;
}

double median_f32(std::vector<float> v)   // np.median(float32) — 짝수 개면 가운데 둘의 float32 평균
{
  std::sort(v.begin(), v.end());
  const size_t n = v.size();
  if (n % 2) {return v[n / 2];}
  return static_cast<double>((v[n / 2 - 1] + v[n / 2]) / 2.0f);
}

inline P2 place1(const Pose & p, const P2 & q)
{
  const double c = std::cos(p[2]), s = std::sin(p[2]);
  return {p[0] + c * q.x - s * q.y, p[1] + s * q.x + c * q.y};
}

std::vector<double> scores(const DistField & f, const std::vector<Pose> & poses, const std::vector<P2> & pts)
{
  std::vector<double> out(poses.size());
  for (size_t i = 0; i < poses.size(); ++i) {
    const double c = std::cos(poses[i][2]), s = std::sin(poses[i][2]);
    double sum = 0;
    for (const auto & q : pts) {sum += f.lookup(poses[i][0] + c * q.x - s * q.y, poses[i][1] + s * q.x + c * q.y);}
    out[i] = static_cast<float>(sum / static_cast<double>(pts.size()));   // Python 판 점수는 float32
  }
  return out;
}

std::vector<double> arange(double start, double stop, double step)   // np.arange: 길이 ceil((stop − start)/step), 값 start + i · (두 번째 − 첫 번째)
{
  const auto n = static_cast<long>(std::ceil((stop - start) / step));
  std::vector<double> v;
  if (n <= 0) {return v;}
  const double d = (start + step) - start;
  for (long i = 0; i < n; ++i) {v.push_back(start + static_cast<double>(i) * d);}
  return v;
}

std::vector<Pose> grid(const Pose & center, double lin, double lin_step, double ang, double ang_step)
{
  const auto xs = arange(-lin, lin + 1e-9, lin_step), ys = arange(-lin, lin + 1e-9, lin_step), yaws = arange(-ang, ang + 1e-9, ang_step);
  std::vector<Pose> out;
  out.reserve(xs.size() * ys.size() * yaws.size());
  for (double x : xs) {for (double y : ys) {for (double a : yaws) {out.push_back({center[0] + x, center[1] + y, center[2] + a});}}}
  return out;
}

std::pair<Pose, double> refine(const DistField & f, const std::vector<P2> & pts, const Pose & pose)
{
  Pose best = pose;
  double bs = 0;
  const double stages[2][4] = {{0.06, 0.01, 4.0, 0.5}, {0.012, 0.003, 0.6, 0.15}};
  for (const auto & st : stages) {
    const auto cand = grid(best, st[0], st[1], st[2] * M_PI / 180.0, st[3] * M_PI / 180.0);
    const auto sc = scores(f, cand, pts);
    const size_t i = static_cast<size_t>(std::min_element(sc.begin(), sc.end()) - sc.begin());   // argmin: 처음 최소
    best = cand[i];
    bs = sc[i];
  }
  return {best, bs};
}

bool apart(const Pose & p, const Pose & q, double sep_xy, double sep_yaw)
{
  return std::hypot(p[0] - q[0], p[1] - q[1]) > sep_xy || std::abs(std::atan2(std::sin(p[2] - q[2]), std::cos(p[2] - q[2]))) > sep_yaw;
}
}  // namespace

double residual_multi(const DistField & f, const std::vector<std::pair<Pose, std::vector<P2>>> & pose_pts, const std::string & mode)
{
  const bool interp = mode == "interp_p75";
  std::vector<float> d;
  for (const auto & [p, q] : pose_pts) {
    for (const auto & pt : q) {
      const P2 w = place1(p, pt);
      d.push_back(interp ? f.lookup_interp(w.x, w.y) : f.lookup(w.x, w.y));
    }
  }
  if (d.empty()) {return std::numeric_limits<double>::quiet_NaN();}
  return interp ? percentile_f32(std::move(d), 75.0) : median_f32(std::move(d));
}

SearchResult search(const DistField & f, const std::vector<P2> & pts, const std::optional<Pose> & center,
  double lin, double ang, int n_refine, double sep_xy, double sep_yaw)
{
  std::vector<Pose> cand;
  if (!center) {   // 지도 전체: 빈 칸 0.1 m 격자 × 5°
    const auto xy = f.free_grid(0.10);
    for (double a : arange(-M_PI, M_PI, 5.0 * M_PI / 180.0)) {
      for (const auto & p : xy) {cand.push_back({p.x, p.y, a});}
    }
  } else {
    cand = grid(*center, lin, 0.04, ang, 3.0 * M_PI / 180.0);
  }
  SearchResult res;
  res.n_cand = cand.size();
  if (cand.empty() || pts.empty()) {res.score = std::numeric_limits<double>::infinity(); return res;}
  const auto sc = scores(f, cand, pts);
  std::vector<size_t> order(cand.size());
  std::iota(order.begin(), order.end(), 0);
  std::stable_sort(order.begin(), order.end(), [&](size_t a, size_t b) {return sc[a] < sc[b];});
  std::vector<Pose> picked;   // 서로 다른 자리 상위 n_refine 개
  for (size_t i : order) {
    const Pose & p = cand[i];
    if (std::all_of(picked.begin(), picked.end(), [&](const Pose & q) {return apart(p, q, sep_xy, sep_yaw);})) {picked.push_back(p);}
    if (static_cast<int>(picked.size()) >= n_refine) {break;}
  }
  std::vector<std::pair<Pose, double>> refined;
  for (const auto & p : picked) {refined.push_back(refine(f, pts, p));}
  std::stable_sort(refined.begin(), refined.end(), [](const auto & a, const auto & b) {return a.second < b.second;});
  res.best = refined[0].first;
  res.score = refined[0].second;
  for (size_t i = 1; i < refined.size(); ++i) {
    if (apart(refined[i].first, res.best, sep_xy, sep_yaw)) {
      res.second = refined[i].first;
      res.second_score = refined[i].second;
      break;
    }
  }
  return res;
}

}  // namespace perception::loc
