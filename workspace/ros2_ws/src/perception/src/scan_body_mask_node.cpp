// ScanBodyMaskNode — 라이다 차체 가림 칸 제거 (localization/scan_body_mask.py 를 C++ 로 옮김, 동작 같음).
// 왜 옮겼나: Python 판이 Jetson 실물 정지 상태에서 CPU 코어 하나의 약 18 % (11-13 #265). 라이다 두 대 스캔마다 numpy 로 계산.
// 원본 토픽은 그대로 두고 <입력 토픽><output_suffix> 로 낸다. 두 가지를 합쳐 inf 로 바꾼다:
//  ① 고정 칸: 첫 스캔 뒤 learn_window 초(기본 0.5~1.9 s, 출발 전 정지 구간) 동안 점이 몸체 상자 안에 든 적 있는 칸.
//     mask_file 이 있으면 그 칸을 쓰고(학습 안 함), 학습했으면 mask_save 에 저장.
//  ② 매 스캔 상자 검사: 그 스캔에서 몸체 상자 안 점.
// 상자 기본값 x −0.175~0.285, |y| ≤ 0.19 m. 학습 끝나기 전 스캔은 ②만 적용해 낸다.
// 라이다가 한 바퀴 점 수를 바꿔 내도(실물 coin_d4: 399 · 400) 학습 칸은 긴 쪽 길이로 OR 한다(or_grow).
#include <cmath>
#include <fstream>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "perception/tf_helper.hpp"

namespace perception
{
using LaserScan = sensor_msgs::msg::LaserScan;

class ScanBodyMaskNode : public rclcpp::Node
{
public:
  explicit ScanBodyMaskNode(const rclcpp::NodeOptions & options)
  : Node("scan_body_mask", options)
  {
    const auto topics = declare_parameter("scan_topics", std::vector<std::string>{"/scan_front", "/scan_rear"});
    const auto suffix = declare_parameter("output_suffix", std::string("_filtered"));
    base_ = declare_parameter("base_frame", std::string("base_link"));
    box_ = declare_parameter("box", std::vector<double>{-0.175, 0.285, 0.19});
    win_ = declare_parameter("learn_window", std::vector<double>{0.5, 1.9});
    const auto mask_file = declare_parameter("mask_file", std::string("none"));
    save_path_ = declare_parameter("mask_save", std::string("none"));
    if (save_path_ == "none") {save_path_.clear();}
    if (box_.size() != 3 || win_.size() != 2) {throw std::invalid_argument("box 는 3 개, learn_window 는 2 개");}
    tf_ = std::make_unique<TfHelper>(*this);
    if (!mask_file.empty() && mask_file != "none") {
      const YAML::Node y = YAML::LoadFile(mask_file);
      std::string s;
      for (const auto & kv : y) {
        auto & v = fixed_[kv.first.as<std::string>()];
        for (const auto & b : kv.second) {v.push_back(b.as<int>());}
        s += kv.first.as<std::string>() + " " + std::to_string(v.size()) + "칸 ";
      }
      RCLCPP_INFO(get_logger(), "고정 칸 불러옴 %s: %s", mask_file.c_str(), s.c_str());
    }
    for (const auto & topic : topics) {
      auto pub = create_publisher<LaserScan>(topic + suffix, 10);
      pubs_.push_back(pub);
      subs_.push_back(create_subscription<LaserScan>(topic, rclcpp::SensorDataQoS(),
          [this, pub, topic](LaserScan::UniquePtr m) {on_scan(std::move(m), pub, topic);}));
    }
    timer_ = rclcpp::create_timer(this, get_clock(), std::chrono::seconds(10), [this] {report();});   // Python 판처럼 노드 시계
  }

private:
  void on_scan(LaserScan::UniquePtr msg, const rclcpp::Publisher<LaserScan>::SharedPtr & pub, const std::string & topic)
  {
    const auto T = tf_->fixed(base_, msg->header.frame_id);
    if (!T) {return;}   // TfHelper 가 5 s 마다 경고
    const auto & R = T->getBasis();
    const auto & p = T->getOrigin();
    const size_t n = msg->ranges.size();
    std::vector<char> inside(n, 0);
    for (size_t i = 0; i < n; ++i) {
      const float r = msg->ranges[i];
      const bool ok = std::isfinite(r);
      const double rr = ok ? static_cast<double>(r) : 0.0;
      const double a = msg->angle_min + static_cast<double>(msg->angle_increment) * static_cast<double>(i);
      const double ca = std::cos(a), sa = std::sin(a);
      const double x = p.x() + rr * (R[0][0] * ca + R[0][1] * sa);
      const double y = p.y() + rr * (R[1][0] * ca + R[1][1] * sa);
      inside[i] = ok && x >= box_[0] && x <= box_[1] && std::abs(y) <= box_[2];
    }
    const double ts = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    if (!have_t0_) {t0_ = ts; have_t0_ = true;}
    if (!fixed_.count(topic)) {
      if (win_[0] <= ts - t0_ && ts - t0_ <= win_[1]) {
        auto & acc = learning_[topic];   // or_grow: 긴 쪽 길이로 칸별 OR
        if (acc.size() < n) {acc.resize(n, 0);}
        for (size_t i = 0; i < n; ++i) {acc[i] |= inside[i];}
      } else if (ts - t0_ > win_[1] && learning_.count(topic)) {
        auto & v = fixed_[topic];
        const auto & acc = learning_[topic];
        for (size_t i = 0; i < acc.size(); ++i) {if (acc[i]) {v.push_back(static_cast<int>(i));}}
        learning_.erase(topic);
        RCLCPP_INFO(get_logger(), "%s: 고정 칸 %zu칸 학습(%.1f°)", topic.c_str(), v.size(),
          v.size() * msg->angle_increment * 180.0 / M_PI);
        save();
      }
    }
    if (auto it = fixed_.find(topic); it != fixed_.end()) {
      for (int k : it->second) {if (k >= 0 && static_cast<size_t>(k) < n) {inside[k] = 1;}}
    }
    long cnt = 0;
    for (size_t i = 0; i < n; ++i) {
      if (inside[i]) {msg->ranges[i] = std::numeric_limits<float>::infinity(); ++cnt;}
    }
    pub->publish(std::move(msg));
    auto & s = stats_[topic];
    s.first += 1;
    s.second += cnt;
  }

  void save()
  {
    if (save_path_.empty()) {return;}
    YAML::Emitter e;
    e << YAML::BeginMap;
    for (const auto & [k, v] : fixed_) {e << YAML::Key << k << YAML::Value << YAML::Flow << v;}
    e << YAML::EndMap;
    std::ofstream(save_path_) << e.c_str() << "\n";
  }

  void report()
  {
    for (const auto & [topic, s] : stats_) {
      if (s.first) {RCLCPP_INFO(get_logger(), "%s: 스캔 %ld, 평균 가린 칸 %.1f", topic.c_str(), s.first, double(s.second) / s.first);}
    }
  }

  std::string base_, save_path_;
  std::vector<double> box_, win_;
  std::unique_ptr<TfHelper> tf_;
  bool have_t0_ = false;
  double t0_ = 0.0;
  std::map<std::string, std::vector<int>> fixed_;
  std::map<std::string, std::vector<char>> learning_;
  std::map<std::string, std::pair<long, long>> stats_;
  std::vector<rclcpp::Publisher<LaserScan>::SharedPtr> pubs_;
  std::vector<rclcpp::Subscription<LaserScan>::SharedPtr> subs_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace perception

RCLCPP_COMPONENTS_REGISTER_NODE(perception::ScanBodyMaskNode)
