// StartupStow — 전원을 켠 뒤 팔을 정리 자세 (startup.pose, 기본 DRIVE = 주행 자세) 로 천천히 옮기는 계산 (manipulator_node startup.*, #144)
//
// 왜: 켜진 팔은 대기 자세 (STOW, 차체 위에 접힘 — opencr_hardware 가 켤 때 이 범위를 확인, #166) 에 있다. 바로 미션 자세로
//     관절 공간 직선 이동을 하면 팔끼리 · 카메라에 닿을 수 있어 (2026-11-08 실물 READY 충돌), 켜질 때 한 번 주행 자세로 편다.
// 어떻게: 관절을 묶음 (stages) 순서로 움직인다. 기본 = J3 → J2 → J4 → J1 → (J5 · 그리퍼, 2026-11-13 새 DRIVE), 대기 → 주행을 한 관절씩
//     실물로 확인한 순서 (#165). 묶음 밖 관절은 지금 자리 (한계 안으로 자른 값) 에 둔다.
//     이미 정리 자세 근처 (skip_tolerance 안) 인 묶음은 건너뛴다.
// 2026-11-18: 팔이 STOW 가 아닌 자리에서 켜지면 먼저 startup.fold_path 순서로 STOW 로 접는다 (parseFoldPath).
//     기본 경로 = 팔을 들어 올림 (J2 · J3 · J4 함께, FOLD_UP) → 손목 · 집게 → J1 을 STOW 쪽으로 → J4 → J2 → J3 (DRIVE → STOW 접기와 같은 끝 순서).
#pragma once

#include <algorithm>
#include <cmath>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace control
{

// "2,3,4,5,6" 같은 관절 번호 묶음 목록 → 0 부터 센 칸 번호. 번호는 1..n, 묶음 사이에도 중복 불가, 빈 묶음 불가
inline std::vector<std::vector<size_t>> parseStartupStages(const std::vector<std::string> & stages, size_t n)
{
  std::vector<std::vector<size_t>> out;
  std::set<size_t> seen;
  for (const auto & text : stages) {
    std::vector<size_t> stage;
    std::istringstream in(text);
    std::string tok;
    while (std::getline(in, tok, ',')) {
      const auto first = tok.find_first_not_of(" \t");
      if (first == std::string::npos) {continue;}
      size_t used = 0;
      int k = 0;
      try {
        k = std::stoi(tok.substr(first), &used);
      } catch (const std::exception &) {
        throw std::invalid_argument("startup.stages: not a joint number '" + tok + "'");
      }
      if (tok.find_first_not_of(" \t", first + used) != std::string::npos) {
        throw std::invalid_argument("startup.stages: not a joint number '" + tok + "'");
      }
      if (k < 1 || static_cast<size_t>(k) > n) {
        throw std::invalid_argument("startup.stages: joint " + std::to_string(k) + " outside 1.." + std::to_string(n));
      }
      if (!seen.insert(static_cast<size_t>(k - 1)).second) {
        throw std::invalid_argument("startup.stages: joint " + std::to_string(k) + " listed twice");
      }
      stage.push_back(static_cast<size_t>(k - 1));
    }
    if (stage.empty()) {
      throw std::invalid_argument("startup.stages: empty stage '" + text + "'");
    }
    out.push_back(stage);
  }
  return out;
}

// 묶음 하나의 목표: 지금 자리 (한계 안으로 자름) + 묶음 관절만 정리 자세 값
inline std::vector<double> startupStageTarget(
  const std::vector<double> & current, const std::vector<double> & pose, const std::vector<size_t> & stage,
  const std::vector<double> & lower, const std::vector<double> & upper)
{
  std::vector<double> q(current.size());
  for (size_t i = 0; i < q.size(); ++i) {q[i] = std::clamp(current[i], lower[i], upper[i]);}
  for (size_t k : stage) {q[k] = pose[k];}
  return q;
}

// 묶음 관절이 모두 정리 자세에서 tol 안이면 false (움직일 필요 없음)
inline bool startupStageNeeded(
  const std::vector<double> & current, const std::vector<double> & pose, const std::vector<size_t> & stage, double tol)
{
  for (size_t k : stage) {
    if (std::abs(current[k] - pose[k]) > tol) {return true;}
  }
  return false;
}

// 접기 경로 한 단계: 움직일 관절 칸 · 그 관절의 목표 이름 자세 (poses.*)
struct FoldStep
{
  std::vector<size_t> joints;
  std::string pose;
};

// startup.fold_path ("5,6:STOW", "2,3,4:FOLD_UP", "1:STOW", …) → 단계 목록 (2026-11-18: 어느 자세에서 켜도 STOW 로 접기).
//   관절 번호 규칙은 parseStartupStages 와 같되, 단계마다 목표 자세가 다를 수 있어 같은 관절이 여러 단계에 나와도 된다.
inline std::vector<FoldStep> parseFoldPath(const std::vector<std::string> & path, size_t n)
{
  std::vector<FoldStep> out;
  for (const auto & text : path) {
    const auto colon = text.rfind(':');
    if (colon == std::string::npos || colon + 1 >= text.size()) {
      throw std::invalid_argument("startup.fold_path: '" + text + "' must be '<joints>:<pose>'");
    }
    std::string pose = text.substr(colon + 1);
    pose.erase(0, pose.find_first_not_of(" \t"));
    pose.erase(pose.find_last_not_of(" \t") + 1);
    if (pose.empty()) {throw std::invalid_argument("startup.fold_path: empty pose in '" + text + "'");}
    const auto joints = parseStartupStages({text.substr(0, colon)}, n);   // 한 묶음 안 중복 · 범위 검사
    out.push_back({joints.front(), pose});
  }
  return out;
}

// 이동 시간 = 가장 많이 움직이는 관절 / max_vel (그리퍼 포함), min_duration 이상
inline double startupStageDuration(
  const std::vector<double> & from, const std::vector<double> & to, double max_vel, double min_duration)
{
  double max_delta = 0.0;
  for (size_t i = 0; i < std::min(from.size(), to.size()); ++i) {
    max_delta = std::max(max_delta, std::abs(to[i] - from[i]));
  }
  return std::max(min_duration, max_delta / max_vel);
}

}  // namespace control
