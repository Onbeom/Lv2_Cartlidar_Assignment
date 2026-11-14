// 재시도를 다 쓴 단계 처리 (R1, 2026-11-14): 퍽 하나만 건너뛰기 · HOME 복귀 자리 찾기 (헤더만, 단계 원문으로 판단)
//
// 계획기 단계 (mission_planner/mission_lines.py) 의 퍽 하나 집기 묶음:
//   [skill drive turn_<p|m><deg>] · skill manipulator target_look:… · skill manipulator toppick_tilt ·
//   skill manipulator pallet_load_<열>_<칸>_tilt · [skill drive turn_0]
// 놓기 묶음: skill manipulator pallet_unload_<열>_<칸>_tilt · skill manipulator place_<L>_<dx>_<dy>_tilt
// 복귀: goto <HOME x y yaw> · wait <s> (맨 끝)
#pragma once

#include <optional>
#include <sstream>
#include <string>
#include <vector>

namespace mission
{

// "skill manipulator toppick_tilt 40 mode=…" -> {"skill", "manipulator", "toppick_tilt", …}
inline std::vector<std::string> step_tokens(const std::string & text)
{
  std::istringstream in(text);
  std::vector<std::string> t;
  std::string w;
  while (in >> w) {t.push_back(w);}
  return t;
}

// skill <who> <name> 이고 name 이 prefix 로 시작하면 name, 아니면 빈 문자열
inline std::string skill_name(const std::string & text, const std::string & who, const std::string & prefix)
{
  const auto t = step_tokens(text);
  if (t.size() < 3 || t[0] != "skill" || t[1] != who || t[2].rfind(prefix, 0) != 0) {return "";}
  return t[2];
}

// pallet_load_1_2_tilt · pallet_unload_1_2_tilt -> "1_2" (아니면 빈 문자열)
inline std::string pallet_slot(const std::string & text, const std::string & kind)
{
  const std::string n = skill_name(text, "manipulator", "pallet_" + kind + "_");
  const std::string head = "pallet_" + kind + "_", tail = "_tilt";
  if (n.empty() || n.size() <= head.size() + tail.size() || n.compare(n.size() - tail.size(), tail.size(), tail) != 0) {
    return "";
  }
  return n.substr(head.size(), n.size() - head.size() - tail.size());
}

// 집기 묶음 안 단계 (pallet_load 앞): 제자리 회전 (turn_0 제외) · target_look · toppick_tilt
inline bool is_pick_group_step(const std::string & text)
{
  const std::string turn = skill_name(text, "drive", "turn_");
  if (!turn.empty()) {return turn != "turn_0";}
  return !skill_name(text, "manipulator", "target_look:").empty() ||
         !skill_name(text, "manipulator", "toppick_tilt").empty();
}

struct PickSkip
{
  std::string slot;     // 빈 칸이 되는 팔레트 칸 "열_칸"
  size_t resume{0};     // 다시 시작할 단계 = pallet_load 다음 (turn_0 이 있으면 그것, 퍽 없이 정렬 방향으로 되돌림)
};

// failed 단계가 집기 묶음 안이면 같은 묶음의 pallet_load 를 찾아 건너뛸 범위를 줌.
// 묶음 밖 단계 (주행 · 정렬 · 놓기 · pallet_load 자체) 이거나 pallet_load 가 없으면 nullopt
inline std::optional<PickSkip> pick_skip(const std::vector<std::string> & texts, size_t failed)
{
  if (failed >= texts.size() || !is_pick_group_step(texts[failed])) {return std::nullopt;}
  for (size_t i = failed + 1; i < texts.size(); ++i) {
    const std::string slot = pallet_slot(texts[i], "load");
    if (!slot.empty()) {return PickSkip{slot, i + 1};}
    if (!is_pick_group_step(texts[i])) {return std::nullopt;}
  }
  return std::nullopt;
}

// 놓기 묶음 건너뛰기: unload 단계 idx 다음이 place_ 면 그 뒤, 아니면 idx + 1
inline size_t after_unload(const std::vector<std::string> & texts, size_t idx)
{
  if (idx + 1 < texts.size() && !skill_name(texts[idx + 1], "manipulator", "place_").empty()) {return idx + 2;}
  return idx + 1;
}

// 복귀 줄: 맨 끝 wait 바로 앞 goto (계획기 return). 없으면 nullopt (계획 전 · tag 모드)
inline std::optional<size_t> home_index(const std::vector<std::string> & texts)
{
  if (texts.size() < 2) {return std::nullopt;}
  const auto last = step_tokens(texts.back());
  if (last.empty() || last[0] != "wait") {return std::nullopt;}
  const auto prev = step_tokens(texts[texts.size() - 2]);
  if (prev.empty() || prev[0] != "goto") {return std::nullopt;}
  return texts.size() - 2;
}

}  // namespace mission
