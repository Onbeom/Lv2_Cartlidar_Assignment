// 미션 시작 전 준비 확인 (R3, 2026-11-14): 시작 단계별 항목 · 터미널 표 (헤더만)
//
// mission_node 가 1 s 마다 항목을 모아 바뀌면 터미널에 표로 찍고 /mission/ready 로 낸다.
// 새 시작 (/mission/start, PAUSED 재개 제외) 은 모든 항목이 OK 일 때만 받는다.
#pragma once

#include <sstream>
#include <string>
#include <vector>

namespace mission
{

enum class ReadyState { OK, FAIL, WAIT };   // FAIL = 답은 왔는데 나쁨 · WAIT = 아직 답 없음 (안 뜸 · 응답 늦음)

struct ReadyItem
{
  std::string stage;    // 시작 단계 (1 안전 · 2 컨트롤러 · 3 노드 서버 · 4 위치 추정 · 5 Nav2)
  std::string name;
  ReadyState state{ReadyState::WAIT};
  std::string detail;   // active · inactive · LOST · 서버 없음 …
};

inline const char * ready_label(ReadyState s)
{
  switch (s) {
    case ReadyState::OK: return "[ OK ]";
    case ReadyState::FAIL: return "[실패]";
    default: return "[대기]";
  }
}

inline bool all_ready(const std::vector<ReadyItem> & items)
{
  for (const auto & i : items) {
    if (i.state != ReadyState::OK) {return false;}
  }
  return true;
}

// OK 아닌 항목 "이름 (detail), …" — 시작 거절 메시지 · /mission/ready
inline std::string ready_missing(const std::vector<ReadyItem> & items)
{
  std::string out;
  for (const auto & i : items) {
    if (i.state == ReadyState::OK) {continue;}
    if (!out.empty()) {out += ", ";}
    out += i.name + (i.detail.empty() ? "" : " (" + i.detail + ")");
  }
  return out;
}

// 터미널 표: 머리줄 (OK 수 / 전체) + 단계가 바뀔 때 단계 이름 + 항목 줄
inline std::string format_ready(const std::vector<ReadyItem> & items)
{
  size_t ok = 0;
  for (const auto & i : items) {ok += i.state == ReadyState::OK;}
  std::ostringstream o;
  o << "[준비 확인] " << (ok == items.size() ? "READY" : "NOT READY") << " " << ok << "/" << items.size();
  std::string stage;
  for (const auto & i : items) {
    if (i.stage != stage) {
      stage = i.stage;
      o << "\n  " << stage;
    }
    o << "\n    " << ready_label(i.state) << " " << i.name;
    if (!i.detail.empty()) {o << ": " << i.detail;}
  }
  return o.str();
}

}  // namespace mission
