// perception_survey 완료 파일 -> 계획기 퍽 등록표 (pucks: [{id, color, shape, box, side, pose, xy}]). ROS 의존 없음 (시험용으로 분리)
//   xy: 잰 퍽 중심 [m, 지도] = x_mm · y_mm / 1000. measured 가 true 인 칸만 (아니면 칸 기본 자리라 안 씀).
//   계획기는 12 개 모두 xy 가 있을 때만 잰 자리 집기 규칙 (mission_planner measured.py) 을 씀
//   shape: cuboid -> box (계획기 이름). pose (standing | lying, #193) 는 있으면 옮기고, 없거나 null 이면 쓰지 않음
//   (계획기는 pose 가 없으면 standing 으로 읽음). 그 밖의 pose 값은 오류
#pragma once

#include <sstream>
#include <string>

#include "yaml-cpp/yaml.h"

namespace mission
{

inline bool surveyToPucksYaml(const YAML::Node & doc, const std::string & path, std::string & out, std::string & err)
{
  const YAML::Node slots = doc["slots"];
  if (!slots || slots.size() != 12) {err = "complete file must have 12 slots: " + path; return false;}
  std::ostringstream o;
  o << "# mission_node 가 perception_survey 완료 파일 (" << path << ") 에서 만든 퍽 등록표\npucks:\n";
  for (const auto & t : slots) {
    if (!t["id"] || t["id"].IsNull() || !t["color"] || t["color"].IsNull() || !t["shape"] || t["shape"].IsNull()) {
      err = "slot without id · color · shape in " + path;
      return false;
    }
    const std::string shape = t["shape"].as<std::string>() == "cuboid" ? "box" : t["shape"].as<std::string>();
    o << "  - {id: " << t["id"].as<int>() << ", color: " << t["color"].as<std::string>() << ", shape: " << shape
      << ", box: " << t["box"].as<int>() << ", side: " << t["side"].as<int>();
    if (t["pose"] && !t["pose"].IsNull()) {
      const std::string pose = t["pose"].as<std::string>();
      if (pose != "standing" && pose != "lying") {err = "slot pose must be standing or lying (" + pose + ") in " + path; return false;}
      o << ", pose: " << pose;
    }
    if (t["measured"] && t["measured"].as<bool>() && t["x_mm"] && t["y_mm"]) {
      o << ", xy: [" << t["x_mm"].as<double>() / 1000.0 << ", " << t["y_mm"].as<double>() / 1000.0 << "]";
    }
    o << "}\n";
  }
  out = o.str();
  return true;
}

}  // namespace mission
