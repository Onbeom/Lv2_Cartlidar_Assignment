// Nav2 서버 · bt_navigator · lifecycle_manager 를 한 프로세스에 (#276, 2026-11-15)
//
// 왜: Jetson 에서 bt_navigator 가 활성화 때 BT 를 만들며 planner_server 의 is_path_valid · compute_path_to_pose 를
//   DDS 찾기로 제때 못 찾아 (15 s 기다려도) Nav2 전체가 멈춤. 같은 프로세스 = 같은 DDS 참가자라 찾기 지연이 없다.
// 왜 component_container 가 아닌가: launch 가 load_node 서비스로 노드를 싣는데, 그 응답이 같은 DDS 응답 유실
//   ("failed to send response … client will not receive response") 로 사라지면 launch 가 멈춤 (PC 에서 첫 시험에 재현).
//   여기서는 서비스 없이 class_loader 로 컴포넌트를 직접 만든다 (component_container_isolated 와 같은 실행 방식: 노드마다 실행기 스레드).
//
// 사용 (navigation.launch.py 가 부름):
//   nav2_composed --node <패키지>/<클래스>:<이름> … --manager <패키지>/<클래스>:<이름> --delay <s> --ros-args --params-file <노드별 yaml> -r …
//   매개변수 · 리매핑은 --ros-args (프로세스 전체) 로 준다. 노드별 값은 yaml 의 노드 이름 칸, 리매핑은 <노드>:<원래>:=<새것> 규칙으로 가른다.
#include <chrono>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "ament_index_cpp/get_resource.hpp"
#include "class_loader/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/node_factory.hpp"

namespace
{

struct Spec
{
  std::string pkg, cls, name;
};

// "nav2_planner/nav2_planner::PlannerServer:planner_server"
Spec parse_spec(const std::string & s)
{
  const auto slash = s.find('/');
  const auto colon = s.rfind(':');
  if (slash == std::string::npos || colon == std::string::npos || colon <= slash) {
    throw std::invalid_argument("노드 지정 형식은 <패키지>/<클래스>:<이름> : " + s);
  }
  return {s.substr(0, slash), s.substr(slash + 1, colon - slash - 1), s.substr(colon + 1)};
}

// ament 색인 rclcpp_components/<패키지> 에서 클래스의 라이브러리 경로 (component_container 와 같은 방법)
std::string find_library(const Spec & sp)
{
  const auto res = ament_index_cpp::get_resource("rclcpp_components", sp.pkg);
  if (!res.resourcePath) {
    throw std::runtime_error("컴포넌트 색인에 패키지 없음: " + sp.pkg);
  }
  const std::string base = res.resourcePath->string();
  std::istringstream in(res.contents);
  std::string line;
  while (std::getline(in, line)) {
    const auto semi = line.find(';');
    if (semi == std::string::npos || line.substr(0, semi) != sp.cls) {continue;}
    std::string lib = line.substr(semi + 1);
    return lib.front() == '/' ? lib : base + "/" + lib;
  }
  throw std::runtime_error("패키지 " + sp.pkg + " 에 컴포넌트 " + sp.cls + " 없음");
}

class Composed
{
public:
  rclcpp_components::NodeInstanceWrapper load(const Spec & sp)
  {
    auto loader = std::make_unique<class_loader::ClassLoader>(find_library(sp));
    auto factory = loader->createInstance<rclcpp_components::NodeFactory>(
      "rclcpp_components::NodeFactoryTemplate<" + sp.cls + ">");
    // 프로세스 --ros-args (매개변수 파일 · 리매핑) 는 그대로 쓰고, 이름만 노드마다
    auto opts = rclcpp::NodeOptions().use_global_arguments(true)
      .arguments({"--ros-args", "-r", "__node:=" + sp.name});
    auto w = factory->create_node_instance(opts);
    loaders_.push_back(std::move(loader));
    auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    exec->add_node(w.get_node_base_interface());
    threads_.emplace_back([exec] {exec->spin();});
    executors_.push_back(exec);
    nodes_.push_back(w);
    RCLCPP_INFO(rclcpp::get_logger("nav2_composed"), "실음: %s (%s)", sp.name.c_str(), sp.cls.c_str());
    return w;
  }

  ~Composed()
  {
    for (auto & e : executors_) {e->cancel();}
    for (auto & t : threads_) {if (t.joinable()) {t.join();}}
    executors_.clear();
    nodes_.clear();   // 노드를 먼저 지우고 라이브러리를 내림
    loaders_.clear();
  }

private:
  std::vector<std::unique_ptr<class_loader::ClassLoader>> loaders_;
  std::vector<rclcpp_components::NodeInstanceWrapper> nodes_;
  std::vector<std::shared_ptr<rclcpp::executors::SingleThreadedExecutor>> executors_;
  std::vector<std::thread> threads_;
};

}  // namespace

int main(int argc, char ** argv)
{
  const auto args = rclcpp::init_and_remove_ros_arguments(argc, argv);
  std::vector<Spec> nodes;
  std::vector<Spec> managers;
  double delay = 0.0;
  try {
    for (size_t i = 1; i < args.size(); ++i) {
      const std::string & a = args[i];
      if ((a == "--node" || a == "--manager" || a == "--delay") && i + 1 >= args.size()) {
        throw std::invalid_argument(a + " 뒤 값 없음");
      }
      if (a == "--node") {nodes.push_back(parse_spec(args[++i]));}
      else if (a == "--manager") {managers.push_back(parse_spec(args[++i]));}
      else if (a == "--delay") {delay = std::stod(args[++i]);}
      else {throw std::invalid_argument("모르는 인자: " + a);}
    }
    if (nodes.empty()) {throw std::invalid_argument("--node 가 하나도 없음");}
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("nav2_composed"), "%s", e.what());
    rclcpp::shutdown();
    return 2;
  }

  int rc = 0;
  {
    Composed c;
    try {
      for (const auto & sp : nodes) {c.load(sp);}
      // 켜 주는 노드는 서버가 다 실린 뒤 delay 만큼 늦게 (예전 lifecycle_delay 와 같은 뜻)
      const auto until = std::chrono::steady_clock::now() + std::chrono::duration<double>(delay);
      while (rclcpp::ok() && std::chrono::steady_clock::now() < until) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
      for (const auto & sp : managers) {
        if (rclcpp::ok()) {c.load(sp);}
      }
      while (rclcpp::ok()) {std::this_thread::sleep_for(std::chrono::milliseconds(100));}
    } catch (const std::exception & e) {
      RCLCPP_FATAL(rclcpp::get_logger("nav2_composed"), "노드를 싣지 못함: %s", e.what());
      rc = 1;
    }
    if (rclcpp::ok()) {rclcpp::shutdown();}   // 실행기 멈춤 → 노드 정리
  }
  return rc;
}
