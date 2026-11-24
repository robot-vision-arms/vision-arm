// Team 2 Gazebo server lifecycle wrapper. No Gazebo/UR plugin sources are modified.
#include <gz/sim/Server.hh>
#include <gz/sim/ServerConfig.hh>
#include <gz/common/Console.hh>
#include <rclcpp/rclcpp.hpp>
#include <pal_statistics/pal_statistics_macros.hpp>
#include <csignal>
#include <cstdio>
#include <chrono>
#include <memory>
#include <thread>

namespace { volatile std::sig_atomic_t stopping = 0; void stop_requested(int) { stopping = 1; } }

int main(int argc, char** argv)
{
  if (argc < 2) return 2;
  // ROS 컨트롤러가 이 context를 공유하게 하고 자동 SIGINT 해제 순서에 의존하지 않는다.
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  gz::common::Console::SetVerbosity(4);
  gz::sim::ServerConfig config;
  config.SetSdfFile(argv[1]);
  auto server = std::make_unique<gz::sim::Server>(config);
  if (!server->Run(false, 0, false)) { rclcpp::shutdown(); return 3; }
  // Gazebo가 설치한 핸들러 대신 중지 의사만 기록한다. 실제 정리는 main thread가 한다.
  std::signal(SIGINT, stop_requested);
  std::signal(SIGTERM, stop_requested);
  while (!stopping && server->Running()) std::this_thread::sleep_for(std::chrono::milliseconds(50));
  // 핵심: gz_ros2_control의 executor를 깨운 다음 플러그인을 파괴한다.
  // 서버/플러그인 파괴부터 하면 살아 있는 ROS 스레드를 기다리며 종료가 멈출 수 있다.
  // 통계 스레드도 ROS clock으로 잠든다. clock/context가 살아 있을 때 먼저 join한다.
  // registry 자체는 지우지 않아 controller_manager의 정상 해제 순서를 유지한다.
  for (const auto * key : {"ros2_control", "cm_execution_statistics"})
    if (auto registry = pal_statistics::getRegistry(key)) registry->stopPublisherThread();
  rclcpp::shutdown();
  server->Stop();
  server.reset();
  std::fprintf(stderr, "Team2 Gazebo: ROS context and server released cleanly\n");
  return 0;
}
