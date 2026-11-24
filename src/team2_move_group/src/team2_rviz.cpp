#include <csignal>
#include <memory>
#include <string>
#include <vector>
#include <iostream>
#include <QApplication>
#include <QTimer>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/ros_integration/ros_client_abstraction.hpp>
#include <rviz_common/visualization_frame.hpp>

namespace
{
volatile std::sig_atomic_t stopping = 0;
void requestStop(int) { stopping = 1; }
}

int main(int argc, char **argv)
{
  auto args = rclcpp::remove_ros_arguments(argc, argv);
  std::vector<char *> qt_args;
  QString config;
  for (size_t i = 0; i < args.size(); ++i)
  {
    qt_args.push_back(args[i].data());
    if (args[i] == "-d" && i + 1 < args.size())
      config = QString::fromStdString(args[i + 1]);
  }
  qt_args.push_back(nullptr);
  int qt_argc = static_cast<int>(args.size());
  QApplication app(qt_argc, qt_args.data());
  rviz_common::ros_integration::RosClientAbstraction ros;
  auto node = ros.init(argc, argv, "rviz2_moveit", false);
  // ROS 기본 신호 처리 대신 GUI 스레드가 닫도록 해 화면 정리 동안 context를 유지한다.
  std::signal(SIGINT, requestStop);
  std::signal(SIGTERM, requestStop);
  auto frame = std::make_unique<rviz_common::VisualizationFrame>(node);
  frame->setApp(&app);
  frame->initialize(node, config);
  frame->show();
  QTimer timer;
  QObject::connect(&timer, &QTimer::timeout, [&]() {
    if (stopping)
    {
      frame->setWindowModified(false);
      app.quit();
    }
  });
  timer.start(100);
  std::cout << "Team 2 RViz ready" << std::endl;
  int result = app.exec();
  timer.stop();
  // 구독·MoveIt 플러그인과 화면을 먼저 해제하고 ROS 노드/context는 마지막에 종료한다.
  frame.reset();
  ros.shutdown();
  std::cout << "Team 2 RViz released before ROS shutdown" << std::endl;
  return result;
}
