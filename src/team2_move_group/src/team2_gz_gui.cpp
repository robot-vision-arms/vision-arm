#include <csignal>
#include <iostream>
#include <QTimer>
#include <gz/sim/gui/Gui.hh>

namespace
{
volatile std::sig_atomic_t stopping = 0;
void requestStop(int) { stopping = 1; }
}

int main(int argc, char **argv)
{
  // gz 도구의 Ruby 로더 대신 공식 C++ GUI API를 직접 호출한다.
  // discovery 스레드가 실행되는 중 로더가 환경 변수를 바꾸는 경로를 피한다.
  const char *world = argc > 1 ? argv[1] : nullptr;
  int gui_argc = 1;
  std::signal(SIGINT, requestStop);
  std::signal(SIGTERM, requestStop);
  auto app = gz::sim::gui::createGui(gui_argc, argv, nullptr, nullptr, true, world, 0);
  if (!app)
    return 1;
  // 신호 처리기에서는 Qt 객체를 만지지 않고 GUI 스레드에서 정상 종료한다.
  QTimer timer;
  QObject::connect(&timer, &QTimer::timeout, [&app]() {
    if (stopping)
      app->quit();
  });
  timer.start(100);
  std::cout << "Team 2 Gazebo GUI ready" << std::endl;
  const int result = app->exec();
  app.reset();
  std::cout << "Team 2 Gazebo GUI released cleanly" << std::endl;
  return result;
}
