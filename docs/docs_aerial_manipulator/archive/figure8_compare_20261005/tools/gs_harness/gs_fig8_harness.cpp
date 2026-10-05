// gs_fig8_harness.cpp -- the REAL arm-GS "EE Trajectory" panel, offscreen, driven
// through the figure-8 hardware workflow by its own widgets (2026-10-05).
//
// Built against the station's joint_plot_core library (CMakeLists.txt beside this
// file); started by ../hw_fig8_workflow_check.py, which runs the real planner on the
// HARDWARE yaml and a fake vehicle under /uav_hwcheck on ROS_DOMAIN_ID 77. This
// program does what the operator does: reads the Figure-8 fields once the planner's
// parameters have loaded, selects Figure-8 in the combo (the panel then writes
// ee_traj_fig8_a/b, lap_time, laps and re-selects), and presses Go To Start, Start
// Trajectory, Back To Origin and Start Transition, each only once the panel has
// ENABLED that button. It prints "GSVAL <key> <value>" lines for the checker and
// saves two screenshots.
//
//   QT_QPA_PLATFORM=offscreen ./gs_fig8_harness <namespace> <png prefix>
#include <QApplication>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QPushButton>
#include <QSpinBox>
#include <QTimer>

#include <cstdio>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "ee_trajectory_panel.hpp"

int main(int argc, char * argv[])
{
  if (argc < 3) {
    std::fprintf(stderr, "usage: gs_fig8_harness <station namespace> <png prefix>\n");
    return 2;
  }
  const std::string ns = argv[1];
  const QString png = QString::fromLocal8Bit(argv[2]);
  rclcpp::init(1, argv);
  QApplication app(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("gs_fig8_harness", ns);

  // the planner's state, read the way the panel reads it (latched)
  std::string status, ee_status;
  const rclcpp::QoS latched = rclcpp::QoS(1).reliable().transient_local();
  std::string gov = ns.substr(0, ns.find_last_of('/'));
  auto s1 = node->create_subscription<std_msgs::msg::String>(
    gov + "/whole_body_planner/status", latched,
    [&status](const std_msgs::msg::String & m) {status = m.data;});
  auto s2 = node->create_subscription<std_msgs::msg::String>(
    gov + "/whole_body_planner/ee_trajectory/status", latched,
    [&ee_status](const std_msgs::msg::String & m) {ee_status = m.data;});

  auto * panel = new EeTrajectoryPanel(node);
  panel->resize(1500, 1100);
  panel->show();

  QTimer pump;
  QObject::connect(&pump, &QTimer::timeout, [&node]() {rclcpp::spin_some(node);});
  pump.start(5);

  const auto spin = [panel](const char * name) -> double {
      if (auto * d = panel->findChild<QDoubleSpinBox *>(name)) {return d->value();}
      if (auto * i = panel->findChild<QSpinBox *>(name)) {return i->value();}
      return -1.0;
    };
  const auto button = [panel](const QString & prefix) -> QPushButton * {
      for (auto * b : panel->findChildren<QPushButton *>()) {
        if (b->text().startsWith(prefix)) {return b;}
      }
      return nullptr;
    };
  const auto say = [](const char * key, const std::string & v) {
      std::printf("GSVAL %s %s\n", key, v.c_str());
      std::fflush(stdout);
    };

  int state = 0;
  int ticks = 0;
  bool seen_exec = false;
  QTimer step;
  QObject::connect(&step, &QTimer::timeout, [&]() {
      ++ticks;
      if (ticks > 3000) {             // 300 s
        say("FAIL", "timeout in state " + std::to_string(state) + " status " + status +
        " ee " + ee_status);
        QApplication::exit(1);
        return;
      }
      switch (state) {
        case 0:                       // the planner's parameters load after ~1 s; give it 8
          if (ticks < 80) {return;}
          say("fig8_a", std::to_string(spin("ee_traj_fig_a")));
          say("fig8_b", std::to_string(spin("ee_traj_fig_b")));
          say("fig8_vel", std::to_string(spin("ee_traj_vel_fig8")));
          say("fig8_laps", std::to_string(spin("ee_traj_laps_fig8")));
          say("circle_radius", std::to_string(spin("ee_traj_radius")));
          say("circle_vel", std::to_string(spin("ee_traj_vel_circle")));
          say("circle_laps", std::to_string(spin("ee_traj_laps_circle")));
          if (auto * c = panel->findChild<QComboBox *>("ee_traj_type")) {
            c->setCurrentIndex(2);    // the operator picks Figure-8
            say("selected", c->currentText().toStdString());
          }
          state = 1;
          return;
        case 1:
          if (ee_status.rfind("READY", 0) != 0) {return;}
          say("ee_status", ee_status);
          panel->grab().save(png + "_ready.png");
          state = 2;
          return;
        case 2:
          if (auto * b = button("Go To Start"); b && b->isEnabled()) {
            say("press", "Go To Start");
            b->click();
            seen_exec = false;
            state = 3;
          }
          return;
        case 3:                       // the transition flies, then HOLDs at the start rest
          if (status.rfind("EXECUTING", 0) == 0) {seen_exec = true;}
          if (seen_exec && status == "HOLD") {state = 4;}
          return;
        case 4:                       // the rig moves onto the start; the panel enables Start
          if (auto * b = button("Start Trajectory"); b && b->isEnabled()) {
            say("press", "Start Trajectory");
            b->click();
            seen_exec = false;
            state = 5;
          }
          return;
        case 5:
          if (status.rfind("EXECUTING", 0) == 0) {seen_exec = true;}
          if (seen_exec && status == "HOLD") {
            say("run", "complete");
            panel->grab().save(png + "_done.png");
            state = 6;
          }
          return;
        case 6:
          if (auto * b = button("Back To Origin"); b && b->isEnabled()) {
            say("press", "Back To Origin");
            b->click();
            state = 7;
          }
          return;
        case 7:                       // it stops at PLANNED; Start becomes Start Transition
          if (status.rfind("PLANNED", 0) != 0) {return;}
          if (auto * b = button("Start Transition"); b && b->isEnabled()) {
            say("press", "Start Transition");
            b->click();
            seen_exec = false;
            state = 8;
          }
          return;
        case 8:
          if (status.rfind("EXECUTING", 0) == 0) {seen_exec = true;}
          if (seen_exec && status == "HOLD") {
            say("origin", "complete");
            say("DONE", "ok");
            QApplication::exit(0);
          }
          return;
        default:
          return;
      }
    });
  step.start(100);

  const int ret = app.exec();
  rclcpp::shutdown();
  return ret;
}
