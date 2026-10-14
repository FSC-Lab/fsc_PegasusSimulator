// gs_drop_harness.cpp -- the arm GS "Pick & Place" and "Pick & Drop" tabs, offscreen
// (2026-10-14). Run by ../gs_drop_check.sh on a private ROS domain with the REAL
// planner up (the hardware 4-D yaml: pick_place_place_drop true):
//
//   1. the planner flies the DROP place: the Pick & Drop tab is active, the
//      Pick & Place tab says the planner is in pick-and-drop mode;
//   2. a planner ABORT (ABORTING -> ABORTED on pick_place/status, published here):
//      only the ACTIVE tab commands the gripper (open, then close);
//   3. Plan on Pick & Place switches the planner's pick_place_place_drop to false
//      (then plans; the planner refuses that -- not in DIRECT -- which is fine);
//      the activity swaps;
//   4. the same abort: now only Pick & Place commands the gripper;
//   5. Plan on Pick & Drop switches it back to true.
//
// Prints "GSVAL <key> <value>" lines and saves both tabs as PNGs.
//
//   QT_QPA_PLATFORM=offscreen ./gs_drop_harness <station ns> <png prefix>
#include <QApplication>
#include <QLabel>
#include <QPushButton>
#include <QTimer>

#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "pick_place_panel.hpp"

namespace
{
void say(const std::string & key, const std::string & v)
{
  std::printf("GSVAL %s %s\n", key.c_str(), v.c_str());
  std::fflush(stdout);
}

// the status lamp of an INACTIVE tab starts with this (pick_place_panel.cpp)
bool says_other_mode(QWidget * panel)
{
  for (auto * l : panel->findChildren<QLabel *>()) {
    if (l->text().startsWith("The planner is in")) {return true;}
  }
  return false;
}

QPushButton * button(QWidget * panel, const QString & text)
{
  for (auto * b : panel->findChildren<QPushButton *>()) {
    if (b->text() == text) {return b;}
  }
  return nullptr;
}
}  // namespace

int main(int argc, char * argv[])
{
  if (argc < 3) {
    std::fprintf(stderr, "usage: gs_drop_harness <station namespace> <png prefix>\n");
    return 2;
  }
  const std::string ns = argv[1];
  const QString png = QString::fromLocal8Bit(argv[2]);
  rclcpp::init(1, argv);
  QApplication app(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("gs_drop_harness", ns);
  const std::string gov = ns.substr(0, ns.find_last_of('/'));

  auto * place = new PickPlacePanel(node, false);
  auto * drop = new PickPlacePanel(node, true);
  for (auto * p : {place, drop}) {p->resize(1500, 1100); p->show();}
  int place_open = 0, place_close = 0, drop_open = 0, drop_close = 0;
  QObject::connect(place, &PickPlacePanel::gripperRequested,
    [&](bool open) {(open ? place_open : place_close)++;});
  QObject::connect(drop, &PickPlacePanel::gripperRequested,
    [&](bool open) {(open ? drop_open : drop_close)++;});

  const rclcpp::QoS latched = rclcpp::QoS(1).reliable().transient_local();
  auto status_pub = node->create_publisher<std_msgs::msg::String>(
    gov + "/whole_body_planner/pick_place/status", latched);
  auto params = std::make_shared<rclcpp::AsyncParametersClient>(
    node, gov + "/whole_body_trajectory_planner");
  int drop_param = -1;          // -1 unknown, 0 false, 1 true
  bool reading = false;
  const auto read = [&]() {
      if (reading || !params->service_is_ready()) {return;}
      reading = true;
      params->get_parameters({"pick_place_place_drop"},
        [&](std::shared_future<std::vector<rclcpp::Parameter>> f) {
          const auto v = f.get();
          if (v.size() == 1 && v[0].get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
            drop_param = v[0].as_bool() ? 1 : 0;
          }
          reading = false;
        });
    };
  // ABORTING, then ABORTED half a second later (the status QoS keeps ONE
  // message: back to back, the panel would only ever see the second)
  const auto abort_cycle = [&]() {
      std_msgs::msg::String m;
      m.data = "ABORTING: harness";
      status_pub->publish(m);
      QTimer::singleShot(500, [status_pub]() {
          std_msgs::msg::String a;
          a.data = "ABORTED -- harness";
          status_pub->publish(a);
        });
    };

  QTimer pump;
  QObject::connect(&pump, &QTimer::timeout, [&node]() {rclcpp::spin_some(node);});
  pump.start(5);

  int state = 0, ticks = 0, wait = 0, rc = 0;
  bool fail = false;
  const auto check = [&](const std::string & key, bool ok, const std::string & detail) {
      say(key, std::string(ok ? "ok " : "WRONG ") + detail);
      if (!ok) {fail = true;}
    };
  QTimer step;
  QObject::connect(&step, &QTimer::timeout, [&]() {
      ++ticks;
      read();
      if (wait > 0) {--wait; return;}
      const std::string counts = "place open/close " + std::to_string(place_open) + "/" +
        std::to_string(place_close) + ", drop " + std::to_string(drop_open) + "/" +
        std::to_string(drop_close);
      if (state == 0 && drop_param == 1 && says_other_mode(place)) {
        check("1_drop_mode", !says_other_mode(drop), "planner drop=true: Pick & Drop active, "
          "Pick & Place says the other mode");
        place->grab().save(png + "_place_inactive.png");
        drop->grab().save(png + "_drop_active.png");
        abort_cycle();
        state = 1; wait = 30;
      } else if (state == 1) {
        check("2_abort_drop_active", drop_open == 1 && drop_close == 1 && place_open == 0 &&
          place_close == 0, counts);
        QPushButton * b = button(place, "Plan");
        if (!b) {say("error", "no Plan button"); app.exit(1); return;}
        Q_EMIT b->clicked(false);      // (disabled: the planner is not in DIRECT)
        state = 2;
      } else if (state == 2 && drop_param == 0 && says_other_mode(drop)) {
        check("3_plan_switches_to_place", !says_other_mode(place),
          "planner drop=false after Pick & Place's Plan; activity swapped");
        place->grab().save(png + "_place_active.png");
        abort_cycle();
        state = 3; wait = 30;
      } else if (state == 3) {
        check("4_abort_place_active", place_open == 1 && place_close == 1 && drop_open == 1 &&
          drop_close == 1, counts);
        Q_EMIT button(drop, "Plan")->clicked(false);
        state = 4;
      } else if (state == 4 && drop_param == 1 && says_other_mode(place)) {
        check("5_plan_switches_to_drop", !says_other_mode(drop), "planner drop=true again");
        say("result", fail ? "FAIL" : "PASS");
        rc = fail ? 1 : 0;
        app.exit(rc);
      }
      if (ticks > 1200) {say("result", "TIMEOUT state " + std::to_string(state)); app.exit(1);}
    });
  step.start(50);
  const int r = app.exec();
  rclcpp::shutdown();
  return r ? r : rc;
}
