// gs_auto_exit_harness.cpp -- the arm GS "Pick & Drop" tab's AUTOMATIC Exit To Drop
// (2026-10-15), offscreen, against a FAKE planner (no Isaac, no real planner):
//
//   1. the fake planner holds ON the drop point (Execute To Drop done, holding
//      there): the tab's 4.2 Drop button is enabled; the Drop Exit row is gone
//      and the Drop Margin button is there;
//   2. the harness clicks 4.2 Drop: the tab asks for the gripper OPEN; the harness
//      answers like the station (onGripperOpenFinished) and the tab must call the
//      planner's exit_place BY ITSELF, exactly once;
//   3. a second, unrelated OPEN result must NOT call it again;
//   4. the fake planner then reports the exit done ON LAND START (exit_place
//      flown, Go To Land Start done): the tab must ask for the gripper CLOSE.
//   5. THE BACK MARGIN (2026-10-15): its button sits under the Drop Margin, in
//      the same column as Vertical / Side Margin; it loads the planner's
//      pick_place_drop_exit_back and its button writes the typed value.
//
// Prints "GSVAL <key> <value>" lines.
//
//   QT_QPA_PLATFORM=offscreen ROS_DOMAIN_ID=77 ./gs_auto_exit_harness
#include <QApplication>
#include <QDoubleSpinBox>
#include <QGridLayout>
#include <QLabel>
#include <QPushButton>
#include <QTimer>

#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "pick_place_panel.hpp"

namespace
{
void say(const std::string & key, const std::string & v)
{
  std::printf("GSVAL %s %s\n", key.c_str(), v.c_str());
  std::fflush(stdout);
}

QPushButton * buttonStarting(QWidget * panel, const QString & prefix)
{
  for (auto * b : panel->findChildren<QPushButton *>()) {
    if (b->text().startsWith(prefix)) {return b;}
  }
  return nullptr;
}

// (row, column) of a widget in its grid; (-1, -1) when it is not in one
std::pair<int, int> gridCell(QWidget * w)
{
  if (!w || !w->parentWidget()) {return {-1, -1};}
  auto * g = qobject_cast<QGridLayout *>(w->parentWidget()->layout());
  if (!g) {return {-1, -1};}
  const int i = g->indexOf(w);
  if (i < 0) {return {-1, -1};}
  int r, c, rs, cs;
  g->getItemPosition(i, &r, &c, &rs, &cs);
  return {r, c};
}

// the spin box right of a button in the same grid row
QDoubleSpinBox * boxRightOf(QWidget * b)
{
  const auto rc = gridCell(b);
  auto * g = qobject_cast<QGridLayout *>(b->parentWidget()->layout());
  if (!g || rc.first < 0) {return nullptr;}
  auto * it = g->itemAtPosition(rc.first, rc.second + 1);
  return it ? qobject_cast<QDoubleSpinBox *>(it->widget()) : nullptr;
}

bool hasLabel(QWidget * panel, const QString & text)
{
  for (auto * l : panel->findChildren<QLabel *>()) {
    if (l->text() == text) {return true;}
  }
  return false;
}
}  // namespace

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  QApplication app(argc, argv);
  const std::string ns = "/uav_gscheck";

  // ---- the fake planner: its parameters, its pick_place topics, exit_place ----
  auto planner = std::make_shared<rclcpp::Node>(
    "whole_body_trajectory_planner", ns,
    rclcpp::NodeOptions().allow_undeclared_parameters(true));
  planner->declare_parameter<bool>("pick_place_place_drop", true);
  planner->declare_parameter<double>("pick_place_drop_height", 0.10);
  planner->declare_parameter<double>("pick_place_drop_altitude", 0.0);
  planner->declare_parameter<double>("pick_place_drop_exit_back", 0.20);
  planner->declare_parameter<double>("pick_place_approach_dz", 0.20);
  planner->declare_parameter<double>("pick_place_pick_approach_back", 0.10);
  planner->declare_parameter<double>("pick_place_place_exit_back", 0.10);
  planner->declare_parameter<std::vector<double>>("pick_place_ee_offset", {-0.02, 0.0, 0.16});
  for (const char * n : {"pick_place_start", "pick_place_place_start", "pick_place_place_point",
      "pick_place_land_start", "pick_place_land"})
  {
    planner->declare_parameter<std::vector<double>>(n, {0.0, 0.0, 1.0, 0.0});
  }
  const auto latched = rclcpp::QoS(1).transient_local();
  const std::string base = ns + "/whole_body_planner/pick_place";
  auto info_pub = planner->create_publisher<std_msgs::msg::Float64MultiArray>(base + "/info", latched);
  auto pp_status_pub = planner->create_publisher<std_msgs::msg::String>(base + "/status", latched);
  auto status_pub = planner->create_publisher<std_msgs::msg::String>(
    ns + "/whole_body_planner/status", latched);
  // the planner's state, as the info block carries it
  struct State
  {
    double done = 3, at_table = 1, exiting = -1, exited_place = 0;
    std::string pp = "DONE execute_place next=exit_place (open the gripper first)";
  } st;
  int exit_calls = 0;
  auto exit_srv = planner->create_service<std_srvs::srv::Trigger>(
    base + "/exit_place",
    [&](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> r) {
      ++exit_calls;
      r->success = true;
      r->message = "fake: exit_place";
    });
  auto publish = [&]() {
      std_msgs::msg::Float64MultiArray m;
      m.data.assign(88, 0.0);
      for (int k = 8; k < 14; ++k) {m.data[k] = 5.0;}    // leg durations
      m.data[4] = 1.0;                                     // planned
      m.data[5] = st.done;
      m.data[6] = -1.0;                                    // no leg flying
      m.data[83] = 1.0;                                    // exit_pick flown
      m.data[84] = st.exited_place;
      m.data[85] = st.at_table;
      m.data[86] = st.exiting;
      info_pub->publish(m);
      std_msgs::msg::String s;
      s.data = st.pp;
      pp_status_pub->publish(s);
      s.data = "HOLD";
      status_pub->publish(s);
    };

  // ---- the station's node and the REAL Pick & Drop panel ----
  auto node = std::make_shared<rclcpp::Node>("arm_gs_harness", ns + "/fsc_open_manipulator");
  auto * panel = new PickPlacePanel(node, true);
  panel->resize(1600, 1100);
  panel->show();
  int opens = 0, closes = 0;
  QObject::connect(panel, &PickPlacePanel::gripperRequested,
    [&](bool open) {
      if (open) {
        ++opens;
        // the station's GripperCommand OPEN succeeds a moment later
        QTimer::singleShot(300, [&]() {panel->onGripperOpenFinished();});
      } else {
        ++closes;
      }
    });

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(planner);
  exec.add_node(node);
  QTimer spin;
  QObject::connect(&spin, &QTimer::timeout, [&]() {exec.spin_some(std::chrono::milliseconds(0));});
  spin.start(10);
  QTimer pub;
  QObject::connect(&pub, &QTimer::timeout, publish);
  pub.start(200);

  int step = 0;
  bool back_layout_ok = false, back_load_ok = false;
  QDoubleSpinBox * back_box = nullptr;
  QTimer script;
  QObject::connect(&script, &QTimer::timeout, [&]() {
      ++step;
      if (step == 40) {   // 4 s: parameters loaded, the hold on the drop point seen
        // 5. the margins column: Vertical, Side, (gap), Drop, Back
        const auto vm = gridCell(buttonStarting(panel, "Vertical Margin"));
        const auto sm = gridCell(buttonStarting(panel, "Side Margin"));
        const auto dm = gridCell(buttonStarting(panel, "Drop Margin"));
        const auto bm = gridCell(buttonStarting(panel, "Back Margin"));
        say("margin_cells", "V(" + std::to_string(vm.first) + "," + std::to_string(vm.second) +
          ") S(" + std::to_string(sm.first) + "," + std::to_string(sm.second) +
          ") D(" + std::to_string(dm.first) + "," + std::to_string(dm.second) +
          ") B(" + std::to_string(bm.first) + "," + std::to_string(bm.second) + ")");
        back_layout_ok = bm.first == dm.first + 1 && bm.second == dm.second &&
          vm.second == dm.second && sm.second == dm.second && sm.first == vm.first + 1 &&
          dm.first > sm.first + 1;
        say("back_margin_under_drop_margin", back_layout_ok ? "yes" : "NO");
        back_box = boxRightOf(buttonStarting(panel, "Back Margin"));
        say("back_margin_loaded", back_box ? QString::number(back_box->value(), 'f', 2).toStdString() : "no box");
        back_load_ok = back_box && std::abs(back_box->value() - 0.20) < 1e-9 && back_box->isEnabled() &&
          back_box->styleSheet().isEmpty();
        auto * drop = buttonStarting(panel, "4.2");
        say("drop_exit_row_gone", hasLabel(panel, "Drop Exit Δ") ? "NO" : "yes");
        say("drop_margin_button", buttonStarting(panel, "Drop Margin") ? "yes" : "NO");
        say("drop_button_enabled", drop && drop->isEnabled() ? "yes" : "NO");
        if (drop) {drop->click();}
      } else if (step == 60) {   // 2 s later
        say("gripper_open_requests", std::to_string(opens));
        say("exit_place_calls_after_drop", std::to_string(exit_calls));
        // a second, unrelated OPEN result: no second exit
        panel->onGripperOpenFinished();
      } else if (step == 70) {
        say("exit_place_calls_after_stray_open", std::to_string(exit_calls));
        // the exit is flying, then done ON LAND START (Go To Land Start done)
        st.exiting = 3;
        st.at_table = 0;
        st.pp = "FLYING exit_place";
      } else if (step == 85) {
        st.exiting = -1;
        st.exited_place = 1;
        st.done = 4;
        st.pp = "DONE go_to_land_start next=execute_land";
      } else if (step == 90) {
        // type 0.25 into the Back Margin and press its button
        if (back_box) {back_box->setValue(0.25);}
        say("back_margin_amber_when_typed", back_box && !back_box->styleSheet().isEmpty() ? "yes" : "NO");
        if (auto * b = buttonStarting(panel, "Back Margin")) {b->click();}
      } else if (step == 105) {
        say("gripper_close_requests_at_land_start", std::to_string(closes));
        const double written = planner->get_parameter("pick_place_drop_exit_back").as_double();
        say("back_margin_written", QString::number(written, 'f', 2).toStdString());
        const bool back_ok = back_layout_ok && back_load_ok && std::abs(written - 0.25) < 1e-9 &&
          back_box && back_box->styleSheet().isEmpty();
        say("back_margin", back_ok ? "PASS" : "FAIL");
        const bool ok = opens == 1 && exit_calls == 1 && closes == 1 && back_ok;
        say("RESULT", ok ? "PASS" : "FAIL");
        app.quit();
      }
    });
  script.start(100);
  const int rc = app.exec();
  rclcpp::shutdown();
  return rc;
}
