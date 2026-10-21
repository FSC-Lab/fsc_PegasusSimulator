// gs_wf_snapshot.cpp -- the REAL arm-GS "Box Pushing" panel, offscreen, against a FAKE
// planner (no Isaac, no PX4, no planner node), for the box-pushing workflow page
// (~/Documents/box_pushing_workflow, 2026-10-21). Run by ../gs_wf_snapshot.sh on a
// private ROS domain.
//
// One scene: the push has just finished (the planner's FLYING push -> DONE push edge),
// so the Mission row shows "Exit To Push starts by itself in 1.0 s". The fake planner
// carries the 2026-10-21 numbers: the start mark (-0.60, 0), the box (0.675, 0.28), a
// 0.10 m push at 21 mm/s (+ the 1.5 s hold = 6.3 s), Land Start 1.30 m / Land 0.30 m,
// and the step times the real planner gave for that layout (layout_waypoints.json).
// The Orin's raw-mocap guard is played as healthy (MOCAP, 120 Hz).
//
// Writes <out>/gs_wf_auto_exit_full.png and prints the Mission row's rectangle in panel
// pixels ("GSVAL mission_rect x y w h") for the crop.
#include <QApplication>
#include <QPushButton>
#include <QTimer>

#include <chrono>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "control_panel.hpp"
#include "push_pull_panel.hpp"

namespace
{
using Trigger = std_srvs::srv::Trigger;
const double NaN = std::numeric_limits<double>::quiet_NaN();

void say(const std::string & key, const std::string & v)
{
  std::printf("GSVAL %s %s\n", key.c_str(), v.c_str());
  std::fflush(stdout);
}

// the steps' planned durations [s]: the real planner on the 2026-10-21 layout
// (layout_waypoints.json), the push for 0.10 m at 21 mm/s: 1.5 s hold + 4.76 s
const double kT[6] = {10.65, 3.16, 1.5 + 0.10 / 0.021, 6.0, 23.94, 7.43};
// per step [goal base x, y, z, yaw deg, claw x, y, z] (layout_waypoints.json; push and
// exit moved for 0.10 m instead of 0.50 m)
const double kGoal[6][7] = {
  {0.6766, 0.6671, 1.3148, -90.0, 0.675, 0.40, 1.1832},
  {0.6766, 0.5471, 1.1648, -90.0, 0.675, 0.28, 1.0332},
  {0.6766, 0.4471, 1.1648, -90.0, 0.675, 0.18, 1.0332},
  {0.6766, 0.4471, 1.3648, -90.0, 0.675, 0.1942, 1.3041},
  {-0.60, 0.0, 1.30, -360.0, -0.3471, -0.0016, 1.2393},
  {-0.60, 0.0, 0.30, -360.0, -0.3471, -0.0016, 0.2393}};
// the Orin's estimator, healthy (the layout in push_pull_panel.cpp; gs_mocap_harness)
const std::vector<double> kMocap{0, 0, 0.008, 0, 1, 0.008, NaN, 120, 0, 0, 0, 0, 0, NaN, NaN, NaN};
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  QApplication app(argc, argv);
  const std::string out = argc > 1 ? argv[1] : ".";
  const std::string gov = "/uav_gswf";

  auto node = std::make_shared<rclcpp::Node>("gs_wf_snapshot", gov + "/fsc_open_manipulator");
  node->declare_parameter<bool>("simulation", false);        // the hardware station
  node->declare_parameter<double>("push_pull_auto_exit_delay_s", 1.0);
  // the fake planner: the parameters the tab loads
  auto planner = std::make_shared<rclcpp::Node>("whole_body_trajectory_planner", gov);
  planner->declare_parameter<std::vector<double>>("push_pull_start_mark", {-0.6, 0.0, 1.0, 0.0});
  planner->declare_parameter<std::vector<double>>("push_pull_land_start", {-0.6, 0.0, 1.3, -360.0});
  planner->declare_parameter<std::vector<double>>("push_pull_land", {-0.6, 0.0, 0.3, -360.0});
  planner->declare_parameter<std::vector<double>>("push_pull_ee_offset", {0.0, 0.0, 0.183});
  planner->declare_parameter<std::vector<double>>("push_pull_box_point", {0.675, 0.28, 0.85, 0.0});
  planner->declare_parameter<std::vector<double>>("push_pull_table", {0.600, 0.750, -0.5, 0.5, 0.80});
  planner->declare_parameter<double>("push_pull_approach_dz", 0.15);
  planner->declare_parameter<double>("push_pull_approach_back", 0.12);
  planner->declare_parameter<double>("push_pull_push_distance", 0.10);
  planner->declare_parameter<double>("push_pull_turn_deg", 0.0);
  planner->declare_parameter<double>("push_pull_table_clearance", 0.05);
  planner->declare_parameter<double>("push_pull_gear_depth", 0.248);
  planner->declare_parameter<double>("push_pull_gear_radius", 0.20);
  planner->declare_parameter<double>("push_pull_table_margin_xy", 0.05);

  const std::string base = gov + "/whole_body_planner/push_pull";
  const auto latched = rclcpp::QoS(1).reliable().transient_local();
  auto pl_status = planner->create_publisher<std_msgs::msg::String>(base + "/status", latched);
  auto status = planner->create_publisher<std_msgs::msg::String>(gov + "/whole_body_planner/status", latched);
  auto info = planner->create_publisher<std_msgs::msg::Float64MultiArray>(base + "/info", latched);
  std::vector<rclcpp::ServiceBase::SharedPtr> srvs;
  for (const char * svc : {"ready", "push", "exit", "release", "abort"}) {
    srvs.push_back(planner->create_service<Trigger>(
        base + "/" + std::string(svc), [](const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr r) {
          r->success = true;
          r->message = "(fake) ok";
        }));
  }
  auto guard_pub = planner->create_publisher<std_msgs::msg::Float64MultiArray>(
    gov + "/state_estimator/mocap_guard", rclcpp::SensorDataQoS());
  auto guard_tick = planner->create_wall_timer(std::chrono::milliseconds(20), [&]() {
      std_msgs::msg::Float64MultiArray m;
      m.data = kMocap;
      guard_pub->publish(m);
    });

  // the planner state: pushed = false -> the push flying; true -> done
  const auto publish = [&](bool pushed) {
      std_msgs::msg::Float64MultiArray m;
      m.data = {0.0, 0.0, 0.0, 0.0, 1.0, pushed ? 2.0 : 1.0, pushed ? -1.0 : 2.0, 0.02};
      for (double t : kT) {m.data.push_back(t);}
      m.data.insert(m.data.end(), {1.0, 0.675, 0.28, 0.85, 0.0});
      for (const auto & g : kGoal) {m.data.insert(m.data.end(), g, g + 7);}
      m.data.insert(m.data.end(), {1.0, pushed ? 0.0 : 1.0, 1.0, 0.10, 0.4, -90.0, 0.0, 0.0});
      m.data.insert(m.data.end(), {0.021, 3.0, 1.5, 0.10 / 0.021});
      m.data.insert(m.data.end(), {0.10, 0.50, 0.008, 0.030});
      info->publish(m);
      std_msgs::msg::String s;
      s.data = pushed ? "DONE push next=exit (the gripper opens first)"
                      : "FLYING push (CONTACT) T=6.3s";
      pl_status->publish(s);
      s.data = "HOLD";
      status->publish(s);
    };
  publish(false);

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(planner);
  QTimer spin;
  QObject::connect(&spin, &QTimer::timeout, [&]() {exec.spin_some(std::chrono::milliseconds(0));});
  spin.start(5);

  auto * panel = new PushPullPanel(node);
  panel->resize(1300, 900);
  panel->show();

  // let discovery, the latched topics and the parameter load settle, then end the push
  QTimer::singleShot(3000, &app, [&]() {
      publish(true);
      QTimer::singleShot(350, &app, [&]() {
          panel->grab().save(QString::fromStdString(out + "/gs_wf_auto_exit_full.png"));
          QPushButton * plan = nullptr;
          QPushButton * land = nullptr;
          QPushButton * abort = nullptr;
          for (QPushButton * b : panel->findChildren<QPushButton *>()) {
            if (b->text() == "Plan") {plan = b;}
            if (b->text().startsWith("6  ")) {land = b;}
            if (b->text() == "ABORT") {abort = b;}
          }
          if (plan && land && abort) {
            const QPoint p = plan->mapTo(panel, QPoint(0, 0));
            const int top = std::min(p.y(), abort->mapTo(panel, QPoint(0, 0)).y());
            const QPoint l = land->mapTo(panel, QPoint(land->width(), land->height()));
            say("mission_rect", std::to_string(p.x()) + " " + std::to_string(top) + " " +
              std::to_string(l.x() - p.x()) + " " + std::to_string(l.y() - top));
          } else {
            say("mission_rect", "none");
          }
          for (QPushButton * b : panel->findChildren<QPushButton *>()) {
            if (QString(b->text()).contains("  ")) {say("button", b->text().replace('\n', " | ").toStdString());}
          }
          QTimer::singleShot(100, &app, [&]() {app.quit();});
        });
    });
  const int rc = app.exec();
  delete panel;
  rclcpp::shutdown();
  return rc;
}
