// gs_monitor_harness.cpp -- the REAL arm-GS "Box Pushing" panel, offscreen (2026-10-21).
//
// Checks the MONITOR row added under the points (user request): the landing
// gear's height above the table top, NOW (whole_body_planner/current_base) and
// the LOWEST on the plan over the table (push_pull/drone_path). Run by
// ../gs_monitor_check.sh on a private ROS domain against the REAL planner on the
// box-pushing sim yaml, which supplies push_pull_table / push_pull_gear_* through
// its parameter service. This program plays the vehicle (current_base) and a
// planned mission (drone_path, the 2026-10-21 nominal layout's body waypoints),
// steps the vehicle through four places and reads the panel's own boxes.
//
// Prints "GSVAL <key> <value>"; the last is "GSVAL result PASS|FAIL ...".
// Also the workflow page's two panel pictures (the box captured with Get at the
// nominal place): <png prefix>_panel.png (the two groups) and _rows.png (the
// points, the Box row boxed in red).
//   QT_QPA_PLATFORM=offscreen ./gs_monitor_harness <station ns> <png prefix> <box mocap topic>
#include <QApplication>
#include <QLabel>
#include <QGroupBox>
#include <QLineEdit>
#include <QPainter>
#include <QPushButton>
#include <QTimer>

#include <array>
#include <cmath>
#include <cstdio>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <fsc_autopilot_ros2_msgs/msg/mocap.hpp>

#include "push_pull_panel.hpp"

namespace
{
void say(const std::string & key, const std::string & v)
{
  std::printf("GSVAL %s %s\n", key.c_str(), v.c_str());
  std::fflush(stdout);
}

// the nominal 2026-10-21 plan, the drone body's waypoints (extract_waypoints.py)
const std::vector<std::array<double, 3>> kPlan{
  {-0.600, 0.000, 1.3002}, {0.677, 0.667, 1.3152}, {0.677, 0.547, 1.1652}, {0.677, 0.047, 1.1652},
  {0.677, 0.047, 1.3652}, {-0.600, 0.000, 1.3002}, {-0.600, 0.000, 0.3002}};
// (z + 0.2 mm: with the 0.2425 m gear a 3-decimal z rounds on a tie)
}  // namespace

int main(int argc, char * argv[])
{
  if (argc < 4) {
    std::fprintf(stderr, "usage: gs_monitor_harness <station namespace> <png prefix> <box topic>\n");
    return 2;
  }
  const std::string box_topic = argv[3];
  const std::string ns = argv[1];
  const QString png = QString::fromLocal8Bit(argv[2]);
  rclcpp::init(1, argv);
  QApplication app(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("gs_monitor_harness", ns);
  const std::string gov = ns.substr(0, ns.find_last_of('/'));

  auto * panel = new PushPullPanel(node);
  panel->resize(1300, 900);
  panel->show();

  QTimer pump;
  QObject::connect(&pump, &QTimer::timeout, [&node]() {rclcpp::spin_some(node);});
  pump.start(5);

  // the vehicle: current_base at 20 Hz, wherever `pos` says
  std::array<double, 3> pos{-0.600, 0.000, 1.3002};
  auto base_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(
    gov + "/whole_body_planner/current_base", 10);
  QTimer base;
  QObject::connect(&base, &QTimer::timeout, [&]() {
      geometry_msgs::msg::PoseStamped m;
      m.header.stamp = node->now();
      m.header.frame_id = "map";
      m.pose.position.x = pos[0];
      m.pose.position.y = pos[1];
      m.pose.position.z = pos[2];
      m.pose.orientation.w = 1.0;
      base_pub->publish(m);
    });
  base.start(50);

  // the plan: drone_path rows [t, x, y, z, speed, qx, qy, qz, qw], 50 per leg, republished
  // every second (the planner latches an EMPTY path at startup, which must not win)
  std_msgs::msg::Float64MultiArray path;
  for (size_t k = 0; k + 1 < kPlan.size(); ++k) {
    for (int i = 0; i < 50; ++i) {
      const double s = i / 50.0;
      path.data.insert(path.data.end(), {static_cast<double>(k * 50 + i),
        kPlan[k][0] + s * (kPlan[k + 1][0] - kPlan[k][0]), kPlan[k][1] + s * (kPlan[k + 1][1] - kPlan[k][1]),
        kPlan[k][2] + s * (kPlan[k + 1][2] - kPlan[k][2]), 0.0, 0.0, 0.0, 0.0, 1.0});
    }
  }
  const rclcpp::QoS latched = rclcpp::QoS(1).reliable().transient_local();
  auto path_pub = node->create_publisher<std_msgs::msg::Float64MultiArray>(
    gov + "/whole_body_planner/push_pull/drone_path", latched);
  bool path_on = false;
  QTimer path_timer;
  QObject::connect(&path_timer, &QTimer::timeout, [&]() {if (path_on) {path_pub->publish(path);}});
  path_timer.start(1000);

  // the box at its nominal place (0.675, 0.28), yaw 0 -- for Get
  auto box_pub = node->create_publisher<fsc_autopilot_ros2_msgs::msg::Mocap>(box_topic, 10);
  int box_n = 0;
  QTimer box_timer;
  QObject::connect(&box_timer, &QTimer::timeout, [&]() {
      fsc_autopilot_ros2_msgs::msg::Mocap m;
      m.header.stamp = node->now();
      m.header.frame_id = "map";
      const double j = 0.0003 * ((box_n++ % 7) - 3) / 3.0;   // a little jitter (Get refuses a frozen feed)
      m.pose.position.x = 0.675 + j;
      m.pose.position.y = 0.280 - j;
      m.pose.position.z = 0.850 + 0.5 * j;
      m.pose.orientation.w = 1.0;
      box_pub->publish(m);
    });
  box_timer.start(10);
  QPushButton * get = nullptr;
  for (auto * b : panel->findChildren<QPushButton *>()) {if (b->text() == "Get") {get = b;}}

  QLineEdit * now_box = nullptr, * plan_box = nullptr;
  for (auto * e : panel->findChildren<QLineEdit *>()) {
    if (e->toolTip().startsWith("NOW: the vehicle")) {now_box = e;}   // the gear row (the Push rows say NOW: too)
    if (e->toolTip().startsWith("PLAN: the LOWEST")) {plan_box = e;}
  }
  QLabel * note = nullptr;
  for (auto * l : panel->findChildren<QLabel *>()) {
    if (l->text().contains("planner keeps") || l->text().contains("waiting for the planner's table")) {
      note = l;
    }
  }
  bool ok = now_box != nullptr && plan_box != nullptr;
  std::string why = ok ? "" : "monitor boxes not found";
  const auto fail = [&](const std::string & w) {if (ok) {ok = false; why = w;}};
  const auto colour = [](QLineEdit * e) {
      const QString s = e->styleSheet();
      return s.contains("#2e7d32") ? std::string("green") : s.contains("#e58a00") ? std::string("amber")
        : s.contains("#c62828") ? std::string("red") : std::string("grey");
    };
  const auto check = [&](const std::string & tag, const QString & want_now, const std::string & now_col,
      const QString & want_plan, const std::string & plan_col) {
      for (auto * l : panel->findChildren<QLabel *>()) {
        if (l->text().contains("planner keeps")) {note = l;}
      }
      const std::string got = now_box->text().toStdString() + " " + colour(now_box) + " | " +
        plan_box->text().toStdString() + " " + colour(plan_box);
      say(tag, got + (note ? " | " + note->text().toStdString() : ""));
      if (now_box->text() != want_now || colour(now_box) != now_col || plan_box->text() != want_plan ||
        colour(plan_box) != plan_col)
      {
        fail(tag + ": got " + got + ", want " + want_now.toStdString() + " " + now_col + " | " +
          want_plan.toStdString() + " " + plan_col);
      }
      panel->grab().save(png + "_" + QString::fromStdString(tag) + ".png");
    };

  // sim yaml: gear 0.2425 under the body, table top 0.80, clearance 0.05, reach 0.25 + 0.05
  // (expected = z - 0.2425 - 0.8, shown to 3 decimals)
  std::vector<std::pair<int, std::function<void()>>> steps;
  steps.push_back({6000, [&]() {   // takeoff hover, no plan: 1.3 - 0.2425 - 0.8 = 0.2575, not over
      check("1_hover_no_plan", "0.258", "grey", "—", "grey");   // 0.2577
      if (get) {get->click();}
      path_on = true;
      path_pub->publish(path);
    }});
  steps.push_back({2500, [&]() {   // the plan: lowest over the table 1.165 - 0.2425 - 0.8 = 0.1225
      check("2_hover_planned", "0.258", "grey", "0.123", "green");   // plan min 0.1227
      pos = {0.677, 0.300, 1.1652};   // at the push, over the table
    }});
  steps.push_back({1500, [&]() {
      check("3_push", "0.123", "green", "0.123", "green");
      pos = {0.677, 0.300, 1.0802};   // 0.0377 above: under the planner's 0.05
    }});
  steps.push_back({1500, [&]() {
      check("4_low", "0.038", "amber", "0.123", "green");
      pos = {0.677, 0.300, 0.9902};   // -0.0523: below the table top
    }});
  steps.push_back({1500, [&]() {
      check("5_below", "-0.052", "red", "0.123", "green");
      pos = {0.677, 0.300, 1.1652};
    }});
  steps.push_back({1500, [&]() {
      check("6_final", "0.123", "green", "0.123", "green");
      // the workflow page's pictures: the two groups, and the points with the Box row boxed
      QGroupBox * targets = nullptr, * mission = nullptr;
      for (auto * g : panel->findChildren<QGroupBox *>()) {
        if (g->title().startsWith("Calibration")) {targets = g;}
        if (g->title() == "Mission") {mission = g;}
      }
      if (!targets || !mission || !get) {fail("groups / Get not found"); return;}
      panel->grab(QRect(0, 0, panel->width(), mission->geometry().bottom() + 2)).save(png + "_panel.png");
      QRect box_row = get->geometry();
      for (auto * s : panel->findChildren<QWidget *>()) {
        if (s->toolTip().startsWith("The box pose")) {box_row = box_row.united(s->geometry());}
      }
      box_row.setLeft(6);
      const QPoint off = targets->mapTo(panel, QPoint(0, 0));
      const int right = get->mapTo(panel, QPoint(get->width(), 0)).x();
      int setting_right = right;
      for (auto * b : panel->findChildren<QPushButton *>()) {
        if (b->text() == "Push Distance") {setting_right = b->mapTo(panel, QPoint(b->width(), 0)).x() + 90;}
      }
      QPixmap rows = panel->grab(QRect(0, 0, std::max(right, setting_right) + 8, off.y() + targets->height() + 2));
      QPainter pt(&rows);
      pt.setPen(QPen(QColor(220, 30, 30), 3));
      pt.setRenderHint(QPainter::Antialiasing);
      pt.drawRoundedRect(QRect(off + box_row.topLeft() - QPoint(0, 4),
        QSize(right - off.x() - box_row.left() + 6, box_row.height() + 8)), 4, 4);
      pt.end();
      rows.save(png + "_rows.png");
      say("box_text", get ? "got" : "-");
    }});
  int i = 0;
  std::function<void()> next = [&]() {
      if (i >= static_cast<int>(steps.size())) {
        say("result", ok ? "PASS" : "FAIL " + why);
        app.exit(ok ? 0 : 1);
        return;
      }
      const auto s = steps[i++];
      QTimer::singleShot(s.first, [&, s]() {s.second(); next();});
    };
  next();
  const int rc = app.exec();
  rclcpp::shutdown();
  return rc;
}
