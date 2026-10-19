// gs_box_harness.cpp -- the REAL arm-GS "Box Pushing" panel, offscreen (2026-10-16).
//
// Checks the 2026-10-16 panel change (user request): the Vertical / Side Margin,
// Push Distance and Turn boxes in a column RIGHT of the points (as on the Pick &
// Pick-and-Drop tabs), and the Box row as editable boxes bound to the planner's
// push_pull_box_point. Run by ../gs_box_check.sh on a private ROS domain (never
// beside a live stack). The planner is the REAL node; this program plays the
// mocap system (the box body, Mocap, 100 Hz, 0.3 mm jitter) and the operator (it
// presses the panel's own Get and types into its own Box box), and reads the
// planner's parameters back through a parameter client of its own.
//
//   nothing captured -> the Box boxes disabled, a dash
//   Get              -> push_pull_box_point = the measured pose; the boxes fill, unlock
//   edit Box x       -> the parameter follows, y / z / yaw keep their full precision
//   the margins      -> Vertical = push_pull_approach_dz, Side = push_pull_approach_back
//
// Prints "GSVAL <key> <value>" lines; the last is "GSVAL result PASS|FAIL ...".
//
//   QT_QPA_PLATFORM=offscreen ./gs_box_harness <station ns> <png prefix> <mocap topic>
#include <QApplication>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QPushButton>
#include <QTimer>

#include <array>
#include <cmath>
#include <cstdio>
#include <functional>
#include <memory>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include <fsc_autopilot_ros2_msgs/msg/mocap.hpp>
#include <rclcpp/rclcpp.hpp>

#include "push_pull_panel.hpp"

namespace
{
void say(const std::string & key, const std::string & v)
{
  std::printf("GSVAL %s %s\n", key.c_str(), v.c_str());
  std::fflush(stdout);
}

std::string vec(const std::vector<double> & v, int prec = 6)
{
  std::ostringstream o;
  o.setf(std::ios::fixed);
  o.precision(prec);
  o << "[";
  for (size_t i = 0; i < v.size(); ++i) {o << (i ? ", " : "") << v[i];}
  o << "]";
  return o.str();
}

// the box (mocap frame) -- deliberately NOT round numbers
const std::array<double, 3> kBox{1.2747, 0.2812, 0.8503};   // the 2026-10-17 bench layout (y 0.28)
const double kBoxYaw = 1.7;   // [deg]
}  // namespace

int main(int argc, char * argv[])
{
  if (argc < 4) {
    std::fprintf(stderr, "usage: gs_box_harness <station namespace> <png prefix> <mocap topic>\n");
    return 2;
  }
  const std::string ns = argv[1];
  const QString png = QString::fromLocal8Bit(argv[2]);
  const std::string mocap_topic = argv[3];
  rclcpp::init(1, argv);
  QApplication app(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("gs_box_harness", ns);
  const std::string gov = ns.substr(0, ns.find_last_of('/'));

  auto * panel = new PushPullPanel(node);
  panel->resize(1300, 900);
  panel->show();
  auto * vpanel = new PushPullPanel(node, true);   // the Valve Turning tab (2026-10-16)
  vpanel->resize(1300, 900);
  vpanel->show();

  QTimer pump;
  QObject::connect(&pump, &QTimer::timeout, [&node]() {rclcpp::spin_some(node);});
  pump.start(5);

  auto mocap_pub = node->create_publisher<fsc_autopilot_ros2_msgs::msg::Mocap>(mocap_topic, 10);
  std::mt19937 rng(7);
  std::normal_distribution<double> gauss(0.0, 1.0);
  QTimer mocap;
  QObject::connect(&mocap, &QTimer::timeout, [&]() {
      fsc_autopilot_ros2_msgs::msg::Mocap m;
      m.header.stamp = node->now();
      m.header.frame_id = "map";
      m.pose.position.x = kBox[0] + 0.0003 * gauss(rng);
      m.pose.position.y = kBox[1] + 0.0003 * gauss(rng);
      m.pose.position.z = kBox[2] + 0.0003 * gauss(rng);
      const double yaw = kBoxYaw * M_PI / 180.0;
      m.pose.orientation.z = std::sin(0.5 * yaw);
      m.pose.orientation.w = std::cos(0.5 * yaw);
      mocap_pub->publish(m);
    });
  mocap.start(10);

  auto params = std::make_shared<rclcpp::AsyncParametersClient>(node, gov + "/whole_body_trajectory_planner");
  std::vector<double> box_param;
  double vmargin = NAN, smargin = NAN, turn_param = NAN;
  const auto read = [&](std::function<void()> then) {
      params->get_parameters(
        {"push_pull_box_point", "push_pull_approach_dz", "push_pull_approach_back", "push_pull_turn_deg"},
        [&, then](std::shared_future<std::vector<rclcpp::Parameter>> f) {
          const auto v = f.get();
          if (v.size() == 4) {
            box_param = v[0].as_double_array();
            vmargin = v[1].as_double();
            smargin = v[2].as_double();
            turn_param = v[3].as_double();
          }
          then();
        });
    };

  // the panel's own widgets
  std::array<QDoubleSpinBox *, 4> box_spin{};
  QDoubleSpinBox * vbox = nullptr, * sbox = nullptr;
  QPushButton * get = nullptr;
  for (auto * s : panel->findChildren<QDoubleSpinBox *>()) {
    // the Box row: its tooltip; the margins: their button's neighbour is found by value below
    if (s->toolTip().startsWith("The box pose")) {
      for (auto & b : box_spin) {if (!b) {b = s; break;}}
    }
  }
  for (auto * b : panel->findChildren<QPushButton *>()) {
    if (b->text() == "Get") {get = b;}
  }
  bool ok = box_spin[3] != nullptr && get != nullptr;
  std::string why = ok ? "" : "widgets not found";
  const auto fail = [&](const std::string & w) {if (ok) {ok = false; why = w;}};
  const auto near = [](double a, double b, double tol) {return std::abs(a - b) <= tol;};

  // the steps, one after another
  std::vector<std::pair<int, std::function<void()>>> steps;
  steps.push_back({5000, [&]() {           // loaded: nothing captured
      panel->grab().save(png + "_0_not_captured.png");
      say("box_enabled_before_get", box_spin[0]->isEnabled() ? "1" : "0");
      say("box_text_before_get", box_spin[0]->text().toStdString());
      if (box_spin[0]->isEnabled()) {fail("Box boxes enabled before Get");}
      read([&]() {
          say("margins", vec({vmargin, smargin}, 3));
          if (!near(vmargin, 0.15, 1e-9) || !near(smargin, 0.12, 1e-9)) {fail("margins not the yaml's 0.15 / 0.12");}
          // find the two margin boxes by their values
          for (auto * s : panel->findChildren<QDoubleSpinBox *>()) {
            if (s->toolTip().startsWith("The box pose")) {continue;}
            if (!vbox && near(s->value(), vmargin, 5e-4) && s->maximum() == 0.50) {vbox = s;}
            if (!sbox && near(s->value(), smargin, 5e-4) && s->maximum() == 0.30) {sbox = s;}
          }
          say("vertical_box", vbox ? std::to_string(vbox->value()) : "MISSING");
          say("side_box", sbox ? std::to_string(sbox->value()) : "MISSING");
          if (!vbox || !sbox) {fail("margin boxes not showing the planner's values");}
        });
      get->click();
    }});
  steps.push_back({2500, [&]() {           // after Get
      read([&]() {
          say("param_after_get", vec(box_param));
          if (box_param.size() != 4 || !near(box_param[0], kBox[0], 2e-3) ||
            !near(box_param[1], kBox[1], 2e-3) || !near(box_param[3], kBoxYaw, 0.2))
          {
            fail("Get did not write push_pull_box_point");
          }
          say("box_text_after_get", (box_spin[0]->text() + " " + box_spin[1]->text() + " " +
            box_spin[2]->text() + " " + box_spin[3]->text()).toStdString());
          say("box_enabled_after_get", box_spin[0]->isEnabled() ? "1" : "0");
          if (!box_spin[0]->isEnabled()) {fail("Box boxes still disabled after Get");}
          panel->grab().save(png + "_1_after_get.png");
          box_spin[0]->setValue(1.30);       // the operator corrects x (Enter: keyboard tracking off)
        });
    }});
  std::vector<double> before;
  steps.push_back({2500, [&]() {           // after the edit
      before = box_param;
      read([&]() {
          say("param_after_edit", vec(box_param));
          if (box_param.size() != 4 || !near(box_param[0], 1.30, 1e-9)) {fail("the edit did not reach the planner");}
          // y, z, yaw kept at the measured precision, not rounded to the shown 2 decimals
          for (int f = 1; f < 4 && ok; ++f) {
            if (!near(box_param[f], before[f], 1e-9)) {fail("an unedited value was rounded");}
          }
          panel->grab().save(png + "_2_after_edit.png");
        });
    }});
  // ---- the two tabs (2026-10-16): no Turn on Box Pushing; each Plan switches the mode ----
  const auto lamp = [](QWidget * w) {
      for (auto * l : w->findChildren<QLabel *>()) {if (l->wordWrap()) {return l->text().toStdString();}}
      return std::string("?");
    };
  const auto button = [](QWidget * w, const QString & text) -> QPushButton * {
      for (auto * b : w->findChildren<QPushButton *>()) {if (b->text() == text) {return b;}}
      return nullptr;
    };
  steps.push_back({500, [&]() {
      QPushButton * bt = button(panel, "Turn [°]");
      QPushButton * vt = button(vpanel, "Turn [°]");
      QPushButton * vd = button(vpanel, "Push Distance");
      say("box_turn_visible", bt && bt->isVisibleTo(panel) ? "1" : "0");
      say("valve_turn_visible", vt && vt->isVisibleTo(vpanel) ? "1" : "0");
      say("valve_distance_visible", vd && vd->isVisibleTo(vpanel) ? "1" : "0");
      if (bt && bt->isVisibleTo(panel)) {fail("Turn shown on the Box Pushing tab");}
      if (!vt || !vt->isVisibleTo(vpanel)) {fail("no Turn on the Valve Turning tab");}
      if (vd && vd->isVisibleTo(vpanel)) {fail("Push Distance shown on the Valve Turning tab");}
      say("valve_lamp_turn0", lamp(vpanel));
      if (lamp(vpanel).find("set to box pushing") == std::string::npos) {fail("the valve tab does not say it is inactive");}
      // the operator types 90 deg on the valve tab and presses its Plan
      for (auto * s : vpanel->findChildren<QDoubleSpinBox *>()) {
        if (s->isVisibleTo(vpanel) && s->maximum() == 720.0) {s->setValue(90.0);}
      }
      button(vpanel, "Plan")->setEnabled(true);   // (Plan waits for HOLD in DIRECT; the switch comes first)
      button(vpanel, "Plan")->click();
    }});
  steps.push_back({2500, [&]() {
      read([&]() {
          say("turn_after_valve_plan", std::to_string(turn_param));
          if (!near(turn_param, 90.0, 1e-9)) {fail("the valve tab's Plan did not switch the planner");}
          say("box_lamp_turn90", lamp(panel));
          if (lamp(panel).find("set to valve turning") == std::string::npos) {fail("the box tab does not say it is inactive");}
          vpanel->grab().save(png + "_3_valve_tab.png");
          button(panel, "Plan")->setEnabled(true);
          button(panel, "Plan")->click();
        });
    }});
  steps.push_back({2500, [&]() {
      read([&]() {
          say("turn_after_box_plan", std::to_string(turn_param));
          if (!near(turn_param, 0.0, 1e-9)) {fail("the box tab's Plan did not switch the planner back");}
          if (lamp(vpanel).find("set to box pushing") == std::string::npos) {fail("the valve tab active at turn 0");}
          panel->grab().save(png + "_4_box_tab.png");
          say("result", ok ? "PASS" : "FAIL " + why);
          QTimer::singleShot(200, &app, &QApplication::quit);
        });
    }});
  int t = 0;
  for (auto & s : steps) {
    t += s.first;
    QTimer::singleShot(t, &app, s.second);
  }
  QTimer::singleShot(t + 15000, &app, [&]() {say("result", "FAIL timeout " + why); app.quit();});
  const int rc = app.exec();
  rclcpp::shutdown();
  return rc;
}
