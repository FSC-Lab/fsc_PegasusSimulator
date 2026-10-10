// gs_get_harness.cpp -- the REAL arm-GS "Pick & Place" panel, offscreen (2026-10-10).
//
// Checks the 2026-10-10 panel change (operator request after the first hardware
// pick-and-place flights): a Get on the PLACE row, and the PICK row as editable
// boxes bound to the planner's pick_place_pick_point. Run by ../gs_get_check.sh on
// a private ROS domain (never beside a live stack). The planner is the REAL node;
// this program plays the mocap system (it publishes the payload body obj_0,
// fsc_autopilot_ros2_msgs/Mocap, 100 Hz, 0.3 mm jitter) and the operator (it
// presses the panel's own buttons and types into its own boxes), and reads the
// planner's parameters back through a parameter client of its own.
//
//   get  <station ns> <png prefix> [<mocap topic>]   the NEW planner:
//        nothing captured -> Pick boxes disabled, a dash, nothing written;
//        payload on the place platform -> Place Get -> pick_place_place_point
//        x, y, z, the typed yaw kept; the operator lowers z by 0.02 -> x, y keep
//        their measured precision; payload on the pick platform -> Pick Get ->
//        pick_place_pick_point, the Pick boxes fill and unlock; an edit of one
//        Pick box -> the parameter and the planner's capture (pick_place/info)
//        follow, the other three keep their precision; the Planned-goal preview;
//        a FROZEN feed -> both Gets refused on the status line, nothing changed.
//   old  <station ns> <png prefix> [<mocap topic>]   a planner from BEFORE the
//        change (no pick_place_pick_point, no get_place): the tab still loads,
//        the Pick boxes stay read-only and show Get's capture from
//        pick_place/info, nothing is written, the Place Get reports the missing
//        service.
//
// Prints "GSVAL <key> <value>" lines; the last is "GSVAL result PASS|FAIL ...".
//
//   QT_QPA_PLATFORM=offscreen ./gs_get_harness get|old <station ns> <png prefix>
#include <QApplication>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QPushButton>
#include <QTimer>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <memory>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include <fsc_autopilot_ros2_msgs/msg/mocap.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "pick_place_panel.hpp"

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

// the payload's two stations (mocap frame) -- deliberately NOT round numbers
const std::array<double, 3> kPlacePlatform{-1.0034, -0.9978, 1.0405};
const double kPlacePayloadYaw = 3.0;       // [deg] the payload's own yaw there: must NOT be used
const std::array<double, 3> kPickPlatform{1.0021, 0.9967, 1.0412};
const double kPickPayloadYaw = -2.0;       // [deg]
}  // namespace

int main(int argc, char * argv[])
{
  if (argc < 4) {
    std::fprintf(stderr, "usage: gs_get_harness get|old <station namespace> <png prefix> [<mocap topic>]\n");
    return 2;
  }
  const std::string mode = argv[1];
  const std::string ns = argv[2];
  const QString png = QString::fromLocal8Bit(argv[3]);
  const std::string mocap_topic = argc > 4 ? argv[4] : "/obj_0/mocap";
  const bool modern = mode == "get";
  rclcpp::init(1, argv);
  QApplication app(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("gs_get_harness", ns);
  const std::string gov = ns.substr(0, ns.find_last_of('/'));

  auto * panel = new PickPlacePanel(node);
  panel->resize(1500, 1100);
  panel->show();

  QTimer pump;
  QObject::connect(&pump, &QTimer::timeout, [&node]() {rclcpp::spin_some(node);});
  pump.start(5);

  // ---- the mocap system: the payload body ----------------------------------
  auto mocap_pub = node->create_publisher<fsc_autopilot_ros2_msgs::msg::Mocap>(mocap_topic, 10);
  std::array<double, 3> pay_p = kPickPlatform;
  double pay_yaw_deg = 0.0;
  bool pay_on = false, pay_frozen = false;
  std::mt19937 rng(7);
  std::normal_distribution<double> gauss(0.0, 1.0);
  QTimer mocap;
  QObject::connect(&mocap, &QTimer::timeout, [&]() {
      if (!pay_on) {return;}
      fsc_autopilot_ros2_msgs::msg::Mocap m;
      m.header.stamp = node->now();          // a frozen body is republished with FRESH stamps
      m.header.frame_id = "map";
      const double j = pay_frozen ? 0.0 : 0.0003;
      m.pose.position.x = pay_p[0] + j * gauss(rng);
      m.pose.position.y = pay_p[1] + j * gauss(rng);
      m.pose.position.z = pay_p[2] + j * gauss(rng);
      const double yaw = (pay_yaw_deg + (pay_frozen ? 0.0 : 0.05 * gauss(rng))) * M_PI / 180.0;
      m.pose.orientation.z = std::sin(0.5 * yaw);
      m.pose.orientation.w = std::cos(0.5 * yaw);
      mocap_pub->publish(m);
    });
  mocap.start(10);

  // ---- the planner, read back independently of the panel -------------------
  auto params = std::make_shared<rclcpp::AsyncParametersClient>(
    node, gov + "/whole_body_trajectory_planner");
  std::vector<double> pick_param, place_param, offset_param;
  int reads = 0;                 // completed reads
  bool reading = false;
  const auto read = [&]() {
      if (reading || !params->service_is_ready()) {return;}
      reading = true;
      std::vector<std::string> names{"pick_place_place_point", "pick_place_ee_offset"};
      if (modern) {names.push_back("pick_place_pick_point");}   // (an old planner answers nothing)
      params->get_parameters(
        names, [&, names](std::shared_future<std::vector<rclcpp::Parameter>> f) {
          const auto v = f.get();
          if (v.size() == names.size()) {
            place_param = v[0].as_double_array();
            offset_param = v[1].as_double_array();
            if (modern) {pick_param = v[2].as_double_array();}
            ++reads;
          }
          reading = false;
        });
    };
  std::vector<double> info;
  const rclcpp::QoS latched = rclcpp::QoS(1).reliable().transient_local();
  auto info_sub = node->create_subscription<std_msgs::msg::Float64MultiArray>(
    gov + "/whole_body_planner/pick_place/info", latched,
    [&info](const std_msgs::msg::Float64MultiArray & m) {info.assign(m.data.begin(), m.data.end());});

  // ---- the operator: the panel's own widgets -------------------------------
  const auto boxes = [panel](const QString & tip) {
      std::vector<QDoubleSpinBox *> v;
      for (auto * s : panel->findChildren<QDoubleSpinBox *>()) {
        if (s->toolTip().startsWith(tip)) {v.push_back(s);}
      }
      std::sort(v.begin(), v.end(), [](QDoubleSpinBox * a, QDoubleSpinBox * b) {return a->x() < b->x();});
      return v;
    };
  const auto button = [panel](const QString & tip) -> QPushButton * {
      for (auto * b : panel->findChildren<QPushButton *>()) {
        if (b->text() == "Get" && b->toolTip().startsWith(tip)) {return b;}
      }
      return nullptr;
    };
  const auto lamp = [panel]() -> QString {       // the status line (the one wrapping label)
      for (auto * l : panel->findChildren<QLabel *>()) {
        if (l->wordWrap()) {return l->text();}
      }
      return {};
    };
  const auto previews = [panel]() {              // the claw rows' "Planned goal" (green), Pick then Place
      std::vector<QLabel *> v;
      for (auto * l : panel->findChildren<QLabel *>()) {
        if (l->styleSheet().contains("#2e7d32") && !l->wordWrap()) {v.push_back(l);}
      }
      std::sort(v.begin(), v.end(), [](QLabel * a, QLabel * b) {return a->y() < b->y();});
      return v;
    };
  const auto texts = [](const std::vector<QDoubleSpinBox *> & v) {
      QStringList t;
      for (auto * s : v) {t << s->text();}
      return t.join(" ").toStdString();
    };
  const auto all_enabled = [](const std::vector<QDoubleSpinBox *> & v, bool want) {
      return std::all_of(v.begin(), v.end(), [want](QDoubleSpinBox * s) {return s->isEnabled() == want;});
    };
  const auto shows = [](const std::vector<QDoubleSpinBox *> & v, const std::vector<double> & a) {
      if (v.size() != 4 || a.size() != 4) {return false;}
      for (int f = 0; f < 4; ++f) {
        if (std::abs(v[f]->value() - a[f]) > 0.005 + 1e-9) {return false;}
      }
      return true;
    };
  const auto xyz = [](double x, double y, double z) {
      return QString("[%1, %2, %3]").arg(x, 0, 'f', 2).arg(y, 0, 'f', 2).arg(z, 0, 'f', 2);
    };
  const auto near3 = [](const std::vector<double> & a, const std::array<double, 3> & b, double tol) {
      return a.size() >= 3 && std::abs(a[0] - b[0]) < tol && std::abs(a[1] - b[1]) < tol &&
             std::abs(a[2] - b[2]) < tol;
    };

  int state = 0, ticks = 0, mark = 0, reads_mark = 0, fails = 0, rc = 1;
  std::vector<double> place_got, place_edit, pick_got, pick_edit;
  const auto check = [&fails](const std::string & what, bool ok, const std::string & detail = {}) {
      say("check", std::string(ok ? "ok    " : "WRONG ") + what + (detail.empty() ? "" : "  (" + detail + ")"));
      if (!ok) {++fails;}
    };
  const auto finish = [&]() {
      say("result", fails == 0 ? "PASS" : "FAIL (" + std::to_string(fails) + " wrong)");
      rc = fails == 0 ? 0 : 1;
      app.exit(rc);
    };
  const auto go = [&](int s) {state = s; mark = ticks; reads_mark = reads;};
  // two reads completed since the last step: what is read was asked AFTER it
  const auto reread = [&]() {return reads >= reads_mark + 2;};

  QTimer step;
  QObject::connect(&step, &QTimer::timeout, [&]() {
      ++ticks;
      read();
      const auto pick = boxes("The PICK pose");
      const auto place = boxes("The PLACE pose");
      QPushButton * pick_get = button("PICK point");
      QPushButton * place_get = button("PLACE point");
      if (pick.size() != 4 || place.size() != 4 || !pick_get || !place_get) {
        say("error", "the Pick / Place rows are not four boxes and a Get each");
        app.exit(1);
        return;
      }
      if (ticks - mark > 400) {       // 20 s in one state
        say("lamp", lamp().toStdString());
        say("pick_param", vec(pick_param));
        say("place_param", vec(place_param));
        say("result", "TIMEOUT in state " + std::to_string(state));
        app.exit(1);
        return;
      }
      const bool loaded = all_enabled(place, true) && reads > 0 && info.size() >= 80;

      if (modern) {
        switch (state) {
          case 0:       // the tab has loaded; nothing captured yet
            if (!loaded || !reread()) {break;}
            say("minimum_size_hint", std::to_string(panel->minimumSizeHint().width()) + " x " +
              std::to_string(panel->minimumSizeHint().height()));
            say("start_pick_param", vec(pick_param));
            say("start_place_param", vec(place_param));
            say("start_pick_boxes", texts(pick));
            say("start_place_boxes", texts(place));
            check("nothing captured: pick_place_pick_point is []", pick_param.empty());
            check("... the Pick boxes are disabled and show a dash",
              all_enabled(pick, false) && texts(pick) == "— — — —", texts(pick));
            check("the Place boxes show the planner's place point", shows(place, place_param));
            check("both Get buttons are enabled", pick_get->isEnabled() && place_get->isEnabled());
            panel->grab().save(png + "_0_start.png");
            // a programmatic poke at a disabled Pick box must not reach the planner
            pick[0]->setValue(0.5);
            pay_p = kPlacePlatform;                 // the payload stands on the PLACE platform
            pay_yaw_deg = kPlacePayloadYaw;
            pay_on = true;
            go(1);
            break;
          case 1:       // a full capture window of the payload there, then the Place row's Get
            if (ticks - mark < 20 || !reread()) {break;}
            check("a value forced into a disabled Pick box was NOT written (never zeros as a pick pose)",
              pick_param.empty(), vec(pick_param));
            place_get->click();
            go(2);
            break;
          case 2:
            if (!near3(place_param, kPlacePlatform, 0.001) || !shows(place, place_param)) {break;}
            if (!reread()) {break;}
            place_got = place_param;
            say("after_place_get_param", vec(place_param));
            say("after_place_get_boxes", texts(place));
            check("Place Get: x, y, z are the payload on the place platform (< 1 mm)", true);
            check("... the typed yaw is KEPT, exactly (-180, not the payload's 3)",
              place_param.size() == 4 && place_param[3] == -180.0, vec(place_param));
            check("... z is the resting height: nothing subtracted",
              std::abs(place_param[2] - kPlacePlatform[2]) < 0.001);
            check("... the pick point is untouched ([]), its boxes still disabled",
              pick_param.empty() && all_enabled(pick, false));
            // the operator lowers z by 0.02 by hand
            place[2]->setValue(place[2]->value() - 0.02);
            go(3);
            break;
          case 3:
            if (place_param.size() != 4 || std::abs(place_param[2] - place[2]->value()) > 1e-9) {break;}
            if (!reread()) {break;}
            place_edit = place_param;
            say("after_place_z_edit_param", vec(place_param));
            say("after_place_z_edit_boxes", texts(place));
            check("Place z lowered by hand: the planner has the typed z",
              std::abs(place_param[2] - (std::round(place_got[2] * 100.0) / 100.0 - 0.02)) < 1e-9);
            check("... and x, y, yaw exactly as measured (not rounded to the boxes' 2 decimals)",
              place_param[0] == place_got[0] && place_param[1] == place_got[1] &&
              place_param[3] == place_got[3]);
            pay_p = kPickPlatform;                  // the payload carried to the PICK platform
            pay_yaw_deg = kPickPayloadYaw;
            go(4);
            break;
          case 4:
            if (ticks - mark < 20) {break;}
            pick_get->click();
            go(5);
            break;
          case 5:
            if (pick_param.size() != 4 || !near3(pick_param, kPickPlatform, 0.001) ||
              !shows(pick, pick_param) || !all_enabled(pick, true))
            {
              break;
            }
            if (!reread()) {break;}
            pick_got = pick_param;
            say("after_pick_get_param", vec(pick_param));
            say("after_pick_get_boxes", texts(pick));
            check("Pick Get: pick_place_pick_point = the payload on the pick platform (< 1 mm)", true);
            check("... with its yaw", std::abs(pick_param[3] - kPickPayloadYaw) < 0.1, vec(pick_param));
            check("... the Pick boxes are filled and ENABLED", all_enabled(pick, true));
            check("... the place point is untouched", place_param == place_edit, vec(place_param));
            panel->grab().save(png + "_1_after_both_gets.png");
            pick[0]->setValue(1.05);                // the operator corrects the pick x
            go(6);
            break;
          case 6:
            if (pick_param.size() != 4 || std::abs(pick_param[0] - 1.05) > 1e-9) {break;}
            if (info.size() <= 80 || std::abs(info[60] - 1.05) > 1e-9) {break;}
            if (!reread()) {break;}
            pick_edit = pick_param;
            say("after_pick_x_edit_param", vec(pick_param));
            say("after_pick_x_edit_boxes", texts(pick));
            say("after_pick_x_edit_info_60_63_80",
              vec({info[60], info[61], info[62], info[63], info[80]}));
            check("Pick x edited: the planner has the typed x", true);
            check("... y, z, yaw exactly as measured",
              pick_param[1] == pick_got[1] && pick_param[2] == pick_got[2] &&
              pick_param[3] == pick_got[3]);
            check("... the planner's CAPTURE follows (pick_place/info [60..63], [80])",
              info[60] == pick_param[0] && info[61] == pick_param[1] && info[62] == pick_param[2] &&
              info[63] == 1.0 && std::abs(info[80] - pick_param[3]) < 1e-9);
            {
              // "Planned goal" before Plan: the point + Rz(its yaw) * the EE offset
              const auto pv = previews();
              const auto goal = [&](const std::vector<double> & p) {
                  const double y = p[3] * M_PI / 180.0, c = std::cos(y), s = std::sin(y);
                  const auto & o = offset_param;
                  return xyz(p[0] + c * o[0] - s * o[1], p[1] + s * o[0] + c * o[1], p[2] + o[2]);
                };
              const bool have = pv.size() == 2 && offset_param.size() == 3;
              say("ee_offset_param", vec(offset_param, 3));
              say("planned_goal_pick", have ? pv[0]->text().toStdString() : "?");
              say("planned_goal_place", have ? pv[1]->text().toStdString() : "?");
              check("the Planned-goal preview of the Pick row = the edited pick pose + the EE offset",
                have && pv[0]->text() == goal(pick_param),
                have ? goal(pick_param).toStdString() : "");
              check("... and of the Place row = the place pose + the EE offset",
                have && pv[1]->text() == goal(place_param),
                have ? goal(place_param).toStdString() : "");
            }
            panel->grab().save(png + "_2_after_edits.png");
            pay_frozen = true;                      // the body is lost: its last pose, republished
            go(7);
            break;
          case 7:
            if (ticks - mark < 20) {break;}
            pick_get->click();
            go(8);
            break;
          case 8:
            if (!lamp().contains("frozen") || !lamp().startsWith("Get Pick")) {break;}
            if (!reread()) {break;}
            say("frozen_pick_get_lamp", lamp().toStdString());
            check("a FROZEN feed: the Pick Get is refused on the status line, the pick point unchanged",
              pick_param == pick_edit, vec(pick_param));
            place_get->click();
            go(9);
            break;
          case 9:
            if (!lamp().contains("frozen") || !lamp().startsWith("Get Place")) {break;}
            if (!reread()) {break;}
            say("frozen_place_get_lamp", lamp().toStdString());
            check("... and the Place Get too, the place point unchanged",
              place_param == place_edit, vec(place_param));
            panel->grab().save(png + "_3_frozen_refused.png");
            finish();
            break;
          default:
            break;
        }
        return;
      }

      // ---- an OLDER planner: no pick_place_pick_point, no get_place ----------
      switch (state) {
        case 0:
          if (!loaded || !reread()) {break;}
          say("start_place_param", vec(place_param));
          say("start_pick_boxes", texts(pick));
          say("start_place_boxes", texts(place));
          check("older planner: the tab LOADS (the Place boxes show its place point, enabled)",
            shows(place, place_param));
          check("... the Pick boxes are disabled and show a dash",
            all_enabled(pick, false) && texts(pick) == "— — — —", texts(pick));
          pay_p = kPickPlatform;
          pay_yaw_deg = kPickPayloadYaw;
          pay_on = true;
          go(1);
          break;
        case 1:
          if (ticks - mark < 20) {break;}
          pick_get->click();
          go(2);
          break;
        case 2:
          if (info.size() < 80 || !(info[63] > 0.5)) {break;}
          {
            const double yaw = info.size() > 80 && std::isfinite(info[80]) ? info[80] : 0.0;
            if (!shows(pick, {info[60], info[61], info[62], yaw})) {break;}
          }
          say("after_pick_get_boxes", texts(pick));
          say("after_pick_get_info_60_63_80", vec({info[60], info[61], info[62], info[63], info[80]}));
          check("older planner: Pick Get still captures, and the Pick boxes show it from pick_place/info",
            near3({info[60], info[61], info[62]}, kPickPlatform, 0.001));
          check("... READ-ONLY (disabled), as the labels were", all_enabled(pick, false));
          pick[0]->setValue(1.23);     // forced: must not be sent (the planner has no such parameter)
          go(3);
          break;
        case 3:
          if (ticks - mark < 30) {break;}
          check("... and a value forced into a Pick box is not sent to the planner",
            !lamp().contains("not set"), lamp().toStdString());
          place_get->click();
          go(4);
          break;
        case 4:
          if (!lamp().startsWith("Get Place") || !lamp().contains("service not available")) {break;}
          if (!reread()) {break;}
          say("place_get_lamp", lamp().toStdString());
          check("older planner: the Place Get reports the missing service, the place point unchanged",
            place_param.size() == 4 && shows(place, place_param), vec(place_param));
          panel->grab().save(png + "_old_planner.png");
          finish();
          break;
        default:
          break;
      }
    });
  step.start(50);
  const int r = app.exec();
  rclcpp::shutdown();
  return r ? r : rc;
}
