// gs_axis_harness.cpp -- the REAL arm-GS "Valve Turning" panel, offscreen (2026-10-28).
//
// Checks the Axis Offset row (user request: obj_0 = Motive's rigid body of the valve's five
// markers, its DEFAULT pivot = their centroid): the row loads the planner's
// push_pull_valve_axis_offset, an edit + the Axis Offset button writes it back (the other
// two values at full precision), and the Box Pushing tab does not show the row. Run by
// ../gs_axis_check.sh on a private ROS domain (never beside a live stack), against the REAL
// planner started on the simulation valve-turning yaml.
//
// Prints "GSVAL <key> <value>" lines; the last is "GSVAL result PASS|FAIL ...".
//
//   QT_QPA_PLATFORM=offscreen ./gs_axis_harness <station ns> <png prefix>
#include <QApplication>
#include <QDoubleSpinBox>
#include <QPushButton>
#include <QTimer>

#include <cmath>
#include <cstdio>
#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "push_pull_panel.hpp"

namespace
{
void say(const std::string & key, const std::string & v)
{
  std::printf("GSVAL %s %s\n", key.c_str(), v.c_str());
  std::fflush(stdout);
}

std::string vec(const std::vector<double> & v, int prec = 5)
{
  std::ostringstream o;
  o.setf(std::ios::fixed);
  o.precision(prec);
  o << "[";
  for (size_t i = 0; i < v.size(); ++i) {o << (i ? ", " : "") << v[i];}
  o << "]";
  return o.str();
}

// the yaml's value: minus the v2 valve's five-marker centroid
const std::vector<double> kYaml{-0.0720, 0.0093, -0.0851};
}  // namespace

int main(int argc, char * argv[])
{
  if (argc < 3) {
    std::fprintf(stderr, "usage: gs_axis_harness <station namespace> <png prefix>\n");
    return 2;
  }
  const std::string ns = argv[1];
  const QString png = QString::fromLocal8Bit(argv[2]);
  rclcpp::init(1, argv);
  QApplication app(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("gs_axis_harness", ns);
  const std::string gov = ns.substr(0, ns.find_last_of('/'));

  auto * vpanel = new PushPullPanel(node, true);   // Valve Turning
  vpanel->resize(1300, 900);
  vpanel->show();
  auto * bpanel = new PushPullPanel(node);         // Box Pushing: no Axis Offset row
  bpanel->resize(1300, 900);
  bpanel->show();

  QTimer pump;
  QObject::connect(&pump, &QTimer::timeout, [&node]() {rclcpp::spin_some(node);});
  pump.start(5);

  auto params = std::make_shared<rclcpp::AsyncParametersClient>(node, gov + "/whole_body_trajectory_planner");
  std::vector<double> axis_param;
  const auto read = [&](std::function<void()> then) {
      params->get_parameters(
        {"push_pull_valve_axis_offset"}, [&, then](std::shared_future<std::vector<rclcpp::Parameter>> f) {
          const auto v = f.get();
          if (v.size() == 1) {axis_param = v[0].as_double_array();}
          then();
        });
    };

  // the row: the Axis Offset button and the three boxes beside it (its row widget's children)
  const auto findRow = [](PushPullPanel * p, QPushButton ** btn, std::vector<QDoubleSpinBox *> * boxes) {
      *btn = nullptr;
      for (auto * b : p->findChildren<QPushButton *>()) {
        if (b->text() == "Axis Offset") {*btn = b;}
      }
      boxes->clear();
      if (*btn && (*btn)->parentWidget()) {
        for (auto * s : (*btn)->parentWidget()->findChildren<QDoubleSpinBox *>()) {boxes->push_back(s);}
      }
    };
  QPushButton * vbtn = nullptr, * bbtn = nullptr;
  std::vector<QDoubleSpinBox *> vbox, bbox;
  findRow(vpanel, &vbtn, &vbox);
  findRow(bpanel, &bbtn, &bbox);
  bool ok = vbtn != nullptr && vbox.size() == 3;
  std::string why = ok ? "" : "Valve Turning: the Axis Offset row not found";

  const auto check = [&](bool cond, const std::string & what) {
      say(what, cond ? "ok" : "FAIL");
      if (!cond && ok) {ok = false; why = what;}
    };
  const auto shown = [](const std::vector<QDoubleSpinBox *> & b) {
      std::vector<double> v;
      for (auto * s : b) {v.push_back(s->value());}
      return v;
    };
  const auto near = [](const std::vector<double> & a, const std::vector<double> & b, double tol) {
      if (a.size() != b.size()) {return false;}
      for (size_t i = 0; i < a.size(); ++i) {if (std::abs(a[i] - b[i]) > tol) {return false;}}
      return true;
    };

  std::vector<std::function<void(std::function<void()>)>> steps;
  // 1. loaded: the row reads the yaml's value, enabled; the Box Pushing tab hides it
  steps.push_back([&](std::function<void()> next) {
      QTimer::singleShot(5000, [&, next]() {
        say("valve_row_shown", vec(shown(vbox)));
        check(near(shown(vbox), kYaml, 5e-5), "A1_row_loads_the_yaml_value");
        check(vbtn->isEnabled(), "A2_button_enabled");
        check(vbtn->isVisible(), "A3_valve_row_visible");
        check(bbtn == nullptr || !bbtn->isVisible(), "A4_box_tab_row_hidden");
        vpanel->grab().save(png + "_0_loaded.png");
        next();
      });
    });
  // 2. edit x, press the button: the planner's parameter follows, y and z kept exactly
  steps.push_back([&](std::function<void()> next) {
      vbox[0]->setValue(-0.0700);
      vbox[0]->editingFinished();
      QTimer::singleShot(300, [&, next]() {
        check(!vbox[0]->styleSheet().isEmpty(), "B1_typed_value_is_amber");
        vbtn->click();
        QTimer::singleShot(1500, [&, next]() {
          read([&, next]() {
            say("axis_param_after_edit", vec(axis_param));
            check(near(axis_param, {-0.0700, 0.0093, -0.0851}, 1e-9), "B2_parameter_written");
            check(vbox[0]->styleSheet().isEmpty(), "B3_amber_cleared");
            vpanel->grab().save(png + "_1_after_edit.png");
            next();
          });
        });
      });
    });
  // 3. a change from outside (a script, another station) shows up in the row
  steps.push_back([&](std::function<void()> next) {
      params->set_parameters(
        {rclcpp::Parameter("push_pull_valve_axis_offset", kYaml)},
        [&, next](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>) {
          QTimer::singleShot(1500, [&, next]() {
            say("valve_row_after_outside_set", vec(shown(vbox)));
            check(near(shown(vbox), kYaml, 5e-5), "C1_row_follows_the_parameter");
            next();
          });
        });
    });

  std::function<void(size_t)> run = [&](size_t i) {
      if (!ok || i >= steps.size()) {
        say("result", ok ? "PASS" : ("FAIL " + why));
        QTimer::singleShot(100, [&]() {app.quit();});
        return;
      }
      steps[i]([&, i]() {run(i + 1);});
    };
  QTimer::singleShot(100, [&]() {run(0);});
  QTimer::singleShot(30000, [&]() {
      say("result", "FAIL timeout");
      app.quit();
    });
  app.exec();
  rclcpp::shutdown();
  return ok ? 0 : 1;
}
