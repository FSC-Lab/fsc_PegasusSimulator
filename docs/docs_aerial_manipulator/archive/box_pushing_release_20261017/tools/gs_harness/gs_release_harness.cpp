// gs_release_harness.cpp -- the REAL arm-GS "Box Pushing" panel, offscreen, and its
// Exit To Push sequence after the 2026-10-17 release-window fix, against a FAKE
// planner (no Isaac, no PX4, no planner node). Run by ../gs_release_check.sh on a
// private ROS domain.
//
// The fake planner publishes the state "the push is done, the claw on the handle,
// HOLD" and serves push_pull/release and push_pull/exit, recording every call. The
// harness plays the gripper: it records the panel's gripper requests and reports
// OPEN (onGripperOpenFinished) 0.4 s after an open request, or never.
//
//   A  release ok           -> release, THEN the open, THEN (on the OPEN report) exit
//   B  release refused      -> no open, no exit
//   C  no OPEN report       -> no exit (timeout 1.0 s), the Exit button usable again
//   D  no release service   -> NOTHING opened, no exit, Exit usable again (2026-10-19:
//                              the open-first fallback for an older planner was the
//                              10-17 crash sequence and is gone -- a WiFi discovery drop
//                              on a current planner looked the same)
//
// Prints "GSVAL <key> <value>" lines; the last is "GSVAL result PASS|FAIL ...".
//
//   QT_QPA_PLATFORM=offscreen ./gs_release_harness
#include <QApplication>
#include <QPushButton>
#include <QTimer>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "push_pull_panel.hpp"

namespace
{
using Clock = std::chrono::steady_clock;
using Trigger = std_srvs::srv::Trigger;

void say(const std::string & key, const std::string & v)
{
  std::printf("GSVAL %s %s\n", key.c_str(), v.c_str());
  std::fflush(stdout);
}

// one fake planner + one panel in its own vehicle namespace
struct Rig
{
  rclcpp::Node::SharedPtr node;
  PushPullPanel * panel{nullptr};
  std::vector<std::pair<std::string, double>> events;   // (what, t [s])
  bool release_ok{true};
  bool report_open{true};
  Clock::time_point t0{Clock::now()};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pl_status, status;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr info;
  std::vector<rclcpp::ServiceBase::SharedPtr> srvs;

  double now() const {return std::chrono::duration<double>(Clock::now() - t0).count();}
  int count(const std::string & what) const
  {
    int n = 0;
    for (const auto & e : events) {n += e.first == what;}
    return n;
  }
  double first(const std::string & what) const
  {
    for (const auto & e : events) {if (e.first == what) {return e.second;}}
    return NAN;
  }

  Rig(const std::string & gov, bool with_release)
  {
    node = std::make_shared<rclcpp::Node>("gs_release_harness", gov + "/fsc_open_manipulator");
    // a short no-OPEN timeout for case C (the panel reads it at construction)
    node->declare_parameter<double>("push_pull_release_timeout_s", 1.0);
    const std::string base = gov + "/whole_body_planner/push_pull";
    const auto latched = rclcpp::QoS(1).reliable().transient_local();
    pl_status = node->create_publisher<std_msgs::msg::String>(base + "/status", latched);
    status = node->create_publisher<std_msgs::msg::String>(gov + "/whole_body_planner/status", latched);
    info = node->create_publisher<std_msgs::msg::Float64MultiArray>(base + "/info", latched);
    if (with_release) {
      srvs.push_back(node->create_service<Trigger>(
          base + "/release", [this](const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr r) {
            events.emplace_back("release", now());
            r->success = release_ok;
            r->message = release_ok ? "release: (fake) CoM anchor" : "release: (fake) push is in flight";
          }));
    }
    srvs.push_back(node->create_service<Trigger>(
        base + "/exit", [this](const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr r) {
          events.emplace_back("exit", now());
          r->success = true;
          r->message = "exit: (fake) climbing";
        }));
    panel = new PushPullPanel(node);
    panel->resize(1300, 900);
    panel->show();
    QObject::connect(panel, &PushPullPanel::gripperRequested, [this](bool open) {
        events.emplace_back(open ? "open" : "close", now());
        if (open && report_open) {
          QTimer::singleShot(400, panel, [this]() {
              events.emplace_back("open_report", now());
              panel->onGripperOpenFinished();
            });
        }
      });
    // the push done, the claw on the handle, the planner holding
    std_msgs::msg::Float64MultiArray m;
    m.data.assign(69, 0.0);
    m.data[4] = 1.0;    // planned
    m.data[5] = 2.0;    // done = push
    m.data[6] = -1.0;   // flying: none
    m.data[61] = 1.0;   // at the handle
    info->publish(m);
    std_msgs::msg::String s;
    s.data = "DONE push next=exit (the gripper opens first)";
    pl_status->publish(s);
    s.data = "HOLD";
    status->publish(s);
  }

  QPushButton * exitButton() const
  {
    for (QPushButton * b : panel->findChildren<QPushButton *>()) {
      if (b->text().contains("Exit To Push")) {return b;}
    }
    return nullptr;
  }
};
}  // namespace

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  QApplication app(argc, argv);
  Rig rig("/uav_gsrel", true);
  Rig old("/uav_gsold", false);

  QTimer pump;
  QObject::connect(&pump, &QTimer::timeout, [&]() {
      rclcpp::spin_some(rig.node);
      rclcpp::spin_some(old.node);
    });
  pump.start(5);

  std::vector<std::string> fails;
  const auto check = [&](bool ok, const std::string & what) {
      say(what, ok ? "ok" : "FAILED");
      if (!ok) {fails.push_back(what);}
    };
  // each case: press Exit when it is enabled, then judge after `wait_ms`
  std::vector<std::function<void()>> cases;
  size_t next = 0;
  std::function<void()> run_next;
  const auto press_then = [&](Rig & r, int wait_ms, std::function<void()> judge) {
      const auto t_start = Clock::now();
      auto * poll = new QTimer(&app);
      QObject::connect(poll, &QTimer::timeout, [&, poll, wait_ms, judge, t_start]() {
          QPushButton * b = r.exitButton();
          if (b && b->isEnabled()) {
            poll->stop();
            poll->deleteLater();
            r.events.clear();
            r.t0 = Clock::now();
            b->click();
            QTimer::singleShot(wait_ms, &app, [&, judge]() {judge(); run_next();});
          } else if (Clock::now() - t_start > std::chrono::seconds(10)) {
            poll->stop();
            poll->deleteLater();
            check(false, "exit_button_enabled");
            run_next();
          }
        });
      poll->start(50);
    };

  cases.push_back([&]() {   // A: release ok -> release, open, OPEN report, exit
      rig.release_ok = true;
      rig.report_open = true;
      press_then(rig, 1500, [&]() {
          const double tr = rig.first("release"), to = rig.first("open");
          const double tre = rig.first("open_report"), te = rig.first("exit");
          say("A_events_s", "release " + std::to_string(tr) + " open " + std::to_string(to) +
              " open_report " + std::to_string(tre) + " exit " + std::to_string(te));
          check(rig.count("release") == 1 && rig.count("open") == 1 && rig.count("exit") == 1, "A_one_each");
          check(tr <= to && to < tre && tre <= te, "A_order_release_open_report_exit");
          check(te - tre < 0.15, "A_exit_on_the_report");
        });
    });
  cases.push_back([&]() {   // B: release refused -> nothing opens, nothing exits
      rig.release_ok = false;
      rig.report_open = true;
      press_then(rig, 1500, [&]() {
          check(rig.count("release") == 1, "B_release_asked");
          check(rig.count("open") == 0 && rig.count("exit") == 0, "B_no_open_no_exit");
        });
    });
  cases.push_back([&]() {   // C: no OPEN report -> no exit after the 1.0 s timeout
      rig.release_ok = true;
      rig.report_open = false;
      press_then(rig, 2000, [&]() {
          check(rig.count("release") == 1 && rig.count("open") == 1, "C_released_and_opened");
          check(rig.count("exit") == 0, "C_no_exit_without_the_report");
          QPushButton * b = rig.exitButton();
          check(b && b->isEnabled(), "C_exit_button_usable_again");
        });
    });
  cases.push_back([&]() {   // D: no push_pull/release service -> nothing opened (2026-10-19)
      old.report_open = true;
      press_then(old, 1500, [&]() {
          check(old.count("open") == 0 && old.count("exit") == 0, "D_no_open_no_exit");
          QPushButton * b = old.exitButton();
          check(b && b->isEnabled(), "D_exit_button_usable_again");
        });
    });

  run_next = [&]() {
      if (next < cases.size()) {
        cases[next++]();
        return;
      }
      say("result", fails.empty() ? "PASS" : "FAIL " + std::to_string(fails.size()));
      app.quit();
    };
  QTimer::singleShot(1500, &app, [&]() {run_next();});   // let the latched topics arrive
  QTimer::singleShot(60000, &app, [&]() {say("result", "FAIL timeout"); app.exit(1);});
  const int rc = app.exec();
  rclcpp::shutdown();
  return rc != 0 ? rc : (fails.empty() ? 0 : 1);
}
