// gs_valve_harness.cpp -- the REAL arm-GS "Valve Turning" and "Box Pushing" panels,
// offscreen, against FAKE planners (no Isaac, no PX4, no planner node), 2026-10-18
// (valve README 14). Run by ../gs_valve_check.sh on a private ROS domain.
//
// Each rig = one fake planner (the push_pull status / info topics, the release and
// exit services, and a parameter-only node named whole_body_trajectory_planner so the
// panel learns push_pull_turn_deg) + one panel. The harness plays the gripper: it
// records the panel's gripper requests and reports OPEN 0.4 s after an open, or never.
//
//   VA  valve Exit            -> NO release call; open at once; Exit 1.0 s after the OPEN report
//   VB  valve, no OPEN report -> no Exit (timeout 1.0 s)
//   VC  valve, the node leaves DIRECT while turning -> exactly one open
//   VD  valve, the node leaves DIRECT off the handle -> no open
//   BA  box Exit              -> release FIRST, then open, Exit on the report (2026-10-17, unchanged)
//   BC  box, the node leaves DIRECT while pushing -> exactly one open
//
// Prints "GSVAL <key> <value>"; the last is "GSVAL result PASS|FAIL ...".
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

struct Rig
{
  rclcpp::Node::SharedPtr node, planner;
  PushPullPanel * panel{nullptr};
  std::vector<std::pair<std::string, double>> events;
  bool report_open{true};
  Clock::time_point t0{Clock::now()};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pl_status, status;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr info;
  std::vector<rclcpp::ServiceBase::SharedPtr> srvs;
  bool valve;

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

  Rig(const std::string & gov, bool valve_mode) : valve(valve_mode)
  {
    node = std::make_shared<rclcpp::Node>("gs_valve_harness", gov + "/fsc_open_manipulator");
    node->declare_parameter<double>("push_pull_release_timeout_s", 1.0);
    // the fake planner's parameters: the panel's active() reads push_pull_turn_deg
    planner = std::make_shared<rclcpp::Node>("whole_body_trajectory_planner", gov);
    planner->declare_parameter<std::vector<double>>("push_pull_start_mark", {0.0, 0.0, 1.0, 0.0});
    planner->declare_parameter<std::vector<double>>("push_pull_land_start", {-0.5, 0.0, 1.3, -360.0});
    planner->declare_parameter<std::vector<double>>("push_pull_land", {-0.5, 0.0, 0.8, -360.0});
    planner->declare_parameter<std::vector<double>>("push_pull_ee_offset", {0.16, 0.0006, 0.179});
    planner->declare_parameter<double>("push_pull_approach_dz", 0.10);
    planner->declare_parameter<double>("push_pull_push_distance", 0.5);
    planner->declare_parameter<double>("push_pull_turn_deg", valve_mode ? 360.0 : 0.0);
    planner->declare_parameter<double>("push_pull_approach_back", 0.12);
    const std::string base = gov + "/whole_body_planner/push_pull";
    const auto latched = rclcpp::QoS(1).reliable().transient_local();
    pl_status = node->create_publisher<std_msgs::msg::String>(base + "/status", latched);
    status = node->create_publisher<std_msgs::msg::String>(gov + "/whole_body_planner/status", latched);
    info = node->create_publisher<std_msgs::msg::Float64MultiArray>(base + "/info", latched);
    srvs.push_back(node->create_service<Trigger>(
        base + "/release", [this](const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr r) {
          events.emplace_back("release", now());
          r->success = true;
          r->message = "release: (fake) CoM anchor";
        }));
    srvs.push_back(node->create_service<Trigger>(
        base + "/exit", [this](const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr r) {
          events.emplace_back("exit", now());
          r->success = true;
          r->message = "exit: (fake) climbing";
        }));
    panel = new PushPullPanel(node, valve_mode);
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
    onHandle("DONE push next=exit (the gripper opens first)");
    std_msgs::msg::String s;
    s.data = "HOLD";
    status->publish(s);
  }

  void publishInfo(bool at_handle, int done, int flying)
  {
    std_msgs::msg::Float64MultiArray m;
    m.data.assign(69, 0.0);
    m.data[4] = 1.0;
    m.data[5] = done;
    m.data[6] = flying;
    m.data[60] = valve ? 360.0 : 0.0;
    m.data[61] = at_handle ? 1.0 : 0.0;
    info->publish(m);
  }
  void onHandle(const std::string & st, int done = 2, int flying = -1)
  {
    publishInfo(true, done, flying);
    std_msgs::msg::String s;
    s.data = st;
    pl_status->publish(s);
  }
  void plStatus(const std::string & st)
  {
    std_msgs::msg::String s;
    s.data = st;
    pl_status->publish(s);
  }

  QPushButton * exitButton() const
  {
    for (QPushButton * b : panel->findChildren<QPushButton *>()) {
      if (b->text().contains(valve ? "Exit To Turn" : "Exit To Push")) {return b;}
    }
    return nullptr;
  }
};
}  // namespace

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  QApplication app(argc, argv);
  Rig valve("/uav_gsvalve", true);
  Rig box("/uav_gsbox", false);
  QTimer pump;
  QObject::connect(&pump, &QTimer::timeout, [&]() {
      rclcpp::spin_some(valve.node);
      rclcpp::spin_some(valve.planner);
      rclcpp::spin_some(box.node);
      rclcpp::spin_some(box.planner);
    });
  pump.start(5);

  std::vector<std::string> fails;
  const auto check = [&](bool ok, const std::string & what) {
      say(what, ok ? "ok" : "FAILED");
      if (!ok) {fails.push_back(what);}
    };
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
            check(false, std::string(r.valve ? "valve" : "box") + "_exit_button_enabled");
            run_next();
          }
        });
      poll->start(50);
    };
  // publish a status sequence, judge after wait_ms
  const auto statuses_then = [&](Rig & r, std::function<void()> prep, const std::string & last,
                                 int wait_ms, std::function<void()> judge) {
      prep();
      QTimer::singleShot(400, &app, [&, last, wait_ms, judge]() {
          r.events.clear();
          r.t0 = Clock::now();
          r.plStatus(last);
          QTimer::singleShot(wait_ms, &app, [&, judge]() {judge(); run_next();});
        });
    };

  cases.push_back([&]() {   // VA
      valve.report_open = true;
      press_then(valve, 2500, [&]() {
          const double to = valve.first("open"), tre = valve.first("open_report"), te = valve.first("exit");
          say("VA_events_s", "open " + std::to_string(to) + " open_report " + std::to_string(tre) +
              " exit " + std::to_string(te));
          check(valve.count("release") == 0, "VA_no_release_call");
          check(valve.count("open") == 1 && valve.count("exit") == 1, "VA_open_and_exit");
          check(to < 0.15, "VA_open_at_once");
          check(te - tre > 0.85 && te - tre < 1.2, "VA_exit_1s_after_the_report");
        });
    });
  cases.push_back([&]() {   // VB
      valve.onHandle("DONE push next=exit (the gripper opens first)");
      valve.report_open = false;
      press_then(valve, 2000, [&]() {
          check(valve.count("open") == 1, "VB_opened");
          check(valve.count("exit") == 0 && valve.count("release") == 0, "VB_no_exit_without_the_report");
        });
    });
  cases.push_back([&]() {   // VC
      valve.report_open = true;
      statuses_then(valve, [&]() {valve.onHandle("FLYING push (CONTACT) T=37.5s", 1, 2);},
        "NOT IN DIRECT", 800, [&]() {
          check(valve.count("open") == 1, "VC_open_when_DIRECT_lost_turning");
          check(valve.count("exit") == 0, "VC_no_exit");
        });
    });
  cases.push_back([&]() {   // VD
      statuses_then(valve, [&]() {
          valve.publishInfo(false, 4, -1);
          valve.plStatus("DONE go_to_land_start next=land");
        }, "NOT IN DIRECT", 800, [&]() {
          check(valve.count("open") == 0, "VD_no_open_off_the_handle");
        });
    });
  cases.push_back([&]() {   // BA
      box.report_open = true;
      press_then(box, 1500, [&]() {
          const double tr = box.first("release"), to = box.first("open");
          const double tre = box.first("open_report"), te = box.first("exit");
          check(box.count("release") == 1 && box.count("open") == 1 && box.count("exit") == 1, "BA_one_each");
          check(tr <= to && to < tre && tre <= te && te - tre < 0.15, "BA_release_first_unchanged");
        });
    });
  cases.push_back([&]() {   // BC
      statuses_then(box, [&]() {box.onHandle("FLYING push (CONTACT) T=25.5s", 1, 2);},
        "NOT IN DIRECT", 800, [&]() {
          check(box.count("open") == 1, "BC_open_when_DIRECT_lost_pushing");
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
  QTimer::singleShot(2500, &app, [&]() {run_next();});   // latched topics + the parameter load
  QTimer::singleShot(60000, &app, [&]() {say("result", "FAIL timeout"); app.exit(1);});
  const int rc = app.exec();
  rclcpp::shutdown();
  return rc != 0 ? rc : (fails.empty() ? 0 : 1);
}
