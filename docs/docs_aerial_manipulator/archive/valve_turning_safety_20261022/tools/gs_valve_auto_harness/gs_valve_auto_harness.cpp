// gs_valve_auto_harness.cpp -- the REAL arm-GS "Valve Turning" panel, offscreen, against a
// FAKE planner (no Isaac, no PX4, no planner node), for the 2026-10-22 changes: the box push's
// automatic Ready / Push / Exit and its mocap-guard lamp, carried over to the valve tab. Run
// by ../gs_valve_auto_check.sh on a private ROS domain. Built from the box push's
// archive/box_pushing_hardware_20261019/tools/gs_grasp_harness.
//
// Each case builds a FRESH valve panel in its own vehicle namespace beside a fake planner node
// (whole_body_trajectory_planner with push_pull_turn_deg 360, so the valve tab is the active
// one: the push-and-pull status, info, arrival_error and the step services, every call
// recorded). The harness plays the gripper: an OPEN is reported (onGripperOpenFinished) 0.4 s
// later; a CLOSE ends 0.4 s later with a stall on the fin's shim (the SIMULATION grasp rule).
//
//   V1  Ready To Turn starts by itself 1 s after the FLYING go_to_start -> DONE go_to_start
//       edge: open, the descent on the OPEN report
//   V2  Turn starts by itself on the FLYING ready -> DONE ready edge: the close at the gate,
//       the turn on the grasp
//   V3  Exit To Turn starts by itself 0.2 s (the station's default) after the FLYING push ->
//       DONE push edge -- THE VALVE'S EXIT: open with the claw still world-held (NO release
//       call), Exit push_pull_valve_exit_wait_s (1 s) after the OPEN report
//   V4  a manual Exit inside the delay: exactly one exit
//   V5  ABORTING inside the delay: no exit, the jaws opened once
//   V6  delay 0: no automatic exit
//   V7  a latched DONE push without the edge (a restarted station): nothing
//   V8  the mocap lamp: "guard off" without a message, LOST on a LOST message
//
// Prints "GSVAL <key> <value>" lines; the last is "GSVAL result PASS|FAIL ...".
#include <QApplication>
#include <QLabel>
#include <QPushButton>
#include <QTimer>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <functional>
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
using Clock = std::chrono::steady_clock;
using Trigger = std_srvs::srv::Trigger;

void say(const std::string & key, const std::string & v)
{
  std::printf("GSVAL %s %s\n", key.c_str(), v.c_str());
  std::fflush(stdout);
}

enum class State {kAboveHandle, kOnHandle, kPushing, kPushed};

struct Rig
{
  rclcpp::Node::SharedPtr node, planner;
  PushPullPanel * panel{nullptr};
  std::vector<std::pair<std::string, Clock::time_point>> events;
  State state{State::kOnHandle};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pl_status, status;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr info, arrival, guard;
  std::vector<rclcpp::ServiceBase::SharedPtr> srvs;
  rclcpp::TimerBase::SharedPtr tick;

  ~Rig() {delete panel;}

  int count(const std::string & what) const
  {
    int n = 0;
    for (const auto & e : events) {n += e.first == what;}
    return n;
  }
  int at(const std::string & what) const
  {
    for (size_t i = 0; i < events.size(); ++i) {if (events[i].first == what) {return static_cast<int>(i);}}
    return -1;
  }
  double since(const std::string & a, const std::string & b) const   // t(b) - t(a) [s]
  {
    const int ia = at(a), ib = at(b);
    if (ia < 0 || ib < 0) {return std::nan("");}
    return std::chrono::duration<double>(events[ib].second - events[ia].second).count();
  }
  void push(const std::string & e) {events.emplace_back(e, Clock::now());}

  Rig(const std::string & gov, State s, double auto_exit_s)
  : state(s)
  {
    node = std::make_shared<rclcpp::Node>("gs_valve_auto_harness", gov + "/fsc_open_manipulator");
    node->declare_parameter<bool>("simulation", true);
    // NaN = leave it undeclared: the panel then uses its OWN default (0.2 s)
    if (std::isfinite(auto_exit_s)) {node->declare_parameter<double>("push_pull_auto_exit_delay_s", auto_exit_s);}
    node->declare_parameter<double>("push_pull_release_timeout_s", 3.0);
    node->declare_parameter<double>("push_pull_grip_timeout_s", 1.0);
    planner = std::make_shared<rclcpp::Node>("whole_body_trajectory_planner", gov);
    planner->declare_parameter<std::vector<double>>("push_pull_start_mark", {0.0, 0.0, 1.0, 0.0});
    planner->declare_parameter<std::vector<double>>("push_pull_land_start", {-0.5, 0.0, 1.3, -360.0});
    planner->declare_parameter<std::vector<double>>("push_pull_land", {-0.5, 0.0, 0.8, -360.0});
    planner->declare_parameter<std::vector<double>>("push_pull_ee_offset", {0.160, 0.0006, 0.179});
    planner->declare_parameter<double>("push_pull_approach_dz", 0.10);
    planner->declare_parameter<double>("push_pull_push_distance", 0.50);
    planner->declare_parameter<double>("push_pull_turn_deg", 360.0);   // the VALVE tab is active
    planner->declare_parameter<double>("push_pull_approach_back", 0.12);
    planner->declare_parameter<std::vector<double>>("push_pull_box_point", {1.20, 0.0, 0.80, 180.0});
    const std::string base = gov + "/whole_body_planner/push_pull";
    const auto latched = rclcpp::QoS(1).reliable().transient_local();
    pl_status = planner->create_publisher<std_msgs::msg::String>(base + "/status", latched);
    status = planner->create_publisher<std_msgs::msg::String>(gov + "/whole_body_planner/status", latched);
    info = planner->create_publisher<std_msgs::msg::Float64MultiArray>(base + "/info", latched);
    arrival = planner->create_publisher<std_msgs::msg::Float64MultiArray>(base + "/arrival_error", 10);
    guard = planner->create_publisher<std_msgs::msg::Float64MultiArray>(
      gov + "/state_estimator/mocap_guard", rclcpp::SensorDataQoS());
    for (const char * svc : {"ready", "push", "exit", "release", "abort"}) {
      const std::string name = svc;
      srvs.push_back(planner->create_service<Trigger>(
          base + "/" + name, [this, name](const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr r) {
            push(name);
            r->success = true;
            r->message = "(fake) ok";
          }));
    }
    publishState();
    tick = planner->create_wall_timer(std::chrono::milliseconds(100), [this]() {
        if (state != State::kOnHandle) {return;}
        std_msgs::msg::Float64MultiArray a;   // the claw on the grasp point, centred
        a.data.assign(21, 0.0);
        a.data[0] = 1.0;
        a.data[1] = 1.0;
        a.data[2] = 0.005;
        a.data[6] = 0.05;
        a.data[7] = 1.0;
        a.data[8] = 1.0;
        a.data[9] = 0.001;
        arrival->publish(a);
      });
    panel = new PushPullPanel(node, true);
    panel->resize(1300, 900);
    panel->show();
    QObject::connect(panel, &PushPullPanel::gripperRequested, [this](bool open) {
        push(open ? "open" : "close");
        if (open) {
          QTimer::singleShot(400, panel, [this]() {
              push("open_report");
              panel->onGripperOpenFinished();
            });
        } else {   // Isaac: the fingers stall on the fin's 20 mm shim -- the grasp
          QTimer::singleShot(400, panel, [this]() {panel->onGripperCloseResult(ControlPanel::kCloseStalled, 0.0040);});
        }
      });
  }

  void publishState(const std::string & override_status = "")
  {
    const bool on = state != State::kAboveHandle;
    std_msgs::msg::Float64MultiArray m;
    m.data.assign(69, 0.0);
    m.data[4] = 1.0;
    m.data[5] = state == State::kAboveHandle ? 0.0 : state == State::kPushed ? 2.0 : 1.0;
    m.data[6] = state == State::kPushing ? 2.0 : -1.0;
    const double t_step[6] = {8.0, 3.0, 37.5, 6.8, 23.9, 0.1};   // the valve's plan (10-18)
    for (int k = 0; k < 6; ++k) {m.data[8 + k] = t_step[k];}
    m.data[14] = 1.0;
    m.data[60] = 360.0;
    m.data[61] = on ? 1.0 : 0.0;
    m.data[66] = -90.0;
    info->publish(m);
    std_msgs::msg::String s;
    s.data = !override_status.empty() ? override_status
             : state == State::kPushing ? "FLYING push (CONTACT) T=37.5s"
             : state == State::kPushed ? "DONE push next=exit (the gripper opens first)"
             : on ? "DONE ready next=push (CONTACT on: the gripper closes first)"
                  : "DONE go_to_start next=ready (the gripper opens first) -- claw 3 mm from the point above the handle";
    pl_status->publish(s);
    s.data = "HOLD";
    status->publish(s);
  }
  void finishPush()
  {
    state = State::kPushed;
    publishState();
  }
  QPushButton * button(const QString & prefix) const
  {
    for (QPushButton * b : panel->findChildren<QPushButton *>()) {
      if (b->text().startsWith(prefix)) {return b;}
    }
    return nullptr;
  }
  bool labelWith(const std::string & text) const
  {
    for (QLabel * l : panel->findChildren<QLabel *>()) {
      if (l->text().toStdString().find(text) != std::string::npos) {return true;}
    }
    return false;
  }
};
}  // namespace

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  QApplication app(argc, argv);
  std::unique_ptr<Rig> rig;
  QTimer pump;
  QObject::connect(&pump, &QTimer::timeout, [&]() {
      if (rig) {
        rclcpp::spin_some(rig->node);
        rclcpp::spin_some(rig->planner);
      }
    });
  pump.start(5);

  std::vector<std::string> fails;
  const auto check = [&](bool ok, const std::string & what) {
      say(what, ok ? "ok" : "FAILED");
      if (!ok) {fails.push_back(what);}
    };
  struct Case
  {
    std::string name;
    State state;
    std::function<void(Rig &)> setup;
    QString press;
    int wait_ms;
    std::function<void(Rig &)> judge;
    std::function<void(Rig &)> act{};
    double auto_exit_s{std::numeric_limits<double>::quiet_NaN()};
    int tail_ms{900};
  };
  std::vector<Case> cases;
  const auto finish = [](Rig & r) {r.finishPush();};
  const auto go_done = [](Rig & r) {r.publishState();};

  cases.push_back({"V1_auto_ready_after_go_to_start", State::kAboveHandle,
    [&](Rig & r) {r.publishState("FLYING go_to_start T=8.0s");}, "", 500,
    [&](Rig & r) {
      check(r.count("open") == 0 && r.count("ready") == 0, "V1_nothing_inside_the_1s_delay");
      QTimer::singleShot(1500, r.panel, [&]() {
          check(r.count("open") == 1 && r.count("ready") == 1, "V1_open_then_ready_by_2s");
          check(r.at("open") < r.at("open_report") && r.at("open_report") < r.at("ready"),
            "V1_descent_only_on_the_open_report");
        });
    }, go_done, std::numeric_limits<double>::quiet_NaN(), 2300});
  cases.push_back({"V2_auto_turn_after_ready", State::kOnHandle,
    [&](Rig & r) {r.publishState("FLYING ready T=3.0s");}, "", 1200,
    [&](Rig & r) {
      check(r.count("close") == 1, "V2_the_gripper_closed_by_itself");
      check(r.count("push") == 1 && r.at("close") < r.at("push"), "V2_the_turn_followed_the_grasp");
    }, go_done, std::numeric_limits<double>::quiet_NaN(), 600});
  cases.push_back({"V3_auto_exit_after_the_turn_is_the_valve_exit", State::kPushing,
    [&](Rig &) {}, "", 120,
    [&](Rig & r) {
      check(r.count("open") == 0, "V3_nothing_at_0p12_s");
      QTimer::singleShot(2200, r.panel, [&]() {
          check(r.count("release") == 0, "V3_no_release_call_the_claw_stays_world_held");
          check(r.count("open") == 1 && r.count("exit") == 1, "V3_one_open_one_exit");
          const double w = r.since("open_report", "exit");
          check(r.at("open") < r.at("open_report") && r.at("open_report") < r.at("exit") && w > 0.9 && w < 1.4,
            "V3_exit_1s_after_the_open_report");
          say("V3_exit_after_open_report_s", std::to_string(w));
        });
    }, finish, std::numeric_limits<double>::quiet_NaN(), 2600});
  cases.push_back({"V4_manual_exit_inside_the_delay", State::kPushing,
    [&](Rig &) {}, "", 100,
    [&](Rig & r) {
      QPushButton * b = r.button("4  ");
      check(b && b->isEnabled(), "V4_exit_button_enabled");
      if (b && b->isEnabled()) {b->click();}
      QTimer::singleShot(2200, r.panel, [&]() {
          check(r.count("open") == 1 && r.count("exit") == 1 && r.count("release") == 0, "V4_exactly_one_exit");
        });
    }, finish, 1.0, 2600});
  cases.push_back({"V5_aborting_inside_the_delay", State::kPushing,
    [&](Rig &) {}, "", 300,
    [&](Rig & r) {
      r.publishState("ABORTING (SAFETY GUARD: q3 2.1 deg < 3.0 deg) -- gripper open, contact off, "
        "climbing 0.30 m, then the arm home");
      QTimer::singleShot(2000, r.panel, [&]() {
          check(r.count("exit") == 0, "V5_no_exit_after_ABORTING");
          check(r.count("open") == 1, "V5_ABORTING_opened_the_jaws_once");
        });
    }, finish, 1.0, 2400});
  cases.push_back({"V6_delay_zero_is_by_hand", State::kPushing,
    [&](Rig &) {}, "", 2500,
    [&](Rig & r) {check(r.count("open") == 0 && r.count("exit") == 0, "V6_no_automatic_exit");},
    finish, 0.0});
  cases.push_back({"V7_latched_done_push_no_edge", State::kPushed,
    [&](Rig &) {}, "", 2500,
    [&](Rig & r) {check(r.count("open") == 0 && r.count("exit") == 0, "V7_restarted_station_does_not_exit");}});
  cases.push_back({"V8_mocap_lamp_on_the_valve_tab", State::kAboveHandle,
    [&](Rig &) {}, "", 300,
    [&](Rig & r) {
      check(r.labelWith("guard off"), "V8_guard_off_without_a_message");
      std_msgs::msg::Float64MultiArray g;
      g.data.assign(16, 0.0);
      g.data[0] = 3.0;   // LOST
      g.data[1] = 2.0;   // frozen
      g.data[2] = 1.4;
      g.data[7] = 0.0;
      g.data[13] = g.data[14] = g.data[15] = std::nan("");
      for (int i = 0; i < 5; ++i) {r.guard->publish(g);}
      QTimer::singleShot(700, r.panel, [&]() {
          check(r.labelWith("LOST -- LAND WITH THE RC"), "V8_lamp_reads_LOST");
          if (const char * dir = std::getenv("GS_VALVE_SHOT")) {
            r.panel->grab().save(QString::fromLocal8Bit(dir) + "/gs_valve_mocap_lost.png");
          }
        });
    }, nullptr, std::numeric_limits<double>::quiet_NaN(), 1200});

  size_t next = 0;
  std::function<void()> run_next;
  run_next = [&]() {
      if (next >= cases.size()) {
        say("result", fails.empty() ? "PASS" : "FAIL " + std::to_string(fails.size()));
        app.quit();
        return;
      }
      const Case c = cases[next];
      const std::string gov = "/uav_gsv" + std::to_string(next);
      ++next;
      rig.reset();
      rig = std::make_unique<Rig>(gov, c.state, c.auto_exit_s);
      c.setup(*rig);
      QTimer::singleShot(2500, &app, [&, c]() {
          rig->events.clear();
          if (!c.press.isEmpty()) {
            QPushButton * b = rig->button(c.press);
            if (!(b && b->isEnabled())) {
              check(false, c.name + "_button_enabled");
              run_next();
              return;
            }
            b->click();
          }
          if (c.act) {c.act(*rig);}
          QTimer::singleShot(c.wait_ms, &app, [&, c]() {
              c.judge(*rig);
              QTimer::singleShot(c.tail_ms, &app, [&]() {run_next();});
            });
        });
    };
  QTimer::singleShot(500, &app, [&]() {run_next();});
  QTimer::singleShot(120000, &app, [&]() {say("result", "FAIL timeout"); app.exit(1);});
  const int rc = app.exec();
  rig.reset();
  rclcpp::shutdown();
  return rc != 0 ? rc : (fails.empty() ? 0 : 1);
}
