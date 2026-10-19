// gs_grasp_harness.cpp -- the REAL arm-GS "Box Pushing" panel, offscreen, against a
// FAKE planner (no Isaac, no PX4, no planner node), for the 2026-10-19 hardware
// changes. Run by ../gs_grasp_check.sh on a private ROS domain.
//
// Each case builds a FRESH panel in its own vehicle namespace beside a fake planner
// node (whole_body_trajectory_planner: the push-and-pull parameters, status, info,
// arrival_error and the step services, every call recorded). The harness plays the
// gripper: an OPEN is reported (onGripperOpenFinished) 0.4 s later unless the case
// says never; a CLOSE ends 0.4 s later with the case's outcome and finger position
// (onGripperCloseResult), or never.
//
//   H  the HARDWARE rule (simulation false): a full close IS the grasp; a stall wider
//      than push_pull_grip_max_open is a wrong grip; a failed / missing close result
//      clears the wait; Push pressed again while gripping pushes at once
//   S  the SIMULATION rule (simulation true): a stall is the grasp, a full close is not
//   R  Ready opens and descends only on the OPEN report
//   A  ABORT opens the jaws only when this mode holds the handle, and only once
//   E  nothing that drops the plan is editable while the claw is on the handle
//   X  the AUTOMATIC exit: push_pull_auto_exit_delay_s after the planner's FLYING
//      push -> DONE push edge -- release, open, exit on the OPEN report; never on a
//      latched DONE push without the edge, never after ABORTING / NOT IN DIRECT /
//      a manual Exit inside the delay, never with the delay 0
//
// Prints "GSVAL <key> <value>" lines; the last is "GSVAL result PASS|FAIL ...".
#include <QApplication>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QPushButton>
#include <QTimer>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <functional>
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

// the planner state a case starts from
enum class State {kAboveHandle, kOnHandle, kPushing, kPushed};

struct Rig
{
  rclcpp::Node::SharedPtr node, planner;
  PushPullPanel * panel{nullptr};
  std::vector<std::string> events;
  bool report_open{true};
  int close_outcome{ControlPanel::kCloseReached};   // -1: no result at all
  double close_pos{-0.0092};
  bool push_ok{true};
  State state{State::kOnHandle};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pl_status, status;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr info, arrival;
  std::vector<rclcpp::ServiceBase::SharedPtr> srvs;
  rclcpp::TimerBase::SharedPtr tick;

  ~Rig() {delete panel;}

  int count(const std::string & what) const
  {
    int n = 0;
    for (const auto & e : events) {n += e == what;}
    return n;
  }

  Rig(const std::string & gov, bool simulation, State s, double auto_exit_s)
  : state(s)
  {
    node = std::make_shared<rclcpp::Node>("gs_grasp_harness", gov + "/fsc_open_manipulator");
    node->declare_parameter<bool>("simulation", simulation);
    node->declare_parameter<double>("push_pull_auto_exit_delay_s", auto_exit_s);
    node->declare_parameter<double>("push_pull_release_timeout_s", 1.0);
    node->declare_parameter<double>("push_pull_grip_timeout_s", 1.0);
    // the fake planner: its parameters make the tab load (and count as box pushing)
    planner = std::make_shared<rclcpp::Node>("whole_body_trajectory_planner", gov);
    planner->declare_parameter<std::vector<double>>("push_pull_start_mark", {0.0, 0.0, 1.0, 0.0});
    planner->declare_parameter<std::vector<double>>("push_pull_land_start", {-0.5, 0.0, 1.3, -360.0});
    planner->declare_parameter<std::vector<double>>("push_pull_land", {-0.5, 0.0, 0.8, -360.0});
    planner->declare_parameter<std::vector<double>>("push_pull_ee_offset", {0.0, 0.0, 0.183});
    planner->declare_parameter<double>("push_pull_approach_dz", 0.15);
    planner->declare_parameter<double>("push_pull_push_distance", 0.50);
    planner->declare_parameter<double>("push_pull_turn_deg", 0.0);
    planner->declare_parameter<double>("push_pull_approach_back", 0.12);
    planner->declare_parameter<std::vector<double>>("push_pull_box_point", {1.275, 0.28, 0.85, 0.0});
    const std::string base = gov + "/whole_body_planner/push_pull";
    const auto latched = rclcpp::QoS(1).reliable().transient_local();
    pl_status = planner->create_publisher<std_msgs::msg::String>(base + "/status", latched);
    status = planner->create_publisher<std_msgs::msg::String>(gov + "/whole_body_planner/status", latched);
    info = planner->create_publisher<std_msgs::msg::Float64MultiArray>(base + "/info", latched);
    arrival = planner->create_publisher<std_msgs::msg::Float64MultiArray>(base + "/arrival_error", 10);
    for (const char * svc : {"ready", "push", "exit", "release", "abort"}) {
      const std::string name = svc;
      srvs.push_back(planner->create_service<Trigger>(
          base + "/" + name, [this, name](const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr r) {
            events.push_back(name);
            r->success = name != "push" || push_ok;
            r->message = r->success ? "(fake) ok" : "(fake) refused";
            if (name == "push" && !push_ok) {push_ok = true;}   // refused once
          }));
    }
    publishState();
    tick = planner->create_wall_timer(std::chrono::milliseconds(100), [this]() {
        if (state != State::kOnHandle) {return;}
        std_msgs::msg::Float64MultiArray a;   // the claw on the grasp point, centred
        a.data.assign(21, 0.0);
        a.data[0] = 1.0;       // step: ready
        a.data[1] = 1.0;       // a claw target
        a.data[2] = 0.005;     // 5 mm from the grasp point
        a.data[6] = 0.05;
        a.data[7] = 1.0;
        a.data[8] = 1.0;       // holding
        a.data[9] = 0.001;     // 1 mm across the fin
        arrival->publish(a);
      });
    panel = new PushPullPanel(node);
    panel->resize(1300, 900);
    panel->show();
    QObject::connect(panel, &PushPullPanel::gripperRequested, [this](bool open) {
        events.push_back(open ? "open" : "close");
        if (open && report_open) {
          QTimer::singleShot(400, panel, [this]() {
              events.push_back("open_report");
              panel->onGripperOpenFinished();
            });
        } else if (!open && close_outcome >= 0) {
          QTimer::singleShot(400, panel, [this]() {panel->onGripperCloseResult(close_outcome, close_pos);});
        }
      });
  }

  void publishState(const std::string & override_status = "")
  {
    const bool on = state != State::kAboveHandle;
    std_msgs::msg::Float64MultiArray m;
    m.data.assign(69, 0.0);
    m.data[4] = 1.0;                  // planned
    // done: go_to_start / ready / push
    m.data[5] = state == State::kAboveHandle ? 0.0 : state == State::kPushed ? 2.0 : 1.0;
    m.data[6] = state == State::kPushing ? 2.0 : -1.0;   // the push flying, else nothing
    // the steps' durations [s] (test_push_pull_loopback.py on the sim box-pushing yaml)
    const double t_step[6] = {10.4, 3.2, 25.5, 6.0, 23.9, 5.1};
    for (int k = 0; k < 6; ++k) {m.data[8 + k] = t_step[k];}
    m.data[14] = 1.0;                 // box captured
    m.data[61] = on ? 1.0 : 0.0;      // at the handle
    m.data[66] = -90.0;
    info->publish(m);
    std_msgs::msg::String s;
    s.data = !override_status.empty() ? override_status
             : state == State::kPushing ? "FLYING push (CONTACT) T=25.5s"
             : state == State::kPushed ? "DONE push next=exit (the gripper opens first)"
             : on ? "DONE ready next=push (CONTACT on: the gripper closes first)"
                  : "DONE go_to_start next=ready (the gripper opens first) -- claw 3 mm from the point above the handle";
    pl_status->publish(s);
    s.data = "HOLD";
    status->publish(s);
  }

  // the planner's FLYING push -> DONE push edge (info first: the panel re-checks it)
  void finishPush()
  {
    state = State::kPushed;
    publishState();
  }
  int at(const std::string & what) const   // index of the first such event, -1 = none
  {
    for (size_t i = 0; i < events.size(); ++i) {if (events[i] == what) {return static_cast<int>(i);}}
    return -1;
  }

  QPushButton * button(const QString & prefix) const
  {
    for (QPushButton * b : panel->findChildren<QPushButton *>()) {
      if (b->text().startsWith(prefix)) {return b;}
    }
    return nullptr;
  }
  std::string lamp() const
  {
    std::string best;   // the status lamp: the longest bold label text
    for (QLabel * l : panel->findChildren<QLabel *>()) {
      if (l->styleSheet().contains("font-weight: bold") && l->text().size() > static_cast<int>(best.size())) {
        best = l->text().toStdString();
      }
    }
    return best;
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
    bool simulation;
    State state;
    std::function<void(Rig &)> setup;
    QString press;                 // a button text prefix, "" = none
    int wait_ms;
    std::function<void(Rig &)> judge;
    std::function<void(Rig &)> act{};   // instead of / besides a press, at the same moment
    double auto_exit_s{1.0};           // push_pull_auto_exit_delay_s
    int tail_ms{900};                  // after the judge, before the next case
  };
  std::vector<Case> cases;
  const auto hw = [](Rig & r, int outcome, double pos) {r.close_outcome = outcome; r.close_pos = pos;};

  // ---- the HARDWARE grasp rule -------------------------------------------------
  cases.push_back({"H1_full_close_is_the_grasp", false, State::kOnHandle,
    [&](Rig & r) {hw(r, ControlPanel::kCloseReached, -0.0092);}, "3  ", 1500,
    [&](Rig & r) {check(r.count("close") == 1 && r.count("push") == 1, "H1_closed_then_pushed");}});
  cases.push_back({"H2_wide_stall_is_a_wrong_grip", false, State::kOnHandle,
    [&](Rig & r) {hw(r, ControlPanel::kCloseStalled, -0.0010);}, "3  ", 1500,
    [&](Rig & r) {
      check(r.count("close") == 1 && r.count("push") == 0, "H2_no_push");
      check(r.lamp().find("WRONG GRIP") != std::string::npos, "H2_says_wrong_grip");
    }});
  cases.push_back({"H3_narrow_stall_is_the_grasp", false, State::kOnHandle,
    [&](Rig & r) {hw(r, ControlPanel::kCloseStalled, -0.0085);}, "3  ", 1500,
    [&](Rig & r) {check(r.count("push") == 1, "H3_pushed");}});
  cases.push_back({"H4_failed_close", false, State::kOnHandle,
    [&](Rig & r) {hw(r, ControlPanel::kCloseFailed, NAN);}, "3  ", 1500,
    [&](Rig & r) {
      check(r.count("push") == 0, "H4_no_push");
      check(r.lamp().find("did not complete") != std::string::npos, "H4_says_so");
      QPushButton * b = r.button("3  ");
      check(b && b->isEnabled(), "H4_push_usable_again");
    }});
  cases.push_back({"H5_no_close_result", false, State::kOnHandle,
    [&](Rig & r) {hw(r, -1, NAN);}, "3  ", 2200,
    [&](Rig & r) {
      check(r.count("push") == 0, "H5_no_push");
      QPushButton * b = r.button("3  ");
      check(b && b->isEnabled(), "H5_push_usable_again_after_the_timeout");
    }});
  cases.push_back({"H6_push_again_while_gripping", false, State::kOnHandle,
    [&](Rig & r) {hw(r, ControlPanel::kCloseReached, -0.0092); r.push_ok = false;}, "3  ", 1500,
    [&](Rig & r) {
      check(r.count("close") == 1 && r.count("push") == 1, "H6_first_push_refused");
      QPushButton * b = r.button("3  ");
      if (b && b->isEnabled()) {b->click();}
      QTimer::singleShot(600, r.panel, [&]() {
          check(r.count("close") == 1 && r.count("push") == 2, "H6_second_press_pushes_without_a_new_close");
        });
    }});
  // ---- the SIMULATION rule ------------------------------------------------------
  cases.push_back({"S1_full_close_is_nothing", true, State::kOnHandle,
    [&](Rig & r) {hw(r, ControlPanel::kCloseReached, -0.0100);}, "3  ", 1500,
    [&](Rig & r) {
      check(r.count("push") == 0, "S1_no_push");
      check(r.lamp().find("NOTHING") != std::string::npos, "S1_says_nothing_in_the_jaws");
    }});
  cases.push_back({"S2_stall_is_the_grasp", true, State::kOnHandle,
    [&](Rig & r) {hw(r, ControlPanel::kCloseStalled, 0.0040);}, "3  ", 1500,
    [&](Rig & r) {check(r.count("push") == 1, "S2_pushed");}});
  // ---- Ready waits for the OPEN report -------------------------------------------
  cases.push_back({"R1_ready_on_the_report", false, State::kAboveHandle,
    [&](Rig & r) {r.report_open = true;}, "2  ", 1200,
    [&](Rig & r) {
      check(r.events.size() >= 3 && r.events[0] == "open" && r.events[1] == "open_report" &&
        r.events[2] == "ready" && r.count("ready") == 1, "R1_open_report_then_ready");
    }});
  cases.push_back({"R2_no_report_no_descent", false, State::kAboveHandle,
    [&](Rig & r) {r.report_open = false;}, "2  ", 1800,
    [&](Rig & r) {
      check(r.count("ready") == 0, "R2_no_ready");
      QPushButton * b = r.button("2  ");
      check(b && b->isEnabled(), "R2_ready_usable_again");
    }});
  // ---- ABORT ---------------------------------------------------------------------
  cases.push_back({"A1_abort_off_the_handle", false, State::kAboveHandle,
    [&](Rig &) {}, "ABORT", 800,
    [&](Rig & r) {check(r.count("abort") == 1 && r.count("open") == 0, "A1_abort_without_opening");}});
  cases.push_back({"A2_abort_on_the_handle", false, State::kOnHandle,
    [&](Rig &) {}, "ABORT", 300,
    [&](Rig & r) {
      check(r.count("abort") == 1 && r.count("open") == 1, "A2_abort_and_one_open");
      r.publishState("ABORTING -- gripper open, contact off, climbing 0.30 m, then the arm home");
      QTimer::singleShot(600, r.panel, [&]() {check(r.count("open") == 1, "A2_no_second_open_on_ABORTING");});
    }});
  // ---- edits on the handle --------------------------------------------------------
  cases.push_back({"E1_no_edits_on_the_handle", false, State::kOnHandle,
    [&](Rig &) {}, "", 200,
    [&](Rig & r) {
      bool any_spin = false;
      for (QDoubleSpinBox * s : r.panel->findChildren<QDoubleSpinBox *>()) {any_spin = any_spin || s->isEnabled();}
      check(!any_spin, "E1_no_editable_box");
      for (const QString t : {"Adjust", "EE Offset", "Vertical Margin", "Side Margin", "Push Distance"}) {
        QPushButton * b = r.button(t);
        check(b && !b->isEnabled(), ("E1_" + t + "_disabled").toStdString());
      }
    }});
  cases.push_back({"E2_edits_above_the_handle", false, State::kAboveHandle,
    [&](Rig &) {}, "", 200,
    [&](Rig & r) {
      QPushButton * b = r.button("Vertical Margin");
      check(b && b->isEnabled(), "E2_editable_off_the_handle");
    }});
  // ---- the AUTOMATIC exit after the push -------------------------------------------
  const auto finish = [](Rig & r) {r.finishPush();};
  cases.push_back({"X1_auto_exit_after_the_push", false, State::kPushing,
    [&](Rig &) {}, "", 700,
    [&](Rig & r) {
      check(r.count("release") == 0 && r.count("open") == 0, "X1_nothing_inside_the_delay");
      check(r.lamp().find("starts by itself") != std::string::npos, "X1_lamp_says_it_will_start");
      if (const char * dir = std::getenv("GS_GRASP_SHOT")) {   // the workflow page's picture
        r.panel->grab().save(QString::fromLocal8Bit(dir) + "/gs_auto_exit_pending.png");
      }
      QPushButton * b = r.button("4  ");
      check(b && b->isEnabled(), "X1_exit_button_usable_inside_the_delay");
      QTimer::singleShot(1500, r.panel, [&]() {
          check(r.count("release") == 1 && r.count("open") == 1 && r.count("exit") == 1,
            "X1_one_release_one_open_one_exit");
          check(r.at("release") >= 0 && r.at("release") < r.at("open") &&
            r.at("open") < r.at("open_report") && r.at("open_report") < r.at("exit"),
            "X1_release_then_open_then_exit_on_the_report");
        });
    }, finish, 1.0, 2000});
  cases.push_back({"X2_latched_done_push_no_edge", false, State::kPushed,
    [&](Rig &) {}, "", 2500,
    [&](Rig & r) {
      check(r.count("release") == 0 && r.count("open") == 0 && r.count("exit") == 0,
        "X2_restarted_station_does_not_exit_by_itself");
      check(r.lamp().find("starts by itself") == std::string::npos, "X2_lamp_promises_nothing");
    }});
  cases.push_back({"X3_aborting_inside_the_delay", false, State::kPushing,
    [&](Rig &) {}, "", 400,
    [&](Rig & r) {
      r.publishState("ABORTING (SAFETY GUARD: tilt 11 deg > 10 deg) -- gripper open, contact off, "
        "climbing 0.30 m, then the arm home");
      QTimer::singleShot(1300, r.panel, [&]() {
          check(r.count("release") == 0 && r.count("exit") == 0, "X3_no_release_no_exit_after_ABORTING");
          check(r.count("open") == 1, "X3_ABORTING_opened_the_jaws_once");
        });
    }, finish, 1.0, 1800});
  cases.push_back({"X4_manual_exit_inside_the_delay", false, State::kPushing,
    [&](Rig &) {}, "", 300,
    [&](Rig & r) {
      QPushButton * b = r.button("4  ");
      check(b && b->isEnabled(), "X4_exit_button_enabled");
      if (b && b->isEnabled()) {b->click();}
      QTimer::singleShot(1700, r.panel, [&]() {
          check(r.count("release") == 1 && r.count("open") == 1 && r.count("exit") == 1,
            "X4_exactly_one_exit");
        });
    }, finish, 1.0, 2200});
  cases.push_back({"X5_delay_zero_is_by_hand", false, State::kPushing,
    [&](Rig &) {}, "", 2500,
    [&](Rig & r) {
      check(r.count("release") == 0 && r.count("open") == 0 && r.count("exit") == 0,
        "X5_no_automatic_exit");
      check(r.lamp().find("starts by itself") == std::string::npos, "X5_lamp_promises_nothing");
    }, finish, 0.0});
  cases.push_back({"X6_not_in_direct_inside_the_delay", false, State::kPushing,
    [&](Rig &) {}, "", 400,
    [&](Rig & r) {
      r.publishState("NOT IN DIRECT");
      QTimer::singleShot(1300, r.panel, [&]() {
          check(r.count("release") == 0 && r.count("exit") == 0, "X6_no_release_no_exit");
          check(r.count("open") == 1, "X6_jaws_opened_once_on_the_DIRECT_loss");
        });
    }, finish, 1.0, 1800});

  size_t next = 0;
  std::function<void()> run_next;
  run_next = [&]() {
      if (next >= cases.size()) {
        say("result", fails.empty() ? "PASS" : "FAIL " + std::to_string(fails.size()));
        app.quit();
        return;
      }
      const Case c = cases[next];
      const std::string gov = "/uav_gsg" + std::to_string(next);
      ++next;
      rig.reset();
      rig = std::make_unique<Rig>(gov, c.simulation, c.state, c.auto_exit_s);
      c.setup(*rig);
      // let discovery, the latched topics and the parameter load settle, then press
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
  QTimer::singleShot(180000, &app, [&]() {say("result", "FAIL timeout"); app.exit(1);});
  const int rc = app.exec();
  rig.reset();
  rclcpp::shutdown();
  return rc != 0 ? rc : (fails.empty() ? 0 : 1);
}
