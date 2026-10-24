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
//   C  (2026-10-24) the close gate: 8 mm across on hardware / 3 mm in Isaac, 25 mm, no close
//      while the claw sweeps (> 30 mm/s), a close once it stops, ABORT after 2.5 s
//   G  (2026-10-21) the AUTOMATIC Ready To Push + Push: Ready push_pull_auto_ready_delay_s
//      after the FLYING go_to_start -> DONE go_to_start edge (a settle refusal retried),
//      Push on the FLYING ready -> DONE ready edge (the close at the gate, the push on the
//      grasp); never on a latched DONE without the edge, never after ABORT in the delay
//   P  (2026-10-24 afternoon) the planner's HAND-OVER SETTLE (info [87]): no close and no
//      2.5 s limit while it settles, the close once it hands over; the settle's own limit
//      (push_pull_box_settle_wait_max_s) aborts; the DONE ready status arriving before the
//      info that says settling does not start the 2.5 s limit
//   K  (2026-10-24 afternoon) the CALM automatic exit: while the vehicle (current_base) moves
//      faster than 40 mm/s the exit waits, then starts 0.3 s after it is calm; never calm ->
//      it starts anyway after 3 s; no current_base -> at the delay (the X cases)
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
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
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
  int ready_refusals{0};   // the next N ready calls are refused "wait for it to settle" (A2)
  // (2026-10-24) the claw's error the fake planner reports in the hold: across the fin
  // (world x, [3] and [9]), along it ([4]); sweep_mm_s > 0 sweeps it across between
  // +-sweep_amp at that speed (a triangle), until sweep_stop_s after the case starts
  double across{0.001}, along{0.0049}, sweep_mm_s{0.0}, sweep_amp{0.006}, sweep_stop_s{1e9};
  Clock::time_point t0{Clock::now()};
  State state{State::kOnHandle};
  // (2026-10-24 afternoon) the hand-over settle: info [87] (published only with info_size 89;
  // 69 = an older planner), and the vehicle's speed on current_base (< 0: not published)
  size_t info_size{69};
  bool settling{false};
  double base_speed{-1.0}, base_x{0.0};
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr base_pub;
  rclcpp::TimerBase::SharedPtr base_tick;
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

  Rig(const std::string & gov, bool simulation, State s, double auto_exit_s, double settle_wait_s)
  : state(s)
  {
    node = std::make_shared<rclcpp::Node>("gs_grasp_harness", gov + "/fsc_open_manipulator");
    node->declare_parameter<bool>("simulation", simulation);
    if (std::isfinite(settle_wait_s)) {
      node->declare_parameter<double>("push_pull_box_settle_wait_max_s", settle_wait_s);
    }
    // NaN = leave it undeclared: the panel then uses its OWN default (the X7 case, 2026-10-21)
    if (std::isfinite(auto_exit_s)) {node->declare_parameter<double>("push_pull_auto_exit_delay_s", auto_exit_s);}
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
            if (name == "ready" && ready_refusals > 0) {
              --ready_refusals;
              r->success = false;
              r->message = "the claw is 62 mm from the point above the handle (> 50 mm) -- wait for it to settle";
              return;
            }
            r->success = name != "push" || push_ok;
            r->message = r->success ? "(fake) ok" : "(fake) refused";
            if (name == "push" && !push_ok) {push_ok = true;}   // refused once
          }));
    }
    base_pub = planner->create_publisher<geometry_msgs::msg::PoseStamped>(
      gov + "/whole_body_planner/current_base", 10);
    base_tick = planner->create_wall_timer(std::chrono::milliseconds(67), [this]() {
        if (base_speed < 0.0) {return;}
        base_x += base_speed * 0.067;
        geometry_msgs::msg::PoseStamped b;
        b.header.frame_id = "map";
        b.pose.position.x = base_x;
        b.pose.position.z = 1.1;
        b.pose.orientation.w = 1.0;
        base_pub->publish(b);
      });
    publishState();
    tick = planner->create_wall_timer(std::chrono::milliseconds(100), [this]() {
        if (state != State::kOnHandle) {return;}
        std_msgs::msg::Float64MultiArray a;   // the claw on the grasp point, centred
        a.data.assign(21, 0.0);
        double x = across;
        const double ts = std::chrono::duration<double>(Clock::now() - t0).count();
        if (sweep_mm_s > 0.0 && ts < sweep_stop_s) {   // a triangle between +-sweep_amp
          const double period = 4.0 * sweep_amp / (1e-3 * sweep_mm_s);
          const double ph = std::fmod(ts, period) / period;
          x = sweep_amp * (ph < 0.5 ? -1.0 + 4.0 * ph : 3.0 - 4.0 * ph);
        }
        a.data[0] = 1.0;       // step: ready
        a.data[1] = 1.0;       // a claw target
        a.data[2] = std::hypot(x, along);   // default: 5 mm from the grasp point
        a.data[3] = x;         // the error, world (x across the fin, y along)
        a.data[4] = along;
        a.data[6] = 0.05;
        a.data[7] = 1.0;
        a.data[8] = 1.0;       // holding
        a.data[9] = x;         // across the fin (default 1 mm)
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
    m.data.assign(info_size, 0.0);
    if (info_size >= 89) {m.data[87] = settling ? 1.0 : 0.0;}
    m.data[4] = 1.0;                  // planned
    // done: go_to_start / ready / push
    m.data[5] = state == State::kAboveHandle ? 0.0 : state == State::kPushed ? 2.0 : 1.0;
    m.data[6] = state == State::kPushing ? 2.0 : -1.0;   // the push flying, else nothing
    // the steps' durations [s] (test_push_pull_loopback.py on the sim box-pushing yaml)
    // the 0.10 m push at 21 mm/s (the 2026-10-21 workflow; was the 0.50 m plan's 25.5 s push)
    const double t_step[6] = {10.7, 3.2, 6.3, 6.0, 23.9, 7.4};
    for (int k = 0; k < 6; ++k) {m.data[8 + k] = t_step[k];}
    m.data[14] = 1.0;                 // box captured
    m.data[61] = on ? 1.0 : 0.0;      // at the handle
    m.data[66] = -90.0;
    info->publish(m);
    std_msgs::msg::String s;
    s.data = !override_status.empty() ? override_status
             : state == State::kPushing ? "FLYING push (CONTACT) T=6.3s"
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
    double settle_wait_s{NAN};         // push_pull_box_settle_wait_max_s (NaN: the panel's 6 s)
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
  // 2026-10-21: the station's DEFAULT delay is 0.2 s (was 1.0; the first hardware push tripped
  // the q3 guard 0.78 s after the push ended) -- nothing at 0.12 s, the whole exit by 0.62 s
  cases.push_back({"X7_default_delay_is_0p2_s", false, State::kPushing,
    [&](Rig &) {}, "", 120,
    [&](Rig & r) {
      check(r.count("release") == 0 && r.count("open") == 0, "X7_nothing_at_0p12_s");
      check(r.lamp().find("0.2 s") != std::string::npos, "X7_lamp_names_0p2_s");
      if (const char * dir = std::getenv("GS_GRASP_SHOT_DEFAULT")) {   // the workflow page's picture (0.2 s)
        r.panel->grab().save(QString::fromLocal8Bit(dir) + "/gs_auto_exit_pending.png");
      }
      QTimer::singleShot(500, r.panel, [&]() {
          check(r.count("release") == 1 && r.count("open") == 1 && r.count("exit") == 1,
            "X7_release_open_exit_by_0p62_s");
        });
    }, finish, std::numeric_limits<double>::quiet_NaN(), 1200});

  // ---- 2026-10-21: the AUTOMATIC Ready To Push + Push --------------------------------
  // the planner's FLYING go_to_start -> DONE go_to_start edge: Ready starts by itself after
  // push_pull_auto_ready_delay_s (1.0 s, the default), the descent on the OPEN report
  const auto go_done = [](Rig & r) {r.publishState();};
  cases.push_back({"G1_auto_ready_after_go_to_start", false, State::kAboveHandle,
    [&](Rig & r) {r.publishState("FLYING go_to_start T=10.7s");}, "", 500,
    [&](Rig & r) {
      check(r.count("open") == 0 && r.count("ready") == 0, "G1_nothing_inside_the_1s_delay");
      check(r.lamp().find("starts by itself in 1.0 s") != std::string::npos, "G1_lamp_says_it_will_start");
      if (const char * dir = std::getenv("GS_GRASP_SHOT_DEFAULT")) {
        r.panel->grab().save(QString::fromLocal8Bit(dir) + "/gs_auto_ready_pending.png");
      }
      QTimer::singleShot(1500, r.panel, [&]() {
          check(r.count("open") == 1 && r.count("ready") == 1, "G1_open_then_ready_by_2s");
          check(r.at("open") < r.at("open_report") && r.at("open_report") < r.at("ready"),
            "G1_descent_only_on_the_open_report");
        });
    }, go_done, 1.0, 2300});
  cases.push_back({"G2_auto_ready_retries_a_settle_refusal", false, State::kAboveHandle,
    [&](Rig & r) {r.ready_refusals = 2; r.publishState("FLYING go_to_start T=10.7s");}, "", 2800,
    [&](Rig & r) {
      check(r.count("ready") == 3, "G2_two_refusals_then_the_ready");
      check(r.count("open") == 1, "G2_the_gripper_opened_once");
    }, go_done, 1.0, 600});
  // the planner's FLYING ready -> DONE ready edge: Push starts by itself -- the gripper closes
  // at the first arrival sample in the gate, and the push follows the grasp
  cases.push_back({"G3_auto_push_after_ready", false, State::kOnHandle,
    [&](Rig & r) {r.publishState("FLYING ready T=3.2s");}, "", 1200,
    [&](Rig & r) {
      check(r.count("close") == 1, "G3_the_gripper_closed_by_itself");
      check(r.count("push") == 1 && r.at("close") < r.at("push"), "G3_the_push_followed_the_grasp");
    }, go_done, 1.0, 600});
  cases.push_back({"G4_latched_done_ready_no_auto_push", false, State::kOnHandle,
    [&](Rig &) {}, "", 1500,
    [&](Rig & r) {
      check(r.count("close") == 0 && r.count("push") == 0, "G4_restarted_station_does_not_push_by_itself");
    }, nullptr, 1.0, 300});
  cases.push_back({"G5_abort_inside_the_ready_delay", false, State::kAboveHandle,
    [&](Rig & r) {r.publishState("FLYING go_to_start T=10.7s");}, "", 300,
    [&](Rig & r) {
      QPushButton * b = r.button("ABORT");
      if (b) {b->click();}
      QTimer::singleShot(1500, r.panel, [&]() {
          check(r.count("ready") == 0, "G5_no_ready_after_ABORT");
        });
    }, go_done, 1.0, 1900});

  // ---- 2026-10-24: the Box Pushing close gate (the 10-23 hardware grasp) -----------------
  // hardware: 25 mm / 8 mm across / slower than 30 mm/s, else ABORT after 2.5 s; Isaac: 3 mm
  cases.push_back({"C1_hardware_closes_7mm_across", false, State::kOnHandle,
    [&](Rig & r) {r.across = 0.007; r.along = 0.020;}, "3  ", 1500,
    [&](Rig & r) {check(r.count("close") == 1 && r.count("push") == 1, "C1_closed_at_7mm_across_21mm_off");}});
  cases.push_back({"C2_isaac_keeps_3mm", true, State::kOnHandle,
    [&](Rig & r) {r.across = 0.007; hw(r, ControlPanel::kCloseStalled, 0.0040);}, "3  ", 1500,
    [&](Rig & r) {check(r.count("close") == 0, "C2_isaac_no_close_at_7mm");},
    nullptr, 1.0, 1800});   // its 2.5 s wait limit fires in the tail; nothing checks it here
  cases.push_back({"C3_no_close_while_sweeping", false, State::kOnHandle,
    // turning outside the gate (+-20 mm): every 0.3 s window ending inside it is mid-ramp
    [&](Rig & r) {r.sweep_mm_s = 40.0; r.sweep_amp = 0.020;}, "3  ", 1800,
    [&](Rig & r) {
      check(r.count("close") == 0, "C3_no_close_in_a_40mm_s_sweep_inside_the_gate");
      check(r.lamp().find("mm/s") != std::string::npos, "C3_lamp_shows_the_speed");
      if (const char * dir = std::getenv("GS_GRASP_SHOT")) {
        r.panel->grab().save(QString::fromLocal8Bit(dir) + "/gs_close_gate_sweep.png");
      }
    }, nullptr, 1.0, 1500});
  cases.push_back({"C4_closes_once_it_stops", false, State::kOnHandle,
    [&](Rig & r) {r.sweep_mm_s = 40.0; r.sweep_amp = 0.020; r.sweep_stop_s = 3.3;}, "3  ", 2200,
    [&](Rig & r) {
      check(r.count("close") == 1 && r.count("push") == 1 && r.count("abort") == 0,
        "C4_closed_after_the_sweep_stopped");
    }});
  cases.push_back({"C5_wait_limit_aborts", false, State::kOnHandle,
    [&](Rig & r) {r.across = 0.015;}, "3  ", 2000,
    [&](Rig & r) {
      check(r.count("abort") == 0 && r.count("close") == 0, "C5_nothing_before_2p5_s");
      QTimer::singleShot(900, r.panel, [&]() {
          check(r.count("abort") == 1 && r.count("close") == 0, "C5_abort_at_2p5_s_no_close");
          check(r.lamp().find("close gate") != std::string::npos, "C5_lamp_says_why");
        });
    }, nullptr, 1.0, 1300});
  cases.push_back({"C6_auto_push_waits_for_a_sweep_too", false, State::kOnHandle,
    [&](Rig & r) {r.sweep_mm_s = 40.0; r.sweep_amp = 0.020; r.sweep_stop_s = 3.6; r.publishState("FLYING ready T=3.2s");},
    "", 2400,
    [&](Rig & r) {
      check(r.count("close") == 1 && r.count("push") == 1 && r.count("abort") == 0,
        "C6_auto_close_after_the_sweep_stopped");
    }, [](Rig & r) {r.publishState();}, 1.0, 600});

  // ---- the hand-over settle (2026-10-24 afternoon) -----------------------------------
  cases.push_back({"P1_close_waits_for_the_hand_over", false, State::kOnHandle,
    [&](Rig & r) {r.info_size = 89; r.settling = true; r.publishState("FLYING ready T=3.2s");},
    "", 3000,
    [&](Rig & r) {
      check(r.count("close") == 0 && r.count("abort") == 0, "P1_no_close_no_abort_for_3s_while_settling");
      check(r.lamp().find("settling") != std::string::npos, "P1_lamp_says_settling");
      if (const char * dir = std::getenv("GS_GRASP_SHOT")) {   // the workflow page's picture
        r.panel->grab().save(QString::fromLocal8Bit(dir) + "/gs_settle_wait.png");
      }
      r.settling = false;
      r.publishState();
      QTimer::singleShot(800, r.panel, [&]() {
          check(r.count("close") == 1 && r.count("push") == 1 && r.count("abort") == 0,
            "P1_closed_and_pushed_after_the_hand_over");
        });
    }, [](Rig & r) {r.publishState();}, 1.0, 1500});
  cases.push_back({"P2_settle_limit_aborts", false, State::kOnHandle,
    [&](Rig & r) {r.info_size = 89; r.settling = true; r.publishState("FLYING ready T=3.2s");},
    "", 1000,
    [&](Rig & r) {
      check(r.count("abort") == 0, "P2_nothing_at_1s");
      QTimer::singleShot(1000, r.panel, [&]() {
          check(r.count("abort") == 1 && r.count("close") == 0, "P2_abort_at_1p5s_no_close");
          check(r.lamp().find("did not settle") != std::string::npos, "P2_lamp_says_why");
        });
    }, [](Rig & r) {r.publishState();}, 1.0, 1500, 1.5});
  cases.push_back({"P3_status_before_the_settling_info", false, State::kAboveHandle,
    [&](Rig & r) {r.info_size = 89; r.publishState("FLYING ready T=3.2s");},
    "", 3200,
    [&](Rig & r) {
      check(r.count("close") == 0 && r.count("abort") == 0, "P3_no_close_no_2p5s_abort_while_settling");
      r.settling = false;
      r.publishState();
      QTimer::singleShot(800, r.panel, [&]() {
          check(r.count("close") == 1 && r.count("abort") == 0, "P3_closed_after_the_hand_over");
        });
    }, [](Rig & r) {
      std_msgs::msg::String st;   // the DONE ready status first, the info still the old one
      st.data = "DONE ready next=push (settling: the claw 30 mm from the grasp point)";
      r.pl_status->publish(st);
      QTimer::singleShot(150, r.panel, [&r]() {
          r.state = State::kOnHandle;
          r.settling = true;
          r.publishState();
        });
    }, 1.0, 1500});
  // ---- the calm automatic exit (2026-10-24 afternoon) --------------------------------
  cases.push_back({"K1_exit_waits_for_a_calm_vehicle", false, State::kPushing,
    [&](Rig & r) {r.base_speed = 0.10;}, "", 1000,
    [&](Rig & r) {
      check(r.count("release") == 0 && r.count("open") == 0, "K1_no_release_while_moving_100mm_s");
      check(r.lamp().find("calm") != std::string::npos, "K1_lamp_says_it_waits_for_calm");
      if (const char * dir = std::getenv("GS_GRASP_SHOT")) {   // the workflow page's picture
        r.panel->grab().save(QString::fromLocal8Bit(dir) + "/gs_exit_calm.png");
      }
      r.base_speed = 0.0;
      QTimer::singleShot(1700, r.panel, [&]() {
          check(r.count("release") == 1 && r.count("open") == 1 && r.count("exit") == 1,
            "K1_release_open_exit_once_calm");
        });
    }, [](Rig & r) {r.finishPush();}, 0.2, 2200});
  cases.push_back({"K2_never_calm_exits_after_3s", false, State::kPushing,
    [&](Rig & r) {r.base_speed = 0.10;}, "", 2400,
    [&](Rig & r) {
      check(r.count("release") == 0, "K2_no_release_at_2p4s");
      QTimer::singleShot(1700, r.panel, [&]() {
          check(r.count("release") == 1 && r.count("exit") == 1, "K2_release_and_exit_by_4p1s");
        });
    }, [](Rig & r) {r.finishPush();}, 0.2, 2200});
  cases.push_back({"K3_calm_vehicle_exits_at_once", false, State::kPushing,
    [&](Rig & r) {r.base_speed = 0.005;}, "", 1400,
    [&](Rig & r) {
      check(r.count("release") == 1 && r.count("exit") == 1, "K3_release_and_exit_within_1p4s");
    }, [](Rig & r) {r.finishPush();}, 0.2, 900});

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
      rig = std::make_unique<Rig>(gov, c.simulation, c.state, c.auto_exit_s, c.settle_wait_s);
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
  QTimer::singleShot(300000, &app, [&]() {say("result", "FAIL timeout"); app.exit(1);});
  const int rc = app.exec();
  rig.reset();
  rclcpp::shutdown();
  return rc != 0 ? rc : (fails.empty() ? 0 : 1);
}
