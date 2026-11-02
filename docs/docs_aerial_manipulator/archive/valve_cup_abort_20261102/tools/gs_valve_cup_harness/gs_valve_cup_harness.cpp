// gs_valve_cup_harness.cpp -- the REAL arm-GS "Valve Turning" and "Box Pushing" panels, offscreen,
// against a FAKE planner and a FAKE arm controller (no Isaac, no PX4), for the 2026-11-02 valve HALF
// CLOSE and ABORT (Pegasus archive/valve_cup_abort_20261102), on top of the isolation's cases (V1-V9,
// archive/valve_box_isolation_20261102/tools/gs_valve_phase_harness, copied from there). Run by
// ../gs_valve_cup_check.sh on a private ROS domain.
//
// The valve's close is now a HALF CLOSE (push_pull_valve_cup, default true; hardware position 0.0):
// the harness answers it as the gripper would -- REACHED at the requested position unless a case
// sets a stall. The fake planner also serves push_pull/abort_jaws_open and records each call.
//
//   C1  Valve: the grasp is a half close to 0.0 (no full close); reached -> the turn
//   C2  Valve: the half close stalls at +0.010 (wider than cup_max_open 0.004) -> a wrong grip, no turn
//   C3  Valve: the half close stalls at +0.002 (a pad on the fin) -> still the cup, the turn
//   A1  Valve, a GUARD abort (ABORTING) in the turn: OPEN, its report, then push_pull/abort_jaws_open
//   A2  Valve, the ABORT BUTTON in the turn: OPEN + abort, the report, abort_jaws_open; the ABORTING
//       that follows sends no second OPEN
//   A3  Valve, the ABORT BUTTON far from the handle (Go To Start flying): the jaws still open, reported
//   A4  Box tab, a guard abort: OPEN as before, NO abort_jaws_open (the box keeps its 10-27 abort)
//
// Each case builds a FRESH panel in its own vehicle namespace beside a fake planner node (turn 90 deg
// for the Valve tab, 0 for the Box tab: the tab whose mode the planner is in is active) and a fake
// external_torque_controller holding passthrough_error_damping_joints [0, 0, 0, 0.05] and
// trim_target_includes_error_damping false (the hardware yaml's). The harness plays the gripper: an
// OPEN is reported 0.4 s later; a CLOSE stalls on the fin 0.4 s later. The fake planner serves
// push_pull/grasp and records every call.
//
//   V1  Valve: -> FLYING go_to_start CLOSES the jaws (never opens them), raises the damping and sets
//       the trim switch; FLYING exit -> DONE exit closes them again and restores both
//   V2  Valve: the automatic Ready OPENS the jaws and descends on the report; the close waits for the
//       planner's hand-over (info [87] settling) and then closes; the grasp TURNS (Push), and no
//       grasp event is sent in the turn mode
//   V7  Valve turn, the automatic exit: DONE push -> waits while the base moves, then OPEN (no
//       release), Exit after the report
//   V8  Valve, push_pull_valve_box_exit true: the box's order (release, OPEN, Exit)
//   V9  Valve turn, the base-error wait (planner arrival_error [32]): 45 mm holds the exit, 10 mm
//       lets it go (calm_max 10 s, so only the error can release it)
//   (V3-V5, the static hold, were deleted with it the same day; the fake planner still serves
//   push_pull/grasp so a station that called it would show up as "grasp_true" -- none may)
//   S1  Valve, an old launch that still passes push_pull_valve_hold_s 10.0: after Ready nothing
//       closes and nothing turns; a Turn press is refused
//   V6  Box tab = 10-27: no gripper command at Go To Start or after the Exit; NOT IN DIRECT with only
//       the station's own grip state (the planner off the handle) opens nothing
//
// Prints "GSVP <key> <value>" lines; the last is "GSVP result PASS|FAIL ...".
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
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "control_panel.hpp"
#include "push_pull_panel.hpp"

namespace
{
using Clock = std::chrono::steady_clock;
using Trigger = std_srvs::srv::Trigger;
using SetBool = std_srvs::srv::SetBool;
const std::vector<double> kArmDamping{0.0, 0.0, 0.0, 0.05};
const std::vector<double> kValveDamping{0.2, 0.0, 0.0, 0.2};

void say(const std::string & key, const std::string & v)
{
  std::printf("GSVP %s %s\n", key.c_str(), v.c_str());
  std::fflush(stdout);
}

bool same(const std::vector<double> & a, const std::vector<double> & b)
{
  if (a.size() != b.size()) {return false;}
  for (size_t i = 0; i < a.size(); ++i) {if (std::abs(a[i] - b[i]) > 1e-9) {return false;}}
  return true;
}

struct Rig
{
  rclcpp::Node::SharedPtr node, planner, arm;
  PushPullPanel * panel{nullptr};
  std::vector<std::string> events;
  bool on_handle{false};
  bool settling{false};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pl_status, status;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr info, arrival;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr base_pub;
  rclcpp::TimerBase::SharedPtr base_tick;
  double base_speed = -1.0, base_x = 0.0, base_err = 0.005;
  // the half close's answer (2026-11-02): kCloseReached at the requested position by default
  int cup_outcome = ControlPanel::kCloseReached;
  double cup_result = std::nan("");
  double cup_requested = std::nan("");
  std::vector<rclcpp::ServiceBase::SharedPtr> srvs;
  rclcpp::TimerBase::SharedPtr tick;
  std::string last_status{"READY next=go_to_start"};
  int last_done{0};

  ~Rig() {delete panel;}

  int count(const std::string & what) const
  {
    int n = 0;
    for (const auto & e : events) {n += e == what;}
    return n;
  }
  // the index of the first `what` at or after `from` (-1: none)
  int at(const std::string & what, int from = 0) const
  {
    for (int i = from; i < static_cast<int>(events.size()); ++i) {if (events[i] == what) {return i;}}
    return -1;
  }
  std::string trace() const
  {
    std::string t;
    for (const auto & e : events) {t += (t.empty() ? "" : ",") + e;}
    return t.empty() ? "-" : t;
  }
  std::vector<double> damping() const
  {
    return arm->get_parameter("passthrough_error_damping_joints").as_double_array();
  }
  bool trim() const {return arm->get_parameter("trim_target_includes_error_damping").as_bool();}

  Rig(const std::string & gov, bool valve, double hold_s, bool grasp_srv, bool box_exit = false,
      double calm_max_s = 2.0)
  {
    // hold_s > 0: an OLD launch that still asks for the removed static hold (a parameter override)
    rclcpp::NodeOptions opts;
    if (hold_s > 0.0) {opts.parameter_overrides({rclcpp::Parameter("push_pull_valve_hold_s", hold_s)});}
    node = std::make_shared<rclcpp::Node>("gs_valve_phase_harness", gov + "/fsc_open_manipulator", opts);
    node->declare_parameter<bool>("simulation", false);
    node->declare_parameter<double>("push_pull_release_timeout_s", 3.0);
    node->declare_parameter<double>("push_pull_grip_timeout_s", 1.0);
    node->declare_parameter<double>("push_pull_valve_grip_wait_max_s", 1.5);
    node->declare_parameter<double>("push_pull_auto_exit_calm_max_s", calm_max_s);
    node->declare_parameter<double>("push_pull_valve_exit_wait_s", 0.5);
    if (box_exit) {node->declare_parameter<bool>("push_pull_valve_box_exit", true);}
    arm = std::make_shared<rclcpp::Node>("external_torque_controller", gov + "/fsc_open_manipulator");
    arm->declare_parameter<std::vector<double>>("passthrough_error_damping_joints", kArmDamping);
    arm->declare_parameter<bool>("trim_target_includes_error_damping", false);
    planner = std::make_shared<rclcpp::Node>("whole_body_trajectory_planner", gov);
    planner->declare_parameter<std::vector<double>>("push_pull_start_mark", {-1.0, 0.0, 1.0, 0.0});
    planner->declare_parameter<std::vector<double>>("push_pull_land_start", {-1.0, 0.0, 1.3, -180.0});
    planner->declare_parameter<std::vector<double>>("push_pull_land", {-1.0, 0.0, 0.3, -180.0});
    planner->declare_parameter<std::vector<double>>("push_pull_ee_offset", {0.160, 0.0006, 0.179});
    planner->declare_parameter<double>("push_pull_approach_dz", 0.10);
    planner->declare_parameter<double>("push_pull_push_distance", 0.40);
    planner->declare_parameter<double>("push_pull_turn_deg", valve ? 90.0 : 0.0);
    planner->declare_parameter<double>("push_pull_approach_back", 0.12);
    planner->declare_parameter<std::vector<double>>("push_pull_box_point", {0.60, 0.0, 0.89, 180.0});
    const std::string base = gov + "/whole_body_planner/push_pull";
    const auto latched = rclcpp::QoS(1).reliable().transient_local();
    pl_status = planner->create_publisher<std_msgs::msg::String>(base + "/status", latched);
    status = planner->create_publisher<std_msgs::msg::String>(gov + "/whole_body_planner/status", latched);
    info = planner->create_publisher<std_msgs::msg::Float64MultiArray>(base + "/info", latched);
    arrival = planner->create_publisher<std_msgs::msg::Float64MultiArray>(base + "/arrival_error", 10);
    base_pub = planner->create_publisher<geometry_msgs::msg::PoseStamped>(gov + "/whole_body_planner/current_base", 10);
    base_tick = planner->create_wall_timer(std::chrono::milliseconds(50), [this]() {
        if (base_speed < 0.0) {return;}   // < 0: no base pose at all
        base_x += base_speed * 0.05;
        geometry_msgs::msg::PoseStamped m;
        m.pose.position.x = base_x;
        m.pose.position.z = 1.2;
        base_pub->publish(m);
      });
    for (const char * svc : {"go_to_start", "ready", "push", "exit", "release", "abort", "abort_jaws_open"}) {
      const std::string name = svc;
      srvs.push_back(planner->create_service<Trigger>(
          base + "/" + name, [this, name](const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr r) {
            events.push_back(name);
            r->success = true;
            r->message = "(fake) ok";
          }));
    }
    if (grasp_srv) {
      srvs.push_back(planner->create_service<SetBool>(
          base + "/grasp", [this](const SetBool::Request::SharedPtr q, SetBool::Response::SharedPtr r) {
            events.push_back(q->data ? "grasp_true" : "grasp_false");
            r->success = true;
            r->message = q->data ? "(fake) grasp on: the contact phase is on" : "(fake) grasp off";
          }));
    }
    publish("READY next=go_to_start");
    tick = planner->create_wall_timer(std::chrono::milliseconds(100), [this]() {
        if (!on_handle) {return;}
        std_msgs::msg::Float64MultiArray a;   // the claw on the grasp point, centred, still, heading ok
        a.data.assign(34, 0.0);
        a.data[0] = 1.0;
        a.data[1] = 1.0;
        a.data[2] = 0.005;
        a.data[6] = 0.05;
        a.data[7] = 1.0;
        a.data[8] = 1.0;
        a.data[9] = 0.001;
        a.data[30] = 2.0;
        a.data[31] = 0.16;
        a.data[32] = base_err;   // the law's CoM error (2026-11-02)
        arrival->publish(a);
      });
    panel = new PushPullPanel(node, valve);
    panel->resize(1300, 900);
    panel->show();
    QObject::connect(panel, &PushPullPanel::gripperRequested, [this](bool open) {
        events.push_back(open ? "open" : "close");
        if (open) {
          QTimer::singleShot(400, panel, [this]() {events.push_back("open_report"); panel->onGripperOpenFinished();});
        } else {
          QTimer::singleShot(400, panel, [this]() {panel->onGripperCloseResult(ControlPanel::kCloseStalled, -0.0090);});   // on the fin (hw -0.0097)
        }
      });
    // the valve's half close: reached at the requested position, or the case's stall
    QObject::connect(panel, &PushPullPanel::gripperCupRequested, [this](double position) {
        events.push_back("cup");
        cup_requested = position;
        QTimer::singleShot(400, panel, [this, position]() {
            panel->onGripperCloseResult(cup_outcome, std::isfinite(cup_result) ? cup_result : position);
          });
      });
  }

  void publish(const std::string & st, bool on = false, int done = 0, bool settle = false)
  {
    on_handle = on;
    settling = settle;
    last_status = st;
    last_done = done;
    std_msgs::msg::Float64MultiArray m;
    m.data.assign(89, 0.0);
    m.data[4] = 1.0;
    m.data[5] = done;
    m.data[6] = -1.0;
    const double t_step[6] = {19.0, 3.0, 19.5, 6.8, 25.0, 7.4};
    for (int k = 0; k < 6; ++k) {m.data[8 + k] = t_step[k];}
    m.data[14] = 1.0;
    m.data[60] = 90.0;
    m.data[61] = on ? 1.0 : 0.0;
    m.data[66] = 90.0;
    m.data[87] = settle ? 1.0 : 0.0;
    info->publish(m);
    std_msgs::msg::String s;
    s.data = st;
    pl_status->publish(s);
    s.data = "HOLD";
    status->publish(s);
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
        rclcpp::spin_some(rig->arm);
      }
    });
  pump.start(5);

  std::vector<std::string> fails;
  const auto check = [&](bool ok, const std::string & what, const Rig * r = nullptr) {
      say(what, ok ? "ok" : ("FAILED" + (r ? " [" + r->trace() + "]" : std::string())));
      if (!ok) {fails.push_back(what);}
    };
  using Step = std::pair<int, std::function<void(Rig &)>>;
  struct Case {std::string name; bool valve; double hold_s; bool grasp_srv; std::vector<Step> steps; bool box_exit = false;
    double calm_max_s = 2.0;};
  std::vector<Case> cases;
  static const std::string DONE_READY = "DONE ready next=push (CONTACT on: the gripper closes first)";

  cases.push_back({"V1_closed_in_free_flight", true, 0.0, true, {
    {0, [](Rig & r) {r.publish("FLYING go_to_start T=19.0s");}},
    {1500, [&](Rig & r) {
      check(r.count("close") == 1 && r.count("open") == 0, "V1_go_to_start_closes_never_opens", &r);
      check(same(r.damping(), kValveDamping), "V1_damping_raised", &r);
      check(r.trim(), "V1_trim_switch_on_for_the_mission", &r);
      r.publish("FLYING exit T=6.8s", false, 2);
    }},
    {500, [](Rig & r) {r.publish("DONE exit next=go_to_land_start", false, 3);}},
    {1200, [&](Rig & r) {
      check(r.count("close") == 2 && r.count("open") == 0, "V1_closed_again_after_the_exit", &r);
      check(same(r.damping(), kArmDamping), "V1_damping_restored", &r);
      check(!r.trim(), "V1_trim_switch_restored", &r);
    }}}});
  cases.push_back({"V2_ready_opens_hand_over_close_turn", true, 0.0, true, {
    {0, [](Rig & r) {r.publish("FLYING go_to_start T=19.0s");}},
    {600, [](Rig & r) {r.publish("DONE go_to_start next=ready", false, 0);}},
    {2200, [&](Rig & r) {   // the automatic Ready (1.0 s) opened and descended on the report
      check(r.at("open") > r.at("close") && r.count("ready") == 1, "V2_auto_ready_opened_then_descended", &r);
      r.publish("FLYING ready T=3.0s", true, 1);
    }},
    {300, [](Rig & r) {r.publish(DONE_READY + " (settling)", true, 1, true);}},
    {1000, [&](Rig & r) {
      check(r.count("close") == 1, "V2_no_close_while_the_planner_settles", &r);   // only Go To Start's
      r.publish(DONE_READY, true, 1, false);
    }},
    {1200, [&](Rig & r) {
      check(r.count("close") == 1 && r.count("cup") == 1, "V2_half_closed_after_the_hand_over", &r);
      check(r.count("push") == 1, "V2_the_grasp_turns", &r);
      check(r.count("grasp_true") == 0 && r.count("grasp_false") == 0, "V2_no_grasp_event_ever", &r);
    }}}});
  cases.push_back({"V7_turn_auto_exit_box_way", true, 0.0, true, {
    {0, [](Rig & r) {r.base_speed = 0.10; r.publish("FLYING push (CONTACT) T=19.5s", true, 1);}},   // base at 100 mm/s
    {300, [](Rig & r) {r.publish("DONE push next=exit (the gripper opens first)", true, 2);}},
    {600, [&](Rig & r) {   // 0.2 s + 0.4 s: the base still moving -> no release yet
      check(r.count("release") == 0 && r.count("open") == 0, "V7_waits_for_a_calm_base", &r);
      r.base_speed = 0.0;   // the base calms
    }},
    {2000, [&](Rig & r) {
      const int op = r.at("open"), ex = r.at("exit");
      check(r.count("release") == 0 && op >= 0 && ex > op, "V7_open_then_exit_no_release", &r);
    }}}});
  cases.push_back({"V8_box_exit_switch", true, 0.0, true, {
    {0, [](Rig & r) {r.publish("FLYING push (CONTACT) T=19.5s", true, 1);}},
    {300, [](Rig & r) {r.publish("DONE push next=exit (the gripper opens first)", true, 2);}},
    {1800, [&](Rig & r) {   // 0.2 s, release, open, report 0.4 s, exit on it
      const int rl = r.at("release"), op = r.at("open"), ex = r.at("exit");
      check(rl >= 0 && op > rl && ex > op, "V8_release_open_exit", &r);
    }}}, true});
  cases.push_back({"V9_base_error_wait", true, 0.0, true, {
    {0, [](Rig & r) {r.base_speed = 0.0; r.base_err = 0.045; r.publish("FLYING push (CONTACT) T=19.5s", true, 1);}},
    {300, [](Rig & r) {r.publish("DONE push next=exit (the gripper opens first)", true, 2);}},
    {1500, [&](Rig & r) {   // still, but 45 mm off: no exit
      check(r.count("open") == 0 && r.count("release") == 0, "V9_holds_while_the_base_is_off", &r);
      r.base_err = 0.010;
    }},
    {1500, [&](Rig & r) {   // 10 mm: calm 0.3 s, open, report 0.4 s, exit 0.5 s after
      const int op = r.at("open"), ex = r.at("exit");
      check(op >= 0 && ex > op && r.count("release") == 0, "V9_exits_once_the_base_is_back", &r);
    }}}, false, 10.0});
  // ---- THE STATIC HOLD WAS REMOVED (2026-11-02): an old launch asking for one turns nothing ----
  cases.push_back({"S1_stale_hold_parameter_turns_nothing", true, 10.0, true, {
    {0, [](Rig & r) {r.publish("FLYING ready T=3.0s", true, 1);}},
    {300, [](Rig & r) {r.publish(DONE_READY, true, 1);}},
    {1500, [&](Rig & r) {
      check(r.count("cup") == 0 && r.count("close") == 0, "S1_no_close_after_ready", &r);
      check(r.count("push") == 0 && r.count("grasp_true") == 0, "S1_no_turn_no_grasp_event", &r);
      bool clicked = false;
      for (auto * b : r.panel->findChildren<QPushButton *>()) {
        if (b->text().startsWith("3  Turn") && b->isEnabled()) {b->click(); clicked = true; break;}
      }
      check(clicked, "S1_turn_button_pressed", &r);
    }},
    {1500, [&](Rig & r) {
      check(r.count("cup") == 0 && r.count("push") == 0, "S1_turn_press_refused", &r);
    }}}});
  // ---- THE HALF CLOSE (2026-11-02) ----
  cases.push_back({"C1_cup_reached_turns", true, 0.0, true, {
    {0, [](Rig & r) {r.publish("FLYING ready T=3.0s", true, 1);}},
    {300, [](Rig & r) {r.publish(DONE_READY, true, 1);}},
    {1200, [&](Rig & r) {
      check(r.count("cup") == 1 && r.count("close") == 0, "C1_half_close_not_a_full_close", &r);
      check(r.count("grasp_true") == 0, "C1_no_grasp_event", &r);
      check(std::abs(r.cup_requested - 0.0) < 1e-9, "C1_half_close_to_0.0_on_hardware", &r);
      check(r.count("push") == 1, "C1_the_cup_turns", &r);
    }}}});
  cases.push_back({"C2_cup_stall_wide_refused", true, 0.0, true, {
    {0, [](Rig & r) {
      r.cup_outcome = ControlPanel::kCloseStalled;
      r.cup_result = 0.010;
      r.publish("FLYING ready T=3.0s", true, 1);
    }},
    {300, [](Rig & r) {r.publish(DONE_READY, true, 1);}},
    {1200, [&](Rig & r) {
      check(r.count("cup") == 1 && r.count("push") == 0, "C2_wide_stall_is_a_wrong_grip_no_turn", &r);
    }}}});
  cases.push_back({"C3_cup_stall_narrow_accepted", true, 0.0, true, {
    {0, [](Rig & r) {
      r.cup_outcome = ControlPanel::kCloseStalled;
      r.cup_result = 0.002;
      r.publish("FLYING ready T=3.0s", true, 1);
    }},
    {300, [](Rig & r) {r.publish(DONE_READY, true, 1);}},
    {1200, [&](Rig & r) {
      check(r.count("cup") == 1 && r.count("push") == 1, "C3_a_pad_on_the_fin_still_turns", &r);
    }}}});
  // ---- THE ABORT: OPEN, its report, the planner climbs (2026-11-02) ----
  cases.push_back({"A1_guard_abort_reports_the_open", true, 0.0, true, {
    {0, [](Rig & r) {r.publish("FLYING push (CONTACT) T=19.5s", true, 1);}},
    {300, [](Rig & r) {r.publish("ABORTING (SAFETY GUARD: the vehicle sways 160 mm/s) -- gripper open", true, 1);}},
    {1000, [&](Rig & r) {
      const int op = r.at("open"), rp = r.at("open_report"), jo = r.at("abort_jaws_open");
      check(op >= 0 && rp > op && jo > rp, "A1_open_report_then_abort_jaws_open", &r);
      check(r.count("abort_jaws_open") == 1, "A1_reported_once", &r);
    }}}});
  cases.push_back({"A2_button_abort_reports_the_open", true, 0.0, true, {
    {0, [](Rig & r) {r.publish("FLYING push (CONTACT) T=19.5s", true, 1);}},
    {300, [&](Rig & r) {
      bool clicked = false;
      for (auto * b : r.panel->findChildren<QPushButton *>()) {
        if (b->text() == "ABORT" && b->isEnabled()) {b->click(); clicked = true; break;}
      }
      check(clicked, "A2_abort_button_clicked", &r);
    }},
    {100, [](Rig & r) {r.publish("ABORTING (operator) -- gripper open", true, 1);}},
    {1000, [&](Rig & r) {
      const int op = r.at("open"), ab = r.at("abort"), rp = r.at("open_report"), jo = r.at("abort_jaws_open");
      check(op >= 0 && ab >= 0 && rp > op && jo > rp, "A2_open_abort_report_then_abort_jaws_open", &r);
      check(r.count("open") == 1, "A2_no_second_open_on_aborting", &r);
    }}}});
  cases.push_back({"A3_button_abort_far_from_the_handle_opens_too", true, 0.0, true, {
    {0, [](Rig & r) {r.publish("FLYING go_to_start T=19.0s");}},
    {1200, [&](Rig & r) {
      check(r.count("close") == 1 && r.count("open") == 0, "A3_closed_in_free_flight", &r);
      for (auto * b : r.panel->findChildren<QPushButton *>()) {
        if (b->text() == "ABORT" && b->isEnabled()) {b->click(); break;}
      }
    }},
    {1000, [&](Rig & r) {
      const int op = r.at("open"), jo = r.at("abort_jaws_open");
      check(op >= 0 && jo > op, "A3_opened_and_reported", &r);
    }}}});
  cases.push_back({"A4_box_abort_no_report", false, 0.0, true, {
    {0, [](Rig & r) {r.publish("FLYING push (CONTACT) T=19.5s", true, 1);}},
    {300, [](Rig & r) {r.publish("ABORTING (SAFETY GUARD: box tilted 2.1 deg) -- gripper open", true, 1);}},
    {1000, [&](Rig & r) {
      check(r.count("open") == 1 && r.count("open_report") == 1, "A4_box_opens_on_aborting", &r);
      check(r.count("abort_jaws_open") == 0, "A4_box_sends_no_report", &r);
    }}}});
  cases.push_back({"V6_box_tab_is_10_27", false, 0.0, true, {
    {0, [](Rig & r) {r.publish("FLYING go_to_start T=19.0s");}},
    {1200, [&](Rig & r) {
      check(r.count("close") == 0 && r.count("open") == 0, "V6_box_no_gripper_at_go_to_start", &r);
      check(same(r.damping(), kArmDamping) && !r.trim(), "V6_box_arm_untouched", &r);
      r.publish("FLYING ready T=3.0s", true, 1);
    }},
    {300, [](Rig & r) {r.publish(DONE_READY, true, 1);}},
    {1200, [&](Rig & r) {
      check(r.count("push") == 1, "V6_box_pushed_on_the_grasp", &r);
      r.publish("PLANNING push_pull ...", false, 1);   // the planner's last word: off the handle
    }},
    {400, [](Rig & r) {r.publish("NOT IN DIRECT", false, 1);}},
    {800, [&](Rig & r) {
      check(r.count("open") == 0, "V6_box_not_in_direct_reads_the_planner_only", &r);
      r.publish("FLYING exit T=6.8s", false, 2);
    }},
    {300, [](Rig & r) {r.publish("DONE exit next=go_to_land_start", false, 3);}},
    {800, [&](Rig & r) {check(r.count("close") == 1, "V6_box_no_close_after_the_exit", &r);}}}});

  size_t next = 0;
  std::function<void()> run_next;
  std::function<void(const Case &, size_t)> run_step;
  run_step = [&](const Case & c, size_t i) {
      if (i >= c.steps.size()) {QTimer::singleShot(300, &app, [&]() {run_next();}); return;}
      QTimer::singleShot(c.steps[i].first, &app, [&, c, i]() {
          c.steps[i].second(*rig);
          run_step(c, i + 1);
        });
    };
  run_next = [&]() {
      if (next >= cases.size()) {
        say("result", fails.empty() ? "PASS" : "FAIL " + std::to_string(fails.size()));
        app.quit();
        return;
      }
      const Case c = cases[next];
      const std::string gov = "/uav_gsc" + std::to_string(next);
      ++next;
      rig.reset();
      rig = std::make_unique<Rig>(gov, c.valve, c.hold_s, c.grasp_srv, c.box_exit, c.calm_max_s);
      QTimer::singleShot(2500, &app, [&, c]() {
          rig->events.clear();
          run_step(c, 0);
        });
    };
  QTimer::singleShot(500, &app, [&]() {run_next();});
  QTimer::singleShot(240000, &app, [&]() {say("result", "FAIL timeout"); app.exit(1);});
  const int rc = app.exec();
  rig.reset();
  rclcpp::shutdown();
  return rc != 0 ? rc : (fails.empty() ? 0 : 1);
}
