// gs_valve_phase_harness.cpp -- the REAL arm-GS "Valve Turning" and "Box Pushing" panels, offscreen,
// against a FAKE planner and a FAKE arm controller (no Isaac, no PX4), for the 2026-11-02 isolation
// (Pegasus archive/valve_box_isolation_20261102): the valve's phases the 10-27 box way. Run by
// ../gs_valve_phase_check.sh on a private ROS domain. Built from
// archive/valve_safety_fixes_20261030/tools/gs_valve_grasp_harness.
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
//   V3  Valve static hold (push_pull_valve_hold_s 1.0): the grasp -> grasp(true), 1 s, grasp(false),
//       OPEN with no release call, Exit after the OPEN report; no Push
//   V4  Valve static hold, ABORTING during the hold: no grasp(false), no Exit; ABORTED closes the jaws
//   V5  Valve static hold, a planner without push_pull/grasp: nothing moves (no Push, no open)
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

  Rig(const std::string & gov, bool valve, double hold_s, bool grasp_srv)
  {
    node = std::make_shared<rclcpp::Node>("gs_valve_phase_harness", gov + "/fsc_open_manipulator");
    node->declare_parameter<bool>("simulation", false);
    node->declare_parameter<double>("push_pull_release_timeout_s", 3.0);
    node->declare_parameter<double>("push_pull_grip_timeout_s", 1.0);
    node->declare_parameter<double>("push_pull_valve_grip_wait_max_s", 1.5);
    node->declare_parameter<double>("push_pull_valve_hold_s", hold_s);
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
    for (const char * svc : {"go_to_start", "ready", "push", "exit", "release", "abort"}) {
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
        a.data.assign(32, 0.0);
        a.data[0] = 1.0;
        a.data[1] = 1.0;
        a.data[2] = 0.005;
        a.data[6] = 0.05;
        a.data[7] = 1.0;
        a.data[8] = 1.0;
        a.data[9] = 0.001;
        a.data[30] = 2.0;
        a.data[31] = 0.16;
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
  struct Case {std::string name; bool valve; double hold_s; bool grasp_srv; std::vector<Step> steps;};
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
      check(r.count("close") == 2, "V2_closed_after_the_hand_over", &r);
      check(r.count("push") == 1, "V2_the_grasp_turns", &r);
      check(r.count("grasp_true") == 0, "V2_no_grasp_event_in_the_turn_mode", &r);
    }}}});
  cases.push_back({"V3_static_hold", true, 1.0, true, {
    {0, [](Rig & r) {r.publish("FLYING ready T=3.0s", true, 1);}},
    {300, [](Rig & r) {r.publish(DONE_READY, true, 1);}},
    {800, [&](Rig & r) {   // the close (0 s), its stall (0.4 s), grasp(true)
      check(r.count("grasp_true") == 1 && r.count("push") == 0, "V3_grasp_event_not_a_turn", &r);
      check(r.count("grasp_false") == 0 && r.count("open") == 0, "V3_holding", &r);
    }},
    {3000, [&](Rig & r) {   // 1 s hold, grasp(false), open (0.4 s), exit 1 s after the report
      const int gf = r.at("grasp_false"), op = r.at("open"), ex = r.at("exit");
      check(gf >= 0 && op > gf && ex > op, "V3_release_reported_then_open_then_exit", &r);
      check(r.count("release") == 0 && r.count("push") == 0, "V3_no_release_call_no_turn", &r);
    }}}});
  cases.push_back({"V4_abort_during_the_hold", true, 3.0, true, {
    {0, [](Rig & r) {r.publish("FLYING ready T=3.0s", true, 1);}},
    {300, [](Rig & r) {r.publish(DONE_READY, true, 1);}},
    {900, [&](Rig & r) {
      check(r.count("grasp_true") == 1, "V4_grasped", &r);
      r.publish("ABORTING (SAFETY GUARD: the vehicle sways 160 mm/s) -- gripper open", true, 1);
    }},
    {800, [](Rig & r) {r.publish("ABORTED (SAFETY GUARD: the vehicle sways 160 mm/s) -- holding higher", false, 1);}},
    {3500, [&](Rig & r) {
      check(r.count("grasp_false") == 0 && r.count("exit") == 0, "V4_the_hold_was_retired", &r);
      const int ab = r.at("open");
      check(ab >= 0 && r.at("close", ab) > ab, "V4_aborting_opened_aborted_closed", &r);
    }}}});
  cases.push_back({"V5_no_grasp_service_nothing_moves", true, 1.0, false, {
    {0, [](Rig & r) {r.publish("FLYING ready T=3.0s", true, 1);}},
    {300, [](Rig & r) {r.publish(DONE_READY, true, 1);}},
    {2500, [&](Rig & r) {
      check(r.count("close") == 1, "V5_closed_on_the_fin", &r);
      check(r.count("push") == 0 && r.count("open") == 0 && r.count("exit") == 0, "V5_nothing_moved", &r);
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
      const std::string gov = "/uav_gsp" + std::to_string(next);
      ++next;
      rig.reset();
      rig = std::make_unique<Rig>(gov, c.valve, c.hold_s, c.grasp_srv);
      QTimer::singleShot(2500, &app, [&, c]() {
          rig->events.clear();
          run_step(c, 0);
        });
    };
  QTimer::singleShot(500, &app, [&]() {run_next();});
  QTimer::singleShot(150000, &app, [&]() {say("result", "FAIL timeout"); app.exit(1);});
  const int rc = app.exec();
  rig.reset();
  rclcpp::shutdown();
  return rc != 0 ? rc : (fails.empty() ? 0 : 1);
}
