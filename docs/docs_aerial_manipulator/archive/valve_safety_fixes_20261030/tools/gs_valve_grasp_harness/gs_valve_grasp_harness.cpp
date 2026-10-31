// gs_valve_grasp_harness.cpp -- the REAL arm-GS "Valve Turning" panel, offscreen, against a
// FAKE planner and a FAKE arm controller (no Isaac, no PX4), for the 10-29 valve report's items
// 6 and 7 (2026-10-30). Run by ../gs_valve_grasp_check.sh on a private ROS domain. Built from
// archive/valve_turning_safety_20261022/tools/gs_valve_auto_harness.
//
// Each case builds a FRESH valve panel in its own vehicle namespace beside a fake planner node
// (push_pull_turn_deg 90: the valve tab is active) and a fake external_torque_controller holding
// passthrough_error_damping_joints [0, 0, 0, 0.05] (the hardware yaml's). The harness plays the
// gripper: an OPEN is reported 0.4 s later; a CLOSE stalls on the fin 0.4 s later.
//
//   G1  -> FLYING go_to_start: the jaws open at once, and the arm's damping goes to
//       [0.2, 0, 0, 0.2]; DONE exit puts [0, 0, 0, 0.05] back
//   G2  an ABORTED mission puts it back too; a refusal-like PLANNING status does not
//   G3  the close gate waits while the law's heading error (arrival_error [30]) is 25 deg and
//       closes once it is 5 deg
//   G4  a heading error that never comes down: ABORT at the gate's wait limit, jaws never closed
//   G5  an older planner (no [30]): the gate closes on position alone
//   G6  an arm that refuses the damping: nothing to restore, the mission goes on
//
// Prints "GSVG <key> <value>" lines; the last is "GSVG result PASS|FAIL ...".
#include <QApplication>
#include <QPushButton>
#include <QTimer>

#include <chrono>
#include <cmath>
#include <cstdio>
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
const std::vector<double> kArmDamping{0.0, 0.0, 0.0, 0.05};
const std::vector<double> kValveDamping{0.2, 0.0, 0.0, 0.2};

void say(const std::string & key, const std::string & v)
{
  std::printf("GSVG %s %s\n", key.c_str(), v.c_str());
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
  std::vector<std::pair<std::string, Clock::time_point>> events;
  bool on_handle{false};
  double head{std::nan("")};   // NaN: an older planner's arrival_error (no [30])
  bool arm_refuses{false};
  int arm_sets{0};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pl_status, status;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr info, arrival;
  std::vector<rclcpp::ServiceBase::SharedPtr> srvs;
  rclcpp::TimerBase::SharedPtr tick;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr arm_cb;

  ~Rig() {delete panel;}

  int count(const std::string & what) const
  {
    int n = 0;
    for (const auto & e : events) {n += e.first == what;}
    return n;
  }
  void push(const std::string & e) {events.emplace_back(e, Clock::now());}
  std::vector<double> damping() const
  {
    return arm->get_parameter("passthrough_error_damping_joints").as_double_array();
  }

  explicit Rig(const std::string & gov)
  {
    node = std::make_shared<rclcpp::Node>("gs_valve_grasp_harness", gov + "/fsc_open_manipulator");
    node->declare_parameter<bool>("simulation", false);
    node->declare_parameter<double>("push_pull_release_timeout_s", 3.0);
    node->declare_parameter<double>("push_pull_grip_timeout_s", 1.0);
    node->declare_parameter<double>("push_pull_valve_grip_wait_max_s", 1.5);
    arm = std::make_shared<rclcpp::Node>("external_torque_controller", gov + "/fsc_open_manipulator");
    arm->declare_parameter<std::vector<double>>("passthrough_error_damping_joints", kArmDamping);
    arm_cb = arm->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> &) {
        rcl_interfaces::msg::SetParametersResult r;
        r.successful = !arm_refuses;
        if (arm_refuses) {r.reason = "(fake) the arm is in current mode";} else {++arm_sets;}
        return r;
      });
    planner = std::make_shared<rclcpp::Node>("whole_body_trajectory_planner", gov);
    planner->declare_parameter<std::vector<double>>("push_pull_start_mark", {-1.0, 0.0, 1.0, 0.0});
    planner->declare_parameter<std::vector<double>>("push_pull_land_start", {-1.0, 0.0, 1.3, -180.0});
    planner->declare_parameter<std::vector<double>>("push_pull_land", {-1.0, 0.0, 0.3, -180.0});
    planner->declare_parameter<std::vector<double>>("push_pull_ee_offset", {0.160, 0.0006, 0.179});
    planner->declare_parameter<double>("push_pull_approach_dz", 0.10);
    planner->declare_parameter<double>("push_pull_push_distance", 0.50);
    planner->declare_parameter<double>("push_pull_turn_deg", 90.0);   // the VALVE tab is active
    planner->declare_parameter<double>("push_pull_approach_back", 0.12);
    planner->declare_parameter<std::vector<double>>("push_pull_box_point", {0.60, 0.0, 0.89, 180.0});
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
            push(name);
            r->success = true;
            r->message = "(fake) ok";
          }));
    }
    publish("READY next=go_to_start");
    tick = planner->create_wall_timer(std::chrono::milliseconds(100), [this]() {
        if (!on_handle) {return;}
        std_msgs::msg::Float64MultiArray a;   // the claw on the grasp point, centred and still
        a.data.assign(std::isfinite(head) ? 32 : 21, 0.0);
        a.data[0] = 1.0;
        a.data[1] = 1.0;
        a.data[2] = 0.005;
        a.data[6] = 0.05;
        a.data[7] = 1.0;
        a.data[8] = 1.0;
        a.data[9] = 0.001;
        if (std::isfinite(head)) {
          a.data[30] = head;
          a.data[31] = 0.16;
        }
        arrival->publish(a);
      });
    panel = new PushPullPanel(node, true);
    panel->resize(1300, 900);
    panel->show();
    QObject::connect(panel, &PushPullPanel::gripperRequested, [this](bool open) {
        push(open ? "open" : "close");
        if (open) {
          QTimer::singleShot(400, panel, [this]() {push("open_report"); panel->onGripperOpenFinished();});
        } else {
          QTimer::singleShot(400, panel, [this]() {panel->onGripperCloseResult(ControlPanel::kCloseStalled, 0.0040);});
        }
      });
  }

  void publish(const std::string & st, bool on = false, int done = 0)
  {
    on_handle = on;
    std_msgs::msg::Float64MultiArray m;
    m.data.assign(69, 0.0);
    m.data[4] = 1.0;
    m.data[5] = done;
    m.data[6] = -1.0;
    const double t_step[6] = {19.0, 3.0, 19.5, 6.8, 25.0, 7.4};
    for (int k = 0; k < 6; ++k) {m.data[8 + k] = t_step[k];}
    m.data[14] = 1.0;
    m.data[60] = 90.0;
    m.data[61] = on ? 1.0 : 0.0;
    m.data[66] = 90.0;
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
  const auto check = [&](bool ok, const std::string & what) {
      say(what, ok ? "ok" : "FAILED");
      if (!ok) {fails.push_back(what);}
    };
  // a case = a list of (delay ms, action) steps run in order on a fresh rig
  using Step = std::pair<int, std::function<void(Rig &)>>;
  struct Case {std::string name; std::vector<Step> steps;};
  std::vector<Case> cases;
  const std::string DONE_READY = "DONE ready next=push (CONTACT on: the gripper closes first)";

  cases.push_back({"G1_open_and_damping_at_go_to_start", {
    {0, [](Rig & r) {r.publish("FLYING go_to_start T=19.0s");}},
    {1500, [&](Rig & r) {
      check(r.count("open") == 1, "G1_jaws_opened_at_go_to_start");
      check(same(r.damping(), kValveDamping), "G1_damping_raised_for_the_mission");
      r.publish("DONE exit next=go_to_land_start", false, 3);
    }},
    {1200, [&](Rig & r) {
      check(same(r.damping(), kArmDamping), "G1_damping_back_after_the_exit");
      check(r.arm_sets == 2, "G1_two_sets_raise_and_restore");
    }}}});
  cases.push_back({"G2_aborted_restores_planning_does_not", {
    {0, [](Rig & r) {r.publish("FLYING go_to_start T=19.0s");}},
    {1200, [](Rig & r) {r.publish("PLANNING push_pull ...", false, 1);}},
    {800, [&](Rig & r) {
      check(same(r.damping(), kValveDamping), "G2_planning_keeps_the_damping");
      r.publish("ABORTED (SAFETY GUARD: the vehicle sways 160 mm/s) -- holding higher", false, 1);
    }},
    {1200, [&](Rig & r) {check(same(r.damping(), kArmDamping), "G2_aborted_restores_it");}}}});
  cases.push_back({"G3_heading_gate_waits_then_closes", {
    {0, [](Rig & r) {r.head = 25.0; r.publish("FLYING ready T=3.0s", true, 1);}},
    {300, [&](Rig & r) {r.publish(DONE_READY, true, 1);}},
    {900, [&](Rig & r) {
      check(r.count("close") == 0, "G3_no_close_at_25_deg");
      r.head = 5.0;
    }},
    {900, [&](Rig & r) {check(r.count("close") == 1, "G3_closed_at_5_deg");}}}});
  cases.push_back({"G4_heading_never_comes_down_aborts", {
    {0, [](Rig & r) {r.head = 25.0; r.publish("FLYING ready T=3.0s", true, 1);}},
    {300, [&](Rig & r) {r.publish(DONE_READY, true, 1);}},
    {2200, [&](Rig & r) {
      check(r.count("close") == 0, "G4_never_closed");
      check(r.count("abort") == 1, "G4_aborted_at_the_wait_limit");
    }}}});
  cases.push_back({"G5_older_planner_position_only", {
    {0, [](Rig & r) {r.publish("FLYING ready T=3.0s", true, 1);}},
    {300, [&](Rig & r) {r.publish(DONE_READY, true, 1);}},
    {900, [&](Rig & r) {check(r.count("close") == 1, "G5_closed_without_a_heading");}}}});
  cases.push_back({"G6_arm_refuses_the_damping", {
    {0, [](Rig & r) {r.arm_refuses = true; r.publish("FLYING go_to_start T=19.0s");}},
    {1200, [&](Rig & r) {
      check(r.count("open") == 1, "G6_jaws_still_opened");
      check(same(r.damping(), kArmDamping) && r.arm_sets == 0, "G6_damping_unchanged");
      r.arm_refuses = false;
      r.publish("DONE exit next=go_to_land_start", false, 3);
    }},
    {1000, [&](Rig & r) {check(r.arm_sets == 0, "G6_nothing_to_restore");}}}});

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
      const std::string gov = "/uav_gsg" + std::to_string(next);
      ++next;
      rig.reset();
      rig = std::make_unique<Rig>(gov);
      QTimer::singleShot(2500, &app, [&, c]() {
          rig->events.clear();
          run_step(c, 0);
        });
    };
  QTimer::singleShot(500, &app, [&]() {run_next();});
  QTimer::singleShot(120000, &app, [&]() {say("result", "FAIL timeout"); app.exit(1);});
  const int rc = app.exec();
  rig.reset();
  rclcpp::shutdown();
  return rc != 0 ? rc : (fails.empty() ? 0 : 1);
}
