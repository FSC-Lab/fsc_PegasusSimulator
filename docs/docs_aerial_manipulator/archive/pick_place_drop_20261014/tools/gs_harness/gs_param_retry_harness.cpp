// gs_param_retry_harness.cpp -- the arm GS pick tabs survive a LOST parameter
// reply (2026-10-15), offscreen, against a FAKE planner whose parameter
// services are hand-made so they can drop a request without answering it -- what
// the live planner logged on a fresh launch ("failed to send response to
// .../get_parameters (timeout): client will not receive response"), after which
// the Pick & Drop tab stayed grey: it never asked twice.
//
// The fake planner drops (never answers) the FIRST main load (the request naming
// pick_place_approach_dz) and the FIRST Back Margin load, and answers everything
// else. The tab must ask again and end up loaded: the margins showing the
// planner's values (not the boxes' defaults) and enabled.
//
// Prints "GSVAL <key> <value>" lines.
//
//   QT_QPA_PLATFORM=offscreen ROS_DOMAIN_ID=77 ./gs_param_retry_harness
#include <QApplication>
#include <QDoubleSpinBox>
#include <QGridLayout>
#include <QPushButton>
#include <QTimer>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/srv/describe_parameters.hpp>
#include <rcl_interfaces/srv/get_parameter_types.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rcl_interfaces/srv/list_parameters.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <rcl_interfaces/srv/set_parameters_atomically.hpp>

#include "pick_place_panel.hpp"

namespace
{
void say(const std::string & key, const std::string & v)
{
  std::printf("GSVAL %s %s\n", key.c_str(), v.c_str());
  std::fflush(stdout);
}

QPushButton * buttonStarting(QWidget * panel, const QString & prefix)
{
  for (auto * b : panel->findChildren<QPushButton *>()) {
    if (b->text().startsWith(prefix)) {return b;}
  }
  return nullptr;
}

QDoubleSpinBox * boxRightOf(QWidget * b)
{
  if (!b || !b->parentWidget()) {return nullptr;}
  auto * g = qobject_cast<QGridLayout *>(b->parentWidget()->layout());
  if (!g) {return nullptr;}
  const int i = g->indexOf(b);
  if (i < 0) {return nullptr;}
  int r, c, rs, cs;
  g->getItemPosition(i, &r, &c, &rs, &cs);
  auto * it = g->itemAtPosition(r, c + 1);
  return it ? qobject_cast<QDoubleSpinBox *>(it->widget()) : nullptr;
}
}  // namespace

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  QApplication app(argc, argv);
  const std::string ns = "/uav_gsretry";
  using namespace rcl_interfaces::srv;

  // ---- the fake planner: hand-made parameter services ----
  auto planner = std::make_shared<rclcpp::Node>(
    "whole_body_trajectory_planner", ns, rclcpp::NodeOptions().start_parameter_services(false));
  const std::string fqn = ns + "/whole_body_trajectory_planner";
  std::map<std::string, rclcpp::ParameterValue> vals{
    {"pick_place_place_drop", rclcpp::ParameterValue(true)},
    {"pick_place_approach_dz", rclcpp::ParameterValue(0.20)},
    {"pick_place_pick_approach_back", rclcpp::ParameterValue(0.10)},
    {"pick_place_place_exit_back", rclcpp::ParameterValue(0.10)},
    {"pick_place_ee_offset", rclcpp::ParameterValue(std::vector<double>{-0.04, 0.0, 0.16})},
    {"pick_place_drop_height", rclcpp::ParameterValue(0.12)},
    {"pick_place_drop_altitude", rclcpp::ParameterValue(0.0)},
    {"pick_place_drop_exit_back", rclcpp::ParameterValue(0.15)},
    {"pick_place_pick_point", rclcpp::ParameterValue(std::vector<double>{1.0, 1.0, 0.81, 0.0})},
    {"pick_place_start", rclcpp::ParameterValue(std::vector<double>{0.1, -0.07, 1.0, 0.0})},
    {"pick_place_place_start", rclcpp::ParameterValue(std::vector<double>{0.8, -1.0, 1.0, -180.0})},
    {"pick_place_place_point", rclcpp::ParameterValue(std::vector<double>{-1.0, -1.0, 0.8455, -180.0})},
    {"pick_place_land_start", rclcpp::ParameterValue(std::vector<double>{-0.5, 0.8, 1.0, -360.0})},
    {"pick_place_land", rclcpp::ParameterValue(std::vector<double>{-0.5, 0.8, 0.35, -360.0})}};
  int main_gets = 0, main_dropped = 0, back_gets = 0, back_dropped = 0;
  rclcpp::Service<GetParameters>::SharedPtr get_srv;
  get_srv = planner->create_service<GetParameters>(
    fqn + "/get_parameters",
    [&](std::shared_ptr<rmw_request_id_t> hdr, std::shared_ptr<GetParameters::Request> req) {
      const auto & n = req->names;
      const bool is_main = std::find(n.begin(), n.end(), "pick_place_approach_dz") != n.end();
      const bool is_back = n.size() == 1 && n[0] == "pick_place_drop_exit_back";
      if (is_main) {++main_gets;}
      if (is_back) {++back_gets;}
      if (is_main && main_dropped == 0) {++main_dropped; return;}   // LOST: no response
      if (is_back && back_dropped == 0) {++back_dropped; return;}   // LOST: no response
      GetParameters::Response r;
      for (const auto & name : n) {
        const auto it = vals.find(name);
        if (it == vals.end()) {r.values.clear(); break;}   // like rclcpp: undeclared = none
        r.values.push_back(it->second.to_value_msg());
      }
      get_srv->send_response(*hdr, r);
    });
  auto types_srv = planner->create_service<GetParameterTypes>(
    fqn + "/get_parameter_types",
    [](const std::shared_ptr<GetParameterTypes::Request>, std::shared_ptr<GetParameterTypes::Response>) {});
  auto set_srv = planner->create_service<SetParameters>(
    fqn + "/set_parameters",
    [&](const std::shared_ptr<SetParameters::Request> q, std::shared_ptr<SetParameters::Response> r) {
      for (const auto & p : q->parameters) {
        vals[p.name] = rclcpp::ParameterValue(p.value);
        rcl_interfaces::msg::SetParametersResult ok;
        ok.successful = true;
        r->results.push_back(ok);
      }
    });
  auto atomic_srv = planner->create_service<SetParametersAtomically>(
    fqn + "/set_parameters_atomically",
    [](const std::shared_ptr<SetParametersAtomically::Request>,
    std::shared_ptr<SetParametersAtomically::Response> r) {r->result.successful = true;});
  auto list_srv = planner->create_service<ListParameters>(
    fqn + "/list_parameters",
    [](const std::shared_ptr<ListParameters::Request>, std::shared_ptr<ListParameters::Response>) {});
  auto describe_srv = planner->create_service<DescribeParameters>(
    fqn + "/describe_parameters",
    [](const std::shared_ptr<DescribeParameters::Request>, std::shared_ptr<DescribeParameters::Response>) {});

  // ---- the station's node and the REAL Pick & Drop panel ----
  auto node = std::make_shared<rclcpp::Node>("arm_gs_retry", ns + "/fsc_open_manipulator");
  auto * panel = new PickPlacePanel(node, true);
  panel->resize(1600, 1100);
  panel->show();

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(planner);
  exec.add_node(node);
  QTimer spin;
  QObject::connect(&spin, &QTimer::timeout, [&]() {exec.spin_some(std::chrono::milliseconds(0));});
  spin.start(10);

  // the loaded state: every box shows the planner's value, and is enabled
  const auto loaded = [&]() {
      struct B {const char * button; double want;};
      for (const B & b : {B{"Vertical Margin", 0.20}, B{"Side Margin", 0.10}, B{"Drop Margin", 0.12},
          B{"Back Margin", 0.15}})
      {
        auto * box = boxRightOf(buttonStarting(panel, b.button));
        if (!box || !box->isEnabled() || std::abs(box->value() - b.want) > 1e-9) {return false;}
      }
      return true;
    };
  int step = 0;
  double loaded_at = -1.0;
  QTimer script;
  QObject::connect(&script, &QTimer::timeout, [&]() {
      ++step;
      if (loaded_at < 0.0 && loaded()) {loaded_at = 0.1 * step;}
      if (step == 120) {   // 12 s
        say("main_loads_asked", std::to_string(main_gets) + " (first dropped: " +
          std::to_string(main_dropped) + ")");
        say("back_margin_loads_asked", std::to_string(back_gets) + " (first dropped: " +
          std::to_string(back_dropped) + ")");
        for (const char * b : {"Vertical Margin", "Side Margin", "Drop Margin", "Back Margin"}) {
          auto * box = boxRightOf(buttonStarting(panel, b));
          say(std::string(b), box ? QString("%1 %2").arg(box->value(), 0, 'f', 2)
            .arg(box->isEnabled() ? "enabled" : "GREY").toStdString() : "no box");
        }
        say("loaded_after_s", loaded_at < 0.0 ? "never" : QString::number(loaded_at, 'f', 1).toStdString());
        const bool ok = loaded_at > 0.0 && main_dropped == 1 && main_gets >= 2 &&
          back_dropped == 1 && back_gets >= 2;
        say("RESULT", ok ? "PASS" : "FAIL");
        app.quit();
      }
    });
  script.start(100);
  const int rc = app.exec();
  rclcpp::shutdown();
  return rc;
}
