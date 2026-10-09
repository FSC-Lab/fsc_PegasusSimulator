// gs_pnp_harness.cpp -- the REAL arm-GS "Pick & Place" panel, offscreen (2026-10-08).
//
// Two checks of the 2026-10-08 panel changes, run by ../gs_pnp_check.sh on a private
// ROS domain (never beside a live stack):
//
//   side   <station ns> <png>  -- with the REAL planner up: the Side Margin box shows
//          the planner's pick_place_pick_approach_back and is enabled (hook grasp);
//          typing 0.12 and pressing Side Margin sets BOTH pick_place_pick_approach_back
//          and pick_place_place_exit_back; then 0.10 is written back the same way.
//   close  <station ns> <png>  -- NO planner: this program publishes the planner's
//          pick_place/info itself and counts the panel's gripper requests. The panel
//          must CLOSE the gripper once per flown Exit To Place, never while the exit
//          is in flight.
//
// Prints "GSVAL <key> <value>" lines; the checker scores them.
//
//   QT_QPA_PLATFORM=offscreen ./gs_pnp_harness side|close <station ns> <png>
#include <QApplication>
#include <QDoubleSpinBox>
#include <QPushButton>
#include <QTimer>

#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "pick_place_panel.hpp"

namespace
{
void say(const char * key, const std::string & v)
{
  std::printf("GSVAL %s %s\n", key, v.c_str());
  std::fflush(stdout);
}
}  // namespace

int main(int argc, char * argv[])
{
  if (argc < 4) {
    std::fprintf(stderr, "usage: gs_pnp_harness side|close <station namespace> <png>\n");
    return 2;
  }
  const std::string mode = argv[1];
  const std::string ns = argv[2];
  const QString png = QString::fromLocal8Bit(argv[3]);
  rclcpp::init(1, argv);
  QApplication app(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("gs_pnp_harness", ns);
  const std::string gov = ns.substr(0, ns.find_last_of('/'));

  auto * panel = new PickPlacePanel(node);
  panel->resize(1500, 1100);
  panel->show();

  int opens = 0, closes = 0;
  QObject::connect(panel, &PickPlacePanel::gripperRequested, [&opens, &closes](bool open) {
      (open ? opens : closes)++;
    });

  QTimer pump;
  QObject::connect(&pump, &QTimer::timeout, [&node]() {rclcpp::spin_some(node);});
  pump.start(5);

  const auto button = [panel](const QString & text) -> QPushButton * {
      for (auto * b : panel->findChildren<QPushButton *>()) {
        if (b->text() == text) {return b;}
      }
      return nullptr;
    };
  const auto spin = [panel](const QString & tip) -> QDoubleSpinBox * {
      for (auto * s : panel->findChildren<QDoubleSpinBox *>()) {
        if (s->toolTip().startsWith(tip)) {return s;}
      }
      return nullptr;
    };

  int state = 0, ticks = 0, rc = 1;
  QTimer step;
  // everything the timer lambdas use lives HERE, not in the mode blocks below
  rclcpp::AsyncParametersClient::SharedPtr params;
  double pick_back = NAN, place_back = NAN;
  bool reading = false;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub;

  if (mode == "side") {
    params = std::make_shared<rclcpp::AsyncParametersClient>(
      node, gov + "/whole_body_trajectory_planner");
    const auto read = [&]() {
        if (reading || !params->service_is_ready()) {return;}
        reading = true;
        params->get_parameters(
          {"pick_place_pick_approach_back", "pick_place_place_exit_back"},
          [&](std::shared_future<std::vector<rclcpp::Parameter>> f) {
            const auto v = f.get();
            if (v.size() == 2) {pick_back = v[0].as_double(); place_back = v[1].as_double();}
            reading = false;
          });
      };
    QObject::connect(&step, &QTimer::timeout, [&, read]() {
        ++ticks;
        QPushButton * b = button("Side Margin");
        QDoubleSpinBox * s = spin("Side margin");
        if (!b || !s) {say("error", "no Side Margin widgets"); app.exit(1); return;}
        read();
        const auto want = [&](double v) {
            return std::abs(pick_back - v) < 1e-6 && std::abs(place_back - v) < 1e-6;
          };
        if (state == 0 && b->isEnabled() && s->isEnabled() && std::isfinite(pick_back)) {
          say("side_shown", std::to_string(s->value()));
          say("side_planner_start", std::to_string(pick_back) + "," + std::to_string(place_back));
          panel->grab().save(png + "_side.png");
          s->setValue(0.12);
          b->click();
          state = 1;
        } else if (state == 1 && want(0.12)) {
          say("after_send_012", std::to_string(pick_back) + "," + std::to_string(place_back));
          say("box_amber_after_send", s->styleSheet().isEmpty() ? "no" : "yes");
          s->setValue(0.10);
          b->click();
          state = 2;
        } else if (state == 2 && want(0.10)) {
          say("after_send_010", std::to_string(pick_back) + "," + std::to_string(place_back));
          say("result", "PASS");
          rc = 0;
          app.exit(0);
        }
        if (ticks > 600) {say("result", "TIMEOUT state " + std::to_string(state)); app.exit(1);}
      });
  } else {
    // the planner's pick_place/info, published here (latched, like the planner's)
    const rclcpp::QoS latched = rclcpp::QoS(1).reliable().transient_local();
    pub = node->create_publisher<std_msgs::msg::Float64MultiArray>(
      gov + "/whole_body_planner/pick_place/info", latched);
    // layout (pick_place_panel.cpp): [5] done, [6] flying, [83 / 84] exit flown pick / place,
    // [85] at the table, [86] exit in flight, [87] phase
    const auto info = [pub](double exited_place, double exiting) {
        std_msgs::msg::Float64MultiArray m;
        m.data.assign(88, 0.0);
        m.data[5] = 3.0;          // the place leg is done
        m.data[6] = -1.0;         // nothing flying
        m.data[84] = exited_place;
        m.data[85] = 0.0;
        m.data[86] = exiting;
        pub->publish(m);
      };
    // each stage: what to publish, and the close count expected after it
    struct Stage {double exited, exiting; int closes; const char * what;};
    static const std::vector<Stage> stages = {
      {0.0, -1.0, 0, "before the exit"},
      {0.0, 3.0, 0, "exit in flight"},
      {1.0, 3.0, 0, "exit flagged but still in flight"},
      {1.0, -1.0, 1, "exit done -> ONE close"},
      {1.0, -1.0, 1, "republished, still one"},
      {0.0, -1.0, 1, "place re-flown (flag cleared)"},
      {1.0, -1.0, 2, "second exit done -> second close"},
    };
    QObject::connect(&step, &QTimer::timeout, [&, info]() {
        ++ticks;
        const int k = ticks / 20;                     // 20 x 50 ms per stage
        if (ticks % 20 == 1 && k < static_cast<int>(stages.size())) {
          info(stages[k].exited, stages[k].exiting);
        }
        if (ticks % 20 == 0 && ticks > 0) {
          const int j = ticks / 20 - 1;
          if (j < static_cast<int>(stages.size())) {
            const bool ok = closes == stages[j].closes && opens == 0;
            say((std::string("stage") + std::to_string(j)).c_str(),
              std::string(ok ? "ok" : "WRONG") + " closes=" + std::to_string(closes) +
              " opens=" + std::to_string(opens) + " (" + stages[j].what + ")");
            if (!ok) {state = -1;}
          } else {
            panel->grab().save(png + "_close.png");
            say("result", state < 0 ? "FAIL" : "PASS");
            rc = state < 0 ? 1 : 0;
            app.exit(rc);
          }
        }
      });
  }
  step.start(50);
  const int r = app.exec();
  rclcpp::shutdown();
  return r ? r : rc;
}
