// gs_push_harness.cpp -- the REAL arm-GS "Box Pushing" panel, offscreen (2026-10-21).
//
// Checks the Monitor's Push rows (user request): the push's time, contact time and
// mean speed from the planner's ONE mean-speed parameter (push_pull_push_mean_speed)
// and the Push Distance -- previewed in amber while a typed distance is not sent --
// then the box's travel and speed live in a push and held as its summary. Run by
// ../gs_push_check.sh on a private ROS domain (never beside a live stack). The
// planner is the REAL node; this program plays the operator (types into the panel's
// own Push Distance box and presses its button) and, for the live frames, the
// planner's push monitor (push_pull/arrival_error [21..29], published here because
// the real planner only publishes it in DIRECT).
//
//   loaded     -> the plan column = the planner's numbers for its distance
//   type 0.10  -> the plan column previews 0.10 m at the mean speed, amber
//   send       -> push_pull_push_distance = 0.10, the planner's numbers, not amber
//   pushing    -> the now column live, the plan column the reference's
//   summary    -> the push's numbers held
//
// Prints "GSVAL <key> <value>" lines; the last is "GSVAL result PASS|FAIL ...".
//
//   QT_QPA_PLATFORM=offscreen ./gs_push_harness <station ns> <png prefix>
#include <QApplication>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QTimer>

#include <array>
#include <cmath>
#include <cstdio>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "push_pull_panel.hpp"

namespace
{
void say(const std::string & key, const std::string & v)
{
  std::printf("GSVAL %s %s\n", key.c_str(), v.c_str());
  std::fflush(stdout);
}

const char * const kNowTip[5] = {
  "NOW: the push's moving time", "NOW: time since the push step", "NOW: the box's travel so far",
  "NOW: the box (mocap obj_0)", "NOW: the box's speed"};
const char * const kPlanTip[5] = {
  "PLAN: Push Distance / mean speed", "PLAN: the settle + the push time", "PLAN: the planner's push mean",
  "PLAN: in the push, the claw", "PLAN: the claw reference's speed"};
const char * const kRow[5] = {"push_time", "contact_time", "mean_speed", "box_travel", "box_speed"};
}  // namespace

int main(int argc, char * argv[])
{
  if (argc < 3) {
    std::fprintf(stderr, "usage: gs_push_harness <station namespace> <png prefix>\n");
    return 2;
  }
  const std::string ns = argv[1];
  const QString png = QString::fromLocal8Bit(argv[2]);
  rclcpp::init(1, argv);
  QApplication app(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("gs_push_harness", ns);
  const std::string gov = ns.substr(0, ns.find_last_of('/'));

  auto * panel = new PushPullPanel(node);
  panel->resize(1300, 900);
  panel->show();

  QTimer pump;
  QObject::connect(&pump, &QTimer::timeout, [&node]() {rclcpp::spin_some(node);});
  pump.start(5);

  auto params = std::make_shared<rclcpp::AsyncParametersClient>(node, gov + "/whole_body_trajectory_planner");
  double v = NAN, floor = NAN, settle = NAN, dist = NAN;
  const auto read = [&](std::function<void()> then) {
      params->get_parameters(
        {"push_pull_push_mean_speed", "push_pull_push_min_time_s", "push_pull_push_settle_s",
          "push_pull_push_distance"},
        [&, then](std::shared_future<std::vector<rclcpp::Parameter>> f) {
          const auto r = f.get();
          if (r.size() == 4) {
            v = r[0].as_double();
            floor = r[1].as_double();
            settle = r[2].as_double();
            dist = r[3].as_double();
          }
          then();
        });
    };

  // the planner's push monitor, played here for the live frames
  auto arrival_pub = node->create_publisher<std_msgs::msg::Float64MultiArray>(
    gov + "/whole_body_planner/push_pull/arrival_error", 10);
  std::vector<double> mon;   // [21..29]; empty = not publishing
  QTimer arrival;
  QObject::connect(&arrival, &QTimer::timeout, [&]() {
      if (mon.empty()) {return;}
      std_msgs::msg::Float64MultiArray m;
      m.data.assign(21, 0.0);
      m.data[0] = 2.0;     // the push step
      m.data[1] = 1.0;
      m.data.insert(m.data.end(), mon.begin(), mon.end());
      arrival_pub->publish(m);
    });
  arrival.start(100);

  // the panel's own widgets
  std::array<QLineEdit *, 5> now{}, plan{};
  for (auto * e : panel->findChildren<QLineEdit *>()) {
    for (int r = 0; r < 5; ++r) {
      if (e->toolTip().startsWith(kNowTip[r])) {now[r] = e;}
      if (e->toolTip().startsWith(kPlanTip[r])) {plan[r] = e;}
    }
  }
  QPushButton * send = nullptr;
  for (auto * b : panel->findChildren<QPushButton *>()) {
    if (b->text() == "Push Distance") {send = b;}
  }
  bool ok = send != nullptr;
  for (int r = 0; r < 5; ++r) {ok = ok && now[r] && plan[r];}
  std::string why = ok ? "" : "widgets not found";
  const auto fail = [&](const std::string & w) {if (ok) {ok = false; why = w;}};
  QDoubleSpinBox * dist_box = nullptr;
  const auto rows = [&](const std::string & tag) {
      for (int r = 0; r < 5; ++r) {
        say(tag + "_" + kRow[r], now[r]->text().toStdString() + " / " + plan[r]->text().toStdString() +
          (plan[r]->styleSheet().contains("#e58a00") ? " (amber)" : ""));
      }
    };
  const auto num = [](QLineEdit * e) {bool k = false; const double x = e->text().toDouble(&k); return k ? x : NAN;};
  const auto pushTime = [&](double d) {return std::max(std::abs(d) / v, floor);};
  // THE HEALTH COLOURS (2026-10-21): green #2e7d32 inside the tested range, red #c62828
  const auto colourOf = [](QLineEdit * e) {
      const QString ss = e->styleSheet();
      return ss.contains("#2e7d32") ? std::string("green") : ss.contains("#c62828") ? std::string("red")
                                                                            : std::string("plain");
    };
  const auto noteText = [&]() {
      for (auto * l : panel->findChildren<QLabel *>()) {
        const QString t = l->text();
        if (t.contains("retype") || t.startsWith("preview") || t == "pushing" || t.startsWith("the last push")) {
          return t.toStdString();
        }
      }
      return std::string("");
    };
  const auto colours = [&](const std::string & tag) {
      std::string c;
      for (int r = 0; r < 5; ++r) {c += (r ? " " : "") + colourOf(plan[r]);}
      say(tag + "_colours", c);
      say(tag + "_note", noteText());
      return c;
    };

  std::vector<std::pair<int, std::function<void()>>> steps;
  steps.push_back({5000, [&]() {   // loaded
      if (!ok) {return;}
      read([&]() {
          say("planner", "speed " + std::to_string(v) + " floor " + std::to_string(floor) + " settle " +
            std::to_string(settle) + " distance " + std::to_string(dist));
          for (auto * s : panel->findChildren<QDoubleSpinBox *>()) {
            if (std::abs(s->value() - dist) < 1e-6 && s->decimals() == 3) {dist_box = s;}
          }
          if (!dist_box) {fail("Push Distance box not found");}
          rows("loaded");
          if (!(std::abs(num(plan[0]) - pushTime(dist)) < 0.051)) {fail("loaded push time");}
          if (!(std::abs(num(plan[1]) - (settle + pushTime(dist))) < 0.051)) {fail("loaded contact time");}
          if (!(std::abs(num(plan[2]) - dist / pushTime(dist) * 1e3) < 0.051)) {fail("loaded mean speed");}
          panel->grab().save(png + "_1_loaded.png");
        });
    }});
  // the planner's 0.50 m: inside the tested range -> green (box speed uncoloured)
  steps.push_back({300, [&]() {
      if (!ok) {return;}
      if (colours("loaded") != "green green green green plain") {fail("loaded not green");}
    }});
  // three distances outside the tested range: red, and the note says retype
  for (const auto & bad : std::vector<std::pair<double, std::string>>{
      {0.05, "1b_short"}, {-0.10, "1c_pull"}, {0.60, "1d_long"}})
  {
    steps.push_back({600, [&, bad]() {
        if (ok) {dist_box->setValue(bad.first); say("set " + bad.second, std::to_string(dist_box->value()));}
      }});
    steps.push_back({800, [&, bad]() {
        if (!ok) {return;}
        rows(bad.second);
        const std::string c = colours(bad.second);
        // 0.05 m: the floor makes the mean 16.7 mm/s, inside 8-30 -> that row stays green
        const std::string want = bad.first == 0.05 ? "red red green red plain" : "red red green red plain";
        if (c.rfind("red red", 0) != 0 || colourOf(plan[3]) != "red") {fail("not red at " + bad.second);}
        (void)want;
        if (noteText().find("retype") == std::string::npos) {fail("no retype note at " + bad.second);}
        panel->grab().save(png + "_" + QString::fromStdString(bad.second) + ".png");
      }});
  }
  steps.push_back({600, [&]() {   // type 0.10, not sent
      if (!ok) {return;}
      dist_box->setValue(0.10);
    }});
  steps.push_back({800, [&]() {
      if (!ok) {return;}
      say("preview_box_value", std::to_string(dist_box->value()) + (dist_box->hasFocus() ? " focus" : "") +
        (dist_box->isEnabled() ? " enabled" : " disabled"));
      rows("preview");
      if (colours("preview") != "green green green green plain") {fail("preview not green");}
      if (noteText().rfind("preview", 0) != 0) {fail("preview note missing");}
      if (!(std::abs(num(plan[0]) - pushTime(0.10)) < 0.051)) {fail("preview push time");}
      panel->grab().save(png + "_2_preview.png");
      send->click();
    }});
  steps.push_back({2000, [&]() {   // sent
      if (!ok) {return;}
      read([&]() {
          say("planner_distance_after_send", std::to_string(dist));
          if (std::abs(dist - 0.10) > 1e-9) {fail("distance not sent");}
          rows("sent");
          if (colours("sent") != "green green green green plain") {fail("sent not green");}
          if (!noteText().empty()) {fail("a note after a healthy send");}
          if (!(std::abs(num(plan[0]) - pushTime(0.10)) < 0.051)) {fail("sent push time");}
          panel->grab().save(png + "_3_sent.png");
          // a push 3.5 s into its moving part: the box 4 mm behind its reference
          const double T = pushTime(0.10);
          mon = {1.0, 0.0, settle + 3.5, settle + T, settle, 0.051, 0.055, 0.0176, 0.0201};
        });
    }});
  steps.push_back({1500, [&]() {   // pushing
      if (!ok) {return;}
      rows("pushing");
      if (colours("pushing") != "plain plain plain plain plain") {fail("live rows coloured");}
      if (!(std::abs(num(now[0]) - 3.5) < 0.051)) {fail("live push time");}
      if (!(std::abs(num(now[3]) - 51.0) < 0.51)) {fail("live box travel");}
      if (!(std::abs(num(plan[3]) - 55.0) < 0.51)) {fail("live planned travel");}
      if (!(std::abs(num(now[4]) - 17.6) < 0.051)) {fail("live box speed");}
      panel->grab().save(png + "_4_pushing.png");
      const double T = pushTime(0.10);
      mon = {1.0, 1.0, settle + T, settle + T, settle, 0.094, 0.100, 0.0, 0.0};   // the summary
    }});
  steps.push_back({1500, [&]() {   // summary
      if (!ok) {return;}
      rows("summary");
      if (!(std::abs(num(now[3]) - 94.0) < 0.51) || !(std::abs(num(plan[3]) - 100.0) < 0.51)) {
        fail("summary travel");
      }
      if (now[4]->text() != "—") {fail("summary box speed not blank");}
      panel->grab().save(png + "_5_last_push.png");
      panel->resize(840, 900);   // the live station's left pane: the points table squeezed
    }});
  steps.push_back({800, [&]() {
      if (!ok) {return;}
      panel->grab().save(png + "_6_narrow.png");
      {
        const char * order[7] = {"Start", "Push Time [s]", "Contact Time [s]", "Mean Speed [mm/s]",
          "Box Speed [mm/s]", "Box Travelled [mm]", "Table Offset [m]"};
        int last_y = -1;
        std::string seq;
        for (const char * want : order) {
          int y = -1;
          for (auto * l : panel->findChildren<QLabel *>()) {
            if (l->text() == want) {y = l->mapTo(panel, QPoint(0, 0)).y();}
          }
          seq += std::string(want) + "@" + std::to_string(y) + " ";
          if (y <= last_y) {fail(std::string("order: ") + want);}
          last_y = y;
        }
        say("order", seq);
      }
      // every Monitor label whole: its own grid sizes it to its text
      for (auto * l : panel->findChildren<QLabel *>()) {
        const QString t = l->text();
        if (t.endsWith("[s]") || t.endsWith("[mm/s]") || t.endsWith("[mm]") || t.startsWith("Table Offset")) {
          const int need = l->fontMetrics().horizontalAdvance(t);
          say("label_width " + t.toStdString(), std::to_string(l->width()) + " of " + std::to_string(need));
          if (l->width() < need) {fail("label clipped: " + t.toStdString());}
        }
      }
    }});
  steps.push_back({500, [&]() {
      say("result", ok ? "PASS" : ("FAIL " + why));
      QApplication::quit();
    }});

  // each step waits its delay AFTER the previous one ran (2026-10-21): fixed times from the
  // start let a stalled event loop fire a "type" and its "check" back to back, before the
  // panel's 200 ms redraw -- a stale frame that read as a GS bug
  std::function<void(size_t)> run = [&](size_t i) {
      if (i >= steps.size()) {return;}
      QTimer::singleShot(steps[i].first, [&, i]() {steps[i].second(); run(i + 1);});
    };
  run(0);
  const int rc = app.exec();
  rclcpp::shutdown();
  return ok ? rc : 1;
}
