// gs_mocap_harness.cpp -- the REAL arm-GS "Box Pushing" panel, offscreen (2026-10-21).
//
// Checks the MOCAP block (the raw-mocap safety guard's display, right of the Monitor's
// push rows): it plays the Orin's estimator on <vehicle ns>/state_estimator/mocap_guard
// (Float64MultiArray, BEST_EFFORT, 50 Hz -- the layout in push_pull_panel.cpp) through
// the states below, reads the panel's own lamp and lines, checks that the block sits
// right of the now / plan boxes without overlapping the push note (moved to the header
// row) or the Table Offset note, and that no line is clipped. No planner, no Isaac: the
// push rows are fed a fake push monitor (push_pull/arrival_error [21..29]) so the
// pictures show them populated.
//
//   1_guard_off   nothing ever received
//   2_mocap       healthy
//   3_bridging    stale, last good 0.08 s ago
//   4_confirmed   frozen, 0.42 s
//   5_lost_a/_b   frozen, 2.3 s, the bridge (offset 12 mm) -- two frames 0.4 s apart: the flash
//   6_recovered   MOCAP again, last fault "frozen 2.84 s, 5 s ago"
//   7_no_link     the stream stopped 1.6 s ago
//
// Prints "GSVAL <key> <value>"; the last is "GSVAL result PASS|FAIL ...".
//   QT_QPA_PLATFORM=offscreen ./gs_mocap_harness <station ns> <png prefix>
#include <QApplication>
#include <QFrame>
#include <QLabel>
#include <QLineEdit>
#include <QRegularExpression>
#include <QTimer>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <limits>
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

constexpr double NaN = std::numeric_limits<double>::quiet_NaN();

// [0] state, [1] cause, [2] age, [3] source, [4] EKF2 fresh, [5] |mocap-EKF2|, [6] |d0|,
// [7] rate, [8] rejected, [9] episodes, [10] episode t, [11] recovering, [12] test,
// [13] last cause, [14] last duration, [15] last since
const std::vector<double> kMocap{0, 0, 0.008, 0, 1, 0.008, NaN, 120, 0, 0, 0, 0, 0, NaN, NaN, NaN};
const std::vector<double> kBridging{1, 1, 0.080, 1, 1, 0.009, 0.009, 108, 0, 1, 0.030, 0, 0, NaN, NaN, NaN};
const std::vector<double> kConfirmed{2, 2, 0.420, 1, 1, 0.011, 0.011, 120, 40, 1, 0.370, 0, 0, NaN, NaN, NaN};
const std::vector<double> kLost{3, 2, 2.300, 1, 1, 0.012, 0.012, 120, 276, 1, 2.250, 0, 0, NaN, NaN, NaN};
const std::vector<double> kRecovered{0, 0, 0.008, 0, 1, 0.010, NaN, 120, 341, 1, 0, 0, 0, 2, 2.84, 5.0};
}  // namespace

int main(int argc, char * argv[])
{
  if (argc < 3) {
    std::fprintf(stderr, "usage: gs_mocap_harness <station namespace> <png prefix>\n");
    return 2;
  }
  const std::string ns = argv[1];
  const QString png = QString::fromLocal8Bit(argv[2]);
  rclcpp::init(1, argv);
  QApplication app(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("gs_mocap_harness", ns);
  const std::string gov = ns.substr(0, ns.find_last_of('/'));

  auto * panel = new PushPullPanel(node);
  panel->resize(1300, 900);
  panel->show();

  QTimer pump;
  QObject::connect(&pump, &QTimer::timeout, [&node]() {rclcpp::spin_some(node);});
  pump.start(5);

  // the Orin's estimator: the guard message at 50 Hz, BEST_EFFORT, whatever `guard` holds
  auto guard_pub = node->create_publisher<std_msgs::msg::Float64MultiArray>(
    gov + "/state_estimator/mocap_guard", rclcpp::SensorDataQoS());
  std::vector<double> guard;   // empty = publish nothing
  QTimer guard_timer;
  QObject::connect(&guard_timer, &QTimer::timeout, [&]() {
      if (guard.empty()) {return;}
      std_msgs::msg::Float64MultiArray m;
      m.data = guard;
      guard_pub->publish(m);
    });
  guard_timer.start(20);

  // the push rows, populated (a push in flight), so the pictures show the whole Monitor
  auto arrival_pub = node->create_publisher<std_msgs::msg::Float64MultiArray>(
    gov + "/whole_body_planner/push_pull/arrival_error", 10);
  QTimer arrival;
  QObject::connect(&arrival, &QTimer::timeout, [&]() {
      std_msgs::msg::Float64MultiArray m;
      m.data.assign(21, 0.0);
      m.data[0] = 2.0;   // the push step
      m.data[1] = 1.0;
      m.data.insert(m.data.end(), {1.0, 0.0, 5.0, 6.3, 1.5, 0.051, 0.055, 0.0176, 0.0201});
      arrival_pub->publish(m);
    });
  arrival.start(100);

  // the panel's own widgets
  QFrame * block = panel->findChild<QFrame *>("mocapGuardBlock");
  QLabel * lamp = nullptr;
  std::vector<QLabel *> lines;
  if (block) {
    for (auto * l : block->findChildren<QLabel *>()) {
      if (l->text() == "Mocap") {continue;}
      if (l->toolTip().startsWith("The guard's state")) {lamp = l;} else {lines.push_back(l);}
    }
  }
  // top to bottom
  std::sort(lines.begin(), lines.end(), [](QLabel * a, QLabel * b) {return a->y() < b->y();});
  bool ok = block && lamp && lines.size() == 4;
  std::string why = ok ? "" : "the Mocap block / lamp / 4 lines not found";
  const auto fail = [&](const std::string & w) {if (ok) {ok = false; why = w;}};
  const auto plain = [](const QString & rich) {
      QString t = rich;
      t.remove(QRegularExpression("<[^>]*>"));
      return t.toStdString();
    };

  // the layout: right of the plan boxes, inside the push rows, clear of both notes
  const auto layoutCheck = [&](const std::string & tag) {
      const QRect b(block->mapTo(panel, QPoint(0, 0)), block->size());
      int plan_right = 0, push_top = 1 << 30, table_top = -1;
      QRect push_note, gear_note;
      for (auto * e : panel->findChildren<QLineEdit *>()) {
        const QRect r(e->mapTo(panel, QPoint(0, 0)), e->size());
        if (e->toolTip().startsWith("PLAN:")) {plan_right = std::max(plan_right, r.right());}
      }
      for (auto * l : panel->findChildren<QLabel *>()) {
        const QRect r(l->mapTo(panel, QPoint(0, 0)), l->size());
        if (l->text() == "Push Time [s]") {push_top = r.top();}
        if (l->text() == "Table Offset [m]") {table_top = r.top();}
        if (l->text() == "pushing") {push_note = r;}
        if (l->text().contains("waiting for the planner's table") || l->text().contains("planner keeps")) {
          gear_note = r;
        }
      }
      say(tag + "_block", std::to_string(b.left()) + "," + std::to_string(b.top()) + " " +
        std::to_string(b.width()) + "x" + std::to_string(b.height()) + " (plan boxes end " +
        std::to_string(plan_right) + ", push rows " + std::to_string(push_top) + ".." +
        std::to_string(table_top) + ")");
      if (b.left() <= plan_right) {fail(tag + ": the block overlaps the plan boxes");}
      if (b.top() < push_top - 4 || b.bottom() >= table_top) {fail(tag + ": the block leaves the push rows");}
      if (push_note.isNull()) {
        fail(tag + ": the push note (\"pushing\") is not visible");
      } else if (push_note.intersects(b)) {
        fail(tag + ": the push note overlaps the block");
      }
      if (gear_note.isNull()) {
        fail(tag + ": the Table Offset note is not visible");
      } else if (gear_note.intersects(b)) {
        fail(tag + ": the Table Offset note overlaps the block");
      }
    };
  // every line whole: its text's own width fits the label
  const auto check = [&](const std::string & tag, const std::string & want_lamp, const std::string & want_in) {
      std::string all;
      for (size_t i = 0; i < lines.size(); ++i) {
        const std::string t = plain(lines[i]->text());
        say(tag + "_line" + std::to_string(i + 1), t + "  [" + std::to_string(lines[i]->sizeHint().width()) +
          " of " + std::to_string(lines[i]->width()) + " px]");
        all += t + "\n";
        if (lines[i]->sizeHint().width() > lines[i]->width()) {fail(tag + ": line clipped: " + t);}
      }
      say(tag + "_lamp", lamp->text().toStdString() + "  {" + lamp->styleSheet().toStdString() + "}");
      if (lamp->text().toStdString() != want_lamp) {fail(tag + ": lamp '" + lamp->text().toStdString() + "'");}
      if (!want_in.empty() && all.find(want_in) == std::string::npos) {fail(tag + ": no '" + want_in + "'");}
      layoutCheck(tag);
      panel->grab().save(png + "_" + QString::fromStdString(tag) + ".png");
    };

  QString lost_style;
  std::vector<std::pair<int, std::function<void()>>> steps;
  steps.push_back({1500, [&]() {
      check("1_guard_off", "guard off", "no guard message since the station started");
      guard = kMocap;
    }});
  steps.push_back({1000, [&]() {
      check("2_mocap", "MOCAP", "healthy, last good 0.01 s ago · 120 Hz");
      guard = kBridging;
    }});
  steps.push_back({600, [&]() {
      check("3_bridging", "EKF2 BRIDGE", "stale, last good 0.08 s ago");
      guard = kConfirmed;
    }});
  steps.push_back({600, [&]() {
      check("4_confirmed", "EKF2 BRIDGE -- CONFIRMED", "frozen, last good 0.42 s ago");
      guard = kLost;
    }});
  steps.push_back({600, [&]() {
      check("5_lost_a", "LOST -- LAND WITH THE RC", "odometry: EKF2 + offset 12 mm");
      lost_style = lamp->styleSheet();
    }});
  steps.push_back({400, [&]() {
      check("5_lost_b", "LOST -- LAND WITH THE RC", "frozen, last good 2.30 s ago");
      say("flash", lamp->styleSheet() != lost_style ? "toggles" : "STEADY");
      if (lamp->styleSheet() == lost_style) {fail("LOST: the lamp does not flash");}
      guard = kRecovered;
    }});
  steps.push_back({600, [&]() {
      check("6_recovered", "MOCAP", "last fault: frozen 2.84 s, 5 s ago");
      guard.clear();
    }});
  steps.push_back({1600, [&]() {
      check("7_no_link", "NO LINK", "the last one said MOCAP -- not current");
      // the live station's left pane: the points table squeezed (the push harness's 840 px)
      panel->resize(840, 900);
    }});
  steps.push_back({600, [&]() {
      check("8_narrow", "NO LINK", "the WiFi or the Orin's estimator is down");
    }});
  int i = 0;
  std::function<void()> next = [&]() {
      if (!ok || i >= static_cast<int>(steps.size())) {
        say("result", ok ? "PASS" : "FAIL " + why);
        app.exit(ok ? 0 : 1);
        return;
      }
      const auto s = steps[i++];
      // each step waits its delay AFTER the previous one ran (a stalled loop cannot fire two at once)
      QTimer::singleShot(s.first, [&, s]() {s.second(); next();});
    };
  next();
  const int rc = app.exec();
  rclcpp::shutdown();
  return rc;
}
