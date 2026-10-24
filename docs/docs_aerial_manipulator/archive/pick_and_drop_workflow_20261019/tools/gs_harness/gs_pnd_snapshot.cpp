// gs_pnd_snapshot.cpp -- the arm GS "Pick & Drop" tab, offscreen, driven and
// photographed on cue (2026-10-19, the Pick-and-Drop workflow page). Run by
// ../snapshots.sh on a private ROS domain beside the REAL planner and the
// loopback rig (../pnd_rig.py), which sends commands on /pndshot/cmd:
//
//   click:<text>[#n]   press the n-th (top to bottom, default 0) enabled button
//                      whose text contains <text> (line breaks read as spaces)
//   snap:<name>        save the panel as <out>/<name>.png, and every visible
//                      widget's class, text and rectangle as <out>/<name>.json
//   quit
//
// and gets "ok <cmd>" / "err <cmd>: <why>" back on /pndshot/ack.
//
// The station's gripper is emulated: the panel's gripperRequested(open) is
// answered 0.7 s later (the real gripper's measured open time) with
// onGripperOpenFinished() / onGripperCloseFinished(false) -- a FULL close, the
// hook grasp's result -- so the tab's own automation (Exit To Pick after the
// grasp, the release on the planner's drop_now, Exit To Drop on the OPEN
// report) runs exactly as in the station.
//
//   QT_QPA_PLATFORM=offscreen ./gs_pnd_snapshot <station ns> <out dir>
#include <QAbstractButton>
#include <QAbstractSpinBox>
#include <QApplication>
#include <QDir>
#include <QFile>
#include <QGroupBox>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QPushButton>
#include <QTimer>

#include <algorithm>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "pick_place_panel.hpp"

namespace
{
QString flat(QString s)
{
  return s.replace('\n', ' ').simplified();
}

QJsonObject describe(QWidget * w, QWidget * root)
{
  QJsonObject o;
  o["class"] = w->metaObject()->className();
  QString text;
  if (auto * b = qobject_cast<QAbstractButton *>(w)) {text = b->text();}
  if (auto * l = qobject_cast<QLabel *>(w)) {text = l->text();}
  if (auto * g = qobject_cast<QGroupBox *>(w)) {text = g->title();}
  if (auto * s = qobject_cast<QAbstractSpinBox *>(w)) {text = s->text();}
  o["text"] = flat(text);
  o["enabled"] = w->isEnabled();
  const QPoint p = w->mapTo(root, QPoint(0, 0));
  o["x"] = p.x();
  o["y"] = p.y();
  o["w"] = w->width();
  o["h"] = w->height();
  return o;
}
}  // namespace

int main(int argc, char * argv[])
{
  if (argc < 3) {
    std::fprintf(stderr, "usage: gs_pnd_snapshot <station namespace> <out dir>\n");
    return 2;
  }
  const std::string ns = argv[1];
  const QString out = QString::fromLocal8Bit(argv[2]);
  QDir().mkpath(out);
  rclcpp::init(1, argv);
  QApplication app(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("gs_pnd_snapshot", ns);

  auto * panel = new PickPlacePanel(node, true);
  panel->resize(1300, 1500);
  panel->show();

  QObject::connect(panel, &PickPlacePanel::gripperRequested, [panel](bool open) {
      std::printf("GRIPPER %s requested\n", open ? "OPEN" : "CLOSE");
      std::fflush(stdout);
      QTimer::singleShot(700, panel, [panel, open]() {
          if (open) {
            panel->onGripperOpenFinished();
          } else {
            panel->onGripperCloseFinished(false);
          }
          std::printf("GRIPPER %s finished\n", open ? "OPEN" : "CLOSE");
          std::fflush(stdout);
        });
    });

  auto ack = node->create_publisher<std_msgs::msg::String>("/pndshot/ack", 10);
  const auto reply = [ack](const std::string & s) {
      std_msgs::msg::String m;
      m.data = s;
      ack->publish(m);
      std::printf("%s\n", s.c_str());
      std::fflush(stdout);
    };
  std::vector<std::string> queue;
  auto sub = node->create_subscription<std_msgs::msg::String>(
    "/pndshot/cmd", 10, [&queue](const std_msgs::msg::String & m) {queue.push_back(m.data);});

  QTimer pump;
  QObject::connect(&pump, &QTimer::timeout, [&]() {
      rclcpp::spin_some(node);
      while (!queue.empty()) {
        const std::string cmd = queue.front();
        queue.erase(queue.begin());
        const QString c = QString::fromStdString(cmd);
        if (c == "quit") {
          reply("ok " + cmd);
          app.quit();
          return;
        }
        if (c.startsWith("snap:")) {
          const QString name = c.mid(5);
          panel->grab().save(out + "/" + name + ".png");
          QJsonArray all;
          for (auto * w : panel->findChildren<QWidget *>()) {
            if (w->isVisibleTo(panel)) {all.append(describe(w, panel));}
          }
          QJsonObject doc;
          doc["width"] = panel->width();
          doc["height"] = panel->height();
          doc["widgets"] = all;
          QFile f(out + "/" + name + ".json");
          if (f.open(QIODevice::WriteOnly)) {f.write(QJsonDocument(doc).toJson());}
          reply("ok " + cmd);
          continue;
        }
        if (c.startsWith("click:")) {
          QString want = c.mid(6);
          int nth = 0;
          const int hash = want.lastIndexOf('#');
          if (hash > 0) {
            nth = want.mid(hash + 1).toInt();
            want = want.left(hash);
          }
          std::vector<QPushButton *> hits;
          for (auto * b : panel->findChildren<QPushButton *>()) {
            if (b->isVisibleTo(panel) && flat(b->text()).contains(want)) {hits.push_back(b);}
          }
          std::sort(hits.begin(), hits.end(), [panel](QPushButton * a, QPushButton * b) {
              return a->mapTo(panel, QPoint(0, 0)).y() < b->mapTo(panel, QPoint(0, 0)).y();
            });
          if (nth >= static_cast<int>(hits.size())) {
            reply("err " + cmd + ": no such button");
          } else if (!hits[nth]->isEnabled()) {
            reply("err " + cmd + ": disabled (" + flat(hits[nth]->text()).toStdString() + ")");
          } else {
            hits[nth]->click();
            reply("ok " + cmd);
          }
          continue;
        }
        reply("err " + cmd + ": unknown command");
      }
    });
  pump.start(5);
  const int r = app.exec();
  rclcpp::shutdown();
  return r;
}
