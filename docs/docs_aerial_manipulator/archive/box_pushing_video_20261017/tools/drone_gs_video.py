#!/usr/bin/env python3
"""The drone ground station (ros2_ground_station_gui, unmodified) laid out for the
right half of the video frame. The station's page is a fixed 1662 x 861 layout;
here the same widgets are moved into a TALL page:

  left column     everything the station already has at x < 730 (state, mode,
                  the Move ENU / Controller tabs, Return to baseline, labels) -- unchanged
  right column    the rotor N/T display and the command log
  bottom          the Flight Log tab (position + body-angle plots), enlarged

and opens on the Flight Log tab and the Controller tab. Frameless, so the client
area is the whole window. Run with ROS sourced and an interpreter that has PyQt5
and rclpy (/usr/bin/python3 here):

    drone_gs_video.py [x y w h]          (default 1316 27 1244 1400)
"""
import os
import sys

GUI = os.path.expanduser("~/ros2_ws/src/ros2_ground_station_gui")
sys.path.insert(0, os.path.join(GUI, "src"))
os.chdir(GUI)                       # the stack's pane runs it from the repo root

from PyQt5 import QtCore, QtWidgets  # noqa: E402

import rclpy  # noqa: E402
from GUI.single_drone_flight import Ui_MultiDroneGroundControlStation  # noqa: E402
from ROS_Node.ros_single_drone_control import SingleDroneRosThread  # noqa: E402

X, Y, W, H = (map(int, sys.argv[1:5]) if len(sys.argv) >= 5 else (1316, 27, 1244, 1400))
TAB_BAR = 34                        # the top-level tab bar ("Autonomous Flight")


def relayout(ui):
    pw, ph = W - 4, H - TAB_BAR
    rx = 745                        # right column
    cw = pw - rx - 15
    ui.label_2.setGeometry(QtCore.QRect(rx, 20, cw, 31))
    ui.display_quad_rotor_NT.setGeometry(QtCore.QRect(rx + (cw - 360) // 2, 55, 360, 300))
    ui.label_total_nt.setGeometry(QtCore.QRect(588, 230, 140, 31))    # "Total N/T: 56%" was cut
    # the controller / task / vehicle labels and Back to Baseline, out of the left
    # block where the labels were cut off at 241 px
    ui.verticalLayoutWidget.setGeometry(QtCore.QRect(rx + 10, 370, cw - 10, 120))
    ui.buttom_back_to_baseline.setGeometry(QtCore.QRect(rx, 500, cw, 70))
    ui.list_cmd_log.setGeometry(QtCore.QRect(rx, 585, cw, 225))
    # the Move ENU / Controller tabs take the freed width, and the controller table with them
    ui.tabWidget.setGeometry(QtCore.QRect(30, 490, 700, 321))
    ui.avaliable_controllers.setGeometry(QtCore.QRect(160, 30, 528, 252))
    # the lab's network lamps (OptiTrack VRPN, Orin WiFi, Orin CPU) have no source in
    # simulation and would read red for the whole video
    for w in (ui.optitrack_status, ui.wifi_status, ui.cpu_status):
        w.hide()
    top = 825
    ui.Additional_Function.setGeometry(QtCore.QRect(15, top, pw - 30, ph - top - 10))
    gw, gh = pw - 36, ph - top - 10 - 34          # the tab page inside
    half = (gw - 15) // 2
    ui.display_x_y_z.setGeometry(QtCore.QRect(5, 5, half, gh - 10))
    ui.display_body_angle.setGeometry(QtCore.QRect(10 + half, 5, half, gh - 10))
    ui.Additional_Function.setCurrentIndex(ui.Additional_Function.indexOf(ui.Flight_Graph))
    ui.tabWidget.setCurrentIndex(ui.tabWidget.indexOf(ui.special_controller_tab))


if __name__ == "__main__":
    rclpy.init(args=None)
    app = QtWidgets.QApplication(sys.argv[:1])
    win = QtWidgets.QTabWidget()
    ui = Ui_MultiDroneGroundControlStation()
    ui.setupUi(win)
    relayout(ui)                    # BEFORE the ROS thread sizes its plots to the containers
    th = SingleDroneRosThread(ui)
    th.start()
    hdr = ui.avaliable_controllers.horizontalHeader()      # its 3 columns exist now
    hdr.setSectionResizeMode(0, QtWidgets.QHeaderView.ResizeToContents)
    hdr.setSectionResizeMode(2, QtWidgets.QHeaderView.ResizeToContents)
    hdr.setSectionResizeMode(1, QtWidgets.QHeaderView.Stretch)
    win.setWindowTitle("Drone Ground Station")
    win.setWindowFlags(win.windowFlags() | QtCore.Qt.FramelessWindowHint)
    win.setGeometry(X, Y, W, H)
    win.show()
    print("System Started", flush=True)
    try:
        sys.exit(app.exec_())
    finally:
        rclpy.shutdown()
