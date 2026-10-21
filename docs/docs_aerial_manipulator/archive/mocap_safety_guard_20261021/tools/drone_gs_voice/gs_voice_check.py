"""Offscreen check of the drone GS's mocap-guard alarm (run from the patched repo root).

The REAL SingleDroneRosThread runs offscreen; its siren and speech players are replaced
by stand-ins that log what would have played (start, end, cancel) with timestamps. A
second node in this process plays a scripted world on the same (private) ROS domain:
guard health at 50 Hz, PX4 vehicle_status, raw OptiTrack, the Orin's system status.
Prints the audio log and the GS flight log, then the checks.
"""
import json
import os
import sys
import threading
import time

LOG = sys.argv[1]
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.getcwd(), 'src'))

import rclpy  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy  # noqa: E402
from PyQt5 import QtWidgets, QtCore  # noqa: E402
from geometry_msgs.msg import PoseStamped  # noqa: E402
from std_msgs.msg import Float64MultiArray, String  # noqa: E402
from px4_msgs.msg import VehicleStatus  # noqa: E402

import ROS_Node.ros_single_drone_control as gs  # noqa: E402
from GUI.single_drone_flight import Ui_MultiDroneGroundControlStation  # noqa: E402

gs.SPEECH_COMMAND = os.path.join(HERE, 'fake_say.sh')
gs.SIREN_PLAYER = [os.path.join(HERE, 'fake_siren.sh')]
open(LOG, 'w').close()

T0 = time.monotonic()
ARMED, DISARMED = 2, 1
OFFBOARD, STAB = 14, 15


def world(t):
    """(guard state or None = silent, cause, PX4 arming, PX4 nav, OptiTrack on, Orin status on)."""
    g, c = 0, 0
    arm, nav, opt, orin = ARMED, OFFBOARD, True, True
    if 3.0 <= t < 4.2:
        g, c = 2, 2                       # a short fault: CONFIRMED, never LOST
    if 6.0 <= t < 6.1:
        g, c = 1, 3                       # one rejected flip: must stay quiet
    if 9.0 <= t < 30.0:                   # Motive pause, armed in OFFBOARD
        g, c = (2, 2) if t < 9.85 else (3, 2)
        opt = t >= 26.0
    if t >= 22.0 and t < 38.0:
        nav = STAB                        # the pilot took over: repeats stop
    if 33.0 <= t < 36.0:
        g = None                          # guard silent (estimator or link)
    if 38.0 <= t < 48.0:                  # a fault while disarmed: no takeover words
        arm = DISARMED
        g, c = (2, 1) if t < 38.85 else (3, 1)
    if 52.5 <= t < 60.0:                  # LOST while "No OptiTrack" is playing
        opt = False
        if t >= 53.3:
            g, c = (2, 2) if t < 53.9 else (3, 2)
    return g, c, arm, nav, opt, orin


class World(Node):
    def __init__(self):
        super().__init__('voice_check_world')
        be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        durability=DurabilityPolicy.VOLATILE,
                        history=HistoryPolicy.KEEP_LAST, depth=5)
        px4 = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=5)
        self.guard = self.create_publisher(Float64MultiArray, gs.MOCAP_GUARD_TOPIC, be)
        self.status = self.create_publisher(VehicleStatus, '/uav_0/fmu/out/vehicle_status_v1', px4)
        self.opt = self.create_publisher(PoseStamped, gs.OPTITRACK_TOPIC, be)
        self.orin = self.create_publisher(String, gs.SYSTEM_STATUS_TOPIC, be)
        self.n = 0
        self.create_timer(0.01, self.tick)

    def tick(self):
        t = time.monotonic() - T0
        g, c, arm, nav, opt, orin = world(t)
        self.n += 1
        if g is not None and self.n % 2 == 0:
            m = Float64MultiArray()
            d = [float('nan')] * 16
            d[0], d[1], d[2], d[12] = float(g), float(c), 0.0 if g == 0 else 0.5, 0.0
            m.data = d
            self.guard.publish(m)
        if opt:
            self.opt.publish(PoseStamped())
        if self.n % 20 == 0:
            s = VehicleStatus()
            s.arming_state, s.nav_state, s.pre_flight_checks_pass = arm, nav, True
            self.status.publish(s)
        if orin and self.n % 100 == 0:
            self.orin.publish(String(data=json.dumps({
                'wifi': {'connected': True, 'signal_dbm': -50, 'ping_ms': 5, 'freq_mhz': 5180,
                         'udp_txq_max_bytes': 0, 'qdisc_backlog_max_pkts': 0},
                'cpu': {'load_pct': [10, 10, 10, 10, 10, 10]}})))


rclpy.init()
world_node = World()
ex = SingleThreadedExecutor()
ex.add_node(world_node)
threading.Thread(target=ex.spin, daemon=True).start()

app = QtWidgets.QApplication(sys.argv)
win = QtWidgets.QTabWidget()
ui = Ui_MultiDroneGroundControlStation()
ui.setupUi(win)
ros_thread = gs.SingleDroneRosThread(ui)
ros_thread.start()
win.show()


def finish():
    with open(LOG) as f:
        audio = [line.split(' ', 1) for line in f.read().splitlines()]
    print('--- audio (s since start) ---')
    for stamp, what in audio:
        print(f'{float(stamp) - T0_WALL:7.2f}  {what}')
    print('--- GS flight log ---')
    for i in range(ui.list_cmd_log.count()):
        print(ui.list_cmd_log.item(i).text())
    rclpy.shutdown()
    os._exit(0)


T0_WALL = time.time() - (time.monotonic() - T0)
QtCore.QTimer.singleShot(64000, finish)
app.exec_()
