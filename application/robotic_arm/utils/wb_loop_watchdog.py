#!/usr/bin/env python3
"""Whole-body control-loop health watchdog -- a recordable debug output.

WHY THIS EXISTS (2026-08-31). The whole-body DIRECT loop has almost no
tolerance for a control-loop stall: measured on shiqi-desktop, a 330 ms freeze
of the control node produced a 0.6 m position excursion, and a 500 ms freeze
crashed the vehicle. The stall itself is NON-deterministic -- it comes from OS
scheduling on a non-real-time desktop (randomized systemd timers, P-core/E-core
migration on the i9-14900KF, GUI/Isaac bursts) -- so a run that diverges and a
run that does not can be byte-identical in configuration.

This node makes the trigger VISIBLE and RECORDABLE, so a divergence in a rosbag
can be classified as "stall-induced" or "something else" instead of guessed at.

It measures the ARRIVAL cadence of wb_control_debug (published every control
tick, 250 Hz). Caveat, stated honestly: under heavy load this node can itself be
descheduled, so a reported gap is "the loop OR this monitor stalled". It is a
canary, not ground truth; the node's own dt is. It is still the cheapest
detector, and it is what first exposed the 329 ms gap that preceded a watchdog
trip.

Publishes ~5 Hz on <ns>/wb_loop_health (std_msgs/Float32MultiArray):
    [0] dt of the most recent interval                       [s]
    [1] worst dt in the last window                          [s]
    [2] worst dt since start                                 [s]
    [3] measured rate over the last window                   [Hz]
    [4] gaps > 20 ms  (cumulative)
    [5] gaps > 50 ms  (cumulative)   <- jitter worth noticing
    [6] gaps > 200 ms (cumulative)   <- excursion territory
    [7] gaps > 400 ms (cumulative)   <- crash territory
    [8] 1 while the node reports DIRECT, else 0

Record it alongside everything else:
    ros2 bag record /uav_0/wb_loop_health \
        /uav_0/fsc_autopilot_ros2/whole_body_direct_actuation/wb_control_debug

Run (namespace matches the vehicle, same as the flight stack):
    python3 wb_loop_watchdog.py --ros-args -r __ns:=/uav_0
"""
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String

DEBUG_TOPIC = ("fsc_autopilot_ros2/whole_body_direct_actuation/wb_control_debug")
MODE_TOPIC = "fsc_autopilot_ros2/whole_body_direct_actuation/mode"

# Thresholds. NOMINAL is the 250 Hz control period; the rest are the measured
# consequence bands from the 2026-08-31 stall campaign.
NOMINAL_DT = 1.0 / 250.0
WARN_DT = 0.020          # 5x nominal -- jitter, not yet dangerous
EXCURSION_DT = 0.200     # ~0.33 s produced a 0.6 m excursion
CRASH_DT = 0.400         # 0.50 s crashed the vehicle
WINDOW_S = 1.0


class LoopWatchdog(Node):
    def __init__(self):
        super().__init__("wb_loop_watchdog")
        self._t_prev = None
        self._direct = False
        self._worst_ever = 0.0
        self._worst_win = 0.0
        self._dt_last = 0.0
        self._n_win = 0
        self._t_win = time.monotonic()
        self._counts = [0, 0, 0, 0]      # >20ms, >50ms, >200ms, >400ms

        self.create_subscription(Float32MultiArray, DEBUG_TOPIC, self._on_debug, 50)
        self.create_subscription(String, MODE_TOPIC, self._on_mode, 10)
        self._pub = self.create_publisher(Float32MultiArray, "wb_loop_health", 10)
        self.create_timer(0.2, self._publish)
        self.get_logger().info(
            "whole-body loop watchdog up: watching %s, publishing wb_loop_health "
            "(warn >%.0f ms, excursion >%.0f ms, crash >%.0f ms)"
            % (DEBUG_TOPIC, WARN_DT * 1e3, EXCURSION_DT * 1e3, CRASH_DT * 1e3))

    def _on_mode(self, msg):
        self._direct = (msg.data.strip().upper() == "DIRECT")

    def _on_debug(self, _msg):
        now = time.monotonic()
        if self._t_prev is not None:
            dt = now - self._t_prev
            self._dt_last = dt
            self._n_win += 1
            self._worst_win = max(self._worst_win, dt)
            self._worst_ever = max(self._worst_ever, dt)
            for i, thr in enumerate((WARN_DT, 0.050, EXCURSION_DT, CRASH_DT)):
                if dt > thr:
                    self._counts[i] += 1
            # Loud, immediate, and only for gaps that actually matter: a
            # silent log is useless when the vehicle is already diverging.
            if dt > CRASH_DT:
                self.get_logger().error(
                    "CONTROL LOOP STALLED %.0f ms -- past the 400 ms band that "
                    "CRASHED the vehicle in test. Expect divergence." % (dt * 1e3))
            elif dt > EXCURSION_DT:
                self.get_logger().error(
                    "CONTROL LOOP STALLED %.0f ms -- excursion territory "
                    "(330 ms gave 0.6 m in test)." % (dt * 1e3))
            elif dt > WARN_DT and self._direct:
                self.get_logger().warn(
                    "control loop jitter %.0f ms (nominal %.0f ms)"
                    % (dt * 1e3, NOMINAL_DT * 1e3), throttle_duration_sec=2.0)
        self._t_prev = now

    def _publish(self):
        now = time.monotonic()
        span = now - self._t_win
        rate = (self._n_win / span) if span > 0 else 0.0
        m = Float32MultiArray()
        m.data = [float(self._dt_last), float(self._worst_win),
                  float(self._worst_ever), float(rate),
                  float(self._counts[0]), float(self._counts[1]),
                  float(self._counts[2]), float(self._counts[3]),
                  1.0 if self._direct else 0.0]
        self._pub.publish(m)
        if span >= WINDOW_S:
            self._t_win, self._n_win, self._worst_win = now, 0, 0.0


def main():
    try:
        rclpy.init()
    except RuntimeError:
        pass                      # already initialised is not an error
    node = LoopWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(
            "worst gap seen: %.0f ms | gaps >20/50/200/400 ms: %d/%d/%d/%d"
            % (node._worst_ever * 1e3, *node._counts))


if __name__ == "__main__":
    main()
