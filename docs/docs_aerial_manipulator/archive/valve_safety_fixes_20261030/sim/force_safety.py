#!/usr/bin/env python3
"""Force the whole-body node out of DIRECT DELAY s into the turn (the 10-30 safety fixes' item 3 check):
waits for the push-and-pull status "FLYING push", then calls set_direct_mode(false) once and logs the
answer. Run beside a valve flight (raw ROS, the campaign's chain starts it in the background).

    /usr/bin/python3 force_safety.py [DELAY_S] [LOG]
"""
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool

DELAY = float(sys.argv[1]) if len(sys.argv) > 1 else 5.0
LOG = sys.argv[2] if len(sys.argv) > 2 else "/dev/stdout"


def main():
    rclpy.init()
    n = Node("force_safety_check")
    st = {"s": "", "t0": None}
    n.create_subscription(String, "/uav_0/whole_body_planner/push_pull/status",
                          lambda m: st.__setitem__("s", m.data), 10)
    cli = n.create_client(SetBool, "/uav_0/fsc_autopilot_ros2/whole_body_direct_actuation/set_direct_mode")
    out = open(LOG, "a")
    t_end = time.monotonic() + 900.0
    while rclpy.ok() and time.monotonic() < t_end:
        rclpy.spin_once(n, timeout_sec=0.05)
        if st["t0"] is None and st["s"].startswith("FLYING push"):
            st["t0"] = time.monotonic()
            out.write(f"{time.time():.3f} FLYING push seen\n")
            out.flush()
        if st["t0"] is not None and time.monotonic() - st["t0"] >= DELAY:
            cli.wait_for_service(timeout_sec=5.0)
            fut = cli.call_async(SetBool.Request(data=False))
            t1 = time.monotonic()
            while not fut.done() and time.monotonic() - t1 < 5.0:
                rclpy.spin_once(n, timeout_sec=0.05)
            r = fut.result()
            out.write(f"{time.time():.3f} set_direct_mode(false) {DELAY:.1f} s into the turn: "
                      f"{r.success if r else None} {r.message if r else 'no answer'} (status was {st['s'][:60]})\n")
            out.flush()
            break
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
