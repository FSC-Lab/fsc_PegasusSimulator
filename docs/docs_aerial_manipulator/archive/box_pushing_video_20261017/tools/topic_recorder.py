#!/usr/bin/env python3
"""Log the planner topics the Isaac viewport's overlays are drawn from, for a
REPLAYED video (2026-10-18; record_scene.py logs the poses, replay_scene.py
redraws both). One JSON line per message, wall-clock stamped:

    {"t": <time.time()>, "topic": "path" | "pose" | "info", "data": [...]}

  path  /uav_0/whole_body_planner/viz_path        06's blue (drone) / red (EE) curves, latched
  pose  /uav_0/whole_body_planner/viz_pose        06's two heading arrows
  info  /uav_0/whole_body_planner/push_pull/info  the 07 scenes' waypoint balls + labels, latched
  ppinfo  /uav_0/whole_body_planner/pick_place/info  the pick-and-place scene's waypoints (2026-10-18), latched
  params  the planner parameters the pick-and-place waypoints are drawn from before Plan, polled
          every 2 s and logged on a change: {"t", "topic": "params", "data": {name: value}}

    topic_recorder.py OUT.jsonl        (system python with ROS 2 sourced; stop with SIGINT/SIGTERM)
"""
import json
import signal
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64MultiArray

TOPICS = {"path": ("/uav_0/whole_body_planner/viz_path", True),
          "pose": ("/uav_0/whole_body_planner/viz_pose", False),
          "info": ("/uav_0/whole_body_planner/push_pull/info", True),
          "ppinfo": ("/uav_0/whole_body_planner/pick_place/info", True)}
# 07 pick-and-place's WP_BASE_PARAMS / WP_CLAW_PARAMS / WP_PLACE_POINT_PARAM / WP_DROP_PARAMS /
# WP_DROP_EXIT_PARAMS (its _wp_poll asks the planner for exactly these)
PLANNER = "/uav_0/whole_body_trajectory_planner"
PARAMS = ["pick_place_start", "pick_place_place_start", "pick_place_land_start", "pick_place_land",
          "pick_place_ee_offset", "pick_place_place_point", "pick_place_place_drop", "pick_place_drop_height",
          "pick_place_drop_exit_offset", "pick_place_drop_altitude", "pick_place_drop_exit_climb"]


def main():
    out = open(sys.argv[1], "a")
    rclpy.init()
    node = Node("video_topic_recorder")
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    count = {k: 0 for k in TOPICS}

    def cb(key):
        def f(msg):
            out.write(json.dumps({"t": time.time(), "topic": key, "data": list(msg.data)}) + "\n")
            out.flush()
            count[key] += 1
        return f

    for key, (name, is_latched) in TOPICS.items():
        node.create_subscription(Float64MultiArray, name, cb(key), latched if is_latched else 10)
    from rcl_interfaces.srv import GetParameters
    cli = node.create_client(GetParameters, PLANNER + "/get_parameters")
    poll = {"t": 0.0, "fut": None, "last": None}

    def poll_params():
        if poll["fut"] is not None and poll["fut"].done():
            vals = {}
            try:
                for n, v in zip(PARAMS, poll["fut"].result().values):
                    if v.type == 8:                       # PARAMETER_DOUBLE_ARRAY
                        vals[n] = list(v.double_array_value)
                    elif v.type == 1:                     # PARAMETER_BOOL
                        vals[n] = bool(v.bool_value)
                    elif v.type == 3:                     # PARAMETER_DOUBLE
                        vals[n] = float(v.double_value)
            except Exception:
                vals = None
            poll["fut"] = None
            if vals and vals != poll["last"]:
                poll["last"] = vals
                out.write(json.dumps({"t": time.time(), "topic": "params", "data": vals}) + "\n")
                out.flush()
                count["params"] = count.get("params", 0) + 1
        if poll["fut"] is None and time.time() - poll["t"] >= 2.0 and cli.service_is_ready():
            req = GetParameters.Request()
            req.names = list(PARAMS)
            poll["fut"] = cli.call_async(req)
            poll["t"] = time.time()

    stop = {"now": False}
    for s in (signal.SIGINT, signal.SIGTERM):
        signal.signal(s, lambda *_: stop.update(now=True))
    print(f"[topic_recorder] logging {', '.join(n for n, _ in TOPICS.values())} -> {sys.argv[1]}", flush=True)
    while not stop["now"]:
        rclpy.spin_once(node, timeout_sec=0.1)
        poll_params()
    print(f"[topic_recorder] stopped: {count}", flush=True)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
