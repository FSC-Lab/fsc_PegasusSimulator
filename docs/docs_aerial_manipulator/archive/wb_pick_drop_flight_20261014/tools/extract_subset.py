#!/usr/bin/env python3
"""Reduced rosbag2 -> npz extractor for the flight-history scan (loose-arm signature, 2026-10-14 analysis).

Same flattening as ../../wb_pick_place_flight_20261009/tools/extract_bag.py, but only the topics the
yaw / motor / vibration comparison needs, so all hardware bags since 2026-09-18 extract in minutes.
Usage: /usr/bin/python3 extract_subset.py <bag dir> <out.npz>   (ROS 2 + ~/ros2_ws/install sourced)
"""
import os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "wb_pick_place_flight_20261009", "tools"))
import extract_bag as E  # noqa: E402

KEEP = ["wb", "wbmode", "l1", "gmode", "act_motors", "mocap", "batt", "vatt", "sc", "vstatus", "pp_status",
        "ctype", "vinfo"]
E.TOPICS = {k: v for k, v in E.TOPICS.items() if k in KEEP}

if __name__ == "__main__":
    E.main(sys.argv[1], sys.argv[2])
