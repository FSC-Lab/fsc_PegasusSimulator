#!/usr/bin/env python3
"""Bag -> npz for the 2026-10-23 valve spin test (obj_0 = the valve's rotor).

Usage: extract.py <bag_dir> <out.npz>
Writes, per topic, receive time (s), header stamp (s), position (m), quaternion (x,y,z,w);
for /obj_0/mocap also every numeric field, flattened.
"""
import sqlite3, sys, glob, os, numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

bag, out = sys.argv[1], sys.argv[2]
db = os.path.join(bag, [f for f in os.listdir(bag) if f.endswith(".db3")][0])  # no glob: the path has [ ]
con = sqlite3.connect(db)
topics = {r[0]: (r[1], r[2]) for r in con.execute("select id, name, type from topics")}
res = {}
def flat(m, pre, d):
    for k in m.get_fields_and_field_types():
        v = getattr(m, k)
        if hasattr(v, "get_fields_and_field_types"):
            flat(v, pre + k + ".", d)
        elif isinstance(v, (int, float, bool)):
            d.setdefault(pre + k, []).append(float(v))
for tid, (name, typ) in topics.items():
    cls = get_message(typ)
    rows = con.execute("select timestamp, data from messages where topic_id=? order by timestamp", (tid,)).fetchall()
    d = {"recv": []}
    for ts, data in rows:
        m = deserialize_message(data, cls)
        d["recv"].append(ts * 1e-9)
        flat(m, "", d)
    key = "vrpn" if "vrpn" in name else "mocap"
    for k, v in d.items():
        res[f"{key}__{k}"] = np.asarray(v)
    print(name, typ, len(rows), sorted(d.keys())[:40])
np.savez(out, **res)
