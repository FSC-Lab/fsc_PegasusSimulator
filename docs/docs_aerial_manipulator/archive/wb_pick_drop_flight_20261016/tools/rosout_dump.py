#!/usr/bin/env python3
"""Dump /rosout (time since bag start, node, message) of a bag.  usage: rosout_dump.py <bag dir>"""
import os, sqlite3, sys
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
b = sys.argv[1]
db = [f for f in os.listdir(b) if f.endswith(".db3")][0]
c = sqlite3.connect(os.path.join(b, db)).cursor()
t0 = c.execute("SELECT MIN(timestamp) FROM messages").fetchone()[0]
tid, typ = c.execute("SELECT id,type FROM topics WHERE name='/rosout'").fetchone()
cls = get_message(typ)
for ts, blob in c.execute("SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp", (tid,)):
    m = deserialize_message(bytes(blob), cls)
    print(f"{(ts - t0) * 1e-9:8.2f} [{m.name}] {m.msg}")
