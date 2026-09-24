"""Controller-switch history of a bag: every change on active_controller, and the message count on each cascade's cmd/status topic -- a clean flight changes only at the engage and the handover (2026-09-23). Usage: bag_switch_history.py <bag dir>"""
import sqlite3, sys, glob
from rclpy.serialization import deserialize_message
from std_msgs.msg import String
d = glob.glob(sys.argv[1] + "/*.db3")[0]
c = sqlite3.connect(d)
tops = {i: n for i, n, *_ in c.execute("select id,name,type from topics")}
for i, n in tops.items():
    if "cascade" in n and ("status" in n or "cmd" in n):
        k = c.execute("select count(*) from messages where topic_id=?", (i,)).fetchone()[0]
        print(f"{n}: {k}")
ac = [i for i, n in tops.items() if n.endswith("active_controller")][0]
rows = c.execute("select timestamp,data from messages where topic_id=? order by timestamp", (ac,)).fetchall()
prev = None; ch = 0; t0 = rows[0][0]
for ts, dat in rows:
    s = deserialize_message(dat, String).data
    if s != prev:
        ch += 1; print(f"  t+{(ts-t0)/1e9:7.1f}s  -> {s}")
        prev = s
print("messages", len(rows), "changes", ch)
