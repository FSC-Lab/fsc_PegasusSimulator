#!/usr/bin/env python3
"""A stand-in for the basket's mocap body (/obj_0/mocap) for trying the arm GS's Get buttons without a vehicle.

Enter moves the basket between the PLACE hat and the PICK hat; f freezes its pose (what an untracked body looks
like: both Gets must refuse it); q quits. Run by gs_demo.sh on a private ROS domain.
"""
import math, random, sys, threading
import rclpy
from rclpy.node import Node
from fsc_autopilot_ros2_msgs.msg import Mocap

SPOTS = {"place": (-1.031, -1.028, 0.880, 2.0), "pick": (1.011, 1.021, 0.865, 1.0)}   # x, y, z [m], yaw [deg]


class Basket(Node):
    def __init__(self):
        super().__init__("fake_basket")
        self.pub = self.create_publisher(Mocap, "/obj_0/mocap", 10)
        self.where, self.frozen, self.last = "place", False, None
        self.create_timer(0.01, self.tick)

    def tick(self):
        x, y, z, yaw = SPOTS[self.where]
        if not self.frozen or self.last is None:
            j = lambda: random.gauss(0.0, 0.0003)
            self.last = (x + j(), y + j(), z + j(), math.radians(yaw + random.gauss(0.0, 0.05)))
        m = Mocap(); m.header.stamp = self.get_clock().now().to_msg(); m.header.frame_id = "world"
        m.pose.position.x, m.pose.position.y, m.pose.position.z = self.last[:3]
        m.pose.orientation.z = math.sin(0.5 * self.last[3]); m.pose.orientation.w = math.cos(0.5 * self.last[3])
        self.pub.publish(m)


def main():
    rclpy.init(); n = Basket()
    threading.Thread(target=rclpy.spin, args=(n,), daemon=True).start()
    say = lambda: print(f"\nThe basket is on the {n.where.upper()} hat at {SPOTS[n.where][:3]}, yaw {SPOTS[n.where][3]} deg"
                        f"{'  [FROZEN: both Gets should refuse]' if n.frozen else ''}.\n"
                        "  Enter = move it to the other hat    f = freeze / unfreeze its pose    q = quit\n> ", end="", flush=True)
    say()
    for line in sys.stdin:
        c = line.strip().lower()
        if c == "q":
            break
        if c == "f":
            n.frozen = not n.frozen
        else:
            n.where = "pick" if n.where == "place" else "place"; n.frozen = False
        say()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
