#!/usr/bin/env python

import rospy
from zeno_python.msg import WaypointPath
from geometry_msgs.msg import Point


def main():

    rospy.init_node("waypoint_test_publisher")

    pub = rospy.Publisher("/waypoint_path", WaypointPath, queue_size=1 , latch=True)

    rospy.sleep(1.0)  # IMPORTANT: aspetta connessione subscriber

    msg = WaypointPath()

    points = [
        (11.0, 2.0),
        (11.0, 22.0),
        (21.0, 23.0),
        (21.0, 2.0),
        (31.0, 2.0),
        (31.0, 23.0),
        (41.0, 22.0),
        (41.0, 2.0)
    ]

    for n, e in points:
        p = Point()
        p.x = n
        p.y = e
        msg.waypoints.append(p)

    rospy.loginfo("Publishing waypoint path...")

    pub.publish(msg)

    rospy.loginfo("Done (one-shot publish)")

    rospy.sleep(1.0)


if __name__ == "__main__":
    main()