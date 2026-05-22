#!/usr/bin/env python

import rospy
import matplotlib.pyplot as plt

from marta_msgs.msg import NavStatus
from geodetic_functions import ll2ne
from matplotlib.patches import Circle

import threading

# Dati Global

xs = []
ys = []
origin = None


lock = threading.Lock()

def callback(msg):
    global xs, ys, origin

    lat = msg.position.latitude
    lon = msg.position.longitude

    n, e = ll2ne(origin, (lat, lon))
    
    with lock:
        xs.append(n)
        ys.append(e)



# MAIN

def listener():

    global origin

    rospy.init_node("plot_phase1")

    # origin dal server parametri
    lat0 = rospy.get_param("/origin/coordinates/latitude")
    lon0 = rospy.get_param("/origin/coordinates/longitude")
    origin = (lat0, lon0)

    
    #waypoint (in NED) da file params.yamal
    waypoints = rospy.get_param("/waypoints")
    wp_ns = []
    wp_es = []
    for n, e in waypoints:
        wp_ns.append(n)
        wp_es.append(e)
    
   
    rospy.Subscriber("/nav_status", NavStatus, callback)

    # matplotlib interactive mode
    plt.ion()
    fig, ax = plt.subplots()

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():

        with lock:
            if len(xs) == 0:
                rate.sleep()
                continue
            
            x_copy = xs[:]
            y_copy = ys[:]

        ax.clear()

        # traiettoria
        if len(xs) > 0: 
            ax.plot(x_copy, y_copy, 'b-', linewidth=2)

        # posizione attuale
        ax.scatter(x_copy[-1], y_copy[-1], c='red', s=120)

        # plot waypoint
        if len(wp_es) > 0:
            ax.scatter(wp_ns, wp_es, c='green', s=60, marker='o', label='waypoints')

        ax.set_xlabel("North (m)")
        ax.set_ylabel("East (m)")
        ax.set_title("Zeno trajectory in NED")
        ax.axis("equal")

        ax.legend()

        plt.pause(0.001)
        rate.sleep()


# ENTRY POINT

if __name__ == "__main__":
    try:
        listener()
    except rospy.ROSInterruptException:
        pass