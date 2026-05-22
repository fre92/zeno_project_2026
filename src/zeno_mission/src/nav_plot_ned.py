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

    rospy.init_node("nav_plot_ned")

    # origin dal server parametri
    lat0 = rospy.get_param("/origin/coordinates/latitude")
    lon0 = rospy.get_param("/origin/coordinates/longitude")
    origin = (lat0, lon0)

    
    #waypoint (in NED) da file params.yamal
    waypoints = rospy.get_param("/waypoints_RRT")
    wp_ns = []
    wp_es = []
    for n, e in waypoints:
        wp_ns.append(n)
        wp_es.append(e)
    
    # target (North, East)
    targets = rospy.get_param("/targets")
    target_ns = []
    target_es = []
    for n, e in targets:
        target_ns.append(n)
        target_es.append(e)


    #ostacoli (North, East, Radius)
    obstacles = rospy.get_param("/obstacles")   


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

        # targets
        if len(target_ns) > 0: 
            ax.scatter(target_ns,target_es, c='magenta',s=120,marker='x',linewidths=3,label='Targets')

        #plot obstacles 
        for n, e, r in obstacles:
            obstacle_circle = Circle(
            (n, e),      # centro
            r,           # raggio
            color='red',
            alpha=0.3)
            ax.add_patch(obstacle_circle)

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