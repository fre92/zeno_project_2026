#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import matplotlib.pyplot as plt
from marta_msgs.msg import NavStatus
from zeno_python.msg import WaypointPath
from geodetic_functions import ll2ne
from matplotlib.patches import Circle
import threading

# --- Variabili Globali ---
xs = []
ys = []
wp_ns = []
wp_es = []
origin = None
lock = threading.Lock()

def nav_callback(msg):
    global xs, ys, origin
    lat = msg.position.latitude
    lon = msg.position.longitude
    n, e = ll2ne(origin, (lat, lon))
    
    with lock:
        xs.append(n)
        ys.append(e)

def wp_callback(msg):
    # Quando l'A* pubblica sul topic, salviamo i waypoint per disegnarli
    global wp_ns, wp_es
    with lock:
        wp_ns = [p.x for p in msg.waypoints]
        wp_es = [p.y for p in msg.waypoints]

def listener():
    global origin
    rospy.init_node("nav_plot_completo")

    # 1. Lettura Origine
    try:
        lat0 = rospy.get_param("/origin/coordinates/latitude")
        lon0 = rospy.get_param("/origin/coordinates/longitude")
        origin = (lat0, lon0)
    except KeyError:
        rospy.logerr("Origine non trovata! Lancio interrotto.")
        return

    # 2. Lettura Targets (già in NED)
    try:
        targets = rospy.get_param("/targets")
        target_ns = [t[0] for t in targets]
        target_es = [t[1] for t in targets]
    except KeyError:
        target_ns, target_es = [], []

    # 3. Lettura Ostacoli (già in NED)
    try:
        obstacles = rospy.get_param("/obstacles")
    except KeyError:
        obstacles = []

    # 4. Lettura Poligono (Direttamente dal file fisico!)
    poly_n, poly_e = [], []
    file_path = "/media/sf_ros_condivisa/src/zeno_python/src/vertici_area.txt" 
    
    try:
        with open(file_path, 'r') as f:
            for line in f:
                if line.strip():
                    lat, lon = map(float, line.strip().split(','))
                    n, e = ll2ne(origin, (lat, lon))
                    poly_n.append(n)
                    poly_e.append(e)
        
        # Chiudiamo il perimetro unendo l'ultimo punto al primo
        if len(poly_n) > 0:
            poly_n.append(poly_n[0])
            poly_e.append(poly_e[0])
            rospy.loginfo("Poligono caricato con successo nel plotter.")
            
    except IOError:
        rospy.logerr("File vertici_area.txt non trovato. Il plotter non disegnerà l'area di gara.")

    # Iscrizione ai Topic
    rospy.Subscriber("/nav_status", NavStatus, nav_callback)
    rospy.Subscriber("/waypoint_path", WaypointPath, wp_callback)

    # --- SETUP GRAFICO ---
    plt.ion()
    fig, ax = plt.subplots(figsize=(10, 8))
    rate = rospy.Rate(10)

    rospy.loginfo("Plotter in ascolto. In attesa del planner A*...")

    while not rospy.is_shutdown():
        with lock:
            x_copy = xs[:]
            y_copy = ys[:]
            wp_n_copy = wp_ns[:]
            wp_e_copy = wp_es[:]

        ax.clear()

        # Disegna l'Area di Gara
        if len(poly_n) > 0:
            ax.plot(poly_n, poly_e, 'k--', linewidth=2, label='Area di Gara')

        # Disegna Ostacoli
        for obs in obstacles:
            n, e, r = obs[0], obs[1], obs[2]
            obstacle_circle = Circle((n, e), r, color='red', alpha=0.3)
            ax.add_patch(obstacle_circle)
            ax.scatter(n, e, c='darkred', s=20, marker='x')

        # Disegna Targets
        if len(target_ns) > 0:
            ax.scatter(target_ns, target_es, c='gold', edgecolors='black', s=300, marker='*', zorder=5, label='Targets')

        # Disegna Waypoints
        if len(wp_e_copy) > 0:
            ax.scatter(wp_n_copy, wp_e_copy, c='limegreen', s=40, marker='o', zorder=4, label='Waypoints')
            ax.plot(wp_n_copy, wp_e_copy, c='lightgreen', linestyle=':', linewidth=1)

        # Disegna Traiettoria
        if len(x_copy) > 0:
            ax.plot(x_copy, y_copy, 'b-', linewidth=2.5, label='Traiettoria Zeno')
            ax.scatter(x_copy[-1], y_copy[-1], c='red', edgecolors='white', s=150, zorder=6, label='Zeno')

        ax.set_xlabel("North [m]", fontsize=12)
        ax.set_ylabel("East [m]", fontsize=12)
        ax.set_title("Zeno AUV - Mission Control", fontsize=14, fontweight='bold')
        ax.axis("equal")
        ax.grid(True, linestyle=':', alpha=0.6)
        
        handles, labels = ax.get_legend_handles_labels()
        by_label = dict(zip(labels, handles))
        ax.legend(by_label.values(), by_label.keys(), loc='upper right')

        plt.pause(0.05)
        rate.sleep()

if __name__ == "__main__":
    try:
        listener()
    except rospy.ROSInterruptException:
        pass