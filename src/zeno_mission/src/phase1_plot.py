#!/usr/bin/env python

import rospy
import matplotlib.pyplot as plt

from marta_msgs.msg import NavStatus
from geodetic_functions import ll2ne
from matplotlib.patches import Circle
import os
import rospkg
import time
import numpy as np



import threading

# Dati Global

n_traj = []
e_traj = []
origin = None


lock = threading.Lock()

def callback(msg):
    global n_traj, e_traj, origin

    if origin is None:
        return

    lat = msg.position.latitude
    lon = msg.position.longitude

    n, e = ll2ne(origin, (lat, lon))
    
    with lock:
        n_traj.append(n)
        e_traj.append(e)


def save_final_plot_on_shutdown():
    try:
        
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('zeno_mission') 
        
        # 2. Gestione cartella 'img'
        img_dir = os.path.join(pkg_path, 'img')
        if not os.path.exists(img_dir):
            os.makedirs(img_dir)
            
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        image_name = "mission_phase1_{}.png".format(timestamp)
        image_path = os.path.join(img_dir, image_name)
        
        plt.gcf().savefig(image_path, dpi=800, bbox_inches='tight')
        
        rospy.loginfo("Immagine di missione salvata con successo in: %s", image_path)
        
    except Exception as e:
        rospy.logwarn("Impossibile salvare l'immagine finale durante lo shutdown: %s", str(e))        


def listener():

    global n_traj, e_traj, origin

    rospy.init_node("phase1_plot", anonymous=True)

    rospy.on_shutdown(save_final_plot_on_shutdown)


    # CARICAMENTO PARAMETRI 
    # origine 
    if not rospy.has_param("/origin/coordinates/latitude"):
        rospy.logerr("Origine non trovata sul server! Esco.")
        sys.exit(1)
        
    lat0 = rospy.get_param("/origin/coordinates/latitude")
    lon0 = rospy.get_param("/origin/coordinates/longitude")
    origin = (lat0, lon0)

    #poligono 
    if not rospy.has_param("polygon_vertices/original") or not rospy.has_param("polygon_vertices/restricted"):
        rospy.logerr("Poligoni non trovati sul server!")
        sys.exit(1)

    poly_orig_raw = rospy.get_param("polygon_vertices/original")
    poly_orig = np.array(poly_orig_raw)
    poly_orig = np.vstack((poly_orig, poly_orig[0]))

    poly_restr_raw = rospy.get_param("polygon_vertices/restricted")
    poly_restr = np.array(poly_restr_raw)
    poly_restr = np.vstack((poly_restr, poly_restr[0]))

    #CARICAMENTO WAYPOINT 
    if not rospy.has_param("waypoints_phase1"):
        rospy.logerr("Waypoints Fase 1 non trovati sul server! Esco.")
        sys.exit(1)

    waypoints_raw = rospy.get_param("waypoints_phase1")
    wp_ns = [wp[0] for wp in waypoints_raw]
    wp_es = [wp[1] for wp in waypoints_raw]

    
   
    rospy.Subscriber("/nav_status", NavStatus, callback)

    # PLOT 

    plt.ion()
    fig, ax = plt.subplots()

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():

        with lock:
            if len(n_traj) == 0:
                rate.sleep()
                continue
            
            n_copy = n_traj[:]
            e_copy = e_traj[:]

        ax.clear()

        # Poligoni 
        ax.plot(poly_orig[:, 1], poly_orig[:, 0], 'k-', linewidth=2, label='Poligono Originale')
        ax.plot(poly_restr[:, 1], poly_restr[:, 0], 'g--', linewidth=2, label='Poligono Ristretto')

        # Waypoint (e percorso ideale)
        if len(wp_es) > 0:
            ax.plot(wp_es, wp_ns, 'bo', markersize=4, alpha=0.5, label='Rotta Pianificata')

        #traiettoria effettiva di Zeno
        if len(n_copy) > 0: 
            ax.plot(e_copy, n_copy, 'm-', linewidth=2.5, label='Traiettoria Zeno ')
            ax.scatter(e_copy[-1], n_copy[-1], c='red', s=120, zorder=5, label='Zeno' )

       
        ax.set_xlabel("East [m]")
        ax.set_ylabel("North [m]")
        ax.set_title("Mission Control - Phase 1 Coverage")
        ax.axis("equal") # Cruciale per non deformare la mappa!
        ax.grid(True, linestyle=':', alpha=0.7)
        ax.legend(loc='upper right', fontsize=9)

        #
        plt.pause(0.05)
        rate.sleep()


# ENTRY POINT

if __name__ == "__main__":
    try:
        listener()
    except rospy.ROSInterruptException:
        pass