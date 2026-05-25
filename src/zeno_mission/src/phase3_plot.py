#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import matplotlib.pyplot as plt
from marta_msgs.msg import NavStatus
from zeno_mission.msg import WaypointPath
from geodetic_functions import ll2ne
from matplotlib.patches import Circle
import threading
import matplotlib.path as mpltPath
import json
from datetime import datetime

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

    # 2. Lettura Targets (in GPS) e conversione in NED
    target_ns, target_es = [], []
    # 3. Lettura Ostacoli (in GPS) e conversione in NED
    obstacles = []
    try:
        obstacles_gps = rospy.get_param("/obstacles")
        for obs in obstacles_gps:
            lat = obs[0]
            lon = obs[1]
            radius = obs[2]
            n, e = ll2ne(origin, (lat, lon))
            obstacles.append([n, e, radius])
    except KeyError:
        rospy.logwarn("Parametro /obstacles non trovato.")

    # =========================================================
    # 2 & 4. Lettura Poligoni (NED), Target (GPS->NED) e Geofencing
    # =========================================================
    poly_original_ned = []
    poly_restricted_ned = []
    raw_targets_ned = []
    target_ns, target_es = [], []
    poly_n, poly_e = [], []

    # A. Carichiamo entrambi i poligoni (GIÀ IN NED)
    try:
        poly_original_ned = rospy.get_param("/polygon_vertices/original")
    except KeyError:
        rospy.logwarn("Parametro /polygon_vertices/original non trovato.")

    try:
        poly_restricted_ned = rospy.get_param("/polygon_vertices/restricted")
    except KeyError:
        rospy.logwarn("Parametro /polygon_vertices/restricted non trovato.")

    # B. Carichiamo tutti i target "grezzi" (convertendoli da GPS a NED)
    try:
        for t in rospy.get_param("/targets"):
            n, e = ll2ne(origin, (t[0], t[1]))
            raw_targets_ned.append([n, e])
    except KeyError:
        rospy.logwarn("Parametro /targets non trovato.")

    # C. Applichiamo la stessa logica decisionale del planner
    if poly_original_ned and poly_restricted_ned and raw_targets_ned:
        path_orig = mpltPath.Path(poly_original_ned)
        path_restr = mpltPath.Path(poly_restricted_ned)
        usa_originale = False

        for t in raw_targets_ned:
            in_restr = path_restr.contains_point((t[0], t[1]))
            in_orig = path_orig.contains_point((t[0], t[1]))

            if in_restr:
                target_ns.append(t[0])
                target_es.append(t[1])
            elif in_orig:
                target_ns.append(t[0])
                target_es.append(t[1])
                usa_originale = True
                
        # Scegliamo la linea da disegnare
        poly_to_draw = poly_original_ned if usa_originale else poly_restricted_ned
    else:
        # Fallback se mancano pezzi
        poly_to_draw = poly_restricted_ned if poly_restricted_ned else poly_original_ned
        for t in raw_targets_ned:
            target_ns.append(t[0])
            target_es.append(t[1])

    # D. Prepariamo le liste per Matplotlib (e chiudiamo il perimetro)
    if poly_to_draw and len(poly_to_draw) > 0:
        poly_n = [p[0] for p in poly_to_draw]
        poly_e = [p[1] for p in poly_to_draw]
        poly_n.append(poly_n[0])
        poly_e.append(poly_e[0])


    # Iscrizione ai Topic
    rospy.Subscriber("/nav_status", NavStatus, nav_callback)
    rospy.Subscriber("/waypoint_path", WaypointPath, wp_callback)

# --- SETUP GRAFICO ---
    plt.ion()
    fig, ax = plt.subplots(figsize=(10, 8))
    rate = rospy.Rate(10)

    rospy.loginfo("Plotter in ascolto. In attesa del planner A*...")

    # ========================================================
    # AGGIUNTA 1: Flag per gestire il primissimo avvio
    # ========================================================
    primo_disegno = True 

    while not rospy.is_shutdown():
        with lock:
            x_copy = xs[:]
            y_copy = ys[:]
            wp_n_copy = wp_ns[:]
            wp_e_copy = wp_es[:]

        # ========================================================
        # AGGIUNTA 2: Salva i limiti (solo se ha già inquadrato la mappa)
        # ========================================================
        if not primo_disegno:
            limiti_x = ax.get_xlim()
            limiti_y = ax.get_ylim()

        ax.clear()

        # Disegna l'Area di Gara (Prima l'Est, poi il Nord)
        if len(poly_n) > 0:
            ax.plot(poly_e, poly_n, 'k--', linewidth=2, label='Area di Gara')

        # Disegna Ostacoli
        for obs in obstacles:
            n, e, r = obs[0], obs[1], obs[2]
            obstacle_circle = Circle((e, n), r, color='red', alpha=0.3)
            ax.add_patch(obstacle_circle)
            ax.scatter(e, n, c='darkred', s=20, marker='x')

        # Disegna Targets
        if len(target_ns) > 0:
            ax.scatter(target_es, target_ns, c='gold', edgecolors='black', s=300, marker='*', zorder=5, label='Targets')

        # Disegna Waypoints
        if len(wp_e_copy) > 0:
            ax.scatter(wp_e_copy, wp_n_copy, c='limegreen', s=40, marker='o', zorder=4, label='Waypoints')
            ax.plot(wp_e_copy, wp_n_copy, c='lightgreen', linestyle=':', linewidth=1)

        # Disegna Traiettoria
        if len(x_copy) > 0:
            ax.plot(y_copy, x_copy, 'b-', linewidth=2.5, label='Traiettoria Zeno')
            ax.scatter(y_copy[-1], x_copy[-1], c='red', edgecolors='white', s=150, zorder=6, label='Zeno')

        ax.set_xlabel("East [m]", fontsize=12)
        ax.set_ylabel("North [m]", fontsize=12)
        ax.set_title("Zeno AUV - Mission Control", fontsize=14, fontweight='bold')
        ax.axis("equal")
        ax.grid(True, linestyle=':', alpha=0.6)
        
        handles, labels = ax.get_legend_handles_labels()
        # Piccolo fix di sicurezza: crea la legenda solo se ci sono etichette
        if labels: 
            by_label = dict(zip(labels, handles))
            ax.legend(by_label.values(), by_label.keys(), loc='upper right')

        # ========================================================
        # AGGIUNTA 3: Ripristina i limiti salvati 
        # ========================================================
        if not primo_disegno:
            ax.set_xlim(limiti_x)
            ax.set_ylim(limiti_y)
        elif len(poly_n) > 0 or len(wp_n_copy) > 0 or len(x_copy) > 0:
            # Se ha finalmente disegnato qualcosa di concreto, 
            # dal prossimo frame abilitiamo il salvataggio dello zoom!
            primo_disegno = False
        # ========================================================

        plt.pause(0.05)
        rate.sleep()

    rospy.loginfo("Chiusura Plotter. Salvataggio dati e immagine in corso...")
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    
    # 1. Salva l'immagine del grafico finale in alta risoluzione
    percorso_img = "/media/sf_ros_condivisa/mappa_finale_{}.png".format(timestamp)
    fig.savefig(percorso_img, dpi=300, bbox_inches='tight')
    rospy.loginfo("Mappa salvata in: %s", percorso_img)

    # 2. Salva la traiettoria reale (le liste xs e ys)
    with lock:
        dati_reali = {
            "timestamp": timestamp,
            "x_reali_ned": xs,
            "y_reali_ned": ys
        }
        
    percorso_json = "/media/sf_ros_condivisa/traiettoria_reale_{}.json".format(timestamp)
    with open(percorso_json, 'w') as f:
        json.dump(dati_reali, f, indent=4)
        
    rospy.loginfo("Traiettoria reale salvata in: %s", percorso_json)

if __name__ == "__main__":
    try:
        listener()
    except rospy.ROSInterruptException:
        pass