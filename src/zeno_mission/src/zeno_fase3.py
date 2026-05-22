#!/usr/bin/env python 
# -*- coding: utf-8 -*-
from __future__ import division
import rospy
import math
import random
import numpy as np
import matplotlib.pyplot as plt

# Importazione dei messaggi custom 
from joystick_command.msg import Rel_error_joystick
from marta_msgs.msg import NavStatus

# Importazione della libreria geodetica ufficiale (WGS-84 -> NED)
from geodetic_functions import ll2ne

# --- FUNZIONI GEOMETRICHE (NORD-EST) ---
def is_point_in_polygon(e, n, polygon):
    """Ray-casting algorithm (Est=X, Nord=Y per la mappa 2D)"""
    num = len(polygon)
    inside = False
    p1e, p1n = polygon[0]
    for i in range(num + 1):
        p2e, p2n = polygon[i % num]
        if n > min(p1n, p2n):
            if n <= max(p1n, p2n):
                if e <= max(p1e, p2e):
                    if p1n != p2n:
                        xints = (n - p1n) * (p2e - p1e) / (p2n - p1n) + p1e
                    if p1e == p2e or e <= xints:
                        inside = not inside
        p1e, p1n = p2e, p2n
    return inside

# --- CLASSE RRT CINEMATICO ---
class Node:
    def __init__(self, e, n, yaw):
	# Posizioni Est e Nord, Orientazione Yaw
        self.e = e 
        self.n = n 
        self.yaw = yaw 
	# Genitore
        self.parent = None

class KinodynamicRRT:
    def __init__(self, start, goal, polygon, obstacles, max_iter=3000): # 3000 sono i tentativi totali che fa l'algoritmo per trovare un percorso accettabile
        self.start = Node(start[0], start[1], start[2])
        self.goal = Node(goal[0], goal[1], 0) 
        self.polygon = polygon       
        self.obstacles = obstacles   
        self.max_iter = max_iter
        self.node_list = [self.start]
        
        self.max_yaw_rate = math.radians(45.0)
	#La velocità è impostata costante quindi non ottimale --> da correggere con la formula di Ettore
        self.v = 0.20 
	# "Balzi" di 3 secondi alla volta --> si traduce in una distanza tra nodi di 0.6m
        self.dt = 3.0 

    def plan(self):
        for _ in range(self.max_iter):
	    # Cerco un nodo random dove andare
            rnd_node = self.get_random_node()
	    # Prendo l'indice del nodo più vicino
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd_node)
	    # Prendo il nodo più vicino
            nearest_node = self.node_list[nearest_ind]
            
	    # Genera un collegamento tra l'ultimo nodo random generato e il nodo già appartenente all'albero più vicino
            new_node = self.steer(nearest_node, rnd_node)
            
	    # Controllo che il nuovo nodo sia accettabile con il bordo e gli ostacoli
            if self.check_collision(new_node):
                self.node_list.append(new_node)
                
                # Accettazione Waypoint a 1.5 metri di distanza --> se entro nel raggio di 1.5m del target vuol dire che l'ho preso (va diminuito??)
                if math.hypot(new_node.e - self.goal.e, new_node.n - self.goal.n) <= 1.5:
		    # Solo dopo che ho generato tutta la traiettoria verso un target creo effettivamente la guida generata dalla lista di nodi
                    return self.generate_final_course(len(self.node_list) - 1)
        return None

    def steer(self, from_node, to_node):
        # NED Frame: 0 e' Nord, Est e' 90 deg -> atan2(Est, Nord)
        target_yaw = math.atan2(to_node.e - from_node.e, to_node.n - from_node.n)
        yaw_error = target_yaw - from_node.yaw
	    # Normalizzazione
        yaw_error = (yaw_error + math.pi) % (2 * math.pi) - math.pi
        yaw_error = max(-self.max_yaw_rate, min(self.max_yaw_rate, yaw_error))
        
        new_yaw = from_node.yaw + yaw_error
        new_n = from_node.n + self.v * math.cos(new_yaw) * self.dt
        new_e = from_node.e + self.v * math.sin(new_yaw) * self.dt
        
        new_node = Node(new_e, new_n, new_yaw)
        new_node.parent = from_node
        return new_node

    def check_collision(self, node):
	# Controllo che il nuovo nodo sia nel poligono
        if not is_point_in_polygon(node.e, node.n, self.polygon):
            return False
	# Controllo che il nuovo nodo non colpisca un ostacolo
        for (oe, on, r) in self.obstacles:
            if math.hypot(node.e - oe, node.n - on) <= r + 0.5: 
                return False
        return True

    def get_random_node(self):
        # GOAL BIAS: Il 10% delle volte "forza" l'esplorazione verso il target attuale
	# Praticamente genero un numero random da 0 a 100, se questo numero è tra 0 e 9 il prossimo nodo è esattamente il target
        if random.randint(0, 100) < 10:
            return Node(self.goal.e, self.goal.n, 0)
            
        # Il restante 90% delle volte esplora la mappa
	# Coordinate Est del poligono
        e_p = [p[0] for p in self.polygon]
	# Coordinate Nord del poligono
        n_p = [p[1] for p in self.polygon]
        return Node(random.uniform(min(e_p), max(e_p)), random.uniform(min(n_p), max(n_p)), 0)

    def get_nearest_node_index(self, node_list, rnd_node):
        dlist = []
        for node in node_list:
            dist = math.hypot(node.e - rnd_node.e, node.n - rnd_node.n)
            angle_to_rnd = math.atan2(rnd_node.e - node.e, rnd_node.n - node.n)
            angle_diff = abs((angle_to_rnd - node.yaw + math.pi) % (2*math.pi) - math.pi)
            cost = dist + (angle_diff * 2.0) 
            dlist.append(cost)
        return dlist.index(min(dlist))

    def generate_final_course(self, last_index):
        path = []
        node = self.node_list[last_index]
        while node is not None:
            # Salviamo [Est, Nord, Yaw]
	    # Aggiungo al path un nodo e poi lo tratto come una lista e prendo il suo genitore per creare una ciclo
            path.append([node.e, node.n, node.yaw]) 
            node = node.parent
        return path[::-1]

# --- NODO ROS PRINCIPALE ---
class ZenoController:
    def __init__(self):
        rospy.init_node('zeno_rrt_planner', anonymous=True)
        
	# Per pubblicare su Joystick
        self.pub = rospy.Publisher('/relative_error', Rel_error_joystick, queue_size=1)
	# Per leggere Nav_Status
        rospy.Subscriber('/nav_status', NavStatus, self.nav_callback)
        
        self.current_lat = None
        self.current_lon = None
        self.current_yaw = None
        
        self.origin_lat = None
        self.origin_lon = None
        
        self.rate = rospy.Rate(5) 

    def nav_callback(self, msg):
        if msg.initialized:
            self.current_lat = msg.position.latitude
            self.current_lon = msg.position.longitude
            self.current_yaw = msg.orientation.yaw
            
            if self.origin_lat is None:
                self.origin_lat = self.current_lat
                self.origin_lon = self.current_lon

    def wait_for_sensors(self):
        rospy.loginfo("In attesa del segnale GPS /nav_status da Zeno...")
	# Finchè non ho dati (o il sistema viene spento) aspetto
        while not rospy.is_shutdown() and self.current_lat is None:
            self.rate.sleep()
        rospy.loginfo("Segnale acquisito. Origine impostata.")

    def execute_mission(self):
        self.wait_for_sensors()
        
        # --- 1. GENERAZIONE SCENARIO MULTI-TARGET ---
        # Poligono e Ostacoli in [Est, Nord]
        polygon = [
            [-5, -5], [25, -2], [37, 20], [18, 32], [-4, 15]
        ]
        
	# Problema: c'è il rischio di uscire dal bordo se il target è molto vicino ad esso --> fare attenzione al controllo della velocità
        # Percorso a 3 tappe
        targets_en = [
            [5, 18],  # T1
            [28, 22],  # T2
            [15, 15]    # T3 (corretto, interno al poligono!)
        ]
        
	# Ostacoli --> Da scegliere il raggio da evitare
        obstacles = [
            (0, 5, 3.0), (7.5, 5, 3.0), (10, 22, 3.0) , 
        ]
        
        # --- 2. PIANIFICAZIONE SEQUENZIALE ---
        rospy.loginfo("Avvio RRT Kinodynamic Multi-Target (WGS-84)...")
        
        # CHIAMATA ALLA LIBRERIA GEODETICA UFFICIALE --> Uso la funzione data dal professore per portarmi nel sistema di riferimento in metri NED
        origin_ll = (self.origin_lat, self.origin_lon)
        current_ll = (self.current_lat, self.current_lon)
        curr_n, curr_e = ll2ne(origin_ll, current_ll)
        
        current_start = [curr_e, curr_n, self.current_yaw]
        
        full_path = []
        
        # Ciclo sui target per unire i percorsi
        for i, target in enumerate(targets_en):
	    # Per ogni target creo un percorso, e per ognuno di questi lo start point è diverso --> devo ripartire da dove ho incontrato lo scorso target
            rospy.loginfo("Pianificazione Target {}/{} in ({}, {})".format(i+1, len(targets_en), target[0], target[1]))
            rrt = KinodynamicRRT(current_start, target, polygon, obstacles)
            path_segment = rrt.plan()
            
            if not path_segment:
                rospy.logerr("Impossibile raggiungere il target {}. Spegnimento.".format(i+1))
                self.stop_zeno()
                return
            
            if full_path:
                full_path.extend(path_segment[1:]) # Evita di duplicare il nodo di giunzione
            else:
                full_path.extend(path_segment)
                
            current_start = path_segment[-1] # Il nuovo start e' la fine di questo segmento
            
        rospy.loginfo("Percorso completo trovato con {} waypoint totali.".format(len(full_path)))
        
        # --- PREPARAZIONE GRAFICO REAL-TIME ---
        path = full_path # Rinominiamo per comodita'
      
        # --- SALVATAGGIO WAYPOINT SU FILE YAML ---
        
        with open("/home/student/catkin_ws/src/zeno_python/config/waypoints.yaml", "w") as f:

            f.write("waypoints_RRT:\n")

            for wp in full_path:

                east = wp[0]
                north = wp[1]

                # Salva nel formato [North, East]
                f.write("  - [{:.2f}, {:.2f}]\n".format(north, east))

        rospy.loginfo("Waypoints salvati in waypoints.yaml")







        plt.ion() 
        fig, ax = plt.subplots(figsize=(8, 8))
        
        poly_disp = polygon + [polygon[0]]
        pe, pn = zip(*poly_disp)

        path_e = [wp[0] for wp in path]
        path_n = [wp[1] for wp in path]

        # --- 3. INSEGUIMENTO TRAIETTORIA ---
        for wp in path:
            target_e, target_n = wp[0], wp[1]
            rospy.loginfo("In navigazione verso waypoint: ({:.2f}, {:.2f})".format(target_e, target_n))
            
            while not rospy.is_shutdown():
                # CHIAMATA CONTINUA ALLA LIBRERIA GEODETICA UFFICIALE
                current_ll = (self.current_lat, self.current_lon)
                curr_n, curr_e = ll2ne(origin_ll, current_ll)
                
                # - AGGIORNAMENTO GRAFICO -
                ax.cla() 
                
                # Mappa e Ostacoli
                ax.plot(pe, pn, '-k', linewidth=2, label='Confini Mappa')
                for (oe, on, r) in obstacles:
                    circle = plt.Circle((oe, on), r, color='red', alpha=0.4)
                    ax.add_patch(circle)
                
                # Scia RRT
                ax.plot(path_e, path_n, '--b', alpha=0.4, label='Path RRT')
                ax.plot(path_e, path_n, 'b.', alpha=0.6) 
                
                # TUTTI I Target Finali
                for idx, t in enumerate(targets_en):
                    ax.plot(t[0], t[1], 'rX', markersize=14)
                    ax.text(t[0] + 1, t[1] + 1, 'T{}'.format(idx+1), 
				color='red', fontsize=12, fontweight='bold')
                
                # Waypoint Inseguito Attualmente
                # ax.plot(target_e, target_n, 'y*', markersize=14, markeredgecolor='black', label='WP Attuale')
                
                # Zeno e Freccia orientamento (sin=Est, cos=Nord)
                ax.plot(curr_e, curr_n, 'go', markersize=8, label='Zeno')
                ax.arrow(curr_e, curr_n, math.sin(self.current_yaw)*2, math.cos(self.current_yaw)*2, 
                         head_width=1, head_length=1.5, fc='green', ec='green')
                
                # Vista
                ax.set_xlim([-10, 45])
                ax.set_ylim([-10, 40])
                ax.set_xlabel("Est (m)")
                ax.set_ylabel("Nord (m)")
                ax.grid(True)
                ax.set_title("Zeno Kinodynamic RRT Live (NED Frame WGS-84)")
                ax.legend(loc='upper left')
                
                plt.pause(0.05) 

                # - LOGICA DI CONTROLLO NED -
		# Comandi da inviare a Zeno per seguire la rotta prestabilita
                dist = math.hypot(target_e - curr_e, target_n - curr_n)
                if dist < 1.0: 
                    break 
                
                desired_yaw_rad = math.atan2(target_e - curr_e, target_n - curr_n)
                
                error_yaw_rad = desired_yaw_rad - self.current_yaw
                error_yaw_rad = (error_yaw_rad + math.pi) % (2 * math.pi) - math.pi 
                
                error_yaw_deg = math.degrees(error_yaw_rad)
                error_yaw_deg = max(-45.0, min(45.0, error_yaw_deg))
                
                msg = Rel_error_joystick()
		# Velocità impostata constante non ottimale --> da cambiare con la formula di Ettore
                msg.error_surge_speed = 0.20 
                msg.error_yaw = error_yaw_deg
                self.pub.publish(msg)
                
                self.rate.sleep()
                
        rospy.loginfo("Missione Completata! Fermo il veicolo.")
        self.stop_zeno()
        
        plt.ioff()
        plt.show()

    def stop_zeno(self):
        stop_msg = Rel_error_joystick()
        self.pub.publish(stop_msg)

if __name__ == '__main__':
    try:
        controller = ZenoController()
        controller.execute_mission()
    except rospy.ROSInterruptException:
        pass