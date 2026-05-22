#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import sys
import numpy as np

from marta_msgs.msg import NavStatus
from joystick_command.msg import Rel_error_joystick
from geodetic_functions import ll2ne

class Phase1Controller:

    def __init__(self):

        rospy.init_node("phase1_pure_pursuit_controller" , anonymous=True)

        # CARICAMENTO PARAMETRI
        if not rospy.has_param("/origin/coordinates/latitude") or not rospy.has_param("/origin/coordinates/longitude"):
            rospy.logerr("ERRORE FATALE: Coordinate di origine non trovate sul Parameter Server.")
            sys.exit(1)

        lat0 = rospy.get_param("/origin/coordinates/latitude")
        lon0 = rospy.get_param("/origin/coordinates/longitude")
        self.origin = (lat0, lon0)

       
        # CARICAMENTO WAYPOINT
        if not rospy.has_param("waypoints_phase1"):
            rospy.logerr("ERRORE FATALE: Parametro 'waypoints_phase1' non trovato")
            sys.exit(1)
            
        raw_wps = rospy.get_param("waypoints_phase1")
        self.waypoints = np.array(raw_wps)
        
        rospy.loginfo("Inizializzazione completata. Caricati %d waypoint", len(self.waypoints))

        # Stato di Navigazione
        self.current_lat = None
        self.current_lon = None
        self.current_yaw = None

        self.current_wp_idx = 0
        self.mission_started = False  # Flag per gestire l'allineamento iniziale
        
        self.total_distance = 0.0
        self.last_n = None
        self.last_e = None
        self.t_start = None

        # Parametri di Controllo
        self.position_th = 0.8 #parametro per considerare wp raggiunto 
        self.max_surge_speed = 0.2
        self.max_track_error = 3.0 #massimo cross_error ammesso 
        self.max_heading_deg = 45.0
        self.distance_gain = 2.0 #parametro per quando inziare a ridurrre la velocità

        # ROS I/O
        self.joystic_pub = rospy.Publisher("/relative_error", Rel_error_joystick, queue_size=1)
        rospy.Subscriber("/nav_status", NavStatus, self.nav_callback)

        self.timer = rospy.Timer(rospy.Duration(0.1), self.control_loop) # Loop a 10Hz
        rospy.on_shutdown(self.on_shutdown_hook)

    def nav_callback(self, msg):
        self.current_lat = msg.position.latitude
        self.current_lon = msg.position.longitude
        self.current_yaw = msg.orientation.yaw

    def control_loop(self, event):

        # ATTESA PRIMO SEGNALE GPS
        if None in (self.current_lat, self.current_lon, self.current_yaw):
            rospy.loginfo_throttle(5.0, "Attesa dati GPS / NavStatus...") #Stampo ogni 5 Secondi 
            return

        current_n, current_e = ll2ne(self.origin, (self.current_lat, self.current_lon))

        # AVVIO MISSIONE (se non ancora avviata )
        # =======================================================
        # 3. LOGICA DI AVVIO MISSIONE E SMART START
        # =======================================================
        if not self.mission_started:
            
            # Quanti waypoint abbiamo?
            N = len(self.waypoints)
            current_pos = np.array([current_n, current_e])
            
            # Procediamo allo Smart Start solo se abbiamo almeno 2 transetti (4 wp)
            # e se il numero di wp è pari (come dovrebbe essere per i tagliaerba)
            if N >= 4 and N % 2 == 0:
                
                # 1. Calcoliamo la distanza dai 4 angoli nevralgici
                d0 = np.linalg.norm(current_pos - self.waypoints[0])   # Inizio originale
                d1 = np.linalg.norm(current_pos - self.waypoints[1])   # Fine primo transetto
                d2 = np.linalg.norm(current_pos - self.waypoints[-2])  # Inizio ultimo transetto
                d3 = np.linalg.norm(current_pos - self.waypoints[-1])  # Fine ultimo transetto
                
                distances = [d0, d1, d2, d3]
                min_dist = min(distances)
                
                # si prende la prima occorrenza 
                best_corner = distances.index(min_dist)

                # 2. Riordino della lista in base al corner più vicino
                if best_corner == 0:
                    #ordine attuale dei wp 
                    
                elif best_corner == 1:
                    # Trasforma in coppie -> inverte le colonne (sx-dx) -> riporta a lista
                    self.waypoints = self.waypoints.reshape(-1, 2, 2)[:, ::-1].reshape(-1, 2)
                    
                elif best_corner == 2:
                    # Trasforma in coppie -> inverte l'ordine delle righe -> riporta a lista
                    self.waypoints = self.waypoints.reshape(-1, 2, 2)[::-1, :, :].reshape(-1, 2)
                    
                elif best_corner == 3:
                    # Capovolge l'intera lista
                    self.waypoints = self.waypoints[::-1]
                
                dist_to_start = min_dist
            else:
                dist_to_start = np.linalg.norm(current_pos - self.waypoints[0])
            
            # Se siamo lontani, tracciamo la rotta per avvicinarci a wp[0]
            if dist_to_start > 1.0:
                rospy.loginfo("Generazione punto di rientro (Zeno -> Start Point).")
                self.waypoints = np.vstack([[current_n, current_e], self.waypoints])

            #rospy.set_param("waypoints_phase1_active", self.waypoints.tolist())

            self.t_start = rospy.Time.now()
            self.mission_started = True
            rospy.loginfo("MISSIONE FASE 1 INIZIATA! Zeno in movimento.")
            return # Al prossimo ciclo parte effettivamente la missione 

        # Aggiornamento distanza percorsa
        if self.last_n is not None:
            self.total_distance += math.sqrt((current_n - self.last_n)**2 + (current_e - self.last_e)**2)

        self.last_n = current_n
        self.last_e = current_e

        # CHECK FINE MISSIONE
    
        if self.current_wp_idx >= len(self.waypoints) - 1:

            # messaggio di stop 
            self.joystic_pub.publish(Rel_error_joystick(error_surge_speed=0.0, error_yaw=0.0))

            t_end = rospy.Time.now()
            mission_time = (t_end - self.t_start).to_sec()

            rospy.loginfo("==============================================")
            rospy.loginfo("MISSIONE FASE 1 COMPLETATA!")
            rospy.loginfo("Tempo di missione: %.2f secondi", mission_time)
            rospy.loginfo("Distanza totale percorsa: %.2f metri", self.total_distance)
            rospy.loginfo("==============================================")
            
            rospy.signal_shutdown("Fase 1 terminata con successo.")
            return

        # CONTROLLO PURE PURSUIT

        wp_start = self.waypoints[self.current_wp_idx]
        wp_end = self.waypoints[self.current_wp_idx + 1]

        wp_start_n, wp_start_e = wp_start
        wp_end_n, wp_end_e = wp_end

        dx = wp_end_n - wp_start_n
        dy = wp_end_e - wp_start_e
        segment_length = math.sqrt(dx**2 + dy**2)

        rx = current_n - wp_start_n
        ry = current_e - wp_start_e

        yaw_path = math.atan2(dy, dx)   

        # Proiezione sul segmento
        t = (rx*dx + ry*dy) / (dx**2 + dy**2)
        t = max(0.0, min(1.0, t))

        proj_n = wp_start_n + t * dx
        proj_e = wp_start_e + t * dy

        # Errore Cross Track 
        error_track = (current_n - proj_n)*(-math.sin(yaw_path)) + (current_e - proj_e)*(math.cos(yaw_path))

        # Modulazione Look-ahead: Errore = 0m -> L = 3.5m ; Errore >= 3m -> L = 2m  ;   Errore = (0 , 3)m -> L = (2 , 3.5)m 
        L = 2.0 + 1.5 * (1.0 - min(abs(error_track) / self.max_track_error, 1.0))

        if abs(error_track) >= L:
            dl = 0.0
        else:
            dl = math.sqrt(L**2 - abs(error_track)**2)

        t_lookahead = t + (dl / segment_length)    
        t_lookahead = min(1.0, t_lookahead) 

        target_n = wp_start_n + t_lookahead * dx
        target_e = wp_start_e + t_lookahead * dy

        # Prua desiderata (Virtual Point)
        yaw_des = math.atan2(target_e - current_e, target_n - current_n)

        # Distanza da fine segmento
        dist = math.sqrt((wp_end_n - current_n)**2 + (wp_end_e - current_e)**2)
        rospy.loginfo_throttle(1.0, "Distanza al WP %d: %.2f m", self.current_wp_idx + 1, dist)

        # Transizione Waypoint
        if dist < self.position_th or t >= 1.0:
            rospy.loginfo("Waypoint %d raggiunto.", self.current_wp_idx + 1)
            self.current_wp_idx += 1
            return

        # CALCOLO COMANDI 
        yaw_error = self.wrapToPi(yaw_des - self.current_yaw)
        
        max_heading = np.deg2rad(self.max_heading_deg)
        yaw_error = max(-max_heading, min(max_heading, yaw_error))
        yaw_error_deg = math.degrees(yaw_error)

        heading_factor = 1.0 - min(abs(yaw_error_deg) / self.max_heading_deg, 1.0)
        distance_factor = min(dist / self.distance_gain, 1.0)
        track_factor = 1.0 - min(abs(error_track) / self.max_track_error, 1.0)

        surge_factor = min(heading_factor, distance_factor, track_factor)
        surge = self.max_surge_speed * surge_factor

        cmd = Rel_error_joystick()
        cmd.error_yaw = yaw_error_deg
        cmd.error_surge_speed = surge

        self.joystic_pub.publish(cmd)

    def wrapToPi(self, angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def on_shutdown_hook(self):
        #Se controllo viene killato (Ctrl+C) diamo comuqnue il comando di STOP a zeno 
        self.joystic_pub.publish(Rel_error_joystick(error_surge_speed=0.0, error_yaw=0.0))

if __name__ == "__main__":
    try:
        controller = Phase1Controller()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
