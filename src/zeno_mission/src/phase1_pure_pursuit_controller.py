#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import sys
import csv
import os
import rospkg
import time
import numpy as np

from marta_msgs.msg import NavStatus
from joystick_command.msg import Rel_error_joystick
from geodetic_functions import ll2ne
from std_msgs.msg import String



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
        self.current_timestamp = None

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

        # Parametri comunicazione con SSS
        self.sss_margin = 2.0  # [m] Raggio della bolla di spegnimento
        self.sss_state = "OFF" # Stato iniziale

        # ROS I/O
        self.joystic_pub = rospy.Publisher("/relative_error", Rel_error_joystick, queue_size=1)
        rospy.Subscriber("/nav_status", NavStatus, self.nav_callback)

        self.telemetry_pub = rospy.Publisher("/phase1/telemetry", String, queue_size=1)
        self.sss_pub = rospy.Publisher("/phase1/SSS", String, queue_size=1)

        # Setup cartella logs
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('zeno_mission') # Usa il nome del tuo pacchetto
        log_dir = os.path.join(pkg_path, 'logs')
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)

        # Creazione del file con Timestamp
        timestamp_str = time.strftime("%Y%m%d_%H%M%S")
        self.csv_path = os.path.join(log_dir, "phase1_log_{}.csv".format(timestamp_str))
        self.csv_file = open(self.csv_path, mode='w')
        self.csv_writer = csv.writer(self.csv_file, delimiter=',')
        
        # Scrittura dell'Intestazione del CSV
        self.csv_writer.writerow([
            "Time_sec ", " Lat ", " Lon ", " Yaw_deg ", " North_m ", " East_m ", 
            " Target_WP_idx ", " Dist_to_WP_m ", " CrossTrackErr_m ", 
            " Surge_Cmd ", " Yaw_Cmd_deg ", " Event"
        ])
        rospy.loginfo("Data Logger avviato. File: %s", self.csv_path)    


        self.timer = rospy.Timer(rospy.Duration(0.1), self.control_loop) # Loop a 10Hz
        rospy.on_shutdown(self.on_shutdown_hook)

    def nav_callback(self, msg):
        self.current_lat = msg.position.latitude
        self.current_lon = msg.position.longitude
        self.current_yaw = msg.orientation.yaw
        self.current_timestamp = msg.header.stamp

    def control_loop(self, event):

        if rospy.is_shutdown():
            return


        # ATTESA PRIMO SEGNALE GPS
        if None in (self.current_lat, self.current_lon, self.current_yaw):
            rospy.loginfo_throttle(5.0, "Attesa dati GPS / NavStatus...") #Stampo ogni 5 Secondi 
            return

        current_n, current_e = ll2ne(self.origin, (self.current_lat, self.current_lon))

        #varibaile per registre eventi 
        event_msg = ""

        # AVVIO MISSIONE (se non ancora avviata )
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

                event_msg = "Smart Start: Corner {} scelto".format(best_corner)

                # Riordino della lista in base al corner più vicino
                if best_corner == 0:
                    event_msg = "Smart Start: Corner 0 (Inizio Originale)."
                    
                elif best_corner == 1:
                    event_msg = "Smart Start: Corner 1 (Inversione sx-dx)."
                    self.waypoints = self.waypoints.reshape(-1, 2, 2)[:, ::-1, :].reshape(-1, 2)
                    
                elif best_corner == 2:
                    event_msg = "Smart Start: Corner 2 (Partenza dal fondo)."
                    self.waypoints = self.waypoints.reshape(-1, 2, 2)[::-1, :, :].reshape(-1, 2)
                    
                elif best_corner == 3:
                    event_msg = "Smart Start: Corner 3 (Inversione Totale)."
                    self.waypoints = self.waypoints[::-1]
                
                dist_to_start = min_dist
            else:
                dist_to_start = np.linalg.norm(current_pos - self.waypoints[0])
            
            # Se siamo lontani, tracciamo la rotta per avvicinarci a wp[0]
            if dist_to_start > 1.0:
                self.waypoints = np.vstack([[current_n, current_e], self.waypoints])
                event_msg += " | Inserito WP Rientro"

            #rospy.set_param("waypoints_phase1_active", self.waypoints.tolist())

            self.t_start = self.current_timestamp
            self.mission_started = True
            rospy.loginfo("MISSIONE FASE 1 INIZIATA! Zeno in movimento.")

            self.log_state(current_n, current_e, 0.0, 0.0, 0.0, 0.0, event_msg)
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

            t_end = self.current_timestamp
            mission_time = (t_end - self.t_start).to_sec()

            rospy.loginfo("==============================================")
            rospy.loginfo("MISSIONE FASE 1 COMPLETATA!")
            rospy.loginfo("Tempo di missione: %.2f secondi", mission_time)
            rospy.loginfo("Distanza totale percorsa: %.2f metri", self.total_distance)
            rospy.loginfo("==============================================")

            #riepilogo missione, scatola nera 
            if not self.csv_file.closed:
                self.csv_writer.writerow([
                    "{:.2f}".format(mission_time),  # Time_sec
                    "", "", "", "", "",             # Lat, Lon, Yaw, North, East vuoti
                    "",                             # Target_WP_idx vuoto
                    "{:.2f}".format(self.total_distance), # campo della distanza per idnicare la distanza totale 
                    "", "", "",                     # CrossTrack, Surge, Yaw_cmd vuoti
                    "RIEPILOGO: Missione completata con successo! Distanza totale: {:.2f}m, Tempo: {:.2f}s".format(self.total_distance, mission_time)
                ])

         
            rospy.signal_shutdown("Fase 1 terminata con successo.")
            return


        # CONTROLLO 
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
        #rospy.loginfo_throttle(1.0, "Distanza al WP %d: %.2f m", self.current_wp_idx + 1, dist)


        # Accensione/Spegnimento side scan sonar 
        # ist da wp precedente 
        dist_from_start = math.sqrt((current_n - wp_start_n)**2 + (current_e - wp_start_e)**2)

        # Se in ingresso/uscita curva OFF , altrimenti ON 
        if dist < self.sss_margin or dist_from_start < self.sss_margin:
            new_sss_state = "OFF"
        else:
            new_sss_state = "ON"

        # Avviso stato cambiato
        if new_sss_state != self.sss_state:
            rospy.loginfo("Comando SSS -> %s (Dist. Inizio: %.1fm | Dist. Fine: %.1fm)", new_sss_state, dist_from_start, dist)
            self.sss_state = new_sss_state
        
        # Pubblicazione su topic  (pubblichiamo il comando in continuo a 10Hz )
        self.sss_pub.publish(String(data=self.sss_state))


        # Transizione Waypoint
        if dist < self.position_th or t >= 1.0:
            #rospy.loginfo("Waypoint %d raggiunto.", self.current_wp_idx + 1)
            event_msg = "WP {} Reached".format(self.current_wp_idx + 1)
            self.log_state(current_n, current_e, dist, error_track, 0.0, 0.0, event_msg)

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

        self.log_state(current_n, current_e, dist, error_track, surge, yaw_error_deg, event_msg)


    def wrapToPi(self, angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def on_shutdown_hook(self):
        #Se controllo viene killato (Ctrl+C) diamo comuqnue il comando di STOP a zeno 
        self.joystic_pub.publish(Rel_error_joystick(error_surge_speed=0.0, error_yaw=0.0))

        # Chiude e salva il file CSV in modo pulito
        if not self.csv_file.closed:
            self.csv_file.close()
            rospy.loginfo("Scatola Nera salvata con successo.")

    def log_state(self, current_n, current_e, dist, error_track, surge, yaw_error_deg, event_msg):
        
        telemetry_str = "WP: {}/{} | Dist: {:.2f}m | X-Track: {:.2f}m | Surge: {:.2f} | YawErr: {:.1f} deg".format(
            self.current_wp_idx + 1, len(self.waypoints)-1, dist, error_track, surge, yaw_error_deg
        )
        self.telemetry_pub.publish(String(data=telemetry_str))

        # Scrittura CSV
        try:
            if not self.csv_file.closed:
                if self.t_start and self.current_timestamp:
                    current_time = (self.current_timestamp - self.t_start).to_sec()
                else:
                    current_time = 0.0

                self.csv_writer.writerow([
                    "{:.2f}  ".format(current_time),
                    "{:.6f}  ".format(self.current_lat),
                    "{:.6f}  ".format(self.current_lon),
                    "{:.2f}  ".format(math.degrees(self.current_yaw)),
                    "{:.2f}  ".format(current_n),
                    "{:.2f}  ".format(current_e),
                    self.current_wp_idx + 1,
                    "{:.2f}  ".format(dist),
                    "{:.2f}  ".format(error_track),
                    "{:.2f}  ".format(surge),
                    "{:.2f}  ".format(yaw_error_deg),
                    event_msg
                ])
        except ValueError:
    
            pass


        


if __name__ == "__main__":
    try:
        controller = Phase1Controller()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
