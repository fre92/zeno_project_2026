#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import numpy as np
import csv
import os
import rospkg
import time
from std_msgs.msg import String


from marta_msgs.msg import NavStatus
from joystick_command.msg import Rel_error_joystick
from geodetic_functions import ll2ne

from zeno_mission.msg import WaypointPath


class WPController:

    def __init__(self):

        rospy.init_node("path_controller")

        # Lettura parametri da file 
        lat0 = rospy.get_param("/origin/coordinates/latitude")
        lon0 = rospy.get_param("/origin/coordinates/longitude")
        self.origin = (lat0, lon0)

        # Stato
        self.current_lat = None
        self.current_lon = None
        self.current_yaw = None
        self.current_stamp = None

        self.waypoints = []
        self.current_wp_idx = 0

        self.path_received = False
        self.path_active = False
        self.path_waiting_logged = False

        self.mission_id = 0

        self.total_distance = 0.0
        self.last_n = None
        self.last_e = None

        self.t_start = None

        # Parametri
        self.position_th = 1
        self.max_surge_speed = 0.2

        self.K = 0.7
        self.max_track_error = 3.0

        self.max_heading_deg = 45.0
        self.distance_gain = 2.0

	self.sss_state = "OFF" # Stato iniziale


        # ROS I/O
        self.joystic_pub = rospy.Publisher( "/relative_error",Rel_error_joystick, queue_size=1 )

        rospy.Subscriber( "/nav_status", NavStatus, self.nav_callback)

        rospy.Subscriber( "/waypoint_path", WaypointPath, self.path_callback)

        self.telemetry_pub = rospy.Publisher("/phase3/telemetry", String, queue_size=1)
	self.sss_pub = rospy.Publisher("/phase3/SSS", String, queue_size=1)
        
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('zeno_mission') 
        log_dir = os.path.join(pkg_path, 'logs')
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)

        timestamp_str = time.strftime("%Y%m%d_%H%M%S")
        self.csv_path = os.path.join(log_dir, "phase3_log_{}.csv".format(timestamp_str))
        self.csv_file = open(self.csv_path, mode='w')
        self.csv_writer = csv.writer(self.csv_file, delimiter=',')
        
        # Intestazione CSV 
        self.csv_writer.writerow([
            "Time_sec", "Lat", "Lon", "Yaw_deg", "North_m", "East_m", 
            "Dist_to_Node_m", "CrossTrackErr_m", 
            "Surge_Cmd", "Yaw_Cmd_deg", "Event"
        ])
        rospy.loginfo("Path_ctr: Data Logger Fase 3 avviato. File: %s", self.csv_path)

        # Timer
        self.timer = rospy.Timer(rospy.Duration(0.1), self.control_loop)
        rospy.on_shutdown(self.on_shutdown_hook)
    
    def nav_callback(self, msg):
        self.current_lat = msg.position.latitude
        self.current_lon = msg.position.longitude
        self.current_yaw = msg.orientation.yaw
        self.current_stamp = msg.header.stamp
    
    def path_callback(self, msg):

        if self.path_active:
            return

        rospy.loginfo("Path_ctr: New waypoint path received")

        self.waypoints = []


        for p in msg.waypoints:
            self.waypoints.append((p.x, p.y , p.z))

        self.waypoints = np.array(self.waypoints)    

        if None not in (self.current_lat, self.current_lon, self.origin):

            current_n, current_e = ll2ne(self.origin, (self.current_lat, self.current_lon))
            
            # distanza euclidea dal primo wp
            dist_to_start = math.sqrt((self.waypoints[0][0] - current_n)**2 + (self.waypoints[0][1] - current_e)**2)
            
            if dist_to_start > 1.0:
                #rospy.loginfo("Zeno si trova a %.2f metri dal primo WP.", dist_to_start)
                
                self.waypoints = np.vstack([[current_n, current_e , 0], self.waypoints])
        else:
            rospy.logwarn("Dati GPS o Origine non ancora pronti.")    

        self.path_received = True
        self.path_active = True
        self.mission_id += 1
        self.t_start = None
       

   
    def control_loop(self, event):

        if rospy.is_shutdown():
            return


        if not self.path_received:
            if not self.path_waiting_logged:
                rospy.loginfo("Path_ctr: Atessa waypoint path...")
                self.path_waiting_logged = True # Blocco stampe successive
            return

        if None in (self.current_lat, self.current_lon, self.current_yaw, self.origin, self.current_stamp):
            rospy.loginfo_throttle(5.0, "Path_ctr: Attesa dati (GPS/Stamp)...")
            return

        if self.t_start is None and self.path_active:
            self.t_start = self.current_stamp
            rospy.loginfo("MISSIONE FASE 3 INIZIATA! Zeno in movimento.")

        event_msg = ""    

        # POSIZIONE NED
        current_n, current_e = ll2ne(self.origin, (self.current_lat, self.current_lon))

        if self.last_n is not None:
            self.total_distance += math.sqrt( (current_n - self.last_n)**2 + (current_e - self.last_e)**2 )

        self.last_n = current_n
        self.last_e = current_e



        # CHECK FINE MISSIONE
        if self.path_active and self.current_wp_idx >= len(self.waypoints) - 1:

            self.joystic_pub.publish(Rel_error_joystick(error_surge_speed=0.0, error_yaw=0.0))

            
            mission_time = (self.current_stamp - self.t_start).to_sec() if self.t_start else 0.0

            rospy.loginfo("==============================================")
            rospy.loginfo("MISSIONE FASE 3 COMPLETATA!")
            rospy.loginfo("Tempo di missione: %.2f secondi", mission_time)
            rospy.loginfo("Distanza totale percorsa: %.2f metri", self.total_distance)
            rospy.loginfo("==============================================")

            #Riepilogo fine missione: 
            try:
                if not self.csv_file.closed:
                    self.csv_writer.writerow([
                        "{:.2f}".format(mission_time), "", "", "", "", "", 
                        "{:.2f}".format(self.total_distance), "", "", "", 
                        "RIEPILOGO FASE 3: Missione completata! Distanza totale: {:.2f}m, Tempo: {:.2f}s".format(self.total_distance, mission_time)
                    ])
            except Exception as e:
                rospy.logwarn("Errore scrittura riepilogo: %s", str(e))


            self.path_active = False
            self.path_received = False

            #Reset parametri 
            self.total_distance = 0.0
            self.last_n = None
            self.last_e = None
            self.current_wp_idx = 0
            self.t_start  = None
            self.path_waiting_logged = False 
            return
   
        #CONTROLLO 
        wp_start = self.waypoints[self.current_wp_idx]
        wp_end = self.waypoints[self.current_wp_idx + 1]

        wp_start_n=wp_start[0]
        wp_start_e = wp_start[1]
        wp_end_n = wp_end[0]
        wp_end_e = wp_end[1]

        #definzione segmento
        dx = wp_end_n - wp_start_n
        dy = wp_end_e - wp_start_e
        segment_length = math.sqrt(dx**2 + dy**2)

        #vettore che unisce Zeno e wp_start
        rx = current_n - wp_start_n
        ry = current_e - wp_start_e

        yaw_path = math.atan2(dy,dx)   

        # proiezione sul segmento 
        t = (rx*dx + ry*dy) / (dx**2 + dy**2)
        t = max(0.0, min(1.0, t))

        proj_n = wp_start_n + t * dx
        proj_e = wp_start_e + t * dy

    
        # Errore Cross Track 
        error_track = (current_n - proj_n)*(-math.sin(yaw_path)) + (current_e - proj_e)*( math.cos(yaw_path))
       
        # Ricerca target nelle vicinanza ( Waypoint che hanno z=1)
        target_in_sight = False
        dist_to_special_target = 999.0
        
        # Scansione dei successivi 3 wp (se esistono)
        look_ahead_range = min(3, len(self.waypoints) - self.current_wp_idx)

        for i in range(look_ahead_range):
            wp = self.waypoints[self.current_wp_idx + i]
            if len(wp) > 2 and wp[2] in [1.0, 3.0]: 
                target_in_sight = True

                dist_to_special_target = math.sqrt((wp[0] - current_n)**2 + (wp[1] - current_e)**2)
                #rospy.loginfo_throttle(2.0, "Bersaglio rilevato in zona! Distanza attuale: %.2fm", dist_to_special_target)
                break

	wp_curva = self.waypoints[self.current_wp_idx]
	if wp_curva[2] in [-1.0, 3.0]:
	    new_sss_state = "OFF"
	else:
            new_sss_state = "ON"
	self.sss_state = new_sss_state

	# Pubblicazione su topic  (pubblichiamo il comando in continuo a 10Hz )
        self.sss_pub.publish(String(data=self.sss_state))
       
        L_standard = 1.0 + 1.0 * (1.0 - min(abs(error_track) / self.max_track_error, 1.0))
        
        if target_in_sight:
            #Target avvistati, si forza il passaggio vicino 
            L_target = max(0.3, dist_to_special_target * 0.5)
            
            L = min(L_standard, L_target)
        else:
            L = L_standard

        if abs(error_track) >= L:
            dl = 0.0
        else:
            dl = math.sqrt(L**2 - abs(error_track)**2)
            

        # PATH TRACKING (Scorrimento dei wp)
        distance_to_consume = dl
        lookahead_idx = self.current_wp_idx
        current_t = t
        
        while True:
            wp_s = self.waypoints[lookahead_idx]
            wp_e = self.waypoints[lookahead_idx + 1]
            
            dx_l = wp_e[0] - wp_s[0]
            dy_l = wp_e[1] - wp_s[1]
            seg_len = math.sqrt(dx_l**2 + dy_l**2)
            
            dist_available = (1.0 - current_t) * seg_len
            
            if distance_to_consume <= dist_available:
                final_t = current_t + (distance_to_consume / seg_len) if seg_len > 0.001 else 1.0
                target_n = wp_s[0] + final_t * dx_l
                target_e = wp_s[1] + final_t * dy_l
                break
            else:
                distance_to_consume -= dist_available
                lookahead_idx += 1
                
                if lookahead_idx >= len(self.waypoints) - 1:
                    target_n = self.waypoints[-1][0]
                    target_e = self.waypoints[-1][1]
                    break
                
                current_t = 0.0

        yaw_des = math.atan2(target_e - current_e, target_n - current_n)


        # Gestione fine path 
        dist_to_current_end = math.sqrt((wp_end_n - current_n)**2 + (wp_end_e - current_e)**2)
        is_target_node = (len(wp_end) > 2 and wp_end[2] in [1.0, 3.0])

        if is_target_node:
            current_position_th = 0.5 
            #rospy.loginfo_throttle(2.0, "Avvicinamento critico: Soglia d'ingaggio stretta a 0.5m")
        else:
            current_position_th = self.position_th

        if dist_to_current_end < current_position_th or t >= 1.0: 
            
            if is_target_node:
                event_msg = "BERSAGLIO RAGGIUNTO! Distanza di passaggio: {:.2f}m".format(dist_to_current_end)
                rospy.loginfo(">>> " + event_msg + " <<<")
            else:
                event_msg = "" 
            
            self.log_state(current_n, current_e, dist_to_current_end, error_track, 0.0, 0.0, event_msg)
            
            self.current_wp_idx += 1
            return


        # CALCOLO COMANDI 
        yaw_error = self.wrapToPi(yaw_des - self.current_yaw)
        
        max_heading = np.deg2rad(self.max_heading_deg)
        yaw_error = max(-max_heading, min(max_heading, yaw_error))
        yaw_error_deg = math.degrees(yaw_error)

        final_wp_n = self.waypoints[-1][0]
        final_wp_e = self.waypoints[-1][1]
        dist_to_mission_end = math.sqrt((final_wp_n - current_n)**2 + (final_wp_e - current_e)**2)

        heading_factor = 1 - min(abs(yaw_error_deg) / self.max_heading_deg, 1.0)
        distance_factor = min(dist_to_mission_end / self.distance_gain, 1.0) 
        track_factor = 1 - min(abs(error_track) / self.max_track_error, 1.0)

        surge_factor = min(heading_factor, distance_factor, track_factor)
        surge = self.max_surge_speed * surge_factor

        cmd = Rel_error_joystick()
        cmd.error_yaw = yaw_error_deg
        cmd.error_surge_speed = surge
        self.joystic_pub.publish(cmd)

        self.log_state(current_n, current_e, dist_to_current_end, error_track, surge, yaw_error_deg, event_msg)

    def log_state(self, current_n, current_e, dist, error_track, surge, yaw_error_deg, event_msg):
            
            telemetry_str = "X-Track: {:.2f}m | Surge: {:.2f} | YawErr: {:.1f} deg".format(
                error_track, surge, yaw_error_deg
            )
            
            if event_msg != "":
                telemetry_str += " | " + event_msg

            self.telemetry_pub.publish(String(data=telemetry_str))

            try:
                if not self.csv_file.closed:
                    current_time = (self.current_stamp - self.t_start).to_sec() if self.t_start else 0.0
                    
                    # ATTENZIONE: Nessuno spazio extra nelle parentesi graffe!
                    self.csv_writer.writerow([
                        "{:.2f}".format(current_time),
                        "{:.6f}".format(self.current_lat),
                        "{:.6f}".format(self.current_lon),
                        "{:.2f}".format(math.degrees(self.current_yaw)),
                        "{:.2f}".format(current_n),
                        "{:.2f}".format(current_e),
                        "{:.2f}".format(dist),
                        "{:.2f}".format(error_track),
                        "{:.2f}".format(surge),
                        "{:.2f}".format(yaw_error_deg),
                        event_msg
                    ])
            except Exception as e:
                # Ora se c'è un errore di sintassi, te lo stampa, invece di zittirlo!
                rospy.logwarn_throttle(2.0, "Errore scrittura CSV: %s", str(e))

    def wrapToPi(self, angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi
    
    def on_shutdown_hook(self):
        self.joystic_pub.publish(Rel_error_joystick(error_surge_speed=0.0, error_yaw=0.0))
        if not self.csv_file.closed:
            self.csv_file.close()
            rospy.loginfo("Path_ctr: Scatola Nera Fase 3 salvata con successo.")



if __name__ == "__main__":
    try:
        controller = WPController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
