#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import numpy as np

from marta_msgs.msg import NavStatus
from joystick_command.msg import Rel_error_joystick
from geodetic_functions import ll2ne

from zeno_python.msg import WaypointPath


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

        self.waypoints = []
        self.current_wp_idx = 0

        self.path_received = False
        self.path_active = False
        self.path_waiting_logged = False

        self.mission_id = 0

        self.total_distance = 0.0
        self.last_n = None
        self.last_e = None
        self.mode = "TRACK_PATH"

        self.t_start = None
        
        
        # Parametri
        self.position_th = 1
        self.max_surge_speed = 0.2

        self.K = 0.7
        self.max_track_error = 3.0

        self.max_heading_deg = 45.0
        self.distance_gain = 2.0


        # ROS I/O
        self.joystic_pub = rospy.Publisher(
            "/relative_error",
            Rel_error_joystick,
            queue_size=1
        )

        rospy.Subscriber(
            "/nav_status",
            NavStatus,
            self.nav_callback
        )

        rospy.Subscriber(
            "/waypoint_path",
            WaypointPath,
            self.path_callback
        )

        self.timer = rospy.Timer(
            rospy.Duration(0.1), #control Loop a 10Hz
            self.control_loop
        )

        rospy.on_shutdown(self.timer.shutdown)


    
    def nav_callback(self, msg):
        self.current_lat = msg.position.latitude
        self.current_lon = msg.position.longitude
        self.current_yaw = msg.orientation.yaw
    
    def path_callback(self, msg):

        if self.path_active:
            return

        rospy.loginfo("New waypoint path received")

        self.waypoints = []


        for p in msg.waypoints:
            self.waypoints.append((p.x, p.y , p.z))

        self.waypoints = np.array(self.waypoints)    

        if None not in (self.current_lat, self.current_lon, self.origin):

            current_n, current_e = ll2ne(self.origin, (self.current_lat, self.current_lon))
            
            # distanza euclidea dal primo wp
            dist_to_start = math.sqrt((self.waypoints[0][0] - current_n)**2 + (self.waypoints[0][1] - current_e)**2)
            
            if dist_to_start > 1.0:
                rospy.loginfo("Zeno si trova a %.2f metri dal primo WP.", dist_to_start)
                
                self.waypoints = np.vstack([[current_n, current_e , 0], self.waypoints])
        else:
            rospy.logwarn("Dati GPS o Origine non ancora pronti.")    

        self.path_received = True
        self.path_active = True
        self.mission_id += 1
        self.t_start = rospy.Time.now()

       

   
    def control_loop(self, event):

        if not self.path_received:
            if not self.path_waiting_logged:
                rospy.loginfo("Waiting for waypoint path...")
                self.path_waiting_logged = True # Blocco stampe successive
            return

        if None in (self.current_lat, self.current_lon, self.current_yaw, self.origin):
            rospy.loginfo_throttle(5.0, "Waiting for data")
            return

        # POSIZIONE NED
        current_n, current_e = ll2ne(self.origin, (self.current_lat, self.current_lon))

        if self.last_n is not None:
            self.total_distance += math.sqrt( (current_n - self.last_n)**2 + (current_e - self.last_e)**2 )

        self.last_n = current_n
        self.last_e = current_e


        #MODE:"TRACK_PATH" 

        # CHECK FINE MISSIONE
        if self.path_active and self.current_wp_idx >= len(self.waypoints) - 1:

            stop_msg = Rel_error_joystick()
            self.joystic_pub.publish(stop_msg)

            t_end = rospy.Time.now()
            mission_time = (t_end - self.t_start).to_sec()

            rospy.loginfo("Mission completed")
            rospy.loginfo("Mission time = %.2f seconds", mission_time)
            rospy.loginfo("Total distance = %.2f m", self.total_distance)

            self.path_active = False
            self.path_received = False

             #Reset parametri 
            self.total_distance = 0.0
            self.last_n = None
            self.last_e = None
            self.current_wp_idx = 0
            self.mode = "GOTO_FIRST_WP"
            self.t_start  = None
            self.path_waiting_logged = False 
            
            return
   

 
        # WAYPOINT SEGMENT & YAW PATH
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

        # projection on segment
        t = (rx*dx + ry*dy) / (dx**2 + dy**2)
        t = max(0.0, min(1.0, t))

        proj_n = wp_start_n + t * dx
        proj_e = wp_start_e + t * dy

    
        # CROSS TRACK 
        error_track = (current_n - proj_n)*(-math.sin(yaw_path)) + (current_e - proj_e)*( math.cos(yaw_path))

        #Look-ahead di base di 2 metri, che si allunga fino a 3.5 metri se siamo speculari alla linea
        #L = 2.0 + 1.5 * (1.0 - min(abs(error_track) / self.max_track_error, 1.0))

        # PER L'RRT: L oscilla tra 1.0 metri (errore grande) e 2.0 metri (errore nullo)
        L = 1.0 + 1.0 * (1.0 - min(abs(error_track) / self.max_track_error, 1.0))

       # CROSS TRACK 
        error_track = (current_n - proj_n)*(-math.sin(yaw_path)) + (current_e - proj_e)*( math.cos(yaw_path))

        # ==========================================================
        # 1. RICERCA DEI TARGET SPECIALI NELLE VICINANZE (Hard Constraint)
        # ==========================================================
        target_in_sight = False
        dist_to_special_target = 999.0
        
        # Scansioniamo i prossimi 3 waypoint (se esistono) per cercare z=1
        look_ahead_range = min(3, len(self.waypoints) - self.current_wp_idx)
        for i in range(look_ahead_range):
            wp = self.waypoints[self.current_wp_idx + i]
            # wp e' una riga di NumPy: [North, East, z_flag]
            if len(wp) > 2 and wp[2] == 1.0: 
                target_in_sight = True
                rospy.loginfo("Trovato bersaglio speciale")
                # Calcoliamo la distanza fisica da Zeno al bersaglio speciale
                dist_to_special_target = math.sqrt((wp[0] - current_n)**2 + (wp[1] - current_e)**2)
                break

        # ==========================================================
        # 2. CALCOLO DEL LOOK-AHEAD DINAMICO (L)
        # ==========================================================
        # Formula base RRT (ammorbidita e veloce)
        L_base = 1.0 + 1.0 * (1.0 - min(abs(error_track) / self.max_track_error, 1.0))
        
        if target_in_sight:
            # Allarme Target! Stringiamo L in modo proporzionale per frenare l'Index Rolling
            # Il minimo assoluto è 0.3m per forzare il passaggio millimetrico
            L_target = max(0.3, dist_to_special_target * 0.5)
            # Prendiamo il valore più severo (restrittivo) tra i due
            L = min(L_base, L_target)
        else:
            L = L_base

        if abs(error_track) >= L:
            # siamo molto lontani, puntiamo alla proiezione 
            dl = 0.0
        else:
            # Teorema di Pitagora per trovare la distanza lungo il percorso
            dl = math.sqrt(L**2 - abs(error_track)**2)

        # ==========================================================
        # 3. PATH TRACKING (Index Rolling)
        # ==========================================================
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

        # ==========================================================
        # 4. GESTIONE AVANZAMENTO FISICO SUL PERCORSO & SOGLIA
        # ==========================================================
        dist_to_current_end = math.sqrt((wp_end_n - current_n)**2 + (wp_end_e - current_e)**2)

        # Override della soglia: se il wp da superare è un target (z=1), esigi altissima precisione!
        if len(wp_end) > 2 and wp_end[2] == 1.0:
            current_position_th = 0.5 # Passaggio quasi obbligato
            rospy.loginfo("PASSAGGIO OBLLIGAto A 0.5")
        else:
            current_position_th = self.position_th # Tolleranza morbida standard

        if dist_to_current_end < current_position_th or t >= 1.0: 
            rospy.loginfo("Segmento %d completato.", self.current_wp_idx)
            self.current_wp_idx += 1
            return

        # ==========================================================
        # 5. HEADING & SPEED CONTROL (Invariato)
        # ==========================================================
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

    def wrapToPi(self, angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi


if __name__ == "__main__":

    try:
        controller = WPController()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()
    except rospy.ROSInterruptException:
        pass

