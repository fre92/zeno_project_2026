#!/usr/bin/env python

import rospy
import math
import numpy as np

from marta_msgs.msg import NavStatus
from joystick_command.msg import Rel_error_joystick
from geodetic_functions import ll2ne

from zeno_python.msg import WaypointPath


class WPController:

    def __init__(self):

        rospy.init_node("cross_track_error_controller")

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
        self.position_th = 0.25
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
            self.waypoints.append((p.x, p.y))

        self.waypoints = np.array(self.waypoints)    

        if None not in (self.current_lat, self.current_lon, self.origin):

            current_n, current_e = ll2ne(self.origin, (self.current_lat, self.current_lon))
            
            # distanza euclidea dal primo wp
            dist_to_start = math.sqrt((self.waypoints[0][0] - current_n)**2 + (self.waypoints[0][1] - current_e)**2)
            
            if dist_to_start > 1.0:
                rospy.loginfo("Zeno si trova a %.2f metri dal primo WP.", dist_to_start)
                
                self.waypoints = np.vstack([[current_n, current_e], self.waypoints])
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


        if self.mode == "GOTO_FIRST_WP":
            
            #MODE:"GOTO_FIRST_WP" 
            wp0_n, wp0_e = self.waypoints[0]

            dn = wp0_n - current_n
            de = wp0_e - current_e

            dist0 = math.sqrt(dn**2 + de**2)
            rospy.loginfo("Distance to WP %d = %.2f m", self.current_wp_idx+1,dist0)

            # HEADING CONTROL
            yaw_des = math.atan2(de, dn)
            yaw_error = self.wrapToPi(yaw_des - self.current_yaw)

            #saturation
            max_heading = np.deg2rad(self.max_heading_deg)
            yaw_error_sat = max(-max_heading, min(max_heading, yaw_error))

            # rad to deg
            yaw_error_deg = math.degrees(yaw_error_sat)

            # SPEED CONTROL
            heading_factor = 1 - min(abs(yaw_error_deg) / self.max_heading_deg, 1.0)
            distance_factor = min(dist0 / self.distance_gain, 1.0)
      
            surge = self.max_surge_speed * heading_factor * distance_factor
            surge = max(0.05, surge)

            # ARRIVATO AL PRIMO WP
            if dist0 < self.position_th:
                rospy.loginfo("Waypoint %d reached-> switching to TRACK mode", self.current_wp_idx + 1)
                self.mode = "TRACK_PATH"
                self.current_wp_idx = 0
                return

            cmd = Rel_error_joystick()
            cmd.error_yaw = yaw_error_deg
            cmd.error_surge_speed = surge
            self.joystic_pub.publish(cmd)


            return

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

        wp_start_n, wp_start_e = wp_start
        wp_end_n, wp_end_e = wp_end

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
        L = 2.0 + 1.5 * (1.0 - min(abs(error_track) / self.max_track_error, 1.0))

        # PER L'RRT (punti a 60cm)
        #L = 0.4 + 0.9 * (1.0 - min(abs(error_track) / self.max_track_error, 1.0))

        if abs(error_track) >= L:
            #siamo molto lontani, puntiamo alla proiezione 
            dl = 0.0
        else:
            # Teorema di Pitagora per trovare la distanza lungo il percorso
            dl = math.sqrt(L**2 - abs(error_track)**2)

        t_lookahead = t + (dl / segment_length)    
        
        t_lookahead = min(1.0, t_lookahead) # limito a 1.0 per evitare di puntare oltre il wp di fine segmento 

        #Coordinate Nord e EST del Look-Ahead point 
        target_n = wp_start_n + t_lookahead * dx
        target_e = wp_start_e + t_lookahead * dy

        #Direzione Prua verso il Virtual Poit 
        yaw_des = math.atan2(target_e - current_e, target_n - current_n)


        # DISTANCE TO END WP
        dist = math.sqrt((wp_end_n - current_n)**2 + (wp_end_e - current_e)**2)

        #rospy.loginfo("Distance to WP %d = %.2f m", self.current_wp_idx+2, dist)
        rospy.loginfo("Distance to WP = %.2f m", dist)

        if dist < self.position_th or t >= 1.0: #se t==1 significa che il veicolo ha geometricamente suparato la fine del segmento anche se magari non e' entrato nel raggio previsto
            rospy.loginfo("Waypoint reached")
            self.current_wp_idx += 1
            return

        
        # HEADING CONTROL
        yaw_error = self.wrapToPi(yaw_des - self.current_yaw)
        
        #saturation
        max_heading = np.deg2rad(self.max_heading_deg)
        yaw_error = max(-max_heading, min(max_heading, yaw_error))

        #rad to deg
        yaw_error_deg = math.degrees(yaw_error)

        # SPEED CONTROL
        heading_factor = 1 - min(abs(yaw_error_deg) / self.max_heading_deg, 1.0)
        distance_factor = min(dist / self.distance_gain, 1.0)
        track_factor = 1 - min(abs(error_track) / self.max_track_error, 1.0)

        surge_factor = min(heading_factor, distance_factor, track_factor)
        

        surge = self.max_surge_speed * surge_factor
        #surge = max(0.05, surge)

        # PUBLISH
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

