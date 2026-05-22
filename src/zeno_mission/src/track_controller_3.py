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

        rospy.init_node("track_controller_1")

        # Stato
        self.current_lat = None
        self.current_lon = None
        self.current_yaw = None

        self.origin = None
        self.waypoints = rospy.get_param("/waypoints_RRT")
        self.current_wp_idx = 0

        self.total_distance = 0.0
        self.last_n = None
        self.last_e = None
        self.mode = "GOTO_FIRST_WP"

        self.t_start = rospy.Time.now()

        
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

        self.timer = rospy.Timer(
            rospy.Duration(0.1), #control Loop a 10Hz
            self.control_loop
        )

        rospy.on_shutdown(self.timer.shutdown)

        # origin
        lat0 = rospy.get_param("/origin/coordinates/latitude")
        lon0 = rospy.get_param("/origin/coordinates/longitude")
        self.origin = (lat0, lon0)

    
    def nav_callback(self, msg):
        self.current_lat = msg.position.latitude
        self.current_lon = msg.position.longitude
        self.current_yaw = msg.orientation.yaw

   
    def control_loop(self, event):

        if None in (self.current_lat, self.current_lon, self.current_yaw):
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
          

            # ARRIVATO AL PRIMO WP
            if dist0 < self.position_th:
                rospy.loginfo("Waypoint %d reached-> switching to TRACK mode", self.current_wp_idx + 1)
                self.mode = "TRACK_PATH"
                self.current_wp_idx += 1
                return

            cmd = Rel_error_joystick()
            cmd.error_yaw = yaw_error_deg
            cmd.error_surge_speed = surge
            self.joystic_pub.publish(cmd)


            return

        #MODE:"TRACK_PATH" 

        # CHECK FINE MISSIONE
        if self.current_wp_idx >= len(self.waypoints) - 1:

            stop_msg = Rel_error_joystick()
            self.joystic_pub.publish(stop_msg)

            t_end = rospy.Time.now()
            mission_time = (t_end - self.t_start).to_sec()

            rospy.loginfo("Mission completed")
            rospy.loginfo("Mission time = %.2f seconds", mission_time)
            rospy.loginfo("Total distance = %.2f m", self.total_distance)

            rospy.signal_shutdown("Mission completed")
            return
   

 
        # WAYPOINT SEGMENT & YAW PATH
        wp_start = self.waypoints[self.current_wp_idx-1]
        wp_end = self.waypoints[self.current_wp_idx]

        wp_start_n, wp_start_e = wp_start
        wp_end_n, wp_end_e = wp_end

        yaw_path = math.atan2((wp_end_e - wp_start_e ),(wp_end_n - wp_start_n ))   

    
        # CROSS TRACK ERROR
        error_track= ( - (current_n - wp_start_n)*math.sin(yaw_path) + (current_e - wp_start_e)*math.cos(yaw_path) )

        yaw_des = yaw_path - math.atan(self.K * error_track)

        # DISTANCE TO END WP
        dist = math.sqrt((wp_end_n - current_n)**2 + (wp_end_e - current_e)**2)

        rospy.loginfo("Distance to WP %d = %.2f m", self.current_wp_idx+1, dist)

        if dist < self.position_th:
            rospy.loginfo("Waypoint %d reached", self.current_wp_idx + 1)
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

        surge = self.max_surge_speed * heading_factor * distance_factor * track_factor
        

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
        rospy.spin()

    except rospy.ROSInterruptException:
        pass