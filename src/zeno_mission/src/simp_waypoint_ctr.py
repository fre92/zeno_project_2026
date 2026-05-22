#!/usr/bin/env python

import rospy
import math

import  numpy as np


from marta_msgs.msg import NavStatus
from joystick_command.msg import Rel_error_joystick

from geodetic_functions import ll2ne


#VARIABILI GLOBALI 
current_lat = None
current_lon = None
current_yaw = None

origin= None 

waypoints=[]
current_wp_idx=0

total_distance=0.0
last_n = None
last_e = None

pub = None 

#Parametri controllore 
position_th= 0.25 # a position_th metri di soglia, poi si passa al prossimo 
th_surge_speed=2
max_surge_speed=0.2

def wrapToPi(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


#definizione callback, cosa faccio quando arriva un messaggio sul topic 
def nav_callback(msg): 
    global current_lat
    global current_lon
    global current_yaw

    current_lat = msg.position.latitude
    current_lon=msg.position.longitude
    current_yaw=msg.orientation.yaw

def control_loop(event):
    #si definisce separata la logica del controllore, questo perche' la callback parte quando arriva un msg mentre il controllore devi girare con frequenza regolare     
    global current_wp_idx
    global total_distance, last_n, last_e
    global position_th

    #if current_wp_idx in [0, len(waypoints)-1]:
    #    position_th = 0.25
    #else:
    #    position_th = 2.0 


    if current_lat is None:
        return 
    if current_lon is None: 
        return

    if current_yaw is None:
        return    

    if current_wp_idx >= len(waypoints):
        
        stop_msg = Rel_error_joystick()
        pub.publish(stop_msg)
        
        t_end = rospy.Time.now()
        mission_time = (t_end - t_start).to_sec()

        rospy.loginfo("Mission completed")
        rospy.loginfo("Mission time = %.2f seconds", mission_time)
        rospy.loginfo("Total distance travelled = %.2f m", total_distance)

        rospy.signal_shutdown("Mission completed")
            
        return #punti finiti    
    
   
    #posizione currente in NED 
    current_ne=ll2ne(origin,(current_lat,current_lon))
    current_n=current_ne[0]
    current_e=current_ne[1]

    if last_n is not None:
        step_dist = math.sqrt((current_n - last_n)**2 + (current_e - last_e)**2)
        total_distance += step_dist

    last_n = current_n
    last_e = current_e


    
    #posizione target (sono gia' in NED)
    wp = waypoints[current_wp_idx]
    target_n=wp[0]
    target_e=wp[1]

   

    #calcolo dell'errore 
    dn= target_n - current_n
    de=target_e - current_e 

    #distanza (euclide) dal target 
    distance= math.sqrt(dn**2 + de**2)
    rospy.loginfo("Distance to WP %d = %.2f m",
            current_wp_idx+1,
            distance)

    #check se il punto e' stato raggiunto (siamo sufficientemente vicini )
    if distance < position_th: 
        rospy.loginfo("Waypoint reached")        
        current_wp_idx += 1
        return 
    

    #controllo vero e proprio

    yaw_des=math.atan2(de,dn)    

    yaw_error= wrapToPi(yaw_des - current_yaw)
    yaw_error_deg=math.degrees(yaw_error)

    #check se con la richiesta sono arrivato a saturazione 
    if yaw_error_deg > 45:
        yaw_error_deg = 45

    if yaw_error_deg < -45:
        yaw_error_deg = -45

    #formo comando da dare 
    cmd=Rel_error_joystick()
    cmd.error_yaw=yaw_error_deg

    #controllo velocita'
    heading_factor = 1 - min(abs(yaw_error_deg) / 45.0, 1.0) #quanto sbagliato in direzione -> piu' sei storto piu' rallenti per poi girare 
    distance_factor = min(distance / th_surge_speed, 1.0) #quanto sei distante -> piu' sei vicino piu' rallenti
    cmd.error_surge_speed = max_surge_speed * heading_factor * distance_factor
    
    pub.publish(cmd)


#MAIN 
if __name__ == "__main__":

    rospy.init_node("simp_wp_controller")


    #origine NED 
    lat0 = rospy.get_param("/origin/coordinates/latitude")
    lon0 = rospy.get_param("/origin/coordinates/longitude")

    origin=(lat0,lon0)

    #waypoint da file params.yamal
    #waypoints = rospy.get_param("/waypoints22")
    waypoints= [[11.0, 32.0], [21.0, 33.0], [21.0, 2.0]]

    rospy.Subscriber("/nav_status" , NavStatus , nav_callback)
    pub = rospy.Publisher("/relative_error" , Rel_error_joystick , queue_size=1)

    
    t_start = rospy.Time.now()    

    timer = rospy.Timer(rospy.Duration(0.1), control_loop) #thread temporizzato che chiama control_loop(event) ogni 0.1 secondi cioe' 10Hz
    rospy.on_shutdown(timer.shutdown)
            

    rospy.spin() #blocca il main thread 



