#!/usr/bin/env python


import rospy
import yaml 
from marta_msgs.msg import NavStatus

# variabili globali (stato del robot)
lat = 0.0
lon = 0.0
depth = 0.0
yaw = 0.0
vx = 0.0
vy = 0.0
vz = 0.0


def callback(msg):

    global lat, lon, depth, yaw, vx, vy, vz

    # posizione
    lat = msg.position.latitude
    lon = msg.position.longitude
    depth = msg.position.depth 

    # orientazione
    yaw = msg.orientation.yaw

    # velocita' (NED)
    vx = msg.ned_speed.x
    vy = msg.ned_speed.y
    vz = msg.ned_speed.z

    data = {
        "latitude": lat,
        "longitude": lon,
        "depth": depth,
        "yaw": yaw,
        "vx": vx,
        "vy": vy,
        "vz": vz
    }
    
    #Salva YAML
    with open("/home/student/catkin_ws/src/zeno_python/nav_status.yaml", "a") as file: #append
    yaml.dump([data], file)

    # stampa per debug
    rospy.loginfo("Lat: %.6f Long: %.6f Depth: %.2f Yaw: %.2f",
                  lat, lon, depth, yaw)


def listener():

    rospy.init_node('nav_reader', anonymous=True)

    rospy.Subscriber("/nav_status", NavStatus, callback)

    rospy.spin()


if __name__ == '__main__':
    listener()