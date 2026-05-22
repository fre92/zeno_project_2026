#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division
import rospy
import math
import random
import numpy as np

from marta_msgs.msg import NavStatus
from geodetic_functions import ll2ne

# --- IMPORT STANDARD ROS PER LA TRAIETTORIA ---
from zeno_python.msg import WaypointPath

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

# --- FUNZIONI GEOMETRICHE E CLASSI RRT (Rimaste Invariate) ---
def is_point_in_polygon(n, e, polygon):
    num = len(polygon)
    inside = False
    p1n, p1e = polygon[0]
    for i in range(num + 1):
        p2n, p2e = polygon[i % num]
        if e > min(p1e, p2e):
            if e <= max(p1e, p2e):
                if n <= max(p1n, p2n):
                    if p1e != p2e:
                        xints = (e - p1e) * (p2n - p1n) / (p2e - p1e) + p1n
                    if p1e == p2e or n <= xints:
                        inside = not inside
        p1n, p1e = p2n, p2e
    return inside

class Node:
    def __init__(self, n, e, yaw):
        self.n = n 
        self.e = e 
        self.yaw = yaw 
        self.parent = None

class KinodynamicRRT:
    def __init__(self, start, goal, polygon, obstacles, max_iter=3000): 
        self.start = Node(start[0], start[1], start[2])
        self.goal = Node(goal[0], goal[1], 0) 
        self.polygon = polygon       
        self.obstacles = obstacles   
        self.max_iter = max_iter
        self.node_list = [self.start]
        self.max_yaw_rate = math.radians(45.0)
        self.v = 0.20 
        self.dt = 3.0 

    def plan(self):
        for _ in range(self.max_iter):
            rnd_node = self.get_random_node()
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd_node)
            nearest_node = self.node_list[nearest_ind]
            new_node = self.steer(nearest_node, rnd_node)
            if self.check_collision(new_node):
                self.node_list.append(new_node)
                if math.hypot(new_node.n - self.goal.n, new_node.e - self.goal.e) <= 0.5:
                    return self.generate_final_course(len(self.node_list) - 1)
        return None

    def steer(self, from_node, to_node):
        target_yaw = math.atan2(to_node.e - from_node.e, to_node.n - from_node.n)
        yaw_error = target_yaw - from_node.yaw
        yaw_error = (yaw_error + math.pi) % (2 * math.pi) - math.pi
        yaw_error = max(-self.max_yaw_rate, min(self.max_yaw_rate, yaw_error))
        new_yaw = from_node.yaw + yaw_error
        new_n = from_node.n + self.v * math.cos(new_yaw) * self.dt
        new_e = from_node.e + self.v * math.sin(new_yaw) * self.dt
        new_node = Node(new_n, new_e, new_yaw)
        new_node.parent = from_node
        return new_node

    def check_collision(self, node):
        if not is_point_in_polygon(node.n, node.e, self.polygon):
            return False
        for (on, oe, r) in self.obstacles:
            if math.hypot(node.n - on, node.e - oe) <= r + 0.5: 
                return False
        return True

    def get_random_node(self):
        if random.randint(0, 100) < 10:
            return Node(self.goal.n, self.goal.e, 0)
        n_p = [p[0] for p in self.polygon]
        e_p = [p[1] for p in self.polygon]
        return Node(random.uniform(min(n_p), max(n_p)), random.uniform(min(e_p), max(e_p)), 0)

    def get_nearest_node_index(self, node_list, rnd_node):
        dlist = []
        for node in node_list:
            dist = math.hypot(node.n - rnd_node.n, node.e - rnd_node.e)
            angle_to_rnd = math.atan2(rnd_node.e - node.e, rnd_node.n - node.n)
            angle_diff = abs((angle_to_rnd - node.yaw + math.pi) % (2*math.pi) - math.pi)
            cost = dist + (angle_diff * 2.0) 
            dlist.append(cost)
        return dlist.index(min(dlist))

    def generate_final_course(self, last_index):
        path = []
        node = self.node_list[last_index]
        while node is not None:
            path.append([node.n, node.e, node.yaw]) 
            node = node.parent
        return path[::-1]

# --- NODO ROS PLANNER ---
class ZenoPlanner:
    def __init__(self):
        rospy.init_node('zeno_rrt_planner', anonymous=True)
        
        # Publisher del Path (latch=True permette al controllore di leggerlo anche se avviato dopo)
        self.path_pub = rospy.Publisher('/planned_path', Path, queue_size=1, latch=True)
        self.Subscriber = rospy.Subscriber('/nav_status', NavStatus, self.nav_callback)
        
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
        while not rospy.is_shutdown() and self.current_lat is None:
            self.rate.sleep()
        rospy.loginfo("Segnale acquisito. Inizio pianificazione RRT...")

    def execute_planning(self):
        self.wait_for_sensors()
        
        polygon = [[-5, -5], [25, -2], [37, 20], [18, 32], [-4, 15]]
        targets_ne = [[2, 2], [5, 18], [15, 15], [20, 12], [28, 22]]
        obstacles = [(12, 8, 3.0), (25, 10, 3.0), (10, 22, 3.0), (0, 5, 3.0), (22, 20, 3.0)]
        
        origin_ll = (self.origin_lat, self.origin_lon)
        current_ll = (self.current_lat, self.current_lon)
        curr_n, curr_e = ll2ne(origin_ll, current_ll)
        
        current_start = [curr_n, curr_e, self.current_yaw]
        full_path = []
        
        for i, target in enumerate(targets_ne):
            rospy.loginfo("Pianificazione Target %d/%d...", i+1, len(targets_ne))
            rrt = KinodynamicRRT(current_start, target, polygon, obstacles)
            path_segment = rrt.plan()
            
            if not path_segment:
                rospy.logerr("Impossibile raggiungere il target %d.", i+1)
                return
            
            if full_path:
                full_path.extend(path_segment[1:]) 
            else:
                full_path.extend(path_segment)
                
            current_start = path_segment[-1] 
            
        rospy.loginfo("Percorso completo trovato con %d waypoint totali.", len(full_path))
        
        # --- CREAZIONE E PUBBLICAZIONE MESSAGGIO PATH ---
        ros_path = Path()
        ros_path.header.stamp = rospy.Time.now()
        ros_path.header.frame_id = "ned"
        
        for wp in full_path:
            pose = PoseStamped()
            pose.pose.position.x = wp[0] # Nord
            pose.pose.position.y = wp[1] # Est
            # Salviamo lo yaw nella coordinata Z (trucco per passare anche l'orientazione se serve, altrimenti puoi ometterlo)
            pose.pose.position.z = wp[2] 
            ros_path.poses.append(pose)
            
        self.path_pub.publish(ros_path)
        rospy.loginfo("Traiettoria pubblicata sul topic /planned_path !")

if __name__ == '__main__':
    try:
        planner = ZenoPlanner()
        planner.execute_planning()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
