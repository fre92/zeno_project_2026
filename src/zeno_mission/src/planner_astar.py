#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import math
import heapq
import numpy as np
import matplotlib.path as mpltPath
import time
import json
from datetime import datetime
import os
import rospkg


# Importiamo i messaggi standard e custom
from geometry_msgs.msg import Point
# Controlla che ci sia questa riga esatta in cima al file:
from marta_msgs.msg import NavStatus
from zeno_mission.msg import WaypointPath
from geodetic_functions import ll2ne
from itertools import permutations

# --- PARAMETRI ---
RESOLUTION = 0.50  
EXTRA_SAFETY_MARGIN = 0.5 # Margine extra di sicurezza

class Node:
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position
        self.g = 0
        self.h = 0
        self.f = 0
    def __eq__(self, other):
        return self.position == other.position
    def __lt__(self, other):
        return self.f < other.f

def astar(grid, start, end):
    start_node = Node(None, start)
    end_node = Node(None, end)
    open_list = []
    closed_set = set()
    heapq.heappush(open_list, (start_node.f, start_node))
    neighbors = [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]

    TURN_PENALTY = 1.0 

    while open_list:
        current_node = heapq.heappop(open_list)[1]
        
        if current_node.position in closed_set:
            continue
        closed_set.add(current_node.position)

        # L'algoritmo si ferma SOLO quando tocca esattamente la cella finale
        if current_node.position == end_node.position:
            path = []
            current = current_node
            is_target_node = True  # Flag per il primo nodo (che in realtà è l'ultimo della rotta)
            
            while current is not None:
                if is_target_node:
                    # Assegniamo Z=1.0 all'indice della griglia del target
                    path.append((current.position[0], current.position[1], 1.0))
                    is_target_node = False
                else:
                    # Z=0.0 per tutto il resto del percorso
                    path.append((current.position[0], current.position[1], 0.0))
                    
                current = current.parent
                
            return path[::-1] # Ribaltando la lista, lo Z=1.0 finirà in fondo!

        for new_position in neighbors:
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])
            
            if node_position[0] >= len(grid) or node_position[0] < 0 or \
               node_position[1] >= len(grid[0]) or node_position[1] < 0:
                continue
            if grid[node_position[0]][node_position[1]] == 1:
                continue
            if node_position in closed_set:
                continue
                
            new_node = Node(current_node, node_position)
            step_cost = 1.414 if new_position[0] != 0 and new_position[1] != 0 else 1.0

            # Leggiamo il costo di geofencing della cella di arrivo (sarà 0 se dentro, 50 se fuori)
            geofence_cost = grid[node_position[0]][node_position[1]]
            
            turn_cost = 0.0
            if current_node.parent is not None:
                prev_dir = (current_node.position[0] - current_node.parent.position[0],
                            current_node.position[1] - current_node.parent.position[1])
                if prev_dir != new_position:
                    turn_cost = TURN_PENALTY

            new_node.g = current_node.g + step_cost + turn_cost + geofence_cost
            new_node.h = math.sqrt((new_node.position[0] - end_node.position[0])**2 + (new_node.position[1] - end_node.position[1])**2)
            new_node.f = new_node.g + new_node.h
            
            heapq.heappush(open_list, (new_node.f, new_node))
            
    return None

def create_occupancy_grid(obstacles_ned, poly_ned, all_points_ned):
    all_n = [p[0] for p in all_points_ned]
    all_e = [p[1] for p in all_points_ned]
    
    min_n, max_n = min(all_n) - 5.0, max(all_n) + 5.0
    min_e, max_e = min(all_e) - 5.0, max(all_e) + 5.0

    rows, cols = int((max_n - min_n) / RESOLUTION) + 1, int((max_e - min_e) / RESOLUTION) + 1
    grid = np.zeros((rows, cols))

    # 1. Creazione ostacoli cilindrici
    for obs in obstacles_ned:
        obs_n, obs_e, obs_r = obs[0], obs[1], obs[2]
        inflation_cells = int((obs_r + EXTRA_SAFETY_MARGIN) / RESOLUTION)
        obs_r_idx, obs_c_idx = int((obs_n - min_n) / RESOLUTION), int((obs_e - min_e) / RESOLUTION)
        
        for r in range(max(0, obs_r_idx - inflation_cells), min(rows, obs_r_idx + inflation_cells + 1)):
            for c in range(max(0, obs_c_idx - inflation_cells), min(cols, obs_c_idx + inflation_cells + 1)):
                if math.sqrt((r - obs_r_idx)**2 + (c - obs_c_idx)**2) <= inflation_cells:
                    grid[r][c] = 1

    # 2. Chiusura Poligono (Tutto ciò che è fuori è ostacolo)
    if poly_ned:
        poly_path = mpltPath.Path(poly_ned)
        for r in range(rows):
            for c in range(cols):
                n_real = r * RESOLUTION + min_n
                e_real = c * RESOLUTION + min_e
                if not poly_path.contains_point((n_real, e_real)):
                    grid[r][c] = 10000
                    
    return grid.tolist(), min_n, min_e

def calculate_path_length(segment, resolution):
    """Calcola la lunghezza in metri di un percorso A* sulla griglia."""
    if not segment or len(segment) < 2:
        return float('inf') # Se l'A* non trova strada, costo infinito
    
    dist_totale = 0.0
    for k in range(1, len(segment)):
        dn = segment[k][0] - segment[k-1][0]
        de = segment[k][1] - segment[k-1][1]
        dist_totale += math.sqrt(dn**2 + de**2) * resolution
    return dist_totale

def solve_tsp(start_ned, targets_ned, grid, resolution, min_n, min_e):
    """
    Risolutore TSP Avanzato: calcola la matrice dei costi esatti 
    usando l'A* per aggirare gli ostacoli prima di ordinare i target.
    """
    import itertools
    
    # 1. Combiniamo i punti: Indice 0 = Zeno, Indici 1..N = Target
    all_points = [start_ned] + targets_ned
    num_points = len(all_points)
    
    # 2. Inizializziamo la Matrice dei Costi
    cost_matrix = [[0.0] * num_points for _ in range(num_points)]
    
    # 3. Calcoliamo i percorsi A* per ogni coppia possibile
    for i in range(num_points):
        for j in range(i + 1, num_points):
            # Convertiamo le coordinate in indici griglia
            start_idx = (int((all_points[i][0] - min_n) / resolution), int((all_points[i][1] - min_e) / resolution))
            target_idx = (int((all_points[j][0] - min_n) / resolution), int((all_points[j][1] - min_e) / resolution))
            
            # Lanciamo l'A* tra questi due punti
            segment = astar(grid, start_idx, target_idx)
            
            # Misuriamo i metri esatti evitando gli ostacoli
            real_dist = calculate_path_length(segment, resolution)
            
            # La matrice è simmetrica: la distanza A->B è uguale a B->A
            cost_matrix[i][j] = real_dist
            cost_matrix[j][i] = real_dist

    # 4. Applichiamo la forza bruta (TSP) sulla VERA matrice dei costi
    best_cost = float('inf')
    best_order_indices = None
    
    # Creiamo permutazioni solo per i target (indici da 1 a num_points-1)
    target_indices = list(range(1, num_points))
    for perm in itertools.permutations(target_indices):
        current_cost = 0.0
        current_node = 0 # Partiamo sempre da Zeno (Indice 0)
        
        for next_node in perm:
            current_cost += cost_matrix[current_node][next_node]
            current_node = next_node
            
        if current_cost < best_cost:
            best_cost = current_cost
            best_order_indices = perm
            
    # 5. Ricostruiamo la lista dei target ordinati con la sequenza vincente
    ordered_targets = [all_points[idx] for idx in best_order_indices]
    
    rospy.loginfo("TSP Risolto! Distanza reale prevista: %.2f m", best_cost)
    return ordered_targets, best_cost

def main():
    rospy.init_node('planner_astar')
    
    waypoint_pub = rospy.Publisher("/waypoint_path", WaypointPath, queue_size=1, latch=True)
    
    # 1. Lettura Origine
    try:
        lat0 = rospy.get_param("/origin/coordinates/latitude")
        lon0 = rospy.get_param("/origin/coordinates/longitude")
        origin = (lat0, lon0)
    except KeyError:
        rospy.logerr("Origine non trovata nel parameter server!")
        return

# 2. Lettura parametri gara (in coordinate GPS) e conversione in NED
    try:
        targets_gps = rospy.get_param("/targets")
        obstacles_gps = rospy.get_param("/obstacles")
        
        # Inizializziamo le liste vuote per il sistema NED
        targets_ned = []
        obstacles_ned = []
        
        # Conversione dei target
        for t in targets_gps:
            lat = t[0]
            lon = t[1]
            n, e = ll2ne(origin, (lat, lon))
            targets_ned.append([n, e])
            
        # Conversione degli ostacoli (mantenendo il raggio)
        for obs in obstacles_gps:
            lat = obs[0]
            lon = obs[1]
            radius = obs[2]
            n, e = ll2ne(origin, (lat, lon))
            obstacles_ned.append([n, e, radius])
            
    except KeyError:
        rospy.logerr("File YAML di targets o ostacoli non caricati!")
        return

    poly_ned = []
    # =========================================================
    # 3. Lettura Poligoni (GIÀ IN NED) e Geofencing
    # =========================================================
    poly_original_ned = []
    poly_restricted_ned = []
    
    # 3a. Lettura Poligono Originale in metri
    try:
        poly_original_ned = rospy.get_param("/polygon_vertices/original")
    except KeyError:
        rospy.logwarn("Parametro /polygon_vertices/original non trovato.")

    # 3b. Lettura Poligono Ristretto in metri
    try:
        poly_restricted_ned = rospy.get_param("/polygon_vertices/restricted")
    except KeyError:
        rospy.logwarn("Parametro /polygon_vertices/restricted non trovato.")

    # 3c. Logica decisionale: Controllo Target e Scelta Poligono
    if poly_original_ned and poly_restricted_ned and targets_ned:
        path_orig = mpltPath.Path(poly_original_ned)
        path_restr = mpltPath.Path(poly_restricted_ned)

        valid_targets = []
        usa_originale = False

        for i, t in enumerate(targets_ned):
            # t[0]=Nord, t[1]=Est
            in_restr = path_restr.contains_point((t[0], t[1]))
            in_orig = path_orig.contains_point((t[0], t[1]))

            if in_restr:
                valid_targets.append(t)
            elif in_orig:
                valid_targets.append(t)
                usa_originale = True
                
            else:
                rospy.logerr("ATTENZIONE: Il Target %d è FUORI dall'area di gara! SCARTATO per sicurezza.", i+1)

        # Sovrascriviamo la lista dei target ignorando quelli scartati
        targets_ned = valid_targets

        # Scegliamo quale maschera passare alla creazione della griglia
        if usa_originale:
            poly_ned = poly_original_ned
            rospy.loginfo("Area di navigazione impostata su: ORIGINALE (Massima estensione).")
        else:
            poly_ned = poly_restricted_ned
            rospy.loginfo("Area di navigazione impostata su: RISTRETTA (Tutti i target in comfort zone).")

    else:
        # Fallback se manca un poligono
        poly_ned = poly_restricted_ned if poly_restricted_ned else poly_original_ned
        rospy.logwarn("Geofencing disabilitato: Manca uno dei poligoni o non ci sono target validi.")
# 4. Acquisizione Posizione Reale di Zeno dal Topic GPS
    try:
        # Aspetta un solo messaggio per avere lo scatto iniziale
        nav_msg = rospy.wait_for_message("/nav_status", NavStatus, timeout=10.0)
        current_n, current_e = ll2ne(origin, (nav_msg.position.latitude, nav_msg.position.longitude))
        current_pos_ned = [current_n, current_e]
        rospy.loginfo("Posizione Zeno agganciata in diretta: Nord {:.2f}, Est {:.2f}".format(current_n, current_e))
    except rospy.ROSException:
        rospy.logerr("Timeout! Nessun messaggio su /nav_status")
        # Se il simulatore lagga, impostiamo un valore fittizio per NON far crashare il TSP
        current_pos_ned = [0.0, 0.0]
        return

    # Creazione griglia globale
    all_points = [current_pos_ned] + targets_ned + [[obs[0], obs[1]] for obs in obstacles_ned]
    if poly_ned:
        all_points += poly_ned
        
    grid, min_n, min_e = create_occupancy_grid(obstacles_ned, poly_ned, all_points)

    full_path_ned = []
# --- APPLICAZIONE TSP ---

    # ==========================================
    # START CRONOMETRO
    # ==========================================
    start_time = time.time()
    
    ordered_targets, real_dist = solve_tsp(current_pos_ned, targets_ned, grid, RESOLUTION, min_n, min_e)
    
    end_time = time.time()
    tempo_impiegato = end_time - start_time
    # ==========================================
    # STOP CRONOMETRO
    # ==========================================

# Cicliamo sui target ORDINATI dal modulo TSP
    for i, target_ned in enumerate(ordered_targets):
        
        start_idx = (int((current_pos_ned[0]-min_n)/RESOLUTION), int((current_pos_ned[1]-min_e)/RESOLUTION))
        target_idx = (int((target_ned[0]-min_n)/RESOLUTION), int((target_ned[1]-min_e)/RESOLUTION))
        
        segment = astar(grid, start_idx, target_idx)
        
        if segment:
            for j, idx in enumerate(segment):
                # idx ora contiene (riga, colonna, z)
                z_flag = idx[2] 
                
                # Se è l'ultimo punto del segmento, snap esatto al target
                if j == len(segment) - 1:
                    pos_m = [float(target_ned[0]), float(target_ned[1]), z_flag]
                else:
                    pos_m = [float(idx[0]*RESOLUTION + min_n), float(idx[1]*RESOLUTION + min_e), z_flag]

                
                if not full_path_ned or pos_m[:2] != full_path_ned[-1][:2]:
                    full_path_ned.append(pos_m)                    
            current_pos_ned = full_path_ned[-1]
        else:
            rospy.logerr("Impossibile raggiungere la Tappa TSP %d!" % (i+1))


    if full_path_ned:
        rospy.sleep(1.0) 
        
        msg = WaypointPath()
        for point in full_path_ned:
            p = Point()
            p.x = point[0]
            p.y = point[1]
            p.z = point[2]
            msg.waypoints.append(p)
            
        waypoint_pub.publish(msg)
        rospy.loginfo("Missione Calcolata! {} waypoint pubblicati.".format(len(full_path_ned)))
        # =========================================================
        # SALVATAGGIO
        # =========================================================

        # Recuperiamo dinamicamente il percorso del pacchetto ROS
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('zeno_mission')
        log_dir = os.path.join(pkg_path, 'logs')

        # Se la cartella 'logs' non esiste sul PC del collega, la creiamo al volo
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)

        # Recupero in modo sicuro la variabile (evita errori se non ci sono target)
        usa_orig_sicuro = locals().get('usa_originale', False)

        # Creiamo un dizionario con tutto lo storico della missione
        mission_log = {
            "timestamp": datetime.now().strftime("%Y-%m-%d_%H-%M-%S"),
            "area_utilizzata": "Originale" if usa_orig_sicuro else "Ristretta",
            "target_ordinati": ordered_targets,        
            "distanza_stimata_m": real_dist,
            "tempo_calcolo_tsp_s": tempo_impiegato,
            "waypoint_totali": len(full_path_ned),
            "path": full_path_ned
        }
        
        # Generiamo il percorso corretto per il file JSON usando os.path.join
        nome_file = os.path.join(log_dir, "mission_log_{}.json".format(mission_log["timestamp"]))
        
        with open(nome_file, 'w') as f:
            json.dump(mission_log, f, indent=4)
            
        rospy.loginfo("Log di missione salvato in: %s", nome_file)
        
        # =========================================================
        # ORA METTIAMO IN PAUSA IL NODO
        # =========================================================
        rospy.sleep(1.0)
        rospy.loginfo("Planner in pausa. Tengo vivo il topic /waypoint_path...")
        rospy.spin()
        
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass