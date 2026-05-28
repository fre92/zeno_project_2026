#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import math
import heapq
import numpy as np
import matplotlib.path as mpltPath
import os
import rospkg
import yaml  # <-- AGGIUNTO: Necessario per leggere il file region_params.yaml

# Importiamo i messaggi standard e custom
from geometry_msgs.msg import Point
# Controlla che ci sia questa riga esatta in cima al file:
from marta_msgs.msg import NavStatus
from zeno_python.msg import WaypointPath
from geodetic_functions import ll2ne

# --- MODIFICA QUI: Aggiunto per calcolare il TSP a forza bruta ---
from itertools import permutations

# --- PARAMETRI ---
RESOLUTION = 0.50  
EXTRA_SAFETY_MARGIN = 0.5 # Margine extra per la barca 0.71 m di zeno + 0.29 di sicurezza per fare cifra tonda di celle

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
            if grid[node_position[0]][node_position[1]] != 0:
                continue
            if node_position in closed_set:
                continue
                
            new_node = Node(current_node, node_position)
            step_cost = 1.414 if new_position[0] != 0 and new_position[1] != 0 else 1.0
            
            turn_cost = 0.0
            if current_node.parent is not None:
                prev_dir = (current_node.position[0] - current_node.parent.position[0],
                            current_node.position[1] - current_node.parent.position[1])
                if prev_dir != new_position:
                    turn_cost = TURN_PENALTY

            new_node.g = current_node.g + step_cost + turn_cost
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
    grid = np.zeros((rows, cols), dtype=np.int8)  # <-- Ottimizzazione della RAM mantenuta

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
                    grid[r][c] = 1
                    
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
    rospy.loginfo("Generazione Matrice dei Costi A* tra %d nodi...", num_points)
    
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

    # 2. Lettura parametri gara (GIA' in NED)
    try:
        targets_ned = rospy.get_param("/targets")
        obstacles_ned = rospy.get_param("/obstacles")
    except KeyError:
        rospy.logerr("File YAML di targets o ostacoli non caricati!")
        return

    # --- NUOVO BLOCCO: Lettura target di copertura ---
    # Decommentato per evitare NameError alla riga di creazione di all_points
    try:
        coverage_targets_ned = rospy.get_param("/coverage_targets")
    except KeyError:
        coverage_targets_ned = []  # Se non ci sono target extra, andiamo avanti normalmente

    # 3. Lettura Poligono (Dal file YAML)
    poly_ned = []
    rospack = rospkg.RosPack()
    
    try:
        pkg_path = rospack.get_path('zeno_python') 
        # Assicurati che 'config' sia la cartella corretta, adatta se si trova altrove
        file_path = os.path.join(pkg_path, 'config', 'region_params.yaml')
        
        with open(file_path, 'r') as f:
            config_data = yaml.safe_load(f)
            # Leggiamo i vertici già in metri (no conversione ll2ne necessaria)
            poly_ned = config_data['vertici_poligono']['ristretto']
            
        rospy.loginfo("Poligono 'ristretto' caricato con successo dal file YAML.")
    except Exception as e:
        rospy.logwarn("File region_params.yaml non caricato correttamente (%s). Zeno navigherà senza confini!", e)

    # 4. Acquisizione Posizione Reale di Zeno dal Topic GPS
    rospy.loginfo("In attesa della posizione attuale di Zeno su /nav_status...")
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
    # --- MODIFICA QUI: Aggiunto coverage_targets_ned per scalare correttamente la griglia ---
    all_points = [current_pos_ned] + targets_ned + coverage_targets_ned + [[obs[0], obs[1]] for obs in obstacles_ned]
    if poly_ned:
        all_points += poly_ned
        
    grid, min_n, min_e = create_occupancy_grid(obstacles_ned, poly_ned, all_points)

    full_path_ned = []
    
    # --- APPLICAZIONE TSP ---
    rospy.loginfo("Calcolo dell'ordine ottimo dei target tramite TSP...")
    rospy.loginfo("Avvio Ottimizzatore TSP-A* (Obstacle-Aware)...")
    ordered_targets, real_dist = solve_tsp(current_pos_ned, targets_ned, grid, RESOLUTION, min_n, min_e)
    rospy.loginfo("Ordine ottimo trovato! Distanza: %.2f metri", real_dist)
    
    # Stampiamo l'ordine dei target per verifica nel terminale
    for idx, tg in enumerate(ordered_targets):
        rospy.loginfo(" Target %d originale -> Diventa la tappa #%d: [N: %.2f, E: %.2f]", targets_ned.index(tg)+1, idx+1, tg[0], tg[1])

    # --- NUOVO BLOCCO: Accodamento target di copertura ---
    if coverage_targets_ned:
        rospy.loginfo("Aggiungo %d target di copertura manuali a fine missione.", len(coverage_targets_ned))
        ordered_targets.extend(coverage_targets_ned)

    # Cicliamo sui target ORDINATI (primari + copertura)
    for i, target_ned in enumerate(ordered_targets):
        rospy.loginfo("Esecuzione A* verso la Tappa #%d..." % (i+1))
        
        start_idx = (int((current_pos_ned[0]-min_n)/RESOLUTION), int((current_pos_ned[1]-min_e)/RESOLUTION))
        target_idx = (int((target_ned[0]-min_n)/RESOLUTION), int((target_ned[1]-min_e)/RESOLUTION))
        
        segment = astar(grid, start_idx, target_idx)
        
        if segment:
            for j, idx in enumerate(segment):
                # idx ora contiene (riga, colonna, z)
                z_flag = idx[2] 
                
                if j == 0:
                    pos_m = [float(current_pos_ned[0]), float(current_pos_ned[1]), z_flag]
                else:
                    pos_m = [float(idx[0]*RESOLUTION + min_n), float(idx[1]*RESOLUTION + min_e), z_flag]
                
                if not full_path_ned or pos_m[:2] != full_path_ned[-1][:2]:
                    full_path_ned.append(pos_m)                    
            current_pos_ned = full_path_ned[-1]
        else:
            rospy.logerr("Impossibile raggiungere la Tappa %d!" % (i+1))

    # =========================================================
    # DEBUG A VIDEO: STAMPA I WAYPOINT CON Z=1
    # =========================================================
    rospy.loginfo("--- RICERCA WAYPOINT TARGET (Z=1.0) NELLA ROTTA ---")
    
    for indice, punto in enumerate(full_path_ned):
        # punto[0] è Nord (X), punto[1] è Est (Y), punto[2] è la Z
        if punto[2] == 1.0:
            rospy.loginfo("Trovato Target all'indice %d! Coordinate: [X: %.2f, Y: %.2f, Z: %.1f]", 
                          indice, punto[0], punto[1], punto[2])
    # =========================================================

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
        rospy.sleep(1.0)
        rospy.loginfo("Planner in pausa. Tengo vivo il topic /waypoint_path...")
        rospy.spin()  # <--- QUESTA RIGA EVITA CHE IL NODO MUOIA

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
