#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import math
import heapq
import numpy as np
import matplotlib.path as mpltPath
import time
import json
import copy
from datetime import datetime

# Importiamo i messaggi standard e custom
from geometry_msgs.msg import Point
from marta_msgs.msg import NavStatus
from zeno_mission.msg import WaypointPath
from geodetic_functions import ll2ne
from itertools import permutations

# --- PARAMETRI GLOBALI ---
RESOLUTION = 0.50  
EXTRA_SAFETY_MARGIN = 0.5 # Margine extra per la barca 0.71 m di zeno + 0.29 di sicurezza

# --- NUOVI PARAMETRI (ESPLORAZIONE / TARGET VIRTUALI) ---
MAX_MISSION_DISTANCE = float('inf')  # Budget totale della missione in metri 
SWATH_WIDTH = 10.0             # Larghezza del fascio del sensore in metri (diametro del semicerchio)
MIN_UNCOVERED_AREA = 5.0      # Area minima in m^2 di un "buco" per giustificare un target virtuale

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

def astar(grid, start, end, target_z=1.0):
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

        if current_node.position == end_node.position:
            path = []
            current = current_node
            is_target_node = True
            
            while current is not None:
                if is_target_node:
                    path.append((current.position[0], current.position[1], target_z))
                    is_target_node = False
                else:
                    path.append((current.position[0], current.position[1], 0.0))
                    
                current = current.parent
                
            return path[::-1]

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
    grid = np.zeros((rows, cols))

    for obs in obstacles_ned:
        obs_n, obs_e, obs_r = obs[0], obs[1], obs[2]
        inflation_cells = int((obs_r + EXTRA_SAFETY_MARGIN) / RESOLUTION)
        obs_r_idx, obs_c_idx = int((obs_n - min_n) / RESOLUTION), int((obs_e - min_e) / RESOLUTION)
        
        for r in range(max(0, obs_r_idx - inflation_cells), min(rows, obs_r_idx + inflation_cells + 1)):
            for c in range(max(0, obs_c_idx - inflation_cells), min(cols, obs_c_idx + inflation_cells + 1)):
                if math.sqrt((r - obs_r_idx)**2 + (c - obs_c_idx)**2) <= inflation_cells:
                    grid[r][c] = 1

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
    if not segment or len(segment) < 2:
        return float('inf')
    
    dist_totale = 0.0
    for k in range(1, len(segment)):
        dn = segment[k][0] - segment[k-1][0]
        de = segment[k][1] - segment[k-1][1]
        dist_totale += math.sqrt(dn**2 + de**2) * resolution
    return dist_totale

def solve_tsp(start_ned, targets_ned, grid, resolution, min_n, min_e):
    import itertools
    all_points = [start_ned] + targets_ned
    num_points = len(all_points)
    
    cost_matrix = [[0.0] * num_points for _ in range(num_points)]
    rospy.loginfo("Generazione Matrice Costi Fase 1...")
    
    for i in range(num_points):
        for j in range(i + 1, num_points):
            start_idx = (int((all_points[i][0] - min_n) / resolution), int((all_points[i][1] - min_e) / resolution))
            target_idx = (int((all_points[j][0] - min_n) / resolution), int((all_points[j][1] - min_e) / resolution))
            
            segment = astar(grid, start_idx, target_idx)
            real_dist = calculate_path_length(segment, resolution)
            
            cost_matrix[i][j] = real_dist
            cost_matrix[j][i] = real_dist

    best_cost = float('inf')
    best_order_indices = None
    target_indices = list(range(1, num_points))
    
    for perm in itertools.permutations(target_indices):
        current_cost = 0.0
        current_node = 0
        for next_node in perm:
            current_cost += cost_matrix[current_node][next_node]
            current_node = next_node
        if current_cost < best_cost:
            best_cost = current_cost
            best_order_indices = perm
            
    ordered_targets = [all_points[idx] for idx in best_order_indices]
    return ordered_targets, best_cost

def simulate_sensor_coverage(grid, path, min_n, min_e, resolution, swath_width, cov_grid=None):
    """
    Simula la copertura dei sensori lungo un path e aggiorna la griglia di copertura (cov_grid).
    Se cov_grid non viene fornita, ne crea una nuova basata su 'grid'.
    Restituisce la cov_grid aggiornata.
    """
    if cov_grid is None:
        cov_grid = copy.deepcopy(grid)
        
    rows = len(cov_grid)
    cols = len(cov_grid[0])
    swath_cells = int((swath_width / 2.0) / resolution)
    heading_n, heading_e = 1.0, 0.0
    
    for i in range(len(path)):
        p = path[i]
        r = int((p[0] - min_n) / resolution)
        c = int((p[1] - min_e) / resolution)
        
        if i > 0:
            dn = p[0] - path[i-1][0]
            de = p[1] - path[i-1][1]
            dist = math.sqrt(dn**2 + de**2)
            if dist > 0.001: 
                heading_n = dn / dist
                heading_e = de / dist
                
        for dr in range(-swath_cells, swath_cells + 1):
            for dc in range(-swath_cells, swath_cells + 1):
                if dr*dr + dc*dc <= swath_cells*swath_cells:
                    dot_product = dr * heading_n + dc * heading_e
                    if dot_product >= -0.1:
                        nr, nc = r + dr, c + dc
                        if 0 <= nr < rows and 0 <= nc < cols:
                            # Non sovrascriviamo gli ostacoli originali (che sono 1 in grid)
                            # Segniamo con 2 le celle libere esplorate
                            if grid[nr][nc] == 0:
                                cov_grid[nr][nc] = 2 
                                
    return cov_grid

def calculate_virtual_targets(grid, cov_grid, min_n, min_e, resolution, swath_width, min_area):
    """
    Identifica le zone d'ombra in cov_grid e genera i target virtuali.
    """
    rows = len(cov_grid)
    cols = len(cov_grid[0])
    visited = [[False]*cols for _ in range(rows)]
    virtual_targets = []
    step_cells = max(1, int((swath_width * 0.8) / resolution))
    
    for r in range(rows):
        for c in range(cols):
            # Le zone d'ombra sono quelle ancora a 0 in cov_grid (né ostacoli né esplorate)
            if cov_grid[r][c] == 0 and not visited[r][c]:
                queue = [(r, c)]
                visited[r][c] = True
                blob = []
                
                while queue:
                    curr_r, curr_c = queue.pop(0)
                    blob.append((curr_r, curr_c))
                    
                    for dr, dc in [(-1,0), (1,0), (0,-1), (0,1)]:
                        nr, nc = curr_r + dr, curr_c + dc
                        if 0 <= nr < rows and 0 <= nc < cols:
                            if cov_grid[nr][nc] == 0 and not visited[nr][nc]:
                                visited[nr][nc] = True
                                queue.append((nr, nc))
                                
                area_m2 = len(blob) * (resolution ** 2)
                rospy.loginfo("Area blob: %.2f m", area_m2)
                if area_m2 >= min_area:
                    blob_set = set(blob)
                    min_r, max_r = min([pt[0] for pt in blob]), max([pt[0] for pt in blob])
                    min_c, max_c = min([pt[1] for pt in blob]), max([pt[1] for pt in blob])
                    targets_aggiunti = 0
                    
                    for br in range(min_r, max_r + 1, step_cells):
                        for bc in range(min_c, max_c + 1, step_cells):
                            if (br, bc) in blob_set:
                                if grid[br][bc] == 0: 
                                    n_real = br * resolution + min_n
                                    e_real = bc * resolution + min_e
                                    virtual_targets.append([n_real, e_real])
                                    targets_aggiunti += 1
                                    
                    if targets_aggiunti == 0:
                        avg_r = int(sum([pt[0] for pt in blob]) / len(blob))
                        avg_c = int(sum([pt[1] for pt in blob]) / len(blob))
                        if grid[avg_r][avg_c] == 0: # Correzione: assicuriamoci che il fallback non sia un ostacolo
                            virtual_targets.append([avg_r * resolution + min_n, avg_c * resolution + min_e])
                        else:
                            # Se cade in ostacolo, cerca il punto libero più vicino nel blob
                            best_pt = min(blob, key=lambda pt: (pt[0]-avg_r)**2 + (pt[1]-avg_c)**2)
                            virtual_targets.append([best_pt[0] * resolution + min_n, best_pt[1] * resolution + min_e])
                        
    return virtual_targets

def solve_greedy_tsp(start_ned, targets_ned, grid, resolution, min_n, min_e):
    if not targets_ned: return []
    unvisited = copy.deepcopy(targets_ned)
    ordered = []
    current_pos = start_ned
    
    while unvisited:
        best_cost = float('inf')
        best_target = None
        best_idx = -1
        start_idx = (int((current_pos[0] - min_n) / resolution), int((current_pos[1] - min_e) / resolution))
        
        for i, tg in enumerate(unvisited):
            tg_idx = (int((tg[0] - min_n) / resolution), int((tg[1] - min_e) / resolution))
            segment = astar(grid, start_idx, tg_idx)
            cost = calculate_path_length(segment, resolution)
            
            if cost < best_cost:
                best_cost = cost
                best_target = tg
                best_idx = i
                
        ordered.append(best_target)
        current_pos = best_target
        unvisited.pop(best_idx)
        
    return ordered

# ==============================================================================

def main():
    rospy.init_node('planner_astar_coverage')
    waypoint_pub = rospy.Publisher("/waypoint_path", WaypointPath, queue_size=1, latch=True)
    
    try:
        lat0, lon0 = rospy.get_param("/origin/coordinates/latitude"), rospy.get_param("/origin/coordinates/longitude")
        origin = (lat0, lon0)
        targets_gps, obstacles_gps = rospy.get_param("/targets"), rospy.get_param("/obstacles")
        targets_ned, obstacles_ned = [], []
        for t in targets_gps: targets_ned.append(list(ll2ne(origin, (t[0], t[1]))))
        for o in obstacles_gps: 
            n, e = ll2ne(origin, (o[0], o[1]))
            obstacles_ned.append([n, e, o[2]])
    except KeyError:
        rospy.logerr("Errore caricamento parametri base.")
        return

    poly_ned = []
    poly_original_ned = rospy.get_param("/polygon_vertices/original", [])
    poly_restricted_ned = rospy.get_param("/polygon_vertices/restricted", [])
    
    if poly_original_ned and poly_restricted_ned and targets_ned:
        path_orig = mpltPath.Path(poly_original_ned)
        path_restr = mpltPath.Path(poly_restricted_ned)
        valid_targets = []
        usa_originale = False

        for i, t in enumerate(targets_ned):
            in_restr = path_restr.contains_point((t[0], t[1]))
            in_orig = path_orig.contains_point((t[0], t[1]))
            if in_restr: valid_targets.append(t)
            elif in_orig: 
                valid_targets.append(t)
                usa_originale = True
            else: rospy.logerr("Target %d fuori area! Scartato.", i+1)

        targets_ned = valid_targets
        poly_ned = poly_original_ned if usa_originale else poly_restricted_ned
    else:
        poly_ned = poly_restricted_ned if poly_restricted_ned else poly_original_ned

    rospy.loginfo("In attesa di /nav_status...")
    try:
        nav_msg = rospy.wait_for_message("/nav_status", NavStatus, timeout=10.0)
        current_n, current_e = ll2ne(origin, (nav_msg.position.latitude, nav_msg.position.longitude))
        current_pos_ned = [current_n, current_e]
    except rospy.ROSException:
        current_pos_ned = [0.0, 0.0]

    all_points = [current_pos_ned] + targets_ned + [[obs[0], obs[1]] for obs in obstacles_ned] + poly_ned
    grid, min_n, min_e = create_occupancy_grid(obstacles_ned, poly_ned, all_points)

    start_time = time.time()
    
    ordered_targets, dist_fase_1 = solve_tsp(current_pos_ned, targets_ned, grid, RESOLUTION, min_n, min_e)
    rospy.loginfo("Fase 1 TSP Completata. Costo: %.2f m", dist_fase_1)
    
    full_path_ned = []
    nav_pos = current_pos_ned
    
    for i, target_ned in enumerate(ordered_targets):
        start_idx = (int((nav_pos[0]-min_n)/RESOLUTION), int((nav_pos[1]-min_e)/RESOLUTION))
        target_idx = (int((target_ned[0]-min_n)/RESOLUTION), int((target_ned[1]-min_e)/RESOLUTION))
        
        segment = astar(grid, start_idx, target_idx, target_z=1.0) 
        
        if segment:
            for j, idx in enumerate(segment):
                z_flag = idx[2] 
                if j == len(segment) - 1:
                    pos_m = [float(target_ned[0]), float(target_ned[1]), z_flag]
                else:
                    pos_m = [float(idx[0]*RESOLUTION + min_n), float(idx[1]*RESOLUTION + min_e), z_flag]
                
                if not full_path_ned or pos_m[:2] != full_path_ned[-1][:2]:
                    full_path_ned.append(pos_m)                    
            nav_pos = full_path_ned[-1]

    dist_totale_missione = dist_fase_1
    budget_residuo = MAX_MISSION_DISTANCE - dist_totale_missione
    
    virtual_targets_visitati = 0
    
    # 1. Calcoliamo la copertura della sola Fase 1
    cov_grid = simulate_sensor_coverage(grid, full_path_ned, min_n, min_e, RESOLUTION, SWATH_WIDTH)
    
    if budget_residuo > 0 and full_path_ned:
        # 2. Generiamo i target virtuali basandoci sulle zone ancora a 0 in cov_grid
        virtual_targets = calculate_virtual_targets(
            grid, cov_grid, min_n, min_e, RESOLUTION, SWATH_WIDTH, MIN_UNCOVERED_AREA
        )
        
        ordered_virtual_targets = solve_greedy_tsp(nav_pos[:2], virtual_targets, grid, RESOLUTION, min_n, min_e)
        
        for v_target in ordered_virtual_targets:
            start_idx = (int((nav_pos[0]-min_n)/RESOLUTION), int((nav_pos[1]-min_e)/RESOLUTION))
            target_idx = (int((v_target[0]-min_n)/RESOLUTION), int((v_target[1]-min_e)/RESOLUTION))
            
            segment = astar(grid, start_idx, target_idx, target_z=2.0)
            segment_cost = calculate_path_length(segment, RESOLUTION)
            
            if segment_cost <= budget_residuo:
                budget_residuo -= segment_cost
                dist_totale_missione += segment_cost
                virtual_targets_visitati += 1
                
                segment_path_ned = [] # Salviamo il path del singolo segmento in metri
                
                for j, idx in enumerate(segment):
                    z_flag = idx[2] 
                    if j == len(segment) - 1:
                        pos_m = [float(v_target[0]), float(v_target[1]), z_flag]
                    else:
                        pos_m = [float(idx[0]*RESOLUTION + min_n), float(idx[1]*RESOLUTION + min_e), z_flag]
                    
                    segment_path_ned.append(pos_m)
                    
                    if pos_m[:2] != full_path_ned[-1][:2]:
                        full_path_ned.append(pos_m)
                
                # Aggiorniamo dinamicamente la mappa di copertura (cov_grid) 
                # "dipingendo" man mano che aggiungiamo target virtuali!
                cov_grid = simulate_sensor_coverage(grid, segment_path_ned, min_n, min_e, RESOLUTION, SWATH_WIDTH, cov_grid=cov_grid)
                
                nav_pos = full_path_ned[-1]
            else:
                rospy.logwarn("Budget Esaurito!")
                break
    else:
        rospy.logwarn("Budget insufficiente per generare la Fase 2.")

    # ==========================================================================
    # CALCOLO PERCENTUALE DI COPERTURA FINALE
    # ==========================================================================
    celle_esplorabili = 0
    celle_esplorate = 0
    
    rows, cols = len(grid), len(grid[0])
    for r in range(rows):
        for c in range(cols):
            # Le celle esplorabili sono quelle che nella griglia originaria NON erano ostacoli (0)
            if grid[r][c] == 0:
                celle_esplorabili += 1
                # Le celle esplorate sono quelle che in cov_grid sono state marcate con 2
                if cov_grid[r][c] == 2:
                    celle_esplorate += 1
                    
    if celle_esplorabili > 0:
        percentuale_copertura = (float(celle_esplorate) / float(celle_esplorabili)) * 100.0
        rospy.loginfo("Area totale esplorabile: %.2f m^2", celle_esplorabili * (RESOLUTION**2))
        rospy.loginfo("Area effettivamente coperta: %.2f m^2", celle_esplorate * (RESOLUTION**2))
        rospy.loginfo(">> PERCENTUALE DI COPERTURA RAGGIUNTA: %.2f%% <<", percentuale_copertura)
    else:
        percentuale_copertura = 0.0
        rospy.logwarn("Non ci sono celle esplorabili sulla mappa!")

    end_time = time.time()

    if full_path_ned:

        # Controllo le curve
	if len(full_path_ned) > 2:
            
            for idx in range(1, len(full_path_ned) - 1):
                
                # Calcoliamo le variazioni
                dn_prev = full_path_ned[idx][0] - full_path_ned[idx-1][0]
                de_prev = full_path_ned[idx][1] - full_path_ned[idx-1][1]
                
                dn_new = full_path_ned[idx+1][0] - full_path_ned[idx][0]
                de_new = full_path_ned[idx+1][1] - full_path_ned[idx][1]
                
                # math.atan2 ci dà la direzione pura in radianti (da -pi a pi)
                angolo_prev = math.atan2(de_prev, dn_prev)
                angolo_new = math.atan2(de_new, dn_new)

		diff_angolo = angolo_prev - angolo_new
                diff_normalizzata = math.atan2(math.sin(diff_angolo), math.cos(diff_angolo))
                
                # Usiamo una piccolissima tolleranza per i floating point
                # Se la differenza tra gli angoli è maggiore di un'inezia, stiamo curvando
                if abs(diff_normalizzata) > 0.01: 
                    rospy.loginfo("CURVA indice %d", idx)
                    
                    stato_z_attuale = full_path_ned[idx][2]
                    for j in range(max(-4, 0 - idx), min(4, len(full_path_ned) - idx)):
                    	if stato_z_attuale == 1.0:
                        	# È un TARGET principale ED è una Curva
                        	full_path_ned[idx+j][2] = 3.0 
                    	elif stato_z_attuale == 2.0:
                        	# È un TARGET virtuale ED è una Curva
                        	full_path_ned[idx+j][2] = 4.0 
                    	else:
                        	# È solo una normale curva per evitare un ostacolo
                        	full_path_ned[idx+j][2] = -1.0

        msg = WaypointPath()
        for point in full_path_ned:
            p = Point()
            p.x, p.y, p.z = point[0], point[1], point[2]
            msg.waypoints.append(p)
            
        waypoint_pub.publish(msg)

        usa_orig_sicuro = locals().get('usa_originale', False)
        mission_log = {
            "timestamp": datetime.now().strftime("%Y-%m-%d_%H-%M-%S"),
            "area_utilizzata": "Originale" if usa_orig_sicuro else "Ristretta",
            "target_reali_ordinati": ordered_targets,
            "target_virtuali_visitati": virtual_targets_visitati,
            "distanza_fase1_m": dist_fase_1,
            "distanza_totale_m": dist_totale_missione,
            "copertura_percentuale": round(percentuale_copertura, 2), # Salviamo la % nel JSON!
            "tempo_calcolo_s": end_time - start_time,
            "waypoint_totali": len(full_path_ned)
        }
        
        try:
            nome_file = "/media/sf_ros_condivisa/mission_log_{}.json".format(mission_log["timestamp"])
            with open(nome_file, 'w') as f:
                json.dump(mission_log, f, indent=4)
        except Exception as e:
            rospy.logerr("Salvataggio JSON fallito: %s", e)
            
        rospy.spin() 
        
if __name__ == '__main__':
    try: main()
    except rospy.ROSInterruptException: pass
