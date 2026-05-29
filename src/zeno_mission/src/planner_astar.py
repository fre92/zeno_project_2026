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
import os
import rospkg

# Importiamo i messaggi standard e custom
from geometry_msgs.msg import Point
from marta_msgs.msg import NavStatus
from zeno_mission.msg import WaypointPath
from geodetic_functions import ll2ne
from itertools import permutations

# --- PARAMETRI GLOBALI ---
RESOLUTION = 0.50  
EXTRA_SAFETY_MARGIN = 0.5 # Margine extra di sicurezza per ostacoli

# --- PARAMETRI DI MISSIONE E SENSORI ---
MAX_MISSION_DISTANCE = float('inf')  
MIN_UNCOVERED_AREA = 5.0      

# Parametri FLS (Frontal Looking Sonar)
FLS_MAX_RANGE = 10.0          # Metri massimi in avanti
FLS_MIN_RANGE = 5.0       # Metri minimi in avanti
FLS_FOV_DEG = 130.0       # Gradi totali di apertura azimutale

# Parametri SSS (Side Scan Sonar)
SSS_MIN_RANGE = 6.0       # Blind spot: non legge i primi 6 metri (Nadir coperto dall'FLS)
SSS_MAX_RANGE = 16.0      # Portata massima laterale

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

        if current_node.position == end_node.position:
            path = []
            current = current_node
            is_target_node = True  
            
            while current is not None:
                if is_target_node:
                    path.append((current.position[0], current.position[1], 1.0))
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
            if grid[node_position[0]][node_position[1]] == 1:
                continue
            if node_position in closed_set:
                continue
                
            new_node = Node(current_node, node_position)
            step_cost = 1.414 if new_position[0] != 0 and new_position[1] != 0 else 1.0

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
                    grid[r][c] = 10000
                    
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

def apply_curve_flags(path_ned):
    """Scansiona il percorso e applica i flag Z (-1.0, 3.0) per lo spegnimento SSS in curva."""
    sicurezza_precurva = 6
    sicurezza_postcurva = 6
    
    if len(path_ned) > 2:
        for idx in range(1, len(path_ned) - 1):
            dn_prev = path_ned[idx][0] - path_ned[idx-1][0]
            de_prev = path_ned[idx][1] - path_ned[idx-1][1]
            
            dn_new = path_ned[idx+1][0] - path_ned[idx][0]
            de_new = path_ned[idx+1][1] - path_ned[idx][1]
            
            angolo_prev = math.atan2(de_prev, dn_prev)
            angolo_new = math.atan2(de_new, dn_new)
            
            if abs(angolo_prev - angolo_new) > 0.05:
                start_j = max(0, idx - sicurezza_precurva)
                end_j = min(len(path_ned), idx + sicurezza_postcurva + 1) 
                
                for k in range(start_j, end_j):
                    stato_k = path_ned[k][2]
                    if stato_k == 1.0:
                        path_ned[k][2] = 3.0
                    elif stato_k == 0.0:
                        path_ned[k][2] = -1.0
                        
    return path_ned

def simulate_sensor_coverage(grid, path, min_n, min_e, resolution, cov_grid=None):
    """
    Simula la copertura realistica. FLS sempre attivo, SSS spento in curva leggendo il flag Z.
    """
    if cov_grid is None:
        cov_grid = copy.deepcopy(grid)
        
    rows = len(cov_grid)
    cols = len(cov_grid[0])
    
    max_range_m = max(FLS_MAX_RANGE, SSS_MAX_RANGE)
    max_search_cells = int(max_range_m / resolution) + 1
    cos_fls_threshold = math.cos(math.radians(FLS_FOV_DEG / 2.0))
    
    heading_n, heading_e = 1.0, 0.0
    
    for i in range(len(path)):
        p = path[i]
        r = int((p[0] - min_n) / resolution)
        c = int((p[1] - min_e) / resolution)
        
        # Lettura dello stato SSS dal waypoint
        z_flag = p[2] if len(p) > 2 else 0.0
        is_sss_active = True
        if z_flag in [-1.0, 3.0]:
            is_sss_active = False
            
        if i > 0:
            dn = p[0] - path[i-1][0]
            de = p[1] - path[i-1][1]
            dist_step = math.sqrt(dn**2 + de**2)
            if dist_step > 0.001: 
                heading_n = dn / dist_step
                heading_e = de / dist_step
        
        perp_n = -heading_e
        perp_e = heading_n
                
        for dr in range(-max_search_cells, max_search_cells + 1):
            for dc in range(-max_search_cells, max_search_cells + 1):
                dist_m = math.sqrt((dr * resolution)**2 + (dc * resolution)**2)
                if dist_m > max_range_m or dist_m == 0:
                    continue
                
                dot_product = (dr * resolution * heading_n + dc * resolution * heading_e) / dist_m
                
                # --- FLS ---
                is_fls = False
                if dist_m >= FLS_MIN_RANGE and dist_m <= FLS_MAX_RANGE and dot_product >= cos_fls_threshold:
                    is_fls = True
                    
                # --- SSS ---
                is_sss = False
                if is_sss_active:
                    proj_forward = abs(dr * resolution * heading_n + dc * resolution * heading_e)
                    proj_lateral = abs(dr * resolution * perp_n + dc * resolution * perp_e)
                    if SSS_MIN_RANGE <= proj_lateral <= SSS_MAX_RANGE and proj_forward <= 1.0:
                        is_sss = True

                if (is_fls or is_sss):
                    nr, nc = r + dr, c + dc
                    if 0 <= nr < rows and 0 <= nc < cols:
                        if grid[nr][nc] == 0:
                            cov_grid[nr][nc] = 2 
                                
    return cov_grid

def calculate_virtual_targets(grid, cov_grid, min_n, min_e, resolution, sensor_range, min_area):
    rows = len(cov_grid)
    cols = len(cov_grid[0])
    visited = [[False]*cols for _ in range(rows)]
    virtual_targets = []
    step_cells = max(1, int((sensor_range * 0.8) / resolution))
    
    for r in range(rows):
        for c in range(cols):
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
                        if grid[avg_r][avg_c] == 0: 
                            virtual_targets.append([avg_r * resolution + min_n, avg_c * resolution + min_e])
                        else:
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

def main():
    rospy.init_node('planner_astar')
    waypoint_pub = rospy.Publisher("/waypoint_path", WaypointPath, queue_size=1, latch=True)
    
    try:
        lat0 = rospy.get_param("/origin/coordinates/latitude")
        lon0 = rospy.get_param("/origin/coordinates/longitude")
        origin = (lat0, lon0)
    except KeyError:
        rospy.logerr("Origine non trovata nel parameter server!")
        return

    try:
        targets_gps = rospy.get_param("/targets")
        obstacles_gps = rospy.get_param("/obstacles")
        targets_ned = []
        obstacles_ned = []
        
        for t in targets_gps:
            n, e = ll2ne(origin, (t[0], t[1]))
            targets_ned.append([n, e])
            
        for obs in obstacles_gps:
            n, e = ll2ne(origin, (obs[0], obs[1]))
            obstacles_ned.append([n, e, obs[2]])
            
    except KeyError:
        rospy.logerr("File YAML di targets o ostacoli non caricati!")
        return

    poly_ned = []
    poly_original_ned = []
    poly_restricted_ned = []
    
    try:
        poly_original_ned = rospy.get_param("/polygon_vertices/original")
    except KeyError:
        pass
    try:
        poly_restricted_ned = rospy.get_param("/polygon_vertices/restricted")
    except KeyError:
        pass

    usa_originale = False
    if poly_original_ned and poly_restricted_ned and targets_ned:
        path_orig = mpltPath.Path(poly_original_ned)
        path_restr = mpltPath.Path(poly_restricted_ned)
        valid_targets = []

        for i, t in enumerate(targets_ned):
            if path_restr.contains_point((t[0], t[1])):
                valid_targets.append(t)
            elif path_orig.contains_point((t[0], t[1])):
                valid_targets.append(t)
                usa_originale = True
            else:
                rospy.logerr("ATTENZIONE: Target %d scartato, fuori dall'area!", i+1)

        targets_ned = valid_targets
        poly_ned = poly_original_ned if usa_originale else poly_restricted_ned
        rospy.loginfo("A_Star: Polig. %s", "ORIGINALE" if usa_originale else "RISTRETTO")
    else:
        poly_ned = poly_restricted_ned if poly_restricted_ned else poly_original_ned

    try:
        nav_msg = rospy.wait_for_message("/nav_status", NavStatus, timeout=10.0)
        current_n, current_e = ll2ne(origin, (nav_msg.position.latitude, nav_msg.position.longitude))
        current_pos_ned = [current_n, current_e]
    except rospy.ROSException:
        rospy.logerr("Timeout! Nessun messaggio su /nav_status")
        current_pos_ned = [0.0, 0.0]
        return

    all_points = [current_pos_ned] + targets_ned + [[obs[0], obs[1]] for obs in obstacles_ned]
    if poly_ned:
        all_points += poly_ned
        
    grid, min_n, min_e = create_occupancy_grid(obstacles_ned, poly_ned, all_points)
    full_path_ned = []
    
    # --- FASE 1: TSP SUI TARGET PRINCIPALI ---
    start_time = time.time()
    ordered_targets, real_dist = solve_tsp(current_pos_ned, targets_ned, grid, RESOLUTION, min_n, min_e)
    tempo_impiegato = time.time() - start_time

    for i, target_ned in enumerate(ordered_targets):
        start_idx = (int((current_pos_ned[0]-min_n)/RESOLUTION), int((current_pos_ned[1]-min_e)/RESOLUTION))
        target_idx = (int((target_ned[0]-min_n)/RESOLUTION), int((target_ned[1]-min_e)/RESOLUTION))
        segment = astar(grid, start_idx, target_idx)
        
        if segment:
            for j, idx in enumerate(segment):
                z_flag = idx[2] 
                if j == len(segment) - 1:
                    pos_m = [float(target_ned[0]), float(target_ned[1]), z_flag]
                else:
                    pos_m = [float(idx[0]*RESOLUTION + min_n), float(idx[1]*RESOLUTION + min_e), z_flag]
                
                if not full_path_ned or pos_m[:2] != full_path_ned[-1][:2]:
                    full_path_ned.append(pos_m)                    
            current_pos_ned = full_path_ned[-1]
        else:
            rospy.logerr("Impossibile raggiungere Tappa %d!" % (i+1))

    dist_totale_missione = real_dist
    budget_residuo = MAX_MISSION_DISTANCE - dist_totale_missione
    virtual_targets_visitati = 0

    # -> ORDINE TEMPORALE CORRETTO <-
    # 1. Calcoliamo e applichiamo i flag delle curve
    full_path_ned = apply_curve_flags(full_path_ned)
    
    # 2. Ora la simulazione legge i flag e spegne dinamicamente l'SSS nelle curve!
    cov_grid = simulate_sensor_coverage(grid, full_path_ned, min_n, min_e, RESOLUTION)
    
    # --- FASE 2: ESPLORAZIONE TARGET VIRTUALI ---
    if budget_residuo > 0 and full_path_ned:
        virtual_targets = calculate_virtual_targets(grid, cov_grid, min_n, min_e, RESOLUTION, SSS_MAX_RANGE, MIN_UNCOVERED_AREA)
        ordered_virtual_targets = solve_greedy_tsp(current_pos_ned[:2], virtual_targets, grid, RESOLUTION, min_n, min_e)
        
        for v_target in ordered_virtual_targets:
            start_idx = (int((current_pos_ned[0]-min_n)/RESOLUTION), int((current_pos_ned[1]-min_e)/RESOLUTION))
            target_idx = (int((v_target[0]-min_n)/RESOLUTION), int((v_target[1]-min_e)/RESOLUTION))
            
            segment = astar(grid, start_idx, target_idx)
            segment_cost = calculate_path_length(segment, RESOLUTION)
            
            if segment_cost <= budget_residuo:
                budget_residuo -= segment_cost
                dist_totale_missione += segment_cost
                virtual_targets_visitati += 1
                
                segment_path_ned = [] 
                for j, idx in enumerate(segment):
                    z_flag = idx[2] 
                    if j == len(segment) - 1:
                        pos_m = [float(v_target[0]), float(v_target[1]), z_flag]
                    else:
                        pos_m = [float(idx[0]*RESOLUTION + min_n), float(idx[1]*RESOLUTION + min_e), z_flag]
                    
                    segment_path_ned.append(pos_m)
                    if pos_m[:2] != full_path_ned[-1][:2]:
                        full_path_ned.append(pos_m)
                
                # Ri-applichiamo i flag per le nuove curve introdotte
                full_path_ned = apply_curve_flags(full_path_ned)
                # Dipingiamo la nuova copertura
                cov_grid = simulate_sensor_coverage(grid, segment_path_ned, min_n, min_e, RESOLUTION, cov_grid=cov_grid)
                current_pos_ned = full_path_ned[-1]
            else:
                break

    # --- STATISTICHE E LOG ---
    celle_esplorabili = sum(1 for r in range(len(grid)) for c in range(len(grid[0])) if grid[r][c] == 0)
    celle_esplorate = sum(1 for r in range(len(grid)) for c in range(len(grid[0])) if cov_grid[r][c] == 2)
                    
    if celle_esplorabili > 0:
        percentuale_copertura = (float(celle_esplorate) / float(celle_esplorabili)) * 100.0
        rospy.loginfo("Area totale esplorabile: %.2f m^2", celle_esplorabili * (RESOLUTION**2))
        rospy.loginfo("Area coperta (SSS disattivato in curva): %.2f m^2", celle_esplorate * (RESOLUTION**2))
        rospy.loginfo(">> PERCENTUALE DI COPERTURA: %.2f%% <<", percentuale_copertura)

    if full_path_ned:
        msg = WaypointPath()
        for point in full_path_ned:
            p = Point()
            p.x = point[0]
            p.y = point[1]
            p.z = point[2]
            msg.waypoints.append(p)
            
        waypoint_pub.publish(msg)
        rospy.loginfo("A_star: Pianificazione avvenuta con SUCCESSO")

        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('zeno_mission')
        log_dir = os.path.join(pkg_path, 'logs')
        if not os.path.exists(log_dir): os.makedirs(log_dir)

        usa_orig_sicuro = locals().get('usa_originale', False)
        mission_log = {
            "timestamp": datetime.now().strftime("%Y-%m-%d_%H-%M-%S"),
            "area_utilizzata": "Originale" if usa_orig_sicuro else "Ristretta",
            "target_reali_ordinati": ordered_targets,        
            "target_virtuali_visitati": virtual_targets_visitati, 
            "distanza_stimata_m": real_dist,
            "distanza_totale_m": dist_totale_missione,
            "copertura_percentuale": round(percentuale_copertura, 2) if celle_esplorabili > 0 else 0.0,
            "tempo_calcolo_tsp_s": tempo_impiegato,
            "waypoint_totali": len(full_path_ned),
            "path": full_path_ned
        }
        
        nome_file = os.path.join(log_dir, "mission_log_{}.json".format(mission_log["timestamp"]))
        with open(nome_file, 'w') as f: json.dump(mission_log, f, indent=4)
        rospy.loginfo("A_star: Log salvato in: %s", nome_file)
        
        rospy.sleep(1.0)
        rospy.spin()
        
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
