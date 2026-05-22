#!/usr/bin/env python
import rospy
import math
import numpy as np
import matplotlib.pyplot as plt
from functools import cmp_to_key
import rospkg
import os


from geodetic_functions import ll2ne 

def read_and_convert_to_ned(file_path):
    # Legge il file di testo contenete i vertici del Poligono, trova origin e converte i vertici in NED
    lats = []
    lons = []
    
    if not os.path.exists(file_path):
        rospy.logerr("Il file {} non esiste! Verifica il percorso".format(file_path))
        raise ValueError("Il file {} non esiste!".format(file_path))

    with open(file_path, 'r') as file:
        for line in file:
            if line.strip():
                lon, lat = map(float, line.strip().split(','))
                lats.append(lat)
                lons.append(lon)
                
    # Individuazione origine (spigolo in basso a sinistra della Box)
    lat_origin = min(lats)
    lon_origin = min(lons)
    origin = (lat_origin, lon_origin)
    
    # Conversione vertici in NED 
    points_ned = []
    for lat, lon in zip(lats, lons):
        n, e = ll2ne(origin, (lat, lon))
        points_ned.append([n, e])
        
    return np.array(points_ned), origin

def sort_convex_polygon_ccw(points):
  
    # Calcoliamo il baricentro 
    center_n = np.mean(points[:, 0])
    center_e = np.mean(points[:, 1])
    
    # Calcoliamo l'angolo polare di ogni vertice rispetto al baricentro.
    angles = np.arctan2(points[:, 1] - center_e, points[:, 0] - center_n)
    
    # Ordiniamo (crescente) gli indici in base all'angolo calcolato 
    sorted_idx = np.argsort(angles)
    ordered_polygon = points[sorted_idx]
    
    # --- STAMPA DEI LOG DI VERIFICA ---
    rospy.loginfo("======= VERTICI ORDINATI TRAMITE BARICENTRO =======")
    for idx, pt in enumerate(ordered_polygon):
        rospy.loginfo("Vertice %d -> North: %.3f, East: %.3f", idx, pt[0], pt[1])
    rospy.loginfo("====================================================")
    
    return ordered_polygon


def compute_inset_polygon(hull_points, offset=2.0):
   
    n_v = len(hull_points)
    inset_points = []
    shifted_lines = []
    p1_shifted_list = []
    
    # Spostiamo i segmenti verso l'interno lungo la normale
    for i in range(n_v):
        p1 = hull_points[i]
        p2 = hull_points[(i + 1) % n_v]
        
        # Direzione del segmento
        v = p2 - p1
        norm_v = np.linalg.norm(v)
        
        if norm_v < 1e-6:
            raise ValueError("I vertici {} e {} sono sovrapposti. Impossibile calcolare il poligono.".format(i, (i + 1) % n_v))
            
        u = v / norm_v
        
        # Normale interna ruotata a sinistra 
        normal = np.array([-u[1], u[0]])
        
        # Spostiamo i punti del segmento verso l'interno di una quota pari a 'offset'
        p1_shifted = p1 + normal * offset
        p2_shifted = p2 + normal * offset
        p1_shifted_list.append(p1_shifted)
        
        # Coefficienti della retta implicita cartesiana: A*North + B*East = C
        # Derivazione analitica: A = (Y2 - Y1) -> dEast, B = (X1 - X2) -> -dNorth
        A = p2_shifted[1] - p1_shifted[1]
        B = p1_shifted[0] - p2_shifted[0]
        C = A * p1_shifted[0] + B * p1_shifted[1]
        shifted_lines.append((A, B, C))
        
    # Calcoliamo le nuove intersezioni geometriche risolvendo il sistema lineare
    for i in range(n_v):
        A1, B1, C1 = shifted_lines[(i - 1) % n_v]
        A2, B2, C2 = shifted_lines[i]
        
        # Risoluzione del sistema con la regola di Cramer
        det = A1 * B2 - B1 * A2
        if abs(det) < 1e-6:
            # Fallback robusto per punti collineari (obliqui, verticali o orizzontali)
            inset_points.append(p1_shifted_list[i])
        else:
            
            n_new = (C1 * B2 - B1 * C2) / det
            e_new = (A1 * C2 - C1 * A2) / det
            inset_points.append([n_new, e_new])
            
    return np.array(inset_points)

def save_params_to_yaml(origin_ll, poly_orig, poly_rest):
    try:
        
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('zeno_mission') 
        
        file_path = os.path.join(pkg_path, 'config', 'region_params.yaml')

        with open(file_path, 'w') as f:

            # Scrittura origine 
            f.write("origin:\n")
            f.write("  coordinates:\n")
            f.write("    latitude:   {}\n".format(origin_ll[0]))
            f.write("    longitude:  {}\n".format(origin_ll[1]))
            f.write("    depth:      0.0\n\n")
            
            # Scrittura dei vertici dei poligoni rispetto al fixed frame (NED)
            f.write("# vertici poligono w.r.t the fixed frame\n")
            f.write("polygon_vertices:\n") 
            
            # Poligono Originario
            f.write("  original:\n")
            for wp in poly_orig:
                f.write("    - [{:.3f}, {:.3f}]\n".format(wp[0], wp[1]))
                
            f.write("\n")
            
            # Poligono Ristretto
            f.write("  restricted:\n")
            for wp in poly_rest:
                f.write("    - [{:.3f}, {:.3f}]\n".format(wp[0], wp[1]))

        rospy.loginfo("Parametri della regione salvati con successo in: %s", file_path)

    except Exception as e:
        rospy.logerr("Errore durante il salvataggio del file YAML dei parametri: %s", str(e))    

# --- SCRIPT EXECUTION ---
if __name__ == "__main__":
    rospy.init_node("polygon_ini", anonymous=True)
    
    script_dir = os.path.dirname(os.path.abspath(__file__))
    file_name = os.path.join(script_dir, "safezone.txt")
    
    
    try:
        # Lettura vertici e conversione in NED
        points_ned, origin_ll = read_and_convert_to_ned(file_name)
        rospy.loginfo("Origine NED impostata a (Lat, Lon): %s", str(origin_ll))
        
        # Ordinamento vertici CCW 
        poly_original = sort_convex_polygon_ccw(points_ned)
        rospy.loginfo("Vertici del poligono convesso ordinati correttamente in senso CCW.")
        
        # Calcolo del perimetro interno di sicurezza (-safety_offset metri)
        safety_offset = 2.0
        poly_restricted = compute_inset_polygon(poly_original, offset=safety_offset)
        rospy.loginfo("Poligono di sicurezza calcolato con successo.")
        
        save_params_to_yaml(origin_ll, poly_original, poly_restricted)

        # Rappresentazione grafica dei risultati
        plt.figure(figsize=(8, 7))
        
        
        plt.scatter(points_ned[:, 1], points_ned[:, 0], color='red', s=80, zorder=5, label='VErtici')
        
        # Chiusura dei perimetri per la rappresentazione grafica
        plot_orig = np.vstack([poly_original, poly_original[0]])
        plot_rest = np.vstack([poly_restricted, poly_restricted[0]])
        
        
        plt.plot(plot_orig[:, 1], plot_orig[:, 0], 'b-o', linewidth=2, label='Poligono Originale')
        plt.plot(plot_rest[:, 1], plot_rest[:, 0], 'g--s', linewidth=2, label='Perimetro Ristretto')
        
        # Aggiornamento etichette grafiche coerente con la tua specifica aziendale
        plt.title("Generazione Area di Sicurezza Ristretta")
        plt.xlabel("East [metri]") 
        plt.ylabel("North [metri]")
        plt.grid(True, linestyle=':', alpha=0.6)
        plt.axis('equal')
        plt.legend()
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('zeno_mission') 
        image_path = os.path.join(pkg_path, 'img', 'polygon_ini_debug.png')
        plt.savefig(image_path, dpi=300, bbox_inches='tight')
        rospy.loginfo("Grafico di debug salvato in: %s", image_path)

        plt.show()

        
        
    except Exception as e:
        rospy.logerr("Errore di esecuzione: %s", str(e))