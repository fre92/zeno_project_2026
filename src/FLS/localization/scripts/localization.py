#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import json
import threading
import os  # <--- NUOVO IMPORT per la gestione di cartelle e file
from collections import deque
from std_msgs.msg import String

# Importiamo anche ll2distance per misurare la distanza spaziale in metri tra due (lat, lon)
from geodetic_functions import ne2ll, ll2distance

# NOTA: Sostituisci 'marta_msgs'
from marta_msgs.msg import NavStatus 

class FLSLocalizationNode:
    def __init__(self):
        rospy.init_node('fls_localization_node', anonymous=False)

        # --- PARAMETRI STATICI DEL SENSORE E DEL ROBOT ---
        self.NOMINAL_ALTITUDE = 3.5
        self.R_MAX = 15.0               
        self.U_0 = 512.0                
        self.V_0 = 768.0                
        self.V_ARC = 18.0              
        self.S_F = self.R_MAX / (self.V_0 - self.V_ARC) 
        self.OFFSET_X_BODY = 0.375      
        self.OFFSET_Z_SONAR = 0.103     

        # --- BUFFER DI NAVIGAZIONE ---
        self.nav_buffer = deque(maxlen=100)
        self.buffer_lock = threading.Lock()

        # --- MEMORIA GLOBALE (MAPPA) ---
        self.global_targets = {}
        self.next_global_id = 1
        self.CLUSTER_RADIUS_M = 3.0  # Distanza (metri) per considerare due detection come lo stesso oggetto fisico

        # --- GESTIONE OUTPUT DI TESTO ---
        self.output_dir = "target_list"
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)
        self.log_file_path = os.path.join(self.output_dir, "localized_targets_log.txt")
        
        # Svuotiamo/Creiamo il file di log all'avvio del nodo
        with open(self.log_file_path, "w") as f:
            f.write("=== LOG TARGET LOCALIZZATI ===\n\n")

        # Subscribers
        rospy.Subscriber('/nav_status', NavStatus, self.nav_callback)
        rospy.Subscriber('/perception/target_json', String, self.target_callback)
        
        # Publisher
        self.pub_localized = rospy.Publisher('/perception/target_localized_json', String, queue_size=10)

        rospy.loginfo("Nodo di Localizzazione e Mapping (Clustering Spaziale) avviato.")
        rospy.loginfo("I log formattati verranno salvati in: {}".format(os.path.abspath(self.log_file_path)))

    def nav_callback(self, msg):
        if msg.initialized:
            stamp_sec = msg.header.stamp.to_sec()
            nav_state = {
                'lat': msg.position.latitude,
                'lon': msg.position.longitude,
                'yaw': msg.orientation.yaw
            }
            with self.buffer_lock:
                self.nav_buffer.append((stamp_sec, nav_state))

    def target_callback(self, msg):
        try:
            target_data = json.loads(msg.data)
            
            # 1. SINCRONIZZAZIONE TEMPORALE
            fls_stamp = target_data.get('fls_stamp_sec')
            if fls_stamp is None:
                rospy.logwarn("JSON ricevuto senza fls_stamp_sec. Impossibile sincronizzare.")
                return

            with self.buffer_lock:
                if len(self.nav_buffer) == 0:
                    rospy.logwarn_throttle(2.0, "Buffer navigazione vuoto. Attesa dati...")
                    return

                closest_match = min(self.nav_buffer, key=lambda x: abs(x[0] - fls_stamp))
                nav_state_backup = self.nav_buffer[-1][1] 
            
            matched_time = closest_match[0]
            nav_state = closest_match[1]
            time_diff = abs(matched_time - fls_stamp)
            
            if time_diff > 0.5:
                if time_diff > 100000: 
                    rospy.logwarn_once("ATTENZIONE: Mismatch orologi. Uso ultima posizione nota.")
                    nav_state = nav_state_backup 
                else:
                    return # Lag reale, scartiamo
            
            # 2. CALCOLI GEOMETRICI E WGS-84
            bbox = target_data.get('bbox_px')
            if bbox is None:
                return

            x1, y1, ew, eh = bbox
            u_t = x1 + (ew / 2.0)
            v_t = float(y1 + eh)

            X_slant = (self.V_0 - v_t) * self.S_F
            Y_slant = (u_t - self.U_0) * self.S_F
            R_slant = math.sqrt(X_slant**2 + Y_slant**2)
            h_sonar = self.NOMINAL_ALTITUDE - self.OFFSET_Z_SONAR
            
            if R_slant > h_sonar:
                R_ground = math.sqrt(R_slant**2 - h_sonar**2)
                correction_factor = R_ground / R_slant
                X_body = (X_slant * correction_factor) + self.OFFSET_X_BODY
                Y_body = (Y_slant * correction_factor)
            else:
                return

            yaw = nav_state['yaw']
            auv_lat = nav_state['lat']
            auv_lon = nav_state['lon']

            X_ned = X_body * math.cos(yaw) - Y_body * math.sin(yaw)
            Y_ned = X_body * math.sin(yaw) + Y_body * math.cos(yaw)

            # Otteniamo le coordinate grezze di questo singolo frame
            raw_lat, raw_lon = ne2ll((auv_lat, auv_lon), (X_ned, Y_ned))
            raw_type = target_data.get('type', 'unknown')
            raw_conf = target_data.get('confidence', 0.5)

            # 3. CLUSTERING SPAZIALE (MEMORIA DELLA MAPPA)
            best_global_id = None
            min_dist = float('inf')

            # Cerchiamo se questo punto cade vicino a un bersaglio già noto sulla mappa
            for gid, gdata in self.global_targets.items():
                dist = ll2distance((raw_lat, raw_lon), (gdata['lat'], gdata['lon']))
                if dist < self.CLUSTER_RADIUS_M and dist < min_dist:
                    min_dist = dist
                    best_global_id = gid

            if best_global_id is not None:
                # BERSAGLIO ESISTENTE: Aggiorniamo la sua posizione facendo una media progressiva
                g_tgt = self.global_targets[best_global_id]
                N = g_tgt['obs_count']
                
                # Media pesata per stabilizzare le coordinate
                g_tgt['lat'] = (g_tgt['lat'] * N + raw_lat) / (N + 1)
                g_tgt['lon'] = (g_tgt['lon'] * N + raw_lon) / (N + 1)
                g_tgt['obs_count'] += 1
                g_tgt['last_seen'] = fls_stamp
                
                # Aggiorniamo il tipo e la confidenza fidandoci dell'ultima stima del detector
                g_tgt['type'] = raw_type
                g_tgt['confidence'] = raw_conf
            else:
                # NUOVO BERSAGLIO FONDATORE
                best_global_id = self.next_global_id
                self.next_global_id += 1
                self.global_targets[best_global_id] = {
                    'lat': raw_lat,
                    'lon': raw_lon,
                    'type': raw_type,
                    'confidence': raw_conf,
                    'obs_count': 1,
                    'last_seen': fls_stamp
                }

            # 4. AGGIORNAMENTO JSON FINALE DA PUBBLICARE
            # Sovrascriviamo il target_id dell'immagine col vero ID Globale della Mappa!
            target_data['target_id'] = best_global_id
            
            # Esportiamo le coordinate mediate/stabili invece di quelle grezze saltellanti
            target_data['target_lat'] = self.global_targets[best_global_id]['lat']
            target_data['target_lon'] = self.global_targets[best_global_id]['lon']
            
            # Info accessorie utili per chi legge
            target_data['global_obs_count'] = self.global_targets[best_global_id]['obs_count']
            
            target_data['range_m'] = round(math.sqrt(X_body**2 + Y_body**2), 2)
            target_data['bearing_deg'] = round(math.atan2(Y_body, X_body) * (180.0 / math.pi), 1)

            output_msg = String()
            output_msg.data = json.dumps(target_data)
            self.pub_localized.publish(output_msg)

            # 5. SALVATAGGIO TESTUALE NELLA CARTELLA "target_list"
            try:
                with open(self.log_file_path, "a") as f:
                    # json.dumps con indent=4 genera un file molto leggibile con a capo e spazi
                    f.write(json.dumps(target_data, indent=4))
                    f.write("\n" + "-"*50 + "\n") # Divisore per separare i messaggi
            except Exception as e:
                rospy.logwarn("Impossibile scrivere il file di log dei target: {}".format(e))

        except Exception as e:
            rospy.logerr("Errore in target_callback (Localizzazione): %s", str(e))

if __name__ == '__main__':
    try:
        node = FLSLocalizationNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass