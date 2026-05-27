#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import os
import cv2
import numpy as np
import rospy
import json
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge, CvBridgeError

# ============================================================
# CONFIGURAZIONE GENERALE
# ============================================================
OUTPUT_DIR = "/home/student/progetto_fls_clean/output/ros_test"
IMAGE_TOPIC = "/drivers/fls_sim/cartesian_image/compressed"

# Variabili Globali
frame_count = 0
pub_json = None
pub_img_binary = None
pub_img_contours = None
bridge = None

# ============================================================
# CONFIGURAZIONE TRACKING E CONFIDENZA
# ============================================================
TRACK_MATCH_DIST = 100.0  # Distanza massima in pixel per associare un target tra due frame
TRACK_HISTORY_LEN = 10    # Finestra di memoria (es. gli ultimi 10 frame)
MAX_LOST_FRAMES = 5       # Quanti frame un target può "sparire" prima di essere dimenticato

tracks = []
next_track_id = 1

# ============================================================
# UTILITY
# ============================================================
def ensure_dir(path):
    if not os.path.exists(path):
        os.makedirs(path)

def find_contours_compat(binary_img):
    contours_result = cv2.findContours(
        binary_img,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )
    if len(contours_result) == 3:
        return contours_result[1]
    return contours_result[0]

# ============================================================
# PRE-PROCESSING IMMAGINE FLS
# ============================================================
def keep_largest_component(mask):
    """
    Tiene solo la componente principale della maschera del FOV sonar.
    """
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, 8)
    if num_labels <= 1:
        return mask

    best_label = 1
    best_area = stats[1, cv2.CC_STAT_AREA]

    for label in range(2, num_labels):
        area = stats[label, cv2.CC_STAT_AREA]
        if area > best_area:
            best_area = area
            best_label = label

    clean = np.zeros_like(mask)
    clean[labels == best_label] = 255
    return clean

def compute_base_pipeline(img):
    """
    Pipeline base essenziale per l'isolamento dei target.
    """
    # 1. Filtro Mediano più aggressivo (da 3 a 5)
    # median = cv2.medianBlur(img, 5) 

    median = cv2.GaussianBlur(img, (5, 5), 0)

    background = cv2.GaussianBlur(median, (51, 51), 0)

    bright = cv2.absdiff(median,background)

    _, binary = cv2.threshold(bright, 65, 255, cv2.THRESH_BINARY)


    # diff = median.astype("int16") - background.astype("int16")
    # diff[diff < 0] = 0
    # diff = diff.astype("uint8")

    # low = 10
    # diff[diff < low] = 0

    # bright = cv2.normalize(diff, None, 0, 255, cv2.NORM_MINMAX)

    # mask = np.zeros_like(img, dtype=np.uint8)
    # mask[img > 5] = 255

    # kernel_mask = np.ones((5, 5), np.uint8)
    # mask = cv2.erode(mask, kernel_mask, iterations=1)
    # mask = keep_largest_component(mask)

    # bright[mask == 0] = 0

    # valid_pixels = bright[(mask > 0) & (bright > 0)]
    # if valid_pixels.size == 0:
    #     return None, None

    # # 2. Percentile più selettivo (da 93.0 a 95.0)
    # perc = 98.0 
    # thr_value = np.percentile(valid_pixels, perc)

    # _, binary = cv2.threshold(bright, thr_value, 255, cv2.THRESH_BINARY)

    # h_img, w_img = binary.shape
    # cut_y = int(0.86 * h_img)
    # binary[cut_y:, :] = 0

    # 3. Nuova sequenza morfologica
    # Prima OPEN: Elimina il rumore isolato (i puntini)
    # prova con 2,2 da 3,3 -> risultato migliore
    kernel_open = np.ones((2, 2), np.uint8)
    binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel_open)

    # Poi CLOSE: Ricompatta le forme dei target e unisce i frammenti vicini
    # modificato da 5,5 a 11,11 per migliorare la definizione dei blob
    # seconda prova con 15,15 -> miglior risultato
    # terza prova con 30,30
    kernel_close = np.ones((27, 27), np.uint8)
    binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel_close)

    return bright, binary

# ============================================================
# RAGGRUPPAMENTO TARGET (MERGING)
# ============================================================
def extract_and_merge_targets(binary_img):
    contours = find_contours_compat(binary_img)
    raw_detections = []

    # 1. Estrazione Grezza e Filtro Dimensionale Forte
    for c in contours:
        area = cv2.contourArea(c)
        x, y, w, h = cv2.boundingRect(c)
        
        # --- NUOVO FILTRO ---
        # Scartiamo tutto ciò che è palesemente troppo piccolo per essere 
        # un pezzo rilevante di tubo o una boa, DOPO l'espansione morfologica.
        
        # Filtro 1: Area del contorno vera e propria
        if area < 60: 
            continue
            
        # Filtro 2: Dimensioni minime assolute del Bounding Box
        # Se un blob è largo 3 pixel e alto 3, è rumore.
        if w < 5 or h < 5:
            continue
            
        # Filtro 3 (Opzionale ma utile): Area del Bounding Box
        bbox_area = w * h
        if bbox_area < 80:
            continue
        # --------------------

        cx = x + w / 2.0
        cy = y + h / 2.0
        
        raw_detections.append({
            "x": x, "y": y, "w": w, "h": h, 
            "cx": cx, "cy": cy, "area": area
        })

    # Ordiniamo da sinistra a destra per ottimizzare il ciclo successivo
    raw_detections = sorted(raw_detections, key=lambda d: d["x"])

    groups = []
    used = [False] * len(raw_detections)
    
    # Soglie di distanza in pixel per considerare due frammenti come uniti
    MERGE_DX = 90 # da 55 a 70 per migliorare merge tubi
    MERGE_DY = 150

    # 2. Clustering (Raggruppamento a catena)
    for i in range(len(raw_detections)):
        if used[i]:
            continue

        group = [raw_detections[i]]
        used[i] = True
        changed = True

        # Continua a cercare finché il gruppo non smette di inglobare nuovi pezzi
        while changed:
            changed = False
            for j in range(len(raw_detections)):
                if used[j]:
                    continue
                    
                det = raw_detections[j]
                close_to_group = False

                for g in group:
                    dx = abs(det["cx"] - g["cx"])
                    dy = abs(det["cy"] - g["cy"])
                    
                    if dx < MERGE_DX and dy < MERGE_DY:
                        close_to_group = True
                        break

                if close_to_group:
                    group.append(det)
                    used[j] = True
                    changed = True

        groups.append(group)

    # 3. Fusione e creazione Bounding Box Finale
    merged_targets = []
    for group in groups:
        mx1 = min([d["x"] for d in group])
        my1 = min([d["y"] for d in group])
        mx2 = max([d["x"] + d["w"] for d in group])
        my2 = max([d["y"] + d["h"] for d in group])

        mw = mx2 - mx1
        mh = my2 - my1
        
        # Salviamo sia l'area totale che il numero di pezzi (ci servirà per classificare i tubi frammentati)
        group_area = sum([d["area"] for d in group])
        num_parts = len(group)

        merged_targets.append({
            "bbox": [mx1, my1, mw, mh],
            "area": group_area,
            "parts": num_parts
        })

    return merged_targets
# ============================================================
# CLASSIFICAZIONE TARGET
# ============================================================
def classify_target(w, h, area, parts):
    """
    Classificazione euristica stringente basata su Bounding Box fusa.
    """
    max_side = max(w, h)
    min_side = min(w, h)
    
    if min_side == 0:
        return "unknown"
        
    aspect = float(max_side) / float(min_side)

    # 1. REGOLE BOA: Molto rigorose sulla compattezza e con un limite massimo di grandezza
    is_boa = (
        aspect <= 1.5 and          # Deve essere quadrata/tonda (stretto da 1.8 a 1.5)
        parts <= 2 and             # Al massimo 2 frammenti molto vicini
        20 <= area <= 100 and      # Area contenuta: scarta rumore minuscolo e oggetti troppo grandi
        max_side <= 60             # Nessun lato deve essere troppo lungo
    )

    # 2. REGOLE TUBO: La precondizione fondamentale è che deve essere "grande"
    is_tubo = (
        area >= 120 and            # SCUDO ANTI-RUMORE: Un tubo non può avere un'area minuscola
        (
            aspect >= 2.5 or       # O è palesemente allungato
            parts >= 3 or          # O è frammentato in molti pezzi (es. 4-5)
            max_side >= 80         # O ha una lunghezza assoluta notevole
        )
    )

    # 3. ASSEGNAZIONE
    if is_tubo and not is_boa:
        return "tubo_probabile"
    else:
        return "boa_probabile"
    # else:
    #     # Tutto il rumore filiforme, i frammentini isolati e le forme miste finiscono qui
    #     return "unknown"

# ============================================================
# TRACKING TEMPORALE E CALCOLO CONFIDENZA
# ============================================================
def distance_points(p1, p2):
    dx = p1[0] - p2[0]
    dy = p1[1] - p2[1]
    return np.sqrt(dx * dx + dy * dy)

def update_tracks_and_confidence(detections, frame_id):
    global tracks, next_track_id

    assigned_detections = []

    for det in detections:
        det_cx, det_cy = det["cx"], det["cy"]
        raw_class = det["raw_class"] # Quella calcolata nel singolo frame
        
        best_track = None
        best_dist = float('inf')

        # 1. Associazione spaziale (chi è il target più vicino?)
        for t in tracks:
            dist = distance_points((det_cx, det_cy), (t["cx"], t["cy"]))
            if dist < TRACK_MATCH_DIST and dist < best_dist:
                best_dist = dist
                best_track = t

        # 2. Aggiornamento o Creazione
        if best_track is not None:
            # Aggiorna posizione e memoria
            best_track["cx"] = det_cx
            best_track["cy"] = det_cy
            best_track["last_seen"] = frame_id
            best_track["history"].append(raw_class)
            
            # Manteniamo la finestra scorrevole lunga al massimo TRACK_HISTORY_LEN
            if len(best_track["history"]) > TRACK_HISTORY_LEN:
                best_track["history"].pop(0)
        else:
            # Nuovo target rilevato
            best_track = {
                "id": next_track_id,
                "cx": det_cx,
                "cy": det_cy,
                "last_seen": frame_id,
                "history": [raw_class]
            }
            tracks.append(best_track)
            next_track_id += 1

        # 3. Calcolo della Confidenza (La tua logica 8 su 10 = 80%)
        tubo_count = best_track["history"].count("tubo_probabile")
        boa_count = best_track["history"].count("boa_probabile")
        total_observations = len(best_track["history"])

        if tubo_count >= boa_count:
            final_class = "tubo_probabile"
            conf = float(tubo_count) / float(total_observations)
        else:
            final_class = "boa_probabile"
            conf = float(boa_count) / float(total_observations)

        # 4. Compilazione del target finale per questo frame
        det["target_id"] = best_track["id"]
        det["final_class"] = final_class
        det["confidence"] = conf
        
        assigned_detections.append(det)

    # 5. Pulizia dei track vecchi (dimentica chi non si fa vivo da un po')
    tracks = [t for t in tracks if (frame_id - t["last_seen"]) <= MAX_LOST_FRAMES]

    return assigned_detections

# ============================================================
# CALLBACK ROS
# ============================================================
def fls_callback(msg):
    global frame_count
    global pub_json, pub_img_binary, pub_img_contours, bridge

    frame_count += 1
    fls_stamp_sec = msg.header.stamp.to_sec()

    # 1. Decodifica Immagine
    np_arr = np.fromstring(msg.data, np.uint8)
    img = cv2.imdecode(np_arr, cv2.IMREAD_GRAYSCALE)

    if img is None:
        rospy.logwarn("Impossibile decodificare immagine FLS.")
        return

    # 2. Pipeline di base
    bright, binary = compute_base_pipeline(img)

    if binary is None:
        return

    # Sostituisci la parte del callback dal punto "3. Estrazione contorni base" in poi con questo:

    # 3. Estrazione e Raggruppamento
    merged_targets = extract_and_merge_targets(binary)
    out_img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    
    target_id = 0

    # ... (codice precedente: estrazione e raggruppamento) ...

    # 4. Classificazione Grezza
    raw_detections = []
    for t in merged_targets:
        x, y, w, h = t["bbox"]
        cx = x + w / 2.0
        cy = y + h / 2.0
        
        raw_class = classify_target(w, h, t["area"], t["parts"])
        
        raw_detections.append({
            "bbox": [x, y, w, h],
            "cx": cx, "cy": cy,
            "area": t["area"],
            "parts": t["parts"],
            "raw_class": raw_class
        })

    # 5. Tracking e Confidenza Statistica
    final_targets = update_tracks_and_confidence(raw_detections, frame_count)

    # 6. Esportazione JSON e Disegno Debug
    for t in final_targets:
        x, y, w, h = t["bbox"]
        target_id = t["target_id"]
        target_class = t["final_class"]
        conf = t["confidence"]
        
        if target_class == "boa_probabile":
            color = (0, 255, 0)
        else:
            color = (255, 0, 0)

        # Disegno su RViz
        cv2.rectangle(out_img, (x, y), (x + w, y + h), color, 2)
        
        # Mostriamo ID, Classe e la Confidenza in percentuale
        label = "ID:{} {} {:.0f}%".format(target_id, target_class[:4].upper(), conf * 100)
        cv2.putText(out_img, label, (x, max(0, y - 5)), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

        # Creazione JSON
        target_data = {
            "target_id": target_id,
            "sensor": "FLS",
            "frame": frame_count,
            "type": target_class,
            "confidence": round(conf, 2),
            "area": t["area"],
            "parts": t["parts"],
            "center_px": [int(t["cx"]), int(t["cy"])],
            "bbox_px": [x, y, w, h],
            "fls_stamp_sec": fls_stamp_sec
        }
        
        if pub_json is not None:
            msg_out = String()
            msg_out.data = json.dumps(target_data)
            pub_json.publish(msg_out)

    # 5. Salvataggio Immagini nelle cartelle separate
    dir_binary = os.path.join(OUTPUT_DIR, "frames_binary")
    dir_candidates = os.path.join(OUTPUT_DIR, "frames_candidates")
    
    ensure_dir(dir_binary)
    ensure_dir(dir_candidates)
    
    cv2.imwrite(os.path.join(dir_binary, "frame_{:06d}_binary.jpg".format(frame_count)), binary)
    cv2.imwrite(os.path.join(dir_candidates, "frame_{:06d}_candidates.jpg".format(frame_count)), out_img)

    # 6. Pubblicazione su RViz (solo se c'è qualcuno iscritto al topic)
    try:
        if pub_img_binary.get_num_connections() > 0:
            msg_bin = bridge.cv2_to_imgmsg(binary, encoding="mono8")
            msg_bin.header = msg.header # Mantiene stamp e frame_id originali
            pub_img_binary.publish(msg_bin)

        if pub_img_contours.get_num_connections() > 0:
            msg_cont = bridge.cv2_to_imgmsg(out_img, encoding="bgr8")
            msg_cont.header = msg.header
            pub_img_contours.publish(msg_cont)
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))

    if frame_count % 10 == 0:
        rospy.loginfo("FLS frame={} | Trovati {} contorni grezzi".format(frame_count, target_id))

# ============================================================
# MAIN
# ============================================================
def main():
    global pub_json, pub_img_binary, pub_img_contours, bridge

    ensure_dir(OUTPUT_DIR)
    rospy.init_node("fls_ros_reader_clean")

    bridge = CvBridge()

    rospy.loginfo("Nodo FLS base avviato. Ripulito da tracking e classificazione.")
    rospy.loginfo("Topic immagine: {}".format(IMAGE_TOPIC))
    
    # Publisher JSON
    pub_json = rospy.Publisher('/perception/target_json', String, queue_size=10)
    
    # Nuovi Publisher per il debug visivo
    pub_img_binary = rospy.Publisher('/perception/debug/binary', Image, queue_size=1)
    pub_img_contours = rospy.Publisher('/perception/debug/contours', Image, queue_size=1)

    rospy.Subscriber(
        IMAGE_TOPIC,
        CompressedImage,
        fls_callback,
        queue_size=1
    )

    rospy.spin()

if __name__ == "__main__":
    main()