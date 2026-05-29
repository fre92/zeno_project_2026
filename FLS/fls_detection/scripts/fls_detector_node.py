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
pub_img_saliency = None  # <--- NUOVO: Publisher per la mappa di salienza
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

    background = cv2.GaussianBlur(median, (101, 101), 0)

    bright = cv2.absdiff(median, background)

    _, binary = cv2.threshold(bright, 60, 255, cv2.THRESH_BINARY)

    # 3. Nuova sequenza morfologica
    # Prima OPEN: Elimina il rumore isolato (i puntini)
    kernel_open = np.ones((5, 5), np.uint8)
    binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel_open)

    # Poi CLOSE: Ricompatta le forme dei target e unisce i frammenti vicini
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
        
        # Filtro 1: Area del contorno vera e propria
        if area < 60: 
            continue
            
        # Filtro 2: Dimensioni minime assolute del Bounding Box
        if w < 5 or h < 5:
            continue
            
        # Filtro 3 (Opzionale ma utile): Area del Bounding Box
        bbox_area = w * h
        if bbox_area < 80:
            continue

        cx = x + w / 2.0
        cy = y + h / 2.0
        
        raw_detections.append({
            "x": x, "y": y, "w": w, "h": h, 
            "cx": cx, "cy": cy, "area": area, 
            "contour": c
        })

    # Ordiniamo da sinistra a destra per ottimizzare il ciclo successivo
    raw_detections = sorted(raw_detections, key=lambda d: d["x"])

    groups = []
    used = [False] * len(raw_detections)
    
    # Soglie di distanza in pixel per considerare due frammenti come uniti
    MERGE_DX = 90 
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
        # A) Manteniamo la BBox dritta per la Localizzazione
        mx1 = min([d["x"] for d in group])
        my1 = min([d["y"] for d in group])
        mx2 = max([d["x"] + d["w"] for d in group])
        my2 = max([d["y"] + d["h"] for d in group])
        mw = mx2 - mx1
        mh = my2 - my1
        
        # B) Creiamo il mega-contorno unendo tutti i pezzi del gruppo
        combined_contour = np.vstack([d["contour"] for d in group])
        
        # C) Calcoliamo il Rettangolo Orientato (minAreaRect)
        rect = cv2.minAreaRect(combined_contour)
        rect_w, rect_h = rect[1]
        
        # Estraiamo i 4 vertici del rettangolo storto per poterli disegnare
        box = cv2.boxPoints(rect)
        box = np.int32(box) 
        
        group_area = sum([d["area"] for d in group])
        num_parts = len(group)

        merged_targets.append({
            "bbox": [mx1, my1, mw, mh],  
            "rotated_box": box,          
            "rect_w": rect_w,            
            "rect_h": rect_h,
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

    # REGOLE TUBO: La precondizione fondamentale è che deve essere "grande"
    is_tubo = (
        150 <= area <= 3000 and 
        aspect >= 3.0  
        # (
        #     max_side >= 80 
        # )
    )

    is_boa = (

        30 <= area <= 145 and
        aspect <= 1.5
    )

    if is_tubo:
        return "tubo_probabile"
    elif is_boa:
        return "boa_probabile"

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
        raw_class = det["raw_class"] 
        
        best_track = None
        best_dist = float('inf')

        # 1. Associazione spaziale
        for t in tracks:
            dist = distance_points((det_cx, det_cy), (t["cx"], t["cy"]))
            if dist < TRACK_MATCH_DIST and dist < best_dist:
                best_dist = dist
                best_track = t

        # 2. Aggiornamento o Creazione
        if best_track is not None:
            best_track["cx"] = det_cx
            best_track["cy"] = det_cy
            best_track["last_seen"] = frame_id
            best_track["history"].append(raw_class)
            
            if len(best_track["history"]) > TRACK_HISTORY_LEN:
                best_track["history"].pop(0)
        else:
            best_track = {
                "id": next_track_id,
                "cx": det_cx,
                "cy": det_cy,
                "last_seen": frame_id,
                "history": [raw_class]
            }
            tracks.append(best_track)
            next_track_id += 1

        # 3. Calcolo della Confidenza
        tubo_count = best_track["history"].count("tubo_probabile")
        boa_count = best_track["history"].count("boa_probabile")
        total_observations = len(best_track["history"])


        if tubo_count >= boa_count:
            final_class = "tubo_probabile"
            conf = float(tubo_count) / float(total_observations)
        else:
            final_class = "boa_probabile"
            conf = float(boa_count) / float(total_observations)

        det["target_id"] = best_track["id"]
        det["final_class"] = final_class
        det["confidence"] = conf
        
        assigned_detections.append(det)

    # 5. Pulizia dei track vecchi
    tracks = [t for t in tracks if (frame_id - t["last_seen"]) <= MAX_LOST_FRAMES]

    return assigned_detections

# ============================================================
# CALLBACK ROS
# ============================================================
def fls_callback(msg):
    global frame_count
    global pub_json, pub_img_binary, pub_img_contours, pub_img_saliency, bridge

    frame_count += 1
    fls_stamp_sec = msg.header.stamp.to_sec()

    # 1. Decodifica Immagine
    np_arr = np.fromstring(msg.data, np.uint8)
    img = cv2.imdecode(np_arr, cv2.IMREAD_GRAYSCALE)

    if img is None:
        rospy.logwarn("Impossibile decodificare immagine FLS.")
        return

    # 2. Pipeline di base
    # NOTE: bright è la nostra "Saliency Map" (valori in scala di grigi dei bersagli evidenziati)
    bright, binary = compute_base_pipeline(img)

    if binary is None:
        return

    # 3. Estrazione e Raggruppamento
    merged_targets = extract_and_merge_targets(binary)
    out_img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    
    target_id = 0

    # 4. Classificazione Grezza
    raw_detections = []
    for t in merged_targets:
        x, y, w, h = t["bbox"]
        cx = x + w / 2.0
        cy = y + h / 2.0
        
        raw_class = classify_target(t["rect_w"], t["rect_h"], t["area"], t["parts"])
        
        raw_detections.append({
            "bbox": [x, y, w, h],
            "rotated_box": t["rotated_box"], 
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
        rotated_box = t["rotated_box"]
        
        target_id = t["target_id"]
        target_class = t["final_class"]
        conf = t["confidence"]
        
        if target_class == "boa_probabile":
            color = (0, 255, 0)
        else:
            color = (255, 0, 0)

        cv2.drawContours(out_img, [rotated_box], 0, color, 2)
        
        top_y = min([pt[1] for pt in rotated_box])
        left_x = min([pt[0] for pt in rotated_box])
        
        label = "ID:{} {} {:.0f}%".format(target_id, target_class[:4].upper(), conf * 100)
        cv2.putText(out_img, label, (left_x, max(0, top_y - 5)), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

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

    # 6. Pubblicazione su RViz
    try:
        # Pubblica la Salience Map (bright)
        if pub_img_saliency.get_num_connections() > 0:
            msg_sal = bridge.cv2_to_imgmsg(bright, encoding="mono8")
            msg_sal.header = msg.header 
            pub_img_saliency.publish(msg_sal)

        if pub_img_binary.get_num_connections() > 0:
            msg_bin = bridge.cv2_to_imgmsg(binary, encoding="mono8")
            msg_bin.header = msg.header
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
    global pub_json, pub_img_binary, pub_img_contours, pub_img_saliency, bridge

    ensure_dir(OUTPUT_DIR)
    rospy.init_node("fls_ros_reader_clean")

    bridge = CvBridge()

    rospy.loginfo("Nodo FLS base avviato.")
    rospy.loginfo("Topic immagine: {}".format(IMAGE_TOPIC))
    
    # Publisher JSON
    pub_json = rospy.Publisher('/perception/target_json', String, queue_size=10)
    
    # Nuovi Publisher per il debug visivo
    pub_img_saliency = rospy.Publisher('/perception/debug/saliency', Image, queue_size=1)
    pub_img_binary   = rospy.Publisher('/perception/debug/binary', Image, queue_size=1)
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