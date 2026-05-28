#!/bin/bash

# ==============================================================================
# TRAP: Cattura il Ctrl+C e chiude in modo pulito tutti i processi in background
# ==============================================================================
trap "echo -e '\n[!] Chiusura di tutti i nodi e salvataggio della bag...'; trap - SIGTERM && kill -- -$$" SIGINT SIGTERM EXIT

echo "========================================="
echo "   Avvio Sistema Percezione FLS AUV      "
echo "========================================="

# 1. Avvia i nodi Python tramite il file di launch (in background)
echo "[1/4] Avvio dei nodi Detection e Localization..."
roslaunch fls_detection FLS_launch.launch &
sleep 3 # Diamo tempo ai nodi di inizializzarsi

# 3. Avvia la registrazione dei dati in una nuova bag (in background)
# Nota: La bag verrà salvata nella cartella in cui esegui run.sh
echo "[3/4] Inizio registrazione della bag (output_mappa.bag)..."
rosbag record -O output_mappa.bag \
    /perception/target_localized_json \
    /nav_status \
    /drivers/fls_sim/cartesian_image/compressed &
sleep 1

# 4. Riproduci la bag simulata
# Sostituisci "localizzazione_7.bag" con il percorso effettivo della tua bag di test
echo "[4/4] Riproduzione rosbag in corso..."
rosbag play localizzazione_7.bag

# Quando la bag finisce (o premi Ctrl+C), lo script aspetta che tutto si spenga
echo "========================================="
echo " Riproduzione terminata. In chiusura...  "
echo "========================================="
wait