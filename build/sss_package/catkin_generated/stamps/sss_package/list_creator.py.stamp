#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-

# ============================================================
# IMPORT LIBRERIE
# ============================================================
import rospy
import cv2          # OpenCV
import numpy as np
import os
import rospkg
from marta_msgs.msg import Altitude
from marta_msgs.msg import NavStatus
from sss_package.msg import ImageMetadata
from sensor_msgs.msg import Image as RosImage
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError


# ============================================================
# CLASSE PRINCIPALE NODO LIST CREATOR
# ============================================================
class ListCreatorNode:

    def __init__(self):

	print("Initialization\n")

        # inizializzare subscriber/publisher
        rospy.Subscriber('/waterfall_creator', ImageMetadata, self.list_callback)
        self.pub_list = rospy.Publisher('target_list', String, queue_size=200)
        self.pub_detected_objects = rospy.Publisher('detected_objects', ImageMetadata, queue_size=200)

        # inizializzare CvBridge
        self.bridge = CvBridge()

        # definire parametri Zeno
        self.latitude   = None
        self.longitude  = None
        self.altitude   = None
        self.yaw_angle  = None
        self.omega_body = None
        self.latest_nav = NavStatus()

        # definire parametri filtri
        self.log_filter_scale        = float(rospy.get_param('~log_filter_scale', 255.0))
        self.median_kernel_size      = int(rospy.get_param('~median_kernel_size', 3))
        self.gaussian_kernel_size    = int(rospy.get_param('~gaussian_kernel_size', 5))
        self.gaussian_sigma          = float(rospy.get_param('~gaussian_sigma', 0.0))
        self.clahe_clip_limit        = float(rospy.get_param('~clahe_clip_limit', 2.0))
        self.clahe_tile_grid_size    = int(rospy.get_param('~clahe_tile_grid_size', 8))
        self.percentile_low          = float(rospy.get_param('~percentile_low', 2.0))
        self.percentile_high         = float(rospy.get_param('~percentile_high', 98.0))

        # definire parametri morfologici
        self.bright_morph_first_open_kernel_size  = int(rospy.get_param('~bright_morph_first_open_kernel_size', 3))	# pulizia iniziale leggera
        self.bright_morph_close_kernel_size 	  = int(rospy.get_param('~bright_morph_close_kernel_size', 5))		# chiude piccoli buchi all interno deli oggetti
        self.bright_morph_final_open_kernel_size  = int(rospy.get_param('~bright_morph_final_open_kernel_size', 5))	# pulizia finale piu forte
        self.bright_morph_final_close_kernel_size = int(rospy.get_param('~bright_morph_final_close_kernel_size', 9))	# riconnette frammenti dopo la pulizia finale
        self.bright_morph_iterations        	  = int(rospy.get_param('~bright_morph_iterations', 1))
        self.dark_morph_open_kernel_size   	  = int(rospy.get_param('~dark_morph_open_kernel_size', 3))		# rimuove piccoli blob isolati dopo aver riconnesso le ombre
        self.dark_morph_first_close_kernel_width  = int(rospy.get_param('~dark_morph_first_close_kernel_width', 5))	# connette frammenti prima della pulizia
        self.dark_morph_first_close_kernel_height = int(rospy.get_param('~dark_morph_first_close_kernel_height', 3))
        self.dark_morph_final_close_kernel_width  = int(rospy.get_param('~dark_morph_final_close_kernel_width', 9))	# riconnette frammenti rimasti dopo opening
        self.dark_morph_final_close_kernel_height = int(rospy.get_param('~dark_morph_final_close_kernel_height', 3))
        self.dark_morph_iterations   		  = int(rospy.get_param('~dark_morph_iterations', 1))

        # definire parametri saliency
        self.saliency_percentile   = rospy.get_param('~saliency_percentile', 70.0)
        self.cfar_train_cells      = rospy.get_param('~cfar_train_cells', 18)
        self.cfar_guard_cells      = rospy.get_param('~cfar_guard_cells', 5)
        self.cfar_rank_percentile  = rospy.get_param('~cfar_rank_percentile', 60.0)
        self.cfar_threshold_scale  = rospy.get_param('~cfar_threshold_scale', 1.00)
        self.cfar_threshold_offset = rospy.get_param('~cfar_threshold_offset', 2.0)
        self.cfar_detect_dark      = rospy.get_param('~cfar_detect_dark', True)
        self.cfar_pfa              = rospy.get_param('~cfar_pfa', 1.0e-3)

        # definire parametri binarizzazione
        self.bright_percentile = rospy.get_param('~bright_percentile', 85.0)
        self.dark_percentile   = rospy.get_param('~dark_percentile', 25.0)


        # definire parametri detection e classification
        self.meters_per_pixel_x            = float(rospy.get_param('~meters_per_pixel_x', 0.025))			# risoluzione across-track
        self.meters_per_pixel_y            = float(rospy.get_param('~meters_per_pixel_y', 0.125))			# risoluzioen along-track
        self.min_object_area_px            = int(rospy.get_param('~min_object_area_px', 40))				# 40 << minima area dell oggetto stimata
        self.min_shadow_area_px            = int(rospy.get_param('~min_shadow_area_px', 20))				# 20 << minima area dell ombra stimata
        self.max_shadow_alongtrack_gap_m   = float(rospy.get_param('~max_shadow_alongtrack_gap_m', 2.5))
        self.max_shadow_acrosstrack_gap_m  = float(rospy.get_param('~max_shadow_acrosstrack_gap_m', 8.0))
        self.shadow_height_scale           = float(rospy.get_param('~shadow_height_scale', 1.0))
        self.min_classification_confidence = float(rospy.get_param('~min_classification_confidence', 0.35))

        # 1. Ottieni il percorso base del pacchetto
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('sss_package') 

        # 2. Definisci la cartella 'results' principale (sovrascrivibile da parametro)
        default_results_dir = os.path.join(pkg_path, 'results')
        results_dir = rospy.get_param('~results_folder', default_results_dir)

        # 3. Definisci i percorsi di tutte le sottocartelle
        self.filtered_image_folder = os.path.join(results_dir, "2_filtered_images")
        self.saliency_image_folder = os.path.join(results_dir, "3_saliency_images")
        self.saliency_binary_image_folder = os.path.join(results_dir, "4_saliency_binary_images")

        # Cartelle Binary maps (e relative sottocartelle)
        self.binary_maps_image_folder = os.path.join(results_dir, "5_binary_maps_images")
        self.bright_map_folder = os.path.join(self.binary_maps_image_folder, "bright_maps")
        self.dark_map_folder = os.path.join(self.binary_maps_image_folder, "dark_maps")

        # Cartelle Binary AND salient maps (e relative sottocartelle)
        self.binary_and_salient_maps_image_folder = os.path.join(results_dir, "6_binary_and_salient_maps_images")
        self.bright_and_salient_map_folder = os.path.join(self.binary_and_salient_maps_image_folder, "bright_maps")
        self.dark_and_salient_map_folder = os.path.join(self.binary_and_salient_maps_image_folder, "dark_maps")

        # Cartelle Classification
        self.classification_image_folder = os.path.join(results_dir, "7_classification_images")
        self.classification_text_folder = os.path.join(results_dir, "8_classification_texts")

        # 4. Crea tutte le cartelle in un colpo solo tramite un ciclo
        folders_to_create = [
            self.filtered_image_folder,
            self.saliency_image_folder,
            self.saliency_binary_image_folder,
            self.binary_maps_image_folder,
            self.bright_map_folder,
            self.dark_map_folder,
            self.binary_and_salient_maps_image_folder,
            self.bright_and_salient_map_folder,
            self.dark_and_salient_map_folder,
            self.classification_image_folder,
            self.classification_text_folder
        ]

        for folder in folders_to_create:
            if not os.path.exists(folder):
                os.makedirs(folder)
        
        self.image_index = 0

        pass

# ________________________________________________________________________________________________________________________________


    # ========================================================
    # LETTURA ALTITUDE
    # ========================================================
    def read_altitude_from_bag(self, msg):
        # estrarre topic altitude
        self.altitude = msg.altitude
        pass


    # ========================================================
    # LETTURA NAV_STATUS
    # ========================================================
    def read_nav_status_from_bag(self, msg):
        # estrarre topic nav_status
        self.latitude   = msg.position.latitude
        self.longitude  = msg.position.longitude
        self.yaw_angle  = msg.orientation.yaw            # radianti
        self.omega_body = msg.omega_body.z
        self.latest_nav = msg
        pass


# ________________________________________________________________________________________________________________________________



    # ========================================================
    # CALLBACK PER CREAZIONE LISTA
    # ========================================================
    def list_callback(self, msg):
        # 1. ROS Image -> OpenCV grayscale image
        image = self.ros_image_to_cv2(msg.image)
        if image is None:
            return

        # 2. filtering pipeline	
	#image = self.apply_log_filter(image)
	#image = self.apply_gaussian_filter(image)
	#image = self.apply_median_filter(image)
	#image = self.apply_clahe_filter(image)
	#image = self.apply_percentile_filter(image)

        # 3. saliency
        # OpenCV spectral residual saliency
	#saliency_map = self.compute_spectral_residual_saliency(image)

	# 1D OS-CFAR saliency
	saliency_map = self.compute_os_cfar_1d_saliency(image)

        # 4. bright/dark binary maps
	saliency_binary_map, bright_map, dark_map, bright_and_salient_map, dark_and_salient_map = self.create_bright_dark_maps(image, saliency_map)

        # 5. classificazione oggetti usando oggetto luminoso + ombra scura
        classifications = self.detect_and_classify_objects(bright_and_salient_map, dark_and_salient_map)

        # 6. salvataggio risultati
        image_index = self.image_index
	self.save_filtered_image(image, image_index)
        self.save_saliency_image(saliency_map, image_index)
        self.save_saliency_binary_image(saliency_binary_map, image_index)
        self.save_bright_map_image(bright_map, image_index)
        self.save_dark_map_image(dark_map, image_index)
        self.save_bright_and_salient_map_image(bright_and_salient_map, image_index)
        self.save_dark_and_salient_map_image(dark_and_salient_map, image_index)
        self.save_classification_image(image, classifications, image_index)
        self.save_classification_text(classifications, image_index)
        detected_msg = self.build_detected_objects_message(msg, classifications)
        self.pub_detected_objects.publish(detected_msg)
        self.pub_list.publish(String(data=str(classifications)))
        self.image_index += 1

        return classifications

    def build_detected_objects_message(self, source_msg, classifications):
        detected_msg = ImageMetadata()
        detected_msg.header = source_msg.header
        detected_msg.image = source_msg.image
        detected_msg.ping_indices = source_msg.ping_indices
        detected_msg.ping_stamps = source_msg.ping_stamps
        detected_msg.nav_statuses = source_msg.nav_statuses
        detected_msg.nav_valid = source_msg.nav_valid
        detected_msg.altitudes = source_msg.altitudes
        detected_msg.altitude_valid = source_msg.altitude_valid

        detected_msg.object_classes = []
        detected_msg.object_confidences = []
        detected_msg.object_centroid_x_px = []
        detected_msg.object_centroid_y_px = []
        detected_msg.object_bbox_x_px = []
        detected_msg.object_bbox_y_px = []
        detected_msg.object_bbox_width_px = []
        detected_msg.object_bbox_height_px = []

        for detection in classifications:
            obj = detection['object']
            bbox = obj['bbox_px']
            centroid = obj['centroid_px']
            detected_msg.object_classes.append(detection['classification'])
            detected_msg.object_confidences.append(float(detection['confidence']))
            detected_msg.object_centroid_x_px.append(float(centroid[0]))
            detected_msg.object_centroid_y_px.append(float(centroid[1]))
            detected_msg.object_bbox_x_px.append(int(bbox[0]))
            detected_msg.object_bbox_y_px.append(int(bbox[1]))
            detected_msg.object_bbox_width_px.append(int(bbox[2]))
            detected_msg.object_bbox_height_px.append(int(bbox[3]))

        return detected_msg

    # ========================================================
    # CALLBACK PER ALTITUDE DI ZENO
    # ========================================================
    def altitude_callback(self, msg):
        # 1. estrai altitude
        self.read_altitude_from_bag(msg)
	# 2. plot altitude        
	#self.store_altitude_sample()
        pass


    # ========================================================
    # CALLBACK PER NAV_STATUS DI ZENO
    # ========================================================
    def zeno_callback(self, msg):
        # 1. estrai latitudine, longitudine e yaw dal nav_status
        self.read_nav_status_from_bag(msg)
        # 2. plot della traiettoria
        #self.plot_auv_trajectory(nav_status_stream)
        pass


# ________________________________________________________________________________________________________________________________

    # ========================================================
    # PREPROCESSING IMMAGINE
    # ========================================================
    def ros_image_to_cv2(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        except CvBridgeError as exc:
            rospy.logerr("Errore conversione ROS Image -> OpenCV: {}".format(exc))
            return None
        return np.asarray(image, dtype=np.uint8)

    def normalize_uint8(self, image):
        image = np.asarray(image)
        if image.dtype == np.uint8:
            return image
        image = image.astype(np.float32)
        return cv2.normalize(image, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)


    def apply_log_filter(self, image):
        image = self.normalize_uint8(image).astype(np.float32)
        log_image = np.log1p(image)
        max_log   = float(np.max(log_image))
        if max_log <= 0.0:
            return np.zeros_like(image, dtype=np.uint8)
        return np.clip((log_image / max_log) * self.log_filter_scale, 0, 255).astype(np.uint8)

    def apply_median_filter(self, image):
        kernel_size = int(self.median_kernel_size)
        if kernel_size < 1:
            return image
        if kernel_size % 2 == 0:
            kernel_size += 1
        return cv2.medianBlur(self.normalize_uint8(image), kernel_size)

    def apply_gaussian_filter(self, image):
        kernel_size = int(self.gaussian_kernel_size)
        if kernel_size < 1:
            return image
        if kernel_size % 2 == 0:
            kernel_size += 1
        return cv2.GaussianBlur(self.normalize_uint8(image), (kernel_size, kernel_size), self.gaussian_sigma)

    def apply_clahe_filter(self, image):
        tile_grid_size = int(self.clahe_tile_grid_size)
        if tile_grid_size < 1:
            tile_grid_size = 1
	clahe = cv2.createCLAHE(clipLimit = self.clahe_clip_limit, tileGridSize = (tile_grid_size, tile_grid_size))
        return clahe.apply(self.normalize_uint8(image))

    def apply_percentile_filter(self, image):
        image = self.normalize_uint8(image).astype(np.float32)
        low_percentile  = max(0.0, min(100.0, float(self.percentile_low)))
        high_percentile = max(0.0, min(100.0, float(self.percentile_high)))
        if high_percentile <= low_percentile:
            return image.astype(np.uint8)

        low_value  = np.percentile(image, low_percentile)
        high_value = np.percentile(image, high_percentile)
        if high_value <= low_value:
            return image.astype(np.uint8)

        image = np.clip(image, low_value, high_value)
        image = (image - low_value) / (high_value - low_value)
        return np.clip(image * 255.0, 0, 255).astype(np.uint8)

    # ========================================================
    # SALIENCY MAP
    # ========================================================
    def compute_spectral_residual_saliency(self, image):
	# normalizzare immagine in uint8 [0, 255]
        image = self.normalize_uint8(image)

	# creare mappa di salienza
        saliency = cv2.saliency.StaticSaliencySpectralResidual_create()
        success, saliency_map = saliency.computeSaliency(image)
        if not success:
            return None
        # convertire da float [0,1] a uint8 [0,255]
        saliency_map = (saliency_map * 255).astype(np.uint8)
        return saliency_map


    def compute_os_cfar_1d_saliency(self, image):
	# normalizzare immagine in uint8 [0, 255]
        image = self.normalize_uint8(image)

        # Calcolare la risposta CFAR sui ritorni luminosi dell'immagine originale.
        bright_response = self.compute_os_cfar_1d_response(image)

        # Se non si vogliono rilevare zone scure, usare solo la risposta luminosa.
        if not self.cfar_detect_dark:
            return bright_response

        # Invertire l'immagine: le ombre diventano valori alti rilevabili dallo stesso CFAR.
        inverted_image = 255 - image

        # Calcolare la risposta CFAR delle zone scure sull'immagine invertita.
        dark_response = self.compute_os_cfar_1d_response(inverted_image)

        # Unire risposta bright e dark mantenendo, pixel per pixel, il valore piu forte.
        return np.maximum(bright_response, dark_response).astype(np.uint8)

    def compute_os_cfar_1d_response(self, image):
        # portare l'immagine in float32 per calcolare soglie e differenze
        image = self.normalize_uint8(image)
        image_float = image.astype(np.float32)
        height, width = image_float.shape

        # parametri della finestra CFAR: celle training, celle guard e rank OS
        train = max(1, int(self.cfar_train_cells))
        guard = max(0, int(self.cfar_guard_cells))
        radius = train + guard

        rank_percentile = float(self.cfar_rank_percentile)
        rank_percentile = max(0.0, min(100.0, rank_percentile))
        rank = int(round((rank_percentile / 100.0) * ((2 * train) - 1)))
        rank = max(0, min((2 * train) - 1, rank))

        # parametri della soglia adattiva
        threshold_scale  = float(self.cfar_threshold_scale)
        threshold_offset = float(self.cfar_threshold_offset)
        response = np.zeros_like(image_float, dtype=np.float32)

        # applicare CFAR riga per riga
        for row in range(height):
            data = image_float[row, :]
            padded = np.pad(data, (radius, radius), mode='edge')

            for col in range(width):		# col = CUT
                center = col + radius

                # celle training a sinistra e destra, escludendo le celle guard vicino al CUT
                left_train  = padded[center - guard - train:center - guard]
                right_train = padded[center + guard + 1:center + guard + train + 1]
                training_cells = np.concatenate((left_train, right_train))

                # soglia locale: campione ordinato scelto dal rank, scalato e traslato
                noise_estimate = np.partition(training_cells, rank)[rank]
                threshold = noise_estimate * threshold_scale + threshold_offset

                if data[col] > threshold:
                    response[row, col] = data[col] - threshold

        # Se nessun pixel e stato rilevato, restituire una mappa nera.
        max_response = float(np.max(response))
        if max_response <= 0.0:
            return np.zeros_like(image, dtype=np.uint8)

        # Normalizzare la risposta in [0, 255] per salvarla/combinarla come immagine uint8.
        return cv2.normalize(response, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)


    # ========================================================
    # BINARY MAPS
    # ========================================================
    def create_bright_dark_maps(self, image, saliency_map):
	# normalizzare immagine in uint8 [0, 255]
        image = self.normalize_uint8(image)

	# creare mappa binaria di salienza
        if saliency_map is None:
            empty_map = np.zeros_like(image, dtype=np.uint8)
            return empty_map, empty_map, empty_map, empty_map, empty_map

	# normalizzare immagine in uint8 [0, 255]
        saliency_map = self.normalize_uint8(saliency_map)
        # considerare solo i pixel con salienza non nulla
        nonzero_saliency = saliency_map[saliency_map > 0]
        # se non ci sono pixel salienti, restituire una mappa binaria completamente nera
        if nonzero_saliency.size == 0:
            saliency_binary = np.zeros_like(saliency_map, dtype=np.uint8)
        else:
	    # calcolare una soglia dinamica solo sui pixel salienti non nulli
            threshold = np.percentile(nonzero_saliency, self.saliency_percentile)
	    # creare una maschera binaria: 1 dove la salienza e sopra la soglia, 0 altrove
            saliency_binary = (saliency_map > threshold).astype(np.uint8) * 255

	# calcolare una soglia dinamica per pixel "molto luminosi" e "molto scuri"
        bright_threshold = np.percentile(image, self.bright_percentile)
        dark_threshold   = np.percentile(image, self.dark_percentile)
	# creare maschera binaria
        bright_map = (image >= bright_threshold).astype(np.uint8) * 255
        dark_map   = (image <= dark_threshold).astype(np.uint8) * 255

        # mantenere solo pixel salienti
        bright_and_salient_map = cv2.bitwise_and(bright_map, saliency_binary)
        #dark_and_salient_map   = cv2.bitwise_and(dark_map, saliency_binary)

	# pulizia morfologica
        #bright_and_salient_map = self.morphological_cleaning_bright(bright_and_salient_map)
        bright_and_salient_map = self.morphological_cleaning_bright(saliency_binary)	# usare saliency binary al posto di bright_and_salient
        #dark_and_salient_map   = self.morphological_cleaning_bright(dark_and_salient_map)
        dark_and_salient_map   = self.morphological_cleaning_dark(dark_map)		# non e propriamente dark_and_salient...
        return saliency_binary, bright_map, dark_map, bright_and_salient_map, dark_and_salient_map


    def make_positive_odd(self, value):
        value = int(value)
        if value < 1:
            value = 1
        if value % 2 == 0:
            value += 1
        return value


    def morphological_cleaning_bright(self, binary_map):
        # open-close-open-close: pulire rumore, riconnettere oggetti, ripulire e chiudere frammenti rimasti
        first_open_kernel_size = self.make_positive_odd(self.bright_morph_first_open_kernel_size)
        close_kernel_size      = self.make_positive_odd(self.bright_morph_close_kernel_size)
        final_open_kernel_size = self.make_positive_odd(self.bright_morph_final_open_kernel_size)
        final_close_kernel_size = self.make_positive_odd(self.bright_morph_final_close_kernel_size)
        iterations  = int(self.bright_morph_iterations)
        if iterations < 1:
            iterations = 1

	# generare kernel
        first_open_kernel  = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (first_open_kernel_size, first_open_kernel_size))
	close_kernel       = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (close_kernel_size, close_kernel_size))
        final_open_kernel  = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (final_open_kernel_size, final_open_kernel_size))
        final_close_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (final_close_kernel_size, final_close_kernel_size))
	# opening: eliminare piccoli blob bianchi (rumore) e separare regioni sottilmente collegate
        binary_map = cv2.morphologyEx(binary_map, cv2.MORPH_OPEN, first_open_kernel, iterations=iterations)
	# closing: chiudere piccoli buchi neri e collegare regioni vicine
        binary_map = cv2.morphologyEx(binary_map, cv2.MORPH_CLOSE, close_kernel, iterations=iterations)
	# opening finale: rimuovere piccoli artefatti creati o rimasti dopo il closing
        binary_map = cv2.morphologyEx(binary_map, cv2.MORPH_OPEN, final_open_kernel, iterations=iterations)
	# closing finale: riconnettere frammenti utili rimasti dopo l opening finale
        binary_map = cv2.morphologyEx(binary_map, cv2.MORPH_CLOSE, final_close_kernel, iterations=iterations)
        return binary_map


    def morphological_cleaning_dark(self, binary_map):
        # close-open-close: riconnettere ombre, pulire rumore, poi riconnettere i frammenti rimasti
        first_close_kernel_width  = self.make_positive_odd(self.dark_morph_first_close_kernel_width)
        first_close_kernel_height = self.make_positive_odd(self.dark_morph_first_close_kernel_height)
        final_close_kernel_width  = self.make_positive_odd(self.dark_morph_final_close_kernel_width)
        final_close_kernel_height = self.make_positive_odd(self.dark_morph_final_close_kernel_height)
        open_kernel_size          = self.make_positive_odd(self.dark_morph_open_kernel_size)
        iterations                = int(self.dark_morph_iterations)
        if iterations < 1:
            iterations = 1

        first_close_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (first_close_kernel_width, first_close_kernel_height))
        binary_map 	   = cv2.morphologyEx(binary_map, cv2.MORPH_CLOSE, first_close_kernel, iterations=iterations)

        if open_kernel_size > 1:
            open_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (open_kernel_size, open_kernel_size))
            binary_map  = cv2.morphologyEx(binary_map, cv2.MORPH_OPEN, open_kernel, iterations=iterations)

        #final_close_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (final_close_kernel_width, final_close_kernel_height))
        #binary_map 	    = cv2.morphologyEx(binary_map, cv2.MORPH_CLOSE, final_close_kernel, iterations=iterations)
        return binary_map


    # ========================================================
    # CLASSIFICAZIONE OGGETTI
    # ========================================================
    def extract_blobs(self, binary_map, min_area_px, blob_type):
	# normalizzare immagine in uint8 [0, 255]
        binary_map = self.normalize_uint8(binary_map)
	# cercare componenti connesse
        _, binary_map = cv2.threshold(binary_map, 0, 255, cv2.THRESH_BINARY)

        # etichettare tutte le regioni bianche connesse nella mappa binaria
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(binary_map, connectivity=8)	# 8, pixel connesso anche in diagonale

        blobs = []

        # saltare label 0 perche rappresenta lo sfondo nero
        for label in range(1, num_labels):
            # scartare componenti troppo piccole per essere oggetti/ombre
            area = int(stats[label, cv2.CC_STAT_AREA])								# CC_STAT_AREA = area in pixel, restituita da connectedComponentsWithStats()
            if area < min_area_px:
                continue

            # creare una maschera isolata per la componente corrente
            component_mask = np.zeros_like(binary_map, dtype=np.uint8)						# matrice di zeri
            component_mask[labels == label] = 255								# inserire 255 nella matrice di zeri in corrispondenza del blob

            # estrarre il (primo) contorno della componente per calcolare geometria, area e rettangolo minimo
	    contours = cv2.findContours(component_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	    contours = contours[0] if len(contours) == 2 else contours[1]
            if not contours:
                continue
	    contour = contours[0]
            # area del contorno in pixel quadrati
	    contour_area_px = float(cv2.contourArea(contour))
	    if contour_area_px <= 0.0:
	        continue

            # rettangolo orientato in pixel: centro, lati e angolo inclinazione
            pixel_rect = cv2.minAreaRect(contour)
            (cx_px, cy_px), (w_px, h_px), angle_px = pixel_rect
	    # salvare dimensioni massime e minime del rettangolo in pixel
            max_dimension_px = float(max(w_px, h_px))
            min_dimension_px = float(min(w_px, h_px))
            if min_dimension_px <= 0.0:
                continue

            # convertire il contorno da pixel a metri usando scale diverse x/y del sonar
            metric_contour = contour.astype(np.float32).copy()
            metric_contour[:, :, 0] *= self.meters_per_pixel_x
            metric_contour[:, :, 1] *= self.meters_per_pixel_y

            # rettangolo orientato in metri: centro, lati e angolo inclinazione
            metric_rect = cv2.minAreaRect(metric_contour)
            (_, _), (w_m, h_m), angle_m = metric_rect
	    # salvare dimensioni massime e minime del rettangolo in metri
            max_dimension_m = float(max(w_m, h_m))
            min_dimension_m = float(min(w_m, h_m))
            if min_dimension_m <= 0.0:
                continue

            # area del contorno in metri quadrati
            contour_area_m2 = float(cv2.contourArea(metric_contour))

            # salvare tutte le misure utili
            blobs.append({
                'id': int(label),
                'type': blob_type,
                'area_px': area,
                'area_m2': contour_area_m2,
                'area_px': contour_area_px,
                'centroid': (float(cx_px), float(cy_px)),
                'bbox': (
                    int(stats[label, cv2.CC_STAT_LEFT]),
                    int(stats[label, cv2.CC_STAT_TOP]),
                    int(stats[label, cv2.CC_STAT_WIDTH]),
                    int(stats[label, cv2.CC_STAT_HEIGHT])
                ),
                'pixel_rect': pixel_rect,
                'metric_rect': metric_rect,
                'max_dimension_px': max_dimension_px,
                'min_dimension_px': min_dimension_px,
                'max_dimension_m': max_dimension_m,
                'min_dimension_m': min_dimension_m,
                'aspect_ratio': max_dimension_m / min_dimension_m,
                'angle': float(angle_m),
                'contour': contour
            })

        return blobs

    def is_shadow_candidate_for_object(self, obj, shadow, nadir_column):
        # leggere i centroidi in pixel di oggetto e ombra
        obj_x, obj_y       = obj['centroid']
        shadow_x, shadow_y = shadow['centroid']

        # distanza across-track (dal nadir)
        obj_range    = abs(obj_x - nadir_column)
        shadow_range = abs(shadow_x - nadir_column)

        # ombra deve essere piu lontana dal nadir rispetto all oggetto
        if shadow_range <= obj_range:
            return False

        # oggetto e ombra devono stare dallo stesso lato del nadir
        if (obj_x - nadir_column) * (shadow_x - nadir_column) < 0.0:
            return False

        # distanza tra oggetto e ombra in metri
        alongtrack_gap_m  = abs(shadow_y - obj_y) * self.meters_per_pixel_y
        acrosstrack_gap_m = (shadow_range - obj_range) * self.meters_per_pixel_x

        # scartare coppie troppo lontane per essere un oggetto e la sua ombra
        if alongtrack_gap_m > self.max_shadow_alongtrack_gap_m:
            return False
        if acrosstrack_gap_m > self.max_shadow_acrosstrack_gap_m:
            return False

        return True

    def pair_score(self, obj, shadow, nadir_column):
        # calcolare posizione relativa di oggetto e ombra rispetto al nadir
        obj_x, obj_y       = obj['centroid']
        shadow_x, shadow_y = shadow['centroid']
        obj_range          = abs(obj_x - nadir_column)
        shadow_range       = abs(shadow_x - nadir_column)

        # distanze in metri usate per stimare la qualita della coppia
        acrosstrack_gap_m = max(0.0, (shadow_range - obj_range) * self.meters_per_pixel_x)
        alongtrack_gap_m  = abs(shadow_y - obj_y) * self.meters_per_pixel_y
        shadow_area_ratio = shadow['area_px'] / float(max(obj['area_px'], 1))

        # punteggio semplice: 1 punto per ogni condizione buona della coppia oggetto-ombra
        score = 0.0
        if alongtrack_gap_m <= 0.5 * self.max_shadow_alongtrack_gap_m:
            score += 1.0
        if acrosstrack_gap_m <= 0.5 * self.max_shadow_acrosstrack_gap_m:
            score += 1.0
        if shadow_area_ratio >= 0.25:
            score += 1.0
        return score

    def choose_best_shadow(self, obj, shadows, nadir_column):
        # cercare ombra con punteggio migliore per l oggetto
        best_shadow = None
        best_score = -1.0
        for shadow in shadows:
            # prima applicare i vincoli geometrici minimi
            if not self.is_shadow_candidate_for_object(obj, shadow, nadir_column):
                continue

            # poi calcolare un punteggio per confrontare le ombre candidate
            score = self.pair_score(obj, shadow, nadir_column)
            if score > best_score:
                best_score = score
                best_shadow = shadow
        return best_shadow, best_score

    def score_interval(self, value, min_value, max_value):
        # restituire 1 punto se il valore cade dentro l intervallo, altrimenti 0
        if min_value <= value <= max_value:
            return 1.0
        return 0.0

    def score_tube(self, obj, shadow, estimated_height_m, pair_score):
        # tubo atteso e lungo, stretto e con ombra coerente con un oggetto allungato
        shadow_ratio = shadow['max_dimension_m'] / max(obj['max_dimension_m'], 0.001)
        score = pair_score
        score += self.score_interval(obj['max_dimension_m'], 2.0, 4.0)				# intervallo di dimensioni massime dell oggetto ammissibili
        score += self.score_interval(obj['min_dimension_m'], 0.4, 1.1)				# intervallo di dimensioni minime  dell oggetto ammissibili
        score += self.score_interval(obj['aspect_ratio'], 2.0, 10.0)				# dimensioni bbox descrivono la forma del blob
        score += self.score_interval(shadow_ratio, 0.8, 4.0)					# rapporto ombra/oggetto per verificare che ombra non sia troppo piccola rispetto all oggetto
        score += self.score_interval(estimated_height_m, 0.0, 1.3)				# misurare ombra per stimare altezza oggetto
        return score

    def score_buoy(self, obj, shadow, estimated_height_m, pair_score):
        # boa attesa e piu compatta e ombra abbastanza lunga rispetto all oggetto
        shadow_ratio = shadow['max_dimension_m'] / max(obj['max_dimension_m'], 0.001)
        score = pair_score
        score += self.score_interval(obj['max_dimension_m'], 0.3, 1.6)				# intervallo di dimensioni massime dell oggetto ammissibili
        score += self.score_interval(obj['min_dimension_m'], 0.3, 1.2)				# intervallo di dimensioni minime  dell oggetto ammissibili
        score += self.score_interval(obj['aspect_ratio'], 1.0, 2.0)				# dimensioni bbox descrivono la forma del blob
        score += self.score_interval(shadow_ratio, 1.0, 5.0)					# rapporto ombra/oggetto per verificare che ombra non sia troppo piccola rispetto all oggetto
        score += self.score_interval(estimated_height_m, 0.8, 2.8)				# misurare ombra per stimare altezza oggetto
        return score

    def detect_and_classify_objects(self, bright_map, dark_map):
        # ricavare larghezza immagine e assumere il nadir al centro dell'immagine sonar
        image_width  = bright_map.shape[1]
        nadir_column = image_width / 2.0

        # estrarre blob come possibili oggetti e come possibili ombre
        objects = self.extract_blobs(bright_map, self.min_object_area_px, 'object')	# array blob oggetti
        shadows = self.extract_blobs(dark_map, self.min_shadow_area_px, 'shadow')	# array blob ombre

        detections = []
        for obj in objects:
            # associare all oggetto l ombra piu plausibile
            shadow, pair_score = self.choose_best_shadow(obj, shadows, nadir_column)
            if shadow is None:
                continue

            # stimare altezza e calcolare punteggi separati per tubo e boa
            estimated_height_m = shadow['max_dimension_m'] * self.shadow_height_scale
            tube_score = self.score_tube(obj, shadow, estimated_height_m, pair_score)
            buoy_score = self.score_buoy(obj, shadow, estimated_height_m, pair_score)

            # usare il punteggio migliore come base per la confidenza finale
            max_score = max(tube_score, buoy_score)
            confidence = max_score / 8.0						# 8 e il massimo punteggio ottenibile 

            # sotto soglia la detection resta ignota, sopra soglia vince il punteggio maggiore
            if confidence < self.min_classification_confidence:
                classification = 'unknown'
            elif tube_score >= buoy_score:
                classification = 'tube'
            else:
                classification = 'buoy'

            # salvare oggetto, ombra e punteggi
            object_data = {
                'id': obj['id'],
                'centroid_px': [round(obj['centroid'][0], 2), round(obj['centroid'][1], 2)],
                'bbox_px': list(obj['bbox']),
                'max_dimension_m': round(obj['max_dimension_m'], 3),
                'min_dimension_m': round(obj['min_dimension_m'], 3),
                'aspect_ratio': round(obj['aspect_ratio'], 3),
                'area_m2': round(obj['area_m2'], 3),
                'angle_deg': round(obj['angle'], 3)
            }

            shadow_data = {
                'id': shadow['id'],
                'centroid_px': [round(shadow['centroid'][0], 2), round(shadow['centroid'][1], 2)],
                'bbox_px': list(shadow['bbox']),
                'max_dimension_m': round(shadow['max_dimension_m'], 3),
                'min_dimension_m': round(shadow['min_dimension_m'], 3),
                'aspect_ratio': round(shadow['aspect_ratio'], 3),
                'area_m2': round(shadow['area_m2'], 3),
                'angle_deg': round(shadow['angle'], 3)
            }

            detections.append({
                'classification': classification,
                'confidence': round(float(max(0.0, min(1.0, confidence))), 3),
                'tube_score': round(float(tube_score), 3),
                'buoy_score': round(float(buoy_score), 3),
                'pair_score': round(float(pair_score), 3),
                'estimated_height_m': round(float(estimated_height_m), 3),
                'object': object_data,
                'shadow': shadow_data
            })

        # restituire prima le detection piu affidabili
        detections.sort(key=lambda item: item['confidence'], reverse=True)
        return detections

    # ========================================================
    # GEOMETRIA E LOCALIZZAZIONE
    # ========================================================
    def image_to_body_frame(self, bbox_center):
        # conversione coordinate immagine -> body frame AUV
        return

    def body_to_ned(self, body_coords, nav_status):
        # conversione body frame -> NED
        return

    def geolocalize_object(self, bbox_center, nav_status):
        # ottenere latitudine e longitudine oggetto
        return

    # ========================================================
    # GESTIONE OGGETTI RICONOSCIUTI IN PIU IMMAGINI
    # ========================================================
    def match_objects_across_frames(self, new_objects):
        # verificare se oggetti già visti in frame precedenti
        return

    def resolve_conflicts(self, object_history):
        # gestire classificazioni discordanti dello stesso oggetto
        return

    def remove_duplicates(self, object_list):
        # eliminare oggetti ridondanti basati su distanza spaziale
        return

    # ========================================================
    # OUTPUT FINALE
    # ========================================================
    def generate_final_object_list(self):
        # creare lista finale:
        # - tipo oggetto (tubo/boa)
        # - confidenza
        # - latitudine/longitudine
        pass


# ________________________________________________________________________________________________________________________________

    # ========================================================
    # SALVATAGGIO IMMAGINI
    # ========================================================

    def save_debug_image(self, folder, prefix, image, image_index):
        filename = os.path.join(folder, "{}_{:03d}.png".format(prefix, image_index))
        saved = cv2.imwrite(filename, image)
        if not saved:
            rospy.logwarn("Impossibile salvare immagine: {}".format(filename))
        return filename

    def save_filtered_image(self, image, image_index):
        return self.save_debug_image(self.filtered_image_folder, "filtered", self.normalize_uint8(image), image_index)

    def save_saliency_image(self, saliency_map, image_index):
        return self.save_debug_image( self.saliency_image_folder, "saliency", self.normalize_uint8(saliency_map), image_index)

    def save_saliency_binary_image(self, saliency_binary_map, image_index):
        return self.save_debug_image(self.saliency_binary_image_folder, "saliency_binary", self.normalize_uint8(saliency_binary_map), image_index)

    def save_bright_map_image(self, bright_map, image_index):
        return self.save_debug_image( self.bright_map_folder, "bright", self.normalize_uint8(bright_map), image_index)

    def save_dark_map_image(self, dark_map, image_index):
        return self.save_debug_image(self.dark_map_folder, "dark", self.normalize_uint8(dark_map), image_index)

    def save_bright_and_salient_map_image(self, bright_map, image_index):
        return self.save_debug_image(self.bright_and_salient_map_folder, "bright_and_salient", self.normalize_uint8(bright_map), image_index)

    def save_dark_and_salient_map_image(self, dark_map, image_index):
        return self.save_debug_image(self.dark_and_salient_map_folder, "dark_and_salient", self.normalize_uint8(dark_map), image_index)

    def save_classification_image(self, image, classifications, image_index):
        output = cv2.cvtColor(self.normalize_uint8(image), cv2.COLOR_GRAY2BGR)

        for detection in classifications:
            obj = detection['object']
            shadow = detection['shadow']
            label = "{} {:.2f}".format(detection['classification'], detection['confidence'])

            obj_x, obj_y, obj_w, obj_h = obj['bbox_px']
            shadow_x, shadow_y, shadow_w, shadow_h = shadow['bbox_px']
            cv2.rectangle(output, (int(obj_x), int(obj_y)), (int(obj_x + obj_w), int(obj_y + obj_h)), (0, 255, 0), 2)
            cv2.rectangle(output, (int(shadow_x), int(shadow_y)), (int(shadow_x + shadow_w), int(shadow_y + shadow_h)), (255, 0, 0), 2)

            text_x = int(obj['bbox_px'][0])
            text_y = max(15, int(obj['bbox_px'][1]) - 6)
            cv2.putText(output, label, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1, cv2.LINE_AA)

        return self.save_debug_image(self.classification_image_folder, "classification", output, image_index)

    def save_classification_text(self, classifications, image_index):
        # salvare su file tutte le informazioni numeriche prodotte dalla classificazione
        filename = os.path.join(self.classification_text_folder, "classification_{:03d}.txt".format(image_index))

        try:
            with open(filename, 'w') as text_file:
                text_file.write("image_index: {}\n".format(image_index))
                text_file.write("detections_count: {}\n".format(len(classifications)))
                text_file.write("\nPROCESSING PARAMETERS\n")
                text_file.write("log_filter_scale: {:.3f}\n".format(self.log_filter_scale))
                text_file.write("median_kernel_size: {}\n".format(self.median_kernel_size))
                text_file.write("gaussian_kernel_size: {}\n".format(self.gaussian_kernel_size))
                text_file.write("gaussian_sigma: {:.3f}\n".format(self.gaussian_sigma))
                text_file.write("clahe_clip_limit: {:.3f}\n".format(self.clahe_clip_limit))
                text_file.write("clahe_tile_grid_size: {}\n".format(self.clahe_tile_grid_size))
                text_file.write("percentile_low: {:.3f}\n".format(self.percentile_low))
                text_file.write("percentile_high: {:.3f}\n".format(self.percentile_high))
                text_file.write("bright_morph_first_open_kernel_size: {}\n".format(self.bright_morph_first_open_kernel_size))
                text_file.write("bright_morph_close_kernel_size: {}\n".format(self.bright_morph_close_kernel_size))
                text_file.write("bright_morph_final_open_kernel_size: {}\n".format(self.bright_morph_final_open_kernel_size))
                text_file.write("bright_morph_final_close_kernel_size: {}\n".format(self.bright_morph_final_close_kernel_size))
                text_file.write("bright_morph_iterations: {}\n".format(self.bright_morph_iterations))
                text_file.write("dark_morph_open_kernel_size: {}\n".format(self.dark_morph_open_kernel_size))
                text_file.write("dark_morph_first_close_kernel_width: {}\n".format(self.dark_morph_first_close_kernel_width))
                text_file.write("dark_morph_first_close_kernel_height: {}\n".format(self.dark_morph_first_close_kernel_height))
                text_file.write("dark_morph_final_close_kernel_width: {}\n".format(self.dark_morph_final_close_kernel_width))
                text_file.write("dark_morph_final_close_kernel_height: {}\n".format(self.dark_morph_final_close_kernel_height))
                text_file.write("dark_morph_iterations: {}\n".format(self.dark_morph_iterations))
                text_file.write("saliency_percentile: {:.3f}\n".format(self.saliency_percentile))
                text_file.write("cfar_train_cells: {}\n".format(self.cfar_train_cells))
                text_file.write("cfar_guard_cells: {}\n".format(self.cfar_guard_cells))
                text_file.write("cfar_rank_percentile: {:.3f}\n".format(self.cfar_rank_percentile))
                text_file.write("cfar_threshold_scale: {:.3f}\n".format(self.cfar_threshold_scale))
                text_file.write("cfar_threshold_offset: {:.3f}\n".format(self.cfar_threshold_offset))
                text_file.write("cfar_detect_dark: {}\n".format(self.cfar_detect_dark))
                text_file.write("cfar_pfa: {:.6f}\n".format(self.cfar_pfa))
                text_file.write("bright_percentile: {:.3f}\n".format(self.bright_percentile))
                text_file.write("dark_percentile: {:.3f}\n".format(self.dark_percentile))
                text_file.write("meters_per_pixel_x: {:.6f}\n".format(self.meters_per_pixel_x))
                text_file.write("meters_per_pixel_y: {:.6f}\n".format(self.meters_per_pixel_y))
                text_file.write("nadir_column: center_of_image\n")
                text_file.write("min_object_area_px: {}\n".format(self.min_object_area_px))
                text_file.write("min_shadow_area_px: {}\n".format(self.min_shadow_area_px))
                text_file.write("max_shadow_alongtrack_gap_m: {:.3f}\n".format(self.max_shadow_alongtrack_gap_m))
                text_file.write("max_shadow_accrosstrack_gap_m: {:.3f}\n".format(self.max_shadow_acrosstrack_gap_m))
                text_file.write("shadow_height_scale: {:.3f}\n".format(self.shadow_height_scale))
                text_file.write("min_classification_confidence: {:.3f}\n".format(self.min_classification_confidence))

                if len(classifications) == 0:
                    text_file.write("\nNessuna detection trovata.\n")
                    return filename

                for detection_index, detection in enumerate(classifications):
                    obj = detection['object']
                    shadow = detection['shadow']

                    text_file.write("\nDETECTION {}\n".format(detection_index + 1))
                    text_file.write("classification: {}\n".format(detection['classification']))
                    text_file.write("confidence: {:.3f}\n".format(detection['confidence']))
                    text_file.write("tube_score: {:.3f}\n".format(detection['tube_score']))
                    text_file.write("buoy_score: {:.3f}\n".format(detection['buoy_score']))
                    text_file.write("pair_score: {:.3f}\n".format(detection['pair_score']))
                    text_file.write("estimated_height_m: {:.3f}\n".format(detection['estimated_height_m']))

                    text_file.write("object_id: {}\n".format(obj['id']))
                    text_file.write("object_centroid_px: [{:.2f}, {:.2f}]\n".format(obj['centroid_px'][0], obj['centroid_px'][1]))
                    text_file.write("object_bbox_px: {}\n".format(obj['bbox_px']))
                    text_file.write("object_max_dimension_m: {:.3f}\n".format(obj['max_dimension_m']))
                    text_file.write("object_min_dimension_m: {:.3f}\n".format(obj['min_dimension_m']))
                    text_file.write("object_aspect_ratio: {:.3f}\n".format(obj['aspect_ratio']))
                    text_file.write("object_area_m2: {:.3f}\n".format(obj['area_m2']))
                    text_file.write("object_angle_deg: {:.3f}\n".format(obj['angle_deg']))

                    text_file.write("shadow_id: {}\n".format(shadow['id']))
                    text_file.write("shadow_centroid_px: [{:.2f}, {:.2f}]\n".format(shadow['centroid_px'][0], shadow['centroid_px'][1]))
                    text_file.write("shadow_bbox_px: {}\n".format(shadow['bbox_px']))
                    text_file.write("shadow_max_dimension_m: {:.3f}\n".format(shadow['max_dimension_m']))
                    text_file.write("shadow_min_dimension_m: {:.3f}\n".format(shadow['min_dimension_m']))
                    text_file.write("shadow_aspect_ratio: {:.3f}\n".format(shadow['aspect_ratio']))
                    text_file.write("shadow_area_m2: {:.3f}\n".format(shadow['area_m2']))
                    text_file.write("shadow_angle_deg: {:.3f}\n".format(shadow['angle_deg']))
        except IOError as exc:
            rospy.logwarn("Impossibile salvare report classificazione: {} ({})".format(filename, exc))

        return filename

# ============================================================
# MAIN
# ============================================================
if __name__ == "__main__":
    # try:
        # inizializzare nodo ROS
        rospy.init_node('list_creator', anonymous=True)
        # istanziare ListCreatorNode
        node = ListCreatorNode()
        # spin ROS
        rospy.spin()
	print("All done :)\n")
    # except rospy.ROSInterruptException:
        # pass
