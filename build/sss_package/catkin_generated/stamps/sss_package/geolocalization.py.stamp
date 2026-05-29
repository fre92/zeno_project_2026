#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-

# ============================================================
# IMPORT LIBRERIE
# ============================================================
import math
import os
import rospkg
import sys
import csv
import rospy
from sss_package.msg import ImageMetadata
from sss_package.msg import GeolocatedObject
from sss_package.msg import GeolocatedObjectList
from geodetic_functions import ll2ne 
from geodetic_functions import ne2ll 


# ============================================================
# CLASSE PRINCIPALE NODO GEOLOCALIZATION
# ============================================================
class GeolocalizationNode:

    def __init__(self):

	print("Initialization\n")

        # inizializzare subscriber/publisher
        rospy.Subscriber('/detected_objects', ImageMetadata, self.detected_objects_callback)
        self.pub_objects = rospy.Publisher('geolocated_objects', GeolocatedObjectList, queue_size=200)

        # funzioni geodetiche fornite
        #geodetic_path = rospy.get_param('~geodetic_functions_path', '/home/student/Downloads')
        #self.ne2ll = self.load_ne2ll_function(geodetic_path)

        # definire parametri Zeno e sensore
        self.sonar_range_m     = float(rospy.get_param('~sonar_range_m', 25.0))
        self.sensor_x_offset_m = float(rospy.get_param('~sensor_x_offset_m', 0.063))
        self.sensor_y_offset_m = float(rospy.get_param('~sensor_y_offset_m', 0.354))
        self.sensor_z_offset_m = float(rospy.get_param('~sensor_z_offset_m', 0.096))

        

        # Ottieni il percorso base del pacchetto
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('sss_package') 

        # Costruisci il percorso di default dinamico
        default_folder = os.path.join(pkg_path, 'results', '9_list_texts')

        # Leggi il parametro (se non fornito, usa il default dinamico che hai appena creato)
        self.list_text_folder = rospy.get_param('~list_text_folder', default_folder)

        # Crea le cartelle se non esistono
        if not os.path.exists(self.list_text_folder):
            os.makedirs(self.list_text_folder)

        self.list_text_index = 0


    #def load_ne2ll_function(self, geodetic_path):
        # Aggiunge al path Python la cartella che contiene geodetic_functions.py.
       # if geodetic_path and os.path.isdir(geodetic_path) and geodetic_path not in sys.path:
        #    sys.path.insert(0, geodetic_path)

       # from geodetic_functions import ne2ll
        #return ne2ll


# ________________________________________________________________________________________________________________________________


    # ========================================================
    # CALLBACK PER GEOLOCALIZZAZIONE
    # ========================================================
    def detected_objects_callback(self, msg):
        # 1. prepara il messaggio di output mantenendo lo stesso header dell'immagine/detection
        output = GeolocatedObjectList()
        output.header = msg.header

        # 2. individua il nadir
        image_width = int(msg.image.width)
        localization_infos = []

        # 3. geolocalizza ogni oggetto rilevato
        for object_index in range(len(msg.object_classes)):
            result = self.geolocalize_detection(msg, object_index, image_width)
            if result is not None:
                geolocated_object, localization_info = result
                output.objects.append(geolocated_object)
                localization_infos.append(localization_info)

	# 4. pubblica lista di oggetti geolocalizzati 
        self.pub_objects.publish(output)
        self.save_geolocated_list_text(output, localization_infos)
        self.save_google_my_maps_csv(output)


# ________________________________________________________________________________________________________________________________

    # ========================================================
    # GEOLOCALIZZAZIONE
    # ========================================================

    def geolocalize_detection(self, msg, object_index, image_width):
        # xc e la coordinata across-track; yc identifica il ping dell immagine waterfall
        centroid_x = float(msg.object_centroid_x_px[object_index])
        centroid_y = float(msg.object_centroid_y_px[object_index])
        row_index  = int(round(centroid_y))

        # convertire la coordinata x del centroide in distanza orizzontale sul fondale
        altitude_m = float(msg.altitudes[row_index])
        ranges = self.centroid_x_to_ranges(centroid_x, image_width, altitude_m)
        if ranges is None:
            return None
        slant_range_m, ground_range_m = ranges

        # sensor + conversioni body frame -> NED -> latitudine/longitudine
        nav_status = msg.nav_statuses[row_index]
        body_position = self.ground_range_to_body_position(centroid_x, image_width, ground_range_m)
        north_m, east_m, down_m = self.body_to_ned(body_position, nav_status)

        object_latitude, object_longitude = ne2ll(
            [float(nav_status.position.latitude), float(nav_status.position.longitude)],
            [north_m, east_m]
        )

        # risultati geolocalizzazione
        output = GeolocatedObject()
        output.object_class = msg.object_classes[object_index]
        output.confidence = float(msg.object_confidences[object_index])
        output.latitude   = float(object_latitude)
        output.longitude  = float(object_longitude)
        output.ping_index = int(msg.ping_indices[row_index])
        output.ping_stamp = msg.ping_stamps[row_index]
        output.centroid_x_px = centroid_x
        output.centroid_y_px = centroid_y

        if object_index < len(msg.object_bbox_x_px):
            output.bbox_x_px = int(msg.object_bbox_x_px[object_index])
            output.bbox_y_px = int(msg.object_bbox_y_px[object_index])
            output.bbox_width_px = int(msg.object_bbox_width_px[object_index])
            output.bbox_height_px = int(msg.object_bbox_height_px[object_index])

        localization_info = {
            'object_index': int(object_index),
            'row_index': int(row_index),
            'image_width': int(image_width),
            'auv_latitude': float(nav_status.position.latitude),
            'auv_longitude': float(nav_status.position.longitude),
            'auv_yaw_rad': float(nav_status.orientation.yaw),
            'altitude_m': float(altitude_m),
            'slant_range_m': float(slant_range_m),
            'ground_range_m': float(ground_range_m),
            'body_x_m': float(body_position[0]),
            'body_y_m': float(body_position[1]),
            'body_z_m': float(body_position[2]),
            'north_offset_m': float(north_m),
            'east_offset_m': float(east_m),
            'down_offset_m': float(down_m)
        }

        return output, localization_info

    def centroid_x_to_ranges(self, centroid_x, image_width, altitude_m):
        # la colonna centrale rappresenta il nadir
        nadir_column = image_width / 2.0
        bins_per_side = image_width / 2.0
        if bins_per_side <= 0.0:
            return None

        # ogni pixel x misura una distanza obliqua
        range_bin = abs(float(centroid_x) - nadir_column)
        meters_per_pixel_slant = self.sonar_range_m / bins_per_side
        slant_range_m = range_bin * meters_per_pixel_slant
        if slant_range_m <= altitude_m:
            rospy.logwarn("Detection in water-column/blind-zone: slant={:.3f} altitude={:.3f}".format(slant_range_m, altitude_m))
            return None

        # proiezione sul fondale: ground^2 = slant^2 - altitude^2
        ground_range_m = math.sqrt((slant_range_m * slant_range_m) - (altitude_m * altitude_m))
        return slant_range_m, ground_range_m

    def ground_range_to_body_position(self, centroid_x, image_width, ground_range_m):
        # lato dell immagine rispetto al nadir
        nadir_column = image_width / 2.0
        side = -1.0 if float(centroid_x) < nadir_column else 1.0

        # coordinate nel body frame: offset del sonar + distanza across-track sul fondale
        x_body = self.sensor_x_offset_m
        y_body = side * (self.sensor_y_offset_m + ground_range_m)
        z_body = self.sensor_z_offset_m
        return [x_body, y_body, z_body]

    def body_to_ned(self, body_position, nav_status):
        # orientazione AUV
        roll  = float(nav_status.orientation.roll)
        pitch = float(nav_status.orientation.pitch)
        yaw   = float(nav_status.orientation.yaw)

	'''
	# matrice di rotazione body -> NED (roll-pitch-yaw)
        cr = math.cos(roll)
        sr = math.sin(roll)
        cp = math.cos(pitch)
        sp = math.sin(pitch)
        cy = math.cos(yaw)
        sy = math.sin(yaw)

        x_body, y_body, z_body = body_position

        north = (cy * cp) * x_body + (cy * sp * sr - sy * cr) * y_body + (cy * sp * cr + sy * sr) * z_body
        east = (sy * cp) * x_body + (sy * sp * sr + cy * cr) * y_body + (sy * sp * cr - cy * sr) * z_body
        down = (-sp) * x_body + (cp * sr) * y_body + (cp * cr) * z_body
	'''

	# matrice di rotazione body -> NED (yaw)
	x_body, y_body, z_body = body_position

        north = math.cos(yaw) * x_body - math.sin(yaw) * y_body		# segni!!
        east  = math.sin(yaw) * x_body + math.cos(yaw) * y_body		# segni!!
        down  = z_body

        return north, east, down

    def save_geolocated_list_text(self, geolocated_list, localization_infos):
        # salvare su file la lista finale con coordinate e dettagli della geolocalizzazione
        filename = os.path.join(self.list_text_folder, "geolocated_list_{:03d}.txt".format(self.list_text_index))
        self.list_text_index += 1

        try:
            with open(filename, 'w') as text_file:
                text_file.write("geolocated_objects_count: {}\n".format(len(geolocated_list.objects)))
                text_file.write("sonar_range_m: {:.3f}\n".format(self.sonar_range_m))
                text_file.write("sensor_x_offset_m: {:.3f}\n".format(self.sensor_x_offset_m))
                text_file.write("sensor_y_offset_m: {:.3f}\n".format(self.sensor_y_offset_m))
                text_file.write("sensor_z_offset_m: {:.3f}\n".format(self.sensor_z_offset_m))

                if len(geolocated_list.objects) == 0:
                    text_file.write("\nNessun oggetto geolocalizzato.\n")
                    return filename

                for index, geolocated_object in enumerate(geolocated_list.objects):
                    info = localization_infos[index]
                    text_file.write("\nOBJECT {}\n".format(index + 1))
                    text_file.write("classification: {}\n".format(geolocated_object.object_class))
                    text_file.write("confidence: {:.3f}\n".format(geolocated_object.confidence))
                    text_file.write("latitude: {:.10f}\n".format(geolocated_object.latitude))
                    text_file.write("longitude: {:.10f}\n".format(geolocated_object.longitude))
                    text_file.write("ping_index: {}\n".format(geolocated_object.ping_index))
                    text_file.write("ping_stamp: {:.9f}\n".format(geolocated_object.ping_stamp.to_sec()))
                    text_file.write("row_index: {}\n".format(info['row_index']))
                    text_file.write("centroid_px: [{:.2f}, {:.2f}]\n".format(geolocated_object.centroid_x_px, geolocated_object.centroid_y_px))
                    text_file.write("bbox_px: [{}, {}, {}, {}]\n".format(
                        geolocated_object.bbox_x_px,
                        geolocated_object.bbox_y_px,
                        geolocated_object.bbox_width_px,
                        geolocated_object.bbox_height_px
                    ))
                    text_file.write("auv_latitude: {:.10f}\n".format(info['auv_latitude']))
                    text_file.write("auv_longitude: {:.10f}\n".format(info['auv_longitude']))
                    text_file.write("auv_yaw_rad: {:.6f}\n".format(info['auv_yaw_rad']))
                    text_file.write("altitude_m: {:.3f}\n".format(info['altitude_m']))
                    text_file.write("slant_range_m: {:.3f}\n".format(info['slant_range_m']))
                    text_file.write("ground_range_m: {:.3f}\n".format(info['ground_range_m']))
                    text_file.write("body_position_m: [{:.3f}, {:.3f}, {:.3f}]\n".format(
                        info['body_x_m'],
                        info['body_y_m'],
                        info['body_z_m']
                    ))
                    text_file.write("ned_offset_m: [{:.3f}, {:.3f}, {:.3f}]\n".format(
                        info['north_offset_m'],
                        info['east_offset_m'],
                        info['down_offset_m']
                    ))
        except IOError as exc:
            rospy.logwarn("Impossibile salvare lista geolocalizzata: {} ({})".format(filename, exc))

        return filename

    def save_google_my_maps_csv(self, geolocated_list):
        # salvare un CSV importabile direttamente in Google My Maps
        filename = os.path.join(self.list_text_folder, "google_my_maps_{:03d}.csv".format(self.list_text_index - 1))

        try:
            with open(filename, 'wb') as csv_file:
                writer = csv.writer(csv_file)
                writer.writerow([
                    'name',
                    'latitude',
                    'longitude',
                    'classification',
                    'confidence',
                    'ping_index',
                    'centroid_x_px',
                    'centroid_y_px',
                    'bbox_x_px',
                    'bbox_y_px',
                    'bbox_width_px',
                    'bbox_height_px'
                ])

                for index, geolocated_object in enumerate(geolocated_list.objects):
                    writer.writerow([
                        "object_{:03d}".format(index + 1),
                        "{:.10f}".format(geolocated_object.latitude),
                        "{:.10f}".format(geolocated_object.longitude),
                        geolocated_object.object_class,
                        "{:.3f}".format(geolocated_object.confidence),
                        int(geolocated_object.ping_index),
                        "{:.2f}".format(geolocated_object.centroid_x_px),
                        "{:.2f}".format(geolocated_object.centroid_y_px),
                        int(geolocated_object.bbox_x_px),
                        int(geolocated_object.bbox_y_px),
                        int(geolocated_object.bbox_width_px),
                        int(geolocated_object.bbox_height_px)
                    ])
        except IOError as exc:
            rospy.logwarn("Impossibile salvare CSV Google My Maps: {} ({})".format(filename, exc))

        return filename

if __name__ == "__main__":
    rospy.init_node('geolocalization', anonymous=True)
    node = GeolocalizationNode()
    rospy.spin()
