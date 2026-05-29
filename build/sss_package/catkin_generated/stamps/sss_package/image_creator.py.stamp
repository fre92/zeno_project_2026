#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-

# ============================================================
# IMPORT LIBRERIE
# ============================================================
import rospy
import cv2          # OpenCV
import numpy as np
import os
import copy
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import rosbag
from mpl_toolkits.mplot3d import Axes3D
from marta_msgs.msg import SideScanSonar
from marta_msgs.msg import Altitude
from marta_msgs.msg import NavStatus
from sss_package.msg import ImageMetadata
from std_msgs.msg import Header, UInt8MultiArray
from sensor_msgs.msg import Image as RosImage
from cv_bridge import CvBridge


# ============================================================
# CLASSE PRINCIPALE NODO IMAGE CREATOR
# ============================================================
class ImageCreatorNode:

    def __init__(self):

	print("Initialization\n")

        # inizializzare subscriber/publisher
        rospy.Subscriber('/drivers/sss_sim', SideScanSonar, self.image_callback)
        rospy.Subscriber('/drivers/altitude_sim', Altitude, self.altitude_callback)
        rospy.Subscriber('/nav_status', NavStatus, self.navstatus_callback)
        self.pub_img = rospy.Publisher('waterfall_creator', ImageMetadata, queue_size=200)

        # inizializzare CvBridge
        self.bridge = CvBridge()

        # definire parametri Zeno
        self.latitude   = None
        self.longitude  = None
        self.altitude   = None
        self.latest_altitude = None
        self.yaw_angle  = None
        self.omega_body = None
        self.latest_nav = NavStatus()

        # definire parametri waterfall
        self.waterfall_axis_display = None
        self.waterfall_fig, self.waterfall_ax = plt.subplots()
        self.ping_buffer = []
        self.ping_metadata_buffer = []
        self.ping_index  = 0

        # definire parametri immagini
        self.ping_per_image = 150
        self.ping_overlap   = 50
        self.image_buffer   = []
        self.image_index    = 0

	# definire parametri per altri plots
        self.raw_echo_buffer = []
        self.processed_echo_buffer = []
        self.echo_plot_index = 0
        self.altitude_start_time = None
        self.altitude_samples = []

        # creare cartella per immagini
        self.raw_image_folder = "/home/student/catkin_ws/results/0_raw_images"
        if not os.path.exists(self.raw_image_folder):
            os.makedirs(self.raw_image_folder)

        self.waterfall_image_folder = "/home/student/catkin_ws/results/1_waterfall_images"
        if not os.path.exists(self.waterfall_image_folder):
            os.makedirs(self.waterfall_image_folder)

        self.echo_plot_folder = "/home/student/catkin_ws/results/9_echo_intensity_plots"
        if not os.path.exists(self.echo_plot_folder):
            os.makedirs(self.echo_plot_folder)

        self.altitude_plot_folder = rospy.get_param('~altitude_plot_folder', "/home/student/catkin_ws/results/9_altitude_plots")
        if not os.path.exists(self.altitude_plot_folder):
            os.makedirs(self.altitude_plot_folder)

        rospy.on_shutdown(self.save_echo_intensity_plots)
        rospy.on_shutdown(self.save_altitude_plot)

        pass

# ________________________________________________________________________________________________________________________________

    # ========================================================
    # LETTURA ALTITUDE
    # ========================================================
    def read_altitude_from_bag(self, msg):
        # estrarre topic altitude
        self.altitude   = msg.altitude
        self.latest_altitude = copy.deepcopy(msg)
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
        self.latest_nav = copy.deepcopy(msg)
        pass

# ________________________________________________________________________________________________________________________________


    # ========================================================
    # CALLBACK PER CREAZIONE WATERFALL E IMMAGINI
    # ========================================================
    def image_callback(self, msg):
        # 1. estrai ping
        left, right = self.read_intensity_vectors(msg)
        raw_ping    = self.merge_beams(right, left)

        # 2. preprocessing
        right = self.apply_tvg(right)
        left  = self.apply_tvg(left)
        #right = self.apply_tvg_quadratic(right)
        #left  = self.apply_tvg_quadratic(left)
        #right = self.apply_tvg_logarithmic(right)
        #left  = self.apply_tvg_logarithmic(left)
        #right = self.apply_slant_range_correction(right, msg.range)
        #left  = self.apply_slant_range_correction(left, msg.range)
        #right = self.remove_blind_zone(right)
        #left  = self.remove_blind_zone(left)
        ping  = self.merge_beams(right, left)

	# opzionale: salvare intensita prima e dopo applicazione TVG per plot
        self.store_echo_intensity(raw_ping, ping)

        # 3. controllo navigazione
        #ping = self.check_nav_and_discard_ping(self.latest_nav, ping)

        # 4. stacking pings
        window = self.stack_pings(ping, self.ping_buffer)

        # 5. creazione waterfall
        self.ping_index += 1
        self.stack_ping_metadata(msg)
        self.build_waterfall(window)

        # 6. creazione immagine
        image = self.build_image(self.ping_buffer)
        image_metadata = self.build_image_metadata()

        # 7. pubblicazione immagine sul topic e in folder
        self.publish_image(image, image_metadata)
        pass


    # ========================================================
    # CALLBACK PER ALTITUDE DI ZENO
    # ========================================================
    def altitude_callback(self, msg):
        # 1. estrai altitude
        self.read_altitude_from_bag(msg)
	# 2. salvare altitude per plot       
	self.store_altitude_sample()
        pass


    # ========================================================
    # CALLBACK PER NAV_STATUS DI ZENO
    # ========================================================
    def navstatus_callback(self, msg):
        # 1. estrai latitudine, longitudine e yaw dal nav_status
        self.read_nav_status_from_bag(msg)
        # 2. plot della traiettoria di Zeno
        #self.plot_auv_trajectory(nav_status_stream)
        pass

# ________________________________________________________________________________________________________________________________


    # ========================================================
    # LETTURA DATI
    # ========================================================
    def read_intensity_vectors(self, msg):
	# print("Reading intensity vectors\n")
        # beam destro e sinistro
        if isinstance(msg.left_beam.data, str):
            left = np.frombuffer(msg.left_beam.data, dtype=np.uint8).astype(np.float32)
        else:
            left = np.asarray(msg.left_beam.data, dtype=np.uint8).astype(np.float32)

        if isinstance(msg.right_beam.data, str):
            right = np.frombuffer(msg.right_beam.data, dtype=np.uint8).astype(np.float32)
        else:
            right = np.asarray(msg.right_beam.data, dtype=np.uint8).astype(np.float32)

        return left, right

    # ========================================================
    # PREPROCESSING DATI
    # ========================================================
    def apply_tvg(self, beam, strength=0.5):  		# beam destro o sinistro
	# print("Applying TVG\n")
        # applicare Time Variable Gain:
        beam = np.asarray(beam, dtype=np.float32)
        if beam.size == 0:
            return beam

        normalized_range = np.linspace(0.0, 1.0, beam.size).astype(np.float32)
	gain = 1.0 + strength * np.sqrt(normalized_range)
        corrected_beam = beam * gain
        return np.clip(corrected_beam, 0, 255).astype(np.float32)

    def apply_tvg_quadratic(self, beam, strength=0.5):  # beam destro o sinistro
	# print("Applying quadratic TVG\n")
        # applicare Time Variable Gain quadratico:
        # gain = 1 + strength * (range / range_max)^2
        beam = np.asarray(beam, dtype=np.float32)
        if beam.size == 0:
            return beam

        normalized_range = np.linspace(0.0, 1.0, beam.size).astype(np.float32)
        gain = 1.0 + strength * (normalized_range ** 2)
        corrected_beam = beam * gain
        return np.clip(corrected_beam, 0, 255).astype(np.float32)

    def apply_tvg_logarithmic(self, beam, spreading_gain_db=20.0, absorption_db_per_sample=0.0):   # beam destro o sinistro
        # print("Applying logarithmic TVG\n")
        # applicare Time Variable Gain logaritmico
        # gain_dB = spreading_gain_db * log10(range) + 2 * alpha * range
        beam = np.asarray(beam, dtype=np.float32)
        if beam.size == 0:
            return beam

        sample_range = np.arange(1, beam.size + 1, dtype=np.float32)
        gain_db = (spreading_gain_db * np.log10(sample_range) + 2.0 * absorption_db_per_sample * sample_range)
	# normalizzare rispetto al primo bin (0 dB, nessun amplificazione)
        gain_db = gain_db - gain_db[0]
	# convertire da dB a scala lineare
        gain_linear = np.power(10.0, gain_db / 20.0).astype(np.float32)

        corrected_beam = beam * gain_linear
        return np.clip(corrected_beam, 0, 255).astype(np.float32)

    def apply_slant_range_correction(self, beam, sonar_range, sonar_altitude=None):       # beam destro o sinistro
	# print("Applying slant range correction\n")
        # correggere distorsione geometrica da slant range a ground range:
        # ground_range = sqrt(slant_range^2 - altitude^2)
        beam = np.asarray(beam, dtype=np.float32)
        if beam.size == 0:
            return beam

        if sonar_altitude is None:
            sonar_altitude = self.altitude

        if sonar_altitude is None:
            return beam

        sonar_range    = float(sonar_range)
        sonar_altitude = float(sonar_altitude)

        if sonar_range <= 0.0 or sonar_altitude <= 0.0:
            return beam

        if sonar_altitude >= sonar_range:
            return beam

        n_samples = beam.size
        max_ground_range = np.sqrt(sonar_range ** 2 - sonar_altitude ** 2)

        ground_range = np.linspace(0.0, max_ground_range, n_samples).astype(np.float32)
        slant_range = np.sqrt(ground_range ** 2 + sonar_altitude ** 2)

        source_index = (slant_range / sonar_range) * (n_samples - 1)
        sample_index = np.arange(n_samples, dtype=np.float32)

        corrected_beam = np.interp(source_index, sample_index, beam)
        return corrected_beam.astype(np.float32)

    def remove_blind_zone(self, beam):                  # beam destro o sinistro
	# print("Removing blind zone\n")
        # rimuovere zona cieca sonar
        return beam

    def merge_beams(self, right, left):
	# print("Merging beams\n")
        # unire beam destro e sinistro in singolo profilo (ping)
        left = left[::-1]
        ping = np.concatenate([left, right])
        return ping

    def check_nav_and_discard_ping(self, latest_nav, ping):
	# print("Checking if Zeno is turning\n")
        # verificare se AUV sta curvando e, in caso affermativo, scartare ping
        if latest_nav is None:
            return ping

        return ping


    # ========================================================
    # COSTRUZIONE WATERFALL
    # ========================================================
    def stack_pings(self, ping, ping_buffer):
	# print("Stacking pings\n")
	# aggiungi il nuovo ping in cima alla lista per simulare l'avanzamento
	ping_buffer.insert(0, ping)
	# mantenere solo gli ultimi 500 ping per effetto waterfall
	if len(ping_buffer) > 500:
	    ping_buffer.pop()
	# impilare ping in matrice verticale 2D
	window = np.vstack(ping_buffer).astype(np.float32)
	return window

    def stack_ping_metadata(self, sonar_msg):
        nav_valid = self.latest_nav is not None and self.latest_nav.header.stamp != rospy.Time()
        altitude_valid = self.latest_altitude is not None and bool(self.latest_altitude.validity)
        metadata = {
            'ping_index': int(self.ping_index),
            'ping_stamp': sonar_msg.header.stamp,
            'nav_status': copy.deepcopy(self.latest_nav),
            'nav_valid': bool(nav_valid),
            'altitude': float(self.altitude) if self.altitude is not None else 0.0,
            'altitude_valid': bool(altitude_valid)
        }
        self.ping_metadata_buffer.insert(0, metadata)
        if len(self.ping_metadata_buffer) > 500:
            self.ping_metadata_buffer.pop()

    def build_image_metadata(self):
        if len(self.ping_metadata_buffer) >= self.ping_per_image:
            return self.ping_metadata_buffer[:self.ping_per_image]
        return None

    def build_waterfall(self, window):
	# print("Building the waterfall\n")
        if window is None or window.size == 0:
            return None

        # normalizzare intensita in scala 0-255
        window = cv2.normalize(window, None, 0, 255, cv2.NORM_MINMAX)
        window = window.astype(np.uint8)

        # prima iterazione: creare immagine waterfall 
        if self.waterfall_axis_display is None:
            self.waterfall_axis_display = self.waterfall_ax.imshow(window, cmap='gray', aspect='auto', vmin=0, vmax=255)
            self.waterfall_ax.set_title("Side Scan Sonar waterfall")
            self.waterfall_ax.set_xlabel("Across-track")
            self.waterfall_ax.set_ylabel("Ping number")

        # iterazioni successive: aggiornare solo il contenuto dell immagine
        else:
            self.waterfall_axis_display.set_data(window)

            # fissare scala di intensita
            self.waterfall_axis_display.set_clim(0, 255)

            # aggiornare coordinate verticali
            self.waterfall_axis_display.set_extent([0, window.shape[1], self.ping_index, max(0, self.ping_index - len(self.ping_buffer))])
            self.waterfall_ax.set_ylim(self.ping_index, max(0, self.ping_index - len(self.ping_buffer)))

        self.waterfall_fig.canvas.draw_idle()
        return window

    # ========================================================
    # COSTRUZIONE IMMAGINI
    # ========================================================
    def build_image(self, ping_buffer):
	# print("Building an image\n")
        # creare immagini di 150 ping
        if len(ping_buffer) >= self.ping_per_image:
            img = np.vstack(ping_buffer[:self.ping_per_image]).astype(np.float32)
            img = cv2.normalize(img, None, 0, 255, cv2.NORM_MINMAX)
            img = img.astype(np.uint8)
            return img

        # se <150 ping:
        #   - non scartare dati
        #   - creare immagine parziale o estendere precedente
        return None

    # ========================================================
    # PUBBLICAZIONE IMMAGINE
    # ========================================================
    def publish_image(self, image, image_metadata):
        if image is None or image_metadata is None:
            return

	# print("Publishing an image\n")
        # convertire in formato ROS Image
        ros_img = self.bridge.cv2_to_imgmsg(image, encoding='mono8')
        ros_img.header.seq = self.image_index
        ros_img.header.stamp = image_metadata[0]['ping_stamp']
        ros_img.header.frame_id = "sss_waterfall"
        
        metadata_msg = ImageMetadata()
        metadata_msg.header = ros_img.header
        metadata_msg.image = ros_img
        metadata_msg.ping_indices = [metadata['ping_index'] for metadata in image_metadata]
        metadata_msg.ping_stamps = [metadata['ping_stamp'] for metadata in image_metadata]
        metadata_msg.nav_statuses = [metadata['nav_status'] for metadata in image_metadata]
        metadata_msg.nav_valid = [metadata['nav_valid'] for metadata in image_metadata]
        metadata_msg.altitudes = [metadata['altitude'] for metadata in image_metadata]
        metadata_msg.altitude_valid = [metadata['altitude_valid'] for metadata in image_metadata]

        # pubblicare immagine e metadata sul topic waterfall_creator
        self.pub_img.publish(metadata_msg)
        
             
        # salvare anche il formato .png
        filename = os.path.join(self.waterfall_image_folder, "sonar_{:03d}.png".format(self.image_index))
        cv2.imwrite(filename, image)
        print("Immagine salvata: {}".format(filename))
        self.image_index += 1

        # mantenere i 50 ping piu recenti per l'overlap con l'immagine successiva
        self.ping_buffer = self.ping_buffer[:self.ping_overlap]
        self.ping_metadata_buffer = self.ping_metadata_buffer[:self.ping_overlap]

# ________________________________________________________________________________________________________________________________

    # ========================================================
    # PLOT INTENSITA (opzionale)
    # ========================================================
    def store_echo_intensity(self, raw_ping, processed_ping):
        self.raw_echo_buffer.append(np.asarray(raw_ping, dtype=np.float32))
        self.processed_echo_buffer.append(np.asarray(processed_ping, dtype=np.float32))

    def plot_echo_intensity_3d(self, waterfall, title, filename):
        if len(waterfall) == 0:
            return None

        z_intensity = np.asarray(waterfall, dtype=np.float32)
        x_axis = np.arange(z_intensity.shape[1])
        y_axis = np.arange(z_intensity.shape[0])
        x_grid, y_grid = np.meshgrid(x_axis, y_axis)

        fig = plt.figure(figsize=(16, 10))
        ax = fig.add_subplot(111, projection='3d')
        surface = ax.plot_surface(x_grid, y_grid, z_intensity, cmap='viridis', linewidth=0, antialiased=False)

        nadir_idx = z_intensity.shape[1] // 2
        ax.plot([nadir_idx] * z_intensity.shape[0], y_axis,
            np.max(z_intensity) * np.ones(z_intensity.shape[0]), color='red', linewidth=2)
        
        ax.set_xlabel('Across-track bins')
        ax.set_ylabel('Ping number')
        ax.set_zlabel('Echo intensity')
        ax.set_title(title)

        fig.colorbar(surface, shrink=0.5, aspect=10)
        fig.savefig(filename, dpi=120, bbox_inches='tight')
        plt.close(fig)
        return filename

    def plot_echo_intensity_2d(self, raw_waterfall, processed_waterfall, filename, ping_number=None):
        if len(raw_waterfall) == 0 or len(processed_waterfall) == 0:
            return None

        raw_matrix = np.asarray(raw_waterfall, dtype=np.float32)
        processed_matrix = np.asarray(processed_waterfall, dtype=np.float32)

        if ping_number is None:
            ping_number = np.random.randint(0, raw_matrix.shape[0])
        ping_number = int(np.clip(ping_number, 0, raw_matrix.shape[0] - 1))

        fig, ax = plt.subplots(figsize=(14, 6))
        ax.plot(np.arange(raw_matrix.shape[1]), raw_matrix[ping_number], color='gray', linewidth=1.2, label='Before TVG')
        ax.plot(np.arange(processed_matrix.shape[1]), processed_matrix[ping_number], color='blue', linewidth=1.2, label='After TVG + slant correction')
        ax.axvline(raw_matrix.shape[1] // 2, color='red', linewidth=1.5, label='Nadir')

        ax.set_xlabel('Across-track bins')
        ax.set_ylabel('Echo intensity')
        ax.set_title('2D Echo Intensity - ping {}'.format(ping_number))
        ax.grid(True, alpha=0.3)
        ax.legend()

        fig.savefig(filename, dpi=120, bbox_inches='tight')
        plt.close(fig)
        return filename

    def save_echo_intensity_plots(self):
        if len(self.raw_echo_buffer) == 0:
            print("No echo intensity data collected, plots not generated")
            return

        raw_3d_filename = os.path.join(self.echo_plot_folder, "echo_intensity_raw_3d_{:03d}.png".format(self.echo_plot_index))
        processed_3d_filename = os.path.join(self.echo_plot_folder, "echo_intensity_processed_3d_{:03d}.png".format(self.echo_plot_index))
        comparison_2d_filename = os.path.join(self.echo_plot_folder, "echo_intensity_comparison_2d_{:03d}.png".format(self.echo_plot_index))

        print("Generating final echo intensity plots")
        self.plot_echo_intensity_3d(self.raw_echo_buffer, "Raw 3D Side Scan Sonar Intensity", raw_3d_filename)
        self.plot_echo_intensity_3d(self.processed_echo_buffer, "After TVG + Slant Correction 3D Side Scan Sonar Intensity", processed_3d_filename)
        self.plot_echo_intensity_2d(self.raw_echo_buffer, self.processed_echo_buffer, comparison_2d_filename)

        print("Echo intensity plots saved in {}".format(self.echo_plot_folder))
        self.echo_plot_index += 1


    # ========================================================
    # PLOT ALTITUDE DI ZENO (opzionale)
    # ========================================================
    def get_elapsed_time(self):
        now = rospy.get_time()
        if self.altitude_start_time is None:
            self.altitude_start_time = now
        return now - self.altitude_start_time

    def store_altitude_sample(self):
        if self.altitude is None:
            return
        self.altitude_samples.append((self.get_elapsed_time(), float(self.altitude)))

    def save_altitude_plot(self):
        if len(self.altitude_samples) == 0:
            print("No altitude data collected, plot not generated")
            return

        fig, ax = plt.subplots(figsize=(14, 6))

        altitude_matrix = np.asarray(self.altitude_samples, dtype=np.float64)
        ax.plot(altitude_matrix[:, 0], altitude_matrix[:, 1], color='tab:blue', linewidth=1.2, label='Altitude')

        ax.set_xlabel('Time [s]')
        ax.set_ylabel('Altitude [m]')
        ax.set_title('Altitude over time')
        ax.grid(True, alpha=0.3)
        ax.legend()

        filename = os.path.join(self.altitude_plot_folder, "altitude_over_time.png")
        fig.savefig(filename, dpi=120, bbox_inches='tight')
        plt.close(fig)
        print("Altitude plot saved: {}".format(filename))
        return filename

    # ========================================================
    # PLOT TRAIETTORIA DI ZENO (opzionale)
    # ========================================================
    #def plot_auv_trajectory(self, nav_status_stream): 
        # estrarre latitudine e longitudine
        # convertire coordinate in sistema cartesiano
        # plottare traiettoria su assi x-y
        #pass




# ============================================================
# MAIN
# ============================================================
if __name__ == "__main__":
    # try:
        # inizializzare nodo ROS
        rospy.init_node('image_creator', anonymous=True)
        # istanziare ImageCreatorNode
        node = ImageCreatorNode()
        # spin ROS
        rospy.spin()
	print("All done :)\n")
    # except rospy.ROSInterruptException:
        # pass
