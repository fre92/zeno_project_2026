;; Auto-generated. Do not edit!


(when (boundp 'sss_package::GeolocatedObject)
  (if (not (find-package "SSS_PACKAGE"))
    (make-package "SSS_PACKAGE"))
  (shadow 'GeolocatedObject (find-package "SSS_PACKAGE")))
(unless (find-package "SSS_PACKAGE::GEOLOCATEDOBJECT")
  (make-package "SSS_PACKAGE::GEOLOCATEDOBJECT"))

(in-package "ROS")
;;//! \htmlinclude GeolocatedObject.msg.html


(defclass sss_package::GeolocatedObject
  :super ros::object
  :slots (_object_class _confidence _latitude _longitude _ping_index _ping_stamp _centroid_x_px _centroid_y_px _bbox_x_px _bbox_y_px _bbox_width_px _bbox_height_px ))

(defmethod sss_package::GeolocatedObject
  (:init
   (&key
    ((:object_class __object_class) "")
    ((:confidence __confidence) 0.0)
    ((:latitude __latitude) 0.0)
    ((:longitude __longitude) 0.0)
    ((:ping_index __ping_index) 0)
    ((:ping_stamp __ping_stamp) (instance ros::time :init))
    ((:centroid_x_px __centroid_x_px) 0.0)
    ((:centroid_y_px __centroid_y_px) 0.0)
    ((:bbox_x_px __bbox_x_px) 0)
    ((:bbox_y_px __bbox_y_px) 0)
    ((:bbox_width_px __bbox_width_px) 0)
    ((:bbox_height_px __bbox_height_px) 0)
    )
   (send-super :init)
   (setq _object_class (string __object_class))
   (setq _confidence (float __confidence))
   (setq _latitude (float __latitude))
   (setq _longitude (float __longitude))
   (setq _ping_index (round __ping_index))
   (setq _ping_stamp __ping_stamp)
   (setq _centroid_x_px (float __centroid_x_px))
   (setq _centroid_y_px (float __centroid_y_px))
   (setq _bbox_x_px (round __bbox_x_px))
   (setq _bbox_y_px (round __bbox_y_px))
   (setq _bbox_width_px (round __bbox_width_px))
   (setq _bbox_height_px (round __bbox_height_px))
   self)
  (:object_class
   (&optional __object_class)
   (if __object_class (setq _object_class __object_class)) _object_class)
  (:confidence
   (&optional __confidence)
   (if __confidence (setq _confidence __confidence)) _confidence)
  (:latitude
   (&optional __latitude)
   (if __latitude (setq _latitude __latitude)) _latitude)
  (:longitude
   (&optional __longitude)
   (if __longitude (setq _longitude __longitude)) _longitude)
  (:ping_index
   (&optional __ping_index)
   (if __ping_index (setq _ping_index __ping_index)) _ping_index)
  (:ping_stamp
   (&optional __ping_stamp)
   (if __ping_stamp (setq _ping_stamp __ping_stamp)) _ping_stamp)
  (:centroid_x_px
   (&optional __centroid_x_px)
   (if __centroid_x_px (setq _centroid_x_px __centroid_x_px)) _centroid_x_px)
  (:centroid_y_px
   (&optional __centroid_y_px)
   (if __centroid_y_px (setq _centroid_y_px __centroid_y_px)) _centroid_y_px)
  (:bbox_x_px
   (&optional __bbox_x_px)
   (if __bbox_x_px (setq _bbox_x_px __bbox_x_px)) _bbox_x_px)
  (:bbox_y_px
   (&optional __bbox_y_px)
   (if __bbox_y_px (setq _bbox_y_px __bbox_y_px)) _bbox_y_px)
  (:bbox_width_px
   (&optional __bbox_width_px)
   (if __bbox_width_px (setq _bbox_width_px __bbox_width_px)) _bbox_width_px)
  (:bbox_height_px
   (&optional __bbox_height_px)
   (if __bbox_height_px (setq _bbox_height_px __bbox_height_px)) _bbox_height_px)
  (:serialization-length
   ()
   (+
    ;; string _object_class
    4 (length _object_class)
    ;; float32 _confidence
    4
    ;; float64 _latitude
    8
    ;; float64 _longitude
    8
    ;; uint32 _ping_index
    4
    ;; time _ping_stamp
    8
    ;; float32 _centroid_x_px
    4
    ;; float32 _centroid_y_px
    4
    ;; int32 _bbox_x_px
    4
    ;; int32 _bbox_y_px
    4
    ;; int32 _bbox_width_px
    4
    ;; int32 _bbox_height_px
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _object_class
       (write-long (length _object_class) s) (princ _object_class s)
     ;; float32 _confidence
       (sys::poke _confidence (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float64 _latitude
       (sys::poke _latitude (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _longitude
       (sys::poke _longitude (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; uint32 _ping_index
       (write-long _ping_index s)
     ;; time _ping_stamp
       (write-long (send _ping_stamp :sec) s) (write-long (send _ping_stamp :nsec) s)
     ;; float32 _centroid_x_px
       (sys::poke _centroid_x_px (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _centroid_y_px
       (sys::poke _centroid_y_px (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int32 _bbox_x_px
       (write-long _bbox_x_px s)
     ;; int32 _bbox_y_px
       (write-long _bbox_y_px s)
     ;; int32 _bbox_width_px
       (write-long _bbox_width_px s)
     ;; int32 _bbox_height_px
       (write-long _bbox_height_px s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _object_class
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _object_class (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _confidence
     (setq _confidence (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float64 _latitude
     (setq _latitude (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _longitude
     (setq _longitude (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; uint32 _ping_index
     (setq _ping_index (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; time _ping_stamp
     (send _ping_stamp :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _ping_stamp :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _centroid_x_px
     (setq _centroid_x_px (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _centroid_y_px
     (setq _centroid_y_px (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int32 _bbox_x_px
     (setq _bbox_x_px (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _bbox_y_px
     (setq _bbox_y_px (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _bbox_width_px
     (setq _bbox_width_px (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _bbox_height_px
     (setq _bbox_height_px (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get sss_package::GeolocatedObject :md5sum-) "09b78b232110d33d86e8f0f2984e6e72")
(setf (get sss_package::GeolocatedObject :datatype-) "sss_package/GeolocatedObject")
(setf (get sss_package::GeolocatedObject :definition-)
      "string object_class
float32 confidence

float64 latitude
float64 longitude

uint32 ping_index
time ping_stamp
float32 centroid_x_px
float32 centroid_y_px

int32 bbox_x_px
int32 bbox_y_px
int32 bbox_width_px
int32 bbox_height_px

")



(provide :sss_package/GeolocatedObject "09b78b232110d33d86e8f0f2984e6e72")


