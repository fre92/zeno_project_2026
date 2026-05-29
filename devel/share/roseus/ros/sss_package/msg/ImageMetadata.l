;; Auto-generated. Do not edit!


(when (boundp 'sss_package::ImageMetadata)
  (if (not (find-package "SSS_PACKAGE"))
    (make-package "SSS_PACKAGE"))
  (shadow 'ImageMetadata (find-package "SSS_PACKAGE")))
(unless (find-package "SSS_PACKAGE::IMAGEMETADATA")
  (make-package "SSS_PACKAGE::IMAGEMETADATA"))

(in-package "ROS")
;;//! \htmlinclude ImageMetadata.msg.html
(if (not (find-package "MARTA_MSGS"))
  (ros::roseus-add-msgs "marta_msgs"))
(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass sss_package::ImageMetadata
  :super ros::object
  :slots (_header _image _ping_indices _ping_stamps _nav_statuses _nav_valid _altitudes _altitude_valid _object_classes _object_confidences _object_centroid_x_px _object_centroid_y_px _object_bbox_x_px _object_bbox_y_px _object_bbox_width_px _object_bbox_height_px ))

(defmethod sss_package::ImageMetadata
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:image __image) (instance sensor_msgs::Image :init))
    ((:ping_indices __ping_indices) (make-array 0 :initial-element 0 :element-type :integer))
    ((:ping_stamps __ping_stamps) (let (r) (dotimes (i 0) (push (instance ros::time :init) r)) r))
    ((:nav_statuses __nav_statuses) (let (r) (dotimes (i 0) (push (instance marta_msgs::NavStatus :init) r)) r))
    ((:nav_valid __nav_valid) (let (r) (dotimes (i 0) (push nil r)) r))
    ((:altitudes __altitudes) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:altitude_valid __altitude_valid) (let (r) (dotimes (i 0) (push nil r)) r))
    ((:object_classes __object_classes) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:object_confidences __object_confidences) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:object_centroid_x_px __object_centroid_x_px) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:object_centroid_y_px __object_centroid_y_px) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:object_bbox_x_px __object_bbox_x_px) (make-array 0 :initial-element 0 :element-type :integer))
    ((:object_bbox_y_px __object_bbox_y_px) (make-array 0 :initial-element 0 :element-type :integer))
    ((:object_bbox_width_px __object_bbox_width_px) (make-array 0 :initial-element 0 :element-type :integer))
    ((:object_bbox_height_px __object_bbox_height_px) (make-array 0 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _header __header)
   (setq _image __image)
   (setq _ping_indices __ping_indices)
   (setq _ping_stamps __ping_stamps)
   (setq _nav_statuses __nav_statuses)
   (setq _nav_valid __nav_valid)
   (setq _altitudes __altitudes)
   (setq _altitude_valid __altitude_valid)
   (setq _object_classes __object_classes)
   (setq _object_confidences __object_confidences)
   (setq _object_centroid_x_px __object_centroid_x_px)
   (setq _object_centroid_y_px __object_centroid_y_px)
   (setq _object_bbox_x_px __object_bbox_x_px)
   (setq _object_bbox_y_px __object_bbox_y_px)
   (setq _object_bbox_width_px __object_bbox_width_px)
   (setq _object_bbox_height_px __object_bbox_height_px)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:image
   (&rest __image)
   (if (keywordp (car __image))
       (send* _image __image)
     (progn
       (if __image (setq _image (car __image)))
       _image)))
  (:ping_indices
   (&optional __ping_indices)
   (if __ping_indices (setq _ping_indices __ping_indices)) _ping_indices)
  (:ping_stamps
   (&optional __ping_stamps)
   (if __ping_stamps (setq _ping_stamps __ping_stamps)) _ping_stamps)
  (:nav_statuses
   (&rest __nav_statuses)
   (if (keywordp (car __nav_statuses))
       (send* _nav_statuses __nav_statuses)
     (progn
       (if __nav_statuses (setq _nav_statuses (car __nav_statuses)))
       _nav_statuses)))
  (:nav_valid
   (&optional __nav_valid)
   (if __nav_valid (setq _nav_valid __nav_valid)) _nav_valid)
  (:altitudes
   (&optional __altitudes)
   (if __altitudes (setq _altitudes __altitudes)) _altitudes)
  (:altitude_valid
   (&optional __altitude_valid)
   (if __altitude_valid (setq _altitude_valid __altitude_valid)) _altitude_valid)
  (:object_classes
   (&optional __object_classes)
   (if __object_classes (setq _object_classes __object_classes)) _object_classes)
  (:object_confidences
   (&optional __object_confidences)
   (if __object_confidences (setq _object_confidences __object_confidences)) _object_confidences)
  (:object_centroid_x_px
   (&optional __object_centroid_x_px)
   (if __object_centroid_x_px (setq _object_centroid_x_px __object_centroid_x_px)) _object_centroid_x_px)
  (:object_centroid_y_px
   (&optional __object_centroid_y_px)
   (if __object_centroid_y_px (setq _object_centroid_y_px __object_centroid_y_px)) _object_centroid_y_px)
  (:object_bbox_x_px
   (&optional __object_bbox_x_px)
   (if __object_bbox_x_px (setq _object_bbox_x_px __object_bbox_x_px)) _object_bbox_x_px)
  (:object_bbox_y_px
   (&optional __object_bbox_y_px)
   (if __object_bbox_y_px (setq _object_bbox_y_px __object_bbox_y_px)) _object_bbox_y_px)
  (:object_bbox_width_px
   (&optional __object_bbox_width_px)
   (if __object_bbox_width_px (setq _object_bbox_width_px __object_bbox_width_px)) _object_bbox_width_px)
  (:object_bbox_height_px
   (&optional __object_bbox_height_px)
   (if __object_bbox_height_px (setq _object_bbox_height_px __object_bbox_height_px)) _object_bbox_height_px)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; sensor_msgs/Image _image
    (send _image :serialization-length)
    ;; uint32[] _ping_indices
    (* 4    (length _ping_indices)) 4
    ;; time[] _ping_stamps
    (* 8    (length _ping_stamps)) 4
    ;; marta_msgs/NavStatus[] _nav_statuses
    (apply #'+ (send-all _nav_statuses :serialization-length)) 4
    ;; bool[] _nav_valid
    (* 1    (length _nav_valid)) 4
    ;; float32[] _altitudes
    (* 4    (length _altitudes)) 4
    ;; bool[] _altitude_valid
    (* 1    (length _altitude_valid)) 4
    ;; string[] _object_classes
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _object_classes)) 4
    ;; float32[] _object_confidences
    (* 4    (length _object_confidences)) 4
    ;; float32[] _object_centroid_x_px
    (* 4    (length _object_centroid_x_px)) 4
    ;; float32[] _object_centroid_y_px
    (* 4    (length _object_centroid_y_px)) 4
    ;; int32[] _object_bbox_x_px
    (* 4    (length _object_bbox_x_px)) 4
    ;; int32[] _object_bbox_y_px
    (* 4    (length _object_bbox_y_px)) 4
    ;; int32[] _object_bbox_width_px
    (* 4    (length _object_bbox_width_px)) 4
    ;; int32[] _object_bbox_height_px
    (* 4    (length _object_bbox_height_px)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; sensor_msgs/Image _image
       (send _image :serialize s)
     ;; uint32[] _ping_indices
     (write-long (length _ping_indices) s)
     (dotimes (i (length _ping_indices))
       (write-long (elt _ping_indices i) s)
       )
     ;; time[] _ping_stamps
     (write-long (length _ping_stamps) s)
     (dotimes (i (length _ping_stamps))
       (write-long (send (elt _ping_stamps i) :sec) s) (write-long (send (elt _ping_stamps i) :nsec) s)
       )
     ;; marta_msgs/NavStatus[] _nav_statuses
     (write-long (length _nav_statuses) s)
     (dolist (elem _nav_statuses)
       (send elem :serialize s)
       )
     ;; bool[] _nav_valid
     (write-long (length _nav_valid) s)
     (dotimes (i (length _nav_valid))
       (if (elt _nav_valid i) (write-byte -1 s) (write-byte 0 s))
       )
     ;; float32[] _altitudes
     (write-long (length _altitudes) s)
     (dotimes (i (length _altitudes))
       (sys::poke (elt _altitudes i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; bool[] _altitude_valid
     (write-long (length _altitude_valid) s)
     (dotimes (i (length _altitude_valid))
       (if (elt _altitude_valid i) (write-byte -1 s) (write-byte 0 s))
       )
     ;; string[] _object_classes
     (write-long (length _object_classes) s)
     (dolist (elem _object_classes)
       (write-long (length elem) s) (princ elem s)
       )
     ;; float32[] _object_confidences
     (write-long (length _object_confidences) s)
     (dotimes (i (length _object_confidences))
       (sys::poke (elt _object_confidences i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _object_centroid_x_px
     (write-long (length _object_centroid_x_px) s)
     (dotimes (i (length _object_centroid_x_px))
       (sys::poke (elt _object_centroid_x_px i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _object_centroid_y_px
     (write-long (length _object_centroid_y_px) s)
     (dotimes (i (length _object_centroid_y_px))
       (sys::poke (elt _object_centroid_y_px i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; int32[] _object_bbox_x_px
     (write-long (length _object_bbox_x_px) s)
     (dotimes (i (length _object_bbox_x_px))
       (write-long (elt _object_bbox_x_px i) s)
       )
     ;; int32[] _object_bbox_y_px
     (write-long (length _object_bbox_y_px) s)
     (dotimes (i (length _object_bbox_y_px))
       (write-long (elt _object_bbox_y_px i) s)
       )
     ;; int32[] _object_bbox_width_px
     (write-long (length _object_bbox_width_px) s)
     (dotimes (i (length _object_bbox_width_px))
       (write-long (elt _object_bbox_width_px i) s)
       )
     ;; int32[] _object_bbox_height_px
     (write-long (length _object_bbox_height_px) s)
     (dotimes (i (length _object_bbox_height_px))
       (write-long (elt _object_bbox_height_px i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; sensor_msgs/Image _image
     (send _image :deserialize buf ptr-) (incf ptr- (send _image :serialization-length))
   ;; uint32[] _ping_indices
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _ping_indices (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _ping_indices i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; time[] _ping_stamps
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _ping_stamps (let (r) (dotimes (i n) (push (instance ros::time :init) r)) r))
     (dolist (elem- _ping_stamps)
     (send elem- :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send elem- :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; marta_msgs/NavStatus[] _nav_statuses
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _nav_statuses (let (r) (dotimes (i n) (push (instance marta_msgs::NavStatus :init) r)) r))
     (dolist (elem- _nav_statuses)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; bool[] _nav_valid
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _nav_valid (make-list n))
     (dotimes (i n)
     (setf (elt _nav_valid i) (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
     ))
   ;; float32[] _altitudes
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _altitudes (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _altitudes i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; bool[] _altitude_valid
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _altitude_valid (make-list n))
     (dotimes (i n)
     (setf (elt _altitude_valid i) (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
     ))
   ;; string[] _object_classes
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _object_classes (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _object_classes i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; float32[] _object_confidences
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _object_confidences (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _object_confidences i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _object_centroid_x_px
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _object_centroid_x_px (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _object_centroid_x_px i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _object_centroid_y_px
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _object_centroid_y_px (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _object_centroid_y_px i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; int32[] _object_bbox_x_px
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _object_bbox_x_px (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _object_bbox_x_px i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; int32[] _object_bbox_y_px
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _object_bbox_y_px (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _object_bbox_y_px i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; int32[] _object_bbox_width_px
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _object_bbox_width_px (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _object_bbox_width_px i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; int32[] _object_bbox_height_px
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _object_bbox_height_px (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _object_bbox_height_px i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get sss_package::ImageMetadata :md5sum-) "b7d71bb022c02e7c92c518f300d617c9")
(setf (get sss_package::ImageMetadata :datatype-) "sss_package/ImageMetadata")
(setf (get sss_package::ImageMetadata :definition-)
      "Header header

sensor_msgs/Image image

uint32[] ping_indices
time[] ping_stamps
marta_msgs/NavStatus[] nav_statuses
bool[] nav_valid
float32[] altitudes
bool[] altitude_valid

string[] object_classes
float32[] object_confidences
float32[] object_centroid_x_px
float32[] object_centroid_y_px
int32[] object_bbox_x_px
int32[] object_bbox_y_px
int32[] object_bbox_width_px
int32[] object_bbox_height_px

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: sensor_msgs/Image
# This message contains an uncompressed image
# (0, 0) is at top-left corner of image
#

Header header        # Header timestamp should be acquisition time of image
                     # Header frame_id should be optical frame of camera
                     # origin of frame should be optical center of camera
                     # +x should point to the right in the image
                     # +y should point down in the image
                     # +z should point into to plane of the image
                     # If the frame_id here and the frame_id of the CameraInfo
                     # message associated with the image conflict
                     # the behavior is undefined

uint32 height         # image height, that is, number of rows
uint32 width          # image width, that is, number of columns

# The legal values for encoding are in file src/image_encodings.cpp
# If you want to standardize a new string format, join
# ros-users@lists.sourceforge.net and send an email proposing a new encoding.

string encoding       # Encoding of pixels -- channel meaning, ordering, size
                      # taken from the list of strings in include/sensor_msgs/image_encodings.h

uint8 is_bigendian    # is this data bigendian?
uint32 step           # Full row length in bytes
uint8[] data          # actual matrix data, size is (step * rows)

================================================================================
MSG: marta_msgs/NavStatus
Header header

marta_msgs/Position position
 
marta_msgs/Euler orientation

marta_msgs/Quaternion quaternion

geometry_msgs/Vector3 ned_speed

geometry_msgs/Vector3 omega_body

uint8 gps_status

bool initialized


================================================================================
MSG: marta_msgs/Position
float64 latitude
float64 longitude
float64 depth

================================================================================
MSG: marta_msgs/Euler
float64 roll
float64 pitch
float64 yaw

================================================================================
MSG: marta_msgs/Quaternion
float64 w
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
")



(provide :sss_package/ImageMetadata "b7d71bb022c02e7c92c518f300d617c9")


