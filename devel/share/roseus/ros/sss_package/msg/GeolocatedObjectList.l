;; Auto-generated. Do not edit!


(when (boundp 'sss_package::GeolocatedObjectList)
  (if (not (find-package "SSS_PACKAGE"))
    (make-package "SSS_PACKAGE"))
  (shadow 'GeolocatedObjectList (find-package "SSS_PACKAGE")))
(unless (find-package "SSS_PACKAGE::GEOLOCATEDOBJECTLIST")
  (make-package "SSS_PACKAGE::GEOLOCATEDOBJECTLIST"))

(in-package "ROS")
;;//! \htmlinclude GeolocatedObjectList.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass sss_package::GeolocatedObjectList
  :super ros::object
  :slots (_header _objects ))

(defmethod sss_package::GeolocatedObjectList
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:objects __objects) (let (r) (dotimes (i 0) (push (instance sss_package::GeolocatedObject :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _objects __objects)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:objects
   (&rest __objects)
   (if (keywordp (car __objects))
       (send* _objects __objects)
     (progn
       (if __objects (setq _objects (car __objects)))
       _objects)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; sss_package/GeolocatedObject[] _objects
    (apply #'+ (send-all _objects :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; sss_package/GeolocatedObject[] _objects
     (write-long (length _objects) s)
     (dolist (elem _objects)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; sss_package/GeolocatedObject[] _objects
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _objects (let (r) (dotimes (i n) (push (instance sss_package::GeolocatedObject :init) r)) r))
     (dolist (elem- _objects)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get sss_package::GeolocatedObjectList :md5sum-) "137da611e49ad8c24c63b5e9c7e9e282")
(setf (get sss_package::GeolocatedObjectList :datatype-) "sss_package/GeolocatedObjectList")
(setf (get sss_package::GeolocatedObjectList :definition-)
      "Header header
sss_package/GeolocatedObject[] objects

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
MSG: sss_package/GeolocatedObject
string object_class
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



(provide :sss_package/GeolocatedObjectList "137da611e49ad8c24c63b5e9c7e9e282")


