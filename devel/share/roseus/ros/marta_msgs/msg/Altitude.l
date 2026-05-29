;; Auto-generated. Do not edit!


(when (boundp 'marta_msgs::Altitude)
  (if (not (find-package "MARTA_MSGS"))
    (make-package "MARTA_MSGS"))
  (shadow 'Altitude (find-package "MARTA_MSGS")))
(unless (find-package "MARTA_MSGS::ALTITUDE")
  (make-package "MARTA_MSGS::ALTITUDE"))

(in-package "ROS")
;;//! \htmlinclude Altitude.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass marta_msgs::Altitude
  :super ros::object
  :slots (_header _altitude _validity ))

(defmethod marta_msgs::Altitude
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:altitude __altitude) 0.0)
    ((:validity __validity) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _altitude (float __altitude))
   (setq _validity (round __validity))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:altitude
   (&optional __altitude)
   (if __altitude (setq _altitude __altitude)) _altitude)
  (:validity
   (&optional __validity)
   (if __validity (setq _validity __validity)) _validity)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64 _altitude
    8
    ;; uint8 _validity
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64 _altitude
       (sys::poke _altitude (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; uint8 _validity
       (write-byte _validity s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64 _altitude
     (setq _altitude (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; uint8 _validity
     (setq _validity (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get marta_msgs::Altitude :md5sum-) "cba450d8dd1690f277323b357f511c58")
(setf (get marta_msgs::Altitude :datatype-) "marta_msgs/Altitude")
(setf (get marta_msgs::Altitude :definition-)
      "Header header

float64 altitude

uint8 validity
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

")



(provide :marta_msgs/Altitude "cba450d8dd1690f277323b357f511c58")


