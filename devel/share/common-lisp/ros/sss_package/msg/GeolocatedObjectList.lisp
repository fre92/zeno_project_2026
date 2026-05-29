; Auto-generated. Do not edit!


(cl:in-package sss_package-msg)


;//! \htmlinclude GeolocatedObjectList.msg.html

(cl:defclass <GeolocatedObjectList> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (objects
    :reader objects
    :initarg :objects
    :type (cl:vector sss_package-msg:GeolocatedObject)
   :initform (cl:make-array 0 :element-type 'sss_package-msg:GeolocatedObject :initial-element (cl:make-instance 'sss_package-msg:GeolocatedObject))))
)

(cl:defclass GeolocatedObjectList (<GeolocatedObjectList>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GeolocatedObjectList>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GeolocatedObjectList)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sss_package-msg:<GeolocatedObjectList> is deprecated: use sss_package-msg:GeolocatedObjectList instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <GeolocatedObjectList>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sss_package-msg:header-val is deprecated.  Use sss_package-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'objects-val :lambda-list '(m))
(cl:defmethod objects-val ((m <GeolocatedObjectList>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sss_package-msg:objects-val is deprecated.  Use sss_package-msg:objects instead.")
  (objects m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GeolocatedObjectList>) ostream)
  "Serializes a message object of type '<GeolocatedObjectList>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'objects))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'objects))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GeolocatedObjectList>) istream)
  "Deserializes a message object of type '<GeolocatedObjectList>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'objects) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'objects)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'sss_package-msg:GeolocatedObject))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GeolocatedObjectList>)))
  "Returns string type for a message object of type '<GeolocatedObjectList>"
  "sss_package/GeolocatedObjectList")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GeolocatedObjectList)))
  "Returns string type for a message object of type 'GeolocatedObjectList"
  "sss_package/GeolocatedObjectList")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GeolocatedObjectList>)))
  "Returns md5sum for a message object of type '<GeolocatedObjectList>"
  "137da611e49ad8c24c63b5e9c7e9e282")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GeolocatedObjectList)))
  "Returns md5sum for a message object of type 'GeolocatedObjectList"
  "137da611e49ad8c24c63b5e9c7e9e282")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GeolocatedObjectList>)))
  "Returns full string definition for message of type '<GeolocatedObjectList>"
  (cl:format cl:nil "Header header~%sss_package/GeolocatedObject[] objects~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: sss_package/GeolocatedObject~%string object_class~%float32 confidence~%~%float64 latitude~%float64 longitude~%~%uint32 ping_index~%time ping_stamp~%float32 centroid_x_px~%float32 centroid_y_px~%~%int32 bbox_x_px~%int32 bbox_y_px~%int32 bbox_width_px~%int32 bbox_height_px~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GeolocatedObjectList)))
  "Returns full string definition for message of type 'GeolocatedObjectList"
  (cl:format cl:nil "Header header~%sss_package/GeolocatedObject[] objects~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: sss_package/GeolocatedObject~%string object_class~%float32 confidence~%~%float64 latitude~%float64 longitude~%~%uint32 ping_index~%time ping_stamp~%float32 centroid_x_px~%float32 centroid_y_px~%~%int32 bbox_x_px~%int32 bbox_y_px~%int32 bbox_width_px~%int32 bbox_height_px~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GeolocatedObjectList>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'objects) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GeolocatedObjectList>))
  "Converts a ROS message object to a list"
  (cl:list 'GeolocatedObjectList
    (cl:cons ':header (header msg))
    (cl:cons ':objects (objects msg))
))
