; Auto-generated. Do not edit!


(cl:in-package sss_package-msg)


;//! \htmlinclude GeolocatedObject.msg.html

(cl:defclass <GeolocatedObject> (roslisp-msg-protocol:ros-message)
  ((object_class
    :reader object_class
    :initarg :object_class
    :type cl:string
    :initform "")
   (confidence
    :reader confidence
    :initarg :confidence
    :type cl:float
    :initform 0.0)
   (latitude
    :reader latitude
    :initarg :latitude
    :type cl:float
    :initform 0.0)
   (longitude
    :reader longitude
    :initarg :longitude
    :type cl:float
    :initform 0.0)
   (ping_index
    :reader ping_index
    :initarg :ping_index
    :type cl:integer
    :initform 0)
   (ping_stamp
    :reader ping_stamp
    :initarg :ping_stamp
    :type cl:real
    :initform 0)
   (centroid_x_px
    :reader centroid_x_px
    :initarg :centroid_x_px
    :type cl:float
    :initform 0.0)
   (centroid_y_px
    :reader centroid_y_px
    :initarg :centroid_y_px
    :type cl:float
    :initform 0.0)
   (bbox_x_px
    :reader bbox_x_px
    :initarg :bbox_x_px
    :type cl:integer
    :initform 0)
   (bbox_y_px
    :reader bbox_y_px
    :initarg :bbox_y_px
    :type cl:integer
    :initform 0)
   (bbox_width_px
    :reader bbox_width_px
    :initarg :bbox_width_px
    :type cl:integer
    :initform 0)
   (bbox_height_px
    :reader bbox_height_px
    :initarg :bbox_height_px
    :type cl:integer
    :initform 0))
)

(cl:defclass GeolocatedObject (<GeolocatedObject>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GeolocatedObject>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GeolocatedObject)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sss_package-msg:<GeolocatedObject> is deprecated: use sss_package-msg:GeolocatedObject instead.")))

(cl:ensure-generic-function 'object_class-val :lambda-list '(m))
(cl:defmethod object_class-val ((m <GeolocatedObject>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sss_package-msg:object_class-val is deprecated.  Use sss_package-msg:object_class instead.")
  (object_class m))

(cl:ensure-generic-function 'confidence-val :lambda-list '(m))
(cl:defmethod confidence-val ((m <GeolocatedObject>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sss_package-msg:confidence-val is deprecated.  Use sss_package-msg:confidence instead.")
  (confidence m))

(cl:ensure-generic-function 'latitude-val :lambda-list '(m))
(cl:defmethod latitude-val ((m <GeolocatedObject>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sss_package-msg:latitude-val is deprecated.  Use sss_package-msg:latitude instead.")
  (latitude m))

(cl:ensure-generic-function 'longitude-val :lambda-list '(m))
(cl:defmethod longitude-val ((m <GeolocatedObject>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sss_package-msg:longitude-val is deprecated.  Use sss_package-msg:longitude instead.")
  (longitude m))

(cl:ensure-generic-function 'ping_index-val :lambda-list '(m))
(cl:defmethod ping_index-val ((m <GeolocatedObject>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sss_package-msg:ping_index-val is deprecated.  Use sss_package-msg:ping_index instead.")
  (ping_index m))

(cl:ensure-generic-function 'ping_stamp-val :lambda-list '(m))
(cl:defmethod ping_stamp-val ((m <GeolocatedObject>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sss_package-msg:ping_stamp-val is deprecated.  Use sss_package-msg:ping_stamp instead.")
  (ping_stamp m))

(cl:ensure-generic-function 'centroid_x_px-val :lambda-list '(m))
(cl:defmethod centroid_x_px-val ((m <GeolocatedObject>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sss_package-msg:centroid_x_px-val is deprecated.  Use sss_package-msg:centroid_x_px instead.")
  (centroid_x_px m))

(cl:ensure-generic-function 'centroid_y_px-val :lambda-list '(m))
(cl:defmethod centroid_y_px-val ((m <GeolocatedObject>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sss_package-msg:centroid_y_px-val is deprecated.  Use sss_package-msg:centroid_y_px instead.")
  (centroid_y_px m))

(cl:ensure-generic-function 'bbox_x_px-val :lambda-list '(m))
(cl:defmethod bbox_x_px-val ((m <GeolocatedObject>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sss_package-msg:bbox_x_px-val is deprecated.  Use sss_package-msg:bbox_x_px instead.")
  (bbox_x_px m))

(cl:ensure-generic-function 'bbox_y_px-val :lambda-list '(m))
(cl:defmethod bbox_y_px-val ((m <GeolocatedObject>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sss_package-msg:bbox_y_px-val is deprecated.  Use sss_package-msg:bbox_y_px instead.")
  (bbox_y_px m))

(cl:ensure-generic-function 'bbox_width_px-val :lambda-list '(m))
(cl:defmethod bbox_width_px-val ((m <GeolocatedObject>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sss_package-msg:bbox_width_px-val is deprecated.  Use sss_package-msg:bbox_width_px instead.")
  (bbox_width_px m))

(cl:ensure-generic-function 'bbox_height_px-val :lambda-list '(m))
(cl:defmethod bbox_height_px-val ((m <GeolocatedObject>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sss_package-msg:bbox_height_px-val is deprecated.  Use sss_package-msg:bbox_height_px instead.")
  (bbox_height_px m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GeolocatedObject>) ostream)
  "Serializes a message object of type '<GeolocatedObject>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'object_class))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'object_class))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'confidence))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'latitude))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'longitude))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ping_index)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'ping_index)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'ping_index)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'ping_index)) ostream)
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'ping_stamp)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'ping_stamp) (cl:floor (cl:slot-value msg 'ping_stamp)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'centroid_x_px))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'centroid_y_px))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'bbox_x_px)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'bbox_y_px)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'bbox_width_px)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'bbox_height_px)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GeolocatedObject>) istream)
  "Deserializes a message object of type '<GeolocatedObject>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'object_class) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'object_class) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'confidence) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'latitude) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'longitude) (roslisp-utils:decode-double-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ping_index)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'ping_index)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'ping_index)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'ping_index)) (cl:read-byte istream))
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ping_stamp) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'centroid_x_px) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'centroid_y_px) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'bbox_x_px) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'bbox_y_px) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'bbox_width_px) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'bbox_height_px) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GeolocatedObject>)))
  "Returns string type for a message object of type '<GeolocatedObject>"
  "sss_package/GeolocatedObject")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GeolocatedObject)))
  "Returns string type for a message object of type 'GeolocatedObject"
  "sss_package/GeolocatedObject")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GeolocatedObject>)))
  "Returns md5sum for a message object of type '<GeolocatedObject>"
  "09b78b232110d33d86e8f0f2984e6e72")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GeolocatedObject)))
  "Returns md5sum for a message object of type 'GeolocatedObject"
  "09b78b232110d33d86e8f0f2984e6e72")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GeolocatedObject>)))
  "Returns full string definition for message of type '<GeolocatedObject>"
  (cl:format cl:nil "string object_class~%float32 confidence~%~%float64 latitude~%float64 longitude~%~%uint32 ping_index~%time ping_stamp~%float32 centroid_x_px~%float32 centroid_y_px~%~%int32 bbox_x_px~%int32 bbox_y_px~%int32 bbox_width_px~%int32 bbox_height_px~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GeolocatedObject)))
  "Returns full string definition for message of type 'GeolocatedObject"
  (cl:format cl:nil "string object_class~%float32 confidence~%~%float64 latitude~%float64 longitude~%~%uint32 ping_index~%time ping_stamp~%float32 centroid_x_px~%float32 centroid_y_px~%~%int32 bbox_x_px~%int32 bbox_y_px~%int32 bbox_width_px~%int32 bbox_height_px~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GeolocatedObject>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'object_class))
     4
     8
     8
     4
     8
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GeolocatedObject>))
  "Converts a ROS message object to a list"
  (cl:list 'GeolocatedObject
    (cl:cons ':object_class (object_class msg))
    (cl:cons ':confidence (confidence msg))
    (cl:cons ':latitude (latitude msg))
    (cl:cons ':longitude (longitude msg))
    (cl:cons ':ping_index (ping_index msg))
    (cl:cons ':ping_stamp (ping_stamp msg))
    (cl:cons ':centroid_x_px (centroid_x_px msg))
    (cl:cons ':centroid_y_px (centroid_y_px msg))
    (cl:cons ':bbox_x_px (bbox_x_px msg))
    (cl:cons ':bbox_y_px (bbox_y_px msg))
    (cl:cons ':bbox_width_px (bbox_width_px msg))
    (cl:cons ':bbox_height_px (bbox_height_px msg))
))
