
(cl:in-package :asdf)

(defsystem "sss_package-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :marta_msgs-msg
               :sensor_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "GeolocatedObject" :depends-on ("_package_GeolocatedObject"))
    (:file "_package_GeolocatedObject" :depends-on ("_package"))
    (:file "GeolocatedObjectList" :depends-on ("_package_GeolocatedObjectList"))
    (:file "_package_GeolocatedObjectList" :depends-on ("_package"))
    (:file "ImageMetadata" :depends-on ("_package_ImageMetadata"))
    (:file "_package_ImageMetadata" :depends-on ("_package"))
  ))