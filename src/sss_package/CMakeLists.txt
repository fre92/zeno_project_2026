cmake_minimum_required(VERSION 2.8.3)
project(sss_package)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  marta_msgs
  message_generation
)


add_message_files(
  FILES
  GeolocatedObject.msg
  GeolocatedObjectList.msg
  ImageMetadata.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  marta_msgs
)

catkin_package(
  CATKIN_DEPENDS
  rospy
  std_msgs
  sensor_msgs
  marta_msgs
  message_runtime
)

catkin_install_python(PROGRAMS
  scripts/image_creator.py
  scripts/list_creator.py
  scripts/geolocalization.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
