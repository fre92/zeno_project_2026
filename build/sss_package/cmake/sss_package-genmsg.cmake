# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "sss_package: 3 messages, 0 services")

set(MSG_I_FLAGS "-Isss_package:/home/student/catkin_ws/src/sss_package/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/melodic/share/sensor_msgs/cmake/../msg;-Imarta_msgs:/home/student/catkin_ws/src/marta_msgs/msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(sss_package_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/student/catkin_ws/src/sss_package/msg/GeolocatedObject.msg" NAME_WE)
add_custom_target(_sss_package_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sss_package" "/home/student/catkin_ws/src/sss_package/msg/GeolocatedObject.msg" ""
)

get_filename_component(_filename "/home/student/catkin_ws/src/sss_package/msg/GeolocatedObjectList.msg" NAME_WE)
add_custom_target(_sss_package_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sss_package" "/home/student/catkin_ws/src/sss_package/msg/GeolocatedObjectList.msg" "sss_package/GeolocatedObject:std_msgs/Header"
)

get_filename_component(_filename "/home/student/catkin_ws/src/sss_package/msg/ImageMetadata.msg" NAME_WE)
add_custom_target(_sss_package_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sss_package" "/home/student/catkin_ws/src/sss_package/msg/ImageMetadata.msg" "sensor_msgs/Image:marta_msgs/NavStatus:marta_msgs/Position:marta_msgs/Quaternion:geometry_msgs/Vector3:std_msgs/Header:marta_msgs/Euler"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(sss_package
  "/home/student/catkin_ws/src/sss_package/msg/GeolocatedObject.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sss_package
)
_generate_msg_cpp(sss_package
  "/home/student/catkin_ws/src/sss_package/msg/GeolocatedObjectList.msg"
  "${MSG_I_FLAGS}"
  "/home/student/catkin_ws/src/sss_package/msg/GeolocatedObject.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sss_package
)
_generate_msg_cpp(sss_package
  "/home/student/catkin_ws/src/sss_package/msg/ImageMetadata.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/sensor_msgs/cmake/../msg/Image.msg;/home/student/catkin_ws/src/marta_msgs/msg/NavStatus.msg;/home/student/catkin_ws/src/marta_msgs/msg/Position.msg;/home/student/catkin_ws/src/marta_msgs/msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/student/catkin_ws/src/marta_msgs/msg/Euler.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sss_package
)

### Generating Services

### Generating Module File
_generate_module_cpp(sss_package
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sss_package
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(sss_package_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(sss_package_generate_messages sss_package_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/student/catkin_ws/src/sss_package/msg/GeolocatedObject.msg" NAME_WE)
add_dependencies(sss_package_generate_messages_cpp _sss_package_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/student/catkin_ws/src/sss_package/msg/GeolocatedObjectList.msg" NAME_WE)
add_dependencies(sss_package_generate_messages_cpp _sss_package_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/student/catkin_ws/src/sss_package/msg/ImageMetadata.msg" NAME_WE)
add_dependencies(sss_package_generate_messages_cpp _sss_package_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(sss_package_gencpp)
add_dependencies(sss_package_gencpp sss_package_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS sss_package_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(sss_package
  "/home/student/catkin_ws/src/sss_package/msg/GeolocatedObject.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sss_package
)
_generate_msg_eus(sss_package
  "/home/student/catkin_ws/src/sss_package/msg/GeolocatedObjectList.msg"
  "${MSG_I_FLAGS}"
  "/home/student/catkin_ws/src/sss_package/msg/GeolocatedObject.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sss_package
)
_generate_msg_eus(sss_package
  "/home/student/catkin_ws/src/sss_package/msg/ImageMetadata.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/sensor_msgs/cmake/../msg/Image.msg;/home/student/catkin_ws/src/marta_msgs/msg/NavStatus.msg;/home/student/catkin_ws/src/marta_msgs/msg/Position.msg;/home/student/catkin_ws/src/marta_msgs/msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/student/catkin_ws/src/marta_msgs/msg/Euler.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sss_package
)

### Generating Services

### Generating Module File
_generate_module_eus(sss_package
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sss_package
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(sss_package_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(sss_package_generate_messages sss_package_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/student/catkin_ws/src/sss_package/msg/GeolocatedObject.msg" NAME_WE)
add_dependencies(sss_package_generate_messages_eus _sss_package_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/student/catkin_ws/src/sss_package/msg/GeolocatedObjectList.msg" NAME_WE)
add_dependencies(sss_package_generate_messages_eus _sss_package_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/student/catkin_ws/src/sss_package/msg/ImageMetadata.msg" NAME_WE)
add_dependencies(sss_package_generate_messages_eus _sss_package_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(sss_package_geneus)
add_dependencies(sss_package_geneus sss_package_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS sss_package_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(sss_package
  "/home/student/catkin_ws/src/sss_package/msg/GeolocatedObject.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sss_package
)
_generate_msg_lisp(sss_package
  "/home/student/catkin_ws/src/sss_package/msg/GeolocatedObjectList.msg"
  "${MSG_I_FLAGS}"
  "/home/student/catkin_ws/src/sss_package/msg/GeolocatedObject.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sss_package
)
_generate_msg_lisp(sss_package
  "/home/student/catkin_ws/src/sss_package/msg/ImageMetadata.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/sensor_msgs/cmake/../msg/Image.msg;/home/student/catkin_ws/src/marta_msgs/msg/NavStatus.msg;/home/student/catkin_ws/src/marta_msgs/msg/Position.msg;/home/student/catkin_ws/src/marta_msgs/msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/student/catkin_ws/src/marta_msgs/msg/Euler.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sss_package
)

### Generating Services

### Generating Module File
_generate_module_lisp(sss_package
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sss_package
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(sss_package_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(sss_package_generate_messages sss_package_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/student/catkin_ws/src/sss_package/msg/GeolocatedObject.msg" NAME_WE)
add_dependencies(sss_package_generate_messages_lisp _sss_package_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/student/catkin_ws/src/sss_package/msg/GeolocatedObjectList.msg" NAME_WE)
add_dependencies(sss_package_generate_messages_lisp _sss_package_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/student/catkin_ws/src/sss_package/msg/ImageMetadata.msg" NAME_WE)
add_dependencies(sss_package_generate_messages_lisp _sss_package_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(sss_package_genlisp)
add_dependencies(sss_package_genlisp sss_package_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS sss_package_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(sss_package
  "/home/student/catkin_ws/src/sss_package/msg/GeolocatedObject.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sss_package
)
_generate_msg_nodejs(sss_package
  "/home/student/catkin_ws/src/sss_package/msg/GeolocatedObjectList.msg"
  "${MSG_I_FLAGS}"
  "/home/student/catkin_ws/src/sss_package/msg/GeolocatedObject.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sss_package
)
_generate_msg_nodejs(sss_package
  "/home/student/catkin_ws/src/sss_package/msg/ImageMetadata.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/sensor_msgs/cmake/../msg/Image.msg;/home/student/catkin_ws/src/marta_msgs/msg/NavStatus.msg;/home/student/catkin_ws/src/marta_msgs/msg/Position.msg;/home/student/catkin_ws/src/marta_msgs/msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/student/catkin_ws/src/marta_msgs/msg/Euler.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sss_package
)

### Generating Services

### Generating Module File
_generate_module_nodejs(sss_package
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sss_package
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(sss_package_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(sss_package_generate_messages sss_package_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/student/catkin_ws/src/sss_package/msg/GeolocatedObject.msg" NAME_WE)
add_dependencies(sss_package_generate_messages_nodejs _sss_package_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/student/catkin_ws/src/sss_package/msg/GeolocatedObjectList.msg" NAME_WE)
add_dependencies(sss_package_generate_messages_nodejs _sss_package_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/student/catkin_ws/src/sss_package/msg/ImageMetadata.msg" NAME_WE)
add_dependencies(sss_package_generate_messages_nodejs _sss_package_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(sss_package_gennodejs)
add_dependencies(sss_package_gennodejs sss_package_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS sss_package_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(sss_package
  "/home/student/catkin_ws/src/sss_package/msg/GeolocatedObject.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sss_package
)
_generate_msg_py(sss_package
  "/home/student/catkin_ws/src/sss_package/msg/GeolocatedObjectList.msg"
  "${MSG_I_FLAGS}"
  "/home/student/catkin_ws/src/sss_package/msg/GeolocatedObject.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sss_package
)
_generate_msg_py(sss_package
  "/home/student/catkin_ws/src/sss_package/msg/ImageMetadata.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/sensor_msgs/cmake/../msg/Image.msg;/home/student/catkin_ws/src/marta_msgs/msg/NavStatus.msg;/home/student/catkin_ws/src/marta_msgs/msg/Position.msg;/home/student/catkin_ws/src/marta_msgs/msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/student/catkin_ws/src/marta_msgs/msg/Euler.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sss_package
)

### Generating Services

### Generating Module File
_generate_module_py(sss_package
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sss_package
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(sss_package_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(sss_package_generate_messages sss_package_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/student/catkin_ws/src/sss_package/msg/GeolocatedObject.msg" NAME_WE)
add_dependencies(sss_package_generate_messages_py _sss_package_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/student/catkin_ws/src/sss_package/msg/GeolocatedObjectList.msg" NAME_WE)
add_dependencies(sss_package_generate_messages_py _sss_package_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/student/catkin_ws/src/sss_package/msg/ImageMetadata.msg" NAME_WE)
add_dependencies(sss_package_generate_messages_py _sss_package_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(sss_package_genpy)
add_dependencies(sss_package_genpy sss_package_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS sss_package_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sss_package)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sss_package
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(sss_package_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(sss_package_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET marta_msgs_generate_messages_cpp)
  add_dependencies(sss_package_generate_messages_cpp marta_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sss_package)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sss_package
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(sss_package_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(sss_package_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()
if(TARGET marta_msgs_generate_messages_eus)
  add_dependencies(sss_package_generate_messages_eus marta_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sss_package)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sss_package
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(sss_package_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(sss_package_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET marta_msgs_generate_messages_lisp)
  add_dependencies(sss_package_generate_messages_lisp marta_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sss_package)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sss_package
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(sss_package_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(sss_package_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()
if(TARGET marta_msgs_generate_messages_nodejs)
  add_dependencies(sss_package_generate_messages_nodejs marta_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sss_package)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sss_package\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sss_package
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(sss_package_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(sss_package_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET marta_msgs_generate_messages_py)
  add_dependencies(sss_package_generate_messages_py marta_msgs_generate_messages_py)
endif()
