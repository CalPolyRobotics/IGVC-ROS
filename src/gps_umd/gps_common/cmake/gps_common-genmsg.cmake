# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "gps_common: 2 messages, 0 services")

set(MSG_I_FLAGS "-Igps_common:/home/michaelroberts/catkin_ws/src/gps_umd/gps_common/msg;-Istd_msgs:/opt/ros/hydro/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(gps_common_generate_messages ALL)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(gps_common
  "/home/michaelroberts/catkin_ws/src/gps_umd/gps_common/msg/GPSStatus.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/gps_common
)
_generate_msg_cpp(gps_common
  "/home/michaelroberts/catkin_ws/src/gps_umd/gps_common/msg/GPSFix.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/michaelroberts/catkin_ws/src/gps_umd/gps_common/msg/GPSStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/gps_common
)

### Generating Services

### Generating Module File
_generate_module_cpp(gps_common
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/gps_common
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(gps_common_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(gps_common_generate_messages gps_common_generate_messages_cpp)

# target for backward compatibility
add_custom_target(gps_common_gencpp)
add_dependencies(gps_common_gencpp gps_common_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS gps_common_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(gps_common
  "/home/michaelroberts/catkin_ws/src/gps_umd/gps_common/msg/GPSStatus.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/gps_common
)
_generate_msg_lisp(gps_common
  "/home/michaelroberts/catkin_ws/src/gps_umd/gps_common/msg/GPSFix.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/michaelroberts/catkin_ws/src/gps_umd/gps_common/msg/GPSStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/gps_common
)

### Generating Services

### Generating Module File
_generate_module_lisp(gps_common
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/gps_common
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(gps_common_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(gps_common_generate_messages gps_common_generate_messages_lisp)

# target for backward compatibility
add_custom_target(gps_common_genlisp)
add_dependencies(gps_common_genlisp gps_common_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS gps_common_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(gps_common
  "/home/michaelroberts/catkin_ws/src/gps_umd/gps_common/msg/GPSStatus.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gps_common
)
_generate_msg_py(gps_common
  "/home/michaelroberts/catkin_ws/src/gps_umd/gps_common/msg/GPSFix.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/michaelroberts/catkin_ws/src/gps_umd/gps_common/msg/GPSStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gps_common
)

### Generating Services

### Generating Module File
_generate_module_py(gps_common
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gps_common
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(gps_common_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(gps_common_generate_messages gps_common_generate_messages_py)

# target for backward compatibility
add_custom_target(gps_common_genpy)
add_dependencies(gps_common_genpy gps_common_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS gps_common_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/gps_common)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/gps_common
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(gps_common_generate_messages_cpp std_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/gps_common)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/gps_common
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(gps_common_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gps_common)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gps_common\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gps_common
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(gps_common_generate_messages_py std_msgs_generate_messages_py)
