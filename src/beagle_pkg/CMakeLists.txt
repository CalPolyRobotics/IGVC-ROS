cmake_minimum_required(VERSION 2.8.3)
project(beagle_pkg)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  liblms1xx
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   status.msg
   imu_msg.msg
)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   dem.srv
   ard.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES beagle_pkg
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs liblms1xx
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(3dmgx2
   src/3dmgx2.cc
)

## Declare a cpp executable
# add_executable(beagle_pkg_node src/beagle_pkg_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(beagle_pkg_node beagle_pkg_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(beagle_pkg_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS beagle_pkg beagle_pkg_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_beagle_pkg.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

add_executable(demo_client src/demo_client.cpp)
target_link_libraries(demo_client ${catkin_LIBRARIES})
add_dependencies(demo_client beagle_pkg_gencpp)

add_executable(master_node src/master_node.cpp src/master_node.cpp src/rs232.c src/SerialComms.cpp   src/CaddyBoardControl.cpp)
target_link_libraries(master_node ${catkin_LIBRARIES})
add_dependencies(master_node beagle_pkg_gencpp)

add_executable(motor_node src/motor_node.cpp)
target_link_libraries(motor_node ${catkin_LIBRARIES})
add_dependencies(motor_node beagle_pkg_gencpp)

add_executable(ard_server src/ard_server.cpp 
src/SerialComms.cpp
src/rs232.c

)
target_link_libraries(ard_server ${catkin_LIBRARIES})
add_dependencies(ard_server beagle_pkg_gencpp)

add_executable(lidar_pub src/lidar_pub.cpp)
target_link_libraries(lidar_pub ${catkin_LIBRARIES})
add_dependencies(lidar_pub beagle_pkg_gencpp)

add_executable(Communication_node src/Communication_node.cpp)
target_link_libraries(Communication_node ${catkin_LIBRARIES})
add_dependencies(Communication_node beagle_pkg_gencpp)

add_executable(LMS1xx_node src/LMS1xx_node.cpp)
target_link_libraries(LMS1xx_node ${catkin_LIBRARIES})
add_dependencies(LMS1xx_node beagle_pkg_gencpp)

add_executable(gps_node src/gps_pub.cpp)
target_link_libraries(gps_node ${catkin_LIBRARIES})
add_dependencies(gps_node beagle_pkg_gencpp)

add_executable(imu_publisher src/imu_publisher.cpp)
target_link_libraries(imu_publisher ${catkin_LIBRARIES} 3dmgx2)
add_dependencies(imu_publisher beagle_pkg_gencpp)
