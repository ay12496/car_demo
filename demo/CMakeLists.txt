cmake_minimum_required(VERSION 2.8.3)
project(demo)

set (PRIUSCUP_MAJOR_VERSION 0)
set (PRIUSCUP_MINOR_VERSION 1)
set (PRIUSCUP_PATCH_VERSION 0)
set (PRIUSCUP_VERSION_FULL ${PRIUSCUP_MAJOR_VERSION}.${PRIUSCUP_MINOR_VERSION}.${PRIUSCUP_PATCH_VERSION})

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  ackermann_model
)

find_package(gazebo 9 REQUIRED)
find_package(ignition-msgs1 REQUIRED)

catkin_package(
 # INCLUDE_DIRS include
 # LIBRARIES gazebo_radar_plugin
 CATKIN_DEPENDS gazebo_ros ackermann_model
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${SDFormat_INCLUDE_DIRS})

install(DIRECTORY models DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY worlds DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})