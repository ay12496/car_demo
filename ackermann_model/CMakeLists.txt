cmake_minimum_required(VERSION 2.8.3)
project(ackermann_model)

set (ACKERMANNMODEL_MAJOR_VERSION 0)
set (ACKERMANNMODEL_MINOR_VERSION 1)
set (ACKERMANNMODEL_PATCH_VERSION 0)
set (ACKERMANNMODEL_VERSION_FULL ${ACKERMANNMODEL_MAJOR_VERSION}.${ACKERMANNMODEL_MINOR_VERSION}.${ACKERMANNMODEL_PATCH_VERSION})

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  message_generation
  std_msgs
  geometry_msgs
)

find_package(gazebo 9 REQUIRED)
find_package(ignition-msgs1 REQUIRED)

add_message_files(
	FILES
  Control.msg
	Status.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
 # INCLUDE_DIRS include
 # LIBRARIES gazebo_radar_plugin
 CATKIN_DEPENDS gazebo_ros message_runtime std_msgs geometry_msgs
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${SDFormat_INCLUDE_DIRS})

add_library(AckermannModelPlugin SHARED src/AckermannModelPlugin.cc)
target_link_libraries(AckermannModelPlugin ${GAZEBO_LIBRARIES} ${IGNITION-MSGS_LIBRARIES})
add_dependencies(AckermannModelPlugin ${catkin_EXPORTED_TARGETS})


install(TARGETS AckermannModelPlugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
