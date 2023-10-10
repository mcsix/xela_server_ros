cmake_minimum_required(VERSION 3.5)
project(xela_server_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
)
################################################
## Declare ROS messages, services and actions ##
################################################
add_message_files(
  FILES
  "SensorFull.msg"
  "SensStream.msg"
  "Taxel.msg"
  "Forces.msg"
)

add_service_files(
  FILES
  "XelaSensorStream.srv"
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs

)
catkin_package(
  CATKIN_DEPENDS 
    rospy 
    std_msgs 
    geometry_msgs
    message_runtime
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

install(PROGRAMS
  scripts/xela_server
  scripts/xela_service
  launch/service.launch
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)