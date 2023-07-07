cmake_minimum_required(VERSION 2.8.3)
project(dvnets_franka_msgs)

find_package(catkin REQUIRED COMPONENTS
    rospy
    sensor_msgs
    genmsg
    actionlib
    actionlib_msgs
    geometry_msgs
)

add_service_files(
  FILES
  Transporter.srv
  GotoPose.srv
  SetGripper.srv
  Home.srv
)

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   actionlib_msgs
   geometry_msgs
   sensor_msgs
 )

catkin_package(
  CATKIN_DEPENDS
    rospy
    sensor_msgs
    actionlib_msgs
    geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
