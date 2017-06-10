cmake_minimum_required(VERSION 2.8.3)
project(classifier_hsr)	

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  sensor_msgs
  message_generation
  std_msgs
  actionlib
  trajectory_msgs
  control_msgs
  cba_msgs
  tf
)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(include)
include_directories(src)


#add_message_files(FILES CBA_NavInfo.msg)
#add_service_files(FILES CBA_Navinoservice.srv)



#generate_messages(
#   DEPENDENCIES
#   std_msgs
#)


catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES classifier
#  CATKIN_DEPENDS roscpp geometry_msgs
  DEPENDS
  	roscpp
  	geometry_msgs
  	sensor_msgs
    cba_msgs
  	message_runtime
    tf
 )





include_directories(
  ${catkin_INCLUDE_DIRS}
)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

install(DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

