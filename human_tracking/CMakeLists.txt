cmake_minimum_required(VERSION 2.8.3)
project(human_tracking)	

SET(SOURCES src/humantracking.cpp)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  sensor_msgs
  message_generation
  std_msgs
  tf
)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(include)
include_directories(src)


catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES humantracking
#  CATKIN_DEPENDS roscpp geometry_msgs
  DEPENDS
  	roscpp
  	geometry_msgs
  	sensor_msgs
  	message_runtime
    tf
 )

include_directories(
  ${catkin_INCLUDE_DIRS}
)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_executable(human_tracker src/tracking_test.cpp ${SOURCES})
target_link_libraries(human_tracker ${catkin_LIBRARIES})
