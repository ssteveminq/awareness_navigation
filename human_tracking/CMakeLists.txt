cmake_minimum_required(VERSION 2.8.3)
project(human_tracking)	

SET(SOURCES src/humantracking.cpp src/human_scan.cpp)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  sensor_msgs
  message_generation
  std_msgs
  std_srvs
  tf
  pcl_ros
  pcl_conversions
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

add_service_files(
   FILES
   peoplefinder.srv
)

generate_messages(
   DEPENDENCIES
   geometry_msgs
   std_msgs
   std_srvs
)


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
    std_msgs
    message_runtime
    tf
 )

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_executable(human_tracker src/tracking_test.cpp ${SOURCES})
target_link_libraries(human_tracker ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_executable(scanner src/scanning_test.cpp ${SOURCES})
target_link_libraries(scanner ${catkin_LIBRARIES} ${PCL_LIBRARIES})
