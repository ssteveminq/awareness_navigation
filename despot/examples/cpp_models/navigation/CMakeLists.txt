cmake_minimum_required(VERSION 2.8.3)
project(despot)	

SET(SOURCES src/navigation.cpp src/main.cpp)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  sensor_msgs
  std_msgs
)


find_package(Eigen3 QUIET)
if(NOT EIGEN3_FOUND)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(EIGEN3 REQUIRED eigen3)
    set(EIGEN3_INCLUDE_DIR ${EIGEN3_INCLUDE_DIRS})
endif()


#find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(include)
include_directories(src)


catkin_package(
  INCLUDE_DIRS ../../../include
#  LIBRARIES classifier
#  CATKIN_DEPENDS libsocket roscpp geometry_msgs
  DEPENDS
  	roscpp
  	geometry_msgs
  	sensor_msgs
 )

include_directories(
  ${catkin_INCLUDE_DIRS}
)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_executable("${PROJECT_NAME}_navigation"
  src/navigation.cpp
  src/main.cpp
)
target_link_libraries("${PROJECT_NAME}_navigation"
  "${PROJECT_NAME}"
)
target_link_libraries("${PROJECT_NAME}_navigation"
  "${catkin_LIBRARIES}"
)

install(TARGETS "${PROJECT_NAME}_navigation"
  RUNTIME DESTINATION "${BINARY_INSTALL_PATH}"
)
