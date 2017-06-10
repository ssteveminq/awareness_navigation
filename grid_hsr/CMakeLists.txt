cmake_minimum_required(VERSION 2.8.3)
project(grid_hsr)	

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  sensor_msgs
  cba_msgs
  message_generation
  classifier_hsr  
  nav_msgs  
  rosbag
  roslib
  tf
)



find_package(PCL 1.7 REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(include)
include_directories(
    include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)


#add_message_files(FILES your_msg_file.msg)


catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES grid
#  CATKIN_DEPENDS roscpp geometry_msgs
  DEPENDS
  	roscpp
  	geometry_msgs
  	sensor_msgs
	  message_runtime
    nav_msgs
    cba_msgs
    rosbag	
    roslib    
    tf
)


add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS} /usr/local/lib)





add_executable(grid_node_hsr src/grid_node.cpp)
add_executable(odom_grid_node_hsr src/odom_grid_node.cpp)
add_executable(feature_grid src/feature_grid.cpp)
add_executable(proj_map_reader_hsr src/proj_map_reader.cpp)
add_executable(proj_map_saver_hsr src/proj_map_saver.cpp)
add_executable(human_box_node src/human_boxes.cpp)

target_link_libraries(grid_node_hsr ${catkin_LIBRARIES})
target_link_libraries(odom_grid_node_hsr ${catkin_LIBRARIES})
target_link_libraries(proj_map_reader_hsr ${catkin_LIBRARIES})
target_link_libraries(proj_map_saver_hsr ${catkin_LIBRARIES})
target_link_libraries(feature_grid ${catkin_LIBRARIES})
target_link_libraries(human_box_node ${catkin_LIBRARIES}  ${PCL_LIBRARIES})

