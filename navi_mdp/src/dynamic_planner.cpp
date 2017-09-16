#include "ros/ros.h"
#include "Dynamic_Manager.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/JointState.h"
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Dense>
#include <sstream>
#include <boost/thread/thread.hpp>
#include "srBSpline.h"

using namespace Eigen;

bool boolSolve=false;
MapParam   dynamicmapParam(10,10,0.75);
static int  Receive_count=0;

void CmdIntCallback(const std_msgs::Int8::ConstPtr& msg)
{


}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dynamic_planner");
  
  Dynamic_Manager dynamicManager; 
  //problemmanager.setRosObj(&nodeObj);
  dynamicManager.setPMapParam(&dynamicmapParam);

  // ros::Rate r(5);
  ros::Subscriber Point_sub;          //subscriber clicked point
  ros::Subscriber dynamicmap_sub;
  ros::Subscriber Basepos_sub;
  ros::Subscriber human_marker_sub;
  ros::Subscriber human_cmd_sub;
  ros::Subscriber global_pos_sub;
  ros::Subscriber human_yolo_sub;
  ros::Subscriber human_leg_sub;
  ros::Subscriber jointstates_sub;
  ros::Subscriber edge_leg_sub;
  ros::Subscriber filtered_result_sub;
  ros::NodeHandle n;
  
  Point_sub        = n.subscribe<geometry_msgs::PointStamped>("/clicked_point", 10, &Dynamic_Manager::ClikedpointCallback,&dynamicManager);
  human_cmd_sub    = n.subscribe<std_msgs::Int8>("/Int_cmd_trackhuman", 50, &Dynamic_Manager::Human_target_cmdCallback,&dynamicManager);
  human_marker_sub = n.subscribe<visualization_msgs::Marker>("/human_target", 50, &Dynamic_Manager::Human_MarkerCallback,&dynamicManager);
  human_yolo_sub   = n.subscribe<visualization_msgs::MarkerArray>("/human_boxes", 10, &Dynamic_Manager::Human_Yolo_Callback,&dynamicManager);
  human_leg_sub    = n.subscribe<geometry_msgs::PoseArray>("/edge_leg_detector", 20, &Dynamic_Manager::human_leg_callback,&dynamicManager);
  dynamicmap_sub   = n.subscribe<nav_msgs::OccupancyGrid>("/dynamic_obstacle_map_ref", 30, &Dynamic_Manager::dynamic_mapCallback,&dynamicManager); 
  jointstates_sub  = n.subscribe<sensor_msgs::JointState>("/hsrb/joint_states", 10, &Dynamic_Manager::joint_states_callback,&dynamicManager);
  global_pos_sub   = n.subscribe<geometry_msgs::PoseStamped>("/global_pose", 10, &Dynamic_Manager::global_pose_callback,&dynamicManager);
  filtered_result_sub = n.subscribe<people_msgs::PositionMeasurement>("people_tracker_filter", 10,&Dynamic_Manager::filter_result_callback,&dynamicManager);  
  ros::Rate loop_rate(30);
  while (ros::ok())
  {
      //problemmanager.MDPsolPublish();
       if(dynamicManager.m_boolSolve)
       {
          ROS_INFO("Start to dynamic mdp");
          dynamicManager.MDPsolve(); 
          dynamicManager.generate_dynamicPath();
          dynamicManager.m_boolSolve=false;
          // dynamicManager.booltrackHuman=false;
          dynamicManager.dyn_path_num++;
       }
       else
       {
         if(dynamicManager.dyn_path_num>0)
           dynamicManager.publishpaths();
       }

     dynamicManager.publish_cameraregion();
     dynamicManager.publish_leg_boxes();
     dynamicManager.Publish_beliefmap();
     dynamicManager.publish_filtered_human_boxes();
     dynamicManager.Publish_filter_measurment(0);

  	 ros::spinOnce();
     loop_rate.sleep();  
  }

  ros::spin();

  return 0;
}




