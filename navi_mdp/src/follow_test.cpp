#include "ros/ros.h"
#include "PathManager.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/JointState.h"
#include <actionlib/client/simple_action_client.h>
#include <controller_manager_msgs/ListControllers.h>
#include <controller_manager_msgs/ControllerState.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <villa_navi_service/GoTargetPos.h>
#include <sstream>
#include <boost/thread/thread.hpp>
#include "srBSpline.h"

using namespace Eigen;


int main(int argc, char **argv)
{
  ros::init(argc, argv, "Path_follower");
  
  PathManager path_manager; 
  
   // ros::Rate r(5);
  ros::Subscriber Point_sub;                        //subscriber clicked point
  ros::Subscriber dynamicmap_sub;
  ros::Subscriber path_sub;
  ros::Subscriber human_marker_sub;
  ros::Subscriber global_pos_sub;
  ros::Subscriber jointstates_sub;
  ros::Subscriber pomdp_cmd_sub;
  // ros::NodeHandle n;

  Client trj_cli("/hsrb/omni_base_controller/follow_joint_trajectory", true);
  trj_cli.waitForServer();
  
  // Point_sub        = n.subscribe<geometry_msgs::PointStamped>("/clicked_point", 10, &PathManager::ClikedpointCallback,&path_manger);
  // human_cmd_sub    = n.subscribe<std_msgs::Int8>("/Int_cmd_trackhuman", 50, &PathManager::Human_target_cmdCallback,&path_manger);
  // human_marker_sub = n.subscribe<visualization_msgs::Marker>("/human_target", 50, &PathManager::Human_MarkerCallback,&path_manger);
    // jointstates_sub  = n.subscribe<sensor_msgs::JointState>("/hsrb/joint_states", 10, &PathManager::joint_states_callback,&path_manger);
  pomdp_cmd_sub   = path_manager.m_node.subscribe<std_msgs::Int8>("/pomdp_cmd", 10, &PathManager::pomdp_cmd_callback,&path_manager);
  global_pos_sub   = path_manager.m_node.subscribe<geometry_msgs::PoseStamped>("/global_pose", 10, &PathManager::global_pose_callback,&path_manager);
  path_sub = path_manager.m_node.subscribe<nav_msgs::Path>("mdp_path_dynamic",10,&PathManager::dyn_path_callback,&path_manager);
  // path_sub = path_manager.m_node.subscribe<nav_msgs::Path>("mdp_path_dynamic",10,&PathManager::dyn_path_callback,&path_manager);
  //service client
  // ros::ServiceClient client = 

  ////////////////
  // nav_msgs::Path::ConstPtr msg = ros::topic::waitForMessage<nav_msgs::Path>("/mdp_path_dynamic");
  nav_msgs::Path::ConstPtr msg = ros::topic::waitForMessage<nav_msgs::Path>("/mdp_path");
  path_manager.setPath(msg);
  ros::Rate loop_rate(30);
  control_msgs::FollowJointTrajectoryGoal subgoal;
  int whileiter=0;

  ///////////////////
  while (ros::ok())
  {
     
  	 ros::spinOnce();
     loop_rate.sleep();

    //  if(whileiter%80==0){
      
    //   if(path_manager.discrete_mode)
    //   {
    //       // printf("discrete mode \n");
    //       // path_manager.discreteCtrltoPos(path_manager.finalTarget);

    //       path_manager.setDynNavTarget();

    //   }
    //   else if(path_manager.scan_mode)
    //   {
    //     std_msgs::Int8 head_msg;
        
    //     if(path_manager.headiter<5)
    //     {
    //       head_msg.data=1;
    //       path_manager.head_cmd_pub.publish(head_msg);
    //     }
    //     else if(path_manager.headiter<10){
    //       head_msg.data=3; 
    //       path_manager.head_cmd_pub.publish(head_msg); 
    //     }
    //     else if(path_manager.headiter<15){
    //       head_msg.data=2;  
    //       path_manager.head_cmd_pub.publish(head_msg);
    //       path_manager.scan_mode=false;
    //       path_manager.discrete_mode=true;
    //     }
    //     else{
    //       head_msg.data=0; 
    //       path_manager.head_cmd_pub.publish(head_msg);

    //     }

    //     path_manager.headiter++;

    //   }
    //   else 
    //   {

    //     path_manager.setNavTarget();

    //   }
      
    //   }
    // whileiter++;

  }

  ros::spin();

  return 0;
}




