#include "ros/ros.h"
#include "humantracking.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseArray.h"
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include <sstream>
#include <boost/thread/thread.hpp>


using namespace Eigen;

int num_x=12;
int num_y=12;

//variables & functions for service
bool g_caught_sigint=false;

void sig_handler(int sig)
{
  g_caught_sigint = true;
  ROS_INFO("caught sigint, init shutdown sequence...");
  ros::shutdown();
  exit(1);
};



int main(int argc, char **argv)
{
  ros::init(argc, argv, "Human_tracker");

  Human_Belief Human_tracker;


  
  // MDPManager problemmanager = humantracking(&mapParam);
  // ros::Rate r(5);
  
  // ros::Publisher  Human_p
  ros::Subscriber Human_detected_sub;  
  ros::Subscriber dynlocalmap_sub;
  ros::Subscriber Basepos_sub;
  ros::Subscriber Human_markerarray_sub;
  ros::Subscriber global_pos_sub;
  ros::Subscriber edge_leg_sub;
  ros::Subscriber jointstates_sub;
  ros::Subscriber laser_pcl_sub;
  ros::Subscriber laser_scan_sub;

  ros::NodeHandle n;
  Human_tracker.human_target_Intcmd_pub=n.advertise<std_msgs::Int8>("/Int_cmd_trackhuman", 10, true);
  Human_tracker.belief_pub=n.advertise<nav_msgs::OccupancyGrid>("/human_belief_map", 10, true);
  Human_tracker.human_target_pub= n.advertise<visualization_msgs::Marker>("/human_target", 10, true);
  Human_tracker.static_belief_map_pub=n.advertise<nav_msgs::OccupancyGrid>("/static_belief_map", 10, true);
  Human_tracker.Headscan_pub=n.advertise<std_msgs::Int8>("/gui_movebase_cmd", 10, true);
  Human_tracker.setNavTarget_pub=n.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/move/goal",50,true);
  Human_tracker.Human_boxes_pub= n.advertise<visualization_msgs::MarkerArray>("/human_leg_boxes", 50, true);
  Human_tracker.human_leg_target_pub=n.advertise<visualization_msgs::Marker>("/human_leg_target", 10, true);
  Human_tracker.Gaze_point_pub= n.advertise<geometry_msgs::Point>("/gazed_point_fixing_node/target_point", 50, true);
  Human_tracker.Gaze_activate_pub= n.advertise<std_msgs::Bool>("/gazed_point_fixing_node/activate", 50, true);
  Human_tracker.human_laser_pub= n.advertise<sensor_msgs::PointCloud2>("/modified_scan_cloud", 50, true);
  Human_tracker.human_laser_scan_pub=n.advertise<sensor_msgs::LaserScan>("/modifiedlaserscan", 50, true);

  dynlocalmap_sub =n.subscribe<nav_msgs::OccupancyGrid>("/scaled_dynamic_map", 30, &Human_Belief::dyn_map_callback,&Human_tracker); 
  Basepos_sub   = n.subscribe<nav_msgs::Odometry>("/hsrb/odom", 10, &Human_Belief::base_pose_callback,&Human_tracker);
  global_pos_sub= n.subscribe<geometry_msgs::PoseStamped>("/global_pose", 10, &Human_Belief::global_pose_callback,&Human_tracker);
  edge_leg_sub=n.subscribe<geometry_msgs::PoseArray>("/edge_leg_detector", 10, &Human_Belief::edge_leg_callback,&Human_tracker);
  jointstates_sub =n.subscribe<sensor_msgs::JointState>("/hsrb/joint_states", 10, &Human_Belief::joint_states_callback,&Human_tracker);
  // laser_pcl_sub =n.subscribe<sensor_msgs::PointCloud2>("/scan_cloud", 10, &Human_Belief::laser_pcl_callback,&Human_tracker);
  // laser_scan_sub=n.subscribe<sensor_msgs::LaserScan>("/hsrb/base_scan", 10, &Human_Belief::laser_scan_callback,&Human_tracker);
  Human_markerarray_sub = n.subscribe<visualization_msgs::MarkerArray>("/human_boxes", 10, &Human_Belief::Human_MarkerarrayCallback,&Human_tracker);
  ros::Timer timer = n.createTimer(ros::Duration(5.0), &Human_Belief::scanforhuman, &Human_tracker);

  ros::ServiceServer service = n.advertiseService("/human_finder",  &Human_Belief::FindHuman,&Human_tracker);

  ros::Rate loop_rate(50);

  //Human_tracker.Publish_nav_target();

  while (ros::ok())
  {
	   
    if(Human_tracker.scanmode)
    {
       // Human_tracker.scanforhuman();
       // Human_tracker.Publish_beliefmap();
    }
    else
    {


    }
       // Human_tracker.scanforhuman();
      Human_tracker.Publish_beliefmap();
     // Human_tracker.Publish_beliefmap();
     // Human_tracker.UpdateTarget();
     // Human_tracker.Publish_human_target();
     // Human_tracker.Publish_human_boxes();
     //Human_tracker.Publish_nav_target();
     ros::spinOnce();
     loop_rate.sleep();  
  }

  ros::spin();

  return 0;
}




