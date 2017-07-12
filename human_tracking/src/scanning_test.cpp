#include "ros/ros.h"
#include "human_scan.h"
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
  ros::init(argc, argv, "Human_scanner");

  Human_Belief_Scan Human_tracker;
  
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
  ros::Subscriber human_num_sub;
  ros::Subscriber face_detected_sub;
  ros::Subscriber keyboard_sub;

  ros::NodeHandle n;
  Human_tracker.setNavTarget_pub=n.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/move/goal",50,true);
  Human_tracker.head_cmd_pub=n.advertise<std_msgs::Int8>("/head_action_cmd", 10, true);
  Human_tracker.belief_pub=n.advertise<nav_msgs::OccupancyGrid>("/Human_Belief_Scan_map", 10, true);
  Human_tracker.human_target_pub= n.advertise<visualization_msgs::Marker>("/human_target", 10, true);
  Human_tracker.static_belief_map_pub=n.advertise<nav_msgs::OccupancyGrid>("/static_belief_map", 10, true);
  Human_tracker.Human_boxes_pub= n.advertise<visualization_msgs::MarkerArray>("/human_leg_boxes", 50, true);
  Human_tracker.founded_human_pub= n.advertise<visualization_msgs::MarkerArray>("/founded_human", 50, true);
  Human_tracker.human_leg_target_pub=n.advertise<visualization_msgs::Marker>("/human_leg_target", 10, true);
  Human_tracker.Gaze_point_pub= n.advertise<geometry_msgs::Point>("/gazed_point_fixing_node/target_point", 50, true);
  Human_tracker.Gaze_activate_pub= n.advertise<std_msgs::Bool>("/gazed_point_fixing_node/activate", 50, true);
  Human_tracker.filter_act_pub=n.advertise<std_msgs::Int8>("/filter_act_cmd", 10, true);


  dynlocalmap_sub =n.subscribe<nav_msgs::OccupancyGrid>("/scaled_dynamic_map", 30, &Human_Belief_Scan::dyn_map_callback,&Human_tracker); 
  global_pos_sub= n.subscribe<geometry_msgs::PoseStamped>("/global_pose", 10, &Human_Belief_Scan::global_pose_callback,&Human_tracker);
  edge_leg_sub=n.subscribe<geometry_msgs::PoseArray>("/edge_leg_detector", 10, &Human_Belief_Scan::edge_leg_callback,&Human_tracker);
  jointstates_sub =n.subscribe<sensor_msgs::JointState>("/hsrb/joint_states", 10, &Human_Belief_Scan::joint_states_callback,&Human_tracker);
  Human_markerarray_sub = n.subscribe<visualization_msgs::MarkerArray>("/human_boxes", 10, &Human_Belief_Scan::Human_MarkerarrayCallback,&Human_tracker);
  human_num_sub=n.subscribe<std_msgs::Int8>("detection/number_of_detected_dobjects",10, &Human_Belief_Scan::number_detected_callback,&Human_tracker);
  face_detected_sub=n.subscribe<std_msgs::String>("/face_detected_name",10, &Human_Belief_Scan::face_detected_name_callback,&Human_tracker);
  keyboard_sub=n.subscribe<keyboard::Key>("/keyboard/keydown",10, &Human_Belief_Scan::keyboard_callback,&Human_tracker);

  Human_tracker.my_timer = n.createTimer(ros::Duration(15.0), &Human_Belief_Scan::scanforhuman, &Human_tracker);
  Human_tracker.face_detect_timer = n.createTimer(ros::Duration(7.5), &Human_Belief_Scan::facedetect_target, &Human_tracker);
  ros::ServiceServer service = n.advertiseService("/human_finder",  &Human_Belief_Scan::FindHuman,&Human_tracker);

  ros::Rate loop_rate(25);
  
  while (ros::ok())
  {
	   
    if(Human_tracker.action_mode==0)
    {
       // Human_tracker.scanforhuman();
       Human_tracker.Publish_beliefmap();     
     
    }
    else if(Human_tracker.action_mode==1)   //wait for setting target
    {
       Human_tracker.Publish_beliefmap();     
      //do face detection
      //find nearest person
      // Human_tracker.SetTarget();
      // Human_tracker.Publish_beliefmap();
    }
    else if(Human_tracker.action_mode==2)   //navigation mode
    {
      //If we set the target
      Human_tracker.Publish_human_target();
      Human_tracker.Publish_nav_target();

    }
    else
    {


    }
    
     // Human_tracker.Publish_beliefmap();     
     Human_tracker.Publish_human_boxes();
     ros::spinOnce();
     loop_rate.sleep();  
  }

  ros::spin();

  return 0;
}




