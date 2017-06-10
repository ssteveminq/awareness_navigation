#include "ros/ros.h"
#include "humantracking.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PointStamped.h"
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include <sstream>
#include <boost/thread/thread.hpp>

using namespace Eigen;

int num_x=12;
int num_y=12;


// void local_mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
// {
//    //ROS_INFO("int msg");
//     localoccupancy.resize(msg->data.size());

//    for(int i(0);i<msg->data.size();i++){
//         localoccupancy[i]=(msg->data)[i];
//    }

// }



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

  ros::NodeHandle n;
  Human_tracker.human_target_Intcmd_pub=n.advertise<std_msgs::Int8>("/Int_cmd_trackhuman", 10, true);
  Human_tracker.belief_pub=n.advertise<nav_msgs::OccupancyGrid>("/human_belief_map", 10, true);
  Human_tracker.human_target_pub= n.advertise<visualization_msgs::Marker>("/human_target", 50, true);
  Human_tracker.static_belief_map_pub=n.advertise<nav_msgs::OccupancyGrid>("/static_belief_map", 10, true);

  dynlocalmap_sub =n.subscribe<nav_msgs::OccupancyGrid>("/scaled_dynamic_map", 30, &Human_Belief::dyn_map_callback,&Human_tracker); 
  Basepos_sub   = n.subscribe<nav_msgs::Odometry>("/hsrb/odom", 10, &Human_Belief::base_pose_callback,&Human_tracker);
  Human_markerarray_sub = n.subscribe<visualization_msgs::MarkerArray>("/human_boxes", 50, &Human_Belief::Human_MarkerarrayCallback,&Human_tracker);

  ros::Rate loop_rate(50);

  while (ros::ok())
  {
	   
     Human_tracker.Publish_beliefmap();
     Human_tracker.Publish_human_target();
  	 ros::spinOnce();
     loop_rate.sleep();  
  }

  ros::spin();

  return 0;
}




