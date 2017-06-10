#include "ros/ros.h"
#include "manager.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "tf/tf.h"
#include <tf/transform_datatypes.h>
// #include "tf/StampedTransform.h"
#include "tf/LinearMath/Transform.h"
//#include "tf/Vector3.h"
#include <Eigen/Dense>
#include <sstream>
#include "cba_msgs/CBA_NavInfo.h"
#include <boost/thread/thread.hpp>


using namespace Eigen;


bool boolReceive=true;
ELMClassifier     elmclassifier_;

ELMClassifier     wow2;
CBAManager      m_Manager;
std_msgs::String GUI_msg;
MapParam        CurrentMap;
MapParam         MatlabMap;

int  CMD_GUI=0;
char sz[18];

void CmdStrCallback(const std_msgs::String::ConstPtr& msg);
void CmdIntCallback(const std_msgs::Int8::ConstPtr& msg);
void NavInfo_Callback(const cba_msgs::CBA_NavInfo::ConstPtr& msg);
void NavInfo_Callback(const cba_msgs::CBA_NavInfo::ConstPtr& msg)
{
  int width = msg->width;
  int height = msg->height;  

  std::vector<int> cell_occupancy_type(msg->cell_occupancy_type.size(),0);
  std::vector<int> action_policy_type(msg->action_policy_type.size(),0);
  std::vector<int> robot_local_cell_indices(msg->robot_local_cell_indices.size(),0);
  std::vector<int> state_type(msg->state_type.size(),0);
  std::vector<float> state_Distance(msg->state_distance.size(),0);
  std::vector<float> NearestHumanVector(2,0.0);
  std::vector<float> RobotHeadingDirection(2,0.0);

  for(int i = 0; i < msg->cell_occupancy_type.size(); i++)
    cell_occupancy_type[i]=msg->cell_occupancy_type[i];

  //manager.m_mdpsol 
  for(int i = 0; i < msg->action_policy_type.size(); i++)
    action_policy_type[i]=msg->action_policy_type[i];

  for(int i = 0; i < msg->robot_local_cell_indices.size(); i++)
    robot_local_cell_indices[i]=msg->robot_local_cell_indices[i];

  for(int i = 0; i < msg->state_type.size(); i++)
    state_type[i]=msg->state_type[i];

  for(int i = 0; i < msg->state_distance.size(); i++)
    state_Distance[i]=msg->state_distance[i];

   NearestHumanVector[0]=msg->unit_x_to_human;
   NearestHumanVector[1]=msg->unit_y_to_human;

   RobotHeadingDirection[0]=msg->unit_base_link_x;
   RobotHeadingDirection[1]=msg->unit_base_link_x;

   int robot_map_id=msg->int_robot_id;

// std::cout<<"NH x:" << NearestHumanVector[0]<< ",  y : "<<NearestHumanVector[1]<<endl;  

// std::cout<<"Heading x:" << RobotHeadingDirection[0]<< ",  y : "<<RobotHeadingDirection[1]<<endl;  

  // std::cout << "I got some message from Kinect" << std::endl;
  // std::cout << msg->cell_occupancy_type.size() << std::endl;
  //std::cout << msg->action_policy_type.size() << std::endl;

  //Save to MapParam
  CurrentMap.setWidth(width);
  CurrentMap.setHeight(height);
  CurrentMap.set_Cell_Info(cell_occupancy_type);
  CurrentMap.set_OCC_Info(action_policy_type);
  CurrentMap.set_Robot_Info(robot_local_cell_indices);
  CurrentMap.set_State_Type(state_type);
  CurrentMap.set_State_Distance(state_Distance);
  CurrentMap.set_RobotId(robot_map_id);
  CurrentMap.set_NearestHuman_V(NearestHumanVector);
  CurrentMap.set_RobotHeading_V(RobotHeadingDirection);

 //printf("I am here\n");

   

  int AutoDesiredaction=0.0;
  
  if(m_Manager.boolAuto){
      vector<float> FeatureVector = m_Manager.getFeaturevector();             //making feature vector for   
     AutoDesiredaction=m_Manager.getDirectionfromCBA(FeatureVector);         //Get command from CBA

     //confidence output
     //delivery command based on the confidence value 

     printf("Auto Mode\n");
     ros::Rate r(0.85);
    
   
     r.sleep();
 
  }
}


void Unitgoal_Callback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{ 

   m_Manager.m_unitGoal[0]=msg->data[0]- m_Manager.robot_global_x_pos;
   m_Manager.m_unitGoal[1]=msg->data[1]- m_Manager.robot_global_y_pos;

   // m_Manager.m_Goal[0]=m_Manager.Local_X_start+scaledGoalPos[0]*m_Manager.Scale_constant-1;
   // m_Manager.m_Goal[1]=m_Manager.Local_Y_start+scaledGoalPos[1]*m_Manager.Scale_constant-1;
   //ROS_INFO("msg-data x: %.3lf, y: %.3lf\n",msg->data[0],msg->data[1]);
   //ROS_INFO("unit goal x: %.3lf, y: %.3lf\n",m_Manager.m_unitGoal[0],m_Manager.m_unitGoal[1]);
   //cout<<"Goal Pose is x: "<<m_Manager.m_Goal[0]<<"y : "<<m_Manager.m_Goal[1]<<endl;

}

void mdpsol_Callback(const std_msgs::Int32MultiArray::ConstPtr& msg)
{

  //ROS_INFO("MDP solution received");
  m_Manager.m_MDPsolutionMap.resize(msg->data.size());
  for(int i(0);i<msg->data.size();i++)
    {
      m_Manager.m_MDPsolutionMap[i]=msg->data[i];
    }
}
void  hsrbodom_Callback(const nav_msgs::Odometry::ConstPtr &msg)
{

  float robot_x = msg->pose.pose.position.x;// - origin_x;
  float robot_y = msg->pose.pose.position.y;// - origin_y;  
  float robot_theta = msg->pose.pose.orientation.z;// - origin_y; //this is not exact=>sensor_imu

   m_Manager.robot_global_x_pos=robot_x;
   m_Manager.robot_global_y_pos=robot_y;
   m_Manager.robot_theta_yaw=asin(robot_theta)*2;

}


void callback1(const ros::TimerEvent&)
{
  ROS_INFO("Callback 1 triggered");
//  std::cout <<"Helloworld" << std::endl; 
}

void callback2(const ros::TimerEvent&)
{
  ROS_INFO("Callback 2 triggered");

}

int main(int argc, char **argv)
{

//////////////////
  ros::init(argc, argv, "talker");
  // ros::Rate r(5);
  m_Manager.pMapParam=&CurrentMap;
  m_Manager.pClassifier=&elmclassifier_;
  //This should be called after the 
  m_Manager.IntializeROS_publisher();
  //m_Manager.LoadDataFile();
 	// std::cout <<"Helloworld" << std::endl; 
  ros::Publisher  Matlab_Pub;
  ros::Publisher  Map_Pub;
  ros::Subscriber cmd_sub;
  ros::Subscriber cmd_sub2;
  ros::Subscriber grid_sub;   
  ros::Subscriber grid_sub2; 
  ros::Subscriber unitgoal_sub; 
  ros::Subscriber mdpsol_sub;
  ros::Subscriber Odometery_sub;
  ros::NodeHandle n;


  tf::TransformListener listener;

 //  ros::Timer timer1 = n.createTimer(ros::Duration(0.5), callback1);
 //  Publisher
   //Subscriber
  cmd_sub  = n.subscribe<std_msgs::String>("/CBA_cmd_str", 50,CmdStrCallback); 
  cmd_sub2 = n.subscribe<std_msgs::Int8>("/CBA_cmd_int", 50,CmdIntCallback);
  grid_sub = n.subscribe<cba_msgs::CBA_NavInfo>("/CBA_featureV", 20, NavInfo_Callback);  
  unitgoal_sub = n.subscribe<std_msgs::Float32MultiArray>("/CBA_unit_goal", 10, Unitgoal_Callback);
  mdpsol_sub = n.subscribe<std_msgs::Int32MultiArray>("/MDP/Solution", 10, mdpsol_Callback);
  Odometery_sub=n.subscribe<nav_msgs::Odometry>("/hsrb/odom", 10, hsrbodom_Callback);

  ros::Rate loop_rate(20);

  while (ros::ok())
  {

    //Transformation example
    // tf::StampedTransform transform;

    // //listener.lookupTransform("/base_link", "/head_rgbd_sensor_rgb_frame",ros::Time(0), transform);
    // try{
    //   listener.lookupTransform("/base_link", "/head_rgbd_sensor_rgb_frame",ros::Time(0), transform);
    // }
    // catch (tf::TransformException &ex) {
    //   ROS_ERROR("%s",ex.what());
    //   ros::Duration(1.0).sleep();
    //   continue;
    // }
    //  tf::Quaternion q = transform.getRotation();
    //  printf("Quaternion : %.3lf, %.3lf,, %.3lf, %.3lf \n ",q[0],q[1],q[2],q[3]);

    //    ROS_INFO("X pose %.3lf, y : %.3lf, z : %.3lf \n" ,  transform.getOrigin().x(),transform.getOrigin().y(),transform.getOrigin().z());
    //    // ROS_INFO("Y pose %.3lf" , transform.getOrigin().y());
    //    // ROS_INFO("Z pose %.3lf " , transform.getOrigin().z());
    //  tfScalar yaw, pitch, roll;
    //  transform.getBasis().getRPY(roll, pitch, yaw);

    //  tf::Vector3 testvec;
    //  geometry_msgs::Vector3Stamped gV, tV;
     
     // testvec.x=2.0;
     // testvec.y=0.0;
     // testvec.y=1.0;

     // float xx=transform.getOrigin().x;
    //  gV.vector.x = 2.0;
    //  gV.vector.y = 0.0;
    //  gV.vector.z = 0.0;
    //  gV.header.stamp = ros::Time();
    //  gV.header.frame_id = "/head_rgbd_sensor_rgb_frame";
    //  listener.transformVector("/base_link", gV, tV);
    //  // ROS_INFO("tg.xyz :  %.3lf, %.3lf,, %.3lf, ",tV.vector.x,tV.vector.y,tV.vector.z);

    //  //pose test
    // geometry_msgs::PoseStamped gPose,tPose;
    // gPose.pose.position.x=2.0;
    // gPose.pose.position.y=0.0;
    // gPose.pose.position.z=0.0;

    // gPose.pose.orientation.x=0.0;
    // gPose.pose.orientation.y=0.0;
    // gPose.pose.orientation.z=0.0;
    // gPose.pose.orientation.w=1.0;

    // listener.transformPose("/base_link", gPose, tPose);
    // //  tf::Matrix3x3 mat(q);
    // //  mat.getEulerYPR(&yaw, &pitch, &roll);
    //  ROS_INFO("tg.xyz :  %.3lf, %.3lf,, %.3lf, ",tPose.pose.position.x,tPose.pose.position.y,tPose.pose.position.z);
    // ROS_INFO("mat :  %.3lf, %.3lf,, %.3lf, ",mat[3],mat[4],mat[5]);
    // ROS_INFO("mat :  %.3lf, %.3lf,, %.3lf, ",mat[6],mat[7],mat[8]);
    /////////////////////////////////////



  	 ros::spinOnce();
     loop_rate.sleep();   



      // ros::spin();
   }
  // ros::spin();
  return 0;
}

void CmdStrCallback(const std_msgs::String::ConstPtr& msg)          
{
    //ROS_INFO("str msg");
    ROS_INFO("Cur cmd :%s ",(msg->data).c_str());


    return;
}

void Send2Trikey(int cmd)
{


}


void CmdIntCallback(const std_msgs::Int8::ConstPtr& msg)
{
    //ROS_INFO("int msg");
    ROS_INFO("Cur cmd :%d ",msg->data);

    int cmdfromGUI=(int)(msg->data);
   
    m_Manager.ActionfromGUICmd(cmdfromGUI);

  
        return;
}

