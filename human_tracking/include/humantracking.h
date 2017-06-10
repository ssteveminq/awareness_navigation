#include <vector>
#include <fstream>
#include <string>
#include <map>
#include <stdint.h>
#include "ros/ros.h"
#include <ros/package.h>
#include <Eigen/Dense>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int8.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "tf/tf.h"
#include <tf/transform_datatypes.h>


#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>

#define FOVW 29				//field of view width
#define MATH_PI 3.15159265359
#define P_H 0.2 // Prior prob
#define P_S_given_H 0.8
#define P_S_given_Hc 0.5

#define P_Sc_given_H 0.01
#define P_Sc_given_Hc 0.99


class Human_Belief{

public:
	Human_Belief();
	Human_Belief(int numofhuman);
	~Human_Belief();


	ros::Publisher static_belief_map_pub;
	ros::Publisher belief_pub;
	ros::Publisher human_target_pub;
	ros::Publisher human_target_Intcmd_pub;
	
	int index;
	int m_numofhuman;
	int m_receiveiter;
	int belief_size;
	tf::TransformListener 	  listener;

	std::vector<double> Robot_Pos;				//x,y,theta
	std::vector<double> Human_target_Pos;				//x,y,theta
	std::vector< std::vector< double > > Last_detected_human;
	std::vector< std::vector< double > > Cur_detected_human;
	std::vector<int> human_occupied_idx;
	std::vector<int> visiblie_idx_set;

	std::vector<int> index_of_human_occ_cells_updated_recently;
	std::map<int, float> map_index_of_human_cells_to_prob;

	
	nav_msgs::OccupancyGrid dynamic_belief_map;
	nav_msgs::OccupancyGrid human_belief_map;
	nav_msgs::OccupancyGrid static_belief_map;
	std_msgs::Int8 track_cmd;


	void Init_parameters();
	void InitializeBelief();
	void setHumanOccupancy(int idx, double dyn_posx,double dyn_posy);
	void Updatemeasurement();
	void dyn_map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
	void number_detected_callback(const std_msgs::Int8::ConstPtr &msg);
	void update_human_occ_belief();
	void base_pose_callback(const nav_msgs::Odometry::ConstPtr& msg);
	void Human_MarkerarrayCallback(const visualization_msgs::MarkerArray::ConstPtr& msg);
	void CoordinateTransform_Global2_dynMap(double global_x, double global_y);
	int CoordinateTransform_Global2_staticMap(double global_x, double global_y);
	void put_human_occ_map();
	void Publish_beliefmap();
	void getCameraregion();
	bool getlinevalue(int line_type,double input_x, double input_y);
	bool NotUpdatedCameraregion(int idx);
	void setNearestHuman();
	double getDistance(double _x, double _y);
	void Publish_human_target();

	std::vector<double> m_dyn_occupancy;
	std::vector<double> m_prob_occupancy;
	std::vector<int>    m_yolo_idx_set;
	std::vector<double> m_human_posx;
	std::vector<double> m_human_posy;


};