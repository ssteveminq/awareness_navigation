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
#include "std_msgs/Bool.h"
#include "srBSpline.h"
#include <actionlib/client/simple_action_client.h>
#include <controller_manager_msgs/ListControllers.h>
#include <controller_manager_msgs/ControllerState.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include <tf/tf.h>
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include <tf/transform_datatypes.h>
#include <cmath>
#include <cfloat>


using namespace Eigen;
using namespace std;

#define POS_ERR 0.2
// const char* kOriginLink2 = "odom";
// const char* kRobotBaseLink2 = "base_link";

 typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> Client;


class PathManager
{
 public:
 	PathManager();
 	~PathManager();

 	void global_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
 	void dyn_path_callback(const nav_msgs::Path::ConstPtr& msg);
 	void setPath(const nav_msgs::Path::ConstPtr& msg);
 	control_msgs::FollowJointTrajectoryGoal getsubgoal();
 	control_msgs::FollowJointTrajectoryGoal getsubgoal(std::vector<double> desired_target);
 	void sendPathAction();
 	void setNavTarget();
 	int setFinalNavTarget();
 	void setDynNavTarget();
 	int findpathidx();
 	int finddynpathidx();
 	double getdistance(const vector<double> cur, const vector<double> goal, int dimension);
 	void setViewpointTarget(const std::vector<double> pos);
 	void publishViewTpointTarget();
 	void discreteCtrltoPos(std::vector<double> pos);
 	int getIndexOfLargestElement(vector<double> arr);
 	int getdirection(std::vector<double> pos);


 	ros::NodeHandle m_node;
 	ros::Publisher setNavTarget_pub;
 	ros::Publisher viewTarget_visual_pub;
 	ros::Publisher Gaze_point_pub;
	ros::Publisher Gaze_activate_pub;
	ros::Publisher nav_cmd_pub;
    ros::Publisher head_cmd_pub;
 	tf::TransformListener     listener;
 	tf2_ros::Buffer tf_buffer_;
 	// _Client trj_cli;

 	std::vector< std::vector<double> > base_trajectory;
	std::vector< std::vector<double> > Recived_path;
	std::vector< std::vector<double> > dyn_Recived_path;
	std::vector<double> x_;
	std::vector<double> y_;

	std::vector<double> dyn_x_;
	std::vector<double> dyn_y_;

	std::vector<double> hsrb_base_pose;
	std::vector<double> global_pose;
	std::vector<double> viewTarget;
	std::vector<double> finalTarget;
	std::vector<double> dyn_finalTarget;
	std::vector<double> lastSubTarget;
	std::vector<double> dyn_lastSubTarget;

	int pathSize;
	int pre_idx;
	int cur_idx;
	int dyn_pre_idx;
	int dyn_cur_idx;
	int viewpub_iters;
	int half_pathsize;

	bool discrete_mode;
	bool scan_mode;
	int headiter;
	bool boolreceive;


};
