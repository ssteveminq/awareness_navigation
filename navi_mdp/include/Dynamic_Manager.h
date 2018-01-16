#include <vector>
#include <fstream>
#include <string>
#include <map>
#include <stdint.h>
#include "ros/ros.h"
#include <ros/package.h>
#include <eigen3/Eigen/Dense>
// #include <Eigen/Dense>
#include <MapParam.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Bool.h"
#include "srBSpline.h"
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
#include <people_msgs/PositionMeasurement.h>
#include <keyboard/Key.h>
#include <cmath>
#include <cfloat>

#define FREE_CELL 0
#define St_OBS_CELL 1
#define Dy_OBS_CELL 1
#define Human_CELL 3

#define Start_X 1
#define Start_Y 1
//80 20
#define Goal_X 2
#define Goal_Y 2

#define DYN_OFFSET_X 3.5
#define DYN_OFFSET_Y 3.5

#define Num_action 8
#define deltaMin 1E-05
#define Maxiteration 300
#define LASER_range_person  2.5
#define min_two_leg_dist    0.5
#define max_off_yolo_laser  1.0
#define YOLO_range_person   4.0


#define ra (-1.0)
#define FOVW 40       //field of view width
#define MATH_PI 3.14159265359


using namespace Eigen;
using namespace std;


class Dynamic_Manager
{
 public:
 	Dynamic_Manager(MapParam* _pMapParam);
 	Dynamic_Manager():maxiter(Maxiteration),Action_dim(8),gamma(1),Ra(ra),publishnum(0),Yolo_iter(0),m_boolSolve(false),OnceTarget(false){}
 	~Dynamic_Manager();

 	MapParam* 	pMapParam;
 	tf::TransformListener 	  listener;
	vector< std::vector<int> > Points;
	vector< vector<int> > ActionCC;
 	vector<int>		  m_Start;							//Start position of(x,y)
  	vector<int>       m_Goal;							//Goal position of (x,y)
 	vector<int>       m_Robot;					    	//Current Robot position of (x,y)
 	int 	          Feature_dim;
 	int               X_mapSize;
 	int               Y_mapSize;
 	int               Num_Grids;
 	int               State_dim;
 	int               Action_dim;
 	vector<char>  	  Policies;		// Policy (Pi)
 	vector<double>    Rewards; 		// R
 	vector<int> 	  MdpSols;		// Solution of MDP
 	vector<int>  	  PolicyNum;	// Policy (PiNum)
 	vector<double>	  Up;			// Uprime, used in updates
 	vector<double>	  U;			// Long term Utility


 	vector<int>		  m_static_obs;
 	vector<int>		  m_dynamic_obs;
 	vector<int>		  m_human_obs;
 	vector<int>		  cell_xy;
 	vector<int>       m_localoccupancy;
 	vector<int>       m_dynamic_occupancy;
 	vector<float>	  human_global;
 	int               human_callback_count;
 	int  			  num_of_detected_human_yolo;
 	vector<double>    filtered_target;


 	//human sets
 	std::vector< std::vector< double > > Cur_leg_human;
 	std::vector< std::vector< double > > Filtered_leg_human;
    std::vector< std::vector< double > > cur_yolo_people;
    std::vector< std::vector< double > > Cur_existed_human;
    std::vector< std::vector<double> > leg_targetSet;

    std::vector<int> human_occupied_idx;
	std::vector<int> human_occupied_leg_idx;
	std::vector<int> cur_coord;
	std::vector<int> Goal_Coord;
	std::vector<int> Human_Goal_Coord;
	std::vector<int> MapCoord;
	std::vector<int> visiblie_idx_set;
	std::vector<double> Head_Pos; 	
	std::vector<double> global_pose;
 	std::vector<double> Map_orig_Vector;
	std::vector<double> CurVector;
	std::vector<double> GoalVector;
	std::vector<double> HeadingVector;
    std::vector<double> viewTarget;
    

 	double Ra;
 	double gamma;
	double Prob_good;
	double Prob_bad;

 	int Local_X_start;
 	int Local_Y_start;
    int viewpub_iters;
 	int maxiter;
 	int publishnum;
 	int ReceiveData;
 	vector<int>  MDPPath;
 	vector<int>  Dyn_MDPPath;
 	srBSpline*          m_Spline;
 	srBSpline*          m_CubicSpline_x;
 	srBSpline*          m_CubicSpline_y;

 	bool 	OnceTarget;
 	bool    m_boolSolve;
 	int     dyn_path_num;
 	double	m_desired_heading;
 	int Yolo_iter;

	ros::NodeHandle  m_node;
	ros::Publisher   obsmap_Pub;
	ros::Publisher   Scaled_static_map_pub;
	ros::Publisher   Scaled_static_map_path_pub;
	ros::Publisher   Scaled_dynamic_map_pub;
	ros::Publisher   Scaled_dynamic_map_path_pub;
    ros::Publisher  viewTarget_visual_pub; 
	ros::Publisher   Path_Pub;
	//ros::Subscriber  Localmap_sub;
	ros::Publisher 	 SplinePath_pub;
	ros::Publisher 	 SplinePath_pub2;
	ros::Publisher   UnitGoalVec_pub;
	ros::Publisher   MDPSol_pub;
	ros::Publisher   RobotHeading_pub;
	ros::Publisher   Leg_boxes_pub;
	ros::Publisher   camera_map_pub;
	ros::Publisher   people_measurement_pub_;
	ros::Publisher   belief_pub;
	ros::Publisher   Human_boxes_pub;
	ros::Publisher   Gaze_point_pub;
	ros::Publisher   Gaze_activate_pub;
  
  	visualization_msgs::MarkerArray human_leg_boxes_array;
  	visualization_msgs::MarkerArray filtered_humans_array;

	//Static_mdp
	int  scaling=12;
	nav_msgs::OccupancyGrid camera_map;
	nav_msgs::OccupancyGrid Scaled_static_map;
	nav_msgs::OccupancyGrid Scaled_dynamic_map;
	nav_msgs::OccupancyGrid Scaled_static_map_path;
	nav_msgs::OccupancyGrid Scaled_dynamic_map_path;
	nav_msgs::OccupancyGrid Human_Belief_Scan_map;
	nav_msgs::Path Pre_dynamicSplinePath;
	nav_msgs::Path path;
	
	bool       booltrackHuman;
	

	//functions
 	void 			Init();								 //Initialize function
 	void 			setPMapParam(MapParam* _pMapParam);
 	void 			setStartConfig (const vector<int> Start );
 	void			setGoalConfig  (const vector<int> Goal);
 	void  			setStaticObs(const vector<int> static_obs);
 	void  			setDynamicObs(const vector<int> dynamic_obs);
 	void  			setHumanObs(const vector<int> humans);
 	void 			CellNum2Coord(const int Cell_idx, vector<int>& cell_xy);
 	int  			Coord2CellNum(vector<int> cell_xy);
 	vector<int>     Global2LocalCoord(vector<int> Global_coord);
 	bool 			getlinevalue(int line_type,double input_x, double input_y);
 	bool            MDPsolve();
 	void			updateUprimePi(int state_id);
 	void 			getMaxValueAction(int x_pos,int y_pos,map<int,double>& maxmap);
 	double 			getactionvalue(int x_pos, int y_pos, int action_ix);
 	bool			checkObs(int cur_stid,int actionNum);
 	bool			checkNoBoundary(vector<int> cur_pos);
 	bool			checkStaticObs(vector<int> cur_pos);
 	vector<int> 	getneighboractionset(int action_idx);
 	int             FindMaxIdx(vector<double> dataset);
 	char            getPolicychar(int policyidx);
 	void			printPath();
 	void 			generatePath();
 	void 			generate_dynamicPath();
 	void            pathPublish();
 	void  			updateMap(vector<int>& localmap_,vector<int>& local_start, vector<int>& local_goal);
 	void 			keyboard_callback(const keyboard::Key::ConstPtr& msg);
 	void            mdppath_callback(const nav_msgs::Path::ConstPtr & msg);
 	void 			joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg);
 	void 			static_mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
 	void 			dynamic_mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
 	void			ClikedpointCallback(const geometry_msgs::PointStamped::ConstPtr& msg);
 	void			Human_MarkerCallback(const visualization_msgs::Marker::ConstPtr& msg);
    void            filter_result_callback(const people_msgs::PositionMeasurement::ConstPtr& msg);
    void 			global_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void 			Global2MapCoord(const vector<double>& _globalcoord, vector<int>& MapCoord);
    void 			CellNum2globalCoord(const int Cell_idx, std::vector<double>& cell_xy);
 	void    		CoordinateTransform_Rviz_Grid_Start(double _x, double _y,int map_type);
	void    		CoordinateTransform_Rviz_Grid_Goal(double _x, double _y,int map_type);
	void 			CoordinateTransform_Rviz_Grid_Human(double _x, double _y,int map_type);
	void 			CoordinateTansform_Rviz_Dyn_map(double _x, double _y,vector<int>& dynamicCoord);
	int 			CoordinateTransform_Global2_beliefMap(double global_x, double global_y);
	int 			CoordinateTransform_Global2_cameraMap(float global_x, float global_y);
  	bool 			check_cameraregion(float x_pos,float y_pos);
	void            Mapcoord2GlobalCoord(const vector<int>& _Mapcoord, vector<double>& GlobalCoord);
	void 			Mapcoord2DynamicCoord(const vector<int>& _Mapcoord, vector<double>& dynamicCoord);
	double			getdistance(vector<double> cur, vector<double> goal);
	double 			getDistance_from_Vec(std::vector<double> origin, double _x, double _y);
	bool            IsinDynamicMap(float global_x, float global_y);
	bool            IsTargetMoved(float global_x, float global_y,float criterion);
	bool 			Comparetwopoistions(std::vector<double> pos,std::vector<double> pos2,double criterion);void 			setDesiredHeading(double _heading);
	void 			getCameraregion();
	bool 			NotUpdatedCameraregion(int idx);
	//Publish
	void			MDPsolPublish();
	void 			publishpaths();
	void 			publishZeropaths();	
	void 			publish_filtered_human_boxes();
  	void 			publish_cameraregion();
  	void            publish_viewpointTarget();
  	void            Publish_filter_measurment(int measurement_type);
  	void 			Publish_beliefmap();
  	void 			Publish_dynamicPath();

  	void 			put_human_occ_map_leg();
  	void 			put_human_occ_map_yolo();
  	void 			put_human_surrounding_beliefmap(int idx,double value);
  	void 			update_human_occ_belief_scan();
  	void			filterhumanbelief();
  	void 			setViewpointTarget(const std::vector<double> pos);
    
};

