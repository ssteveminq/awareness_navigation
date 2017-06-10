#include "ros/ros.h"

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/JointState.h>
#include "cba_msgs/CBA_NavInfo.h"
#include <ros/package.h>
#include "rosbag/bag.h"
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/GridCells.h"
#include "nav_msgs/Odometry.h"
#include <std_msgs/Int8.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include "tf/transform_listener.h"
#include "tf/tf.h"
#include <tf/transform_datatypes.h>

#define TRIKEY_SIDE_WIDTH 0.6
#define CELL_FEATURE_SIZE 0.7

#define CELL_RESOLUTION 0.5
#define FEATURE_CELL_WIDTH 3 // DO NOT CHANGE
#define FEATURE_CELL_HEIGHT 3 // DO NOT CHANGE

#define FREE_CELL 0
#define OBSTACLE_OCCUPIED 1
#define ROBOT_OCCUPIED 2
#define HUMAN_OCCUPIED 3

#define CAMERA_LIST_VIZ 4
#define FEATURE_LIST_VIZ 5
#define SUB_CELLS_LIST_FREE_VIZ 6
#define SUB_CELLS_LIST_OBS_VIZ 7
#define SUB_CELLS_LIST_ROB_VIZ 8
#define SUB_CELLS_LIST_HUM_VIZ 9


#define START_SLICE_X 3.0
#define START_SLICE_Y 13.20
#define END_SLICE_X 21.0
#define END_SLICE_Y 21.0

#define P_H 0.2 // Prior prob
#define P_S_given_H 0.8
#define P_S_given_Hc 0.5

#define P_Sc_given_H 0.01
#define P_Sc_given_Hc 0.99

#define HUMANS_DETECTED 1
#define NO_HUMANS_DETECTED 0
#define PROB_THRESH 0.1

#define CAMERA_VIS_DEPTH 4 // 4 meters
#define CAMERA_MIN_DIST 0.45 //
#define CAMERA_VIS_WIDTH 1.25 // 0.5 meters
#define CAMERA_VIS_RES 0.05 // 0.01 meters

float P_Hc = 1 -P_H; 
float P_S = (P_S_given_H)*P_H + P_S_given_Hc*P_Hc;


/*std::string read_topic_name = "/rtabmap/proj_map";
std::string package_path = ros::package::getPath("grid");
std::string filename = "/home/stevenjj/test.bag";//package_path + "/grid_maps/ETC_Hallway_occupancy12-09-2016.bag";
*/

std::string read_topic_name = "grid/stored_proj_map";
std::string package_path = ros::package::getPath("grid_hsr");
std::string filename = package_path + "/grid_maps/ETC_Hallway_occupancy12-09-2016.bag";

std::string WORLD_FRAME = "world";
std::string MAP_FRAME = "map";


class Cell{
public:
	float x_center;
	float y_center;

	int grid_x_index;
	int grid_y_index;

	int id;

	int cell_type;

	Cell(int id_);
	Cell(float x_init, float y_init, int x_index_init, int y_index_init, int cell_id);
	Cell(float x_init, float y_init, int x_index_init, int y_index_init, int cell_id, int cell_type_in);	
	Cell();
	~Cell();
};


class CellFeature{
public:
	int id;
	float x_center;
	float y_center;	
	float cell_feature_size;

	std::map<int, int>   cell_index_to_type;
	std::map<int, float> cell_index_to_distance_to_center;
	std::map<int, int>   cell_type_to_count;


	int primary_type;

	float min_distance_to_center;


	void init_values();
	void compute_features();
	void clear();
	CellFeature();
	CellFeature(int id_in, float x_center_in, float y_center_in);	

	~CellFeature();
};


class GridMap{
public:
	ros::NodeHandle node;
	ros::Publisher cell_array_pub;
	ros::Publisher robot_cells_pub;
	ros::Publisher renew_dyn_grid_pub;

	ros::Publisher  CBA_grid_pub;
	ros::Publisher  proj_map_pub;
	ros::Publisher  camera_viz_pub;
	ros::Publisher  features_viz_pub;
	ros::Publisher  human_marker_pub;
	ros::Publisher  human_markerarray_pub;

	ros::Publisher feature_map_pub;

	ros::Subscriber trikey_state_sub;
	ros::Subscriber static_obs_sub;
	ros::Subscriber dynamic_obs_sub;
	ros::Subscriber gridcell_sub;
	ros::Subscriber sensor_sub;

	ros::Subscriber human_bounding_boxes_sub;	
	ros::Subscriber detected_humans_number_sub;
	ros::Publisher  human_cells_pub;
	std::vector<CellFeature> robot_envFeatures;


    // Grid Variables
	float cell_x_width;
	float cell_y_width;

	float desired_resolution;

	int num_of_cells_width;
	int num_of_cells_height;	

	int robot_pos_id;


	float origin_x;
	float origin_y;


	float robot_world_x_pos;
	float robot_world_y_pos;
	float robot_world_theta_pos;	


	bool detected_human;


	visualization_msgs::Marker 		map_free_cell_list;
	visualization_msgs::Marker 		map_free_cell_list_dyn;
	visualization_msgs::Marker 		map_obstacle_cell_list;		

	visualization_msgs::Marker      human_cell_list;
	visualization_msgs::Marker      robot_cell_list;

	visualization_msgs::Marker      camera_visibility_cell_list;

	visualization_msgs::Marker      cell_features_list;

	visualization_msgs::Marker      sub_cells_features_free_list;
	visualization_msgs::Marker           sub_cells_features_obstacle_list;
	visualization_msgs::Marker     		 sub_cells_features_robot_list;
	visualization_msgs::Marker     		 sub_cells_features_human_list;
		

	visualization_msgs::MarkerArray      features_viz;	


	tf::TransformBroadcaster 		world_to_map_br;
  	tf::Transform 					world_to_map_transform;	


	tf::TransformListener 			camera_to_world_listener;
	tf::StampedTransform 			camera_to_world_transform;
	tf::StampedTransform 			camera_to_baselink_transform;


	tf::TransformListener 			map_to_world_listener;
	tf::StampedTransform 			map_to_world_transform;



	visualization_msgs::MarkerArray 	 cell_array;
	visualization_msgs::MarkerArray 	 cell_array_dyn;


	//std::map<int, std::vector<int>> 
	//std::map<int, std::vector<int>>  occupancy_type_to_true_index;


	std::vector<Cell> data; // In row-major order;

	std::vector<int> index_of_obstacle_occ_cells;
	
	std::vector<int> index_of_human_occ_cells_updated_recently;
	std::map<int, float> map_index_of_human_cells_to_prob;

	std::vector<int> index_of_robot_occ_cells;	

	std::vector<int> index_of_updated_obstacle_occ_cells;
	std::vector<int> index_of_updated_free_occ_cells;			


	nav_msgs::OccupancyGrid static_map_grid;
	nav_msgs::OccupancyGrid dynamic_map_grid;
	nav_msgs::OccupancyGrid proj_map_grid; // The original map
	nav_msgs::OccupancyGrid feature_map_grid;

	std::vector<int> state_feature;

    // Functions
    visualization_msgs::Marker make_cell_marker(int index, int grid_x_index, int grid_y_index, int cell_color);
	visualization_msgs::Marker make_cell_list_marker(int occupancy_type);

	void init_construct_free_and_occ_cells();
	void construct_robot_cells();

	void static_obs_ref_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
	void dynamic_obs_ref_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
	void static_gridcell_callback(const nav_msgs::GridCells::ConstPtr& msg);
	void sensor_callback(const sensor_msgs::Imu::ConstPtr& msg);

    void joint_state_callback(const nav_msgs::Odometry::ConstPtr &msg);
    void human_detection_callback(const visualization_msgs::MarkerArray::ConstPtr &msg);    
    void number_detected_callback(const std_msgs::Int8::ConstPtr &msg);
	void update_human_occ_belief(int update_type);
	void change_cell_color(int marker_index, int occupancy_type);
	void bounding_box_to_occupany(float x_center, float y_center, float width, float length, int occupancy_type);
	std::map<int, int> bounding_box_to_world_indices(float x_center, float y_center, float width, float length);
	std::map<int, int> extract_world_indices_from_visible_camera_region(float depth, float width, float res);

	int ij_index_to_true_index(int index_i, int index_j);
    void identify_grid_map_occupancy();
    void identify_static_grid_map_occupancy();
    int proj_map_identify_occupancy(int index);
    int static_map_identify_occupancy(int index);
    void broadcast_tf_world_to_map();
    void calculate_robotEnvFeatures();
	void visualize_robotEnvFeatures();
	void change_marker_prop(int occupancy_type, visualization_msgs::Marker &marker);

    void publish();
    void humanpublish();
	void publish_orig_proj_map();
	void CBA_publish();
	int  globalcoord_To_Dyn_map_index(float x_pos, float y_pos);
	int  globalcoord_To_SScaled_map_index(float x_pos,float y_pos);
	bool checkpointHuman(float x_pos,float y_pos);

    // Constructor Destructor Functions
	GridMap();
	~GridMap();	

private:
    void initialize_marker_array();
    int read_proj_map();
    void convert_proj_map_to_cba_grid();
    void convert_static_map_to_cba_grid();
    void initialize_fature_occupancy();

};

