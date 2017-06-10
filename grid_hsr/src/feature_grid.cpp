#include "feature_grid.h"

// This class creates a grid of the world starting at x=0, y=0 and goes to x=CELL_X_WIDTH*NUM_CELL_WIDTH, y =CELL_Y_WIDTH*NUM_CELL_HEIGHT 

Cell::Cell(){
}
Cell::~Cell(){}

Cell::Cell(float x_init, float y_init, int x_index_init, int y_index_init, int cell_id): x_center(x_init),
																			y_center(y_init),
																			grid_x_index(x_index_init),
																			grid_y_index(y_index_init),
																			id(cell_id) {
}

Cell::Cell(float x_init, float y_init, int x_index_init, int y_index_init, int cell_id, int cell_type_in): x_center(x_init),
																			y_center(y_init),
																			grid_x_index(x_index_init),
																			grid_y_index(y_index_init),
																			id(cell_id),
																			cell_type(cell_type_in) {
}


CellFeature::CellFeature(){
	init_values();
}

CellFeature::CellFeature(int id_in, float x_center_in, float y_center_in): id(id_in), x_center(x_center_in), y_center(y_center_in){
	init_values();
}


void CellFeature::init_values(){
	// Initialize Cell Default Values
	cell_feature_size = CELL_FEATURE_SIZE; 
	min_distance_to_center = (cell_feature_size + (cell_feature_size/2.0)) * sqrt(2.0);
	primary_type = FREE_CELL;	
}

void CellFeature::clear(){
	cell_index_to_type.clear();
	cell_index_to_distance_to_center.clear();
	cell_type_to_count.clear();
	init_values();	
}

void CellFeature::compute_features(){
	// identify primary type
	int highest_count = 0;
	typedef std::map<int, int>::iterator it_type;
	for(it_type iterator = cell_type_to_count.begin(); iterator !=  cell_type_to_count.end(); iterator++) {    
	    int cell_type = iterator->first;  	    
   	    int cell_count = iterator->second;
   	    if (cell_type == ROBOT_OCCUPIED){
   	    	continue;
   	    }
   	    if (cell_type == HUMAN_OCCUPIED){
   	    	primary_type = HUMAN_OCCUPIED;
   	    	break;
   	    }
   	    if (cell_type == OBSTACLE_OCCUPIED){
   	    	primary_type = cell_type;
   	    }
	}


	typedef std::map<int, float>::iterator it_type2;
	for(it_type2 iterator = cell_index_to_distance_to_center.begin(); iterator !=  cell_index_to_distance_to_center.end(); iterator++) {    
		float distance = iterator->second;
		if (distance < min_distance_to_center){
			min_distance_to_center = distance;
		}
	}


}


CellFeature::~CellFeature(){}



GridMap::~GridMap(){
	detected_human = false;
}



void GridMap::calculate_robotEnvFeatures(){
	robot_world_x_pos;
	robot_world_y_pos;	
	float lower_left_corner_x = robot_world_x_pos - (CELL_FEATURE_SIZE*1.5);
	float lower_left_corner_y = robot_world_x_pos - (CELL_FEATURE_SIZE*1.5);	

	robot_envFeatures.clear();

	int id = 0;
	for(size_t j = 0; j < (int) FEATURE_CELL_HEIGHT; j++){
		for(size_t i = 0; i < (int) FEATURE_CELL_WIDTH; i++){
			CellFeature cell_feature;

			float x_center = (CELL_FEATURE_SIZE / 2.0) + CELL_FEATURE_SIZE*i + robot_world_x_pos  - (CELL_FEATURE_SIZE*1.5);
			float y_center = (CELL_FEATURE_SIZE / 2.0) + CELL_FEATURE_SIZE*j + robot_world_y_pos  - (CELL_FEATURE_SIZE*1.5);		
			float width = CELL_FEATURE_SIZE;
			float length = CELL_FEATURE_SIZE;

			std::map<int, int>   cell_index_to_type;
			std::map<int, int>   cell_type_to_count;
			std::map<int, float> cell_index_to_distance_to_center;

			std::map<int, int> encapsulated_world_indices = bounding_box_to_world_indices(x_center, y_center, width, length);

			typedef std::map<int, int>::iterator it_type;
			for(it_type iterator = encapsulated_world_indices.begin(); iterator !=  encapsulated_world_indices.end(); iterator++) {
			    int cell_index = iterator->first;
			    int cell_type = data[cell_index].cell_type;

			    // **Store cell type
			    cell_index_to_type[cell_index] = cell_type;

			    if ((cell_type == ROBOT_OCCUPIED) || (cell_type == FREE_CELL)){
			    	continue;
				}

			    // **Store number of occurance for each type
			    if (cell_type_to_count.count(cell_type) == 1){
			    	int count = cell_type_to_count[cell_type];
			    	cell_type_to_count[cell_type] = count + 1; 
			    }else{
					cell_type_to_count[cell_type] = 1;
				}

				int x_index = data[cell_index].grid_x_index;
				int y_index = data[cell_index].grid_y_index;

				float cell_x_center = (cell_x_width/2.0) + cell_x_width*x_index;// + robot_world_x_pos - (CELL_FEATURE_SIZE*1.5);
				float cell_y_center = (cell_y_width/2.0) + cell_y_width*y_index;// + robot_world_y_pos - (CELL_FEATURE_SIZE*1.5);

				// **Store each cell's distance to center
				float distance_to_center = sqrt(pow((robot_world_x_pos - cell_x_center), 2) + pow((robot_world_y_pos - cell_y_center), 2)); 
				cell_index_to_distance_to_center[cell_index] = distance_to_center;
			}

			cell_feature.id = id;
			cell_feature.x_center = x_center;
			cell_feature.y_center = y_center;
			cell_feature.cell_feature_size = CELL_FEATURE_SIZE;	

			cell_feature.cell_index_to_type = cell_index_to_type;
			cell_feature.cell_type_to_count = cell_type_to_count;			
			cell_feature.cell_index_to_distance_to_center = cell_index_to_distance_to_center;

			robot_envFeatures.push_back(CellFeature(cell_feature));
			id++;
		}
	}


	// Calculate features
	// Find minimum distance and primary types
	for(size_t i = 0; i < robot_envFeatures.size(); i++){
		robot_envFeatures[i].compute_features();
	}

/*	for(size_t i = 0; i < robot_envFeatures.size(); i++){
		std::cout << "index:" << robot_envFeatures[i].id <<
				 	 " type: " << robot_envFeatures[i].primary_type <<
				 	 " min_dist " << robot_envFeatures[i].min_distance_to_center << std::endl; 
	}
*/
//	visualize_robotEnvFeatures();

}

void GridMap::visualize_robotEnvFeatures(){
  	cell_features_list.points.clear();
	sub_cells_features_free_list.points.clear();
    sub_cells_features_obstacle_list.points.clear();
    sub_cells_features_robot_list.points.clear();
	sub_cells_features_human_list.points.clear();

	features_viz.markers.clear();


	for(size_t i = 0; i < robot_envFeatures.size(); i++){
		geometry_msgs::Point cell_loc;

		cell_loc.x = robot_envFeatures[i].x_center;
		cell_loc.y = robot_envFeatures[i].y_center;
		cell_loc.z = 0;


		int cell_occ_type = robot_envFeatures[i].primary_type;	
		cell_features_list.points.push_back(cell_loc);	

	}

	cell_features_list.pose.position.z = 1.25;
	cell_features_list.scale.x = CELL_FEATURE_SIZE;
	cell_features_list.scale.y = CELL_FEATURE_SIZE;	
	cell_features_list.scale.z = 0.125;
	cell_features_list.color.r = 0.0;
	cell_features_list.color.g = 0.0;
	cell_features_list.color.b = 0.0;
	cell_features_list.color.a = 1.0;


	features_viz.markers.push_back(cell_features_list);


	for(size_t i = 0; i < robot_envFeatures.size(); i++){
		typedef std::map<int, int>::iterator it_type;
		for(it_type iterator = robot_envFeatures[i].cell_index_to_type.begin(); iterator !=  robot_envFeatures[i].cell_index_to_type.end(); iterator++) {
			geometry_msgs::Point cell_loc;
			int cell_index = iterator->first;
			int cell_type = iterator->second;			

			int x_index = data[cell_index].grid_x_index;
			int y_index = data[cell_index].grid_y_index;

			cell_loc.x = (cell_x_width/2.0) + cell_x_width*x_index;// + robot_world_x_pos - (CELL_FEATURE_SIZE*1.5);
			cell_loc.y = (cell_y_width/2.0) + cell_y_width*y_index;// + robot_world_y_pos - (CELL_FEATURE_SIZE*1.5);
			cell_loc.z = 0.0;

			int cell_occ_type = data[cell_index].cell_type;	

			if (cell_occ_type == FREE_CELL){
				//sub_cells_features_free_list.pose.position.z = 1.25; 
				//sub_cells_features_free_list.scale.z = 0.125;				
				change_marker_prop(cell_occ_type, sub_cells_features_free_list);
				sub_cells_features_free_list.points.push_back(cell_loc);

			}
			if (cell_occ_type == OBSTACLE_OCCUPIED){
				change_marker_prop(cell_occ_type, sub_cells_features_obstacle_list);				
				sub_cells_features_obstacle_list.points.push_back(cell_loc);
			}			
			if (cell_occ_type == ROBOT_OCCUPIED){
				change_marker_prop(cell_occ_type, sub_cells_features_robot_list);				
				sub_cells_features_robot_list.points.push_back(cell_loc);
			}
			if (cell_occ_type == HUMAN_OCCUPIED){
				change_marker_prop(cell_occ_type, sub_cells_features_human_list);
				sub_cells_features_human_list.points.push_back(cell_loc);

			}			
		}
	}

	features_viz.markers.push_back(sub_cells_features_free_list);
	features_viz.markers.push_back(sub_cells_features_obstacle_list);	
	features_viz.markers.push_back(sub_cells_features_robot_list);		
	features_viz.markers.push_back(sub_cells_features_human_list);			


//	features_viz_pub.publish(features_viz);
}

void GridMap::change_marker_prop(int occupancy_type, visualization_msgs::Marker &marker){
	marker.pose.position.z = 1.3125;
	marker.scale.x = 0.5 * cell_x_width;
	marker.scale.y = 0.5 * cell_y_width;	
	marker.scale.z = 0.125;
	if (occupancy_type == ROBOT_OCCUPIED){
		marker.color.r = 0.0; // BLUE color
		marker.color.g = 0.0;
		marker.color.b = 0.8;		
		marker.color.a = 0.9;
	}else if (occupancy_type == HUMAN_OCCUPIED){
		marker.color.r = 0.8; // Purple color
		marker.color.g = 0.0;
		marker.color.b = 0.8;				
		marker.color.a = 0.9;

	}else if(occupancy_type == OBSTACLE_OCCUPIED){
		marker.color.r = 0.8; // Red color
		marker.color.g = 0.0;
		marker.color.b = 0.0;	
		marker.color.a = 0.9;

	}else{
		marker.color.r = 0.3; // Light Gray color
		marker.color.g = 0.3;
		marker.color.b = 0.3;
		marker.color.a = 0.5;

	}


}



void GridMap::broadcast_tf_world_to_map(){
  world_to_map_transform.setOrigin( tf::Vector3(-origin_x, -origin_y, 0.0) );
//  world_to_map_transform.setOrigin( tf::Vector3(0, 0, 0.0) );	
  tf::Quaternion q;
  float yaw = 0;
  q.setRPY(0, 0, yaw);
  world_to_map_transform.setRotation(q);
  world_to_map_br.sendTransform(tf::StampedTransform(world_to_map_transform, ros::Time::now(), WORLD_FRAME, MAP_FRAME));
}


GridMap::GridMap(){
	// int read_attempt = read_proj_map(); 
	// if (read_attempt == 0){
	// 	std::cerr << "Error size of map is 0" << std:: endl;
	// 	throw;
	// };

		// proj_map_grid = (*occ);
		// proj_map_grid.header.stamp =  ros::Time::now	();
		// proj_map_grid.header.frame_id = "map"; 

	initialize_fature_occupancy();

	broadcast_tf_world_to_map();
	
	desired_resolution = CELL_RESOLUTION;

	cell_x_width = desired_resolution;
	cell_y_width = desired_resolution;	

	//convert_proj_map_to_cba_grid();

	std::cout << num_of_cells_width << std::endl;
	std::cout << num_of_cells_height << std::endl;	


	num_of_cells_height=24;
	num_of_cells_width=24;
	
	robot_world_x_pos=0.0;
	robot_world_y_pos=0.0;

	// // Construct Grid Map and Cells:
	int cell_id = 0;
    // -----------------------------------------------------------
    // Initialize cell positions and construct map_grid nxm vector
	for (size_t j = 0; j < num_of_cells_height; j++){
        std::vector<Cell> row;
        // Iterate over columns
		for (size_t i = 0; i < num_of_cells_width; i++){			
            float cell_x_center = i*cell_x_width + (cell_x_width/2.0);
            float cell_y_center = j*cell_y_width + (cell_y_width/2.0);      
            int cell_type = FREE_CELL;      		
			Cell cell_new( cell_x_center, cell_y_center, i, j, cell_id, cell_type);

			data.push_back(cell_new);

            // Add to current row
            row.push_back(cell_new);
            cell_id++;
		}
	}

	// Assign Grid Map cell values mkmk
	//identify_grid_map_occupancy();
	
	identify_static_grid_map_occupancy();
	convert_static_map_to_cba_grid();

	// -----------------------------------------------------------
	//init_construct_free_and_occ_cells();
	//cell_array.markers.push_back(map_free_cell_list);
	// cell_array.markers.push_back(map_obstacle_cell_list);	

	// robot_cell_list = make_cell_list_marker(ROBOT_OCCUPIED);
	// human_cell_list = make_cell_list_marker(HUMAN_OCCUPIED);
	// camera_visibility_cell_list = make_cell_list_marker(CAMERA_LIST_VIZ);
 //  	cell_features_list = make_cell_list_marker(FEATURE_LIST_VIZ);

	// sub_cells_features_free_list = make_cell_list_marker(SUB_CELLS_LIST_FREE_VIZ);
	 //    sub_cells_features_obstacle_list = make_cell_list_marker(SUB_CELLS_LIST_OBS_VIZ);
	 //   sub_cells_features_robot_list = make_cell_list_marker(SUB_CELLS_LIST_ROB_VIZ);
	// sub_cells_features_human_list = make_cell_list_marker(SUB_CELLS_LIST_HUM_VIZ);
	//	initialize_marker_array();
}



int GridMap::read_proj_map(){
    rosbag::Bag bag;

	std::cout << "Opening: " << filename << std::endl;

	bag.open(filename, rosbag::bagmode::Read);

	std::vector<std::string> topics;
	topics.push_back(std::string(read_topic_name));

	rosbag::View view(bag, rosbag::TopicQuery(topics));

	foreach(rosbag::MessageInstance const m, view)
	{
		nav_msgs::OccupancyGrid::ConstPtr occ = m.instantiate<nav_msgs::OccupancyGrid>();
		std::cout <<"Width: " << occ->info.width << std::endl;
		std::cout <<"Height: " << occ->info.height << std::endl;

		std::cout << "X origin:" << occ->info.origin.position.x << std::endl;
		std::cout << "Y origin:" << occ->info.origin.position.y << std::endl;


		std::cout <<"Resolution: " << occ->info.resolution << std::endl;		
		// Copy Data;
		proj_map_grid = (*occ);
		proj_map_grid.header.stamp =  ros::Time::now	();
		proj_map_grid.header.frame_id = "map_local"; 
	}

	std::cout << "Finished reading bag" << std::endl;

    bag.close();		

	if (proj_map_grid.data.size() > 0){
		return 1;
	}else{
		return 0;
	}

}

void GridMap::convert_proj_map_to_cba_grid(){
	num_of_cells_width  = ceil((proj_map_grid.info.width * proj_map_grid.info.resolution) / cell_x_width); //NUM_CELL_WIDTH;
	num_of_cells_height = ceil((proj_map_grid.info.height * proj_map_grid.info.resolution) / cell_y_width);;	
	origin_x = proj_map_grid.info.origin.position.x;
	origin_y = proj_map_grid.info.origin.position.y;
}

void GridMap::convert_static_map_to_cba_grid(){
	num_of_cells_width  = ceil((static_map_grid.info.width * static_map_grid.info.resolution) / cell_x_width); //NUM_CELL_WIDTH;
	num_of_cells_height = ceil((static_map_grid.info.height * static_map_grid.info.resolution) / cell_y_width);;	
	origin_x = static_map_grid.info.origin.position.x;
	origin_y = static_map_grid.info.origin.position.y;
}

void GridMap::identify_static_grid_map_occupancy(){

	nav_msgs::OccupancyGrid::ConstPtr static_msg = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/static_obstacle_map_ref");
	static_map_grid.header.stamp =  ros::Time::now();
	static_map_grid.header.frame_id = "map"; 


	int static_map_grid_width = static_msg->info.width;
	int static_map_grid_height = static_msg->info.width;	
	float static_res = static_msg->info.resolution;

	static_map_grid.data.resize(static_msg->data.size());
	for(int i(0);i<static_msg->data.size();i++)
	{
		static_map_grid.data[i]=static_msg->data[i];
	}

	for(size_t index = 0; index < static_msg->data.size(); index++){
   		int i = index % static_map_grid_width;
   		int j = floor(index / static_map_grid_width);
		float x_coord_center = i*static_res + (static_res/2.0);
		float y_coord_center = j*static_res + (static_res/2.0);			
		int init_occupancy_type = static_map_identify_occupancy(index);

		// Call BoundingBox Occupancy
		if (init_occupancy_type == OBSTACLE_OCCUPIED){
			bounding_box_to_occupany(x_coord_center, y_coord_center, static_res, static_res, init_occupancy_type);
		}
	}
	std::cout << "Done with processing" << std::endl;
}

void GridMap::initialize_fature_occupancy(){

	nav_msgs::OccupancyGrid feature_map_msg;

	feature_map_grid.info.width=3;
	feature_map_grid.info.height=3;
	feature_map_grid.info.resolution=0.5;

	state_feature.resize(9);

	//feature_map_grid.info.origin.position.x=-0.5*feature_map_grid.info.width*0.5;
	//feature_map_grid.info.origin.position.y=-0.5*feature_map_grid.info.height*0.5;
}


void GridMap::identify_grid_map_occupancy(){
	int proj_map_width = (int) proj_map_grid.info.width;
	int proj_map_height = (int) proj_map_grid.info.height;	
	float res = proj_map_grid.info.resolution;

	for(size_t index = 0; index < proj_map_grid.data.size(); index++){
   		int i = index % proj_map_width;
   		int j = floor(index / proj_map_width);

		float x_coord_center = i*res + (res/2.0);
		float y_coord_center = j*res + (res/2.0);			

		int init_occupancy_type = static_map_identify_occupancy(index);


		//std::cout << "(x_c, y_c) = (" << x_coord_center << "," << y_coord_center << ")" << std::endl;

		// Call BoundingBox Occupancy
		if (init_occupancy_type == OBSTACLE_OCCUPIED){
			bounding_box_to_occupany(x_coord_center, y_coord_center, res, res, init_occupancy_type);
		}



	}
	std::cout << "Done with processing" << std::endl;

}



int GridMap::proj_map_identify_occupancy(int index){
	int proj_map_cell_value = (int) proj_map_grid.data[index];
	if ((proj_map_cell_value == 100) || (proj_map_cell_value == -1)) {
		return OBSTACLE_OCCUPIED;
	}

	return FREE_CELL;
}

int GridMap::static_map_identify_occupancy(int index){
	int proj_map_cell_value = (int) static_map_grid.data[index];
	if ((proj_map_cell_value == 1) || (proj_map_cell_value == 0)) {
		return FREE_CELL;
	}

	return OBSTACLE_OCCUPIED;
}


int  GridMap::globalcoord_To_SScaled_map_index(float x_pos,float y_pos)
{
	 std::vector<float> cur_coord(2,0.0);

	
	 //for case of using static map
	float reference_origin_x =-4;
	float reference_origin_y =-4;
	float Grid_STEP=0.5;
	int num_grid=24;

	//for case of using static map
	// double reference_origin_x =-3.5;
	// double reference_origin_y =-3.5;
	float  temp_x  = x_pos-reference_origin_x;
	float  temp_y = y_pos-reference_origin_y;

	cur_coord[0]= (int) (temp_x/Grid_STEP);
 	cur_coord[1]= (int)(temp_y/Grid_STEP);


 	robot_pos_id=num_grid*cur_coord[1]+cur_coord[0];
 	//ROS_INFO("Robot pos ID : %d \n", robot_pos_id);

 	return robot_pos_id;

}
 	






void GridMap::initialize_marker_array(){
   for(size_t index = 0; index < data.size(); index++){
        Cell current_cell;
        current_cell = data[index];     

   		int i = index % num_of_cells_width;
   		int j = floor(index / num_of_cells_width); 
        int cell_status = current_cell.cell_type;//FREE_CELL;

        visualization_msgs::Marker current_marker;
        current_marker = make_cell_marker(index, current_cell.grid_x_index, current_cell.grid_y_index, cell_status);
           
        cell_array.markers.push_back(current_marker);
   }


}

//mk : make free_and obstacle occupancy grid 
void GridMap::init_construct_free_and_occ_cells(){
	map_free_cell_list = make_cell_list_marker(FREE_CELL);
	map_obstacle_cell_list = make_cell_list_marker(OBSTACLE_OCCUPIED);

	for(size_t i = 0; i < data.size(); i++){
		geometry_msgs::Point cell_loc;

		int x_index = data[i].grid_x_index;
		int y_index = data[i].grid_y_index;
		cell_loc.x = (cell_x_width/2.0) + cell_x_width*x_index;
		cell_loc.y = (cell_y_width/2.0) + cell_y_width*y_index;
		cell_loc.z = -0.1;

		int cell_occ_type = data[i].cell_type;	

		if (cell_occ_type == FREE_CELL){
			map_free_cell_list.points.push_back(cell_loc);
		}
		if (cell_occ_type == OBSTACLE_OCCUPIED){
			map_obstacle_cell_list.points.push_back(cell_loc);
		}

	}

}

visualization_msgs::Marker GridMap::make_cell_list_marker(int occupancy_type){
	visualization_msgs::Marker marker;
	marker.header.frame_id = "map";
	marker.header.stamp = ros::Time();
	marker.ns = "grid_cells";
	marker.id = occupancy_type;
	marker.type = visualization_msgs::Marker::CUBE_LIST;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = 0;
	marker.pose.position.y = 0;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 0.95*cell_x_width;
	marker.scale.y = 0.95*cell_y_width;
	if (occupancy_type == OBSTACLE_OCCUPIED){
		marker.scale.z = 0.1;
	}else{
		marker.scale.z = 0.1;
	}
	marker.color.a = 0.7; // Don't forget to set the alpha!

	if (occupancy_type == ROBOT_OCCUPIED){
		marker.color.r = 0.0; // BLUE color
		marker.color.g = 0.0;
		marker.color.b = 0.8;		
	}else if (occupancy_type == HUMAN_OCCUPIED){
		marker.color.r = 0.8; // Purple color
		marker.color.g = 0.0;
		marker.color.b = 0.8;				
	}else if(occupancy_type == OBSTACLE_OCCUPIED){
		marker.color.r = 0.2; // Dark Gray color
		marker.color.g = 0.2;
		marker.color.b = 0.2;	
	}else{
		marker.color.r = 0.3; // Light Gray color
		marker.color.g = 0.3;
		marker.color.b = 0.4;
	}

	return marker;

}

visualization_msgs::Marker GridMap::make_cell_marker(int index, int grid_x_index, int grid_y_index, int cell_status){
	visualization_msgs::Marker marker;
	marker.header.frame_id = "base_link";
	marker.header.stamp = ros::Time();
	marker.ns = "cells";
	marker.id = index;
	marker.type = visualization_msgs::Marker::CUBE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = (cell_x_width/2.0) + cell_x_width*grid_x_index ;
	marker.pose.position.y = (cell_y_width/2.0) + cell_y_width*grid_y_index;
	marker.pose.position.z = -0.05;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 0.9*cell_x_width;
	marker.scale.y = 0.9*cell_y_width;
	marker.scale.z = 0.1;
	marker.color.a = 0.5; // Don't forget to set the alpha!

	if (cell_status == ROBOT_OCCUPIED){
		marker.color.r = 0.0; // BLUE color
		marker.color.g = 0.0;
		marker.color.b = 0.8;		
	}else if (cell_status == HUMAN_OCCUPIED){
		marker.color.r = 0.8; // YELLOW color
		marker.color.g = 0.8;
		marker.color.b = 0.0;				
	}else if(cell_status == OBSTACLE_OCCUPIED){
		marker.color.r = 0.2; // RED color
		marker.color.g = 0.2;
		marker.color.b = 0.2;	
	}else{
		marker.color.r = 0.8; // Gray color
		marker.color.g = 0.8;
		marker.color.b = 0.8;
	}

    marker.lifetime = ros::Duration();

    return marker;
}


void GridMap::construct_robot_cells(){
	robot_cell_list.points.clear();

 	for (size_t i = 0; i < index_of_robot_occ_cells.size(); i++){
		int cell_index = index_of_robot_occ_cells[i]; 		

		geometry_msgs::Point cell_loc;

		int x_index = data[cell_index].grid_x_index;
		int y_index = data[cell_index].grid_y_index;

		cell_loc.x = (cell_x_width/2.0) + cell_x_width*x_index;
		cell_loc.y = (cell_y_width/2.0) + cell_y_width*y_index;
		cell_loc.z = 0.0;

		int cell_occ_type = data[cell_index].cell_type;	

		robot_cell_list.points.push_back(cell_loc);

	}
	robot_cell_list.pose.position.z = 0.4;
	robot_cell_list.scale.z = 0.8;
	robot_cell_list.color.a = 0.4;

}

void GridMap::publish(){
	cell_array_pub.publish(cell_array);
	//robot_cells_pub.publish(cell_array_dyn);

}

void GridMap::humanpublish(){
	
	visualization_msgs::Marker marker_human;
	marker_human.header.frame_id = "/map"; 
    marker_human.header.stamp = ros::Time::now();
    marker_human.ns = "basic_shapes";
    marker_human.id = 0;

    uint32_t shape = visualization_msgs::Marker::CUBE;
    marker_human.type = shape;

    marker_human.pose.position.x = 1;
    marker_human.pose.position.y = 1;
    marker_human.pose.position.z = 1;

    marker_human.pose.orientation.x = 0.0;
    marker_human.pose.orientation.y = 0.0;
    marker_human.pose.orientation.z = 0.0;
    marker_human.pose.orientation.w = 1.0;

    double temp_dist,temp_dist2,temp_dist3;
    temp_dist  =0.25;
    temp_dist2 =0.25;
    temp_dist3 =0.5;

    //ROS_INFO("temp dist : %.3lf, temp dist2 : %.3lf, temp dist3 : %.3lf",temp_dist,temp_dist2,temp_dist3);
    marker_human.scale.x = std::abs(temp_dist);
    marker_human.scale.y = std::abs(temp_dist2);
    marker_human.scale.z = std::abs(temp_dist3);

    marker_human.color.r = ((float) 160.0) / 255.0;
    marker_human.color.g = ((float) 60.0) / 255.0;
    marker_human.color.b = ((float) 200.0 )/ 255.0;
    marker_human.color.a = 0.25;
	// human_boxes_array_pub.publish(marker_human);
	human_marker_pub.publish(marker_human);
	//robot_cells_pub.publish(cell_array_dyn);

}

void GridMap::change_cell_color(int marker_index, int occupancy_type){
	if (occupancy_type == ROBOT_OCCUPIED){
	    cell_array.markers[marker_index].color.r = 0.0;
	    cell_array.markers[marker_index].color.g = 0.0;
	    cell_array.markers[marker_index].color.b = 0.8;
	}else if (occupancy_type == HUMAN_OCCUPIED){
		cell_array.markers[marker_index].color.r = 0.8; // YELLOW color
		cell_array.markers[marker_index].color.g = 0.8;
		cell_array.markers[marker_index].color.b = 0.0;				
	}else if(occupancy_type == OBSTACLE_OCCUPIED){
		cell_array.markers[marker_index].color.r = 0.8; // RED color
		cell_array.markers[marker_index].color.g = 0.0;
		cell_array.markers[marker_index].color.b = 0.0;
	}else{
		cell_array.markers[marker_index].color.r = 0.8; // Gray color
		cell_array.markers[marker_index].color.g = 0.8;
		cell_array.markers[marker_index].color.b = 0.8;
	}



}


void GridMap::static_obs_ref_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
		std::cout <<"static_Width: " << msg->info.width << std::endl;
		std::cout <<"static_Height: " << msg->info.height << std::endl;
		std::cout << "static_X origin:" << msg->info.origin.position.x << std::endl;
		std::cout << "static_Y origin:" << msg->info.origin.position.y << std::endl;
		std::cout <<"static_Resolution: " << msg->info.resolution << std::endl;		

		// Copy Data;
		static_map_grid = (*msg);
		static_map_grid.header.stamp =  ros::Time::now	();
		static_map_grid.header.frame_id = "map"; 
		return;

}

void GridMap::sensor_callback(const sensor_msgs::Imu::ConstPtr& msg)
{

	//	robot_world_theta_pos = msg->orientation.z;// - origin_y;	



}

//mk
void GridMap::dynamic_obs_ref_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
		// dynamic_map_grid.info.width= 140;
		// dynamic_map_grid.info.height= 140;
		// dynamic_map_grid.info.resolution=0.05;
		// dynamic_map_grid.info.origin.position.x=-4.5;
		// dynamic_map_grid.info.origin.position.y=-4.5;
		// dynamic_map_grid.data.resize(140*140);


    	int datasize=9;

		dynamic_map_grid.info.width=msg->info.width;
		dynamic_map_grid.info.height= msg->info.height;
		dynamic_map_grid.info.resolution=msg->info.resolution;
		dynamic_map_grid.info.origin.position.x=msg->info.origin.position.x;
		dynamic_map_grid.info.origin.position.y=msg->info.origin.position.y;
		dynamic_map_grid.data.resize(140*140);
		
		for(int i(0);i<msg->data.size();i++)
			dynamic_map_grid.data[i]=msg->data[i];

		feature_map_grid.info.width=3;
		feature_map_grid.info.height=3;
		feature_map_grid.info.resolution=0.75;
		
		float origin_x= (-1.5)*feature_map_grid.info.resolution;           
		float origin_y= (-1.5)*feature_map_grid.info.resolution;
		float origin_rotated_vector_pos_x=cos(0)*origin_x-sin(0)*origin_y;
		float origin_rotated_vector_pos_y= sin(0)*origin_x+cos(0)*origin_y;

		feature_map_grid.info.origin.position.x=origin_rotated_vector_pos_x;
		feature_map_grid.info.origin.position.y=origin_rotated_vector_pos_y;

		//Initialize constants for calculating width and height
		float smallwindow_res= 0.05;
		int width_ratio_cons= 15;
		int height_ratio_cons=15;
		
		std::map<int,int> occupancyCountMap;
		std::vector<double> x_pos_set;
		std::vector<double> y_pos_set;

		std::vector< std::vector<double> > x_pos_set_dyn;
		std::vector< std::vector<double> > y_pos_set_dyn;
		std::vector<int> map_idset;
		
		x_pos_set_dyn.resize(9);
		y_pos_set_dyn.resize(9);

		float global_pos_x=0.0;
		float global_pos_y=0.0;

		//current robot position
		int numcount=0;
		int humancount=0;
		int mapidx=0;
		//check occupancy from dynamic map
		std::map<int,float> MinDistMap;
		float min_distance=100;

		for(int j(0);j<feature_map_grid.info.height;j++)
			for(int i(0); i<feature_map_grid.info.width;i++)
			{
				int feature_map_idx=j*feature_map_grid.info.height+i;

				float robot_pos_x = robot_world_x_pos;				//global_pos_yal fame
				float robot_pos_y = robot_world_y_pos;				//global fame
				double robot_theta = robot_world_theta_pos; 		//Updated in jointstatescallbackfunction()
				//float robot_theta = 0.315231; //Updated in jointstatescallbackfunction()
				
				float vector_pos_x= (-0.5*feature_map_grid.info.width+i)*feature_map_grid.info.resolution;           
				float vector_pos_y= (-0.5*feature_map_grid.info.height+j)*feature_map_grid.info.resolution;

				float rotated_vector_pos_x=cos(robot_theta)*vector_pos_x-sin(robot_theta)*vector_pos_y;
				float rotated_vector_pos_y=sin(robot_theta)*vector_pos_x+cos(robot_theta)*vector_pos_y;

				global_pos_x=robot_pos_x+rotated_vector_pos_x;
				global_pos_y=robot_pos_y+rotated_vector_pos_y;

				//ROS_INFO("coord x : %.3lf , coord y : %.3lf theta : %.3lf \n",robot_pos_x,robot_pos_y,robot_theta);
				x_pos_set.push_back(global_pos_x);
				y_pos_set.push_back(global_pos_y);

				//check each distance of small grids
				numcount=0;
				humancount=0;
				
				min_distance=100;
				for(int column_j(0);column_j<height_ratio_cons;column_j++)
				 	for(int row_i(0);row_i<width_ratio_cons;row_i++)
				 	{
				 		float SM_move_x=row_i*smallwindow_res;
				 		float SM_move_y=column_j*smallwindow_res;

				 		//find each direction vector from each origin of small map
				 		float SM_rotated_vector_pos_x= cos(robot_theta)*SM_move_x-sin(robot_theta)*SM_move_y;
						float SM_rotated_vector_pos_y= sin(robot_theta)*SM_move_x+cos(robot_theta)*SM_move_y;

						//get global coordinates of small map
						float SM_global_pos_x=global_pos_x+SM_rotated_vector_pos_x;
						float SM_global_pos_y=global_pos_y+SM_rotated_vector_pos_y;		

						int dyn_map_idx=globalcoord_To_Dyn_map_index(SM_global_pos_x,SM_global_pos_y);
						x_pos_set_dyn[mapidx].push_back(SM_global_pos_x);
						y_pos_set_dyn[mapidx].push_back(SM_global_pos_y);

						//check this point is in the human bounding box
						bool isitHuman = checkpointHuman(SM_global_pos_x,SM_global_pos_y);
						if(isitHuman)
							 humancount++;

						//ROS_INFO("coord x, coord y, map index : %d\n",dyn_map_idx);
						float temp_occupancy= msg->data[dyn_map_idx];


						if(temp_occupancy>0){

						 	float dist_from_Robot  = pow(SM_global_pos_x-robot_world_x_pos,2)+pow(SM_global_pos_y-robot_world_y_pos,2);
						 		  dist_from_Robot  = sqrt(dist_from_Robot);

						 	if(dist_from_Robot<min_distance)
						 		min_distance=dist_from_Robot;

						 	numcount++;
						}
				 	}

				 	if(min_distance==100)
				 		min_distance=0;

				 	MinDistMap[feature_map_idx]=min_distance;
				 	
				 	if(humancount)
				 		occupancyCountMap[feature_map_idx]=1000;
				 	else
				 		occupancyCountMap[feature_map_idx]=numcount;

				 	mapidx++;



		 }


		 //Assuming that human global pose is 30, 20;
		 // I have to conver this global coordinate to local dynamic window
		 // 
		 int human_global_x=10.2;
		 int human_global_y=10.2;;

		 int human_idx_local_dynamic_window=globalcoord_To_Dyn_map_index(human_global_x,human_global_y);
		 map_free_cell_list = make_cell_list_marker(HUMAN_OCCUPIED);
		 geometry_msgs::Point cell_loc;
		cell_loc.x = human_global_x;
	 	cell_loc.y = human_global_y;
	 	cell_loc.z = 0.2;
	 	map_free_cell_list.points.push_back(cell_loc);
	 	cell_array.markers.push_back(map_free_cell_list);


		 // Checking map index 
		 // std::map<int,int>::iterator mapiter;
		 // for(mapiter=occupancyCountMap.begin();mapiter!=occupancyCountMap.end();mapiter++)
		 // 	ROS_INFO("map index : %d, counts : %d \n",mapiter->first,mapiter->second);
				

		//------------------checking map origin point -------------------		
		// map_free_cell_list = make_cell_list_marker(FREE_CELL);
		// for(size_t kk = 0; kk < 9; kk++){
		// 	geometry_msgs::Point cell_loc;

		// 	cell_loc.x = x_pos_set[kk];
		// 	cell_loc.y = y_pos_set[kk];
		// 	cell_loc.z = 0.2;
		// 	map_free_cell_list.points.push_back(cell_loc);
		// 	}

		//------------------checking map origin point -------------------
		// 	geometry_msgs::Point cell_loc2;
		// 	cell_loc2.x = robot_world_x_pos;
		// 	cell_loc2.y = robot_world_x_pos;
		// 	cell_loc2.z = 0.2;

		// 	map_free_cell_list.points.push_back(cell_loc2);				
		// 	cell_array.markers.push_back(map_free_cell_list);

		//For checking 
		//  //map_free_cell_list_dyn= make_cell_list_marker(2);
		// for(int m(0);m<9;m++){
		// for(size_t dyn_i(0);dyn_i<15*15;dyn_i++ )
		// {
		// 	geometry_msgs::Point cell_loc_dyn;
		// 	cell_loc_dyn.x = x_pos_set_dyn[m][dyn_i];
		// 	cell_loc_dyn.y = y_pos_set_dyn[m][dyn_i];
		// 	cell_loc_dyn.z = 0.1;
		// 	// ROS_INFO("coord x %.3lf, coord y %.3lf, map index : \n",x_pos_set_dyn[dyn_i],y_pos_set_dyn[dyn_i]);
		// 	map_free_cell_list.points.push_back(cell_loc_dyn);
		// }
		// }
		// cell_array.markers.push_back(map_free_cell_list);


		//Making feature_grid msgs 
		feature_map_grid.data.resize(datasize);
		for(int i(0);i<datasize;i++)
		{	
			feature_map_grid.data[i] = occupancyCountMap[i];

			if(occupancyCountMap[i]>35)
				state_feature[i]=OBSTACLE_OCCUPIED;
			else if(occupancyCountMap[i]>999)
				state_feature[i]=HUMAN_OCCUPIED;
			else
				state_feature[i]=0;

		}	


		feature_map_grid.header.stamp =  ros::Time::now();
		feature_map_grid.header.frame_id = "base_link"; 
		feature_map_pub.publish(feature_map_grid);


		//Making cba_msgs
		cba_msgs::CBA_NavInfo cba_feature_msg;
		cba_feature_msg.header.stamp = ros::Time::now();
		cba_feature_msg.width = num_of_cells_width;
		cba_feature_msg.height = num_of_cells_height;
		cba_feature_msg.int_robot_id=robot_pos_id;
		cba_feature_msg.human_present=1;


		//ROS_INFO("human_present:%d\n",cba_feature_msg.int_robot_id);

		cba_feature_msg.state_type.resize(datasize-1);
		cba_feature_msg.state_distance.resize(datasize-1);

		for(int i(0);i<datasize-1;i++)
		{
			if(i<4)
			{
				cba_feature_msg.state_type[i]=state_feature[i];
				cba_feature_msg.state_distance[i]=MinDistMap[i];
			}	
			else	
			{
				cba_feature_msg.state_type[i]=state_feature[i+1];
				cba_feature_msg.state_distance[i]=MinDistMap[i+1];		
			}
		}

		CBA_grid_pub.publish(cba_feature_msg);
	
		//check for index or dynamic map
		// for(int k(0);k<map_idset.size();k++)
		// 	dynamic_map_grid.data[map_idset[k]]=50;
		//  dynamic_map_grid.header.stamp =  ros::Time::now();
		//  dynamic_map_grid.header.frame_id = "map"; 
		// renew_dyn_grid_pub.publish(dynamic_map_grid);

		return;
}

void GridMap::static_gridcell_callback(const nav_msgs::GridCells::ConstPtr& msg)
{



}

bool GridMap::checkpointHuman(float x_pos,float y_pos)
{
	//let's make bounding box for human and check the global point for human
	bool booldetectedhuman=1;					//this variable should be come from the detected box topic
	float detected_human_x=1;
	float detected_human_y=1;
	float boundingbox_length_x=0.4;
	float boundingbox_length_y=0.4;


	//detected bounding box
	if(booldetectedhuman)
	{
		if((x_pos>detected_human_x-0.5*boundingbox_length_x) && (x_pos<detected_human_x+0.5*boundingbox_length_x))
			if((y_pos>detected_human_y-0.5*boundingbox_length_y) && (y_pos<detected_human_y+0.5*boundingbox_length_y))
				return true;
	}


	return false;

}





int GridMap::globalcoord_To_Dyn_map_index(float x_pos, float y_pos)
{
	float map_width=140;
	float map_height=140;
	float map_resolution=0.05;


	float map_origin_left_corner_x=dynamic_map_grid.info.origin.position.x;
	float map_origin_left_corner_y=dynamic_map_grid.info.origin.position.y;

	float dist_x= x_pos-map_origin_left_corner_x;
	float dist_y= y_pos-map_origin_left_corner_y;

	int map_coord_i=floor(dist_x/map_resolution);
	int map_coord_j=floor(dist_y/map_resolution);

	int map_index=map_width*map_coord_j+map_coord_i;

	return map_index;

}




void GridMap::joint_state_callback(const nav_msgs::Odometry::ConstPtr &msg){
	
	float robot_x = msg->pose.pose.position.x;// - origin_x;
	float robot_y = msg->pose.pose.position.y;// - origin_y;	
	float robot_theta = msg->pose.pose.orientation.z;// - origin_y;	//this is not exact=>sensor_imu

	//ROS_INFO("base_x: %.3lf , _y : %.3lf, theta : %.3lf \n",robot_x,robot_y,robot_theta);
	robot_world_x_pos = robot_x;
	robot_world_y_pos = robot_y;
	robot_world_theta_pos=asin(robot_theta)*2;

	globalcoord_To_SScaled_map_index(robot_x,robot_y);


	//ROS_INFO("robot_theta : %.3lf \n ", robot_world_theta_pos);

	for(size_t i = 0; i < index_of_robot_occ_cells.size(); i++){
		int cell_index = index_of_robot_occ_cells[i];		
		data[cell_index].cell_type = FREE_CELL;
	}

	 index_of_robot_occ_cells.clear();
	// bounding_box_to_occupany(robot_x, robot_y, TRIKEY_SIDE_WIDTH, TRIKEY_SIDE_WIDTH, ROBOT_OCCUPIED);
	// construct_robot_cells();

}

void GridMap::human_detection_callback(const visualization_msgs::MarkerArray::ConstPtr &msg){
	detected_human = true;
	// try{
	//   camera_to_world_listener.lookupTransform("/base_link", "/head_rgbd_sensor_rgb_frame", ros::Time(0), human_detection_callback);
 //    }
 //    catch (tf::TransformException &ex) {
 //      ROS_ERROR("%s",ex.what());
 //      return;
 //      //ros::Duration(1.0).sleep();
 //    }


//     index_of_human_occ_cells_updated_recently.clear();

	for(size_t i = 0; i < msg->markers.size(); i++){
		visualization_msgs::Marker h;
		h = msg->markers[i];
/*		std::cout << "I see human " << i << std::endl;
		std::cout << "Header frame id:" << h.header.frame_id << std::endl;
		std::cout << "(x, y, z) = (" << h.pose.position.x  << "," << h.pose.position.y << "," << h.pose.position.z << ")" << std::endl;	
*/
		// Calculate Position of Box in world wrame
		tf::Vector3 r_vec_camera_frame(h.pose.position.x, h.pose.position.y, h.pose.position.z);
		tf::Stamped<tf::Vector3> stamped_r_vec_camera_frame(r_vec_camera_frame, ros::Time(0), h.header.frame_id);
		tf::Vector3 r_vec_world_frame;
		tf::Stamped<tf::Vector3> stamped_r_vec_world_frame(r_vec_world_frame,  ros::Time(0), "map");

		// Calculate Size of Box
		tf::Vector3 r_size_camera_frame(h.scale.x, h.scale.y, h.scale.z);
		tf::Stamped<tf::Vector3> stamped_r_size_camera_frame(r_size_camera_frame, ros::Time(0), h.header.frame_id);

		tf::Vector3 r_size_world_frame;
		tf::Stamped<tf::Vector3> stamped_r_size_world_frame(r_size_world_frame, ros::Time(0), "map");



// 		camera_to_world_listener.transformVector("world", ros::Time(0), stamped_r_vec_camera_frame, "world", stamped_r_vec_world_frame);
// 		camera_to_world_listener.transformVector("world", ros::Time(0), stamped_r_size_camera_frame, "world", stamped_r_size_world_frame);

// 		float x_center = camera_to_world_transform.getOrigin().x() + stamped_r_vec_world_frame.getX();
// 		float y_center = camera_to_world_transform.getOrigin().y() + stamped_r_vec_world_frame.getY();
// 		float width = stamped_r_size_world_frame.getX();
// 		float length = stamped_r_size_world_frame.getY();	

// 		bounding_box_to_occupany(x_center, y_center, std::abs(width), std::abs(length), HUMAN_OCCUPIED);			

 	}

// 	update_human_occ_belief(HUMANS_DETECTED);

// 	human_cell_list.points.clear();

// 	typedef std::map<int, float>::iterator it_type;
// 	for(it_type iterator = map_index_of_human_cells_to_prob.begin(); iterator !=  map_index_of_human_cells_to_prob.end(); iterator++) {
// 	    //iterator->first = key
// 	    //iterator->second = value

// 		int cell_index = iterator->first;
// 		geometry_msgs::Point cell_loc;

// 		int x_index = data[cell_index].grid_x_index;
// 		int y_index = data[cell_index].grid_y_index;

// 		cell_loc.x = (cell_x_width/2.0) + cell_x_width*x_index;
// 		cell_loc.y = (cell_y_width/2.0) + cell_y_width*y_index;
// 		cell_loc.z = 0.0;

// 		int cell_occ_type = data[cell_index].cell_type;	

// 		human_cell_list.points.push_back(cell_loc);

// 	}
// 	human_cell_list.pose.position.z = 1.25/2.0;
// 	human_cell_list.scale.z = 1.25;
// 	human_cell_list.color.a = 0.4;
// 	human_cells_pub.publish(human_cell_list);

// 	detected_human = false;
}


void GridMap::bounding_box_to_occupany(float x_center, float y_center, float width, float length, int occupancy_type){
	float lower_left_corner_x = (x_center - width/2.0);
	float lower_left_corner_y = (y_center - length/2.0);

	float upper_right_corner_x = (x_center + width/2.0);
	float upper_right_corner_y = (y_center + length/2.0);

	int lower_left_corner_index_i = (int) round( (lower_left_corner_x)/cell_x_width);
	int lower_left_corner_index_j = (int) round( (lower_left_corner_y)/cell_y_width);

	int upper_right_corner_index_i = (int) round( (upper_right_corner_x)/cell_x_width);
	int upper_right_corner_index_j = (int) round( (upper_right_corner_y)/cell_y_width);

/*
	std::cout << lower_left_corner_x << " " << lower_left_corner_y << std::endl;
	std::cout << upper_right_corner_x << " " << upper_right_corner_y << std::endl;	
//	std::cout << "START" << std::endl;
	std::cout << lower_left_corner_index_i << " " << lower_left_corner_index_j << std::endl;
	std::cout << upper_right_corner_index_i << " " << upper_right_corner_index_j << std::endl;	
*/
	// Color all cells inside the box formed by the two corners
	for(int i = 0; i < (upper_right_corner_index_i - lower_left_corner_index_i + 1); i++){
		for(int j = 0; j < (upper_right_corner_index_j - lower_left_corner_index_j+1); j++){

			int i_x = lower_left_corner_index_i + i;
			int j_y = lower_left_corner_index_j + j;

			if (i_x < 0){
				i_x = 0;
			}
			if (j_y < 0){
				j_y = 0;
			}

			if (i_x > num_of_cells_width){
				i_x = (num_of_cells_width - 1);
			}
			if (j_y > num_of_cells_height){
				j_y = (num_of_cells_height - 1);
			}			

			int true_index = ij_index_to_true_index(i_x, j_y);

			//std::cout << "Index i: "<< i << " Index J: " << j << "Type: " << occupancy_type << std::endl; 

//			std::cout << "INDEX:" << true_index << std::endl;
			if (occupancy_type == ROBOT_OCCUPIED){
				if ( (data[true_index].cell_type == FREE_CELL) || (data[true_index].cell_type == ROBOT_OCCUPIED) ){
					 data[true_index].cell_type = ROBOT_OCCUPIED;
					 index_of_robot_occ_cells.push_back(true_index);
				}
			}else if (occupancy_type == OBSTACLE_OCCUPIED){
				// Store Cell Occupancy. by changing status of cell:
				 data[true_index].cell_type = OBSTACLE_OCCUPIED;
			 	 index_of_obstacle_occ_cells.push_back(true_index);
				// Store Robot Occupancy Index

			}else if (occupancy_type = HUMAN_OCCUPIED){
				if ((data[true_index].cell_type != OBSTACLE_OCCUPIED) || (data[true_index].cell_type == HUMAN_OCCUPIED)){
					 data[true_index].cell_type = HUMAN_OCCUPIED;
					 index_of_human_occ_cells_updated_recently.push_back(true_index);

					 if (map_index_of_human_cells_to_prob.count(true_index) == 1){
					 	// Encountered the same cells. Update probability:
					 	// P(H|S) = P(S|H)P(H) / P(S)
						float prior = map_index_of_human_cells_to_prob[true_index]; // P(H)
						float P_S = P_S_given_H*prior + P_S_given_Hc*(1-prior);
						float posterior = (P_S_given_H)*prior / P_S;
						map_index_of_human_cells_to_prob[true_index] = posterior;

					 }else{
					 	map_index_of_human_cells_to_prob[true_index] = 0.1;
					 }				
				}

			}
	


		}
	}
/*
	human_cell_list.points.clear();

	typedef std::map<int, float>::iterator it_type;
	for(it_type iterator = map_index_of_human_cells_to_prob.begin(); iterator !=  map_index_of_human_cells_to_prob.end(); iterator++) {
	    //iterator->first = key
	    //iterator->second = value

		int cell_index = iterator->first;
		geometry_msgs::Point cell_loc;

		int x_index = data[cell_index].grid_x_index;
		int y_index = data[cell_index].grid_y_index;

		cell_loc.x = (cell_x_width/2.0) + cell_x_width*x_index;
		cell_loc.y = (cell_y_width/2.0) + cell_y_width*y_index;
		cell_loc.z = 0.0;

		int cell_occ_type = data[cell_index].cell_type;	

		human_cell_list.points.push_back(cell_loc);

	}
	human_cell_list.pose.position.z = 0.5;
	human_cell_list.scale.z = 1;
	human_cell_list.color.a = 0.4;
	human_cells_pub.publish(human_cell_list);
*/


}

void GridMap::number_detected_callback(const std_msgs::Int8::ConstPtr &msg){
	int number =  (int) msg->data;
	std::cout << "Number of detected humans" << number << std::endl;
	if (number == 0){
		std::cout << "no detection" << std::endl;
		update_human_occ_belief((int) NO_HUMANS_DETECTED);
	}
}

void GridMap::update_human_occ_belief(int update_type){
		std::map<int, int> map_index_recently_updated;
		if (update_type == (int) HUMANS_DETECTED){
			for(int i = 0; i < index_of_human_occ_cells_updated_recently.size(); i++){
				int index = index_of_human_occ_cells_updated_recently[i];
				map_index_recently_updated[index] = index;
			}
		}

		camera_visibility_cell_list.points.clear();
		std::map<int, int> camera_visible_world_indices = extract_world_indices_from_visible_camera_region(CAMERA_VIS_DEPTH, CAMERA_VIS_WIDTH, CAMERA_VIS_RES);
		//std::cout << "size of cell list:" << camera_visible_world_indices.size() << std::endl;
		typedef std::map<int, int>::iterator it_type;
		for(it_type iterator = camera_visible_world_indices.begin(); iterator !=  camera_visible_world_indices.end(); iterator++){
			int cell_index = iterator->first; 		

			//std::cout << "index:" <<  cell_index << std::endl;
			geometry_msgs::Point cell_loc;
			int x_index = data[cell_index].grid_x_index;
			int y_index = data[cell_index].grid_y_index;

			cell_loc.x = (cell_x_width/2.0) + cell_x_width*x_index;
			cell_loc.y = (cell_y_width/2.0) + cell_y_width*y_index;
			cell_loc.z = 0.0;

			camera_visibility_cell_list.points.push_back(cell_loc);

		}
		camera_visibility_cell_list.pose.position.z = 0.1;

		camera_visibility_cell_list.scale.z = 0.2;
		camera_visibility_cell_list.color.r = 0.8;
		camera_visibility_cell_list.color.g = 0.8;
		camera_visibility_cell_list.color.b = 0.0;
		camera_visibility_cell_list.color.a = 0.5;
		
		camera_viz_pub.publish(camera_visibility_cell_list);


//		store recently updated indices in map temporarily
	 
// 		// Update the region in front of the robot
		// Extract cell indices in front of the robot
//		extract_world_indices_from_visible_camera_region(float depth, float width, float res)
		// For each cell, check if is labeled as human.
		// If cell is recently updated, continue;
		// If labeled as human, update probability of cell regions


		std::vector<int> indices_to_assign_as_free;

		typedef std::map<int, int>::iterator it_type;
		for(it_type iterator = camera_visible_world_indices.begin(); iterator !=  camera_visible_world_indices.end(); iterator++) {
			int cell_index = iterator->first;
			if (map_index_recently_updated.count(cell_index) == 1){
				continue;
			}
			if (data[cell_index].cell_type == HUMAN_OCCUPIED){
				// Update probability			
				float prior = map_index_of_human_cells_to_prob[cell_index]; // P(H)
				float P_S = P_Sc_given_H*(prior) + P_Sc_given_Hc*(1-prior);
				float posterior = prior*0.4;
				map_index_of_human_cells_to_prob[cell_index] = posterior;

				//std::cout << "Prob: " << posterior << std::endl;

				if (posterior < PROB_THRESH){
					indices_to_assign_as_free.push_back(cell_index);
				}

			}

		}

		// For each low prob cell, delete it and update the cell type

		for(size_t i = 0; i < indices_to_assign_as_free.size(); i++){
			int index_to_erase =  indices_to_assign_as_free[i];
			map_index_of_human_cells_to_prob.erase(index_to_erase);
			data[index_to_erase].cell_type = FREE_CELL; 
		}

	human_cell_list.points.clear();
	typedef std::map<int, float>::iterator it_type2;
	for(it_type2 iterator = map_index_of_human_cells_to_prob.begin(); iterator !=  map_index_of_human_cells_to_prob.end(); iterator++) {
	    //iterator->first = key
	    //iterator->second = value

		int cell_index = iterator->first;
		geometry_msgs::Point cell_loc;

		int x_index = data[cell_index].grid_x_index;
		int y_index = data[cell_index].grid_y_index;

		cell_loc.x = (cell_x_width/2.0) + cell_x_width*x_index;
		cell_loc.y = (cell_y_width/2.0) + cell_y_width*y_index;
		cell_loc.z = 0.0;

		int cell_occ_type = data[cell_index].cell_type;	

		human_cell_list.points.push_back(cell_loc);

	}
	human_cell_list.pose.position.z = 1.25/2.0;
	human_cell_list.scale.z = 1.25;
	human_cell_list.color.a = 0.4;
	human_cells_pub.publish(human_cell_list);

	detected_human = false;


}


std::map<int, int> GridMap::extract_world_indices_from_visible_camera_region(float depth, float width, float res){
/*		float depth = 4.0; // Depth direction
		float width = 2.0
		float res = 0.5;*/
		std::string origin_frame_id;
		origin_frame_id = "head_rgbd_sensor_gazebo_frame";
		std::string dest_frame_id;
		dest_frame_id = "/base_link";

		float camera_min_distance_to_resolution = CAMERA_MIN_DIST;

		int num_visible_depth_cells = ceil(depth/res);
		int num_visible_width_cells = ceil(width/res);

		std::vector<float> cells_z_value;
		std::vector<float> cells_x_value;
		
		for(size_t j = 0; j < num_visible_depth_cells; j++){
			for(size_t i = 0; i < num_visible_width_cells; i++){
					cells_x_value.push_back(i*res + res/2.0);
					cells_z_value.push_back(j*res + res/2.0);					
			}
		}

		std::map<int, int> world_indices;
		ros::Time this_time =  ros::Time(0);
		try{		
		  camera_to_world_listener.lookupTransform(dest_frame_id, origin_frame_id, this_time, camera_to_world_transform);
		}
		catch (tf::TransformException &ex) {
		  ROS_ERROR("%s",ex.what());
		  return world_indices;
		  //ros::Duration(1.0).sleep();
		}

		for(size_t i = 0; i < cells_x_value.size(); i++){
			float cell_x_center = cells_x_value[i];
			float cell_y_center = cells_z_value[i];
			float cur_cell_width = res;
			float cur_cell_length = res;


			// Calculate Position of Box in world wrame
			tf::Vector3 r_vec_camera_frame((cell_y_center+camera_min_distance_to_resolution), (cell_x_center-width/2.0), 0);
			tf::Stamped<tf::Vector3> stamped_r_vec_camera_frame(r_vec_camera_frame, this_time, origin_frame_id);
			tf::Vector3 r_vec_world_frame;
			tf::Stamped<tf::Vector3> stamped_r_vec_world_frame(r_vec_world_frame,  this_time, dest_frame_id);

			// Calculate Size of Box
			tf::Vector3 r_size_camera_frame(res, 0, res);
			tf::Stamped<tf::Vector3> stamped_r_size_camera_frame(r_size_camera_frame, this_time, origin_frame_id);

			tf::Vector3 r_size_world_frame;
			tf::Stamped<tf::Vector3> stamped_r_size_world_frame(r_size_world_frame, this_time, dest_frame_id);



			camera_to_world_listener.transformVector(dest_frame_id, this_time, stamped_r_vec_camera_frame, WORLD_FRAME, stamped_r_vec_world_frame);
			camera_to_world_listener.transformVector(dest_frame_id, this_time, stamped_r_size_camera_frame, WORLD_FRAME, stamped_r_size_world_frame);

			float x_center = camera_to_world_transform.getOrigin().x() + stamped_r_vec_world_frame.getX();
			float y_center = camera_to_world_transform.getOrigin().y() + stamped_r_vec_world_frame.getY();
			float box_width = res;//stamped_r_size_world_frame.getX();
			float box_length = res;//stamped_r_size_world_frame.getY();	

			std::map<int, int> indices_encapuslated;
			indices_encapuslated = bounding_box_to_world_indices(x_center, y_center, box_width, box_length);


			typedef std::map<int, int>::iterator it_type;
			for(it_type iterator = indices_encapuslated.begin(); iterator !=  indices_encapuslated.end(); iterator++) {
				int world_index = iterator->first; 
				if (world_indices.count(world_index) == 1){
					continue;
				}
				world_indices[world_index] = world_index;
			}

		}

	return world_indices;
}

std::map<int, int> GridMap::bounding_box_to_world_indices(float x_center, float y_center, float width, float length){
	float lower_left_corner_x = (x_center - width/2.0);
	float lower_left_corner_y = (y_center - length/2.0);

	float upper_right_corner_x = (x_center + width/2.0);
	float upper_right_corner_y = (y_center + length/2.0);

	int lower_left_corner_index_i = (int) round( (lower_left_corner_x)/cell_x_width);
	int lower_left_corner_index_j = (int) round( (lower_left_corner_y)/cell_y_width);

	int upper_right_corner_index_i = (int) round( (upper_right_corner_x)/cell_x_width);
	int upper_right_corner_index_j = (int) round( (upper_right_corner_y)/cell_y_width);


	std::map<int, int> world_indices;
	for(int i = 0; i < (upper_right_corner_index_i - lower_left_corner_index_i + 1); i++){
		for(int j = 0; j < (upper_right_corner_index_j - lower_left_corner_index_j+1); j++){

			int i_x = lower_left_corner_index_i + i;
			int j_y = lower_left_corner_index_j + j;

			if (i_x < 0){
				i_x = 0;
			}
			if (j_y < 0){
				j_y = 0;
			}

			if (i_x > num_of_cells_width){
				i_x = (num_of_cells_width - 1);
			}
			if (j_y > num_of_cells_height){
				j_y = (num_of_cells_height - 1);
			}			
			int true_index = ij_index_to_true_index(i_x, j_y);
			if (world_indices.count(true_index) == 1){
				continue;
			}
			world_indices[true_index] = true_index;	

		}
	}
	return world_indices;
}




int GridMap::ij_index_to_true_index(int index_i, int index_j){
	int true_index = index_j*(num_of_cells_width) + index_i;//index_i*(NUM_CELL_HEIGHT) + index_j;
	return true_index;
}

void GridMap::CBA_publish(){
	// look at all cell type occupancies.
	// assume everything is free.
	// Assign robots, humans, and obstacle types.

	cba_msgs::CBA_NavInfo nav_info_msg;
	nav_info_msg.header.stamp = ros::Time();
	nav_info_msg.width = num_of_cells_width;
	nav_info_msg.height = num_of_cells_height;

	// Assign Free Cells first
	for(size_t i = 0; i < data.size(); i++){
		nav_info_msg.cell_occupancy_type.push_back(FREE_CELL);
	}

	// Assign ROBOT Cells:
	for(size_t i = 0; i < index_of_robot_occ_cells.size(); i++){
		int index = index_of_robot_occ_cells[i];
		nav_info_msg.cell_occupancy_type[index] = ROBOT_OCCUPIED;
	}

	// Assign Human Cells:
	typedef std::map<int, float>::iterator it_type;
	for(it_type iterator = map_index_of_human_cells_to_prob.begin(); iterator !=  map_index_of_human_cells_to_prob.end(); iterator++) {
	    int index = iterator->first;
		nav_info_msg.cell_occupancy_type[index] = HUMAN_OCCUPIED;
	}


	// Assign Obstacle types:
	for(size_t i = 0; i < index_of_obstacle_occ_cells.size(); i++){
		int index = index_of_obstacle_occ_cells[i];
		nav_info_msg.cell_occupancy_type[index] = OBSTACLE_OCCUPIED;
	}	
/*
	// Assign Updated Occupancy Based on Sensor Readings
	// Assign updated Free cell types:
	for(size_t i = 0; i < index_of_updated_free_occ_cells.size(); i++){
		int index = index_of_updated_free_occ_cells[i];
		nav_info_msg.cell_occupancy_type[index] = OBSTACLE_OCCUPIED;
		std::cout << nav_info_msg.cell_occupancy_type[index] << std::endl;
	}	

	// Assign updated Obstacle types:
	for(size_t i = 0; i < index_of_updated_obstacle_occ_cells.size(); i++){
		int index = index_of_updated_obstacle_occ_cells[i];
		nav_info_msg.cell_occupancy_type[index] = OBSTACLE_OCCUPIED;
	}
*/
	if (robot_envFeatures.size() > 0){
		nav_info_msg.state_type.push_back(robot_envFeatures[1].primary_type); // 1
		nav_info_msg.state_type.push_back(robot_envFeatures[2].primary_type); // 2
		nav_info_msg.state_type.push_back(robot_envFeatures[5].primary_type); // 3	
		nav_info_msg.state_type.push_back(robot_envFeatures[8].primary_type); // 4
		nav_info_msg.state_type.push_back(robot_envFeatures[7].primary_type); // 5
		nav_info_msg.state_type.push_back(robot_envFeatures[6].primary_type); // 6	
		nav_info_msg.state_type.push_back(robot_envFeatures[3].primary_type); // 7	
		nav_info_msg.state_type.push_back(robot_envFeatures[0].primary_type); // 8	

		nav_info_msg.state_distance.push_back(robot_envFeatures[1].min_distance_to_center); // 1
		nav_info_msg.state_distance.push_back(robot_envFeatures[2].min_distance_to_center); // 2
		nav_info_msg.state_distance.push_back(robot_envFeatures[5].min_distance_to_center); // 3	
		nav_info_msg.state_distance.push_back(robot_envFeatures[8].min_distance_to_center); // 4
		nav_info_msg.state_distance.push_back(robot_envFeatures[7].min_distance_to_center); // 5
		nav_info_msg.state_distance.push_back(robot_envFeatures[6].min_distance_to_center); // 6	
		nav_info_msg.state_distance.push_back(robot_envFeatures[3].min_distance_to_center); // 7	
		nav_info_msg.state_distance.push_back(robot_envFeatures[0].min_distance_to_center); // 8	

		for(size_t i = 0; i < index_of_robot_occ_cells.size(); i++){
			int index = index_of_robot_occ_cells[i];
			nav_info_msg.robot_global_cell_indices.push_back(index);
		}

		if (map_index_of_human_cells_to_prob.size() > 0){
			// Find nearest human cell:
			float min_distance_to_human = 100000.0;
			float unit_x = 0;
			float unit_y = 0;
			typedef std::map<int, float>::iterator it_type;
			for(it_type iterator = map_index_of_human_cells_to_prob.begin(); iterator !=  map_index_of_human_cells_to_prob.end(); iterator++) {
				int cell_index = iterator->first; 

				int x_index = data[cell_index].grid_x_index;
				int y_index = data[cell_index].grid_y_index;

				float cell_x_center = (cell_x_width/2.0) + cell_x_width*x_index;// + robot_world_x_pos - (CELL_FEATURE_SIZE*1.5);
				float cell_y_center = (cell_y_width/2.0) + cell_y_width*y_index;// + robot_world_y_pos - (CELL_FEATURE_SIZE*1.5);

				// **Store each cell's distance to center
				float x_dir = cell_x_center - robot_world_x_pos;
				float y_dir = cell_y_center - robot_world_y_pos;
				float distance_to_center = sqrt(pow((x_dir), 2) + pow((robot_world_y_pos - cell_y_center), 2)); 

				if(distance_to_center < min_distance_to_human){
					min_distance_to_human = distance_to_center;
					unit_x = (x_dir / min_distance_to_human);
					unit_y = (y_dir / min_distance_to_human);

/*					std::cout << "unit_x = " << unit_x << std::endl;					
					std::cout << "unit_y = " << unit_y << std::endl;
					std::cout << "equals1? = " << sqrt(pow(unit_x, 2) + pow(unit_y, 2)) << std::endl;																				*/
				}
			}

			nav_info_msg.unit_x_to_human = unit_x;
			nav_info_msg.unit_y_to_human = unit_y;
			nav_info_msg.human_present = 1;			

		}else{
			nav_info_msg.human_present = 0;			
		}
	}
	CBA_grid_pub.publish(nav_info_msg);
}

void GridMap::publish_orig_proj_map(){
	//proj_map_pub.publish(proj_map_grid);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "grid");
  GridMap gridmap;

  gridmap.cell_array_pub = gridmap.node.advertise<visualization_msgs::MarkerArray>( "grid/cell_array_markers", 1 );
/*  gridmap.robot_cells_pub = gridmap.node.advertise<visualization_msgs::Marker>( "grid/robot_marker", 1 );
  gridmap.CBA_grid_pub = gridmap.node.advertise<cba_msgs::CBA_NavInfo>("/CBA_grid_occ_topic", 0);
*/ 
  gridmap.CBA_grid_pub = gridmap.node.advertise<cba_msgs::CBA_NavInfo>("/CBA_featureV", 0); 
  gridmap.renew_dyn_grid_pub=gridmap.node.advertise<nav_msgs::OccupancyGrid>("grid/renew_dyn_map_cells", 1);
  
  gridmap.human_marker_pub=gridmap.node.advertise<visualization_msgs::Marker>( "/human_3D", 0 );
  //gridmap.human_markerarray_pub=gridmap.node.advertise<visualization_msgs::MarkerArray>( "/humans_boxes_3D", 0 );


//  gridmap.human_cells_pub = gridmap.node.advertise<visualization_msgs::Marker>( "grid/human_cells", 1 );
  gridmap.trikey_state_sub = gridmap.node.subscribe<nav_msgs::Odometry>("/hsrb/odom", 10, boost::bind(&GridMap::joint_state_callback, &gridmap, _1));
  //gridmap.sensor_sub = gridmap.node.subscribe<sensor_msgs::Imu>("/hsrb/base_imu/data", 10, boost::bind(&GridMap::sensor_callback, &gridmap, _1));


  //gridmap.human_bounding_boxes_sub = gridmap.node.subscribe<visualization_msgs::MarkerArray>("/human_boxes_3D", 10, boost::bind(&GridMap::human_detection_callback, &gridmap, _1));
  // gridmap.detected_humans_number_sub = gridmap.node.subscribe<std_msgs::Int8>("/detection/number_of_detected_humans", 10, boost::bind(&GridMap::number_detected_callback, &gridmap, _1));
 // gridmap.camera_viz_pub = gridmap.node.advertise<visualization_msgs::Marker>( "grid/cam_visibility_cells", 1 );
 // gridmap.features_viz_pub = gridmap.node.advertise<visualization_msgs::MarkerArray> ("grid/feature_cells", 1);

  // //added by Mk
  gridmap.feature_map_pub= gridmap.node.advertise<nav_msgs::OccupancyGrid>("grid/feature_map_cells", 1);
  gridmap.static_obs_sub = gridmap.node.subscribe<nav_msgs::OccupancyGrid>("/static_obstacle_map_ref", 10, boost::bind(&GridMap::static_obs_ref_callback, &gridmap, _1));
  gridmap.dynamic_obs_sub = gridmap.node.subscribe<nav_msgs::OccupancyGrid>("/dynamic_obstacle_map_ref", 10, boost::bind(&GridMap::dynamic_obs_ref_callback, &gridmap, _1));
  gridmap.gridcell_sub=gridmap.node.subscribe<nav_msgs::GridCells>("/base_path_planner/inflated_static_obstacle_map", 10, boost::bind(&GridMap::static_gridcell_callback, &gridmap, _1));





  ros::Rate r(10); 
  int counter = 0;

  while (ros::ok())
  {
	ros::spinOnce();

	// if (gridmap.detected_human == false){

	// }

	// gridmap.broadcast_tf_world_to_map();
	// //gridmap.publish_orig_proj_map();
  	 gridmap.publish();
  	 gridmap.humanpublish();
	////gridmap.CBA_publish();
	//gridmap.calculate_robotEnvFeatures();
	// gridmap.visualize_robotEnvFeatures();
	r.sleep();
  }

  ros::spin();
  return 0;
}
	