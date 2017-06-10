#include <ros/ros.h>
#include <ros/package.h>



#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/Point.h>

#include <tmc_yolo2_ros/Detections.h>
#include <tmc_yolo2_ros/Detection.h>

#include <string>
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "tf/tf.h"
#include <tf/transform_datatypes.h>

#define CAMERA_PIXEL_WIDTH 640
#define CAMERA_PIXEL_HEIGHT 480
#define VOXEL_SIZE 0.01 //1cm voxels
#define CLUSTER_DIST_TOLERANCE 0.2
#define CLUSTER_BOXES_NAMESPACE "cluster_3D_boxes"
#define dobject_BOXES_NAMESPACE "dobject_3D_boxes"

#define KINECT_HEIGHT 1//0.7366 //m

#define CUTOFF_NUM 2 // Number of closest bounding boxes to consider

#define BOX_HEIGHT_TOL 0.1
#define BOX_WIDTH_TOL 0.1
#define BOX_DEPTH_TOL 0.1

#define BOX_HEIGHT_MAX_TOL 2.434
#define BOX_WIDTH_MAX_TOL 2.5
#define BOX_DEPTH_MAX_TOL 2.5

#define DEPTH_MAX 10.0

#define PURPLE_COLOR 45
#define GREEN_COLOR 0


#define	MAX_HEIGHT_TOL 2.434 //m Maximum height of person (8ft) to consider



class Cluster3D_BoundingBox{
public:
	float mean_x;
	float mean_y;
	float mean_z;

	float box_x_center; float box_y_center; float box_z_center;

	float x_min; float x_max;
	float y_min; float y_max;
	float z_min; float z_max;	

	pcl::PointIndices voxel_indices;
	int voxel_box_index;

	float box_x_size();
	float box_y_size();	
	float box_z_size();
	float distance_from_origin() const;

	Cluster3D_BoundingBox();
	Cluster3D_BoundingBox(const float _x_min, const float _x_max, 
						  const float _y_min, const float _y_max, 
						  const float _z_min, const float _z_max,
						  const float _mean_x, const float _mean_y, const float _mean_z,
						  const int _voxel_box_index, const pcl::PointIndices &_voxel_indices);
	~Cluster3D_BoundingBox();	

};
Cluster3D_BoundingBox::Cluster3D_BoundingBox(){}
Cluster3D_BoundingBox::~Cluster3D_BoundingBox(){}

Cluster3D_BoundingBox::Cluster3D_BoundingBox(const float _x_min, const float _x_max, 
						  const float _y_min, const float _y_max, 
						  const float _z_min, const float _z_max,
  						  const float _mean_x, const float _mean_y, const float _mean_z,
						  const int _voxel_box_index, const pcl::PointIndices &_voxel_indices):
			x_min(_x_min), x_max(_x_max),
			y_min(_y_min), y_max(_y_max),
			z_min(_z_min), z_max(_z_max),
			mean_x(_mean_x), mean_y(_mean_y), mean_z(_mean_z),
			voxel_box_index(_voxel_box_index){

	voxel_indices = _voxel_indices;

	box_x_center = (x_max + x_min)/2.0;
	box_y_center = (y_max + y_min)/2.0;
	box_z_center = (z_max + z_min)/2.0;

}

float Cluster3D_BoundingBox::box_x_size(){
	return std::abs(x_max - x_min);
}
float Cluster3D_BoundingBox::box_y_size(){
	return std::abs(y_max - y_min);
}
float Cluster3D_BoundingBox::box_z_size(){
	return std::abs(z_max - z_min);
}
float Cluster3D_BoundingBox::distance_from_origin() const{
	return sqrt(pow(box_x_center, 2) + pow(box_y_center, 2) + pow(box_z_center, 2));
}

struct Cluster3D_BoundingBox_distance_compare {
	bool operator() (const Cluster3D_BoundingBox &lhs, const Cluster3D_BoundingBox &rhs) const{ 
		float lhs_dist = lhs.distance_from_origin();
		float rhs_dist = rhs.distance_from_origin();		 
		return lhs_dist < rhs_dist;	
	}

} distance_compare_obj;


//std::sort (myvector.begin(), myvector.end(), distance_cost_compare_obj) 

class BoundingBox_Person_Desc
{
public:
  float tl_x;
  float tl_y;
  float width;
  float height;  

  BoundingBox_Person_Desc(float _tl_x, float _tl_y, 
  						  float _width, float _height); // Constructor
  ~BoundingBox_Person_Desc(); // Destructor
};
BoundingBox_Person_Desc::BoundingBox_Person_Desc(float _tl_x, float _tl_y, float _width, float _height):
							tl_x(_tl_x), 
							tl_y(_tl_y),  
							width(_width), 
							height(_height){}

BoundingBox_Person_Desc::~BoundingBox_Person_Desc(){}

class Bounding_Box_dobject{
public:
	ros::NodeHandle 		  node;

	ros::Publisher 			  prism_voxels_pub;
	ros::Publisher 			  cluster_voxels_pub;	

	ros::Publisher            cluster_boxes_array_pub; //bounding box for candidate dobjects
	ros::Publisher 			  dobject_boxes_array_pub; //bounding box for extracted dobject
	ros::Publisher 			  human_boxes_array_pub; //bounding box for extracted dobject
	ros::Publisher 			  human_box_pub;

	ros::Publisher 			  dobject_points_pub; // publish point cloud of dobject points;
	ros::Publisher 			  voxelized_pub; // publish point cloud of dobject points;	
	ros::Publisher 			  number_of_detected_dobjects_pub; //give number of detected dobjects



	ros::Publisher 			  People_coordinate_pub;		//MK
	ros::Publisher            human_track_cmd_pub;
	tf::TransformListener 	  listener;
	tf::StampedTransform 	  transform_base_head;			//MK

	ros::Subscriber			  detectedObjects_sub;
	ros::Subscriber			  yolo_detectedObjects_sub;	
	ros::Subscriber           registered_cloud_sub;

	std::vector<BoundingBox_Person_Desc> boxes;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;

	std::vector< pcl::PointCloud<pcl::PointXYZRGB> > bounding_box_points;
	std::vector< pcl::PointCloud<pcl::PointXYZ> > voxelized_bounding_box_points;	


	std::vector< std::vector<pcl::PointIndices> > bounding_boxes_clusters; // each element is a bounding box which contains a vector of cluster indices
	std::vector< std::vector<Cluster3D_BoundingBox> > bounding_box_clusters_3D; // each element is a bounding box which contains a vector of 3d boxes for the cluster

	std::vector<Cluster3D_BoundingBox> dobject_3D_boxes;


	visualization_msgs::MarkerArray cluster_boxes_array; 
	visualization_msgs::MarkerArray dobject_boxes_array;
	
	///////////////      mk   //////////////////
	visualization_msgs::MarkerArray human_boxes_array;
	std::vector< std::vector<double> > box_pos_vec;
	int  box_pos_size;
	void publish_human_boxes();
	std::vector< std::vector<pcl::PointIndices> > bounding_boxes_humans; // each element is a bounding box which contains a vector of cluster indices
	std::vector< std::vector<Cluster3D_BoundingBox> > bounding_box_humans_3D; // each element is a bounding box which contains a vector of 3d boxes for the cluster
	/////////////////////////////////////////////



    std::map<int, std::vector<int> > colormap;

  	void voxelize_points();
	void extract_clusters();

	void extract_candidate_dobject_boxes();

	void extract_dobject_boxes();

	void publish_human_boxesarray();
	void publish_dn_points();
	void publish_clusters();
	void publish_clusters_boxes();
	void publish_dobject_3D_boxes();
	void publish_dobject_points();	

	void delete_previous_markers();
	void making_human_bounding_boxes();

	visualization_msgs::Marker make_cell_list_marker(int occupancy_type);

	visualization_msgs::Marker createBoundingBoxMarker(const std::string &array_namespace,
													   const Cluster3D_BoundingBox &box, 
												  	   const int &marker_index, const int &apply_color);


	Bounding_Box_dobject(); // Constructor
	~Bounding_Box_dobject();  // Destructor

private:
  	bool init_vars();

};

Bounding_Box_dobject::Bounding_Box_dobject(): cloud(new pcl::PointCloud<pcl::PointXYZRGB>){
	init_vars();
}

bool Bounding_Box_dobject::init_vars(){

    std::string path = ros::package::getPath("villa_3d_object_extract");
    std::string filename = path + "/src/colors.txt";
    std::cout << filename << std::endl;
    FILE *fp = fopen(filename.c_str(),"rb"); 
    if(fp==NULL) {
        printf("Error trying to read file.\n");
    }

    size_t buffer_size = 80;
    char cmd[buffer_size]; // Character buffer
    int total_color_nums;

    // Get total number of color_nums
    int success = fscanf(fp, "%d", &total_color_nums); // This integer will be on the first line

    for (int i = 0; i < total_color_nums; i++){
      int color_num; int r; int g; int b;
      std::vector<int> color;
      int line_read_success = fscanf(fp, "%d %d %d %d", &color_num, &r, &g, &b);  
      color.push_back(r); color.push_back(g); color.push_back(b); 
      colormap[color_num] = color;

      printf("color_num: %d r: %d g: %d b: %d \n", color_num, r, g, b);

    }

    if (ferror(fp) != 0 || fclose(fp) != 0){
        return false;
    }

    printf(".....Successfully Loaded Colormap Parameters.\n");
    return true; 

	box_pos_size=10;
    box_pos_vec.resize(box_pos_size);
    for(int i(0);i <box_pos_size;i++)
    	box_pos_vec[i].resize(3);

}


Bounding_Box_dobject::~Bounding_Box_dobject(){}

void Bounding_Box_dobject::making_human_bounding_boxes(){
	//std::vector<tmc_yolo2_ros::Detection> objects;
	//	objects = msg->detections;	
	ROS_INFO("Hello world! I detected something");

	int Num_human_objects =3;

	std::cout << "Number of detected objects: " << Num_human_objects << std::endl;


    boxes.clear();		//human boxes


    for(size_t i = 0; i < Num_human_objects; i++){

    int tl_x = 2+2*i;
    int tl_y = 2;
    int width_bound = 0.5;
    int height_bound = 0.5;

    std::string person_str ("person");
	boxes.push_back(BoundingBox_Person_Desc( tl_x,tl_y,  width_bound, height_bound));
        
    // if(person_str.compare(objects[i].class_name) == 0)
    // {	
    // 	ROS_INFO("I detected person : idx : %d", i);
    // 	boxes.push_back(BoundingBox_Person_Desc( tl_x, tl_y,  width_bound,  height_bound));
   	// }
   	// if(person_str.compare(objects[i].class_name) != 0)
    // {	
    // 	// ROS_INFO("I detected but it is not person : idx : %d", i);
    // 	// boxes.push_back(BoundingBox_Person_Desc( tl_x, tl_y,  width_bound,  height_bound));
   	// }
        
    }
    
    // std::cout << "Number of objects detected: " << boxes.size() <<std::endl;
    // std_msgs::Int8 num_msg;
    // if(dobject_boxes_array.markers.size() >= boxes.size() ){;
    //     num_msg.data = boxes.size();
    //     number_of_detected_dobjects_pub.publish(num_msg);
    // }       
}


visualization_msgs::Marker Bounding_Box_dobject::make_cell_list_marker(int occupancy_type){
	
	double cell_x_width=0.5;
	double cell_y_width=0.5;

	visualization_msgs::Marker marker;
	marker.header.frame_id = "/map";
	marker.header.stamp = ros::Time();
	marker.ns = "human_cells";
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
	marker.scale.z = 0.3;
	marker.color.a = 0.7; // Don't forget to set the alpha!

	marker.color.r = 0.3; // Light Gray color
	marker.color.g = 0.3;
	marker.color.b = 0.4;

	if (occupancy_type == 0){
		marker.color.r = 0.0; // BLUE color
		marker.color.g = 0.0;
		marker.color.b = 0.8;		
	}else if (occupancy_type == 3){		//human
		marker.color.r = 0.8; // Purple color
		marker.color.g = 0.0;
		marker.color.b = 0.8;				
	}else if(occupancy_type == 2){	//obstacle
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



visualization_msgs::Marker Bounding_Box_dobject::createBoundingBoxMarker(const std::string &array_namespace,
																	   const Cluster3D_BoundingBox &box, 
																  	   const int &marker_index, const int &apply_color){
//     visualization_msgs::Marker marker;
//     // Set the frame ID and timestamp.  See the TF tutorials for information on these.
//     marker.header.frame_id = (*cloud).header.frame_id; 
//     marker.header.stamp = ros::Time::now();

//     // Set the namespace and id for this marker.  This serves to create a unique ID
//     // Any marker sent with the same namespace and id will overwrite the old one
//     marker.ns = array_namespace;
//     marker.id = marker_index;

//     // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
//     marker.type = visualization_msgs::Marker::CUBE;

//     // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
//     marker.action = visualization_msgs::Marker::ADD;

//     // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header

//     marker.pose.position.x = box.box_x_center;//(box.x_max + box.x_min)/2.0;
//     marker.pose.position.y = box.box_y_center;//(box.y_max + box.y_min)/2.0;
//     marker.pose.position.z = box.box_z_center;//(box.z_max + box.z_min)/2.0;

//     //Mk for publish 
//     geometry_msgs::Vector3Stamped gV, tV;

//     gV.vector.x = box.box_x_center;
//     gV.vector.y = box.box_x_center;
//     gV.vector.z = box.box_x_center;
//     gV.header.stamp = ros::Time();
//     gV.header.frame_id = "/head_rgbd_sensor_rgb_frame";
//     listener.transformVector(std::string("/base_link"), gV, tV);

//     ROS_INFO("Box center x : %.3lf, y : %.3lf, z : %.3lf", gV.vector.x,gV.vector.y,gV.vector.z);
//  //     if(marker_index<5){
// //        ROS_INFO("index : %d \n", marker_index);
// 	//      box_pos_vec[marker_index][0] = gV.vector.x;
// 	//      box_pos_vec[marker_index][1] = gV.vector.y;
// 	//      box_pos_vec[marker_index][2] = gV.vector.z;

// 	// }

//      //human
//     geometry_msgs::Point CenterPoint;
//     tf::Vector3 originV=transform_base_head.getOrigin();
// 	CenterPoint.x=tV.vector.x+originV.x();
// 	CenterPoint.y=tV.vector.y+originV.y();
// 	CenterPoint.z=tV.vector.z+originV.z();

// 	//CenterPoint.


// 	ROS_INFO("Changed frame vector  x : %.3lf, y : %.3lf, z : %.3lf", tV.vector.x,tV.vector.y,tV.vector.z);
// 	ROS_INFO("Frame translation vector x : %.3lf, y : %.3lf, z : %.3lf", originV.x(),originV.y(),originV.z());

// 	People_coordinate_pub.publish(CenterPoint);



// 	// visualization_msgs::Marker marker_human;
// 	// marker_human.header.frame_id = (*cloud).header.frame_id; 
//  //    marker_human.header.stamp = ros::Time::now();
//  //    marker_human.ns = array_namespace;
//  //    marker_human.id = 0;

//  //    marker_human.pose.orientation.x = 0.0;
//  //    marker_human.pose.orientation.y = 0.0;
//  //    marker_human.pose.orientation.z = 0.0;
//  //    marker_human.pose.orientation.w = 1.0;

//  //    double temp_dist,temp_dist2,temp_dist3;
//  //    temp_dist  =box.x_max - box.x_min;
//  //    temp_dist2 =box.y_max - box.y_min;
//  //    temp_dist3 =box.z_max - box.z_min;

//  //    //ROS_INFO("temp dist : %.3lf, temp dist2 : %.3lf, temp dist3 : %.3lf",temp_dist,temp_dist2,temp_dist3);

//  //    marker_human.scale.x = std::abs(temp_dist);
//  //    marker_human.scale.y = std::abs(temp_dist2);
//  //    marker_human.scale.z = std::abs(temp_dist3);

//  //    marker_human.color.r = ((float) colormap[apply_color][0]) / 255.0;
//  //    marker_human.color.g = ((float) colormap[apply_color][1]) / 255.0;
//  //    marker_human.color.b = ((float) colormap[apply_color][2]) / 255.0;
//  //    marker_human.color.a = 0.25;
// 	// human_boxes_array_pub.publish(marker_human);


//     ////////////////////////////////////
//     marker.pose.orientation.x = 0.0;
//     marker.pose.orientation.y = 0.0;
//     marker.pose.orientation.z = 0.0;
//     marker.pose.orientation.w = 1.0;

//     // Set the scale of the marker -- 1x1x1 here means 1m on a side
//     marker.scale.x = std::abs(box.x_max - box.x_min);
//     marker.scale.y = std::abs(box.y_max - box.y_min);
//     marker.scale.z = std::abs(box.z_max - box.z_min);

//     // Set the color -- be sure to set alpha to something non-zero!
//     //int	apply_color = marker_index % colormap.size();

//     marker.color.r = ((float) colormap[apply_color][0]) / 255.0;
//     marker.color.g = ((float) colormap[apply_color][1]) / 255.0;
//     marker.color.b = ((float) colormap[apply_color][2]) / 255.0;
//     marker.color.a = 0.25;

//     marker.lifetime = ros::Duration(1.0);

//     return marker;
}

void Bounding_Box_dobject::extract_dobject_boxes(){
	dobject_3D_boxes.clear();
	// Returns the closest cluster found


	for(size_t i = 0; i < bounding_box_clusters_3D.size() ; i++){
		std::vector<Cluster3D_BoundingBox> current_dn_bounding_box;	

		if (bounding_box_clusters_3D[i].size() > 0){
			// Copy Bounding boxes
			for(size_t j = 0; j < bounding_box_clusters_3D[i].size() ; j++){
				current_dn_bounding_box.push_back(bounding_box_clusters_3D[i][j]);
			}
			// Sort Bounnding Boxes from Closest to Furthest
			std::sort (current_dn_bounding_box.begin(), current_dn_bounding_box.end(), distance_compare_obj);

			int cluster_box_index_indicating_dobject = 0; // The closest
			
			// Ignore if the "person" detected is further than 4m
			if (current_dn_bounding_box[cluster_box_index_indicating_dobject].mean_z > DEPTH_MAX){
				continue;
			}
	
			// Select only the top 3 closest boxes
			//current_dn_bounding_box.resize(CUTOFF_NUM);
				
			// Filter for minimum box size and maximum box height
			std::vector<int> indices_to_consider;
			for(size_t k = 0; k < current_dn_bounding_box.size() ; k++){
				float cb_size_x = current_dn_bounding_box[k].box_x_size();
				float cb_size_y = current_dn_bounding_box[k].box_y_size();			
				float cb_size_z = current_dn_bounding_box[k].box_z_size();
				float cb_max_height =  current_dn_bounding_box[k].y_max;

		       if ( ((cb_size_x >= BOX_WIDTH_TOL) && 
		       	     (cb_size_y >= BOX_HEIGHT_TOL) && 
		       	     (cb_size_z >= BOX_DEPTH_TOL)  && 
		       	     (cb_size_x < BOX_WIDTH_MAX_TOL) &&
		       	     (cb_size_y < BOX_HEIGHT_MAX_TOL) &&
	   	       	     (cb_size_z < BOX_DEPTH_MAX_TOL) &&    	       	     
		       	     (std::abs(cb_max_height + KINECT_HEIGHT) < MAX_HEIGHT_TOL) ) ){
		       			indices_to_consider.push_back(k);
		       }

			}

			
			// If no box satisfies this requirement, claim that the closest cluster must be dobject

			if (indices_to_consider.size() > 0){
				// There must be competing boxes. The tallest box at this point should be dobject
				// Now only select the box with the highest 
				float tallest_box_so_far = std::abs(current_dn_bounding_box[0].y_max); // highest so far
				int best_box_index = 0;
				for(size_t b_i = 0; b_i < indices_to_consider.size(); b_i++){

		  			 std::cout << "INDEX: " << b_i << " " << "dx dy dz" << 
					 std::abs(current_dn_bounding_box[ indices_to_consider[b_i] ].x_max) << " " <<
					 std::abs(current_dn_bounding_box[ indices_to_consider[b_i] ].y_max) << " " <<
					 std::abs(current_dn_bounding_box[ indices_to_consider[b_i] ].z_max) << " " << std::endl;


					if (  std::abs(current_dn_bounding_box[ indices_to_consider[b_i] ].y_max) > tallest_box_so_far){
						best_box_index = indices_to_consider[b_i];
						tallest_box_so_far = std::abs(current_dn_bounding_box[b_i].y_max);
					}

				}
				cluster_box_index_indicating_dobject = best_box_index;
			}
			
			
			
			dobject_3D_boxes.push_back(current_dn_bounding_box[cluster_box_index_indicating_dobject]);
		}

	}

	//std::cout << "Number of 3D Box dobjects: " << dobject_3D_boxes.size() << std::endl;

}




// void publish_dobject_bounding boxes


void Bounding_Box_dobject::publish_dn_points(){

}

void Bounding_Box_dobject::publish_clusters(){



}

//
void Bounding_Box_dobject::publish_clusters_boxes(){

}

void Bounding_Box_dobject::publish_dobject_3D_boxes(){

}


//mk for human
void Bounding_Box_dobject::publish_human_boxes(){

	visualization_msgs::Marker marker_human;
	marker_human.header.frame_id = "/map"; 
    marker_human.header.stamp = ros::Time::now();
    marker_human.ns = "basic_shapes";
    marker_human.id = 0;

    uint32_t shape = visualization_msgs::Marker::CUBE;
    marker_human.type = shape;

    marker_human.pose.position.x = 5.9;
    marker_human.pose.position.y = 0.65;
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
	human_box_pub.publish(marker_human);


	//Delivery cmd for tracking human
	 // std_msgs::Int8 bool_track_msg;
	 // bool_track_msg.data=1;
	 // human_track_cmd_pub.publish(bool_track_msg);

}




void Bounding_Box_dobject::publish_human_boxesarray(){

	visualization_msgs::Marker map_free_cell_list = make_cell_list_marker(2);
		
	int data_num_size=5;
	double cell_x_width=0.5;
	double cell_y_width=0.5;

	for(size_t i = 0; i < 3; i++){
		geometry_msgs::Point cell_loc;

		int x_index = 2+i;
		int y_index = 3+i;
		cell_loc.x = (cell_x_width/2.0) + cell_x_width*x_index;
		cell_loc.y = (cell_y_width/2.0) + cell_y_width*y_index;
		cell_loc.z = -0.1;

		map_free_cell_list.points.push_back(cell_loc);

	}
	human_boxes_array.markers.push_back(map_free_cell_list);


	human_boxes_array_pub.publish(human_boxes_array);
	
}

void Bounding_Box_dobject::delete_previous_markers(){

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "villa_3D_object_bounding_boxes_mk");
	Bounding_Box_dobject bbh_obj;

  	// Create Subsribers
	bbh_obj.cluster_boxes_array_pub = bbh_obj.node.advertise<visualization_msgs::MarkerArray>( "/cluster_boxes", 0 );
	bbh_obj.dobject_boxes_array_pub = bbh_obj.node.advertise<visualization_msgs::MarkerArray>( "/dobject_boxes_3D", 0 );
	bbh_obj.People_coordinate_pub= bbh_obj.node.advertise<geometry_msgs::Point>( "/People_3Dcoordinate", 0 );
	//mk
	bbh_obj.human_track_cmd_pub=bbh_obj.node.advertise<std_msgs::Int8>("/Int_cmd_trackhuman", 0);
	bbh_obj.human_box_pub=bbh_obj.node.advertise<visualization_msgs::Marker>( "/human_target_2", 0 );
	bbh_obj.human_boxes_array_pub = bbh_obj.node.advertise<visualization_msgs::MarkerArray>( "/human_boxes_3D", 0 );
	// bbh_obj.dobject_points_pub = bbh_obj.node.advertise<pcl::PointCloud<pcl::PointXYZRGB> >( "/dobject_voxels", 0 );
	bbh_obj.number_of_detected_dobjects_pub = bbh_obj.node.advertise<std_msgs::Int8>("detection/number_of_detected_dobjects", 0);

	try{
      bbh_obj.listener.lookupTransform("/base_link", "/head_rgbd_sensor_rgb_frame",ros::Time(0), bbh_obj.transform_base_head);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      // continue;
    }

  ros::Rate loop_rate(20); 	

  while (ros::ok())
  {
	   
     bbh_obj.publish_human_boxes();
     bbh_obj.publish_human_boxesarray();
  
   	 ros::spinOnce();
     loop_rate.sleep();  
  }


  ros::spin();
  return 0;
}
