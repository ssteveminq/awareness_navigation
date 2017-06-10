#include "ros/ros.h"
#include <ros/package.h>
#include "rosbag/bag.h"
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include "nav_msgs/OccupancyGrid.h"

ros::Publisher proj_map_pub;
std::string read_topic_name = "grid/stored_proj_map";

std::string new_topic_name = "grid/stored_proj_map";
nav_msgs::OccupancyGrid proj_map_grid;


std::string package_path = ros::package::getPath("grid");
std::string filename = package_path + "/grid_maps/ETC_Hallway_occupancy12-09-2016.bag";

void readBag(){
    rosbag::Bag bag;

    bag.open(filename, rosbag::bagmode::Read);

	std::vector<std::string> topics;
	topics.push_back(std::string(read_topic_name));

	rosbag::View view(bag, rosbag::TopicQuery(topics));

	foreach(rosbag::MessageInstance const m, view)
	{
		nav_msgs::OccupancyGrid::ConstPtr occ = m.instantiate<nav_msgs::OccupancyGrid>();
		std::cout << occ->info.width << std::endl;
		proj_map_grid = (*occ);
	}
	std::cout << "Finished reading bag" << std::endl;
    bag.close();		
}

void publish_map(){
	proj_map_pub.publish(proj_map_grid);
}

int main(int argc, char **argv)
{	
	std::cout << filename << std::endl;
	std::cout << "Opening: " << filename << std::endl;
	readBag();

	ros::init(argc, argv, "proj_map_reader");
	ros::NodeHandle node;

	proj_map_pub = node.advertise<nav_msgs::OccupancyGrid>(new_topic_name, 1);	

	ros::Rate r(0.1); 
	while (ros::ok()){
		publish_map();
		ros::spinOnce();
		r.sleep();
	}


	return 0;
}