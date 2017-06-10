#include "ros/ros.h"
#include "rosbag/bag.h"

#include <rosbag/view.h>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include "nav_msgs/OccupancyGrid.h"

ros::Publisher proj_map_pub;

bool recorded = false;
std::string new_topic_name = "grid/stored_proj_map";

void proj_map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
	std::cout << "Message received!" << std::endl;
	std::cout << "Map Width" << msg->info.width << std::endl;
	std::cout << "Map Height" << msg->info.height << std::endl;

	nav_msgs::OccupancyGrid	remap_msg_topic;
	remap_msg_topic = *msg;	

	if (recorded == false){
	    rosbag::Bag bag;

		std::cout << "Saving topic as " << new_topic_name << " ... " << std::endl;

	    bag.open("remaped_grid_hallway.bag", rosbag::bagmode::Write);

	    bag.write(new_topic_name, ros::Time::now(), remap_msg_topic);
	    bag.close();

	 	std::cout << "Done recording topic as " << new_topic_name << std::endl;
	    recorded = true;
	 }else{
	 	std::cout << "Already recorded." << std::endl;
/*
	    rosbag::Bag bag;

	    bag.open("remaped_grid_hallway.bag", rosbag::bagmode::Read);

		std::vector<std::string> topics;
		topics.push_back(std::string("grid/stored_proj_map"));

		rosbag::View view(bag, rosbag::TopicQuery(topics));

		foreach(rosbag::MessageInstance const m, view)
		{
			nav_msgs::OccupancyGrid::ConstPtr s = m.instantiate<nav_msgs::OccupancyGrid>();
			std::cout << s->info.width << std::endl;
		}
	    bag.close();		
*/
	 }


}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "proj_map_saver");
	ros::NodeHandle node;

	ros::Subscriber proj_map_sub = node.subscribe<nav_msgs::OccupancyGrid>("rtabmap/proj_map", 1, proj_map_callback);
	proj_map_pub = node.advertise<nav_msgs::OccupancyGrid>(new_topic_name, 1);	


	ros::Rate r(0.1); 
	while (ros::ok()){
		ros::spinOnce();
		r.sleep();
	}



	return 0;
}
	