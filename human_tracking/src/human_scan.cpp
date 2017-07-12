#include "human_scan.h"


Human_Belief_Scan::Human_Belief_Scan():index(0),m_numofhuman(0),m_receiveiter(0){
	Init_parameters();
}
Human_Belief_Scan::~Human_Belief_Scan(){}
Human_Belief_Scan::Human_Belief_Scan(int numofhuman)
{
	Init_parameters();
	m_numofhuman=numofhuman;
	if(m_numofhuman>0)
	{
		m_yolo_idx_set.resize(m_numofhuman,0);
		m_human_posx.resize(m_numofhuman,0.0);
		m_human_posy.resize(m_numofhuman,0.0);
	}
}

bool Human_Belief_Scan::FindHuman(human_tracking::peoplefinder::Request &req, human_tracking::peoplefinder::Response &res)
{

	geometry_msgs::PoseArray people_posearrays;
	res.People_array=people_posearrays;
	res.IsHuman=true;
	return res.IsHuman;
}


bool Human_Belief_Scan::Comparetwopoistions(std::vector<double> pos,std::vector<double> pos2)
{
	//return true if there are in criterion distance 
	double temp_dist=0.0;
	for(int i(0);i<2;i++)	
	{
		temp_dist+=pow((pos[i]-pos2[i]),2);
	}
	temp_dist=sqrt(temp_dist);

	if(temp_dist<Same_POS_diff)
		return true;
	

	return false;

}

bool Human_Belief_Scan::Comparetwopoistions(std::vector<double> pos,std::vector<double> pos2, double criterion)
{
	//return true if there are in criterion distance 

	double temp_dist=0.0;
	for(int i(0);i<2;i++)	
	{
		temp_dist+=pow((pos[i]-pos2[i]),2);
	}
	temp_dist=sqrt(temp_dist);

	if(temp_dist<criterion)
		return true;
	

	return false;
}

void Human_Belief_Scan::keyboard_callback(const keyboard::Key::ConstPtr& msg)
{

	ROS_INFO("Received Keyboard");
	std::cout<<msg->code<<std::endl;
	if(msg->code==116)		//if keyboard input is "t"
	{
		set_Current_Human_to_Target();
		send_activation_filter_cmd();
	}
}

void Human_Belief_Scan::send_activation_filter_cmd()
{

	std_msgs::Int8 activation_msg;
	activation_msg.data=1;

	filter_act_pub.publish(activation_msg);

	

}

void Human_Belief_Scan::set_Current_Human_to_Target()
{

	if(Cur_detected_human.size()>0)
	{
		Track_human_target[0]=Cur_detected_human[0][0];
		Track_human_target[1]=Cur_detected_human[0][1];
		OnceTargeted=true;
		action_mode=2;
		ROS_INFO("leg target set: %.3lf, y : %.3lf",Track_human_target[0],Track_human_target[1]);
	}


}

void Human_Belief_Scan::SetTarget_face_detection()
{
	
	if(Cur_detected_human.size()>0 && action_mode==1)
	{
		std::cout<<"SetTarget_face_detection"<<std::endl;
		std::vector<double> Distanceset;
		Distanceset.resize(Cur_leg_human.size(),0.0);
			
		double minDistance=200.0;
		int    minDistance_Idx=0;

		if(Cur_leg_human.size()>0){
			for(int i(0);i<Cur_leg_human.size();i++)
			{
				Distanceset[i]=getDistance_from_Vec(Cur_detected_human[0],Cur_leg_human[i][0],Cur_leg_human[i][1]);

				if(minDistance>Distanceset[i])
				{
					minDistance=Distanceset[i];
					minDistance_Idx=i;
				}
			}

			// Track_human_target[0]=Cur_leg_human[minDistance_Idx][0];
			// Track_human_target[1]=Cur_leg_human[minDistance_Idx][1];
			Track_human_target[0]=Cur_detected_human[0][0];
			Track_human_target[1]=Cur_detected_human[0][1];

			OnceTargeted=true;
			action_mode=2;
			ROS_INFO("leg target set: %.3lf, y : %.3lf",Track_human_target[0],Track_human_target[1]);

		}

	}

	 


}

void Human_Belief_Scan::SetTarget()
{
	//compare yolo detection and leg detector and survive only they are similar to each other
	//Cur_detected_human
	//Cur_existed_human
	m_leg_idx_set.clear();
	//human_occupied_leg_idx.clear();
	int num_yolo_detected_people= Cur_detected_human.size();
	int num_leg_detected_people= Cur_leg_human.size();

	if(num_leg_detected_people >0 && num_yolo_detected_people>0)
	{
		for(int i(0);i<num_yolo_detected_people;i++)
		{
			for(int j(0);j<num_leg_detected_people;j++)
			{
				if(Comparetwopoistions(Cur_detected_human[i],Cur_leg_human[j]))
					{
						//ROS_INFO("leg index : %d ", j);
						m_leg_idx_set.push_back(j);
					}
			}
		}

		Cur_leg_yolo_human.clear();
		for(int i(0);i<m_leg_idx_set.size();i++)
		{
			std::vector<double> temp_detected(2,0.0);
			for(int j(0);j<2;j++)
			{
				temp_detected[j]=Cur_leg_human[m_leg_idx_set[i]][j];
			
			}
			Cur_leg_yolo_human.push_back(temp_detected);

			// int human_leg_mapidx=CoordinateTransform_Global2_staticMap(temp_detected[0],temp_detected[1]);
			// human_occupied_leg_idx.push_back(human_leg_mapidx);
		}
	}

}
double Human_Belief_Scan::getDistance(std::vector<double> pos1,std::vector<double> pos2)
{



}


int Human_Belief_Scan::FindNearesetLegIdx()
{
		//target should be already set
		std::vector<double> Distanceset;
		// Distanceset.resize(Cur_detected_human.size(),0.0);
		Distanceset.resize(Cur_leg_human.size(),0.0);
		
		double minDistance=200.0;
		int    minDistance_Idx=0;

		if(OnceTargeted){
			for(int i(0);i<Cur_leg_human.size();i++)
			{
				// Distanceset[i]=getDistance(Cur_detected_human[i][0],Cur_detected_human[i][1]);
				Distanceset[i]=getDistance_from_Vec(Track_human_target,Cur_leg_human[i][0],Cur_leg_human[i][1]);
				
				if(minDistance>Distanceset[i])
					{
						minDistance=Distanceset[i];
						minDistance_Idx=i;
					}
			}


		}

	
		return minDistance_Idx;
}

void Human_Belief_Scan::face_detected_name_callback(const std_msgs::String::ConstPtr& msg)
{
	std::string face_name = msg->data;

	for(int i(0);i<Names_set.size();i++)
		{
			if(face_name.compare(Names_set[i])==0)
				{
					std::cout<<"Find the person : "<< face_name <<std::endl;
					if((action_mode==1) && (num_of_detected_human>0))
						{
							SetTarget_face_detection();
						}
				}
		}




}

void Human_Belief_Scan::edge_leg_callback(const geometry_msgs::PoseArray::ConstPtr& msg)
{
	int num_leg_detected = msg->poses.size();	
	// ROS_INFO("edge callback data size : %d",num_leg_detected);
	// human_occupied_leg_idx.clear();
	 if(m_leg_updateiter>10)
	 {
		std::vector<double> tempVec(2,0.0);
	
			human_occupied_leg_idx.clear();
			 Cur_leg_human.clear();
			// Cur_leg_human.resize(num_leg_detected);
			for(int i(0);i<num_leg_detected;i++)
			{
				geometry_msgs::Vector3Stamped gV, tV;

			    gV.vector.x = msg->poses[i].position.x;
			    gV.vector.y = msg->poses[i].position.y;
			    gV.vector.z = 1.0;

			    gV.header.stamp = ros::Time();
			    gV.header.frame_id = "/base_range_sensor_link";
			    listener.transformVector("/map", gV, tV);

			    tempVec[0]=tV.vector.x+global_pose[0];
			    tempVec[1]=tV.vector.y+global_pose[1];

			    //distance check between two candidates!
			    bool IsFarEnough = true;
			    bool IsNearfromRobot= false;
			    for(int j(0);j<Cur_leg_human.size();j++)
			    	{
			    		if(Comparetwopoistions(tempVec,Cur_leg_human[j],0.65))
			    			IsFarEnough=false;
			    	}
			   	
			   	if(Comparetwopoistions(global_pose,tempVec,LASER_Dist_person))
			    		IsNearfromRobot=true;


			    //add only if candidate pose ins far from 0.75 from previous candidates
			    if(IsFarEnough && IsNearfromRobot)
				{
					Cur_leg_human.push_back(tempVec);
					
					int human_leg_mapidx=CoordinateTransform_Global2_staticMap(tempVec[0],tempVec[1]);
					human_occupied_leg_idx.push_back(human_leg_mapidx);
				}
			}

		
			// OnceTargeted=true;

		if(action_mode==2)	//following mode - tracking laser -target should be already set
		{
			int NearestLegIdx=FindNearesetLegIdx();
				
			std::vector<double> temp_leg_target(2,0.0);	
			temp_leg_target[0]=Cur_leg_human[NearestLegIdx][0];
			temp_leg_target[1]=Cur_leg_human[NearestLegIdx][1];

			if(Comparetwopoistions(temp_leg_target,Track_human_target,0.75))
			{
					// std::cout<<"update target - person is in a range"<<std::endl;
					Track_human_target[0]=temp_leg_target[0];
					Track_human_target[1]=temp_leg_target[1];
			}

		}
		
		m_leg_updateiter=0;
	}
		

	m_leg_updateiter++;
}

void Human_Belief_Scan::Publish_human_boxes()
{
	human_boxes_array.markers.clear();
	if(Founded_human.size()>0)
	{
		for(int i(0);i<Founded_human.size();i++)
		{
			visualization_msgs::Marker marker_human;
			marker_human.header.frame_id = "/map"; 
		    marker_human.header.stamp = ros::Time::now();
		    marker_human.ns = "Founded_human";
		    marker_human.id = i;

		    uint32_t shape = visualization_msgs::Marker::SPHERE;
		    marker_human.type = shape;

		    marker_human.pose.position.x = Founded_human[i][0];
		    marker_human.pose.position.y = Founded_human[i][1];
		    marker_human.pose.position.z = 1;

		    marker_human.pose.orientation.x = 0.0;
		    marker_human.pose.orientation.y = 0.0;
		    marker_human.pose.orientation.z = 0.0;
		    marker_human.pose.orientation.w = 1.0;

		    double temp_dist,temp_dist2,temp_dist3;
		    temp_dist  =0.5;
		    temp_dist2 =0.5;
		    temp_dist3 =0.5;

		    //ROS_INFO("temp dist : %.3lf, temp dist2 : %.3lf, temp dist3 : %.3lf",temp_dist,temp_dist2,temp_dist3);
		    marker_human.scale.x = std::abs(temp_dist);
		    marker_human.scale.y = std::abs(temp_dist2);
		    marker_human.scale.z = std::abs(temp_dist3);

		    marker_human.color.r = 0.3;
		    marker_human.color.g = 0.6;
		    marker_human.color.b = 1.0;
		    marker_human.color.a = 0.85;

		    human_boxes_array.markers.push_back(marker_human);
	    }

	    founded_human_pub.publish(human_boxes_array);

	}


human_leg_boxes_array.markers.clear();
	if(Cur_leg_human.size()>0)
	{
		for(int i(0);i<Cur_leg_human.size();i++)
		{
			visualization_msgs::Marker marker_human_leg;
			marker_human_leg.header.frame_id = "/map"; 
		    marker_human_leg.header.stamp = ros::Time::now();
		    marker_human_leg.ns = "/human_leg_boxes";
		    marker_human_leg.id = i;

		    uint32_t shape = visualization_msgs::Marker::SPHERE;
		    marker_human_leg.type = shape;

		    marker_human_leg.pose.position.x = Cur_leg_human[i][0];
		    marker_human_leg.pose.position.y = Cur_leg_human[i][1];
		    marker_human_leg.pose.position.z = 1;

		    marker_human_leg.pose.orientation.x = 0.0;
		    marker_human_leg.pose.orientation.y = 0.0;
		    marker_human_leg.pose.orientation.z = 0.0;
		    marker_human_leg.pose.orientation.w = 1.0;

		    double temp_dist,temp_dist2,temp_dist3;
		    temp_dist  =0.5;
		    temp_dist2 =0.5;
		    temp_dist3 =0.5;

		    //ROS_INFO("temp dist : %.3lf, temp dist2 : %.3lf, temp dist3 : %.3lf",temp_dist,temp_dist2,temp_dist3);
		    marker_human_leg.scale.x = std::abs(temp_dist);
		    marker_human_leg.scale.y = std::abs(temp_dist2);
		    marker_human_leg.scale.z = std::abs(temp_dist3);

		    marker_human_leg.color.r = 0.83;
		    marker_human_leg.color.g = 0.6;
		    marker_human_leg.color.b = 0.5;
		    marker_human_leg.color.a = 0.85;

		    human_leg_boxes_array.markers.push_back(marker_human_leg);
	    }

	    Human_boxes_pub.publish(human_leg_boxes_array);
	}




}


void Human_Belief_Scan::laser_scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{

	// int datasize = msg->ranges.size();
	// int datasize_intensity = msg->intensities.size();
	// // std::cout<<"laser data size "<<datasize<<std::endl;	

	// sensor_msgs::LaserScan new_laser_msg;
	// new_laser_msg.header.stamp=ros::Time::now();
	// new_laser_msg.header.frame_id= "base_range_sensor_link"; 

	// new_laser_msg.angle_min       = msg->angle_min;      
	// new_laser_msg.angle_max       = msg->angle_max;
	// new_laser_msg.angle_increment = msg->angle_increment;
	// new_laser_msg.time_increment  = msg->time_increment;
	// new_laser_msg.scan_time       = msg->scan_time;
	// new_laser_msg.range_min       = msg->range_min;
	// new_laser_msg.range_max       = msg->range_max;
	
	// new_laser_msg.ranges.resize(datasize);
	// new_laser_msg.intensities.resize(datasize_intensity,0.0);
	
	// double criterion_angle = 0.0;
	// double margin =80;

	// if(OnceTargeted)
	// {
		
	// 	criterion_angle=angle_people_set[0];
	// 	margin=80;
	// 	// criterion_angle=Robot_Pos[2]+Head_Pos[0];;
	// }
		

	// // std::cout<<criterion_angle<<std::endl;		
	
	// int angle_min_idx=ConvertAngle2LaserIdx(criterion_angle-margin*MATH_PI/180);
	// int angle_max_idx=ConvertAngle2LaserIdx(criterion_angle+margin*MATH_PI/180);


	// // std::cout<<"angle min idx :"<<angle_min_idx<<", angle max idx : "<<angle_max_idx<<std::endl;
	// for(int i(angle_min_idx);i<angle_max_idx;i++)
	// {
	// 		new_laser_msg.ranges[i]=msg->ranges[i];
	// 		//new_laser_msg.intensities[i]=msg->intensities[i];
	// }


	// human_laser_scan_pub.publish(new_laser_msg);


}


void Human_Belief_Scan::laser_pcl_callback(const sensor_msgs::PointCloud2 ::ConstPtr& msg)
{


	// pcl::fromROSMsg(*msg, *cloud);
	
	int laser_pcl_size=msg->data.size();

	sensor_msgs::PointCloud2 pcl2_msg;
	// pcl2_msg.header=msg->header;
	pcl2_msg.header.stamp=ros::Time::now();
	pcl2_msg.header.frame_id= "base_range_sensor_link"; 

	pcl2_msg.fields = msg->fields;
	pcl2_msg.width=msg->width;
	pcl2_msg.height=msg->height;
	pcl2_msg.is_bigendian=msg->is_bigendian;
	pcl2_msg.row_step=msg->row_step;
	pcl2_msg.point_step=msg->point_step;
	pcl2_msg.is_dense=msg->is_dense;
	pcl2_msg.data.resize(laser_pcl_size,0.0);



	double criterion_angle = 0.0;

	if(angle_people_set.size()>0)
		criterion_angle=angle_people_set[0];		
	

	double min_angle=criterion_angle-15*MATH_PI/180;
	double max_angle =criterion_angle+15*MATH_PI/180;

	if(min_angle<LASER_ANGLE_MIN);
		min_angle=LASER_ANGLE_MIN;
	if(min_angle>LASER_ANGLE_MAX);
		max_angle=LASER_ANGLE_MAX;


	int angle_min_idx=ConvertAngle2LaserIdx(min_angle);
	int angle_max_idx=ConvertAngle2LaserIdx(max_angle);

	for(int i(angle_min_idx);i<angle_max_idx;i++)
	{
			pcl2_msg.data[i]= msg->data[i];
	}


	human_laser_pub.publish(pcl2_msg);

}

int Human_Belief_Scan::ConvertAngle2LaserIdx(double angle_rad)
{

	int res = round((angle_rad-LASER_ANGLE_MIN)/LASER_ANGLE_STEP);

	return res;
}


void Human_Belief_Scan::dyn_map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	//update dynamic map info
	dynamic_belief_map.info.width  = msg->info.width;
	dynamic_belief_map.info.height = msg->info.height;
	dynamic_belief_map.info.resolution = msg->info.resolution;
	dynamic_belief_map.info.origin.position.x = msg->info.origin.position.x;
	dynamic_belief_map.info.origin.position.y =msg->info.origin.position.y;
	dynamic_belief_map.data.resize(dynamic_belief_map.info.width*dynamic_belief_map.info.width);
	
	//Initialize for dyn_map_occupancy
	if(m_receiveiter==0)
	{
		int dyn_map_xsize=	msg->info.width;
		int dyn_map_ysize= msg->info.height;
		int dyn_map_resolution = msg->info.resolution;
	}
	else{

	}
	 
	 m_dyn_occupancy.resize(msg->data.size());
	 for(int i(0);i<msg->data.size();i++)
	 {
	 	m_dyn_occupancy[i]=msg->data[i];
	 }
	  

	 // //Publish dynamic_belief_map
	 // dynamic_belief_map.header.stamp =  ros::Time::now();
	 // dynamic_belief_map.header.frame_id = "map"; 
  //    belief_pub.publish(dynamic_belief_map);

}


void Human_Belief_Scan::number_detected_callback(const std_msgs::Int8::ConstPtr &msg){
	num_of_detected_human_yolo =  (int) msg->data;
	// std::cout << "Number of detected humans" << number << std::endl;
	if (num_of_detected_human_yolo == 0){
		// std::cout << "no detection" << std::endl;
		//track_cmd.data=0;
		//update_human_occ_belief(0);
	}
	else
	{

		// track_cmd=1;
	}
}


int Human_Belief_Scan::CoordinateTransform_Global2_staticMap(double global_x, double global_y)
{
	double reference_origin_x=static_belief_map.info.origin.position.x;
	double reference_origin_y=static_belief_map.info.origin.position.y;

	//Find the coordinate w.r.t map origin
	double  temp_x  = global_x - reference_origin_x;
	double  temp_y  = global_y - reference_origin_y;

	//Find the map cell idx for x, y
	std::vector<int> human_coord(2,0);
 	human_coord[0]= (int) (temp_x/static_belief_map.info.resolution);
 	human_coord[1]= (int) (temp_y/static_belief_map.info.resolution);

 	//Find the map index from cell x, y
 	int static_map_idx= human_coord[0]+static_belief_map.info.width*human_coord[1];

 	return static_map_idx;
 	//Save to human_ouccupied_index
 	// human_occupied_idx.push_back(static_map_idx);
 	 
}

void Human_Belief_Scan::CoordinateTransform_Global2_dynMap(double global_x, double global_y)
{
	//Find the reference origin coordinate
	double reference_origin_x=dynamic_belief_map.info.origin.position.x;
	double reference_origin_y=dynamic_belief_map.info.origin.position.y;

	//Find the coordinate w.r.t map origin
	double  temp_x  = global_x - reference_origin_x;
	double  temp_y  = global_y - reference_origin_y;

	//Find the map cell idx for x, y
	std::vector<int> human_coord(2,0);
 	human_coord[0]= (int) (temp_x/dynamic_belief_map.info.resolution);
 	human_coord[1]= (int) (temp_y/dynamic_belief_map.info.resolution);

 	//Find the map index from cell x, y
 	int dyn_map_idx= human_coord[0]+dynamic_belief_map.info.width*human_coord[1];

 	//Save to human_ouccupied_index
 	human_occupied_idx.push_back(dyn_map_idx);
 	
 	return;
}


void Human_Belief_Scan::put_human_occ_map_leg()
{
	if(Human_Belief_Scan_map.data.size()>0){
		for(int i(0);i<human_occupied_leg_idx.size();i++){

		 	Human_Belief_Scan_map.data[human_occupied_leg_idx[i]]=60.0;
		}

	}

}

void Human_Belief_Scan::put_human_surrounding_beliefmap(int idx)
{

	int next_idx=0;
	int mapsize=Human_Belief_Scan_map.info.width*Human_Belief_Scan_map.info.height;

	for(int i(0);i<8;i++){
		switch(i){
		case 0:	next_idx=idx-Human_Belief_Scan_map.info.width-1;
			break;
		case 1:	next_idx=idx-Human_Belief_Scan_map.info.width;
			break;
		case 2:	next_idx=idx-Human_Belief_Scan_map.info.width+1;
			break;
		case 3:	next_idx=idx-1;
			break;
		case 4: next_idx=idx+1;
			break;
		case 5:	next_idx=idx+Human_Belief_Scan_map.info.width-1;
			break;
		case 6: next_idx=idx+Human_Belief_Scan_map.info.width;
			break;
		case 7:	next_idx=idx+Human_Belief_Scan_map.info.width+1;
			break;
		}

		if(next_idx>0 && next_idx<mapsize)
			Human_Belief_Scan_map.data[next_idx]=30.0;
	}
	
}

void Human_Belief_Scan::put_human_occ_map_yolo()
{
	int num_size = human_occupied_idx.size();
	// std::cout<<"human_occupied_size"<<num_size<<std::endl;
	if(num_size>0)
	{
		if(Human_Belief_Scan_map.data.size()>0){
			for(int i(0);i<human_occupied_idx.size();i++){
			 	Human_Belief_Scan_map.data[human_occupied_idx[i]]=80.0;
			 	put_human_surrounding_beliefmap(human_occupied_idx[i]);
			}
		}
	}
	else
	{


	}

}

void Human_Belief_Scan::put_human_occ_map()
{
	//if dynamic_map is initialized, put human idx to dynamic_belief_map
	if(static_belief_map.data.size()>0){
		for(int i(0);i<human_occupied_idx.size();i++){

		 	Human_Belief_Scan_map.data[human_occupied_idx[i]]=100.0;
		}

	}
}

void Human_Belief_Scan::Human_MarkerarrayCallback(const visualization_msgs::MarkerArray::ConstPtr& msg)
{

	if(IsHeadMoving)
	{
		std::cout<<"head is moving so return function"<<std::endl;
			return;
	}
	
	//Human marker from yolo detection
	human_occupied_idx.clear();
	Cur_detected_human.clear();
	

	num_of_detected_human=msg->markers.size();

	if(num_of_detected_human>0)
		Cur_detected_human.resize(num_of_detected_human);
	else
	{
		return;

	}
	// ROS_INFO("number of detected human : %d",num_of_detected_human);
	
	
	for(int i(0);i<num_of_detected_human;i++)
	{
		geometry_msgs::Vector3Stamped gV, tV;

	    gV.vector.x = msg->markers[i].pose.position.x;
	    gV.vector.y = msg->markers[i].pose.position.y;
	    gV.vector.z = msg->markers[i].pose.position.z;

	    // std::cout<<"x :"<<_x<<"_y:"<<_y<<"_z:"<<_z<<std::endl;
	    tf::StampedTransform maptransform;
   		listener.waitForTransform("head_rgbd_sensor_rgb_frame", "map", ros::Time(0), ros::Duration(5.0));
   		listener.lookupTransform("head_rgbd_sensor_rgb_frame", "map", ros::Time(0), maptransform);
	    
	    gV.header.stamp = ros::Time();
	    gV.header.frame_id = "/head_rgbd_sensor_rgb_frame";
	    listener.transformVector(std::string("/map"), gV, tV);
	    	    
		Cur_detected_human[i].resize(2,0.0);
		// Cur_detected_human[i][0]=msg->markers[i].pose.position.x;;
		// Cur_detected_human[i][1]=msg->markers[i].pose.position.y;
		//MKMKMKMK 0624
		Cur_detected_human[i][0]=tV.vector.x+global_pose[0];
		Cur_detected_human[i][1]=tV.vector.y+global_pose[1];
		int human_mapidx=CoordinateTransform_Global2_beliefMap(Cur_detected_human[i][0],Cur_detected_human[i][1]);
		human_occupied_idx.push_back(human_mapidx);
		// std::cout<<"human index : "<<human_mapidx<<std::endl;
		//ROS_INFO("Human idx : %d, Received position x : %.3lf, y : %.3lf",i, msg->markers[i].pose.position.x,msg->markers[i].pose.position.y);
	}

	// put_human_occ_map_yolo();
	

}


bool Human_Belief_Scan::IsJointMoving(int joint_idx)
{




}

void Human_Belief_Scan::publish_headscan()
{

	std_msgs::Int8 headscan_msg;
	headscan_msg.data=9;

	// Headscan_pub.publish(headscan_msg);

}

void Human_Belief_Scan::setNearestHuman_leg()
{

	if(angle_people_set.size()==0)
	 {
		angle_people_set.clear();
		std::vector<double> Distanceset;
		// Distanceset.resize(Cur_detected_human.size(),0.0);
		Distanceset.resize(Cur_leg_yolo_human.size(),0.0);
		
		double minDistance=200.0;
		int    minDistance_Idx=0;

		if(Cur_leg_yolo_human.size()>0){
			for(int i(0);i<Cur_leg_yolo_human.size();i++)
			{
				// Distanceset[i]=getDistance(Cur_detected_human[i][0],Cur_detected_human[i][1]);
				Distanceset[i]=getDistance(Cur_leg_yolo_human[i][0],Cur_leg_yolo_human[i][1]);
				
				if(minDistance>Distanceset[i])
					{
						minDistance=Distanceset[i];
						minDistance_Idx=i;
					}
			}

			leg_target[0]=Cur_leg_yolo_human[minDistance_Idx][0];
			leg_target[1]=Cur_leg_yolo_human[minDistance_Idx][1];

			geometry_msgs::Vector3Stamped gV, tV;

		    gV.vector.x = leg_target[0];
		    gV.vector.y = leg_target[1];
		    gV.vector.z = 1.0;

		    gV.header.stamp = ros::Time();
		    gV.header.frame_id = "/map";
		    
			listener.transformVector("/base_link", gV, tV);
			double temp_angle = atan(tV.vector.y/tV.vector.x);
			angle_people_set.push_back(temp_angle);
			setViewpointTarget(leg_target);
			OnceTargeted=true;
			ROS_INFO("leg target set: %.3lf, y : %.3lf",leg_target[0],leg_target[1]);
			
		}

	 }
}

void Human_Belief_Scan::setNearestHuman()
{
	if(angle_people_set.size()==0)
	{
	std::vector<double> Distanceset;
	//angle_people_set.clear();
	// Distanceset.resize(Cur_detected_human.size(),0.0);
	Distanceset.resize(Cur_existed_human.size(),0.0);
	
	double minDistance=200.0;
	int    minDistance_Idx=0;

	if(Cur_existed_human.size()>0){
		for(int i(0);i<Cur_existed_human.size();i++)
		{
			// Distanceset[i]=getDistance(Cur_detected_human[i][0],Cur_detected_human[i][1]);
			Distanceset[i]=getDistance(Cur_existed_human[i][0],Cur_existed_human[i][1]);
			
			if(minDistance>Distanceset[i])
				{
					minDistance=Distanceset[i];
					minDistance_Idx=i;
				}
		}

		Human_target_candidate[0]=Cur_existed_human[minDistance_Idx][0];
		Human_target_candidate[1]=Cur_existed_human[minDistance_Idx][1];


			
		track_cmd.data=1;
		detect_iters=0;
	}
	else{
		track_cmd.data=1;

		detect_iters++;
		if(detect_iters>5000)
		{
			ROS_INFO("Head scan required");
			detect_iters=0;
		//	publish_headscan();
		}


		//TODO : when robo reach the people , we can do it
	}

	}
}


bool Human_Belief_Scan::IsTargetMoved(const std::vector<double> possible_target_pos, float criterion)
{
	float temp_dist=0.0;
	temp_dist= pow(leg_target[0]-possible_target_pos[0],2);
	temp_dist+=pow(leg_target[1]-possible_target_pos[1],2);
	temp_dist=sqrt(temp_dist);

	if(temp_dist>criterion)
		return true;
	else
		return false;

}

void Human_Belief_Scan::setViewpointTarget(const std::vector<double> pos)
{

	geometry_msgs::Point GazePoint_msg;

	if(pos[0]==0.0 && pos[1]==0.0)
	{
		GazePoint_msg.x=2.0;
		GazePoint_msg.y=0.0;	
	}
	else
	{
		 geometry_msgs::Vector3Stamped gV, tV;

		    gV.vector.x = leg_target[0];
		    gV.vector.y = leg_target[1];
		    gV.vector.z = 1.0;

		    // std::cout<<"x :"<<_x<<"_y:"<<_y<<"_z:"<<_z<<std::endl;
		    

		    gV.header.stamp = ros::Time();
		    gV.header.frame_id = "/map";
		    listener.transformVector("/base_range_sensor_link", gV, tV);

		    std::vector<double> tempVec(3,0.0);
		    tempVec[0]=tV.vector.x;
			tempVec[1]=tV.vector.y;
			tempVec[2]=tV.vector.z;


			GazePoint_msg.x=tempVec[0];
			GazePoint_msg.y=tempVec[1];
			GazePoint_msg.z=tempVec[2];
	
	}

	GazePoint_msg.z=1.0;

	Gaze_point_pub.publish(GazePoint_msg);

	std_msgs::Bool activateGaze_msg;
	activateGaze_msg.data=true;

	Gaze_activate_pub.publish(activateGaze_msg);

	


}

int Human_Belief_Scan::CoordinateTransform_Global2_beliefMap(double global_x, double global_y)
{

	double reference_origin_x=Human_Belief_Scan_map.info.origin.position.x;
	double reference_origin_y=Human_Belief_Scan_map.info.origin.position.y;

	//Find the coordinate w.r.t map origin
	double  temp_x  = global_x - reference_origin_x;
	double  temp_y  = global_y - reference_origin_y;

	//Find the map cell idx for x, y
	std::vector<int> human_coord(2,0);
 	human_coord[0]= (int) (temp_x/Human_Belief_Scan_map.info.resolution);
 	human_coord[1]= (int) (temp_y/Human_Belief_Scan_map.info.resolution);

 	//Find the map index from cell x, y
 	int static_map_idx= human_coord[0]+Human_Belief_Scan_map.info.width*human_coord[1];

 	return static_map_idx;
 	//Save to human_ouccupied_index
 	// human_occupied_idx.push_back(static_map_idx);


}



void Human_Belief_Scan::UpdateTarget()
{
	if(m_updateiter>MAX_UPDATE_ITER)
	 {	
		// ROS_INFO("iter");

		if(targetup==0)
		{
			// leg_target[0]=leg_target[0];
			// leg_target[1]=leg_target[1];
	
			targetup++;
			m_updateiter=0;
			//setViewpointTarget(leg_target);
	
			//setlegtarget
			// leg_target[0]=Track_human_target[0];
			// leg_target[1]=Track_human_target[1];
		}

		 if(!IsTargetMoved(leg_target,2.0))
		 {
			targetup++;
			
			if(m_viewupdateiter>MAX_VIEW_UPDATE_ITER)
				{
					// if(Cur_detected_human.size()==0)
					// 	setViewpointTarget(leg_target);
					// else
				//	setViewpointTarget(leg_target);
					
					m_viewupdateiter=0;
				}

		}

		m_updateiter=0;	
	}


	m_updateiter++;
	m_viewupdateiter++;


	}


void Human_Belief_Scan::Publish_human_target()
{

	visualization_msgs::Marker marker_human;
	marker_human.header.frame_id = "/map"; 
    marker_human.header.stamp = ros::Time::now();
    marker_human.id = 0;
    uint32_t shape = visualization_msgs::Marker::SPHERE;
    marker_human.type = shape;

	if(action_mode==2){
	//publish marker
		marker_human.pose.position.x = Track_human_target[0];
	    marker_human.pose.position.y = Track_human_target[1];
	    marker_human.pose.position.z = 1;

	    marker_human.pose.orientation.x = 0.0;
	    marker_human.pose.orientation.y = 0.0;
	    marker_human.pose.orientation.z = 0.0;
	    marker_human.pose.orientation.w = 1.0;

	    double temp_dist,temp_dist2,temp_dist3;
	    temp_dist  =0.5;
	    temp_dist2 =0.5;
	    temp_dist3 =0.5;

	    //ROS_INFO("temp dist : %.3lf, temp dist2 : %.3lf, temp dist3 : %.3lf",temp_dist,temp_dist2,temp_dist3);
	    marker_human.scale.x = std::abs(temp_dist);
	    marker_human.scale.y = std::abs(temp_dist2);
	    marker_human.scale.z = std::abs(temp_dist3);

	    marker_human.color.r = 0.0;
	    marker_human.color.g = 1.0;
	    marker_human.color.b = 0.0;
	    marker_human.color.a = 0.85;

		human_target_pub.publish(marker_human);
		// human_target_Intcmd_pub.publish(track_cmd);	
	}




	

}
void Human_Belief_Scan::global_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	global_pose[0]=msg->pose.position.x;
	global_pose[1]=msg->pose.position.y;


   tf::StampedTransform baselinktransform;
   listener.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(5.0));
   listener.lookupTransform("map", "base_link", ros::Time(0), baselinktransform);
   double yaw_tf =   tf::getYaw(baselinktransform.getRotation()); 

	global_pose[2]=yaw_tf;


   tf::StampedTransform Camera_transform;
   listener.waitForTransform("map", "head_rgbd_sensor_rgb_frame", ros::Time(0), ros::Duration(5.0));
   listener.lookupTransform("map", "head_rgbd_sensor_rgb_frame", ros::Time(0), Camera_transform);
   double yaw_angle_camera =   tf::getYaw(Camera_transform.getRotation()); 

   Camera_angle=yaw_angle_camera;


}
double Human_Belief_Scan::getDistance_from_Vec(std::vector<double> origin, double _x, double _y)
{
	double temp=0.0;

	temp=(origin[0]-_x)*(origin[0]-_x);
	temp+=(origin[1]-_y)*(origin[1]-_y);
	temp=sqrt(temp);

	return temp;

}

double Human_Belief_Scan::getDistance(double _x, double _y)
{
	double temp=0.0;

	temp=(Robot_Pos[0]-_x)*(Robot_Pos[0]-_x);
	temp+=(Robot_Pos[1]-_y)*(Robot_Pos[1]-_y);
	temp=sqrt(temp);

	return temp;
}


void Human_Belief_Scan::update_human_occ_belief_scan()
{

	float prior=0.0;
	float posterior=0.0;
	//Update human belief with cells in camera visible region
	// for(int i(0);i< visiblie_idx_set.size();i++)
	// {
	
		if(num_of_detected_human==0)
		{
			// for(int j(0);j<Human_Belief_Scan_map.data.size();j++)
			// std::cout<<"no human from yolo"<<std::endl;
			for(int i(0);i< visiblie_idx_set.size();i++)
				Human_Belief_Scan_map.data[visiblie_idx_set[i]]=0.0;
		}
		else{

			for(int i(0);i< visiblie_idx_set.size();i++){
				int belief_map_index=visiblie_idx_set[i];
				
				if( Human_Belief_Scan_map.data[belief_map_index]>0)	//If we detected human
				{
					prior =(float) Human_Belief_Scan_map.data[belief_map_index]/100.0; // P(H)
					// float P_S = P_Sc_given_H*(prior) + P_Sc_given_Hc*(1-prior);
					posterior = prior*0.99;
					Human_Belief_Scan_map.data[belief_map_index] = posterior*100.0;
				}
				else
				{
					Human_Belief_Scan_map.data[belief_map_index]=0.0;

				}
			}
		}



	int mapsize=Human_Belief_Scan_map.info.height*Human_Belief_Scan_map.info.width;
	for(int i(0);i<mapsize;i++)
		{
			if(NotUpdatedCameraregion(i))
			{
				prior =(float) Human_Belief_Scan_map.data[i]*100.0; // P(H)
				// float P_S = P_Sc_given_H*(prior) + P_Sc_given_Hc*(1-prior);
				posterior = prior*0.95;
				Human_Belief_Scan_map.data[i] = posterior/100.0;

/*				Human_Belief_Scan_map.data[i]=Human_Belief_Scan_map.data[i];
*/			}
		}

	//filter for low belief
    for(int i(0);i<mapsize;i++)
	{
		if(Human_Belief_Scan_map.data[i]<15)
		{
			Human_Belief_Scan_map.data[i]=0.0;
		}
	}

	int human_index=0;
	Cur_existed_human.clear();
	for(int i(0);i<mapsize;i++)
		{
			if(Human_Belief_Scan_map.data[i]>50)
			{
				std::vector<double> map_coord;
				CellNum2globalCoord(i, map_coord);
					
				Cur_existed_human.push_back(map_coord);
				
				double global_x_human = map_coord[0];
				double global_y_human = map_coord[1];
				
				// ROS_INFO("Human existance in here : id map index %d,  %d x %.3lf , y : %.3fl ",i,human_index, global_x_human, global_y_human);

				human_index++;
			}

		}	




}

void Human_Belief_Scan::update_human_occ_belief()
{
	float prior=0.0;
	float posterior=0.0;
	//Update human belief with cells in camera visible region
	for(int i(0);i< visiblie_idx_set.size();i++)
	{
	
		if(Cur_detected_human.size()==0)
		{
			// for(int j(0);j<Human_Belief_Scan_map.data.size();j++)
				Human_Belief_Scan_map.data[visiblie_idx_set[i]]=0.0;
		}
		else{

			int belief_map_index=visiblie_idx_set[i];
			
			if( Human_Belief_Scan_map.data[belief_map_index]>0)	//If we detected human
			{
				prior =(float) Human_Belief_Scan_map.data[belief_map_index]/100.0; // P(H)
				// float P_S = P_Sc_given_H*(prior) + P_Sc_given_Hc*(1-prior);
				posterior = prior*0.95;
				Human_Belief_Scan_map.data[belief_map_index] = posterior*100.0;
			}
			else
			{
				Human_Belief_Scan_map.data[belief_map_index]=0.0;

			}

		}

	}

	int mapsize=Human_Belief_Scan_map.info.height*Human_Belief_Scan_map.info.width;
	for(int i(0);i<mapsize;i++)
		{
			if(NotUpdatedCameraregion(i))
			{
				prior =(float) Human_Belief_Scan_map.data[i]*100.0; // P(H)
				// float P_S = P_Sc_given_H*(prior) + P_Sc_given_Hc*(1-prior);
				posterior = prior*0.95;
				Human_Belief_Scan_map.data[i] = posterior/100.0;

/*				Human_Belief_Scan_map.data[i]=Human_Belief_Scan_map.data[i];
*/			}
		}

	//filter for low belief
    for(int i(0);i<mapsize;i++)
	{
		if(Human_Belief_Scan_map.data[i]<15)
		{
			Human_Belief_Scan_map.data[i]=0.0;
		}
	}

	int human_index=0;
	Cur_existed_human.clear();
	for(int i(0);i<mapsize;i++)
		{
			if(Human_Belief_Scan_map.data[i]>60)
			{
				std::vector<double> map_coord;
				CellNum2globalCoord(i, map_coord);
					
				Cur_existed_human.push_back(map_coord);
				
				double global_x_human = map_coord[0];
				double global_y_human = map_coord[1];
				
				// ROS_INFO("Human existance in here : id map index %d,  %d x %.3lf , y : %.3fl ",i,human_index, global_x_human, global_y_human);

				human_index++;
			}

		}	

	setNearestHuman();

}


void Human_Belief_Scan::CellNum2globalCoord(const int Cell_idx, std::vector<double>& cell_xy)
{
	  cell_xy.resize(2,0.0);

	  int res =(int) Cell_idx / Human_Belief_Scan_map.info.width;
	  int div =(int) Cell_idx % Human_Belief_Scan_map.info.width;


	  cell_xy[0]=Human_Belief_Scan_map.info.resolution*div+0.5*Human_Belief_Scan_map.info.resolution+Human_Belief_Scan_map.info.origin.position.x;
	  cell_xy[1]=Human_Belief_Scan_map.info.resolution*res+0.5*Human_Belief_Scan_map.info.resolution+Human_Belief_Scan_map.info.origin.position.y;
}


bool Human_Belief_Scan::NotUpdatedCameraregion(int idx)
{

	for(int i(0);i<visiblie_idx_set.size();i++)
	{
		if(idx==visiblie_idx_set[i])
			return false;
	}

	return true;
}

void Human_Belief_Scan::Init_parameters()
{
	scanmode=true;
	global_pose.resize(3,0.0);
	Human_target_candidate.resize(2,0.0);
	viewpoint_robot.resize(2,0.0);
	viewpoint_robot[0]=2.0;

	Last_Head_vel.resize(2,0.0);
	Head_vel.resize(2,0.0);
	Head_Pos.resize(2,0.0);
	leg_target.resize(2,0.0);

	Track_human_target.resize(2,0.0);
	Robot_Pos.resize(3,0.0);   //x,y,theta
	m_receiveiter=0;
	m_updateiter=0;
	m_viewupdateiter=0;
	m_leg_updateiter=0;
	m_yolo_recieveiter=0;
	targetup=0;
	num_of_detected_human=0;

	static_belief_map.info.width=30;
	static_belief_map.info.height= 30;
	static_belief_map.info.resolution=0.5;
	static_belief_map.info.origin.position.x=-7.5;
	static_belief_map.info.origin.position.y=-7.5;
	belief_size=static_belief_map.info.width*static_belief_map.info.height;
	static_belief_map.data.resize(static_belief_map.info.width*static_belief_map.info.height);
	int belief_map_size=static_belief_map.info.width*static_belief_map.info.height;
	for(int k(0);k<belief_map_size;k++)
		static_belief_map.data[k]=0.01;

	OnceTargeted=false;

	Human_Belief_Scan_map.info.width=30;
	Human_Belief_Scan_map.info.height= 30;
	Human_Belief_Scan_map.info.resolution=0.5;
	Human_Belief_Scan_map.info.origin.position.x=-7.5;
	Human_Belief_Scan_map.info.origin.position.y=-7.5;
	belief_size=Human_Belief_Scan_map.info.width*Human_Belief_Scan_map.info.height;
	Human_Belief_Scan_map.data.resize(Human_Belief_Scan_map.info.width*Human_Belief_Scan_map.info.height);
	// int belief_map_size=static_belief_map.info.width*static_belief_map.info.height;
	for(int k(0);k<belief_map_size;k++)
		Human_Belief_Scan_map.data[k]=0.01;

	pub_iters=0;
	Camera_angle=0.0;
	print_iter=0;
	IsHeadMoving=false;
	cur_scan_idx=0;

	action_mode=0; 	// 0:  scan mode  / 1: waiting mode / 2: navigation mode

	Names_set.push_back("minkyu");
	// Names_set.push_back("minkyu");

}

void Human_Belief_Scan::joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg)
{

	Head_Pos[0]=msg->position[9];			//pan
	Head_Pos[1]=msg->position[10];			//tilt
	
	Head_vel[0]=msg->velocity[9];
	Head_vel[1]=msg->velocity[10];

	if(abs(Head_vel[0])>0.001)
		IsHeadMoving=true;
	else
		IsHeadMoving=false;

	// ROS_INFO("Head moving : %s",IsHeadMoving);
	 // 0.000138, -3.8e-05,
	// 0.000138, -0.001567,

}

void Human_Belief_Scan::facedetect_target(const ros::TimerEvent& event)
{
	if(action_mode==1)
	{
		std::cout<<"Face detect among founded people, "<< cur_scan_idx <<std::endl;
		
		if(cur_scan_idx==0)
			setViewpointTarget(Founded_human[cur_scan_idx]);
	

		int maximum_scan_num=Founded_human.size()-1;
		if(cur_scan_idx<maximum_scan_num)
			cur_scan_idx++;
		
	}
	else
	{	
		return;
		
	}
}

void Human_Belief_Scan::scanforhuman(const ros::TimerEvent& event)
{
	if(action_mode==0){
	//This function will be called every 5 secs
	for(int i(0);i<Cur_detected_human.size();i++)
	{
		bool Isalreadyhuman=false;
		

		for(int j(0);j<Founded_human.size();j++)
			{
				if(Comparetwopoistions(Founded_human[j],Cur_detected_human[i],0.5))
					Isalreadyhuman=true;
			}

		bool IsNearLeg=false;
		for(int k(0);k<Cur_leg_human.size(); k++)
		{
			if(Comparetwopoistions(Cur_detected_human[i],Cur_leg_human[k],0.8))
				IsNearLeg=true;
		}

		if( (!Isalreadyhuman) && IsNearLeg)
			Founded_human.push_back(Cur_detected_human[i]);

	}

	std_msgs::Int8 head_msg;
	head_msg.data=cur_scan_idx;
	if(cur_scan_idx<5)
		head_cmd_pub.publish(head_msg);
	else if(cur_scan_idx==5)
	{	
		head_msg.data=0;
		head_cmd_pub.publish(head_msg);
		action_mode=1;
		cur_scan_idx=-1;
	}

	cur_scan_idx++;

	}
	// ros::Rate loop_rate(50);
	// int view_count=0;

	// if(scanmode){
	// 	int possible_people_size=Cur_leg_human.size();
	// 	if(possible_people_size>0 && (cur_scan_idx<possible_people_size))
	// 	{
	// 			setViewpointTarget(Cur_leg_human[cur_scan_idx]);	
	// 			if(cur_scan_idx == possible_people_size)
	// 			{
	// 				cur_scan_idx=0;
	// 				scanmode=false;
	// 				std::cout<<"exit scan mode"<<std::endl;

	// 			}

	// 			cur_scan_idx++;
	// 	}
	// }
	// else
	// 	return;
}


void Human_Belief_Scan::InitializeBelief()
{


}

void Human_Belief_Scan::setHumanOccupancy(int idx, double dyn_posx,double dyn_posy)
{


}

void Human_Belief_Scan::Check_beliefmap()
{
	//


}


void Human_Belief_Scan::Publish_beliefmap()
{
	// static_belief_map.info.width=30;
	// static_belief_map.info.height= 30;
	// static_belief_map.info.resolution=0.5;
	// static_belief_map.info.origin.position.x=-5;
	// static_belief_map.info.origin.position.y=-5;
	// static_belief_map.data.resize(static_belief_map.info.width*static_belief_map.info.height);
	getCameraregion();
	
	if(!IsHeadMoving)
		{

		put_human_occ_map_yolo();
		// put_human_occ_map();
		update_human_occ_belief_scan();

		 static_belief_map.header.stamp =  ros::Time::now();
		 static_belief_map.header.frame_id = "map"; 
	     static_belief_map_pub.publish(static_belief_map);

	     Human_Belief_Scan_map.header.stamp =  ros::Time::now();
		 Human_Belief_Scan_map.header.frame_id = "map"; 
	     belief_pub.publish(Human_Belief_Scan_map);

 	}

}


void Human_Belief_Scan::base_pose_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
	//Recieve robot pos - x,y theta
   Robot_Pos[0]= global_pose[0];
   Robot_Pos[1]= global_pose[1];
   Robot_Pos[2]= global_pose[2];// - origin_y;	//this is not exact=>sensor_imu
   // Robot_Pos[2]+=global_pose[2];


   // tf::StampedTransform baselinktransform;
   // listener.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(10.0));
   // listener.lookupTransform("map", "base_link", ros::Time(0), baselinktransform);
   // double yaw_tf =   tf::getYaw(baselinktransform.getRotation()); 


   tf::StampedTransform Camera_transform;
   listener.waitForTransform("map", "head_rgbd_sensor_rgb_frame", ros::Time(0), ros::Duration(10.0));
   listener.lookupTransform("map", "head_rgbd_sensor_rgb_frame", ros::Time(0), Camera_transform);
   double yaw_angle_camera =   tf::getYaw(Camera_transform.getRotation()); 

   Camera_angle=yaw_angle_camera;

}


void Human_Belief_Scan::getCameraregion()
{

	double global_robot_x= global_pose[0];
	double global_robot_y= global_pose[1];
	double global_robot_theta = global_pose[2]+Head_Pos[0];


	// std::cout<<"theta:"<<Robot_Pos[2]<<",head :"<<Head_Pos[0]<<", total :"<<global_robot_theta<<std::endl;


	double m_1=tan(30*3.141592/180);
	double m_2=tan(30*3.141592/180);

	visiblie_idx_set.clear();

	global_robot_theta=0.0;
	//Iteration for belief grid
	for(int i(0);i<static_belief_map.info.width;i++)
		for(int j(0);j<static_belief_map.info.height;j++)
	{
		int belief_map_idx=j*static_belief_map.info.height+i;

		// double map_ogirin_x = static_belief_map.info.origin.position.x+global_robot_x;
		// double map_ogirin_y = static_belief_map.info.origin.position.y+global_robot_y;

		double map_ogirin_x = static_belief_map.info.origin.position.x;
		double map_ogirin_y = static_belief_map.info.origin.position.y;


		double trans_vector_x=(i+0.5)*static_belief_map.info.resolution;
		double trans_vector_y=(j+0.5)*static_belief_map.info.resolution;

		double rot_trans_vector_x = cos(global_robot_theta)*trans_vector_x-sin(global_robot_theta)*trans_vector_y;
		double rot_trans_vector_y = sin(global_robot_theta)*trans_vector_x+cos(global_robot_theta)*trans_vector_y;

		double belief_global_x=map_ogirin_x+rot_trans_vector_x;
		double belief_global_y=map_ogirin_y+rot_trans_vector_y;


		//solve
		bool line1_result =getlinevalue(1,belief_global_x,belief_global_y);
		bool line2_result =getlinevalue(2,belief_global_x,belief_global_y);


		if( line1_result && line2_result )
		{
			static_belief_map.data[belief_map_idx]=30;	
			visiblie_idx_set.push_back(belief_map_idx);					//save cell_id 
		}
		else
			static_belief_map.data[belief_map_idx]=0.0;	
	}



}

bool Human_Belief_Scan::getlinevalue(int line_type,double input_x, double input_y)
{

	double global_robot_theta = global_pose[2]+Head_Pos[0];
	// double global_robot_theta =Camera_angle;
	double theta_1=-FOVW*MATH_PI/180+global_robot_theta;
	double theta_2= FOVW*MATH_PI/180+global_robot_theta;
	
	double m_1=tan(theta_1);
	double m_2=tan(theta_2);

	 // std::cout<<"theta_1 :"<<theta_1*180/(MATH_PI)<<std::endl;
	 // std::cout<<"theta_2 :"<<theta_2*180/(MATH_PI)<<std::endl;

	 // std::cout<<"m_1 :"<<m_1<<std::endl;
	 // std::cout<<"m_2 :"<<m_2<<std::endl;

	int isspecial=0;

	if(theta_1<-MATH_PI/2.0 && theta_2 >-MATH_PI/2.0)
	{
		double temp=m_2;
		// m_2=m_1;
		// m_1=temp;
		isspecial=1;
	}
	else if(theta_2> MATH_PI/2.0 && (theta_1 <MATH_PI/2.0))
	{
		isspecial=2;
	}
	else if (theta_1<-MATH_PI/2.0 && theta_2 <-MATH_PI/2.0)
	{
		isspecial=5;
	}
	else if(theta_2< -MATH_PI/2.0)
	{
		isspecial=3;
	}

	else if(theta_1>MATH_PI/2.0 && theta_2> MATH_PI/2.0)
	{
		isspecial=4;	
	}


	 // std::cout<<"camera region section : "<<isspecial<<std::endl;
	
	double m=0.0;
	double coeff_sign=1.0;

	double global_robot_x= global_pose[0];
	double global_robot_y= global_pose[1];

	double res =0.0;

	switch(line_type){
	case 1:
			m=m_1;
			coeff_sign=-1.0;

			if(isspecial==0)
					coeff_sign=-1.0;
			else if(isspecial==1)
				coeff_sign=1.0;
			else if(isspecial==2)
				coeff_sign=-1.0;	
			else if(isspecial==4)
				coeff_sign=1.0;	
			else if(isspecial==5)
				coeff_sign=1.0;	

			break;
	case 2:
			m=m_2;
			coeff_sign=-1.0;
			if(isspecial==1)
				coeff_sign=1.0;	
			else if(isspecial==0)
			 	coeff_sign=1.0;	
			else if(isspecial==3)
				coeff_sign=1.0;
			// else if(isspecial==5)
			// 	coeff_sign=1.0;		
			// if(isspecial==4)
			// 	coeff_sign=1.0;

			// if(isspecial==2 )
			// 	coeff_sign=1.0;				
						

			break;
	default:
		std::cout<<"Wrong line type"<<std::endl;
			m=m_1;
		}

	res= m*input_x-m*global_robot_x+global_robot_y-input_y;

	if(res*coeff_sign>0 || res==0)
		return true;
	else
 		return false;

}



void Human_Belief_Scan::Publish_nav_target()
{
	
	if(pub_iters>200){

		if(action_mode==2){
			move_base_msgs::MoveBaseActionGoal Navmsgs;
		 	Navmsgs.header.stamp =  ros::Time::now();
		 //Navmsgs.header.frame_id = "map"; 
		 	Navmsgs.goal.target_pose.header.frame_id = "map";

			std::vector<double> tempVec(2,0.0);
		  	tempVec[0]=Track_human_target[0];
			tempVec[1]=Track_human_target[1];

			 Navmsgs.goal.target_pose.pose.position.x=tempVec[0]-0.3;
			 Navmsgs.goal.target_pose.pose.position.y=tempVec[1];
			 Navmsgs.goal.target_pose.pose.position.z=0.5;

			 Navmsgs.goal.target_pose.pose.orientation.x=0.0;
			 Navmsgs.goal.target_pose.pose.orientation.y=0.0;
			 Navmsgs.goal.target_pose.pose.orientation.z=0.0;
			 Navmsgs.goal.target_pose.pose.orientation.w=1.0;

			 // setViewpointTarget(Track_human_target);
			 setNavTarget_pub.publish(Navmsgs);
			 ROS_INFO("navgation published");

		pub_iters=0;
	}

	}


	pub_iters++;

}














