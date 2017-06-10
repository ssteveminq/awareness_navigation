#include "humantracking.h"


Human_Belief::Human_Belief():index(0),m_numofhuman(0),m_receiveiter(0){
	Init_parameters();
}
Human_Belief::~Human_Belief(){}
Human_Belief::Human_Belief(int numofhuman)
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


void Human_Belief::dyn_map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
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


void Human_Belief::number_detected_callback(const std_msgs::Int8::ConstPtr &msg){
	int number =  (int) msg->data;
	// std::cout << "Number of detected humans" << number << std::endl;
	if (number == 0){
		std::cout << "no detection" << std::endl;
		//track_cmd.data=0;
		//update_human_occ_belief(0);
	}
	else
	{

		// track_cmd=1;
	}
}


int Human_Belief::CoordinateTransform_Global2_staticMap(double global_x, double global_y)
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

void Human_Belief::CoordinateTransform_Global2_dynMap(double global_x, double global_y)
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

void Human_Belief::put_human_occ_map()
{
	//if dynamic_map is initialized, put human idx to dynamic_belief_map
	if(static_belief_map.data.size()>0){
		for(int i(0);i<human_occupied_idx.size();i++){

		 	human_belief_map.data[human_occupied_idx[i]]=100.0;

		}
	}
}

void Human_Belief::Human_MarkerarrayCallback(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
	human_occupied_idx.clear();
	Cur_detected_human.clear();

	int num_of_detected_human=msg->markers.size();
	//ROS_INFO("number of detected human : %d",num_of_detected_human);
	Cur_detected_human.resize(num_of_detected_human);
	

	for(int i(0);i<num_of_detected_human;i++)
	{
		
		geometry_msgs::Vector3Stamped gV, tV;

	    gV.vector.x = msg->markers[i].pose.position.x;
	    gV.vector.y = msg->markers[i].pose.position.y;
	    gV.vector.z = 1.0;

	    // std::cout<<"x :"<<_x<<"_y:"<<_y<<"_z:"<<_z<<std::endl;
	    gV.header.stamp = ros::Time();
	    gV.header.frame_id = "/base_link";
	    listener.transformVector("/map", gV, tV);


		Cur_detected_human[i].resize(2,0.0);
		Cur_detected_human[i][0]=tV.vector.x;
		Cur_detected_human[i][1]=tV.vector.y;


		int human_mapidx=CoordinateTransform_Global2_staticMap(Cur_detected_human[i][0],Cur_detected_human[i][1]	);
		human_occupied_idx.push_back(human_mapidx);
		std::cout<<"human index : "<<human_mapidx<<std::endl;
		//ROS_INFO("Human idx : %d, Received position x : %.3lf, y : %.3lf",i, msg->markers[i].pose.position.x,msg->markers[i].pose.position.y);
	}

	put_human_occ_map();
	setNearestHuman();
}

void Human_Belief::setNearestHuman()
{

	std::vector<double> Distanceset;
	Distanceset.resize(Cur_detected_human.size(),0.0);
	
	double minDistance=200.0;
	int    minDistance_Idx=0;

	if(Cur_detected_human.size()>0){
		for(int i(0);i<Cur_detected_human.size();i++)
		{
			Distanceset[i]=getDistance(Cur_detected_human[i][0],Cur_detected_human[i][1]);
			
			if(minDistance>Distanceset[i])
				{
					minDistance=Distanceset[i];
					minDistance_Idx=i;
				}
		}


		Human_target_Pos[0]=Cur_detected_human[minDistance_Idx][0];
		Human_target_Pos[1]=Cur_detected_human[minDistance_Idx][1];

		track_cmd.data=1;

	}
	else{
		track_cmd.data=0;

	}



}

void Human_Belief::Publish_human_target()
{

	visualization_msgs::Marker marker_human;
	marker_human.header.frame_id = "/map"; 
    marker_human.header.stamp = ros::Time::now();
    marker_human.id = 0;
    uint32_t shape = visualization_msgs::Marker::SPHERE;
    marker_human.type = shape;

	if(track_cmd.data>0){
	//publish marker
		marker_human.pose.position.x = Human_target_Pos[0];
	    marker_human.pose.position.y = Human_target_Pos[1];
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

	    marker_human.color.r = 1.0;
	    marker_human.color.g = 1.0;
	    marker_human.color.b = 0.2;
	    marker_human.color.a = 0.85;


		human_target_pub.publish(marker_human);
		human_target_Intcmd_pub.publish(track_cmd);	


	}
	//publish command

	

}

double Human_Belief::getDistance(double _x, double _y)
{
	double temp=0.0;

	temp=(Robot_Pos[0]-_x)*(Robot_Pos[0]-_x);
	temp+=(Robot_Pos[1]-_y)*(Robot_Pos[1]-_y);
	temp=sqrt(temp);

	return temp;
}


void Human_Belief::update_human_occ_belief()
{

		// std::map<int, int> map_index_recently_updated;
		// if (update_type == (int) HUMANS_DETECTED){
		// 	for(int i = 0; i < index_of_human_occ_cells_updated_recently.size(); i++){
		// 		int index = index_of_human_occ_cells_updated_recently[i];
		// 		map_index_recently_updated[index] = index;
		// 	}
		// }



		// std::vector<int> indices_to_assign_as_free;

		// typedef std::map<int, int>::iterator it_type;
		// for(it_type iterator = camera_visible_world_indices.begin(); iterator !=  camera_visible_world_indices.end(); iterator++) {
		// 	int cell_index = iterator->first;
		// 	if (map_index_recently_updated.count(cell_index) == 1){
		// 		continue;
		// 	}
		// 	if (data[cell_index].cell_type == HUMAN_OCCUPIED){
		// 		// Update probability			
		// 		float prior = map_index_of_human_cells_to_prob[cell_index]; // P(H)
		// 		float P_S = P_Sc_given_H*(prior) + P_Sc_given_Hc*(1-prior);
		// 		float posterior = prior*0.4;
		// 		map_index_of_human_cells_to_prob[cell_index] = posterior;

		// 		//std::cout << "Prob: " << posterior << std::endl;

		// 		if (posterior < PROB_THRESH){
		// 			indices_to_assign_as_free.push_back(cell_index);
		// 		}

		// 	}

		// }

		// // For each low prob cell, delete it and update the cell type

		// for(size_t i = 0; i < indices_to_assign_as_free.size(); i++){
		// 	int index_to_erase =  indices_to_assign_as_free[i];
		// 	map_index_of_human_cells_to_prob.erase(index_to_erase);
		// 	data[index_to_erase].cell_type = FREE_CELL; 
		// }


	//Update human belief with cells in camera visible region
	for(int i(0);i< visiblie_idx_set.size();i++)
	{
		int belief_map_index=visiblie_idx_set[i];
		if( human_belief_map.data[belief_map_index]>0)	//If we detected human
		{
			float prior =(float) human_belief_map.data[belief_map_index]/100.0; // P(H)
			// float P_S = P_Sc_given_H*(prior) + P_Sc_given_Hc*(1-prior);
			float posterior = prior*0.8;
			human_belief_map.data[belief_map_index] = posterior*100.0;
		}

	}


	int mapsize=human_belief_map.info.height*human_belief_map.info.width;
	for(int i(0);i<mapsize;i++)
		{
			if(NotUpdatedCameraregion(i))
			{
					human_belief_map.data[i]=human_belief_map.data[i];
			}
		}

}

bool Human_Belief::NotUpdatedCameraregion(int idx)
{

	for(int i(0);i<visiblie_idx_set.size();i++)
	{
		if(idx==visiblie_idx_set[i])
			return false;
	}

	return true;
}

void Human_Belief::Init_parameters()
{
	Human_target_Pos.resize(2,0.0);
	Robot_Pos.resize(3,0.0);   //x,y,theta
	m_receiveiter=0;

	static_belief_map.info.width=30;
	static_belief_map.info.height= 30;
	static_belief_map.info.resolution=0.5;
	static_belief_map.info.origin.position.x=-5;
	static_belief_map.info.origin.position.y=-5;
	belief_size=static_belief_map.info.width*static_belief_map.info.height;
	static_belief_map.data.resize(static_belief_map.info.width*static_belief_map.info.height);
	int belief_map_size=static_belief_map.info.width*static_belief_map.info.height;
	for(int k(0);k<belief_map_size;k++)
		static_belief_map.data[k]=0.01;


	human_belief_map.info.width=30;
	human_belief_map.info.height= 30;
	human_belief_map.info.resolution=0.5;
	human_belief_map.info.origin.position.x=-5;
	human_belief_map.info.origin.position.y=-5;
	belief_size=human_belief_map.info.width*human_belief_map.info.height;
	human_belief_map.data.resize(human_belief_map.info.width*human_belief_map.info.height);
	// int belief_map_size=static_belief_map.info.width*static_belief_map.info.height;
	for(int k(0);k<belief_map_size;k++)
		human_belief_map.data[k]=0.01;

}

void Human_Belief::InitializeBelief()
{


}

void Human_Belief::setHumanOccupancy(int idx, double dyn_posx,double dyn_posy)
{


}

void Human_Belief::Publish_beliefmap()
{
	// static_belief_map.info.width=30;
	// static_belief_map.info.height= 30;
	// static_belief_map.info.resolution=0.5;
	// static_belief_map.info.origin.position.x=-5;
	// static_belief_map.info.origin.position.y=-5;
	// static_belief_map.data.resize(static_belief_map.info.width*static_belief_map.info.height);


	getCameraregion();
	put_human_occ_map();
	update_human_occ_belief();

	 static_belief_map.header.stamp =  ros::Time::now();
	 static_belief_map.header.frame_id = "map"; 
     static_belief_map_pub.publish(static_belief_map);

     human_belief_map.header.stamp =  ros::Time::now();
	 human_belief_map.header.frame_id = "map"; 
     belief_pub.publish(human_belief_map);



}


void Human_Belief::base_pose_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
	//Recieve robot pos - x,y theta
   Robot_Pos[0]= msg->pose.pose.position.x;
   Robot_Pos[1]= msg->pose.pose.position.y;
   Robot_Pos[2]= asin(msg->pose.pose.orientation.z)*2;// - origin_y;	//this is not exact=>sensor_imu
}


void Human_Belief::getCameraregion()
{

	double global_robot_x= Robot_Pos[0];
	double global_robot_y= Robot_Pos[1];
	double global_robot_theta = Robot_Pos[2];
	//std::cout<<global_robot_theta<<std::endl;

	double m_1=tan(29*3.141592/180);
	double m_2=tan(29*3.141592/180);

	visiblie_idx_set.clear();

	global_robot_theta=0.0;
	//Iteration for belief grid
	for(int i(0);i<static_belief_map.info.width;i++)
		for(int j(0);j<static_belief_map.info.height;j++)
	{
		int belief_map_idx=j*static_belief_map.info.height+i;

		double map_ogirin_x = static_belief_map.info.origin.position.x;
		double map_ogirin_y = static_belief_map.info.origin.position.y;

		double trans_vector_x=(i+0.5)*static_belief_map.info.resolution;
		double trans_vector_y=(j+0.5)*static_belief_map.info.resolution;

		double rot_trans_vector_x = cos(global_robot_theta)*trans_vector_x-sin(global_robot_theta)*trans_vector_y;
		double rot_trans_vector_y = sin(global_robot_theta)*trans_vector_x+cos(global_robot_theta)*trans_vector_y;

		double belief_global_x=map_ogirin_x+rot_trans_vector_x;
		double belief_global_y=map_ogirin_y+rot_trans_vector_y;


		// if(getlinevalue(1,belief_global_x,belief_global_y) && !(getlinevalue(2,belief_global_x,belief_global_y)))


	
		// if(getlinevalue(1,belief_global_x,belief_global_y)
		if(getlinevalue(1,belief_global_x,belief_global_y) && (getlinevalue(2,belief_global_x,belief_global_y)))
		{
			static_belief_map.data[belief_map_idx]=30;	
			visiblie_idx_set.push_back(belief_map_idx);					//save cell_id 
		}
		else
			static_belief_map.data[belief_map_idx]=0.0;	
	}



}

bool Human_Belief::getlinevalue(int line_type,double input_x, double input_y)
{

	double global_robot_theta = Robot_Pos[2];
	double theta_1=-FOVW*MATH_PI/180+global_robot_theta;
	double theta_2= FOVW*MATH_PI/180+global_robot_theta;
	

	double m_1=tan(theta_1);
	double m_2=tan(theta_2);

	// std::cout<<"theta_1 :"<<theta_1*180/(MATH_PI)<<std::endl;
	// std::cout<<"theta_2 :"<<theta_2*180/(MATH_PI)<<std::endl;

	int isspecial=0;

	if(theta_1<-MATH_PI/2.0)
	{
		double temp=m_2;
		m_2=m_1;
		m_1=temp;
		isspecial=1;
	}
	else if(theta_2> MATH_PI/2.0 && (theta_1 <MATH_PI/2.0))
	{
		isspecial=2;

	}
	
	if(theta_2< -MATH_PI/2.0)
	{
		isspecial=3;
	}

	if(theta_1>MATH_PI/2.0 && theta_2> MATH_PI/2.0)
	{
		double temp=m_2;
		m_2=m_1;
		m_1=temp;
		isspecial=4;	
	}
	
	// double m_2=+tan(29*3.141592/180+3.141592/2+global_robot_theta);	

	double m=0.0;
	double coeff_sign=1.0;

	double global_robot_x= Robot_Pos[0];
	double global_robot_y= Robot_Pos[1];

	double res =0.0;

	switch(line_type){
	case 1:
			m=m_1;
			coeff_sign=1.0;

			if(isspecial==1)
				coeff_sign=-1.0;				
			break;
	case 2:
			m=m_2;
			coeff_sign=-1.0;

			if(isspecial==2 )
				coeff_sign=1.0;				
						

			break;
	default:
		std::cout<<"Wrong line type"<<std::endl;
			m=m_1;
		}

	res= m*input_x-m*global_robot_x+global_robot_y-input_y;

	if(res*coeff_sign<0 || res==0)
		return true;
	else
 		return false;

}














