#include "Dynamic_Manager.h"


Dynamic_Manager::Dynamic_Manager(MapParam* _pMapParam):maxiter(Maxiteration),Action_dim(8),gamma(1),Ra(ra),publishnum(0),ReceiveData(0),m_boolSolve(false),dyn_path_num(0)
{
	pMapParam=_pMapParam;
	Init();
}

void Dynamic_Manager::setPMapParam(MapParam* _pMapParam)
{

	pMapParam=_pMapParam;
	Init();
}

Dynamic_Manager::~Dynamic_Manager()
{
	if(pMapParam!=NULL)
	{
		delete pMapParam;
		pMapParam=NULL;
	}
}



// void Dynamic_Manager::base_pose_callback(const nav_msgs::Odometry::ConstPtr& msg)
// {
//    CurVector[0]= global_pose[0];
//    CurVector[1]= global_pose[1];
//    CurVector[2]= global_pose[2];

// }

void Dynamic_Manager::Init()
{
	cout<<"Initialize"<<endl;
	Local_X_start=0;
 	Local_Y_start=0;
 	ReceiveData=0;
 	human_callback_count=0;
 	
 	X_mapSize=pMapParam->Num_grid_X;
 	Y_mapSize=pMapParam->Num_grid_Y;
 	Num_Grids=pMapParam->MapSize;
 	cout<<" Grid - X :"<<X_mapSize<<", - Y : "<<Y_mapSize<<endl;
 	cout<<" NumofGrids : "<<Num_Grids<<endl;

 	Policies.resize(pMapParam->MapSize, '0');
 	Rewards.resize(pMapParam->MapSize, Ra); 
 	U.resize(pMapParam->MapSize, 0.0); 
 	Up.resize(pMapParam->MapSize, 0.0); 
 	MdpSols.resize(pMapParam->MapSize, 0);
 	PolicyNum.resize(pMapParam->MapSize, 0);

 	m_Start.resize(2,0);	
    m_Goal.resize(2,0);							
 	m_Robot.resize(2,0);
 	Human_Goal_Coord.resize(2,0);
 	human_global.resize(2,0.0);
 	m_Start[0]=Start_X;
 	m_Start[1]=Start_Y;
 	cout<<" Start pos - X :"<<Start_X<<", - Y : "<<Start_Y<<endl;

 	//Current Pos 
 	m_Robot[0]=Start_X;
 	m_Robot[1]=Start_Y;

 	m_Goal[0]=Goal_X;
 	m_Goal[1]=Goal_Y;
 	cout<<" Goal pos - X :"<<Goal_X<<", - Y : "<<Goal_Y<<endl;

 	 Rewards[Coord2CellNum(m_Goal)]=100;
 	Policies[Coord2CellNum(m_Goal)]='+';


 	for(int i(0);i<m_static_obs.size();i++){
 		Rewards[m_static_obs[i]]=0.0;
 		Policies[m_static_obs[i]]='#';
  	}

 	//setActionVector
 	ActionCC.resize(Action_dim);
 	for(int i(0);i<Action_dim;i++)
 		ActionCC[i].resize(2);

 // 	ActionCC[0][0]= 1;   ActionCC[0][1]= 0;
	// ActionCC[1][0]= 0;   ActionCC[1][1]= 1;
	// ActionCC[2][0]= -1;  ActionCC[2][1]= 0;
	// ActionCC[3][0]=0;    ActionCC[3][1]= -1;
 	ActionCC[0][0]= 1;   ActionCC[0][1]= 0;
	ActionCC[1][0]= 1;   ActionCC[1][1]= 1;
	ActionCC[2][0]= 0;   ActionCC[2][1]= 1;
	ActionCC[3][0]=-1;   ActionCC[3][1]= 1;
	ActionCC[4][0]=-1;   ActionCC[4][1]= 0;
	ActionCC[5][0]=-1;   ActionCC[5][1]=-1;
	ActionCC[6][0]= 0;   ActionCC[6][1]=-1;
	ActionCC[7][0]= 1;   ActionCC[7][1]=-1;

	Prob_good = 0.95;
	Prob_bad = (1-Prob_good)/2.0;

	 Map_orig_Vector.resize(2,0.0);
	 Map_orig_Vector[0]= 3.5;
   	 Map_orig_Vector[1]=-3.5;


   	 global_pose.resize(3,0.0);
	 CurVector.resize(3,0.0);
	 GoalVector.resize(2,0.0);
	 HeadingVector.resize(2,0.0);
	 cur_coord.resize(2,0);
	 Goal_Coord.resize(2,0);
	 MapCoord.resize(2,0);
	 filtered_target.resize(2,0.0);
	 m_desired_heading=0.0;
     viewTarget.resize(2,0.0);
     
	 
    viewpub_iters=0;
	 //m_localoccupancy.resize(Grid_Num_X*Grid_Num_Y);
	 m_dynamic_occupancy.resize(Num_Grids);

	 Head_Pos.resize(2,0.0);
  	 // filtered_leg_target.resize(2,0.0);

	//Declare publisher
	 // obsmap_Pub= m_node.advertise<std_msgs::Int32MultiArray>("MDP/costmap", 10);
	SplinePath_pub=  m_node.advertise<nav_msgs::Path>("mdp_path", 10, true);
	SplinePath_pub2=  m_node.advertise<nav_msgs::Path>("mdp_path_dynamic", 10, true);
	Scaled_static_map_pub=m_node.advertise<nav_msgs::OccupancyGrid>("/scaled_static_map", 10, true);
	Scaled_dynamic_map_pub=m_node.advertise<nav_msgs::OccupancyGrid>("/scaled_dynamic_map", 10, true);
	Scaled_dynamic_map_path_pub=m_node.advertise<nav_msgs::OccupancyGrid>("/scaled_dynamic_map_path", 10, true);
	MDPSol_pub = m_node.advertise<std_msgs::Int32MultiArray>("MDP/Solution", 10);
	RobotHeading_pub= m_node.advertise<geometry_msgs::Pose>("mdp_HeadingV", 10);
	Gaze_point_pub= m_node.advertise<geometry_msgs::Point>("/gazed_point_fixing_node/target_point", 50, true);
	Gaze_activate_pub= m_node.advertise<std_msgs::Bool>("/gazed_point_fixing_node/activate", 50, true);
    viewTarget_visual_pub = m_node.advertise<visualization_msgs::Marker>("viewTarget_marker",30, true);
	camera_map_pub=m_node.advertise<nav_msgs::OccupancyGrid>("/camera_region_map", 10, true);
	Human_boxes_pub=m_node.advertise<visualization_msgs::MarkerArray>("/filtered_humans", 10);
	Leg_boxes_pub=m_node.advertise<visualization_msgs::MarkerArray>("/filtered_leg_target", 10);
	people_measurement_pub_ = m_node.advertise<people_msgs::PositionMeasurement>("people_tracker_measurements", 10);
	belief_pub=m_node.advertise<nav_msgs::OccupancyGrid>("/Human_belief_map", 10, true);

	Scaled_static_map_path.info.width=Grid_Num_X;
	Scaled_static_map_path.info.height= Grid_Num_Y;
	Scaled_static_map_path.info.resolution=0.5;
	Scaled_static_map_path.info.origin.position.x=-4;
	Scaled_static_map_path.info.origin.position.y=-4;
	Scaled_static_map_path.data.resize(Scaled_static_map_path.info.width*Scaled_static_map_path.info.height);

   	Scaled_dynamic_map_path.info.width=10;
	Scaled_dynamic_map_path.info.height= 10;
	Scaled_dynamic_map_path.info.resolution=0.75;
	Scaled_dynamic_map_path.info.origin.position.x=CurVector[0]-DYN_OFFSET_X-0.5*Scaled_dynamic_map.info.resolution;
	Scaled_dynamic_map_path.info.origin.position.y=CurVector[1]-DYN_OFFSET_Y-0.5*Scaled_dynamic_map.info.resolution;;
	Scaled_dynamic_map_path.data.resize(Scaled_dynamic_map_path.info.width*Scaled_dynamic_map_path.info.height);
	booltrackHuman=false;
	dyn_path_num=0;

	Human_Belief_Scan_map.info.width=15;
	Human_Belief_Scan_map.info.height= 15;
	Human_Belief_Scan_map.info.resolution=0.5;
	double belief_map_offset =Human_Belief_Scan_map.info.width*Human_Belief_Scan_map.info.resolution;
	Human_Belief_Scan_map.info.origin.position.x=CurVector[0]-0.5*belief_map_offset-0.5*Human_Belief_Scan_map.info.resolution;
	Human_Belief_Scan_map.info.origin.position.y=CurVector[1]-0.5*belief_map_offset-0.5*Human_Belief_Scan_map.info.resolution;
	
	int belief_size=Human_Belief_Scan_map.info.width*Human_Belief_Scan_map.info.height;
	Human_Belief_Scan_map.data.resize(belief_size);
	for(int k(0);k<belief_size;k++)
		Human_Belief_Scan_map.data[k]=0.01;

	
  //camera region
  camera_map.info.width=30;
  camera_map.info.height= 30;
  camera_map.info.resolution=0.5;
  camera_map.info.origin.position.x=-7.5;
  camera_map.info.origin.position.y=-7.5;
  int camera_map_size=camera_map.info.width*camera_map.info.height;
  camera_map.data.resize(camera_map_size);
  for(int k(0);k<camera_map_size;k++)
    camera_map.data[k]=0.01;

}


void Dynamic_Manager::CoordinateTansform_Rviz_Dyn_map(double _x, double _y,vector<int>& dynamicCoord)
{
	dynamicCoord.resize(2,0.0);
	double reference_origin_x=Scaled_dynamic_map.info.origin.position.x;
	double reference_origin_y=Scaled_dynamic_map.info.origin.position.y;

	double  temp_x  = _x-reference_origin_x;
	double  temp_y = _y-reference_origin_y;

	dynamicCoord[0]= static_cast<int>(temp_x/pMapParam->map_step);
 	dynamicCoord[1]= static_cast<int>(temp_y/pMapParam->map_step);

 	// int mapidx=Coord2CellNum(dynamicCoord);


}


//function which relates to get origin w.r.t map (mdp)3232
void Dynamic_Manager::CoordinateTransform_Rviz_Grid_Start(double _x, double _y,int map_type=0)
{
	cur_coord.resize(2,0);

	 //for case of using static map
	double reference_origin_x;
	double reference_origin_y;

	if(map_type==0) //static
	{
		ROS_INFO("static_start");
		reference_origin_x=Scaled_static_map.info.origin.position.x;
		reference_origin_y=Scaled_static_map.info.origin.position.y;
	}
	else
	{
		ROS_INFO("dynamic_start");
		reference_origin_x=Scaled_dynamic_map.info.origin.position.x;
		reference_origin_y=Scaled_dynamic_map.info.origin.position.y;
	}

	double  temp_x  = _x-reference_origin_x;
	double  temp_y = _y-reference_origin_y;


	cur_coord[0]= (int)(temp_x/pMapParam->map_step);
 	cur_coord[1]= (int)(temp_y/pMapParam->map_step);


 	 return;
}

//function which relates to get origin w.r.t map (mdp)
void Dynamic_Manager::CoordinateTransform_Rviz_Grid_Goal(double _x, double _y,int map_type=0)
{
	 Goal_Coord.resize(2,0);
	
	double reference_origin_x;
	double reference_origin_y;

	if(map_type==0) //static
	{
		ROS_INFO("static_goal_setting\n");
		reference_origin_x=Scaled_static_map.info.origin.position.x;
		reference_origin_y=Scaled_static_map.info.origin.position.y;
	}
	else	//dynamic_window
	{
		ROS_INFO("dynamic_goal_setting\n");
		reference_origin_x=Scaled_dynamic_map.info.origin.position.x;
		reference_origin_y=Scaled_dynamic_map.info.origin.position.y;
	}


	double  temp_x  = _x-reference_origin_x;
	double  temp_y = _y-reference_origin_y;

//check
 	Goal_Coord[0]= (int) (temp_x/pMapParam->map_step);
 	Goal_Coord[1]= (int)(temp_y/pMapParam->map_step);

 	ROS_INFO("Goal_Coord : x : %d , y : %d ", Goal_Coord[0],Goal_Coord[1]);

 	 return;

}


void Dynamic_Manager::CoordinateTransform_Rviz_Grid_Human(double _x, double _y,int map_type=0)
{
	 Human_Goal_Coord.resize(2,0);
	
	double reference_origin_x;
	double reference_origin_y;

	if(map_type==0) //static
	{
		
		reference_origin_x=Scaled_static_map.info.origin.position.x;
		reference_origin_y=Scaled_static_map.info.origin.position.y;
	}
	else	//dynamic_window
	{
		
		reference_origin_x=Scaled_dynamic_map.info.origin.position.x;
		reference_origin_y=Scaled_dynamic_map.info.origin.position.y;
	}


	double  temp_x  = _x-reference_origin_x;
	double  temp_y = _y-reference_origin_y;


 	Human_Goal_Coord[0]= (int) (temp_x/pMapParam->map_step);
 	Human_Goal_Coord[1]= (int)(temp_y/pMapParam->map_step);

 	// ROS_INFO("Goal_Coord : x : %d , y : %d ", Goal_Coord[0],Goal_Coord[1]);

 	 return;

}

// void Dynamic_Manager::Human_target_cmdCallback(const std_msgs::Int8::ConstPtr& msg)
// {

// 	if(msg->data==1)
// 	{
// 		booltrackHuman=true;
// 	}
// 	else{
// 		booltrackHuman=false;

// 	}
// }

void Dynamic_Manager::joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg)
{

  Head_Pos[0]=msg->position[9];     //pan
  Head_Pos[1]=msg->position[10];      //tilt
 

}

bool Dynamic_Manager::IsinDynamicMap(float global_x, float global_y)
{
	float margin =0.1;
	float map_start_x=Scaled_dynamic_map.info.origin.position.x-margin;
	float map_start_y=Scaled_dynamic_map.info.origin.position.y-margin;
	float map_end_x =Scaled_dynamic_map.info.origin.position.x+Scaled_dynamic_map.info.width*Scaled_dynamic_map.info.resolution+margin;
	float map_end_y =Scaled_dynamic_map.info.origin.position.y+Scaled_dynamic_map.info.height*Scaled_dynamic_map.info.resolution+margin;

	if( (global_x> map_start_x) && (global_x< map_end_x) )
		if((global_y> map_start_y) && (global_y< map_end_y) )
			return true;

	return false;
}


bool Dynamic_Manager::IsTargetMoved(float target_x, float target_y, float criterion)
{
	float temp_dist=0.0;
	temp_dist= (GoalVector[0]-target_x)*(GoalVector[0]-target_x);
	temp_dist+=(GoalVector[1]-target_y)*(GoalVector[1]-target_y);
	temp_dist=sqrt(temp_dist);

	if(temp_dist>criterion)
		return true;
	else
		return false;

}

double Dynamic_Manager::getdistance(vector<double> cur, vector<double> goal)
{
	double temp_dist=0.0;
	for(int i(0);i<2;i++)
		temp_dist+=pow((cur[i]-goal[i]),2);
	
	temp_dist = sqrt(temp_dist);

	return temp_dist;

}

void Dynamic_Manager::setDesiredHeading(double _heading)
{
	m_desired_heading=_heading;
}


void Dynamic_Manager::mdppath_callback(const nav_msgs::Path::ConstPtr & msg)
{
    nav_msgs::Path mdp_path_msg;    
    mdp_path_msg=(*msg);
    std::vector<int> dyn_coords(2,0);

    Dyn_MDPPath.clear();
    for(int j(0);j<Scaled_dynamic_map_path.data.size();j++)
	 {	
	 	Scaled_dynamic_map_path.data[j]=0.0;
	 }


	int path_size=mdp_path_msg.poses.size();
    for(int i(0);i<path_size;i++)
    {
        // std::cout<<mdp_path_msg.poses[i].pose.position.x<<", "<<mdp_path_msg.poses[i].pose.position.y<<std::endl;

    	CoordinateTansform_Rviz_Dyn_map(mdp_path_msg.poses[i].pose.position.x,mdp_path_msg.poses[i].pose.position.y,dyn_coords);       
		int cur_stid=Coord2CellNum(dyn_coords);
		Scaled_dynamic_map_path.data[cur_stid]=80;
    }

	 Scaled_dynamic_map_path.header.stamp =  ros::Time::now();
	 Scaled_dynamic_map_path.header.frame_id = "map"; 
     Scaled_dynamic_map_path_pub.publish(Scaled_dynamic_map_path);
}


void Dynamic_Manager::Human_MarkerCallback(const visualization_msgs::Marker::ConstPtr& msg)
{
/*
	//float human_target_goal_x=  msg->pose.position.x;
    //float human_target_goal_y=  msg->pose.position.y;

    //CoordinateTransform_Rviz_Grid_Human(human_target_goal_x,human_target_goal_y,1);
   

   check if person moved a lot 
   //if( (dyn_path_num>0) && IsTargetMoved(human_target_goal_x,human_target_goal_y,3.0))
       //return;

   check if person is in a range
    if(!IsinDynamicMap(human_target_goal_x,human_target_goal_y))   
    {
           ROS_INFO("human is out of my dynamic range");
           return;
    }

    dynamic goal setting
    if(booltrackHuman)
    {
        //if(dyn_path_num>0 && (!IsinDynamicMap(human_target_goal_x,human_target_goal_y)))
            //return;

        if(dyn_path_num==0 || IsTargetMoved(human_target_goal_x,human_target_goal_y, 0.5))
        { 
            global coordinate
			//GoalVector.resize(2,0);
			//GoalVector[0]=human_target_goal_x;
            GoalectoFiltered_leg_human.clear();

H   */
}

void Dynamic_Manager::Human_MarkerArrayCallback(const visualization_msgs::MarkerArray::ConstPtr& msg)
{

    int num_of_detected_human=msg->markers.size();

    if(num_of_detected_human>0)
       cur_people.resize(num_of_detected_human);
    else
    {
      return;
    }

    for(int i(0);i<num_of_detected_human;i++)
    {
      geometry_msgs::Vector3Stamped gV, tV;

      gV.vector.x = msg->markers[i].pose.position.x;
      gV.vector.y = msg->markers[i].pose.position.y;
      gV.vector.z = msg->markers[i].pose.position.z;

      // std::cout<<"x :"<<_x<<"_y:"<<_y<<"_z:"<<_z<<std::endl;
      tf::StampedTransform maptransform;
      listener.waitForTransform("head_rgbd_sensor_rgb_frame", "map", ros::Time(0), ros::Duration(1.0));
              
      gV.header.stamp = ros::Time();
      gV.header.frame_id = "/head_rgbd_sensor_rgb_frame";
      listener.transformVector(std::string("/map"), gV, tV);
              
      cur_people[i].resize(2,0.0);
      cur_people[i][0]=tV.vector.x+global_pose[0];
      cur_people[i][1]=tV.vector.y+global_pose[1];
   }
 
    //CoordinateTransform_Rviz_Grid_Human(human_target_goal_x,human_target_goal_y,1);
 

}
/*
void Dynamic_Manager::Human_Yolo_Callback(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
 	std::vector<double> tempVec(2,0.0);
    int num_of_detected_human_yolo=msg->markers.size();
    bool IsNearfromRobot_yolo =false;
    geometry_msgs::Vector3Stamped gV, tV;
        
    if(num_of_detected_human_yolo>0)
      {
         // cur_yolo_people.resize(num_of_detected_human_yolo,0.0);
         cur_yolo_people.clear();
      }
    else
    {
      return;
    }

    for(int i(0);i<num_of_detected_human_yolo;i++)
    {
      IsNearfromRobot_yolo =false;

      gV.vector.x = msg->markers[i].pose.position.x;
      gV.vector.y = msg->markers[i].pose.position.y;
      gV.vector.z = msg->markers[i].pose.position.z;

      // std::cout<<"x :"<<_x<<"_y:"<<_y<<"_z:"<<_z<<std::endl;
      tf::StampedTransform maptransform;
      
      listener.waitForTransform("head_rgbd_sensor_rgb_frame", "map", ros::Time(0), ros::Duration(1.0));
      // listener.waitForTransform(msg->markers[i].header.frame_id, "map", ros::Time(0), ros::Duration(1.0));
      gV.header.stamp = ros::Time();
      gV.header.frame_id = "/head_rgbd_sensor_rgb_frame";
      // gV.header.frame_id =msg->markers[i].header.frame_id;
      listener.transformVector(std::string("/map"), gV, tV);

      // tempVec[0]=tV.vector.x+global_pose[0];
      // tempVec[1]=tV.vector.x+global_pose[1];

      tempVec[0]=tV.vector.x+global_pose[0];
      tempVec[1]=tV.vector.y+global_pose[1];

      if(Comparetwopoistions(global_pose,tempVec,YOLO_range_person))
         IsNearfromRobot_yolo=true;

      if(IsNearfromRobot_yolo)
      {       
        cur_yolo_people.push_back(tempVec);
        
        // std::cout<<"x : "<<tempVec[0]<< " , y : "<<tempVec[1]<<std::endl;
      }

   }

   int people_size=cur_yolo_people.size();

   if(OnceTarget && (people_size==1))
   {
   		if(Yolo_iter>200){
   		int idx=0;
   	    string num_s = std::to_string(idx);
        string person("person ");
        string new_str_name=person+num_s;
        // std::cout<<"person name = "<<new_str_name.c_str()<<std::endl;
		
        people_msgs::PositionMeasurement pos;
        pos.header.stamp = ros::Time();
        pos.header.frame_id = "/map";
        pos.name = "leg_laser";
        pos.object_id =new_str_name;
        pos.pos.x = cur_yolo_people[idx][0];
        pos.pos.y = cur_yolo_people[idx][1];
        pos.pos.z = 1.0;
        pos.reliability = 0.85;
        pos.covariance[0] = pow(0.01 / pos.reliability, 2.0);
        pos.covariance[1] = 0.0;
        pos.covariance[2] = 0.0;
        pos.covariance[3] = 0.0;
        pos.covariance[4] = pow(0.01 / pos.reliability, 2.0);
        pos.covariance[5] = 0.0;
        pos.covariance[6] = 0.0;
        pos.covariance[7] = 0.0;
        pos.covariance[8] = 10000.0;
        pos.initialization = 0;
        people_measurement_pub_.publish(pos);
		Yolo_iter=0;        
        }

        Yolo_iter++;
   }



   // std::cout<<"Yolo size : "<<people_size<<std::endl;
   //current yolo_people are saved to cur_yolo_people
}

*/


void Dynamic_Manager::filterhumanbelief()
{
	int human_index=0;
	Cur_existed_human.clear();
	int mapsize=Human_Belief_Scan_map.info.height*Human_Belief_Scan_map.info.width;
	// for(int i(0);i<mapsize;i++)
	// {
	// 	if(Human_Belief_Scan_map.data[i]>80.0)
	// 	{
	// 		std::vector<double> map_coord;
	// 		CellNum2globalCoord(i, map_coord);			
	// 		Cur_existed_human.push_back(map_coord);
	// 		// std::cout<<"cur_existed_human : "<<std::endl;
	// 	}
	// }


    int largestIndex = 0;
    for (int index = largestIndex; index < Human_Belief_Scan_map.data.size(); index++) {
        if (Human_Belief_Scan_map.data[largestIndex] < Human_Belief_Scan_map.data[index]) {
            largestIndex = index;
        }
    }
     std::vector<double> map_coord;
     CellNum2globalCoord(largestIndex, map_coord);
     
     m_boolSolve=false;
} 


/*
void Dynamic_Manager::human_leg_callback(const geometry_msgs::PoseArray::ConstPtr& msg)
{

  int num_leg_detected = msg->poses.size(); 
  std::vector<double> tempVec(2,0.0);
   Cur_leg_human.clear();
   human_occupied_leg_idx.clear();
  
  for(int i(0);i<num_leg_detected;i++)
      {
      	// printf("leg callback %d\n", i);
      	// listener.waitForTransform("head_rgbd_sensor_rgb_frame", "map", ros::Time(0), ros::Duration(1.0));
        listener.waitForTransform("base_range_sensor_link", "map", ros::Time(0), ros::Duration(2.0));
      	//transform frame from base_ranger_sensor_link to global map frame
        geometry_msgs::Vector3Stamped gV, tV;
        gV.vector.x = msg->poses[i].position.x;
        gV.vector.y = msg->poses[i].position.y;
        gV.vector.z = 1.0;
        gV.header.stamp = ros::Time();
        gV.header.frame_id = "/base_range_sensor_link";
        listener.transformVector("/map", gV, tV);
        tempVec[0]=tV.vector.x+global_pose[0];
        tempVec[1]=tV.vector.y+global_pose[1];
        //global position of candidates of leg position w.r.t map frame
        // if(check_staticObs(tempVec[0],tempVec[1]))
        //    continue;
        if(!check_cameraregion(tempVec[0],tempVec[1]))
            continue;
       
        // std::cout<<"index : "<<i<<std::endl;
        bool IsFarEnough = true;
        bool IsNearfromRobot= false;
        int IsCloseToYolo=0;

        //check if once deteced leg is not so near from new candidate
        for(int j(0);j<Cur_leg_human.size();j++)
        {
	         if(Comparetwopoistions(tempVec,Cur_leg_human[j],min_two_leg_dist))
    	       {
    	       	IsFarEnough=false;
    	       	break;
    	       	// std::cout<<"too close to other candidates "<<std::endl;
    	       }
	    }

	    if(Comparetwopoistions(global_pose,tempVec,LASER_range_person))
           IsNearfromRobot=true;
	    // std::cout<<"here 2"<<std::endl;
        
        if(IsNearfromRobot && IsFarEnough)
        { 
           Cur_leg_human.push_back(tempVec);
           int human_leg_mapidx=CoordinateTransform_Global2_beliefMap(tempVec[0],tempVec[1]);
	   	   human_occupied_leg_idx.push_back(human_leg_mapidx);
        }
      }

      if(OnceTarget)
      	Findlegfromtarget();
      else
		{ int a=0;
			// mixlegyolo();
		}
}

*/

// void Dynamic_Manager::Findlegfromtarget()
// {
// 	if(Cur_leg_human.size()>0)
//     {
//         for(int j(0);j<leg_targetSet.size();j++)
// 		{    
// 		    int NearestLegIdx=FindNearesetLegIdx_Target(j);
//             std::vector<double> temp_leg_target(2,0.0); 
// 	        temp_leg_target[0]=Cur_leg_human[NearestLegIdx][0];
// 	        temp_leg_target[1]=Cur_leg_human[NearestLegIdx][1];

// 	        if(Comparetwopoistions(temp_leg_target,leg_targetSet[j],0.8))
// 	        {
// 	            leg_targetSet[j][0]=temp_leg_target[0];
// 	            leg_targetSet[j][1]=temp_leg_target[1];
// 	        }
//         }

//    }
    
// // }


// int Dynamic_Manager::FindNearesetLegIdx_Target(int target_idx)
// {
//     //target should be already set
// 	//make sure that yolo size is bigger than index 
//     std::vector<double> Distanceset;
//     Distanceset.resize(Cur_leg_human.size(),0.0);
    
//     double minDistance=200.0;
//     int    minDistance_Idx=0;

//       for(int i(0);i<Cur_leg_human.size();i++)
//       {
//         // Distanceset[i]=getDistance(Cur_detected_human[i][0],Cur_detected_human[i][1]);
//         Distanceset[i]=getdistance(leg_targetSet[target_idx],Cur_leg_human[i]);
        
//         if(minDistance>Distanceset[i])
//           {
//             minDistance=Distanceset[i];
//             minDistance_Idx=i;
//           }
//       }
  
//     return minDistance_Idx;
// }


// int Dynamic_Manager::FindNearesetLegIdx(int yolo_idx)
// {
//     //target should be already set
// 	//make sure that yolo size is bigger than index 
//     std::vector<double> Distanceset;
//     Distanceset.resize(Cur_leg_human.size(),0.0);
    
//     double minDistance=200.0;
//     int    minDistance_Idx=0;

//       for(int i(0);i<Cur_leg_human.size();i++)
//       {
//         // Distanceset[i]=getDistance(Cur_detected_human[i][0],Cur_detected_human[i][1]);
//         Distanceset[i]=getdistance(cur_yolo_people[yolo_idx],Cur_leg_human[i]);
        
//         if(minDistance>Distanceset[i])
//           {
//             minDistance=Distanceset[i];
//             minDistance_Idx=i;
//           }
//       }
  
//     return minDistance_Idx;
// }


// void Dynamic_Manager::mixlegyolo(){

//     num_of_detected_human_yolo = cur_yolo_people.size();
// 	Filtered_leg_human.clear();

// 	for(int i(0);i<num_of_detected_human_yolo;i++)
// 	{
// 		int Nearest_leg_idx=FindNearesetLegIdx(i);

// 		if(Comparetwopoistions(Cur_leg_human[Nearest_leg_idx],cur_yolo_people[i],max_off_yolo_laser))
// 		{
// 			Filtered_leg_human.push_back(Cur_leg_human[Nearest_leg_idx]);

//           // string num_s = std::to_string(i);
//           // string person("person ");
//           // string new_str_name=person+num_s;
//           //  std::cout<<"person name = "<<new_str_name.c_str()<<std::endl;
		
//           // people_msgs::PositionMeasurement pos;
//           // pos.header.stamp = ros::Time();
//           // pos.header.frame_id = "/map";
//           // pos.name = "leg_laser";
//           // pos.object_id =new_str_name;
//           // pos.pos.x = Filtered_leg_human[i][0];
//           // pos.pos.y = Filtered_leg_human[i][1];
//           // pos.pos.z = 1.0;
//           // pos.reliability = 0.85;
//           // pos.covariance[0] = pow(0.01 / pos.reliability, 2.0);
//           // pos.covariance[1] = 0.0;
//           // pos.covariance[2] = 0.0;
//           // pos.covariance[3] = 0.0;
//           // pos.covariance[4] = pow(0.01 / pos.reliability, 2.0);
//           // pos.covariance[5] = 0.0;
//           // pos.covariance[6] = 0.0;
//           // pos.covariance[7] = 0.0;
//           // pos.covariance[8] = 10000.0;
//           // pos.initialization = 0;
//           // people_measurement_pub_.publish(pos);
//         }
// 	}

//    //  if(Cur_leg_human.size()>0){
//    //      for(int i(0);i<num_of_detected_human_yolo;i++)
//    //      {
//    //           for(int j(0);j<Cur_leg_human.size();j++)
//    //           {
//    //                 double distance =0.0;
//    //                 	getdistance(cur_yolo_people[i],Cur_leg_human[j])
//    //                 Comparetwopoistions()

    
//    //           } 
    
//    //      }

//    // }
// }
/*
void Dynamic_Manager::keyboard_callback(const keyboard::Key::ConstPtr& msg)
{
  ROS_INFO("Received Keyboard");
  printf("(key board)\n");
  int ReceivedNum= (int) msg->code;
  std::cout<<msg->code<<std::endl;
  leg_targetSet.clear();

  if(ReceivedNum==116)    //if keyboard input is "t"
  {
    if(cur_yolo_people.size()>0)
     {
     	  for(int j(0);j<cur_yolo_people.size();j++)
          {
          	  std::vector<double> temp_leg_target(2,0.0);
	          temp_leg_target[0]=cur_yolo_people[j][0];
	          temp_leg_target[1]=cur_yolo_people[j][1];
	          leg_targetSet.push_back(temp_leg_target);
	          OnceTarget=true;
	          Yolo_iter=0;
          	  std::cout<<"set target : "<<leg_targetSet[j][0]<<" , "<<leg_targetSet[j][1]<<std::endl;
          }
          // ROS_INFO("Filter : Set Target pos X : %.3lf, y : %.3lf", cur_yolo_people[0],cur_yolo_people[1]);
       }

  }
}

*/

bool Dynamic_Manager::Comparetwopoistions(std::vector<double> pos,std::vector<double> pos2, double criterion)
{
  //return true if there are in criterion distance 
  // double temp_dist=0.0;
  // for(int i(0);i<2;i++) 
  // {
  //   temp_dist+=pow((pos[i]-pos2[i]),2);
  // }

  // temp_dist=sqrt(temp_dist);

  double temp_dist=getdistance(pos,pos2);

  if(temp_dist<criterion)
    return true;
  

  return false;

}

void Dynamic_Manager::filter_result_callback(const people_msgs::PositionMeasurement::ConstPtr& msg)
{

   if(OnceTarget){
   if(msg->pos.x && msg->pos.y){
   filtered_target[0]=msg->pos.x;
   filtered_target[1]=msg->pos.y;

   setViewpointTarget(filtered_target);

   }
   }

}
void Dynamic_Manager::setViewpointTarget(const std::vector<double> pos)
{
    viewpub_iters++;
    if(viewpub_iters>20){
	    if(pos.size()<2)
	    {
		    return;
	    }
	    else
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
		        gV.vector.x = pos[0]-global_pose[0];
		        gV.vector.y = pos[1]-global_pose[1];
		        gV.vector.z = 1.0;

   		    // geometry_msgs::Pose origin_to_robot;
		    // origin_to_robot.position.x = transform_stamped.transform.translation.x;
		    // origin_to_robot.position.y = transform_stamped.transform.translation.y;
		     //    tf2_ros::Buffer tf_buffer_;
		 
		 //    geometry_msgs::TransformStamped transform_stamped;
		        tf::StampedTransform baselinktransform;
    			listener.waitForTransform("map", "base_range_sensor_link", ros::Time(0), ros::Duration(5.0));
                //listener.lookupTransform("map", "base_range_sensor_link", ros::Time(0), baselinktransform);

    		     gV.header.stamp = ros::Time();
		         gV.header.frame_id = "/map";
	    	    listener.transformVector("/base_range_sensor_link", gV, tV);

		        std::vector<double> tempVec(3,0.0);
    		    tempVec[0]=tV.vector.x;
    			tempVec[1]=tV.vector.y;
    			tempVec[2]=tV.vector.z;
                viewTarget[0]=tempVec[0];
                viewTarget[1]=tempVec[1];
	    		
	    		GazePoint_msg.x=tempVec[0];
		    	GazePoint_msg.y=tempVec[1];
    			GazePoint_msg.z=tempVec[2];
	    	}

            publish_viewpointTarget();
    		GazePoint_msg.z=1.0;
    		Gaze_point_pub.publish(GazePoint_msg);
    		// std::cout<<"gaze topic published"<<std::endl;

	    	std_msgs::Bool activateGaze_msg;
	    	activateGaze_msg.data=true;
	    	Gaze_activate_pub.publish(activateGaze_msg);
	    }
        viewpub_iters=0;
    }
    else{
    
    viewpub_iters++;
    return; 
    }
 
    return;

}

void Dynamic_Manager::publish_viewpointTarget(){

        visualization_msgs::Marker marker_humans;
        marker_humans.header.frame_id = "base_range_sensor_link";
        marker_humans.header.stamp = ros::Time::now();
        marker_humans.ns = "/view_target";
        marker_humans.id = 0;

        uint32_t shape = visualization_msgs::Marker::SPHERE;
        marker_humans.type = shape;

        marker_humans.pose.position.x = viewTarget[0];
        marker_humans.pose.position.y = viewTarget[1];
        marker_humans.pose.position.z = 1;

        marker_humans.pose.orientation.x = 0.0;
        marker_humans.pose.orientation.y = 0.0;
        marker_humans.pose.orientation.z = 0.0;
        marker_humans.pose.orientation.w = 1.0;

        double temp_dist,temp_dist2,temp_dist3;
        temp_dist  =0.5;
        marker_humans.scale.x = std::abs(temp_dist);
        marker_humans.scale.y = std::abs(temp_dist);
        marker_humans.scale.z = std::abs(temp_dist);

        marker_humans.color.r = 0.23;
        marker_humans.color.g = 0.9;
        marker_humans.color.b = 0.7;
        marker_humans.color.a = 0.85;


     viewTarget_visual_pub.publish(marker_humans);
}
void Dynamic_Manager::CellNum2globalCoord(const int Cell_idx, std::vector<double>& cell_xy)
{
	  cell_xy.resize(2,0.0);

	  int res =(int) Cell_idx / Human_Belief_Scan_map.info.width;
	  int div =(int) Cell_idx % Human_Belief_Scan_map.info.width;

	  cell_xy[0]=Human_Belief_Scan_map.info.resolution*div+0.5*Human_Belief_Scan_map.info.resolution+Human_Belief_Scan_map.info.origin.position.x;
	  cell_xy[1]=Human_Belief_Scan_map.info.resolution*res+0.5*Human_Belief_Scan_map.info.resolution+Human_Belief_Scan_map.info.origin.position.y;
}

void Dynamic_Manager::ClikedpointCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
	printf("Dynamic goal Receive point\n");
    GoalVector.resize(2,0);
    GoalVector[0]=msg->point.x;
	GoalVector[1]=msg->point.y;
    cur_coord.resize(2);
	Goal_Coord.resize(2);
	 
	CoordinateTransform_Rviz_Grid_Start(CurVector[0],CurVector[1],1);
	CoordinateTransform_Rviz_Grid_Goal(GoalVector[0],GoalVector[1],1);
	updateMap(m_dynamic_occupancy,cur_coord,Goal_Coord);

	 
    m_boolSolve=true;

    return;
}


// void Dynamic_Manager::publish_leg_boxes()
// {

//   human_leg_boxes_array.markers.clear();
//   if(Filtered_leg_human.size()>0)
//   {
//     for(int i(0);i<Filtered_leg_human.size();i++)
//     {
//         visualization_msgs::Marker marker_human_leg;
//         marker_human_leg.header.frame_id = "/map"; 
//         marker_human_leg.header.stamp = ros::Time::now();
//         marker_human_leg.ns = "/human_leg_boxes";
//         marker_human_leg.id = i;

//         uint32_t shape = visualization_msgs::Marker::SPHERE;
//         marker_human_leg.type = shape;

//         marker_human_leg.pose.position.x = Filtered_leg_human[i][0];
//         marker_human_leg.pose.position.y = Filtered_leg_human[i][1];
//         marker_human_leg.pose.position.z = 1;

//         marker_human_leg.pose.orientation.x = 0.0;
//         marker_human_leg.pose.orientation.y = 0.0;
//         marker_human_leg.pose.orientation.z = 0.0;
//         marker_human_leg.pose.orientation.w = 1.0;

//         double temp_dist,temp_dist2,temp_dist3;
//         temp_dist  =0.5;
//         temp_dist2 =0.5;
//         temp_dist3 =0.5;

//         //ROS_INFO("temp dist : %.3lf, temp dist2 : %.3lf, temp dist3 : %.3lf",temp_dist,temp_dist2,temp_dist3);
//         marker_human_leg.scale.x = std::abs(temp_dist);
//         marker_human_leg.scale.y = std::abs(temp_dist2);
//         marker_human_leg.scale.z = std::abs(temp_dist3);

//         marker_human_leg.color.r = 0.83;
//         marker_human_leg.color.g = 0.6;
//         marker_human_leg.color.b = 0.5;
//         marker_human_leg.color.a = 0.85;

//         human_leg_boxes_array.markers.push_back(marker_human_leg);
//       }

//       Leg_boxes_pub.publish(human_leg_boxes_array);
//   }
// }

void Dynamic_Manager::publish_filtered_human_boxes()
{

  filtered_humans_array.markers.clear();
  if(Cur_existed_human.size()>0)
  {
    for(int i(0);i<Cur_existed_human.size();i++)
    {
        visualization_msgs::Marker marker_humans;
        marker_humans.header.frame_id = "/map"; 
        marker_humans.header.stamp = ros::Time::now();
        marker_humans.ns = "/human_leg_boxes";
        marker_humans.id = i;

        uint32_t shape = visualization_msgs::Marker::SPHERE;
        marker_humans.type = shape;

        marker_humans.pose.position.x = Cur_existed_human[i][0];
        marker_humans.pose.position.y = Cur_existed_human[i][1];
        marker_humans.pose.position.z = 1;

        marker_humans.pose.orientation.x = 0.0;
        marker_humans.pose.orientation.y = 0.0;
        marker_humans.pose.orientation.z = 0.0;
        marker_humans.pose.orientation.w = 1.0;

        double temp_dist,temp_dist2,temp_dist3;
        temp_dist  =0.5;
        marker_humans.scale.x = std::abs(temp_dist);
        marker_humans.scale.y = std::abs(temp_dist);
        marker_humans.scale.z = std::abs(temp_dist);

        marker_humans.color.r = 0.23;
        marker_humans.color.g = 0.9;
        marker_humans.color.b = 0.7;
        marker_humans.color.a = 0.85;

        filtered_humans_array.markers.push_back(marker_humans);
      }

      Human_boxes_pub.publish(filtered_humans_array);
  }
}



void Dynamic_Manager::put_human_occ_map_leg()
{

	int human_map_legidx=0;
	human_occupied_leg_idx.clear();

	for(int i(0);i<Filtered_leg_human.size();i++)
	{ 
		human_map_legidx = CoordinateTransform_Global2_beliefMap(Filtered_leg_human[i][0],Filtered_leg_human[i][1]);
		human_occupied_leg_idx.push_back(human_map_legidx);
	}
	int num_size = human_occupied_leg_idx.size();
	if(num_size>0)
	{
		if(Human_Belief_Scan_map.data.size()>0){
			for(int i(0);i<human_occupied_leg_idx.size();i++){
			 	Human_Belief_Scan_map.data[human_occupied_leg_idx[i]]=40.0;
			 	put_human_surrounding_beliefmap(human_occupied_leg_idx[i],20.0);
			}
		}
	}
	else
	{


	}



	// if(Human_Belief_Scan_map.data.size()>0){
	// 	for(int i(0);i<human_occupied_leg_idx.size();i++){

	// 	 	Human_Belief_Scan_map.data[human_occupied_leg_idx[i]]=60.0;
	// 	}

	// }
}
int Dynamic_Manager::CoordinateTransform_Global2_beliefMap(double global_x, double global_y)
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
 	int belief_map_idx= human_coord[0]+Human_Belief_Scan_map.info.width*human_coord[1];

 	return belief_map_idx;
}

void Dynamic_Manager::put_human_occ_map_yolo()
{
	int human_mapidx=0;
	human_occupied_idx.clear();

	for(int i(0);i<cur_yolo_people.size();i++)
	{ 
		human_mapidx = CoordinateTransform_Global2_beliefMap(cur_yolo_people[i][0],cur_yolo_people[i][1]);
		human_occupied_idx.push_back(human_mapidx);
	}
	int num_size = human_occupied_idx.size();
	// std::cout<<"human_occupied_size"<<num_size<<std::endl;
	if(num_size>0)
	{
		if(Human_Belief_Scan_map.data.size()>0){
			for(int i(0);i<human_occupied_idx.size();i++){
			 	Human_Belief_Scan_map.data[human_occupied_idx[i]]=80.0;
			 	put_human_surrounding_beliefmap(human_occupied_idx[i],30.0);
			}
		}
	}
	else
	{


	}
}

void Dynamic_Manager::update_human_occ_belief_scan()
{
	int num_of_detected_human =cur_yolo_people.size();
	float prior=0.0;
	float posterior=0.0;
		
		// for(int i(0);i< visiblie_idx_set.size();i++){
		// 	int belief_map_index=visiblie_idx_set[i];
		// 	if( Human_Belief_Scan_map.data[belief_map_index]>0)	//If we detected human
		// 	{
		// 		prior =(float) Human_Belief_Scan_map.data[belief_map_index]/100.0; // P(H)
		// 		// float P_S = P_Sc_given_H*(prior) + P_Sc_given_Hc*(1-prior);
		// 		posterior = prior*0.99;
		// 		Human_Belief_Scan_map.data[belief_map_index] = posterior*100.0;
		// 	}
		// 	else
		// 	{
		// 		Human_Belief_Scan_map.data[belief_map_index]=0.0;
		// 	}
		// }
		

	int mapsize=Human_Belief_Scan_map.info.height*Human_Belief_Scan_map.info.width;
	for(int i(0);i<mapsize;i++)
	{
		if(NotUpdatedCameraregion(i))
		{
			prior =(float) Human_Belief_Scan_map.data[i]*100.0; // P(H)
			// float P_S = P_Sc_given_H*(prior) + P_Sc_given_Hc*(1-prior);
			posterior = prior*0.99;
			Human_Belief_Scan_map.data[i] =(float)  posterior/100.0;
		}
	}

	//filter for low belief
    for(int i(0);i<mapsize;i++)
	{
		if(Human_Belief_Scan_map.data[i]<15.0)
		{
			Human_Belief_Scan_map.data[i]=0.0;
		}

		if(Human_Belief_Scan_map.data[i]>100)
		{
			Human_Belief_Scan_map.data[i]=100.0;
		}

	}

	filterhumanbelief();





}


bool Dynamic_Manager::NotUpdatedCameraregion(int idx)
{

	for(int i(0);i<visiblie_idx_set.size();i++)
	{
		if(idx==visiblie_idx_set[i])
			return false;
	}

	return true;
}

void Dynamic_Manager::put_human_surrounding_beliefmap(int idx, double value)
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
		{	
			// std::cout<<"here"<<std::endl;
			Human_Belief_Scan_map.data[next_idx]=value;
		}
	}
	
}

void Dynamic_Manager::Publish_beliefmap()
{
		
     Human_Belief_Scan_map.info.width=15;
     Human_Belief_Scan_map.info.height= 15;
     Human_Belief_Scan_map.info.resolution=0.5;
     double belief_map_offset =Human_Belief_Scan_map.info.width*Human_Belief_Scan_map.info.resolution;
     Human_Belief_Scan_map.info.origin.position.x=CurVector[0]-0.5*belief_map_offset-0.5*Human_Belief_Scan_map.info.resolution;
     Human_Belief_Scan_map.info.origin.position.y=CurVector[1]-0.5*belief_map_offset-0.5*Human_Belief_Scan_map.info.resolution;
     Human_Belief_Scan_map.data.resize(Human_Belief_Scan_map.info.width*Human_Belief_Scan_map.info.height,0.0);

     getCameraregion();
     put_human_occ_map_leg();
     put_human_occ_map_yolo();
     update_human_occ_belief_scan();

     Human_Belief_Scan_map.header.stamp =  ros::Time::now();
     Human_Belief_Scan_map.header.frame_id = "map"; 
     belief_pub.publish(Human_Belief_Scan_map);

}

// void Dynamic_Manager::Basepos_Callback(const geometry_msgs::PointStamped::ConstPtr& msg)
// {
//   ROS_INFO("base position msg");
//   printf("Map origin x index is %.3f, y index is %.3f \n",Map_orig_Vector[0],Map_orig_Vector[1]); 
// }


void Dynamic_Manager::dynamic_mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	//ROS_INFO("I am calling");
	double small_pos_x, small_pos_y=0.0;
	double dist_x,dist_y=0.0;
	int map_coord_i,map_coord_j=0;
	int numcount=0;
	int	original_width=msg->info.width;			//140
	int	original_height= msg->info.height;		//140
	double original_x=msg->info.origin.position.x;
	double original_y=msg->info.origin.position.y;
	double oroginal_res=0.05;					//0.05

	//for dynamic map space map
	Scaled_dynamic_map.info.width=10;
	Scaled_dynamic_map.info.height= 10;
	Scaled_dynamic_map.info.resolution=0.75;
	Scaled_dynamic_map.info.origin.position.x=CurVector[0]-DYN_OFFSET_X-0.5*Scaled_dynamic_map.info.resolution;
	Scaled_dynamic_map.info.origin.position.y=CurVector[1]-DYN_OFFSET_Y-0.5*Scaled_dynamic_map.info.resolution;
	// Scaled_dynamic_map.info.origin.position.x=CurVector[0]-0.5*Scaled_dynamic_map.info.width*0.5;
	// Scaled_dynamic_map.info.origin.position.y=CurVector[1]-0.5*Scaled_dynamic_map.info.height*0.5;
	Scaled_dynamic_map.data.resize(Scaled_dynamic_map.info.width*Scaled_dynamic_map.info.height);


	Scaled_dynamic_map_path.info.width=10;
	Scaled_dynamic_map_path.info.height= 10;
	Scaled_dynamic_map_path.info.resolution=0.75;
	Scaled_dynamic_map_path.info.origin.position.x=CurVector[0]-DYN_OFFSET_X-0.5*Scaled_dynamic_map.info.resolution;
	Scaled_dynamic_map_path.info.origin.position.y=CurVector[1]-DYN_OFFSET_Y-0.5*Scaled_dynamic_map.info.resolution;








   double base_origin_x =msg->info.origin.position.x;
   double base_origin_y =msg->info.origin.position.y;

	std::map<int,int> occupancyCountMap;
    int scaled_res=10;
    int map_idx=0;
    int scaled_result=0;

   for(int j(0);j<Scaled_dynamic_map.info.height;j++)
   	for(int i(0);i<Scaled_dynamic_map.info.width;i++)
   	{
   		map_idx=j*Scaled_dynamic_map.info.height+i;

   		//get global coordinate
   		double pos_x=i*Scaled_dynamic_map.info.resolution+Scaled_dynamic_map.info.origin.position.x;
   		double pos_y=j*Scaled_dynamic_map.info.resolution+Scaled_dynamic_map.info.origin.position.y;

   		numcount=0;
   		for(int k(0);k<scaled_res;k++)
   			for(int j(0);j<scaled_res;j++)
   			{
   				small_pos_x=pos_x+j*oroginal_res;
   				small_pos_y=pos_y+k*oroginal_res;
   				dist_x= small_pos_x-original_x;
				dist_y= small_pos_y-original_y;
				map_coord_i=floor(dist_x/oroginal_res);
				map_coord_j=floor(dist_y/oroginal_res);
				
				//static_map_ref_index
				int map_data_index=original_width*map_coord_j+map_coord_i;
				float temp_occupancy= msg->data[map_data_index];
    			 if(temp_occupancy>0)
				 	numcount++;
   			}

   			if(numcount>35)
   				scaled_result=50;
   			else
   				scaled_result=0;

   			Scaled_dynamic_map.data[map_idx]=scaled_result;
   	}

   //insert mdp_path_cell


   // insert human occupied cell
   //std::vector<int> HumanMa

   	//remove human occupied cell
       //std::vector<double> HumanCoord(2,0);
       //HumanCoord[0] = 0.6;
       //HumanCoord[1] = 1.2;
       //std::vector<double> HumanCoord2(2,0);
       //HumanCoord2[0] = 2.6;
       //HumanCoord2[1] = 0.9;
       //std::vector<std::vector<double> > Human_Coord_set;
       //Human_Coord_set.push_back(HumanCoord);
       //Human_Coord_set.push_back(HumanCoord2);

       //for(int i(0);i<Human_Coord_set.size();i++)
	//{
		   //Scaled_dynamic_map.data[Coord2CellNum(Human_Coord_set[i])]=90.0;
		
	//}

     //find index from
	 Scaled_dynamic_map.header.stamp =  ros::Time::now();
	 Scaled_dynamic_map.header.frame_id = "map"; 
     Scaled_dynamic_map_pub.publish(Scaled_dynamic_map);

    for(int i(0);i<m_dynamic_occupancy.size();i++)
    	m_dynamic_occupancy[i]=Scaled_dynamic_map.data[i];

}




void Dynamic_Manager::static_mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	// ROS_INFO("staticmap callback start");

	double small_pos_x, small_pos_y=0.0;
	double dist_x,dist_y=0.0;
	int map_coord_i,map_coord_j=0;
	int numcount=0;
	int	original_width=msg->info.width;
	int	original_height= msg->info.height;
	double original_x=-51.225;
	double original_y=-51.225;;
	double oroginal_res=0.05;

	//for static space map
	Scaled_static_map.info.width=Grid_Num_X;
	Scaled_static_map.info.height= Grid_Num_Y;
	Scaled_static_map.info.resolution=0.5;
	Scaled_static_map.info.origin.position.x=-4;
	Scaled_static_map.info.origin.position.y=-4;
	Scaled_static_map.data.resize(Scaled_static_map.info.width*Scaled_static_map.info.height);

	// if(msg->data[0]!=NULL)
	// {
		int datasize=msg->data.size();
		// ROS_INFO("staticmap size : %d",datasize);
		// for(int z(0);z<original_width*original_height;z++)
		// 	std::cout<<msg->data[z]<<std::endl;
	
	//for path map
	// Scaled_static_map_path.info.width=32;
	// Scaled_static_map_path.info.height= 32;
	// Scaled_static_map_path.info.resolution=0.5;
	// Scaled_static_map_path.info.origin.position.x=-4;
	// Scaled_static_map_path.info.origin.position.y=-4;
	//Scaled_static_map_path.data.resize(32*32);

   double base_origin_x =msg->info.origin.position.x;
   double base_origin_y =msg->info.origin.position.y;

	std::map<int,int> occupancyCountMap;
    int scaled_res=10;
    int map_idx=0;
    int scaled_result=0;

   for(int j(0);j<Scaled_static_map.info.height;j++)
   	for(int i(0);i<Scaled_static_map.info.width;i++)
   	{
   		map_idx=j*Scaled_static_map.info.height+i;
   		double pos_x=i*Scaled_static_map.info.resolution+Scaled_static_map.info.origin.position.x;
   		double pos_y=j*Scaled_static_map.info.resolution+Scaled_static_map.info.origin.position.y;

   		numcount=0;
   		for(int k(0);k<scaled_res;k++)
   			for(int j(0);j<scaled_res;j++)
   			{
   				small_pos_x=pos_x+j*oroginal_res;
   				small_pos_y=pos_y+k*oroginal_res;
   				dist_x= small_pos_x-original_x;
				dist_y= small_pos_y-original_y;
				map_coord_i=floor(dist_x/oroginal_res);
				map_coord_j=floor(dist_y/oroginal_res);
				
				int map_data_index=original_width*map_coord_j+map_coord_i;
				float temp_occupancy= msg->data[map_data_index];
    			 if(temp_occupancy>0)
				 	numcount++;
   			}

   			if(numcount>5)
   				scaled_result=50;
   			else
   				scaled_result=0;

   			Scaled_static_map.data[map_idx]=scaled_result;
   	}

     //find index from
	 Scaled_static_map.header.stamp =  ros::Time::now();
	 Scaled_static_map.header.frame_id = "map"; 
     Scaled_static_map_pub.publish(Scaled_static_map);

    //m_localoccupancy.resize(Scaled_static_map.data.size());
    //for(int i(0);i<m_localoccupancy.size();i++)
        //m_localoccupancy[i]=Scaled_static_map.data[i];

	}

   // ROS_INFO("staticmap callback start");

// }


void Dynamic_Manager::global_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	global_pose[0]=msg->pose.position.x;
	global_pose[1]=msg->pose.position.y;


   tf::StampedTransform baselinktransform;
   listener.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(4.0));
   listener.lookupTransform("map", "base_link", ros::Time(0), baselinktransform);
   double yaw_tf =   tf::getYaw(baselinktransform.getRotation()); 

	global_pose[2]=yaw_tf;

   CurVector[0]= global_pose[0];
   CurVector[1]= global_pose[1];
   CurVector[2]= global_pose[2];

}


// void Dynamic_Manager::Local_mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
// {

// 	 if(ReceiveData==0)
//     {
//       ROS_INFO("width : %d" ,msg->info.width);
//       ROS_INFO("height : %d" ,msg->info.height);
//       ROS_INFO("resolution : %lf" ,msg->info.resolution);
//       ReceiveData++;

//       pMapParam->Num_grid_X=(int) msg->info.width;
//       pMapParam->Num_grid_Y=(int) msg->info.height;
//       pMapParam->map_step= (double) msg->info.resolution;  

//     }

//    double base_origin_x =msg->info.origin.position.x;
//    double base_origin_y =msg->info.origin.position.y;

//    //ROS_INFO("origin x: %lf, y : %lf",base_origin_x,base_origin_y);
//    //ROS_INFO("int msg");
//    //  m_localoccupancy.resize(msg->data.size());

//    // for(int i(0);i<msg->data.size();i++){
//    //      m_localoccupancy[i]=(msg->data)[i];
//    // }	
//    ReceiveData++;


// }


void Dynamic_Manager::Global2MapCoord(const vector<double>& _globalcoord,vector<int>& _MapCoord)
{
 
 _MapCoord.resize(2);
    
  _MapCoord[0]= (int)_globalcoord[0]/(pMapParam->map_step);
  _MapCoord[1]= (int)_globalcoord[1]/(pMapParam->map_step);

  //std::cout<<"x:"<<MapCoord[0]<<" , "<<"y:"<<MapCoord[1]<<std::endl;

  return ;
}


void Dynamic_Manager::Publish_filter_measurment(int measurement_type)
{
	//double pos_x = 0.0;
	//double pos_y = 0.0;

     //std::vector< std::vector<double> > targetHumans = Filtered_leg_human;

	//switch(measurement_type){
		//case 0:
			//pos_x =0.0;
			//pos_y =0.0; 
			//break;
		//case 1:
			//pos_x =0.0;
			//pos_y =0.0;
			//break;
		//case 2:
			//pos_x = 0.0;
			//pos_y = 0.0;
			//break;
		//default: 
			//pos_x = 0.0;
			//pos_y = 0.0;
			
	//}

	//for(int idx(0);idx<leg_targetSet.size();idx++){
        //string num_s = std::to_string(idx);
        //string person("person ");
        //string new_str_name=person+num_s;
         //std::cout<<"person name = "<<new_str_name.c_str()<<std::endl;
		
        //people_msgs::PositionMeasurement pos;
        //pos.header.stamp = ros::Time();
        //pos.header.frame_id = "/map";
        //pos.name = "leg_laser";
        //pos.object_id =new_str_name;
        //pos.pos.x = leg_targetSet[idx][0];
        //pos.pos.y = leg_targetSet[idx][1];
        //pos.pos.z = 1.0;
        //pos.reliability = 0.85;
        //pos.covariance[0] = pow(0.01 / pos.reliability, 2.0);
        //pos.covariance[1] = 0.0;
        //pos.covariance[2] = 0.0;
        //pos.covariance[3] = 0.0;
        //pos.covariance[4] = pow(0.01 / pos.reliability, 2.0);
        //pos.covariance[5] = 0.0;
        //pos.covariance[6] = 0.0;
        //pos.covariance[7] = 0.0;
        //pos.covariance[8] = 10000.0;
        //pos.initialization = 0;
        //people_measurement_pub_.publish(pos);
        //}


}


void Dynamic_Manager::updateMap(vector<int>& localmap_, vector<int>& local_start, vector<int>& local_goal)
{
	m_static_obs.clear();
	Policies.clear();	
	Rewards.clear();
	U.clear();
	MdpSols.clear();
	PolicyNum.clear();

	Policies.resize(pMapParam->MapSize, '0');
 	Rewards.resize(pMapParam->MapSize, Ra); 
 	U.resize(pMapParam->MapSize, 0.0); 
 	Up.resize(pMapParam->MapSize, 0.0); 
 	MdpSols.resize(pMapParam->MapSize, 0);
 	PolicyNum.resize(pMapParam->MapSize, 0);

 	m_Start.resize(2,0);	
    m_Goal.resize(2,0);							
 	m_Robot.resize(2,0);

 	m_Start[0]=cur_coord[0];
 	m_Start[1]=cur_coord[1];

	m_Goal[0]=Goal_Coord[0];
 	m_Goal[1]=Goal_Coord[1];

	// cout<<" Start pos - X :"<<m_Start[0]<<", - Y : "<<m_Start[1]<<endl;
	// cout<<" Goal pos - X :"<<m_Goal[0]<<", - Y : "<<m_Goal[1]<<endl;

 	for(int i(0);i<localmap_.size();i++)
	{
		if(localmap_[i]>2.0 )
			m_static_obs.push_back(i);
	}

	vector<int> tempcoord(2,0);
	for(int i(0);i<m_static_obs.size();i++){
 		Rewards[m_static_obs[i]]=0.0;
 		Policies[m_static_obs[i]]='#';
 	}


	Rewards[Coord2CellNum(m_Goal)]=100;
 	Policies[Coord2CellNum(m_Goal)]='+';

 	printf("Update good!\n");
}


vector<int> Dynamic_Manager::Global2LocalCoord(vector<int> Global_coord)
{
	vector<int> Local_coords(2,0);
	
	Local_coords[0]=Global_coord[0]-Local_X_start;
	Local_coords[1]=Global_coord[1]-Local_Y_start;

	return Local_coords;
}

void Dynamic_Manager::setStartConfig( const vector<int> _Start)
{
		m_Start[0]=_Start[0];
		m_Start[1]=_Start[1];
}

void Dynamic_Manager::setGoalConfig( const vector<int> _Goal )
{
		m_Goal[0]=_Goal[0];
		m_Goal[1]=_Goal[1];

}

int  Dynamic_Manager::Coord2CellNum(std::vector<int> cell_xy)
{
	int index= cell_xy[0]+X_mapSize*cell_xy[1];


	return index;
}

void Dynamic_Manager::CellNum2Coord(const int Cell_idx, vector<int>& cell_xy)
{
	  cell_xy.resize(2,0);

	  int res =(int) Cell_idx / X_mapSize;
	  int div =(int) Cell_idx % X_mapSize;

	  cell_xy[0]=div;
	  cell_xy[1]=res;
}

bool Dynamic_Manager::MDPsolve()
{
	ROS_INFO("Solve");
    //why colNum==6?
    int colNum=6;
	int idx=0;

	Points.clear();
	Points.resize(colNum);
	//Check for row-wise or column-wise
	
	for(int j(0);j<Y_mapSize;j++){
		for(int i(0);i<X_mapSize;i++)	
		{
			Points[0].push_back(i);
			Points[1].push_back(j);
			Points[2].push_back((int)Rewards[idx]);
			//Points[2].push_back((int)R(i,j));
			Points[3].push_back(0);			//Up
			Points[4].push_back(0);			//U
			//Points[5].push_back(PiNums(i,j));
			Points[5].push_back(PolicyNum[idx]);
			idx++;
		}
	}	


	int 	iters = 0;
	double  diff  = 0.0;
	double  delta = 0.0;
	
	while(1)
	{
		if(Up.size()==U.size()){
			//cout<<"backup Up"<<endl;
			for(int i(0);i<Up.size();i++){
				U[i]=Up[i];
			}
		}
		
		for(int k(0);k<Num_Grids;k++){
			updateUprimePi(k);				//update Up and Policy, PolicyNum
			diff = abs(Up[k]-U[k]);

			//check maxim error for whole states
			if(diff > delta)
				delta=diff;
		}

		//terminal condition
		if(delta<deltaMin){
			cout<<"error converged"<<endl;
			break;
		}
		else if(iters>maxiter){
			cout<<"max iterations"<<endl;
			break;
		}
	
		iters++;
	}
	return false;
}

void Dynamic_Manager::printPath()
{
	for(int i(Y_mapSize-1);i>-1;i--){
		for(int j(0);j<X_mapSize;j++){

		int pos = i * X_mapSize + j;
		cout<<Rewards[pos]<<",";
		//cout<<Policies[pos];
		}
		cout<<endl;
	}

}

void Dynamic_Manager::setStaticObs(const vector<int> static_obs)
 {
 	for(int i(0);i<static_obs.size();i++){
 		pMapParam->OCC_Info[static_obs[i]]=St_OBS_CELL;
 	}

 }

 void Dynamic_Manager::setDynamicObs(const vector<int> dynamic_obs){

	for(int i(0);i<dynamic_obs.size();i++){
 		pMapParam->OCC_Info[dynamic_obs[i]]=Dy_OBS_CELL;
 	}

 }

 void Dynamic_Manager::setHumanObs(const vector<int> humans){

	for(int i(0);i<humans.size();i++){
	 		pMapParam->OCC_Info[humans[i]]=Human_CELL;
	 	}
 }

char Dynamic_Manager::getPolicychar(int policyidx)
{
	char policychars;
	switch(policyidx)
	{

		case 0:
			policychars='E';				break;
		case 1:
			policychars='R';				break;
		case 2:
			policychars='N';				break;
		case 3:
			policychars='Q';				break;
		case 4:
			policychars='W';				break;
		case 5:	
			policychars='Z';				break;
		case 6:
			policychars='S';				break;
		case 7:
			policychars='C';				break;
		default: 
			policychars='X';
	}

	return policychars;
}

void Dynamic_Manager::updateUprimePi(int state_id)
{
	vector<int> curpos;
	CellNum2Coord(state_id,curpos);
	int x_pos=curpos[0];
	int y_pos=curpos[1];
	// cout<<"---------------------"<<endl;
	// cout<<"state_id:"<<state_id<<"x pos :"<<x_pos<<", y pos :"<<y_pos<<endl;
	// cout<<"---------------------"<<endl;

	//for(int i(0);i<Num_Grids;i++)
	//cout<<"point[0][k] :"<<Points[0][i]<<", point[1][k] :"<<Points[1][i]<<endl;
	//cout<<"--------------num of grid :"<<Num_Grids<<endl;
	//cout<<"Update for Uprime : state_id :"<<state_id<<","<<x_pos<<","<<y_pos<<endl;
	
	for(int k(0);k<Num_Grids;k++){

		if(Points[0][k]==x_pos && Points[1][k]==y_pos){
			
			//Use bellman equation "computed using U(s), not using U'(s)"
			if(Rewards[k]!=Ra){	//Up(k)~=Ra)
				Up[k]=Rewards[k];
			}
			else{
				//cout<<"k:"<<k<<"x pos :"<<x_pos<<", y pos :"<<y_pos<<endl;	
				//get action index and the maximum reward value corresponding that action index
				map<int,double> maxmap;
				getMaxValueAction(x_pos,y_pos,maxmap);
				map<int,double>::iterator mnaxmapiter=maxmap.begin();
				

				//cout<<"maxvalue :"<<mnaxmapiter->second<<endl;
				//Update Uprime and Policyvector and policiesNumvector 
				// new_up_pi=[Up,Pi,PiNums] in matlab
				Up[k]=Rewards[k]+gamma*(mnaxmapiter->second);
				PolicyNum[k]=(mnaxmapiter->first);
				Policies[k]=getPolicychar(mnaxmapiter->first);

				//cout<<"k:"<<k<<", Up :"<<Up[k]<<", Pi[k] :"<<PolicyNum[k]<<endl;

			}
		}
	}
}

//returns true if cur_pos in inside a map // false is collision
bool Dynamic_Manager::checkNoBoundary(vector<int> cur_pos)
{
	int x_pos=cur_pos[0];
	int y_pos=cur_pos[1];

	if((x_pos<0) || (x_pos>X_mapSize-1))
		return false;
	if((y_pos<0) || (y_pos>Y_mapSize-1))
		return false;
		
	return true;

}
	
//returns true if cur_pos is safe (No collision with static obs, ex) Collision : false, NO collision : true
bool Dynamic_Manager::checkStaticObs(vector<int> cur_pos)
{
	// Input:current position 
	// Output : True or false

	int x_pos=cur_pos[0];
	int y_pos=cur_pos[1];

	for(int i(0);i<m_static_obs.size();i++)
	{
		vector<int> static_obs_pos;
		CellNum2Coord(m_static_obs[i],static_obs_pos);

		if((x_pos==static_obs_pos[0]) && (y_pos==static_obs_pos[1]))
			return false;
	}
	//cout<<"No static Obs"<<endl;
	return true;
}



bool Dynamic_Manager::checkObs(int cur_stid,int actionNum)
{
	// vector<int> cur_pos(2,0);
	// cur_pos=CellNum2Coord(cur_stid);
	// vector<int> next_pos(2,0);

	// next_pos[0]= cur_pos[0]+ActionCC[actionNum][0];
	// next_pos[1]= cur_pos[1]+ActionCC[actionNum][1];

	// cout<<"cur_pos: X : "<<cur_pos[0]<<", Y : "<<cur_pos[1]<<endl;
	// cout<<"action num : "<<actionNum<<endl;
	// cout<<"next_pos: X : "<<next_pos[0]<<", Y : "<<next_pos[1]<<endl;

	// if(checkNOBoundary(next_pos) && checkStaticObs(next_pos))
	// {
	// 	int next_pos_id=Coord2CellNum(next_pos);


	// }

}


double Dynamic_Manager::getactionvalue(int x_pos, int y_pos, int action_ix)
{
	//Get Current pos coordinate
	vector<int> cur_pos(2,0);
	cur_pos[0]=x_pos;
	cur_pos[1]=y_pos;
	
	int cur_st_id=Coord2CellNum(cur_pos);

	//Get next pos coordinate
	vector<int> next_pos(2,0);
	next_pos[0]= x_pos+ActionCC[action_ix][0];
	next_pos[1]= y_pos+ActionCC[action_ix][1];

	 //cout<<"cur st_id :"<<cur_st_id<<endl;
	//  cout<<"cur_pos: X : "<<x_pos<<", Y : "<<y_pos<<endl;
	//  cout<<"action num : "<<action_ix<<endl; 
	//  cout<<"next_pos: X : "<<next_pos[0]<<", Y : "<<next_pos[1]<<endl;

	//Cehck obstacles
	//check static obstacle
	if(checkNoBoundary(next_pos) && checkStaticObs(next_pos))
	{
		//cout<<"next_pos: X : "<<next_pos[0]<<", Y : "<<next_pos[1]<<endl;
		//cout<<"No collision"<<endl;
		int next_pos_id=Coord2CellNum(next_pos);
		return U[next_pos_id];
		//Return U
	}
	else{
		//cout<<"collision"<<endl;
		//cout<<"cur_st_id : "<<cur_st_id<<endl;
		return U[cur_st_id];
	}
}

vector<int> Dynamic_Manager::getneighboractionset(int action_idx)
{
	vector<int> neighboractionset(2,0);

	if(action_idx==0){
		neighboractionset[0]=(action_idx+1);
		neighboractionset[1]=7;	
	}
	else if(action_idx==7){
		neighboractionset[0]=0;
		neighboractionset[1]=6;	
	}
	else{
		neighboractionset[0]=(action_idx+1)%Action_dim;
		neighboractionset[1]=(action_idx-1)%Action_dim;
	}

	//cout<<"id:"<<action_idx<<", x :"<<neighboractionset[0]<<", y:"<<neighboractionset[1]<<endl;
	return neighboractionset;
}

int Dynamic_Manager::FindMaxIdx(vector<double> dataset)
{
	map<int,double> MaxIndex;
	MaxIndex.insert(make_pair(0,dataset[0]));
	int maxIndex=0;

	map<int,double>::iterator mapIter = MaxIndex.begin();

	for(int i=1;i<dataset.size();i++)
		{
			mapIter = MaxIndex.begin();		
			
			if(mapIter->second<dataset[i])
			{
				MaxIndex.clear();
				MaxIndex.insert(make_pair(i,dataset[i]));
				maxIndex=i;
			}
	}
	return maxIndex;

}
void Dynamic_Manager::Mapcoord2GlobalCoord(const vector<int>& _Mapcoord, vector<double>& GlobalCoord)
{

	GlobalCoord.resize(2);
	//globalCoord origin x, y;
	GlobalCoord[0]=Scaled_static_map.info.origin.position.x+Scaled_static_map.info.resolution*_Mapcoord[0]+0.5*Scaled_static_map.info.resolution;
	GlobalCoord[1]=Scaled_static_map.info.origin.position.y+Scaled_static_map.info.resolution*_Mapcoord[1]+0.5*Scaled_static_map.info.resolution;

	GlobalCoord[0]=Scaled_static_map.info.origin.position.x+Scaled_static_map.info.resolution*_Mapcoord[0]+0.5*Scaled_static_map.info.resolution;
	GlobalCoord[1]=Scaled_static_map.info.origin.position.y+Scaled_static_map.info.resolution*_Mapcoord[1]+0.5*Scaled_static_map.info.resolution;

}

void Dynamic_Manager::Mapcoord2DynamicCoord(const vector<int>& _Mapcoord, vector<double>& dynamicCoord)
{
	dynamicCoord.resize(2);
	dynamicCoord[0]=Scaled_dynamic_map.info.origin.position.x+Scaled_dynamic_map.info.resolution*_Mapcoord[0]+0.5*Scaled_dynamic_map.info.resolution;
	dynamicCoord[1]=Scaled_dynamic_map.info.origin.position.y+Scaled_dynamic_map.info.resolution*_Mapcoord[1]+0.5*Scaled_dynamic_map.info.resolution;
}


//get action(best) in Matlab
void Dynamic_Manager::getMaxValueAction(int x_pos, int y_pos,map<int,double>& maxmap)
{
	double bestvalue=0.0;
	vector<double> action_value(Action_dim,0.0);
	vector<double> action_valueSum(Action_dim,0.0);
	vector<int>    neighbor_action(2,0);
	
	for(int i(0);i<Action_dim;i++)
	{
		action_value[i]=getactionvalue(x_pos, y_pos,i);
		//cout<<"action value:"<<action_value[i]<<endl;
	}

	for (int i(0);i<Action_dim;i++)
	{
		action_valueSum[i]=Prob_good*action_value[i];
		neighbor_action=getneighboractionset(i);

		for(int j(0);j<2;j++)
			action_valueSum[i]+=Prob_bad*action_value[neighbor_action[j]];			
	}
	//cout<<"Here 3"<<endl;
	int maxIdx=FindMaxIdx(action_valueSum);
	bestvalue = action_valueSum[maxIdx];
	// for(int z(0);z<action_valueSum.size();z++)
	// 	cout<<"idx :"<<z<<", value :"<<action_valueSum[z]<<endl;
	// cout<<"maxidx : "<<maxIdx<<", maxvalue: "<<bestvalue<<endl;
	maxmap.clear();
	maxmap.insert(make_pair(maxIdx,bestvalue));
	return;
}

//Function for generating path
void Dynamic_Manager::pathPublish(){
	// std_msgs::Int32MultiArray obsmap_msg;
	// obsmap_msg.data = m_static_obs;
	// obsmap_Pub.publish(obsmap_msg);
	// MDPPath.clear();
}
//Generate dynamic path from solution

//FixMe
void Dynamic_Manager::Publish_dynamicPath()
{
	// nav_msgs::Path path ;
 //  	path.header.frame_id = "map";
 //  	geometry_msgs::PoseStamped pose;

 //    // // pose.pose.position.x = 8.2;
 //    // //  pose.pose.position.y = 0.65;
 //    // //  printf("spline path index : %d, x coord : %lf , y coord : %lf \n", i,pose.pose.position.x,pose.pose.position.y);
 //    // //  pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.05);
 //    // //  path.poses.push_back(pose);
 //    // pose.pose.position.x = 7.2;
 //    // pose.pose.position.y = 2.3;
 //    // // printf("spline path index : %d, x coord : %lf , y coord : %lf \n", i,pose.pose.position.x,pose.pose.position.y);
 //    // pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.15);
 //    // path.poses.push_back(pose);

 //    // pose.pose.position.x = 8.9;
 //    // pose.pose.position.y = 6.5;
 //    // // printf("spline path index : %d, x coord : %lf , y coord : %lf \n", i,pose.pose.position.x,pose.pose.position.y);
 //    // pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.65);
 //    // path.poses.push_back(pose);

	
	//  SplinePath_pub2.publish(path);




}

void Dynamic_Manager::generate_dynamicPath()
{
	vector<int> cur_pos(2,0);
	for(int i(0);i<2;i++)
		cur_pos[i]=m_Start[i];

	Dyn_MDPPath.clear();
	int cur_stid=Coord2CellNum(m_Start);
	int goal_stid=Coord2CellNum(m_Goal);

	if((goal_stid>pMapParam->MapSize) || (goal_stid<0))
		return;

	vector<double> t_values;
	vector<double> x_values;
	vector<double> y_values;
	vector<double> dyn_global_coords;

	//convert m_start to dynamic_coordinate
	Mapcoord2DynamicCoord(m_Start,dyn_global_coords);

	x_values.push_back(dyn_global_coords[0]);
	y_values.push_back(dyn_global_coords[1]);
	cout<<"cur st id : "<<cur_stid<<endl;
	
	Dyn_MDPPath.push_back(cur_stid);
	int pathcount=0;

	while(1)
	{
		cur_stid=Coord2CellNum(cur_pos);
		if(cur_stid==goal_stid)
			break;
		//get next position from policy solution
		for(int i(0);i<2;i++)
			cur_pos[i]+=ActionCC[PolicyNum[cur_stid]][i];

		Mapcoord2DynamicCoord(cur_pos,dyn_global_coords);
		ROS_INFO("cur pos  : %d, x:  %d, y // global goal x : %.3lf , global goal y : %.3lf \n",cur_pos[0], cur_pos[1],dyn_global_coords[0],dyn_global_coords[1]);
		x_values.push_back(dyn_global_coords[0]);
		y_values.push_back(dyn_global_coords[1]);
		cur_stid=Coord2CellNum(cur_pos);
		Dyn_MDPPath.push_back(cur_stid);

		//ROS_INFO("Id : %d, x:  %d, y : %d \n",cur_stid, cur_pos);

		if(pathcount>20)
			return;

		pathcount++;
		
	}



	//Making Spline path============================================
	int data_size=x_values.size();
	for(int k=0;k<data_size;k++)
		printf("spline path index : %d, x_values : %lf , y values : %lf \n", k,x_values[k],y_values[k]);

	//Making t-vetors;
	t_values.resize(x_values.size());
	//t_values[0]=0;
	double time_length=1.0;
	double time_const=(time_length)/(data_size-1);
	for(int k(0);k<data_size;k++)
		t_values[k]=k*time_const;
	
	m_CubicSpline_x = new srBSpline;
	m_CubicSpline_x->_Clear();

	m_CubicSpline_y = new srBSpline;
	m_CubicSpline_y->_Clear();

	vector<double> Spline_x;
	vector<double> Spline_y;
	m_CubicSpline_x->CubicSplineInterpolation(t_values,x_values,x_values.size());
	m_CubicSpline_y->CubicSplineInterpolation(t_values,y_values,y_values.size());

	int path_size=5;
	if(data_size<4)
		path_size=2;

	double const_path=(time_length)/ path_size ;

	Spline_x.push_back(x_values[0]);
	Spline_y.push_back(y_values[0]);

	double ret_x=0.0;
	double ret_y=0.0;
	double t_idx=0.0;
	 for(int j(0);j<path_size;j++){
  	   t_idx=(j+1)*const_path;
       m_CubicSpline_x->getCurvePoint(ret_x,t_idx);
       m_CubicSpline_y->getCurvePoint(ret_y,t_idx);

       if(!std::isnan(ret_x))
       	Spline_x.push_back(ret_x);
       if(!std::isnan(ret_y))
       	Spline_y.push_back(ret_y);
     }
    
    //Publish SPline Path
  	nav_msgs::Path dynamicSplinePath;
  	dynamicSplinePath.header.frame_id = "map";
  	geometry_msgs::PoseStamped pose;
  	
  	for (int i = 0; i < Spline_x.size(); i++)
 	{
	    pose.pose.position.x=Spline_x[i];
	   	pose.pose.position.y=Spline_y[i];
	    // pose.pose.position.x=Spline_x[i]*0.25-3.5-0.5*0.25;
	   	// pose.pose.position.y=Spline_y[i]*0.25-3.5-0.5*0.25;
	   	printf("spline path2 index : %d, x coord : %lf , y coord : %lf \n", i,pose.pose.position.x,pose.pose.position.y);
	   	pose.pose.orientation = tf::createQuaternionMsgFromYaw(m_desired_heading);
	   	dynamicSplinePath.poses.push_back(pose);
	}

	 //SplinePath_pub.publish(path);
	 SplinePath_pub2.publish(dynamicSplinePath);
	
	// Publish static map_path_grid
	for(int j(0);j<Scaled_dynamic_map_path.data.size();j++)
	 {	
	 	Scaled_dynamic_map_path.data[j]=0.0;
	 }


	 for(int k(0); k<Dyn_MDPPath.size();k++)
	 	Scaled_dynamic_map_path.data[Dyn_MDPPath[k]]=90;

	 Scaled_dynamic_map_path.header.stamp =  ros::Time::now();
	 Scaled_dynamic_map_path.header.frame_id = "map"; 
     Scaled_dynamic_map_path_pub.publish(Scaled_dynamic_map_path);

    //MDP solution publsih
}


void Dynamic_Manager::publish_cameraregion()
{
   getCameraregion();
   camera_map.header.stamp =  ros::Time::now();
   camera_map.header.frame_id = "map"; 
   camera_map_pub.publish(camera_map);
}


void Dynamic_Manager::getCameraregion()
{

  double global_robot_x= global_pose[0];
  double global_robot_y= global_pose[1];
  double global_robot_theta = global_pose[2]+Head_Pos[0];

  visiblie_idx_set.clear();

  global_robot_theta=0.0;
  //Iteration for belief grid
  for(int i(0);i<camera_map.info.width;i++)
    for(int j(0);j<camera_map.info.height;j++)
  {
    int camera_map_idx=j*camera_map.info.height+i;

    // double map_ogirin_x = camera_map.info.origin.position.x+global_robot_x;
    // double map_ogirin_y = camera_map.info.origin.position.y+global_robot_y;

    double map_ogirin_x = camera_map.info.origin.position.x;
    double map_ogirin_y = camera_map.info.origin.position.y;


    double trans_vector_x=(i+0.5)*camera_map.info.resolution;
    double trans_vector_y=(j+0.5)*camera_map.info.resolution;

    double rot_trans_vector_x = cos(global_robot_theta)*trans_vector_x-sin(global_robot_theta)*trans_vector_y;
    double rot_trans_vector_y = sin(global_robot_theta)*trans_vector_x+cos(global_robot_theta)*trans_vector_y;

    double belief_global_x=map_ogirin_x+rot_trans_vector_x;
    double belief_global_y=map_ogirin_y+rot_trans_vector_y;

    //solve
    bool line1_result =getlinevalue(1,belief_global_x,belief_global_y);
    bool line2_result =getlinevalue(2,belief_global_x,belief_global_y);


    if( line1_result && line2_result )
    {
      camera_map.data[camera_map_idx]=30;  
      visiblie_idx_set.push_back(camera_map_idx);         //save cell_id 
    }
    else
      camera_map.data[camera_map_idx]=0.0; 
  }


}

bool Dynamic_Manager::check_cameraregion(float x_pos,float y_pos)
{

  
 //  double dynamic_winodow_xmax=Scaled_dynamic_map.info.origin.position.x+Scaled_dynamic_map.info.width*Scaled_dynamic_map.info.resolution;
 //  double dynamic_winodow_ymax=Scaled_dynamic_map.info.origin.position.y+Scaled_dynamic_map.info.height*Scaled_dynamic_map.info.resolution; 

 //  double map_boundary_x = Scaled_dynamic_map.info.width*Scaled_dynamic_map.info.resolution;
 //  double map_boundary_y = Scaled_dynamic_map.info.height*Scaled_dynamic_map.info.resolution;

 //  double x_pos_local = x_pos-Scaled_dynamic_map.info.origin.position.x;
 //  double y_pos_local = y_pos-Scaled_dynamic_map.info.origin.position.y;

 // std::cout<<"x local & y local "<<std::endl;
 // std::cout<<x_pos_local<<", "<<y_pos_local<<std::endl;
 // std::cout<<"map_boundary_x & map_boundary_y"<<std::endl;
 // std::cout<<map_boundary_x<<", "<<map_boundary_y<<std::endl;

 //  if(x_pos_local<map_boundary_x && y_pos_local<map_boundary_y)
 //  {
 //  //return true if it is occupied with obstacles
 //  if(camera_map.data.size()>0)
 //  {   
 //    int obs_idx=CoordinateTransform_Global2_cameraMap(x_pos,y_pos);
    
 //    if(obs_idx<camera_map.data.size()){
 //      if(camera_map.data[obs_idx]>10.0)
 //        return true;
 //      else
 //        return false;
 //    }
 //  }

 //  }

 //  return true;

  double max_boundary = 7.5 ; //(height or width)*resolution *0.5

 if(abs(x_pos)<max_boundary && abs(y_pos)<max_boundary)
  {
  //return true if it is occupied with obstacles
  if (camera_map.data.size()>0)
  {   
    int obs_idx=CoordinateTransform_Global2_cameraMap(x_pos,y_pos);
    
    if(obs_idx<camera_map.data.size()){
      if(camera_map.data[obs_idx]>10.0)
        return true;
      else
        return false;
    }
  }

  }

  return true;
}



int Dynamic_Manager::CoordinateTransform_Global2_cameraMap(float global_x, float global_y)
{
  double reference_origin_x=camera_map.info.origin.position.x;
  double reference_origin_y=camera_map.info.origin.position.y;

  //Find the coordinate w.r.t map origin
  double  temp_x  = global_x - reference_origin_x;
  double  temp_y  = global_y - reference_origin_y;

  //Find the map cell idx for x, y
  std::vector<int> human_coord(2,0);
  human_coord[0]= (int) (temp_x/camera_map.info.resolution);
  human_coord[1]= (int) (temp_y/camera_map.info.resolution);

  //Find the map index from cell x, y
  int static_map_idx= human_coord[0]+camera_map.info.width*human_coord[1];

  // std::cout<<"map_idx : "<<static_map_idx<<std::endl;
  return static_map_idx;
   
}



bool Dynamic_Manager::getlinevalue(int line_type,double input_x, double input_y)
{

  double global_robot_theta = global_pose[2]+Head_Pos[0];
  // double global_robot_theta =Camera_angle;
  double theta_1=-FOVW*MATH_PI/180+global_robot_theta;
  double theta_2= FOVW*MATH_PI/180+global_robot_theta;
  
  double m_1=tan(theta_1);
  double m_2=tan(theta_2);

  int isspecial=0;

  if(theta_1<-MATH_PI/2.0 && theta_2 >-MATH_PI/2.0)
  {
    double temp=m_2;
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


void Dynamic_Manager::publishZeropaths()
{

	  	nav_msgs::Path ZeroSplinePath;
  		ZeroSplinePath.header.frame_id = "map";
  		geometry_msgs::PoseStamped zeropose;
  	
	
	    zeropose.pose.position.x=CurVector[0];
	   	zeropose.pose.position.y=CurVector[1];
	   	zeropose.pose.orientation = tf::createQuaternionMsgFromYaw(m_desired_heading);
	   	ZeroSplinePath.poses.push_back(zeropose);
	   	// printf("spline path2 index : %d, x coord : %lf , y coord : %lf \n", i,zeropose.pose.position.x,zeropose.pose.position.y);
	
	SplinePath_pub2.publish(ZeroSplinePath);

}


void Dynamic_Manager::publishpaths()
{
	// if(booltrackHuman)
 	// {
 		SplinePath_pub.publish(path);
		//SplinePath_pub2.publish(Pre_dynamicSplinePath);
		Scaled_dynamic_map_path_pub.publish(Scaled_dynamic_map_path);
	// }
	// else
	// {	
	// 	//publishZeropaths();

	// }
}

//Generate path from solution
void Dynamic_Manager::generatePath()
{
	vector<int> cur_pos(2,0);
	for(int i(0);i<2;i++)
		cur_pos[i]=m_Start[i];

	MDPPath.clear();
	int cur_stid=Coord2CellNum(m_Start);
	int goal_stid=Coord2CellNum(m_Goal);

	
	vector<double> t_values;
	vector<double> x_values;
	vector<double> y_values;
	vector<double> global_coords;

	//double* = new double [13]

	// x_values.push_back(m_Start[0]);
	// y_values.push_back(m_Start[1]);

	Mapcoord2GlobalCoord(m_Start,global_coords);
	x_values.push_back(global_coords[0]);
	y_values.push_back(global_coords[1]);

	cout<<"cur st id : "<<cur_stid<<endl;
	MDPPath.push_back(cur_stid);
	
	while(1)
	{
		//get next position from policy solution
		for(int i(0);i<2;i++)
			cur_pos[i]+=ActionCC[PolicyNum[cur_stid]][i];

		Mapcoord2GlobalCoord(cur_pos,global_coords);
		ROS_INFO("cur pos Id : %d, x:  %d, y // global x : %.3lf , global y : %.3lf \n",cur_pos[0], cur_pos[1],global_coords[0],global_coords[1]);

		x_values.push_back(global_coords[0]);
		y_values.push_back(global_coords[1]);

		cur_stid=Coord2CellNum(cur_pos);
		MDPPath.push_back(cur_stid);

		//ROS_INFO("Id : %d, x:  %d, y : %d \n",cur_stid, cur_pos);

		if(cur_stid==goal_stid)
			break;
	}

	//Making Spline path=======================
	int data_size=x_values.size();
	//for(int k=0;k<data_size;k++)
		//printf("spline path index : %d, x_values : %lf , y values : %lf \n", k,x_values[k],y_values[k]);

	//Making t-vetors;
	t_values.resize(x_values.size());
	//t_values[0]=0;
	double time_length=1.0;
	double time_const=(time_length)/data_size;
	for(int k(0);k<data_size;k++)
		t_values[k]=k*time_const;
	
	m_CubicSpline_x = new srBSpline;
	m_CubicSpline_x->_Clear();

	m_CubicSpline_y = new srBSpline;
	m_CubicSpline_y->_Clear();

	vector<double> Spline_x;
	vector<double> Spline_y;
	m_CubicSpline_x->CubicSplineInterpolation(t_values,x_values,x_values.size());
	m_CubicSpline_y->CubicSplineInterpolation(t_values,y_values,y_values.size());

	int path_size=8;	
	double const_path=(time_length)/ path_size ;

	Spline_x.push_back(x_values[0]);
	Spline_y.push_back(y_values[0]);

	double ret_x=0.0;
	double ret_y=0.0;
	double t_idx=0.0;
	 for(int j(0);j<path_size;j++){
  	   t_idx=(j+1)*const_path;
       m_CubicSpline_x->getCurvePoint(ret_x,t_idx);
       m_CubicSpline_y->getCurvePoint(ret_y,t_idx);

       Spline_x.push_back(ret_x);
       Spline_y.push_back(ret_y);
     }

     //Publish SPline Path
  	nav_msgs::Path path ;
  	path.header.frame_id = "map";
  	geometry_msgs::PoseStamped pose;
	// pose.header.frame_id = "map_local";

	 for (int i = 0; i < Spline_x.size(); i++)
 	 {	    	    
	    pose.pose.position.x = Spline_x[i];
	    pose.pose.position.y = Spline_y[i];
	    printf("spline path index : %d, x coord : %lf , y coord : %lf \n", i,pose.pose.position.x,pose.pose.position.y);
	    pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.05);
	    path.poses.push_back(pose);
	}
	 
	 SplinePath_pub.publish(path);

	 // Publish static map_path

    //MDP solution publsih
    std_msgs::Int32MultiArray MDPsolution_msg;
	
	MDPsolution_msg.data.resize(PolicyNum.size()); 
	for(int i(0);i<PolicyNum.size();i++)
		MDPsolution_msg.data[i]=PolicyNum[i];
	MDPSol_pub.publish(MDPsolution_msg);

	//Publish path after planning
	// std_msgs::Int32MultiArray pathmap_msg;
	// pathmap_msg.data = MDPPath;
	// Path_Pub.publish(pathmap_msg);
	// ROS_INFO("publish");
}	



