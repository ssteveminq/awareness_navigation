#include "PathManager.h"



PathManager::PathManager()
{
 // pathSize=3;
	pre_idx=-1;
	dyn_pre_idx=-1;
	pathSize=0;
	cur_idx=0;
	dyn_cur_idx=0;
	viewpub_iters=0;
	base_trajectory.resize(3);
	global_pose.resize(3,0.0);
	hsrb_base_pose.resize(3,0.0);
	viewTarget.resize(2,0.0);
	lastSubTarget.resize(2,0.0);
	finalTarget.resize(2,0.0);
	dyn_finalTarget.resize(2,0.0);
	dyn_lastSubTarget.resize(2,0.0);


	discrete_mode=false;
	scan_mode=false;
	headiter=0;
	half_pathsize=7;
	boolreceive=false;


	setNavTarget_pub=m_node.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/move/goal",50,true);
	viewTarget_visual_pub = m_node.advertise<visualization_msgs::Marker>("viewTarget_marker",30, true);
	Gaze_point_pub= m_node.advertise<geometry_msgs::Point>("/gazed_point_fixing_node/target_point", 50, true);
	Gaze_activate_pub= m_node.advertise<std_msgs::Bool>("/gazed_point_fixing_node/activate", 50, true);
    nav_cmd_pub=m_node.advertise<std_msgs::Int8>("/nav_cmd_int", 50, true);
    head_cmd_pub=m_node.advertise<std_msgs::Int8>("head_cmd_int",50,true);
        
}

PathManager::~PathManager()
{

	
}


void PathManager::global_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  global_pose[0]=msg->pose.position.x;
  global_pose[1]=msg->pose.position.y;

   
   tf::StampedTransform baselinktransform;
   listener.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(3.0));
   listener.lookupTransform("map", "base_link", ros::Time(0), baselinktransform);
   double yaw_tf =   tf::getYaw(baselinktransform.getRotation()); 

   global_pose[2]=yaw_tf;

   // std::cout<<"global x: "<<global_pose[0]<<",global y :" <<global_pose[1]<<std::endl;

   
    
}


void PathManager::setPath(const nav_msgs::Path::ConstPtr& msg)
{
 
	listener.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(3.0));
	geometry_msgs::TransformStamped transform_stamped;

    // try {
    // transform_stamped =tf_buffer_.lookupTransform("map","base_link",ros::Time(0),ros::Duration(3.0));
    // } catch(const tf2::TransformException& ex) {
    //   // ROS_ERROR_STREAM((boost::format("Cannot transform goal pose from '%1%' frame to '%2%' frame.")
    //   //                   % kOriginLink2 % kRobotBaseLink2).str());
    //   return;
    // }
    // geometry_msgs::Pose origin_to_robot;
    // origin_to_robot.position.x = transform_stamped.transform.translation.x;
    // origin_to_robot.position.y = transform_stamped.transform.translation.y;
    // origin_to_robot.orientation = transform_stamped.transform.rotation;

    // std::cout<<"x: "<<origin_to_robot.position.x<<", y :" <<origin_to_robot.position.y<<std::endl;

  std::cout<<"set path"<<std::endl;
  pathSize=msg->poses.size();
  // pathSize=1;
  x_.resize(pathSize,0.0);
  y_.resize(pathSize,0.0);

  Recived_path.clear();
   //save path from rosnode subscribe    
  for(int k(0);k<pathSize;k++)
  {
    x_[k]=msg->poses[k].pose.position.x;
    y_[k]=msg->poses[k].pose.position.y;
    // t_[k]=msg->poses[k].pose.position.z;
    
	    if(k==pathSize-1)
	    {
	    	finalTarget[0] = msg->poses[k].pose.position.x;
	 		finalTarget[1] = msg->poses[k].pose.position.y;
	    }


    std::vector<double> tempVec(2,0.0);
    tempVec[0]=msg->poses[k].pose.position.x;
    tempVec[1]=msg->poses[k].pose.position.y;
    Recived_path.push_back(tempVec);
    printf("mdp path x-coord : %lf, y-coord : %lf \n", x_[k],y_[k]);
   }


   // sendPathAction();

}


void PathManager::sendPathAction()
{

	control_msgs::FollowJointTrajectoryGoal goal;
	goal.trajectory.joint_names.push_back("odom_x");
	goal.trajectory.joint_names.push_back("odom_y");
	goal.trajectory.joint_names.push_back("odom_t");
	// goal.trajectory.header.frame_id = "/odom";

	goal.trajectory.points.resize(pathSize);
	for(int z=0;z<pathSize;z++)
	  {
	    goal.trajectory.points[z].positions.resize(3);
	    goal.trajectory.points[z].positions[0] =0.25;
	    goal.trajectory.points[z].positions[1] =0.25;
	    goal.trajectory.points[z].positions[2] = 0.0;
	    goal.trajectory.points[z].velocities.resize(3);
	    for (size_t i = 0; i < 3; ++i) {
	      goal.trajectory.points[z].velocities[i] = 0.0;
	    }
	    double time_duration=2;
	    goal.trajectory.points[z].time_from_start = ros::Duration(time_duration);

	  }
	  // // send message to the action server
	  
	  // trj_cli.sendGoal(goal);

	  // printf("send action");
	  // // wait for the action server to complete the order
	  // trj_cli.waitForResult(ros::Duration(2.0));

}

double PathManager::getdistance(vector<double> cur, vector<double> goal, int dimension)
{
	double temp_dist=0.0;

	for(int i(0);i<dimension;i++)
		temp_dist+=pow((cur[i]-goal[i]),2);
	
	temp_dist = sqrt(temp_dist);
	return temp_dist;
}


int PathManager::findpathidx()
{
	//among path poses, find the path index of nearest point
	std::vector<double> Distanceset;
    // Distanceset.resize(Cur_detected_human.size(),0.0);
    Distanceset.resize(Recived_path.size(),0.0);
    
    double minDistance=200.0;
    int    minDistance_Idx=0;

      for(int i(0);i<Recived_path.size();i++)
      {
        // Distanceset[i]=getDistance(Cur_detected_human[i][0],Cur_detected_human[i][1]);
        Distanceset[i]=getdistance(global_pose, Recived_path[i],2);
        
        if(minDistance>Distanceset[i])
          {
            minDistance=Distanceset[i];
            minDistance_Idx=i;
          }
      }
  
    return minDistance_Idx;
}

int PathManager::finddynpathidx()
{
	//among path poses, find the path index of nearest point
	std::vector<double> Distanceset;
    // Distanceset.resize(Cur_detected_human.size(),0.0);
    Distanceset.resize(dyn_Recived_path.size(),0.0);
    
    double minDistance=200.0;
    int    minDistance_Idx=0;

      for(int i(0);i<dyn_Recived_path.size();i++)
      {
        // Distanceset[i]=getDistance(Cur_detected_human[i][0],Cur_detected_human[i][1]);
        Distanceset[i]=getdistance(global_pose, dyn_Recived_path[i],2);
        
        if(minDistance>Distanceset[i])
          {
            minDistance=Distanceset[i];
            minDistance_Idx=i;
          }
      }
  
    return minDistance_Idx;
}

void PathManager::publishViewTpointTarget()
{
	 	visualization_msgs::Marker marker_humans;
        // marker_humans.header.frame_id = "base_range_sensor_link";
        marker_humans.header.frame_id = "map";
        marker_humans.header.stamp = ros::Time::now();
        marker_humans.ns = "/view_target";
        marker_humans.id = 0;

        uint32_t shape = visualization_msgs::Marker::SPHERE;
        marker_humans.type = shape;

        marker_humans.pose.position.x = -4.0;
        marker_humans.pose.position.y = 1.0;
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

void PathManager::setViewpointTarget(const std::vector<double> pos)
{
	// printf("hello");
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
		 
		 //    geometry_msgs::TransformStamped transform_stamped;
		        tf::StampedTransform baselinktransform;
    			listener.waitForTransform("map", "base_range_sensor_link", ros::Time(0), ros::Duration(2.0));
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

            //publishViewTpointTarget();
    		GazePoint_msg.z=1.0;
    		Gaze_point_pub.publish(GazePoint_msg);
    		std::cout<<"gaze topic published"<<std::endl;

	    	std_msgs::Bool activateGaze_msg;
	    	activateGaze_msg.data=true;
	    	Gaze_activate_pub.publish(activateGaze_msg);
	    }
        viewpub_iters=0;
 
 
    return;

}



void PathManager::setNavTarget()
{
	// cur_idx=0;
	int find_cur_idx=0;

	if(pre_idx!=-1)
		{
			find_cur_idx = findpathidx();
			printf("cur found idx is %d",find_cur_idx);

	
		}
	else
		find_cur_idx=0;

	// if(cur_idx==pathSize-1)
	if(cur_idx==half_pathsize){
		
	 
	  move_base_msgs::MoveBaseActionGoal Navmsgs;
      Navmsgs.header.stamp =  ros::Time::now();
      ROS_INFO("time");
      std::cout<<Navmsgs.header.stamp <<std::endl;
      Navmsgs.goal.target_pose.header.frame_id = "map";
		Navmsgs.goal.target_pose.pose.position.x=8.2;
     	Navmsgs.goal.target_pose.pose.position.y=0.5;
     	Navmsgs.goal.target_pose.pose.position.z=0.5;

      std::vector<double> GoalVector;
      GoalVector.resize(2,0.0);
      GoalVector[0]=8.2;
      GoalVector[1]=0.5;

      lastSubTarget[0] =8.2;
      lastSubTarget[1] =0.5; 

      GoalVector[0]=GoalVector[0]-global_pose[0];
      GoalVector[1]=GoalVector[1]-global_pose[1];

      double temp_yaw =0.5;
      double temp_roll=0.0;
      double temp_pitch=0.0;
            // poses.orientation = tf::transformations.quaternion_from_euler(0.0, 0.0, temp_yaw);
      
      geometry_msgs::Quaternion q;

      double t0 = cos(temp_yaw * 0.5);
      double t1 = sin(temp_yaw * 0.5);
      double t2 = cos(temp_roll * 0.5);
      double t3 = sin(temp_roll * 0.5);
      double t4 = cos(temp_pitch * 0.5);
      double t5 = sin(temp_pitch * 0.5);
      q.w = t0 * t2 * t4 + t1 * t3 * t5;
      q.x = t0 * t3 * t4 - t1 * t2 * t5;
      q.y = t0 * t2 * t5 + t1 * t3 * t4;
      q.z = t1 * t2 * t4 - t0 * t3 * t5;

      // Navmsgs.goal.target_pose.pose.orientation = head_orientation;

       Navmsgs.goal.target_pose.pose.orientation.x=q.x;
       Navmsgs.goal.target_pose.pose.orientation.y=q.y;
       Navmsgs.goal.target_pose.pose.orientation.z=q.z;
       Navmsgs.goal.target_pose.pose.orientation.w=q.w;
       setNavTarget_pub.publish(Navmsgs);

       double temp_dist = getdistance(global_pose,lastSubTarget,2);
		printf("temp_dist is %.3lf \n",temp_dist);
		if(temp_dist<3*POS_ERR)
			  {
			  	scan_mode=true;

			  }

	 
		return;
	}
	else{

      printf("cur idx is %d",cur_idx);
	  move_base_msgs::MoveBaseActionGoal Navmsgs;
      Navmsgs.header.stamp =  ros::Time::now();
     
      ROS_INFO("time");
      std::cout<<Navmsgs.header.stamp <<std::endl;
      Navmsgs.goal.target_pose.header.frame_id = "map";

     Navmsgs.goal.target_pose.pose.position.x=x_[cur_idx];
     Navmsgs.goal.target_pose.pose.position.y=y_[cur_idx];
     Navmsgs.goal.target_pose.pose.position.z=0.5;

      std::vector<double> GoalVector;
      GoalVector.resize(2,0.0);
      GoalVector[0]=x_[cur_idx];
      GoalVector[1]=y_[cur_idx];

      lastSubTarget[0] = x_[cur_idx];
      lastSubTarget[1] =y_[cur_idx]; 


      GoalVector[0]=GoalVector[0]-global_pose[0];
      GoalVector[1]=GoalVector[1]-global_pose[1];

      double temp_yaw =atan(GoalVector[1]/GoalVector[0]);
      temp_yaw=temp_yaw-global_pose[2];
      double temp_roll=0.0;
      double temp_pitch=0.0;
            // poses.orientation = tf::transformations.quaternion_from_euler(0.0, 0.0, temp_yaw);
      
      geometry_msgs::Quaternion q;

      double t0 = cos(temp_yaw * 0.5);
      double t1 = sin(temp_yaw * 0.5);
      double t2 = cos(temp_roll * 0.5);
      double t3 = sin(temp_roll * 0.5);
      double t4 = cos(temp_pitch * 0.5);
      double t5 = sin(temp_pitch * 0.5);
      q.w = t0 * t2 * t4 + t1 * t3 * t5;
      q.x = t0 * t3 * t4 - t1 * t2 * t5;
      q.y = t0 * t2 * t5 + t1 * t3 * t4;
      q.z = t1 * t2 * t4 - t0 * t3 * t5;

      // Navmsgs.goal.target_pose.pose.orientation = head_orientation;

       Navmsgs.goal.target_pose.pose.orientation.x=q.x;
       Navmsgs.goal.target_pose.pose.orientation.y=q.y;
       Navmsgs.goal.target_pose.pose.orientation.z=q.z;
       Navmsgs.goal.target_pose.pose.orientation.w=q.w;

       setNavTarget_pub.publish(Navmsgs);

      	//check targetdistance
		double temp_dist = getdistance(global_pose,lastSubTarget,2);
		printf("temp_dist is %.3lf \n",temp_dist);
		if(temp_dist<POS_ERR)
			cur_idx++;

       // cur_idx++;


       }
}



void PathManager::setDynNavTarget()
{
	// cur_idx=0;
	int find_cur_idx=0;

	if(dyn_pre_idx!=-1)
		{
			find_cur_idx = finddynpathidx();
			printf("cur found idx is %d",find_cur_idx);

	
		}
	else
		find_cur_idx=0;

	// if(cur_idx==pathSize-1)
	

      printf("cur idx is %d",dyn_cur_idx);
	  move_base_msgs::MoveBaseActionGoal Navmsgs;
      Navmsgs.header.stamp =  ros::Time::now();
     
      ROS_INFO("time");
      std::cout<<Navmsgs.header.stamp <<std::endl;
      Navmsgs.goal.target_pose.header.frame_id = "map";

     Navmsgs.goal.target_pose.pose.position.x=dyn_x_[dyn_cur_idx];
     Navmsgs.goal.target_pose.pose.position.y=dyn_y_[dyn_cur_idx];
     Navmsgs.goal.target_pose.pose.position.z=0.5;

      std::vector<double> GoalVector;
      GoalVector.resize(2,0.0);
      GoalVector[0]=dyn_x_[dyn_cur_idx];
      GoalVector[1]=dyn_y_[dyn_cur_idx];

      dyn_lastSubTarget[0] = dyn_x_[dyn_cur_idx];
      dyn_lastSubTarget[1] =dyn_y_[dyn_cur_idx]; 


      GoalVector[0]=GoalVector[0]-global_pose[0];
      GoalVector[1]=GoalVector[1]-global_pose[1];

      double temp_yaw =atan(GoalVector[1]/GoalVector[0]);
      temp_yaw=temp_yaw-global_pose[2];
      double temp_roll=0.0;
      double temp_pitch=0.0;
            // poses.orientation = tf::transformations.quaternion_from_euler(0.0, 0.0, temp_yaw);
      
      geometry_msgs::Quaternion q;

      double t0 = cos(temp_yaw * 0.5);
      double t1 = sin(temp_yaw * 0.5);
      double t2 = cos(temp_roll * 0.5);
      double t3 = sin(temp_roll * 0.5);
      double t4 = cos(temp_pitch * 0.5);
      double t5 = sin(temp_pitch * 0.5);
      q.w = t0 * t2 * t4 + t1 * t3 * t5;
      q.x = t0 * t3 * t4 - t1 * t2 * t5;
      q.y = t0 * t2 * t5 + t1 * t3 * t4;
      q.z = t1 * t2 * t4 - t0 * t3 * t5;

      // Navmsgs.goal.target_pose.pose.orientation = head_orientation;

       Navmsgs.goal.target_pose.pose.orientation.x=q.x;
       Navmsgs.goal.target_pose.pose.orientation.y=q.y;
       Navmsgs.goal.target_pose.pose.orientation.z=q.z;
       Navmsgs.goal.target_pose.pose.orientation.w=q.w;

       setNavTarget_pub.publish(Navmsgs);


            	//check targetdistance
		double temp_dist = getdistance(global_pose,dyn_lastSubTarget,2);
		printf("temp_dist is %.3lf \n",temp_dist);
		if(temp_dist<POS_ERR)
			dyn_cur_idx++;

       // cur_idx++;


       
}

int PathManager::setFinalNavTarget()
{


	  move_base_msgs::MoveBaseActionGoal Navmsgs;
      Navmsgs.header.stamp =  ros::Time::now();
     
      ROS_INFO("time");
      std::cout<<Navmsgs.header.stamp <<std::endl;
      Navmsgs.goal.target_pose.header.frame_id = "map";

     Navmsgs.goal.target_pose.pose.position.x=finalTarget[0];
     Navmsgs.goal.target_pose.pose.position.y=finalTarget[1];
     Navmsgs.goal.target_pose.pose.position.z=0.5;

      std::vector<double> GoalVector;
      GoalVector.resize(2,0.0);
      GoalVector[0]=finalTarget[0];;
      GoalVector[1]=finalTarget[1];

      GoalVector[0]=GoalVector[0]-global_pose[0];
      GoalVector[1]=GoalVector[1]-global_pose[1];

      double temp_yaw =atan(GoalVector[1]/GoalVector[0]);
      temp_yaw=temp_yaw-global_pose[2];
      double temp_roll=0.0;
      double temp_pitch=0.0;
            // poses.orientation = tf::transformations.quaternion_from_euler(0.0, 0.0, temp_yaw);
      
      geometry_msgs::Quaternion q;

      double t0 = cos(temp_yaw * 0.5);
      double t1 = sin(temp_yaw * 0.5);
      double t2 = cos(temp_roll * 0.5);
      double t3 = sin(temp_roll * 0.5);
      double t4 = cos(temp_pitch * 0.5);
      double t5 = sin(temp_pitch * 0.5);
      q.w = t0 * t2 * t4 + t1 * t3 * t5;
      q.x = t0 * t3 * t4 - t1 * t2 * t5;
      q.y = t0 * t2 * t5 + t1 * t3 * t4;
      q.z = t1 * t2 * t4 - t0 * t3 * t5;

      // Navmsgs.goal.target_pose.pose.orientation = head_orientation;
       Navmsgs.goal.target_pose.pose.orientation.x=q.x;
       Navmsgs.goal.target_pose.pose.orientation.y=q.y;
       Navmsgs.goal.target_pose.pose.orientation.z=q.z;
       Navmsgs.goal.target_pose.pose.orientation.w=q.w;

       printf("final_target published");
       setNavTarget_pub.publish(Navmsgs);



}


int PathManager::getdirection(std::vector<double> pos)
{

	if(pos.size()<2)
		return 0;
	else{

			int RobotHeading=0;
			vector < vector<double> > ActionCC;
			ActionCC.resize(8);
			for(int i(0);i<8;i++)
				ActionCC[i].resize(2);

			ActionCC[0][0]=1;   ActionCC[0][1]=0;
			ActionCC[1][0]=1/sqrt(2);   ActionCC[1][1]=1/sqrt(2);
			ActionCC[2][0]=0;   ActionCC[2][1]=1;
			ActionCC[3][0]=-1/sqrt(2);  ActionCC[3][1]=1/sqrt(2);
			ActionCC[4][0]=-1;  ActionCC[4][1]=0;
			ActionCC[5][0]=-1/sqrt(2);  ActionCC[5][1]=-1/sqrt(2);
			ActionCC[6][0]=0;   ActionCC[6][1]=-1;
			ActionCC[7][0]=1/sqrt(2);   ActionCC[7][1]=-1/sqrt(2);

			vector<double> innervector(8,0.0);

			for(int i(0);i<8;i++)
			 innervector[i]=ActionCC[i][0]*pos[0]+ActionCC[i][1]*pos[1];

			 RobotHeading=getIndexOfLargestElement(innervector);
     		RobotHeading++;

			return RobotHeading;
	}
}

int PathManager::getIndexOfLargestElement(vector<double> arr)
{
    int largestIndex = 0;
    for (int index = largestIndex; index < arr.size(); index++) {
        if (arr[largestIndex] < arr[index]) {
            largestIndex = index;
        }
    }
    return largestIndex;
}

void PathManager::discreteCtrltoPos(std::vector<double> pos)
{

	if(pos.size()<2)
		return;
	else{

			double distance_Target=getdistance(pos,global_pose,2);
			if(distance_Target>0.5)
		    {
			    geometry_msgs::Vector3Stamped gV, tV;
			    gV.vector.x = pos[0]-global_pose[0];
			    gV.vector.y = pos[1]-global_pose[1];
			    gV.vector.z = 1.0;

			    tf::StampedTransform baselinktransform;
				listener.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(3.0));
			    
			    gV.header.stamp = ros::Time();
			    gV.header.frame_id = "/map";
			    listener.transformVector("/base_link", gV, tV);

			    std::vector<double> tempVec(3,0.0);
			    tempVec[0]=tV.vector.x;
				tempVec[1]=tV.vector.y;
				tempVec[2]=tV.vector.z;	

				printf("Converted coords : x is %.3lf, y is %.3lf \n",tempVec[0], tempVec[1]);
				
				int desired_dir=getdirection(tempVec);
				printf("Desired direction is %d \n",desired_dir);
				
				std_msgs::Int8 int_msgs;
				int_msgs.data=desired_dir;
				nav_cmd_pub.publish(int_msgs);

			}	
			else
				return;

		}


}

control_msgs::FollowJointTrajectoryGoal PathManager::getsubgoal()
{

	control_msgs::FollowJointTrajectoryGoal goal;
	goal.trajectory.joint_names.push_back("odom_x");
	goal.trajectory.joint_names.push_back("odom_y");
	goal.trajectory.joint_names.push_back("odom_t");
	goal.trajectory.header.frame_id = "/map";

	int fake_pathsize= pathSize;
	goal.trajectory.points.resize(fake_pathsize);
	for(int z=0;z<fake_pathsize;z++)
	  {
	    goal.trajectory.points[z].positions.resize(3);
	    goal.trajectory.points[z].positions[0] =x_[z];
	    goal.trajectory.points[z].positions[1] =y_[z];
	    goal.trajectory.points[z].positions[2] = global_pose[2];
	    goal.trajectory.points[z].velocities.resize(3);
	    // goal.trajectory.points[z].accelerations.resize(3);
	    // goal.trajectory.points[z].effort.resize(3);
	    
	    for (size_t i = 0; i < 3; ++i) {
	      goal.trajectory.points[z].velocities[i] = 0.0;
	      // goal.trajectory.points[z].accelerations[i]=0.0;
	      // goal.trajectory.points[z].effort[i]=0.0;
	    }

	    double time_duration=z+3;
	    goal.trajectory.points[z].time_from_start = ros::Duration(time_duration);

	  }



	  return goal;


}

control_msgs::FollowJointTrajectoryGoal PathManager::getsubgoal(std::vector<double> desired_target)
{

	control_msgs::FollowJointTrajectoryGoal goal;
	goal.trajectory.joint_names.push_back("odom_x");
	goal.trajectory.joint_names.push_back("odom_y");
	goal.trajectory.joint_names.push_back("odom_t");
	goal.trajectory.header.frame_id = "/base_link";

	goal.trajectory.points.resize(1);
	
    goal.trajectory.points[0].positions.resize(3);
    goal.trajectory.points[0].positions[0] = desired_target[0];
    goal.trajectory.points[0].positions[1] = desired_target[1];
    goal.trajectory.points[0].positions[2] = global_pose[2];
    goal.trajectory.points[0].velocities.resize(3);
    
    for (size_t i = 0; i < 3; ++i) {
        goal.trajectory.points[0].velocities[i] = 0.0;
	      // goal.trajectory.points[z].accelerations[i]=0.0;
	      // goal.trajectory.points[z].effort[i]=0.0;
	    }

	    double time_duration=3;
	    goal.trajectory.points[0].time_from_start = ros::Duration(time_duration);




	  return goal;


}


void PathManager::dyn_path_callback(const nav_msgs::Path::ConstPtr& msg)
{
	// printf("dynamic callback");
	if(boolreceive)
		return;
  else{
	  int	dyn_pathSize=msg->poses.size();
	  //pathSize=3;
	  dyn_x_.resize(dyn_pathSize,0.0);
	  dyn_y_.resize(dyn_pathSize,0.0);

	  //save path from rosnode subscribe    
	  dyn_Recived_path.clear();
	   //save path from rosnode subscribe    
	  for(int k(0);k<dyn_pathSize;k++)
	  {
	    dyn_x_[k]=msg->poses[k].pose.position.x;
	    dyn_y_[k]=msg->poses[k].pose.position.y;
	    // t_[k]=msg->poses[k].pose.position.z;
	    
	    if(k==dyn_pathSize-1)
	    {
	    	dyn_finalTarget[0] = msg->poses[k].pose.position.x;
	 		dyn_finalTarget[1] = msg->poses[k].pose.position.y;
	    }

	    std::vector<double> tempVec(2,0.0);
	    tempVec[0]=msg->poses[k].pose.position.x;
	    tempVec[1]=msg->poses[k].pose.position.y;
	    dyn_Recived_path.push_back(tempVec);
	    printf("dyn_mdp path x-coord : %lf, y-coord : %lf \n", x_[k],y_[k]);

	   }

		dyn_cur_idx=0;
		boolreceive=true;
		}
}
