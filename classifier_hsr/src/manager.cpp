#include "manager.h"

MapParam::MapParam()
{
    map_step=Grid_STEP;
   	m_cell_x_width=0.0;
	m_cell_y_width=0.0;
	Num_grid_X=Grid_Num_X;
	Num_grid_Y=Grid_Num_Y;
	robot_map_id=0;

	NearestHuman_V.resize(2);
	RobotHeading_V.resize(2);

	for(int i(0);i<State_Type.size();i++)
{
	NearestHuman_V[i]=0.0;
	RobotHeading_V[i]=0.0;

}
	//NearestHuman_V =std::vector<int>(2,0.9)

	State_Type.resize(8);
	for(int i(0);i<State_Type.size();i++)
		State_Type[i]=0;
	
	State_Distance.resize(8);
	for(int i(0);i<State_Distance.size();i++)
		State_Distance[i]=0.0;
}
MapParam::~MapParam()
{

}
void MapParam::setWidth (int Width_)
{
	Num_grid_X=Width_;
}

void MapParam::setHeight(int height_)
{
	Num_grid_Y=height_;
}

void MapParam::set_Cell_Info(vector<int> _inputCellInfo)
{
	Cell_Info.resize(_inputCellInfo.size());
	
	for(int i(0);i<_inputCellInfo.size();i++)
		Cell_Info[i]=_inputCellInfo[i];

}

void MapParam::set_State_Type(vector<int> _State_Type)
{
	for(int i(0);i<_State_Type.size();i++)
		State_Type[i]=_State_Type[i];

}

void MapParam::set_State_Distance(vector<float> _State_Distance)
{
	for(int i(0);i<_State_Distance.size();i++)
	State_Distance[i]=_State_Distance[i];

}
void MapParam::set_NearestHuman_V(vector<float> _NearestHuman_V)
{
		for(int i(0);i<_NearestHuman_V.size();i++)
	NearestHuman_V[i]=_NearestHuman_V[i];

}


void MapParam::set_RobotHeading_V(vector<float> _RobotHeading_V)
{
		for(int i(0);i<_RobotHeading_V.size();i++)
	RobotHeading_V[i]=_RobotHeading_V[i];
}

void MapParam::set_RobotId(int _robotid)
{

	robot_map_id=_robotid;
}


void MapParam::set_OCC_Info(vector<int> _inputOCCInfo)
{

	OCC_Info.resize(_inputOCCInfo.size());
	
	for(int i(0);i<_inputOCCInfo.size();i++)
		OCC_Info[i]=_inputOCCInfo[i];
}

void MapParam::set_Robot_Info(vector<int> _inputRobotInfo)
{
	Robot_localpos.resize(_inputRobotInfo.size());
	for(int i(0);i<_inputRobotInfo.size();i++)
		Robot_localpos[i]=_inputRobotInfo[i];
}

CBAManager::CBAManager()
{
	pClassifier=NULL;
	pMapParam=NULL;
	Init();
}

void CBAManager::Init()
{
	X_mapSize = Grid_STEP;
	Num_Grids = Grid_Num_X*Grid_Num_Y;

	m_Start =vector<int>(2,0);
	m_Goal  =vector<int>(2,0);
	m_Robot  =vector<int>(2,0);
	m_unitGoal=vector<float>(2,0.0);

    Desiredaction =0;


   	m_Start[0]=Start_X;
	m_Start[1]=Start_Y;

	m_Goal[0]=Goal_X;
	m_Goal[1]=Goal_Y;

	m_Robot[0]=Start_X;
	m_Robot[1]=Start_Y;

	Feature_dim=Dim_feature;

	vector<float> Tempvector(Dim_feature,0.0); 
	
	//Initialize TrainingData Set / Zero/1
	TrainingDataState.resize(Num_action);
	for(int i(0);i<Num_action;i++)
		{
			TrainingDataSet.push_back(Tempvector);
			TrainingDataState[i]=i;
		}

	storedFeaturevector.resize(Feature_dim,0.0);
	robot_global_x_pos=0.0;
	robot_global_y_pos=0.0;
	robot_theta_yaw=0.0;
	Storedbaddecision=0;

 //    Local_X_start=10;
	// Local_Y_start=90;
	// Local_X_end=159;
	// Local_Y_end=149;
	// Scale_constant=3;

  //getMDPsolutionFile();

	//Matlab_Pub  = n .advertise<matlab_msg::m_gridInfo>("/CBA2Matlab", 30);
	//Trikey_Pubn = n2.advertise<matlab_msg::m_gridInfo>("/CBA2Matlab", 30);
	
	boolMatlabsendData=false;
	boolAuto=false;
	Isbad=false;
}

	


	
CBAManager::~CBAManager()
{
	if(pClassifier!=NULL)
	{
		delete pClassifier;
		pClassifier=NULL;
	}


	if(pMapParam!=NULL)
	{
		delete pMapParam;
		pMapParam=NULL;
	}
	

}


void CBAManager::IntializeROS_publisher()
{
	ros::NodeHandle  m_node; 
	HSR_Pub =m_node.advertise<std_msgs::Int8>("/CBA_action_cmd", 30);

}


vector<int> CBAManager::Global2LocalCoord(vector<int> Global_coord)
{
	vector<int> Local_coords(2,0);
	
	Local_coords[0]=Global_coord[0]-Local_X_start;
	Local_coords[1]=Global_coord[1]-Local_Y_start;

	return Local_coords;
}

int CBAManager::Local2ScaledCellIdx(vector<int> Local_coord,int scale)
{
	 vector<int> Scaled_coords(2,0);
	 int local_X_max=Local_X_end-Local_X_start+1;
	 int local_Y_max=Local_Y_end-Local_Y_start+1;

	int  scaled_X_max= floor(local_X_max/scale);
	int scaled_Y_max= floor(local_Y_max/scale);
	
	cout<<"Scaled X_max :" <<scaled_X_max<<endl;
	cout<<"Scaled Y_max :" <<scaled_Y_max<<endl;

	 int Scaled_coords_X= floor(Local_coord[0]/scale);
	 int Scaled_coords_Y= floor(Local_coord[1]/scale);

 	cout<<"Scaled X_coord :" <<Scaled_coords_X<<endl;
	cout<<"Scaled Y_coord:" <<Scaled_coords_Y<<endl;


	 int Cell_idx=Scaled_coords_Y*scaled_X_max+Scaled_coords_X;

    
	return Cell_idx;
}


void CBAManager::setCalssfier(int dim, int Num_actions_)
{
	if(pClassifier==NULL)
		pClassifier=new ELMClassifier(dim,Num_actions_);
}


void CBAManager::setGoalConfig( const vector<int> _Goal )
{
		m_Goal[0]=_Goal[0];
		m_Goal[1]=_Goal[1];

}

void CBAManager::setStartConfig( const vector<int> _Start)
{
		m_Start[0]=_Start[0];
		m_Start[1]=_Start[1];
}

std::vector<int> CBAManager::getGoalConfig()
{
	return m_Goal;
}
int CBAManager::getCurRobotIdx()
{
	int Cell_idx=0;
	vector<int> globalcoord_robot(2,0);
	vector<int> avgVector(2,0);

	if(pMapParam->Robot_localpos.size()>0)
	{
		for(int i(0);i<pMapParam->Robot_localpos.size();i++)
		{
			globalcoord_robot=CellNum2Coord(pMapParam->Robot_localpos[i]);
			avgVector[0]+=globalcoord_robot[0];
			avgVector[1]+=globalcoord_robot[1];
		}
		avgVector[0]=avgVector[0]/pMapParam->Robot_localpos.size();
		avgVector[1]=avgVector[1]/pMapParam->Robot_localpos.size();
	}

	return Coord2CellNum(avgVector);

}
vector<int> CBAManager::getCurRobotCoord()
{
	vector<int> globalcoord_robot(2,0);
	vector<int> avgVector(2,0);

	if(pMapParam->Robot_localpos.size()>0)
	{
		for(int i(0);i<pMapParam->Robot_localpos.size();i++)
		{
			globalcoord_robot=CellNum2Coord(pMapParam->Robot_localpos[i]);
			avgVector[0]+=globalcoord_robot[0];
			avgVector[1]+=globalcoord_robot[1];
		}
		avgVector[0]=avgVector[0]/(pMapParam->Robot_localpos.size());
		avgVector[1]=avgVector[1]/(pMapParam->Robot_localpos.size());
	}

	return avgVector;

}


int CBAManager::getnearestGoalDirection()
{
//Goal has to be assigned
int GoalDirection=0;

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
float yaw_angle_deg=0.0;
float temp=0.0;
float temp_deg=0.0;
float temp_Robot2Goal=0.0;
temp=sqrt(m_Goal[0]*m_Goal[0]+m_Goal[1]*m_Goal[1]);
vector<float> unitgoal(2,0.0);

//find the unit vector w.r.t robot heading direction
vector<float> robotheadingdirection(2,0.0);

robotheadingdirection[0]=1.0;
robotheadingdirection[1]=tan(robot_theta_yaw);
float nomr_v=sqrt(pow(robotheadingdirection[0],2)+pow(robotheadingdirection[1],2));

if(nomr_v>0){

	robotheadingdirection[0]=robotheadingdirection[0]/nomr_v;
	robotheadingdirection[1]=robotheadingdirection[1]/nomr_v;
}
//ROS_INFO("normv : %.3lf, x : %.3lf , y : %.3lf \n",nomr_v,robotheadingdirection[0],robotheadingdirection[1]);

//Find the angle from robot to Goal poistion
 if(abs(m_unitGoal[0])!=0)
	temp_Robot2Goal=atan(m_unitGoal[1]/m_unitGoal[0]);




//Innerproduct
temp = m_unitGoal[0]*robotheadingdirection[0]+m_unitGoal[1]*robotheadingdirection[1];
float temp_norm = sqrt(pow(m_unitGoal[0],2)+pow(m_unitGoal[1],2))*sqrt(pow(robotheadingdirection[0],2)*pow(robotheadingdirection[1],2));

temp =acos(temp);

yaw_angle_deg=robot_theta_yaw*180/(3.141592);
float temp_headingangle_deg=temp_Robot2Goal*180/(3.141592);


temp_deg=temp_headingangle_deg-yaw_angle_deg;
// temp_deg=temp_deg*180/(3.141592);
//ROS_INFO("Yaw :%.3lf, heading angle : %.3lf,  Between angle : %.3lf",robot_theta_yaw,temp_headingangle,temp);
//ROS_INFO("Yaw :%.3lf, heading angle : %.3lf,  Between angle : %.3lf",yaw_angle_deg,temp_headingangle_deg,temp_deg);
unitgoal[0]=1.0;
unitgoal[1]=tan(temp);
nomr_v = sqrt(pow(unitgoal[0],2)+pow(unitgoal[1],2));
unitgoal[0]=unitgoal[0]/nomr_v;
unitgoal[1]=unitgoal[1]/nomr_v;

int NearestGoal_dir=0;

//These degrees are detrmined with atan(1/3), atan(3) : square grid 
//
//     |---|---|---|
//	   | 4 | 3 | 2 |
//	   |---|---|---|
//	   | 5 | R | 1 |
//	   |---|---|---|
//	   | 6 | 7 | 8 |
//	   |---|---|---|
//
///
/// May be I should include Robot cell itself? 

	if(temp_deg<18.1)
	{
		NearestGoal_dir=1;
	}
	else if(temp_deg < 71)
	{
		NearestGoal_dir=2;
	}
	else if(temp_deg < 107.2)
	{
		NearestGoal_dir=3;
	}
	else if(temp_deg <161)
	{
		NearestGoal_dir=4;
	}
	else if(temp_deg <198)
	{
		NearestGoal_dir=5;
	}
	else if (temp_deg <251)
	{
		NearestGoal_dir=6;
	}
	else if (temp_deg <289)
	{
		NearestGoal_dir=7;
	}
	else if (temp_deg <342)
	{
		NearestGoal_dir=8;
	}
	else{

		NearestGoal_dir=1;
	 }

ROS_INFO("Yaw :%.3lf, heading angle : %.3lf,  Between: %.3lf , NGoal_dir : %d ",yaw_angle_deg,temp_headingangle_deg,temp_deg,NearestGoal_dir);

// unitgoal[0]=m_unitGoal[0]-robotheadingdirection[0];
// unitgoal[1]=m_unitGoal[1]-robotheadingdirection[1];
// nomr_v = sqrt(pow(unitgoal[0],2)+pow(unitgoal[1],2));
// unitgoal[0]=unitgoal[0]/nomr_v;
// unitgoal[1]=unitgoal[1]/nomr_v;

//cout<<"Goal unit direction : "<<unitgoal[0]<<" , "<<unitgoal[1]<<endl;
// for(int i(0);i<8;i++)
// 	 innervector[i]=ActionCC[i][0]*unitgoal[0]+ActionCC[i][1]*unitgoal[1];

// int maxvalue_ix =getIndexOfLargestElement(innervector);

// maxvalue_ix++;

//cout<<"Goal direction : "<<maxvalue_ix<<endl;

return NearestGoal_dir;

}


void CBAManager::updateMaptoVec()
{
	vector<float> tmpvec(Feature_dim,0.0);
	TrainingDataSet.clear();

    TrainingDataState.clear();

    // TrainingDataSet.resize((int)(pClassifier->DataListMap.size()));


    //make a dateset if <int, List>... why?

    // What is your ultimate data type for data and answer?


    //

	map<int,list<RowVectorXd> >::iterator mapiter=pClassifier->DataListMap.begin();
	for(mapiter;mapiter!=pClassifier->DataListMap.end();mapiter++)
		{

			int datasizerow= mapiter->second.size();
			list<RowVectorXd>::iterator tempiter =mapiter->second.begin();	

			for(tempiter; tempiter!=mapiter->second.end();tempiter++)
			{
				for(int k(0);k<Feature_dim;k++)
					tmpvec[k]=(*tempiter)[k];

				cout<<"Tempvec"<<endl;
				for(int m=0;m<Feature_dim;m++)
					cout<<tmpvec[m]<<",";
					cout<<endl;

				TrainingDataSet.push_back(tmpvec);
                TrainingDataState.push_back(mapiter->first);
			}
		}


		cout<<"TrainingState"<<endl;
		for(int i(0);i<TrainingDataState.size();i++)
			cout<<TrainingDataState[i]<<", ";
		cout<<endl;



		cout<<"Trainingdata size"<<TrainingDataSet.size()<<endl;
		for(int k(0);k<TrainingDataSet.size();k++)
		{	for(int j(0);j<TrainingDataSet[0].size();j++)
			{
				cout<<TrainingDataSet[k][j]<<",";


			}
			cout<<endl;
		}



}




void CBAManager::LoadMDPSolutionFile()
{
	int j=0;
	int temps=0;
	char 	spell[500]; 
	int 	iter=0;
	float 	b ;
	int 	MTPSols=0;
	int 	RowNum=50;
	int     ColNum=30;
	char 	toki[1] = {','};
	char 	tok2=',';
	char 	*wow[16];
	char 	*strtokens[5];
	int 	i;
	int 	res;
	int 	DataLenth=0;
	string str;

	std::string FileName = "/home/mk/catkin_ws/src/hri_final_project/Classifier/src/MDpsols.csv";
	ifstream InputFile(FileName.c_str());

	m_MDPsolutionMap.clear();

	if(!InputFile.is_open()){
			cout << "Data load error" << endl;
			exit(0);
		}
		else
		{
			cout<<"Success"<<endl;
			while(!InputFile.eof())
			{
				InputFile.getline(spell, 150);				//
				if (spell[0]=='\0')					//empty line
					continue;
				
				int k=0;
				wow[k] = strtok(spell,",");
				while(wow[k]!=NULL)
					wow[++k] = strtok(NULL,",");	

				for(j=0;j<k;j++)
				{
					str=wow[j];
					str.erase(str.length(),1) ;	
					int ActionPolicy = static_cast<int>(atof(str.c_str()));
					m_MDPsolutionMap.push_back(ActionPolicy);
				}

			}
		}
		InputFile.close();	


}


int CBAManager::getnearestHumanDirection()
{

int Humandirection=0;

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
 innervector[i]=ActionCC[i][0]*pMapParam->NearestHuman_V[0]+ActionCC[i][1]*pMapParam->NearestHuman_V[1];


 Humandirection=getIndexOfLargestElement(innervector);
 Humandirection++;
// float maxvalue =max_element(innervector,innervector+8);
// for(int i(0);i<innervector.size();i++)
// {
// 	if(innervector[i]==maxvalue)
// 		Humandirection=i;
// }


return Humandirection;


//int maxindex=std::max_element(innervector,innervector+innervector.size())-innervector;



// vector<int> RobotCoord=getCurRobotCoord();
//  vector<int> Humancellvector=getHuman_CellVector();  
// vector<int>Humanpos;

// if(Humancellvector.size()>0)
//   Humanpos=CellNum2Coord(Humancellvector[0]);
// else
// {
// 	Humandirection=0;	
// 	return Humandirection;
// }

// int x_diff =Humanpos[0]-RobotCoord[0];
// int y_diff =Humanpos[1]-RobotCoord[1];

// int tempx,tempy=0;

// if(x_diff>0)
//     tempx=1;   
// else if(x_diff<0)
//     tempx=-1;   
// else
//     tempx=0;

// if(y_diff>0)
//     tempy=1;   
// else if(y_diff<0)
//     tempy=-1;   
// else
//     tempy=0;


// for(int i(0);i<Num_action;i++)
// {
//     if((ActionCC[i][0]==tempx) && (ActionCC[i][1]==tempy))
// 	        Humandirection=i;
// }

	return Humandirection;


}

int CBAManager::getIndexOfLargestElement(vector<double> arr) {
    int largestIndex = 0;
    for (int index = largestIndex; index < arr.size(); index++) {
        if (arr[largestIndex] < arr[index]) {
            largestIndex = index;
        }
    }
    return largestIndex;
}

int CBAManager::getRobotHeadingDirection()
{

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
 innervector[i]=ActionCC[i][0]*pMapParam->RobotHeading_V[0]+ActionCC[i][1]*pMapParam->RobotHeading_V[1];

// cout<<"Inner Vector for heading"<<endl;
// 	for(int i(0);i<8;i++)
// 		cout<<innervector[i]<<endl;

 RobotHeading=getIndexOfLargestElement(innervector);
RobotHeading++;
 //cout<<"Robot Heading :" << RobotHeading<<endl;
// float maxvalue =max_element(innervector,innervector+8);
// for(int i(0);i<innervector.size();i++)
// {
// 	if(innervector[i]==maxvalue)
// 		Humandirection=i;
// }


return RobotHeading;

}

int CBAManager::ActionfromGUICmd(int _cmd)
{
	bool IsSave=true;
	bool ReadytoMove=false;
	vector<float> cur_featureV=getFeaturevector();

	for(int i(0);i<cur_featureV.size();i++)
		cout<<cur_featureV[i]<<"\t ";
	cout<<endl;

	std_msgs::Int8 cmd_action;

	switch(_cmd)
	{
		case 10: //Predict				
			boolAuto=false;
			cout<<"predict"<<endl;
			Desiredaction=getDirectionfromCBA(cur_featureV);

			ROS_INFO("predicted policy : %d, Confidence :%.3lf \n", Desiredaction, pClassifier->Confidence);
			
		break;
		case 11: //Good
			cout<<"Good(save)"<<endl;
			SaveCurrentPolicy(storedFeaturevector, Desiredaction);	
			ReadytoMove=true;
			
			//write bad log
			if(Isbad)
			{	
				SaveBadPolicy(storedFeaturevector,Storedbaddecision,Desiredaction);
				Storedbaddecision=0;
				Isbad=false;
			}	

			// HSR_Pub.publish(cmd_action);
			break;
		case 12: //Bad
				cout<<"Bad"<<endl;
				Storedbaddecision=Desiredaction;
				Isbad=true;
				//boolAuto=true;
				// IsSave=false;
				cout<<"Teach me the desired direction"<<endl;
			break;
		case 15: 
				 cout<<"get MDP Solution"<<endl;
				 Desiredaction=getMDPfromFeature();
			break;
		case 13: //Save 
				cout<<"SaveDataFile"<<endl;
				saveCurrentDataFile();
			break;
		case 16: //Load 
				LoadDataFile();
				cout<<"loaded"<<endl;
				updateMaptoVec();
			break;
        case 14: //Auto 
        		cout<<"Auotonomous mode"<<endl;
				// LoadDataFile();
        		boolAuto=!boolAuto;
			break;			
		case 1:  //Turn left
				 cout<<"Desired movement : Turn Left"<<endl;
				 Desiredaction=1;
				 setStoredFeatureVector(cur_featureV);
				 // SaveCurrentPolicy(cur_featureV, Desiredaction);	
				 ReadytoMove=true;
		      break;
		case 2:  //Go Forward
				 cout<<"Desired movement : Go Forward"<<endl;
				 Desiredaction=2;
				 setStoredFeatureVector(cur_featureV);
				 //SaveCurrentPolicy(cur_featureV, Desiredaction);	
				 ReadytoMove=true;
			break;
		case 3:   //Turn right
		  		  cout<<"Desired movement : Turn Right"<<endl;
				  setStoredFeatureVector(cur_featureV);
				  Desiredaction=3;	
			      //SaveCurrentPolicy(cur_featureV, Desiredaction);
			      ReadytoMove=true;
			break;
		case 4 :  //Move
				 cout<<"Move"<<endl;
				 
				 cmd_action.data=Desiredaction;
				 HSR_Pub.publish(cmd_action);
				 ReadytoMove=false;
			break;

		case 5 : //Update
				 cout<<"Update Classifier"<<endl;
				 UpdateClassifier();
				 //Desiredaction=5;
				 //SaveCurrentPolicy(cur_featureV, Desiredaction);
				 
				 ReadytoMove=true;
			break;
		case 6 : cout<<"Desired movement : W-S"<<endl;
				 Desiredaction=6;
				 SaveCurrentPolicy(cur_featureV, Desiredaction);
				 ReadytoMove=true;
			break;

		case 7 : cout<<"Desired movement : S"<<endl;
				 Desiredaction=7;
				 SaveCurrentPolicy(cur_featureV, Desiredaction);
				 ReadytoMove=true;
			break;

		case 8 : cout<<"Desired movement : E-S"<<endl;
				 Desiredaction=8;
				 SaveCurrentPolicy(cur_featureV, Desiredaction);
				 ReadytoMove=true;
			break;

		default: 
		break;

//		default: 

	}


	return 0;

}

void CBAManager::setStoredFeatureVector(const vector<float>& featurevector)
{

	for(int i(0);i<Feature_dim;i++)
		storedFeaturevector[i]=featurevector[i];

}

std::vector<float> CBAManager::getFeaturevector()
{
	std::vector<float> FeatureVector(Feature_dim,0);
	int idx=0;

	// for(int i(0);i<pMapParam->State_Type.size();i++)
	// 	FeatureVector[idx++]=pMapParam->State_Type[i];
	//occupancy type
	FeatureVector[0]=pMapParam->State_Type[1];
	FeatureVector[1]=pMapParam->State_Type[2];
	FeatureVector[2]=pMapParam->State_Type[4];
	FeatureVector[3]=pMapParam->State_Type[7];
	FeatureVector[4]=pMapParam->State_Type[6];
	FeatureVector[5]=pMapParam->State_Type[5];
	FeatureVector[6]=pMapParam->State_Type[3];
	FeatureVector[7]=pMapParam->State_Type[0];

	//distance
	FeatureVector[8]=pMapParam->State_Distance[1];
	FeatureVector[9]=pMapParam->State_Distance[2];
	FeatureVector[10]=pMapParam->State_Distance[4];
	FeatureVector[11]=pMapParam->State_Distance[7];
	FeatureVector[12]=pMapParam->State_Distance[6];
	FeatureVector[13]=pMapParam->State_Distance[5];
	FeatureVector[14]=pMapParam->State_Distance[3];
	FeatureVector[15]=pMapParam->State_Distance[0];

	// for(int j(0);j<pMapParam->State_Distance.size();j++)
	// 	FeatureVector[idx++]=pMapParam->State_Distance[j];
	
	//cout<<"occupancy loaded"<<endl;

	FeatureVector[16]=getnearestGoalDirection();		
	//cout<<"Nearest loaded : "<<getnearestGoalDirection() <<endl;
	FeatureVector[17]=getnearestHumanDirection();	//nearest human
	//cout<<"Human direction"<<getnearestHumanDirection()<<endl;
	//vector<int> RobotCoord=getCurRobotCoord();
	FeatureVector[18]=getMDPfromFeature();			//mdp

//	ROS_INFO("goal D : %d , Human D : %d , MDP sol : %d \n",FeatureVector[16],FeatureVector[17],FeatureVector[18]);


	//cout<<"Heading direction"<<getRobotHeadingDirection()<<endl;

	return FeatureVector;
}


int  CBAManager::Coord2CellNum(std::vector<int> cell_xy)
{
	int index= X_mapSize*cell_xy[0]+cell_xy[1];


	return index;
}

std::vector<int> CBAManager::CellNum2Coord(const int Cell_idx)
{

	  std::vector<int> cell_xy(2,0);
	  //cell_xy.resize(2);
	  
	  int div =Cell_idx / X_mapSize;
	  int res =Cell_idx % X_mapSize;

	  cell_xy[0]=div;
	  cell_xy[1]=res;

	return cell_xy;
}


void CBAManager::SaveCurrentPolicy(const std::vector<float> StateVector, int _Policy)
{
	TrainingDataSet.push_back(StateVector);
	TrainingDataState.push_back(_Policy);

	//Total TrainingVector
	 vector<float>  tempVector(Feature_dim+1);
	 for(int i(0);i<Feature_dim;i++)
	 	tempVector[i]=StateVector[i];
	 tempVector[Feature_dim]=_Policy;

	TotalTrainingDataSet.push_back(tempVector);
	 
	 tempVector.clear();	

}

void CBAManager::SaveBadPolicy(const std::vector<float> StateVector, int bad, int good)
{
	
	// TrainingDataSet.push_back(StateVector);
	// TrainingDataState.push_back(_Policy);
	//Total TrainingVector
	 vector<float>  tempFVector(Feature_dim+2);
	 for(int i(0);i<Feature_dim;i++)
	 	tempFVector[i]=StateVector[i];

	 tempFVector[Feature_dim]=bad;
	 tempFVector[Feature_dim+1]=good;

  	BadDecisionLog.push_back(tempFVector);
	 
	//  tempVector.clear();	

}


bool CBAManager::UpdateClassifier()
{
   	
   	 //FIXEME!!
    // pClassifier->UpdateLearningParameters(TrainingDataSet,TrainingDataState);
   
	 pClassifier->Updatedatalistmap(TrainingDataSet, TrainingDataState);
	 cout<<"Suscces update listmap"<<endl;
	 pClassifier->updateMatrixMap();
	 cout<<"Suscces update matrixmap"<<endl;
	 pClassifier->Learning();
	  cout<<"Suscces Learning"<<endl;


	//Clear Updated dataset
    TrainingDataSet.clear();
    TrainingDataState.clear();


	return  true;

}

int CBAManager::getDirectionfromCBA(const vector<float> featurevector)
{
	int result=0;

	//cout<<"GedDirectionfromCBA"<<endl;

	if(pClassifier->IsLearned)
		{
			cout<<" cmd from Classifier"<<endl;
			result=pClassifier->Classify(featurevector);

		}
	else
	{
		cout<<" cmd from Mdp"<<endl;
		result=getMDPfromFeature();
		//result =floor((rand()/((double) RAND_MAX))*3.2);
		//result = (result % 3)+1;
	}
	
	return result;

}

int CBAManager::getMDPsolution(vector<int> posCoord)
{
	
	 vector<int> globalcoords(2,0);
	 globalcoords[0]=posCoord[0];
	 globalcoords[1]=posCoord[1];
	 int res=0;

	 // cout<<"Local X ~ X"<<Local_X_start<<" , "<<Local_X_end<<endl;
	 // cout<<"Local Y ~ Y"<<Local_Y_start<<" , "<<Local_Y_end<<endl;

	 if(globalcoords[0]<Local_X_start || globalcoords[0]>Local_X_end){

	 	cout<<"out range of local window : X"<<endl;
	 }

	 else if(globalcoords[1]<Local_Y_start || globalcoords[1]>Local_Y_end){

	 	cout<<"out range of local window : Y"<<endl;
	 }

	 else
	 {

	  if(m_MDPsolutionMap.size()>0)
	  { 
	 
		vector<int> localcoords =Global2LocalCoord(globalcoords);

		cout<<"Local coords : x :"<< localcoords[0] << ", y : "<< localcoords[1]<<endl;

		int SI=Local2ScaledCellIdx(localcoords,Scale_constant);

		cout<<"Scaled Indx "<< SI<<endl;

	  	res= m_MDPsolutionMap[SI];
      }
      else
      {
      		cout<<"MDP solution size is zero"<<endl;

      }
   }
	

	return res;


}


int CBAManager::getMDPfromFeature()
{
	 int robotid=pMapParam->robot_map_id;
	 int res=0;

	 	if(m_MDPsolutionMap.size()>0)
		{
			res=m_MDPsolutionMap[robotid];
			//ROS_INFO("mdp sol load id : %d, res : %d", robotid,res);
		}
	return res;
}


int CBAManager::getRobotPos_CellNum()
{
	int cell_robot=0;

	for(int i(0);i<Num_Grids;i++)
	{
		if(m_mapInfo[i]==3)
		{
			cell_robot=i;
		}
	}
	return cell_robot;
}


vector<int> CBAManager::getRobotPos_CellVector()
{
	vector<int> robotcell_V;

	for(int i(0);i<Num_Grids;i++)
	{
		if(m_mapInfo[i]==3)
		{
			robotcell_V.push_back(i);
		}
	}
}
vector<int> CBAManager::getObs_CellVector()
{
	vector<int> Obscell_V;

	for(int i(0);i<Num_Grids;i++)
	{
		if(m_mapInfo[i]==1)
		{
			Obscell_V.push_back(i);
		}
	}
}

vector<int> CBAManager::getHuman_CellVector()
{
	vector<int> Humancell_V;

	for(int i(0);i<Num_Grids;i++)
	{
		if(m_mapInfo[i]==1)
		{
			Humancell_V.push_back(i);
		}
	}

}

vector<int> CBAManager::getStateVector()
{
	vector<int> _stateVector(8,0);

	int 		cell_robot_= getRobotPos_CellNum();
	vector<int> R_pos_Cell = CellNum2Coord(cell_robot_);

	for(int i(0);i<Num_action;i++)
	{
		_stateVector[i]=getStateCell(R_pos_Cell,i);
	}

	return _stateVector;
}	

int CBAManager::getStateCell(vector<int> _Pos, int action)
{
	int res=0;
	vector<int> curPos(2,0);
	
	curPos[0] = _Pos[0];
	curPos[1] = _Pos[1];
	
	curPos[0]+=getMoveVector(action)[0];
	curPos[1]+=getMoveVector(action)[1];

	if(checkNOBoundary(curPos))	
		res=m_mapInfo[Coord2CellNum(curPos)];  //
	else	
			res=1;									//obstacle
	return res;
}


bool CBAManager::checkNOBoundary(vector<int> _Pos)
{
	bool IsNoboundary=true;
	int i=_Pos[0];
	int j=_Pos[1];

	if( (i<0) || (j<0) )
		IsNoboundary= false;
	else if( (i==Grid_Num_X) || (i==Grid_Num_Y))
		IsNoboundary =false;
	else
        IsNoboundary =true;

    return IsNoboundary;
}

vector<int> CBAManager::getMoveVector(int action)
{
	vector<int> moveVector(2,0);

	switch(action)
	{
		case 1:  moveVector[0]=1; moveVector[1]=0;
	    	break;
		case 2:  moveVector[0]=1; moveVector[1]=1;
			break;
		case 3: moveVector[0]=0;  moveVector[1]=1;
			break;
		case 4: moveVector[0]=-1; moveVector[1]=1;
			break;
		case 5: moveVector[0]=-1; moveVector[1]=0;
			break;
		case 6: moveVector[0]=-1; moveVector[1]=-1;
			break;
		case 7: moveVector[0]=0;  moveVector[1]=-1;
			break;
		case 8: moveVector[0]=1;  moveVector[1]=-1;
			break;
		default: moveVector[0]=0; moveVector[1]=0;
			break;
	}
	
	return moveVector;
}

//this function makes data from vector to Map
void CBAManager::ConvertVec2Map()
{	
	int RowSize=TrainingDataSet.size();
	int RowSize2=TrainingDataState.size();
	int ColSize=Dim_feature;
	
	//Variables for counting number of classes
	vector< vector< vector<float> > > Tempvectorset;
	Tempvectorset.resize(Num_action);

	vector<int> CoutClass(Num_action,0);
	
	//the length of TrainingDataSet and TrainingDataState should be same
	if(RowSize!=RowSize2)
		cout<<"Data is wrong! Recheck trainingData"<<endl;

	//Find the number of data ::Numclass
	for(int i(0);i<RowSize;i++)
	{
		for(int j(0); j<Num_action;j++)
		{
			if(TrainingDataState[i]==j)			

				CoutClass[j]+=1;
			    Tempvectorset[j].push_back(TrainingDataSet[i]);
		}

	}

	TrainingDataMap.clear();
	MatrixXd TempMatrix;

	for(int i(0);i<Num_action;i++)
	{
		TempMatrix.resize(CoutClass[i],ColSize);
		for(int k(0);k<CoutClass[i];k++)
			for(int l(0);l<ColSize;l++)
				TempMatrix(k,l)=Tempvectorset[i][k][l];				
		
		TrainingDataMap.insert(std::make_pair(i,TempMatrix));
	}
}



void CBAManager::setMDPSols(vector<int> MDPsolutions)
{
	
	// m_MDPsolutionMap.resize(MDPsolutions.size());
	// for(int i(0); i<MDPsolutions.size();i++)
	// 	{

	// 		m_MDPsolutionMap[i]= MDPsolutions[i];
			
	// 	}

	//  cout<<"MDP sols saved, solution size :"<<m_MDPsolutionMap.size()<<endl;
	// int idx=0;
	// for(int i(20);i<159;i++)
	// 	for(int j(90);j<149;j++)
	// 	{

	// 		cout<<"i : "<<i<<", j : "<<j<<endl;
	// 		cout<<getMDPsolution(i,j)<<endl;

	// 	}
}

void CBAManager::LoadDataFile()
{
	pClassifier->readSignalDataFile();
	pClassifier->Learning();
	saveDatafile2TotalData();

}

void CBAManager::saveDatafile2TotalData()
{
	TotalTrainingDataSet.clear();
	map<int,list<RowVectorXd> >::iterator mapiter=pClassifier->DataListMap.begin();

	for(mapiter;mapiter!=pClassifier->DataListMap.end();mapiter++)
		{

			list<RowVectorXd>::iterator tempiter =mapiter->second.begin();
			vector<float>  tempVector(Feature_dim+1,0.0);
			RowVectorXd temRowvector;

			for(tempiter; tempiter!=mapiter->second.end();tempiter++)			//FIXME: this make error in re-loading 
			{

			 	for(int i(0);i<Feature_dim;i++)
			 	{
					temRowvector=*tempiter;
			 		tempVector[i]=temRowvector(i);

			 	}
			 
			 	tempVector[Feature_dim]=mapiter->first;

				TotalTrainingDataSet.push_back(tempVector);
			}

		}

		cout<<"SaveDatafile2TotalData"<<endl;


}

void CBAManager::saveCurrentDataFile()
{
	 updateMaptoVec();
	
	ofstream TrainingFile;
	ofstream TrainingFileData;
	ofstream TrainingFileState;
	ofstream BadDtaLog;
	
	TrainingFile.open("/home/mk/TotalTrainingFile_recent.csv");
	TrainingFileData.open("/home/mk/TrainingData_recent.csv");
	TrainingFileState.open("/home/mk/TrainingState_recent.csv");
	BadDtaLog.open("/home/mk/BadLog_recent.csv");
	cout<<"Save Data File "<<endl;
	//TrainingStateFile.open("TrainingStateFile.csv");

	int Datasize=TotalTrainingDataSet.size();

	for(int i(0);i<Datasize;i++)
	{
		for(int j(0);j<Feature_dim+1;j++)
		{
			TrainingFile<<TotalTrainingDataSet[i][j]<<',';
		
		}
			//Save State
			TrainingFile<<endl;
	}
	TrainingFile.close();

    //Data
     Datasize=TotalTrainingDataSet.size();

	for(int i(0);i<Datasize;i++)
	{
		for(int j(0);j<Feature_dim;j++)
		{
			TrainingFileData<<TotalTrainingDataSet[i][j]<<',';
		
		}
			//Save State
			TrainingFileData<<endl;
	}
	TrainingFileData.close();


	//State
	for(int i(0);i<Datasize;i++)
	{
	
		TrainingFileState<<TotalTrainingDataSet[i][Feature_dim]<<endl;
		
		
		//Save State
		//	TrainingFileState<<endl;
	}
	TrainingFileState.close();

	//Save Baddecsionlog
	for(int i(0);i< BadDecisionLog.size();i++)
	{
		for(int j(0);j<Feature_dim+2;j++)
			BadDtaLog<<BadDecisionLog[i][j]<<',';

		BadDtaLog<<endl;		
	}
	BadDtaLog.close();

	cout<<"Current Data Size is : "<<TotalTrainingDataSet.size()<<endl;

}

void CBAManager::getMDPsolutionFile(){
	   		int j=0;
		int temps=0;
		std::string FileName = "/home/mk/catkin_ws/src/hri_final_project/Classifier/src/MD.csv";
		ifstream InputFile(FileName.c_str());

		string str;
		RowVectorXd tempDataVec;
		bool    columncheck=false;
		char 	spell[150]; 
		int 	iter=0;
		float 	b ;
		int 	action;
		int 	absIndex=-1, pre_absIndex=-1;
		char 	toki[1] = {','};
		char 	tok2=',';
		char 	*wow[16];
		char 	*strtokens[5];
		int 	i;
		int 	res;
		int 	DataLenth=0;

   		m_MDPsolutionMap.clear();


		//InputFile.open(FileName.c_str());
		if(!InputFile.is_open()){
			cout << "DataState file load error..check file" << endl;
			exit(0);
		}
		else
		{
			iter=0;
			cout<<" Reading Data State File"<<endl;

			while(!InputFile.eof())
			{
				InputFile.getline(spell, 50);				//
				if (spell[0]=='\0')							//empty line
					continue;

				str=strtokens[0] = strtok(spell,",");
				str.erase(str.length(),1);
				action = static_cast<int>(atof(str.c_str()));
				m_MDPsolutionMap.push_back(action);
				
			}
		}

		InputFile.close();	

		for(int i(0);i<m_MDPsolutionMap.size();i++)
			cout<<m_MDPsolutionMap[i]<<endl;


	}