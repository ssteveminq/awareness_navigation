
#include "classifier.h"
#include <vector>
#include <fstream>
#include <string>
#include <map>
#include <stdint.h>
#include "ros/ros.h"
#include <ros/package.h>
#include "cba_msgs/CBA_NavInfo.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Int8.h"


#define Grid_STEP 10
#define Grid_Num_X 10
#define Grid_Num_Y 10

#define Start_X 120 
#define Start_Y 40
//80 20
#define Goal_X 80
#define Goal_Y 20

#define Dim_feature 19
#define Num_action 8

//typedef (std::vector<uint8_t>) DV;
//Map discription

/*
 0  : Freespace
 1  : Obstacles (Hallway)
 2  : Human
*/
class MapParam
{
public :

	MapParam();
	~MapParam();

    int   		  map_step;
   	float 		  m_cell_x_width;
	float 		  m_cell_y_width;
	int   		  Num_grid_X;
	int   		  Num_grid_Y;
	int 		  robot_map_id;
	vector<int>   Cell_Info;
	vector<int>   OCC_Info;
	vector<int>   Robot_localpos;
	vector<int>   State_Type;
	vector<float> State_Distance;
	vector<float> NearestHuman_V;
	vector<float> RobotHeading_V;

	void setWidth (int Width_);
	void setHeight(int height_);
	void set_Cell_Info(vector<int> _inputCellInfo);
	void set_OCC_Info(vector<int> _inputOCCInfo);
	void set_Robot_Info(vector<int> _inputRobotInfo);
	void set_State_Type(vector<int> _State_Type);
	void set_State_Distance(vector<float> _State_Distance);
	void set_NearestHuman_V(vector<float> _NearestHuman_V);
	void set_RobotHeading_V(vector<float> _RobotHeading_V);
	void set_RobotId(int _robotid);
};

class CBAManager
{
public:
	CBAManager();
	~CBAManager();

    ELMClassifier* pClassifier;
    MapParam*		pMapParam;    
   // ros::Publisher  Matlab_Pub;
   
    ros::Publisher  HSR_Pub;

    //Env_index :
    // 0: free
    // 1: Obstacle
    // 2: Human
    //	 3: Robot 

    //Map paratmeters
    vector<int>  m_mapInfo;    
    map<int,int> m_MapState;					    //Map : (cellNumber, Env_index)  
    map<int,int> m_VisibleMap;				    	//Map : (cellNumber, Env_index)
    vector<int>  m_MDPsolutionMap;				    //Map : (cellNumber, MDP sols)  	
    
   	vector<int>  m_Start;							//Start position of(x,y)
	vector<int>  m_Goal;							//Goal position of (x,y)
	vector<int>  m_Robot;					    	//Current Robot position of (x,y)
	vector<float> m_unitGoal;

	vector<float> storedFeaturevector;

	int 	     Feature_dim;
	int          X_mapSize;
	int          Num_Grids;

	int          Local_X_start;
	int          Local_Y_start;
	int          Local_X_end;
	int          Local_Y_end;
	int          Scale_constant;
	bool		 boolMatlabsendData;
	float		 robot_global_x_pos;
    float 		 robot_global_y_pos;
	float		 robot_theta_yaw;


	//Set function
	void Init();								 //Initialize function
	void setCalssfier(int dim, int Num_actions);
	void setStartConfig (const vector<int> Start );
	void setGoalConfig (const vector<int> Goal);
	void SaveCurrentPolicy(const std::vector<float> StateVector, int _Policy);
	void SaveBadPolicy(const std::vector<float> StateVector, int bad, int good);
    bool UpdateClassifier();
    void ConvertVec2Map();
    void IntializeROS_publisher();
    
    void        updateMaptoVec();


    vector<vector<float> > TrainingDataSet;              // Row, col
    vector<int> 		 TrainingDataState;    		  // class information
    map<int, MatrixXd>   TrainingDataMap;				 //data type for learning

    vector<vector<float> > TotalTrainingDataSet;              // Row, col, State(last column)
    vector<vector<float> >  BadDecisionLog;

    //Get function
	vector<int> 	getGoalConfig();
	vector<int> 	CellNum2Coord(const int Cell_idx);
	int  			Coord2CellNum(vector<int> cell_xy);

	vector<int>     Global2LocalCoord(vector<int> Global_coord);
	int             Local2ScaledCellIdx(vector<int> Local_coord,int scale);

	vector<int>     Global_Coord;
	vector<int>     Local_Coord;
	vector<int>     L_Scaled_Coord;
	
	//Get Nearest direction
	vector<float> 	getFeaturevector();
	vector<int> 	getStateVector();								    	//get surroundingVector
	int 			getnearestGoalDirection();							 	//get direction of Goal
	int 			getnearestHumanDirection();	
	int 			getRobotHeadingDirection();																		//get direction of Human
    int 			getMDPsolution(vector<int> posCoord);						//get direction of MDPsolution
    int             getMDPfromFeature();
    int  			getDirectionfromCBA(const vector<float> featurevector);
    int  			getRobotPos_CellNum();
	vector<int> 	getRobotPos_CellVector();
	vector<int> 	getObs_CellVector();
	vector<int> 	getHuman_CellVector();
	bool			checkNOBoundary(vector<int> _Pos);
	int         	getStateCell(vector<int> _Pos, int action);
	vector<int> 	getMoveVector(int action);
	int			    ActionfromGUICmd(int _cmd);
	void            setMDPSols(vector<int> MDPsolutions);
	void 			saveCurrentDataFile();
	void            LoadDataFile();
	void            LoadMDPSolutionFile();
	int 			getCurRobotIdx();
	vector<int> 	getCurRobotCoord();
	void            saveCurrentMDPsols();
	void            getMDPsolutionFile();
	int 			getIndexOfLargestElement(vector<double> arr);
	void 		    saveDatafile2TotalData();
	void 			setStoredFeatureVector(const vector<float>& featurevector);
	int             Desiredaction;
	bool 			boolAuto;
	bool            Isbad;
	int             Storedbaddecision;





//    void            SaveDataFile

	//std::vector<int>& MDPsolutions;

};

