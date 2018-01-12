#include <vector>
#include <fstream>
#include <string>
#include <map>
#include <stdint.h>
#include "ros/ros.h"
#include <ros/package.h>
#include <Eigen/Dense>
#include "srBSpline.h"
#include <tf/tf.h>
#include <cmath>
#include <cfloat>


#define Grid_STEP 0.5
#define Grid_Num_X 24
#define Grid_Num_Y 24

using namespace Eigen;
using namespace std;

class MapParam
{
public :

	MapParam();
	MapParam(int width_,int height_,double res_);
	~MapParam();

    double   		  	map_step;
   	float 		  		m_cell_x_width;
	float 		  		m_cell_y_width;
	int   		  		Num_grid_X;
	int   		  		Num_grid_Y;
	int                 MapSize;
	bool                boolDynamic;

	std::vector<int>    Cell_Info;
	std::vector<int>    OCC_Info;
	std::vector<int>    Robot_localpos;
	std::vector<int>    State_Type;
	std::vector<float>  State_Distance;
	std::vector<float>  NearestHuman_V;
	std::vector<float>  RobotHeading_V;

	void setWidth (int width_);
	void setHeight(int height_);
	void setResolution (double resolution);
	void set_Cell_Info(std::vector<int> _inputCellInfo);
	void set_OCC_Info(std::vector<int> _inputOCCInfo);
	void set_Robot_Info(std::vector<int> _inputRobotInfo);
	void set_State_Type(std::vector<int> _State_Type);
	void set_State_Distance(std::vector<float> _State_Distance);
	void set_NearestHuman_V(std::vector<float> _NearestHuman_V);
	void set_RobotHeading_V(std::vector<float> _RobotHeading_V);
};

