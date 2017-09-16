#include "MapParam.h"

MapParam::MapParam()
{
    map_step=Grid_STEP;
   	m_cell_x_width=0.0;
	m_cell_y_width=0.0;
	Num_grid_X=24;
	Num_grid_Y=24;
	MapSize=Grid_Num_X*Grid_Num_Y;
	Cell_Info.resize(MapSize, 0);
	OCC_Info.resize(MapSize, 0);
	boolDynamic=false;
}

MapParam::MapParam(int width_,int height_,double res_)
{
	setWidth(width_);
	setHeight(height_);
	setResolution(res_);

	m_cell_x_width=0.0;
	m_cell_y_width=0.0;
	MapSize=width_*height_;
	Cell_Info.resize(MapSize, 0);
	OCC_Info.resize(MapSize, 0);
	boolDynamic=true;
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

void MapParam::setResolution(double res_)
{
	map_step=res_;
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
