#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int32.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <despot/core/pomdp.h>
#include <despot/solver/pomcp.h>
#include <despot/core/mdp.h>
#include <despot/util/coord.h>
#include <despot/util/grid.h>

namespace despot {

/* ==============================================================================
 * NavigationState class by MK 
 * ==============================================================================*/

class NavigationState: public State {
public:

	Coord robot_pos;
	int stepSize;				//velocity
	std::vector<Coord> ped_pos;
	std::vector<int>   ped_dir;
	Coord goal_pos;
	std::vector<bool> Goal; // bit vector
	std::vector<int> eyecontact;
	//int power_steps;
	


	NavigationState();
	NavigationState(int _state_id);

  std::string text() const;
};

class Navigation;
class NavigationBelief: public ParticleBelief {
protected:
	const Navigation* Navigation_;
public:
	static int num_particles;
			
	NavigationBelief(std::vector<State*> particles, const DSPOMDP* model, Belief* prior =
		NULL);
	void Update(int action, OBS_TYPE obs);
};



/* ==============================================================================
 * Navigation class
 * ==============================================================================*/
class Navigation: public DSPOMDP {
public:
	//New
	Grid<int> localmap;
	int path_index;
	int num_peds_, passage_y_, pedes_range_;
	double reward_goal, reward_default_, reward_collision_,reward_static_obs_, reward_stop;
	double chase_prob_, defensive_slip_;
	int smell_range_;
	//------------
	int m_xsize, m_ysize;
	int goal_pos_, trap_pos_;
	std::vector<Coord> ped_path1;
	std::vector<Coord> robot_path;
	std::vector<int> ped_pathdir;	

	std::ofstream m_fileout;
	std::ofstream m_Beliefout;

	ros::NodeHandle node;
	ros::Publisher   Action_Pub;
	ros::Publisher   States_Pub;
	ros::Publisher   Costmap_Pub;
	ros::Publisher	 Stateid_pub; 	
    ros::Subscriber  StateVec_Sub;
    std_msgs::Int32  action_cmd;
    std_msgs::Float32MultiArray featureVector;
    // ros::Publisher  Robotcmd_Pub;

	bool ReachGoal;

	std::vector<float> costmapvec;
	std::vector<double> trap_prob_;      // trap_prob_[y * xsize_ + x]
	std::vector<OBS_TYPE> obs_;		 // obs_[y * xsize_ + x]
	std::vector<NavigationState*> states_;

	std::vector<std::vector<std::vector<State> > > transition_probabilities_; //state, action, [state, weight]

	mutable MemoryPool<NavigationState> memory_pool_;

	std::vector<int> default_action_;

protected:
	void Init();
	int NextPosition(int pos, int action) const;

public:
	enum {
		E_PASSABLE, E_SEED, E_POWER
	};
	
	Navigation(int xsize, int ysize_);
	Navigation(std::string params_file = "DEFAULT");
	~Navigation(){m_fileout.close();}



	virtual bool Step(State& state, double random_num, int action,
		double& reward, OBS_TYPE& obs) const;
	inline int NumActions() const {
		return 2;
	}
	int NumStates() const;
	inline int GetIndex(const State* state) const {
		return state->state_id;
	}
	inline const State* GetState(int index) const {
		return states_[index];
	}

	virtual double ObsProb(OBS_TYPE obs, const State& state, int action) const;
	const std::vector<State>& TransitionProbability(int s, int a) const;
	double Reward(int s, int a) const;

	State* CreateStartState(std::string type = "DEFAULT") const;
	//virtual Belief* InitialBelief(const State* start, std::string type = "DEFAULT") const;
	virtual Belief* InitialBelief(const State* start, std::string type = "PARTICLE") const;

	inline double GetMaxReward() const {
		return reward_goal;
	}
	ScenarioUpperBound* CreateScenarioUpperBound(std::string name = "DEFAULT",
		std::string particle_bound_name = "DEFAULT") const;

	inline ValuedAction GetMinRewardAction() const {
		return ValuedAction(0, reward_static_obs_);
	}
	ParticleLowerBound* CreateParticleLowerBound(std::string name = "DEFAULT") const;
	ScenarioLowerBound* CreateScenarioLowerBound(std::string name = "DEFAULT",
		std::string particle_bound_name = "DEFAULT") const;

	POMCPPrior* CreatePOMCPPrior(std::string name = "DEFAULT") const;

	void PrintState(const State& state, std::ostream& out = std::cout) const;
	virtual void PrintBelief(const Belief& belief, std::ostream& out = std::cout) const;
	virtual void PrintObs(const State& state, OBS_TYPE obs, std::ostream& out = std::cout) const;
	void PrintAction(int action, std::ostream& out = std::cout) const;

	virtual void SaveBelief(const Belief& belief);	
	void PrintTransitions() const;
	void PrintMDPPolicy() const;
	
	State* Allocate(int state_id, double weight) const;
	State* Copy(const State* particle) const;
	void Free(State* particle) const;
	int NumActiveParticles() const;

	void ComputeDefaultActions(std::string type);
	int GetAction(const State& navistate) const;
	
	virtual void saveLastAction(State& state, int action);

	//new
	void MovePed(NavigationState& navstate, int g, Random &random) const;
	void MovePedAggressive(NavigationState& navstate, int g, Random &random) const;
	void MovePedDefensive(NavigationState& navstate, int g, Random &random) const;
	void MovePedRandom(NavigationState& navstate, int g, Random &random) const;
	void MovePedPath(NavigationState& navstate, int g, Random &random) const;
	int  SeePed2(const NavigationState& navstate, int action) const;
	bool CheckPedCollision(NavigationState& navstate, int g,int step_col) const;
	bool CheckPosCollision(const Coord& currentPos, NavigationState& navstate, int step_col) const;
	bool CheckAllPedCollision(const NavigationState& navstate, const Coord& pos) const;

	int SeePed(const NavigationState& navstate, int action) const;
	Coord NextPos(const Coord& from, int dir) const;
	Coord NextPosPed(const NavigationState& navstate,const Coord& from) const;
	Coord NextPos_Path(const Coord& from, int action_, int cur_pathidx) const;
	bool SmellGoal(const NavigationState& navstate) const;

	bool Passable(const Coord& pos) const {
		return CheckFlag(localmap(pos), E_PASSABLE);
	}

	int MakeObservations(const NavigationState& navstate) const;
	bool LocalMove(State& state, const History& history, int obs) const;
    void statesVec_callback(const std_msgs::Int32MultiArray::ConstPtr &msg);
    void Sending_cmd_Robot();

    bool StepReal(State& state, double random_num, int action,
		double& reward, OBS_TYPE& obs);
    bool sendcmd();
    //void int_callback(const std_msgs::Int32::ConstPtr &msg){};
            
	
};

} // namespace despot

#endif
