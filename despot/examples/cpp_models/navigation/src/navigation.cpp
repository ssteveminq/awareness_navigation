#include <queue>

#include "navigation.h"
#include <despot/util/coord.h>

using namespace std;

namespace despot {

/* ==============================================================================
 * NavigationState class
 * ==============================================================================*/

NavigationState::NavigationState() {
}

NavigationState::NavigationState(int _state_id) {
	state_id = _state_id;
}

string NavigationState::text() const {
	return "s" + to_string(state_id);
}

/* ==============================================================================
 * Navigation class
 * ==============================================================================*/
int Navigation::flag_size_ = 8;
int Navigation::flag_bits_ = 3;

Navigation::Navigation() {
	// string map =
	// 	string("mapSize = 3 4\n") + string("0 0 0 0\n")
	// 		+ string(
	// 			"0.94778 0.62 0.31480 0.0540\n")
	// + string("-1 0 0 0\n");
	string map =
		string("mapSize = 8 6\n") + string("0.01 0.01 0.01 0.01 0.01 0.01 \n")
			+ string(
				"0.84778 0.520 0.68 0.520 0.45 0\n")
			+ string(
				"0.64778 0.020 0.001 0.001 0.01 0.01 \n")
			+ string(
				"0.64778 0.60 0.018 0.840 0.72 0.9 \n")
			+ string(
				"0.78 0.20 0.008 0.940 0.01 0.01 \n")
			+ string("0.01 0.01 0.1 0.95 0.01 0.01 \n")
			+ string("0.02 0.0 0.5 0.95 0.01 0.01 \n")
			+ string("-1 0.98 0.01 0.4 0.01 0.2\n");
			std::cout<<map<<endl;
	istringstream iss(map);
	Init(iss);

    //node.advertise<classifier::CBA_NavInfo>("/CBA_grid_occ_topic", 0);
	//Initialize MDP
	Costmap_Pub= node.advertise<std_msgs::Float32MultiArray>("POMDP/costmap", 1);
	States_Pub= node.advertise<std_msgs::Int32MultiArray>("POMDP/states", 1);
	Stateid_pub=node.advertise<std_msgs::Int32>("POMDP/stateid", 1);
	Action_Pub= node.advertise<std_msgs::Int32>("POMDP/action", 1);

}

Navigation::Navigation(string params_file) {
	
	ifstream fin(params_file.c_str(), ifstream::in);
	Init(fin);
	fin.close();
}

void Navigation::Init(istream& is) {
	string tok;
	is >> tok >> tok >> ysize_ >> xsize_;
	states_.resize(flag_size_ * (xsize_ * ysize_ + 1));
	obs_.resize(xsize_ * ysize_ + 1);
	trap_prob_.resize(xsize_ * ysize_ + 1);
	trap_pos_ = xsize_ * ysize_;

	//MK
	int tempsize_=0;

	for (int y = 0; y < ysize_; y++) {
		for (int x = 0; x < xsize_; x++) {
			int pos = y * xsize_ + x;
			for (int i = 0; i < flag_size_; i++){
				states_[flag_size_ * pos + i] = new NavigationState(
					flag_size_ * pos + i);
			  tempsize_=flag_size_ * pos + i;
			}
			std::cout<<"second states size:"<<tempsize_<<std::endl;
			is >> trap_prob_[pos];

			if (trap_prob_[pos] == -1) {
				goal_pos_ = pos;
				trap_prob_[pos] = 0;
			}

			if (y == 0 && x == 0)
				obs_[pos] = 0;
			else if (y == 0 && x == xsize_ - 1)
				obs_[pos] = 1;
			else if (y == ysize_ - 1 && x == 0)
				obs_[pos] = 2;
			else if (y == ysize_ - 1 && x == xsize_ - 1)
				obs_[pos] = 3;
			else
				obs_[pos] = 4;
		}
	}
	obs_[trap_pos_] = 4;
	trap_prob_[trap_pos_] = 1.0;
	for (int i = 0; i < flag_size_; i++) {
		states_[flag_size_ * trap_pos_ + i] = new NavigationState(
			flag_size_ * trap_pos_ + i);
	}

    //MK
	//for(int j=0;j<xsize_ * ysize_ + 1;j++)
	//	cout<<"trap prob : " <<trap_prob_[j]<<",  obs :" <<obs_[j]<<endl;
	//cout<<"states size : "<<states_.size()<<endl;


	// Build transition matrix
	transition_probabilities_.resize(states_.size());
	for (int s = 0; s < states_.size(); s++) {
		transition_probabilities_[s].resize(NumActions());
		std::cout<<"s :"<<s<<std::endl;			
		std::cout<<"---------------"<<s<<std::endl;			

		for (int a = 0; a < NumActions(); a++) {
			int flag = s % flag_size_;
			int pos = s >> flag_bits_;
			int next_pos = NextPosition(pos, a);
			//cout<<"s :"<<s<<", flag :"<<flag<<", pos :" <<pos<<", a :"<<a<<", nextpos : "<<next_pos<<endl;

			if (flag == 0) { // Transition for first step
				if (pos == 0) {
					transition_probabilities_[s][a].push_back(State(1, 1.0));
				//	cout<<"Transition 0 0"<<endl;
				} else { // Self transition
					transition_probabilities_[s][a].push_back(State(s, 1.0));
				//	cout<<"Self transition 1"<<endl;
				}
			} else if (flag == 1) { // Transition for second step
				if (pos == 0) {
				//	cout<<"Transition flag 1 pos 0"<<endl;
					transition_probabilities_[s][a].push_back(
						State(3 + 1 * flag_size_, 0.49));
					transition_probabilities_[s][a].push_back(
						State(3 + 1 * flag_size_, 0.49));
						// State(7 + (xsize_ - 2) * flag_size_, 0.51));
				} else { // Self transition
					transition_probabilities_[s][a].push_back(State(s, 1.0));
				//	cout<<"Self transition 2"<<endl;
				}
			} else if (flag == 3 || flag == 7) { // Transition for remaining steps
				double prob = trap_prob_[next_pos];

				if (1 - prob > 0.0) {
				//	cout<<"Transition flag 3 | flag 7 "<<endl;
					transition_probabilities_[s][a].push_back(
						State(3 + next_pos * flag_size_, (1 - prob) * 0.49));
					transition_probabilities_[s][a].push_back(
						State(7 + next_pos * flag_size_, (1 - prob) * 0.51));
				}

				if (prob > 0.0) {
					
			//		cout<<"Transition flag 3 | flag 7 22"<<endl;

					transition_probabilities_[s][a].push_back(
						State(3 + trap_pos_ * flag_size_, prob * 0.49));
					transition_probabilities_[s][a].push_back(
						State(7 + trap_pos_ * flag_size_, prob * 0.51));
				}
			} else { // Self transition
				transition_probabilities_[s][a].push_back(State(s, 1.0));
			//	cout<<"Self transition 3"<<endl;
			}
		
		
		
		// for(int j(0);j<transition_probabilities_[s][a].size();j++)
		// 	std::cout<<"s :"<<s<<"a : "<<a<<" , "<<"j: "<<j<<", prob :"<<transition_probabilities_[s][a][j];

		// cout<<std::endl;
		}


	}

	//Publish costmap info to ROS 
	 int costmapsize=xsize_* ysize_; 
	costmapvec.resize(costmapsize);
	std_msgs::Float32MultiArray costmap_msg;
	for(int i(0);i<costmapsize;i++){
		costmapvec[i]=trap_prob_[i];
	}
	

	ReachGoal=false;

	//MK;
	// PrintTransitions();
}

bool Navigation::Step(State& s, double random_num, int action, double& reward,
	OBS_TYPE& obs) const {
	NavigationState& state = static_cast<NavigationState&>(s);

	//std::cout<<"current state : "<<state.state_id;

	reward = ((state.state_id >> flag_bits_) == goal_pos_) ? 0 : -1;


	const vector<State>& distribution =
		transition_probabilities_[state.state_id][action];
	double sum = 0;
	for (int i = 0; i < distribution.size(); i++) {
		const State& next = distribution[i];
		sum += next.weight;
		if (sum >= random_num) {
			state.state_id = next.state_id;
			break;
		}
	}

	obs = obs_[state.state_id >> flag_bits_];

	return false;
}

int Navigation::NumStates() const {
	return flag_size_ * (xsize_ * ysize_ + 1);
}

double Navigation::ObsProb(OBS_TYPE obs, const State& state, int a) const {
	
//	int tempflag=state.state_id >> flag_bits_;
//	std::cout<<"ObsProb function : stateid>>bits: "<<tempflag<<endl;
//	int pos__=state.state_id >> flag_bits_;
//	double temp= obs == obs_[state.state_id >> flag_bits_];
//	cout<<"pos :"<<pos__<<", obs : " <<obs<<",  obs[pos] :"<<obs_[pos__]<<",  result : "<<temp<<endl;

   return obs == obs_[state.state_id >> flag_bits_];
}

const vector<State>& Navigation::TransitionProbability(int s, int a) const {
	return transition_probabilities_[s][a];
}

//Return State Idx in map
int Navigation::NextPosition(int pos, int a) const {
	if (a == 4 || pos == trap_pos_ || pos == goal_pos_)
		return pos;

	Coord next = Coord(pos % xsize_, pos / xsize_) + Compass::DIRECTIONS[a];
	if (next.x < 0)
		next.x = 0;
	if (next.x >= xsize_)
		next.x = xsize_ - 1;
	if (next.y < 0)
		next.y = 0;
	if (next.y >= ysize_)
		next.y = ysize_ - 1;

	return next.y * xsize_ + next.x;
}

void Navigation::PrintTransitions() const {
	cout << "Transitions (Start)" << endl;
	for (int s = 0; s < NumStates(); s++) {
		cout
			<< "--------------------------------------------------------------------------------"
			<< endl;
		cout << "State " << s << endl;
		PrintState(*GetState(s));
		for (int a = 0; a < NumActions(); a++) {
			cout << transition_probabilities_[s][a].size()
				<< " outcomes for action " << a << endl;
			for (int i = 0; i < transition_probabilities_[s][a].size(); i++) {
				const State& next = transition_probabilities_[s][a][i];
				cout << "Next = (" << next.state_id << ", " << next.weight
					<< ")" << endl;
				PrintState(*GetState(next.state_id));
			}
		}
	}
	cout << "Transitions (End)" << endl;
}

void Navigation::PrintMDPPolicy() const {
	cout << "MDP (Start)" << endl;
	for (int s = 0; s < NumStates(); s++) {
		cout << "State " << s << "; Action = " << policy_[s].action
			<< "; Reward = " << policy_[s].value << endl;
		PrintState(*(states_[s]));
	}
	cout << "MDP (End)" << endl;
}

State* Navigation::CreateStartState(string type) const {
	// double prob = Random::RANDOM.NextDouble();
	// return new NavigationState((prob <= 0.51) ? (xsize_ - 2) : 1);
	return new NavigationState(0);
}

Belief* Navigation::InitialBelief(const State* start, string type) const {
	vector<State*> particles;

	for (int i = 0; i < 1000; i++)
		particles.push_back(static_cast<NavigationState*>(Allocate(0, 0.001)));

	Belief* belief = new ParticleBelief(particles, this);
	return belief;
}

ParticleUpperBound* Navigation::CreateParticleUpperBound(string name) const {
	if (name == "TRIVIAL") {
		return new TrivialParticleUpperBound(this);
	} else if (name == "MDP" || name == "DEFAULT") {
		return new MDPUpperBound(this, *this);
		 PrintMDPPolicy();
	} else {
		cerr << "Unsupported particle lower bound: " << name << endl;
		exit(0);
		return NULL;
	}
}

ScenarioUpperBound* Navigation::CreateScenarioUpperBound(string name,
	string particle_bound_name) const {

	name="TRIVIAL";

	if (name == "TRIVIAL") {
		return new TrivialParticleUpperBound(this);
	} else if (name == "MDP") {
		return new MDPUpperBound(this, *this);
	} else if (name == "LOOKAHEAD") {
		return new LookaheadUpperBound(this, *this,
			CreateParticleUpperBound(particle_bound_name));
	} else {
		cerr << "Unsupported scenario upper bound: " << name << endl;
		exit(1);
		return NULL;
	}
}

ScenarioLowerBound* Navigation::CreateScenarioLowerBound(string name,
	string particle_bound_name) const {
	const DSPOMDP* model = this;
	const StateIndexer* indexer = this;
	const StatePolicy* policy = this;
	if (name == "TRIVIAL") {
		return new TrivialParticleLowerBound(model);
	} else if (name == "RANDOM") {
		return new RandomPolicy(model,
			CreateParticleLowerBound(particle_bound_name));
	} else if (name == "MODE" || name == "DEFAULT") {
		const_cast<Navigation*>(this)->ComputeDefaultActions("MDP");
		return new ModeStatePolicy(model, *indexer, *policy,
			CreateParticleLowerBound(particle_bound_name));
	} else if (name == "MAJORITY") {
		const_cast<Navigation*>(this)->ComputeDefaultActions("MDP");
		return new MajorityActionPolicy(model, *policy,
			CreateParticleLowerBound(particle_bound_name));
	} else {
		cerr << "Unsupported scenario lower bound: " << name << endl;
		exit(1);
		return NULL;
	}
}

void Navigation::PrintState(const State& s, ostream& out) const {
	char buffer[20];
	//out << "Flag = " << (s.state_id % flag_size_) << endl;
	for (int y = 0; y < ysize_; y++) {
		for (int x = 0; x < xsize_; x++) {
			int pos = y * xsize_ + x;
			if (pos == goal_pos_) {
				if ((s.state_id >> flag_bits_) == pos){
					out << "T     ";
					
				}
				else
					out << "G     ";
			} else {
				if ((s.state_id >> flag_bits_) == pos)
					out << "R     ";
				else {
					sprintf(buffer, "%4.2f ", trap_prob_[pos]);
					out << buffer;
				}
			}
		}
		out << endl;
	}

	int cur_stateid=s.state_id >> flag_bits_;

	std_msgs::Int32 stateid_msg;
	stateid_msg.data = cur_stateid;
	Stateid_pub.publish(stateid_msg);

}

void Navigation::PrintBelief(const Belief& belief, ostream& out) const {
}

void Navigation::PrintObs(const State& state, OBS_TYPE obs,
	ostream& out) const {
	out << obs << endl;
}

void Navigation::PrintAction(int action, ostream& out) const {
	out << Compass::CompassString[action] << endl;

	std_msgs::Int32 action_msg;
	
	if(ReachGoal)
		action_msg.data=5;
	else
		action_msg.data = action;

	Action_Pub.publish(action_msg);

	std_msgs::Float32MultiArray costmap_msg;
	costmap_msg.data = costmapvec;
	Costmap_Pub.publish(costmap_msg);
	
}

State* Navigation::Allocate(int state_id, double weight) const {
	NavigationState* state = memory_pool_.Allocate();
	state->state_id = state_id;
	state->weight = weight;
	return state;
}

State* Navigation::Copy(const State* particle) const {
	NavigationState* state = memory_pool_.Allocate();
	*state = *static_cast<const NavigationState*>(particle);
	state->SetAllocated();
	return state;
}

void Navigation::Free(State* particle) const {
	memory_pool_.Free(static_cast<NavigationState*>(particle));
}

int Navigation::NumActiveParticles() const {
	return memory_pool_.num_allocated();
}

void Navigation::ComputeDefaultActions(string type) {
	cerr << "Default action = " << type << endl;
	if (type == "MDP") {
		ComputeOptimalPolicyUsingVI();
		int num_states = NumStates();
		default_action_.resize(num_states);

		double value = 0;
		for (int s = 0; s < num_states; s++) {
			default_action_[s] = policy_[s].action;
			value += policy_[s].value;
		}
		cerr << "MDP upper bound " << policy_[0].value << endl;
	} else {
		cerr << "Unsupported default action type " << type << endl;
		exit(0);
	}
}

int Navigation::GetAction(const State& state) const {
	return default_action_[GetIndex(&state)];
}

double Navigation::Reward(int s, int action) const {
	return ((s >> flag_bits_) == goal_pos_) ? 0 : -1;
}

} // namespace despot
