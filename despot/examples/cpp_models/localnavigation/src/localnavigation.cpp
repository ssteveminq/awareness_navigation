#include <queue>

#include "localnavigation.h"
#include <despot/util/coord.h>

static int roudndNUM=0;
using namespace std;

namespace despot {

int NavigationBelief::num_particles = 5000;

/* ==============================================================================
 * NavigationState class
 * ==============================================================================*/
NavigationBelief::NavigationBelief(vector<State*> particles, const DSPOMDP* model,
	Belief* prior) :
	ParticleBelief(particles, model, prior),
	Navigation_(static_cast<const Navigation*>(model)){}

void NavigationBelief::Update(int action, OBS_TYPE obs){
	
	cout<<"Update Belief"<<endl;
	
	roudndNUM++;
	history_.Add(action, obs);
	
	vector<State*> updated;
	double reward;
	OBS_TYPE o;
	int cur = 0, N = particles_.size(), trials = 0;
	while (updated.size() < num_particles && trials < 10 * num_particles) {
		State* particle = Navigation_->Copy(particles_[cur]);
		bool terminal = Navigation_->Step(*particle, Random::RANDOM.NextDouble(),
			action, reward, o);

		if ((!terminal && o == obs)
			|| Navigation_->LocalMove(*particle, history_, obs)) {
			updated.push_back(particle);
		} else {
			Navigation_->Free(particle);
		}

		cur = (cur + 1) % N;

		trials++;
	}

	for (int i = 0; i < particles_.size(); i++)
		Navigation_->Free(particles_[i]);

	particles_ = updated;

	for (int i = 0; i < particles_.size(); i++)
		particles_[i]->weight = 1.0 / particles_.size();
}


class LocalnavigationParticleUpperBound: public ParticleUpperBound {
protected:
	const Navigation* navigation_;
public:
	LocalnavigationParticleUpperBound(const Navigation* model) :
		navigation_(model) {

		cout<<"CreateLocalNaviUpperBound"<<endl;
	}

	double Value(const State& state) const {
			//cout<<"LocalnavigationParticleUpperBound"<<endl;
		const NavigationState& navstate = static_cast<const NavigationState&>(state);
		return (navigation_->reward_goal);
	}
};

class LocalApproxScenarioUpperBound: public ScenarioUpperBound {
protected:
		const Navigation* navigation_;
public:
	LocalApproxScenarioUpperBound(const Navigation* model) :
		navigation_(model) {
			cout<<"LocalApproxScenarioUpperBound"<<endl;
	}


	double Value(const vector<State*>& particles,
		RandomStreams& streams, History& history) const {
		cout<<"senariouvalue"<<endl;
		double total_value = 0;
		for (int i = 0; i < particles.size(); i++) {
			NavigationState& state = static_cast<NavigationState&>(*(particles[i]));
			double value = 0;

			int max_dist = 0;
			for (int i = 0; i < state.Goal.size(); i++) {
				if (state.Goal[i] != 1)
					continue;
				Coord goal_pos = navigation_->localmap.GetCoord(i);
				int dist = Coord::ManhattanDistance(state.robot_pos, goal_pos);
				value += navigation_->reward_goal * Globals::Discount(dist);
				max_dist = max(max_dist, dist);
			}

			// Clear level
			value += navigation_->reward_goal * pow(Globals::config.discount, max_dist);

			// Default move-reward
			value += navigation_->reward_default_ * (Globals::Discount() < 1
					? (1 - Globals::Discount(max_dist)) / (1 - Globals::Discount())
					: max_dist);

			// If pocman is chasing a ghost, encourage it
			// if (state.power_steps > 0 && history.Size() &&
			// 		(history.LastObservation() & 15) != 0) {
			// 	int act = history.LastAction();
			// 	int obs = history.LastObservation();
			// 	if (CheckFlag(obs, act)) {
			// 		bool seen_ped = false;
			// 		for (int dist = 1; !seen_ped; dist++) {
			// 			Coord ped_pos = state.robot_pos + Compass::DIRECTIONS[act] * dist;
			// 			for (int g = 0; g < navigation_->num_peds_; g++)
			// 				if (state.ped_pos[g] == ped_pos) {
			// 					value += navigation_->reward_goal * Globals::Discount(dist);
			// 					seen_ped = true;
			// 					break;
			// 				}
			// 		}
			// 	}
			// }

			// Ped penalties
			double dist = 0;
			for (int g = 0; g < navigation_->num_peds_; g++)
				dist += Coord::ManhattanDistance(state.robot_pos, state.ped_pos[g]);
			value += navigation_->reward_collision_ * pow(Globals::Discount(), dist / navigation_->num_peds_);

			// Penalize for doubling back, but not so much as to prefer hitting a wall
			if (history.Size() >= 2 &&
					Compass::Opposite(history.Action(history.Size() - 1))
					== history.Action(history.Size() - 2))
				value += navigation_->reward_static_obs_ / 2;

			total_value += state.weight * value;
		}

		return total_value;
	}
};



class LocalLegalParticleLowerBound: public ParticleLowerBound {
protected:
	const Navigation* navigation_;
public:
	LocalLegalParticleLowerBound(const DSPOMDP* model) :
		ParticleLowerBound(model),
		navigation_(static_cast<const Navigation*>(model)) {
	cout<<"CreateLocalNaviLowerBound"<<endl;

	}

	ValuedAction Value(const vector<State*>& particles) const {

		//cout<<"LocalLegalParticleLowerBound"<<endl;

		const NavigationState& navstate =
			static_cast<const NavigationState&>(*particles[0]);
		vector<int> legal;
		for (int a = 0; a < 2; ++a) {
			Coord newpos = navigation_->NextPos(navstate.robot_pos, a);
			if (newpos.x >= 0 && newpos.y >= 0)
				legal.push_back(a);
		}
		return ValuedAction(legal[Random::RANDOM.NextInt(legal.size())],
			State::Weight(particles)
				* (navigation_->reward_collision_
					+ navigation_->reward_default_ / (1 - Globals::Discount())));
	}
};

/* ==============================================================================
 * LocalNaviSmartPolicy class
 * ==============================================================================*/
class LocalnaviSmartPolicy : public Policy {
protected:
	const Navigation* navigation_;
	mutable vector<int> preferred_;
	mutable vector<int> legal_;
public:
	LocalnaviSmartPolicy(const Navigation* model, ParticleLowerBound* bound) :
		Policy(model, bound),
		navigation_(model) {
			cout<<"LocalnaviSmartPolicy"<<endl;
	}

	int Action(const vector<State*>& particles, RandomStreams& streams,
		History& history) const {

		//cout<<"LocalnaviSmartPolicy-Action"<<endl;
		const NavigationState& navstate =
			static_cast<const NavigationState&>(*particles[0]);
		preferred_.clear();
		legal_.clear();

		if (history.Size()) {
			int action = history.LastAction();
			OBS_TYPE observation = history.LastObservation();

				 // Otherwise avoid observed ghosts and avoid changing directions
				for (int a = 0; a < 4; a++) {
					Coord newpos = navigation_->NextPos(navstate.robot_pos, a);
					if (newpos.x >= 0 && newpos.y >= 0
						&& !CheckFlag(observation, a)
						&& Compass::Opposite(a) != action)
						preferred_.push_back(a);
				}
		
			if (preferred_.size() > 0)
				return preferred_[Random::RANDOM.NextInt(preferred_.size())];
		}

		for (int a = 0; a < 4; ++a) {
			Coord newpos = navigation_->NextPos(navstate.robot_pos, a);

		//cout<<"Cur pos.x"<<navstate.robot_pos.x<<", Cur pos.y :"<< navstate.robot_pos.y <<", Next pos.x"<<newpos.x<<", Next pos.y :"<< newpos.y <<endl;
		if (newpos.x >= 0 && newpos.y >= 0)
				legal_.push_back(a);
		}

		//cout<<"legal_size : "<<legal_.size()<<endl;
		return legal_[Random::RANDOM.NextInt(legal_.size())];
	}
};


/* ==============================================================================
 * PocmanPOMCPPrior class
 * ==============================================================================*/
class LocalNaviPOMCPPrior: public POMCPPrior {
private:
	const Navigation* navigation_;

public:
	LocalNaviPOMCPPrior(const Navigation* model) :
		POMCPPrior(model),
		navigation_(model) {
				cout<<"LocalNaviPOMCPPrior"<<endl;
	}

	void ComputePreference(const State& state) {
		cout<<"computePreference"<<endl;
		const NavigationState& navstate = static_cast<const NavigationState&>(state);
		legal_actions_.clear();
		preferred_actions_.clear();

		for (int a = 0; a < 4; a++) {
			Coord newpos = navigation_->NextPos(navstate.robot_pos, a);
			if (newpos.x >= 0 && newpos.y >= 0)
				legal_actions_.push_back(a);
		}

		if (history_.Size()) {
			int action = history_.LastAction();
			OBS_TYPE observation = history_.LastObservation();

			// If power pill and can see a ghost then chase it
			if (navstate.power_steps > 0 && ((observation & 15) != 0)) {
				for (int a = 0; a < 4; a++)
					if (CheckFlag(observation, a))
						preferred_actions_.push_back(a);
			} else { // Otherwise avoid observed ghosts and avoid changing directions
				for (int a = 0; a < 4; a++) {
					Coord newpos = navigation_->NextPos(navstate.robot_pos, a);
					if (newpos.x >= 0 && newpos.y >= 0
						&& !CheckFlag(observation, a)
						&& Compass::Opposite(a) != action)
						preferred_actions_.push_back(a);
				}
			}
		}
	}
};


NavigationState::NavigationState(){
	// Coord robot_pos;
	stepSize =1;	
	eyecontact=false;	
	power_steps=15;		//velocity
		
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
// int Navigation::flag_size_ = 8;
// int Navigation::flag_bits_ = 3;

Navigation::Navigation(int xsize, int ysize):localmap( xsize, ysize), num_peds_(3),pedes_range_(2),	smell_range_(2), round_num(0){
	
	// cout<<xsize<<","<<ysize<<endl;
	// cout<<"localmap"<<endl;
	// Grid xx= Grid(xsize,ysize);
	m_xsize=xsize;
	m_ysize=ysize;
	
	Init();

	//node.advertise<classifier::CBA_NavInfo>("/CBA_grid_occ_topic", 0);
	//Initialize MDP
	Costmap_Pub= node.advertise<std_msgs::Float32MultiArray>("POMDP/costmap", 1);
	States_Pub= node.advertise<std_msgs::Int32MultiArray>("POMDP/states", 1);
	Stateid_pub=node.advertise<std_msgs::Int32>("POMDP/stateid", 1);
	Action_Pub= node.advertise<std_msgs::Int32>("POMDP/action", 1);

}

Navigation::Navigation(string params_file) {
	
	ifstream fin(params_file.c_str(), ifstream::in);
	//Init(fin);
	fin.close();
}

void Navigation::Init() {
	
	cout<<"Initialize"<<endl;
	//num_peds_ = 4;
	passage_y_ = 10;
	chase_prob_=0.75;
	reward_goal=10000;
	reward_default_=-1;
	reward_collision_=-1000;
	reward_static_obs_=-25;
	defensive_slip_=0.25;

	ped_path1.resize(25);
	ped_path1[0]=Coord(7,7);
	ped_path1[1]=Coord(6,7);
	ped_path1[2]=Coord(5,7);
	ped_path1[3]=Coord(4,7);
	ped_path1[4]=Coord(3,7);
	ped_path1[5]=Coord(3,6);
	ped_path1[6]=Coord(4,6);
	ped_path1[7]=Coord(5,6);
	ped_path1[8]=Coord(6,6);
	ped_path1[9]=Coord(6,5);
	ped_path1[10]=Coord(5,5);
	ped_path1[11]=Coord(4,5);
	ped_path1[12]=Coord(3,5);
	ped_path1[13]=Coord(4,5);
	ped_path1[14]=Coord(5,5);

	ped_pathdir.resize(15);
	ped_pathdir[0]=3;
	ped_pathdir[1]=3;
	ped_pathdir[2]=3;
	ped_pathdir[3]=3;
	ped_pathdir[4]=0;
	ped_pathdir[5]=1;
	ped_pathdir[6]=1;
	ped_pathdir[7]=1;
	ped_pathdir[8]=1;
	ped_pathdir[9]=0;
	ped_pathdir[10]=3;
	ped_pathdir[11]=3;
	ped_pathdir[12]=3;
	ped_pathdir[13]=1;
	ped_pathdir[14]=1;


	// ped_path1[15]=Coord(5,5);
	// ped_path1[16]=Coord(5,5);
	// ped_path1[17]=Coord(5,5);
	// ped_path1[18]=Coord(5,5);
	// ped_path1[19]=Coord(5,5);
	// ped_path1[20]=Coord(5,5);
	// ped_path1[21]=Coord(5,5);
	// ped_path1[22]=Coord(5,5);
	// ped_path1[23]=Coord(5,5);
	//ped_path1[24]=Coord(5,5);



	//Initialize states,obs,trap_probbiliey(belief)
	// states_.resize(flag_size_ * (m_xsize * m_ysize + 1));
	// obs_.resize(m_xsize * m_ysize + 1);
	// trap_prob_.resize(m_xsize * m_ysize + 1);
	// trap_pos_ = m_xsize * m_ysize;

	//MK
	//Publish costmap info to ROS 
	//  int costmapsize=m_xsize* m_ysize; 
	// costmapvec.resize(costmapsize);
	// std_msgs::Float32MultiArray costmap_msg;
	// for(int i(0);i<costmapsize;i++){
	// 	costmapvec[i]=trap_prob_[i];
	// }
	
	ReachGoal=false;

	//MK;
	// PrintTransitions();
}

bool Navigation::Step(State& s, double random_num, int action, double& reward,
	OBS_TYPE& obs) const {
	
	//cout<<"STEP"<<endl;
	NavigationState& navstate = static_cast<NavigationState&>(s);
	Random random(random_num);
	
	//set to the default_reward
	reward = reward_default_;
	obs= 0;

	//estimate next pose = Current state + action // 
	Coord newpos = NextPos(navstate.robot_pos, action);

	if (newpos.x >= 0 && newpos.y >= 0 && newpos.x < localmap.xsize() && newpos.y < localmap.ysize())
		navstate.robot_pos = newpos;
	else
		reward += reward_static_obs_;

	int pedCollision = -1;
	for (int g = 0; g < num_peds_; g++) {
		if (navstate.ped_pos[g] == navstate.robot_pos)
			pedCollision = g;
		    MovePed(navstate, g, random);
		  if (navstate.ped_pos[g] == navstate.robot_pos)
		 	pedCollision = g;
	}

	//if collision occurs.. calculation reward
	if (pedCollision >= 0) {
		reward += reward_collision_;
		return false;
	}

	// measurement 
	obs = MakeObservations(navstate);

	//localindex in localmap;
	int naviIndex = localmap.Index(navstate.robot_pos);

	if (navstate.Goal[naviIndex]) {
		   reward += reward_goal;
			return true;
		}
		
	return false;
}

int Navigation::NumStates() const {
	return (m_xsize * m_ysize);
}

double Navigation::ObsProb(OBS_TYPE obs, const State& state, int a) const {
	
return obs == MakeObservations(static_cast<const NavigationState&>(state));
		
}

const vector<State>& Navigation::TransitionProbability(int s, int a) const {
	return transition_probabilities_[s][a];
}

//Return State Idx in map
int Navigation::NextPosition(int pos, int a) const {
	// if (a == 4 || pos == trap_pos_ || pos == goal_pos_)
	// 	return pos;

	// Coord next = Coord(pos % m_xsize, pos / m_xsize) + Compass::DIRECTIONS[a];
	// if (next.x < 0)
	// 	next.x = 0;
	// if (next.x >= m_xsize)
	// 	next.x = m_xsize - 1;
	// if (next.y < 0)
	// 	next.y = 0;
	// if (next.y >= m_ysize)
	// 	next.y = m_ysize - 1;

	//return next.y * m_xsize + next.x;
	return 0;
}

void Navigation::PrintTransitions() const {
	// cout << "Transitions (Start)" << endl;
	// for (int s = 0; s < NumStates(); s++) {
	// 	cout
	// 		<< "--------------------------------------------------------------------------------"
	// 		<< endl;
	// 	cout << "State " << s << endl;
	// 	PrintState(*GetState(s));
	// 	for (int a = 0; a < NumActions(); a++) {
	// 		cout << transition_probabilities_[s][a].size()
	// 			<< " outcomes for action " << a << endl;
	// 		for (int i = 0; i < transition_probabilities_[s][a].size(); i++) {
	// 			const State& next = transition_probabilities_[s][a][i];
	// 			cout << "Next = (" << next.state_id << ", " << next.weight
	// 				<< ")" << endl;
	// 			PrintState(*GetState(next.state_id));
	// 		}
	// 	}
	// }
	// cout << "Transitions (End)" << endl;
}

void Navigation::PrintMDPPolicy() const {
	// cout << "MDP (Start)" << endl;
	// for (int s = 0; s < NumStates(); s++) {
	// 	cout << "State " << s << "; Action = " << policy_[s].action
	// 		<< "; Reward = " << policy_[s].value << endl;
	// 	PrintState(*(states_[s]));
	// }
	// cout << "MDP (End)" << endl;
}

//Initialize start state
State* Navigation::CreateStartState(string type) const {
	
	NavigationState* startState = memory_pool_.Allocate();
	
	startState->stepSize=1;
	startState->ped_pos.resize(num_peds_);
	startState->ped_dir.resize(num_peds_);
	
	startState->goal_pos.x=7;
	startState->goal_pos.y=7;

	//set moving pedestrian intial position

	for (int p = 0; p< num_peds_; p++){ 
		startState->ped_pos[p]= Coord(5,5);
		startState->ped_pos[p].x += p % 2;
		startState->ped_pos[p].y += p / 2;
		startState->ped_dir[p] = -1;
	}

	startState->power_steps=0;
	startState->eyecontact=false;

	startState->Goal.resize(localmap.xsize()*localmap.ysize());

	for (int x = 0; x < localmap.xsize(); x++) {
		for (int y = 0; y < localmap.ysize(); y++) {
			int navIndex = localmap.Index(x, y);
			 //cout << localmap(x, y) << " " << CheckFlag(localmap(x, y), E_SEED) << " " << CheckFlag(localmap(x, y), E_POWER) << endl;
			if (startState->goal_pos.x==x && startState->goal_pos.y==y){
				startState->Goal[navIndex] = 1;
			}
			else {
				startState->Goal[navIndex] = 0;
			}
		}
	}






	return startState;
}

Belief* Navigation::InitialBelief(const State* start, string type) const {
	
	std::cout<<"Intialize Belief"<<endl;
	int N = NavigationBelief::num_particles;
	vector<State*> particles(N);

	for (int i = 0; i < N; i++){
		particles[i]=CreateStartState();
		particles[i]->weight = 1.0/ N;
	}
	
	return new NavigationBelief(particles,this);
}


// ParticleUpperBound* Navigation::CreateParticleUpperBound(string name) const {
// 	if (name == "TRIVIAL") {
// 		return new TrivialParticleUpperBound(this);
// 	} else if (name == "APPROX") {
// 		return new LocalApproxScenarioUpperBound(this);
// 	} else if (name == "SMART" || name == "DEFAULT") {
// 		return new LocalnavigationParticleUpperBound(this);
// 	} else {
// 		cerr << "Unsupported base upper bound: " << name << endl;
// 		exit(1);
// 		return NULL;
// 	}


//}

ScenarioUpperBound* Navigation::CreateScenarioUpperBound(string name,
	string particle_bound_name) const {

	if (name == "TRIVIAL") {
		return new TrivialParticleUpperBound(this);
	} else if (name == "APPROX") {
		return new LocalApproxScenarioUpperBound(this);
	} else if (name == "SMART" || name == "DEFAULT") {
		return new LocalnavigationParticleUpperBound(this);
	} else {
		cerr << "Unsupported base upper bound: " << name << endl;
		exit(1);
		return NULL;

}
}

ParticleLowerBound* Navigation::CreateParticleLowerBound(string name) const {
	cout<<"CreateParticleLowerBound"<<endl;
	if (name == "TRIVIAL") {
		return new TrivialParticleLowerBound(this);
	} else if (name == "LEGAL" || name == "DEFAULT") {
		return new LocalLegalParticleLowerBound(this);
	} else {
		cerr << "Unsupported base lower bound: " << name << endl;
		exit(1);
		return NULL;
	}
}

ScenarioLowerBound* Navigation::CreateScenarioLowerBound(string name,
	string particle_bound_name) const {
	const DSPOMDP* model = this;

	cout<<"CreateNaviSenarioLowerBound"<<endl;

	if (name == "TRIVIAL") {
		return new TrivialParticleLowerBound(this);
	} else if (name == "LEGAL") {
		return new LocalLegalParticleLowerBound(this);
	} else if (name == "SMART" || name == "DEFAULT") {
		return new LocalnaviSmartPolicy(this,
			CreateParticleLowerBound(particle_bound_name));
	} else if (name == "RANDOM") {
		return new RandomPolicy(this,
			CreateParticleLowerBound(particle_bound_name));
	} else {
		cerr << "Unsupported lower bound algorithm: " << name << endl;
		exit(0);
		return NULL;
	}
}

POMCPPrior* Navigation::CreatePOMCPPrior(string name) const {
	if (name == "UNIFORM") {
		return new UniformPOMCPPrior(this);
	} else if (name == "DEFAULT" || name == "SMART") {
		return new LocalNaviPOMCPPrior(this);
	} else {
		cerr << "Unsupported POMCP prior: " << name << endl;
		exit(1);
		return NULL;
	}
}

void Navigation::PrintState(const State& state, ostream& ostr) const {
	char buffer[20];

const NavigationState& navstate = static_cast<const NavigationState&>(state);
	ostr << endl;
	for (int x = 0; x < localmap.xsize() + 2; x++)
		ostr << "X ";
	ostr << endl;
	for (int y = localmap.ysize() - 1; y >= 0; y--) {
		ostr << "X";

		for (int x = 0; x < localmap.xsize(); x++) {
			Coord pos(x, y);
			int index = localmap.Index(pos);
			char c = ' ';
			for (int g = 0; g < num_peds_; g++)
				if (pos == navstate.ped_pos[g])
					c = 'A' + g ;
					// c = (pos == navstate.robot_pos ? '@' :'A' + g );
			if (pos == navstate.robot_pos)
				c = '*';
			if (pos == navstate.goal_pos)
				c = 'G';

			ostr << c << ' ';
		}
		
		ostr << "X" << endl;
	}
	for (int x = 0; x < localmap.xsize() + 2; x++)
		ostr << "X ";
	ostr << endl;

   for (int j=0 ; j < num_peds_; j++)
   	cout<<navstate.ped_pos[j].x<<", "<<navstate.ped_pos[j].y<<endl;

   cout<<"G :"<<navstate.goal_pos.x<<","<<navstate.goal_pos.y<<endl;

	//mCountRound();
}

void Navigation::PrintBelief(const Belief& belief, ostream& out) const {

	Grid<int> counts(localmap.xsize(), localmap.ysize());
	counts.SetAllValues(0);
	vector<State*> particles = static_cast<const ParticleBelief&>(belief).particles();
	for (int i = 0; i < particles.size(); i++) {
		const NavigationState* navstate = static_cast<const NavigationState*>(particles[i]);

		for (int g = 0; g < num_peds_; g++)
			counts(navstate->ped_pos[g])++;
	}

	for (int y = localmap.ysize() - 1; y >= 0; y --) {
		for (int x = 0; x < localmap.xsize(); x++) {
			out.width(6);
			out.precision(2);
			out << fixed << (double) counts(x, y) / particles.size();
		}
		out << endl;
	}

}

void Navigation::PrintObs(const State& state, OBS_TYPE observation,
	ostream& ostr) const {

const NavigationState& navstate = static_cast<const NavigationState&>(state);
	Grid<char> obs(localmap.xsize(), localmap.ysize());
	obs.SetAllValues(' ');

	// Robot
	for (int d = 0; d < 4; d++) {
		// See ped
		if (CheckFlag(observation, d)) {
			Coord eyepos = navstate.robot_pos + Compass::DIRECTIONS[d];
			while (localmap.Inside(eyepos)) {
				eyepos += Compass::DIRECTIONS[d];
			}
		}

		// Feel wall
		if (!CheckFlag(observation, d + 4)
			&& localmap.Inside(navstate.robot_pos + Compass::DIRECTIONS[d]))
			obs(navstate.robot_pos + Compass::DIRECTIONS[d]) = 'X';
	}

	if (CheckFlag(observation, 8)) {
		Coord smellPos;
		for (smellPos.x = -smell_range_; smellPos.x <= smell_range_; smellPos.x++)
			for (smellPos.y = -smell_range_; smellPos.y <= smell_range_;
				smellPos.y++)
				if (localmap.Inside(navstate.robot_pos + smellPos)
					&& obs(navstate.robot_pos + smellPos) == ' ')
					obs(navstate.robot_pos + smellPos) = '.';
	}

	ostr << endl;
	for (int x = 0; x < localmap.xsize() + 2; x++)
		ostr << "# ";
	ostr << endl;
	for (int y = localmap.ysize() - 1; y >= 0; y--) {
		ostr << "# ";
		for (int x = 0; x < localmap.xsize(); x++)
			ostr << obs(x, y) << ' ';
		ostr << "#" << endl;
	}
	for (int x = 0; x < localmap.xsize() + 2; x++)
		ostr << "# ";
	ostr << endl;
}

void Navigation::PrintAction(int action, ostream& out) const {
	out << Compass::CompassString[action] << endl;

	std_msgs::Int32 action_msg;
	
	if(ReachGoal)
		action_msg.data=5;
	else
		action_msg.data = action;

	//publish action to ros
	// Action_Pub.publish(action_msg);
	// std_msgs::Float32MultiArray costmap_msg;
	// costmap_msg.data = costmapvec;
	// Costmap_Pub.publish(costmap_msg);
	
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
	// cerr << "Default action = " << type << endl;
	// if (type == "MDP") {
	// 	ComputeOptimalPolicyUsingVI();
	// 	int num_states = NumStates();
	// 	default_action_.resize(num_states);

	// 	double value = 0;
	// 	for (int s = 0; s < num_states; s++) {
	// 		default_action_[s] = policy_[s].action;
	// 		value += policy_[s].value;
	// 	}
	// 	cerr << "MDP upper bound " << policy_[0].value << endl;
	// } else {
	// 	cerr << "Unsupported default action type " << type << endl;
	// 	exit(0);
	// }
}

int Navigation::GetAction(const State& state) const {
	return default_action_[GetIndex(&state)];
}

double Navigation::Reward(int s, int action) const {
	return 1;
}

void Navigation::MovePed(NavigationState& navstate, int g, Random &random) const {
	//if (Coord::ManhattanDistance(navstate.robot_pos, navstate.ped_pos[g])< pedes_range_) {
	// 	if (navstate.power_steps > 0)
	// 		MovePedDefensive(navstate, g, random);
	// 	else
	// 		MovePedAggressive(navstate, g, random);
	// } else {
	// 		MovePedRandom(navstate, g, random);
	// }
	if(g<num_peds_-1)
		MovePedRandom(navstate, g, random);
	else
		MovePedPath(navstate,g,random);
	//MovePedRandom(navstate, g, random);
}

void Navigation::MovePedAggressive(NavigationState& navstate, int g, Random &random) const {
	if (random.NextDouble() > chase_prob_) {
		MovePedRandom(navstate, g, random);
		return;
	}

	int bestDist = localmap.xsize() + localmap.ysize();
	Coord bestPos = navstate.ped_pos[g];
	int bestDir = -1;
	for (int dir = 0; dir < 4; dir++) {
		int dist = Coord::DirectionalDistance(navstate.robot_pos,
			navstate.ped_pos[g], dir);
		Coord newpos = NextPos(navstate.ped_pos[g], dir);
		if (dist <= bestDist && newpos.x >= 0 && newpos.y >= 0
			&& Compass::Opposite(dir) != navstate.ped_dir[g]) {
			bestDist = dist;
			bestPos = newpos;
		}
	}

	navstate.ped_pos[g] = bestPos;
	navstate.ped_dir[g] = bestDir;
}

void Navigation::MovePedDefensive(NavigationState& navstate, int g,
	Random &random) const {
	if (random.NextDouble() < defensive_slip_ && navstate.ped_dir[g] >= 0) {
		navstate.ped_dir[g] = -1;
		return;
	}

	int bestDist = 0;
	Coord bestPos = navstate.ped_pos[g];
	int bestDir = -1;
	for (int dir = 0; dir < 4; dir++) {
		int dist = Coord::DirectionalDistance(navstate.robot_pos,
			navstate.ped_pos[g], dir);
		Coord newpos = NextPos(navstate.ped_pos[g], dir);
		if (dist >= bestDist && newpos.x >= 0 && newpos.y >= 0
			&& Compass::Opposite(dir) != navstate.ped_dir[g]) {
			bestDist = dist;
			bestPos = newpos;
		}
	}

	navstate.ped_pos[g] = bestPos;
	navstate.ped_dir[g] = bestDir;
}

void Navigation::MovePedRandom(NavigationState& navstate, int g,
	Random &random) const {
	// Never switch to opposite direction
	// Currently assumes there are no dead-ends.
	Coord newpos;
	int dir;
	
	 //   dir = random.NextInt(4);
		// cout<<"dir :"<<dir<<endl;
		// newpos = NextPos(navstate.ped_pos[g], dir);

	do {
		dir = random.NextInt(4);
		//cout<<"dir :"<<dir<<endl;
		newpos = NextPos(navstate.ped_pos[g], dir);
	} while (!(newpos.x >= 0 && newpos.y >= 0 && newpos.x<localmap.xsize() && newpos.y<localmap.ysize()));

	navstate.ped_pos[g] = newpos;
	navstate.ped_dir[g] = dir;
}

void Navigation::MovePedPath(NavigationState& navstate, int g,
	Random &random) const {
	// Never switch to opposite direction
	// Currently assumes there are no dead-ends.
	Coord newpos;
	int dir=random.NextInt(4);
	int ww=roudndNUM;

	if(ww<13){
		newpos=ped_path1[ww];
		dir = ped_pathdir[ww];
	}
	else{
		newpos=ped_path1[dir];
		dir=random.NextInt(4);
	}
	
	navstate.ped_pos[g] = newpos;
	navstate.ped_dir[g] = dir;
}


Coord Navigation::NextPos(const Coord& from, int dir) const {
	Coord nextPos;
	nextPos = from + Compass::DIRECTIONS[dir];

	if (localmap.Inside(nextPos))
	{
		return nextPos;
	}
	else{
		return Coord(-1, -1);
	}
}

//Calcuate if pedestrian can be seen by robot
int Navigation::SeePed(const NavigationState& navstate, int action) const {
	Coord eyepos = navstate.robot_pos + Compass::DIRECTIONS[action];
	//we assumed that robot can see if ped in local map;
	//Todo : I have to change I already know the position in local map
	// If localmap.Inside(pedpos) return g;

	while (localmap.Inside(eyepos)) {
		for (int g = 0; g < num_peds_; g++)
			if (navstate.ped_pos[g] == eyepos)
				return g;
		eyepos += Compass::DIRECTIONS[action];
	}
	return -1;
}

bool Navigation::SmellGoal(const NavigationState& navstate) const {
	Coord smellPos;
	for (smellPos.x = -smell_range_; smellPos.x <= smell_range_; smellPos.x++)
		for (smellPos.y = -smell_range_; smellPos.y <= smell_range_; smellPos.y++)
			if (localmap.Inside(navstate.robot_pos + smellPos)
				&& navstate.Goal[localmap.Index(navstate.robot_pos + smellPos)])
				return true;
	return false;
}

//Make observation 
int Navigation::MakeObservations(const NavigationState& navstate) const {
	int observation = 0;
	for (int d = 0; d < 4; d++) {
		if (SeePed(navstate, d) >= 0)
			SetFlag(observation, d);
		Coord wpos = NextPos(navstate.robot_pos, d);
		if (wpos.x >= 0 && wpos.y >= 0)
			SetFlag(observation, d + 4);
	}

	if (SmellGoal(navstate))
	SetFlag(observation, 8);


	return observation;
}

bool Navigation::LocalMove(State& state, const History& history, int obs) const {
	NavigationState& navstate = static_cast<NavigationState&>(state);
	int numPeds = Random::RANDOM.NextInt(1, 3); // Change 1 or 2 peds at a time
	for (int i = 0; i < numPeds; ++i) {
		int g = Random::RANDOM.NextInt(num_peds_);
		navstate.ped_pos[g] = Coord(Random::RANDOM.NextInt(localmap.xsize()),
			Random::RANDOM.NextInt(localmap.ysize()));
		if (navstate.ped_pos[g] == navstate.robot_pos)
			return false;
	}

	// Just check the last time-step, don't check for full consistency
	if (history.Size() == 0)
		return true;
	int observation = MakeObservations(navstate);
	return history.LastObservation() == observation;
}


} // namespace despot