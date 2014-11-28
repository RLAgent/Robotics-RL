#include "ros/ros.h"
#include <string> //std::string
#include <sstream> //std::ostringstream
#include <stdlib.h> // rand
#include <map> // std::map
#include <limits> // quiet_NaN(), std::numeric_limits
#include <climits> //INT_MAX
#include <algorithm> // std::advance
#include <fstream> // ofstream

#include "std_msgs/Float32.h"
#include "lilee_rl_discrete/AgentEnvironment.h"
#include "lilee_rl_discrete/GoalLocation.h"
#include "lilee_rl_discrete/actions.h"
#include "lilee_rl_discrete/TeleopCmd.h"

ros::ServiceClient reinforcement_client_; // Service for reinforcement learning
ros::Subscriber teleopcmdsub_;

std::map<std::string, float> qValues;
double EPSILON_ = 0;//20; // prob. (in %) of choosing random action
double DELTA_ = 0;//40; // prob. (in %) of following user's policy
double ALPHA_ = 0.1; // learning rate
double DISCOUNT_ = 0.9; // discount rate


std::string qValue_filename = "saved_qValue_learningrate%f";
std::string qlearning_plot_filename = "qlearning_plot_learningrate%f";
int PRINT_globalCnt_ = 1000; // Print qValues to file after PRINT_globalCnt_ updates
int PARAM_globalCnt_ = 2000; // Update q-learning parameters after PARAM_globalCnt_ updates
int globalCnt, episodicCnt, episodeNum;

double cumulReward;

std::map<std::string, std::map<int, int> > userPolicy;
std::string userPolicy_filename = "saved_userPolicy";
bool userControl_, teleopcmdFlag_;
int userActionCmd_;

lilee_rl_discrete::AgentEnvironment srv;
lilee_rl_discrete::State state_;


/*********************************************************
 * Save and load saved policies
 *********************************************************/
 std::string getPlotFilename()
{
    char buffer [100];
    sprintf(buffer, qlearning_plot_filename.c_str(), ALPHA_);
    return buffer;
}
 std::string getQValueFilename()
{
    char buffer [100];
    sprintf(buffer, qValue_filename.c_str(), ALPHA_);
    return buffer;
}
void saveQValueToFile()
{
    ROS_ERROR("agent: Saving QValue to file");
    std::string filename = getQValueFilename();
    FILE *fp = fopen(filename.c_str(), "w");
    for (std::map<std::string, float>::iterator it=qValues.begin(); it != qValues.end(); ++it)
    {
        fprintf(fp, "%f\t%s\n", it->second, it->first.c_str());
    }
    fclose(fp);
}
int loadQValueFromFile()
{
    std::string filename = getQValueFilename();
    FILE *fp = fopen(filename.c_str(), "r");
    
    float newVal; char line[4096]; char key[4096];
    
    if (!fp)
    {
        ROS_ERROR("Could not open file %s for reading", filename.c_str());
        return -1;
    }
    
    if (fp != NULL)
    {
        while (fgets(line, sizeof(line), fp) != NULL)
        {
            sscanf(line, "%f %[^\t\n]", &newVal, key);
            
            std::map<std::string, float>::iterator it = qValues.find(key);
            if (it != qValues.end())
                qValues.erase(key);
            qValues.insert(std::pair<std::string, float>(key, newVal));
        }
    }
    fclose(fp);
    return 0;
}
 void saveRewardHeadingToFile()
{
    std::string filename = getPlotFilename();
    std::ofstream logging;
    logging.open(filename.c_str(), std::ios_base::app);
    logging << "learningRate=" << ALPHA_ << ",  EPSILON_=" << EPSILON_ << ",  DELTA_=" << DELTA_ << '\n';
    logging.close();
}
void saveRewardToFile(int episode, int steps, int reward, int success)
{
    std::string filename = getPlotFilename();
    std::ofstream logging;
    logging.open(filename.c_str(), std::ios_base::app);
    logging
        << episode
        << "\t"
        << steps
        << "\t"
        << reward
        << "\t"
        << success
        << '\n';
    logging.close();
    
    ROS_ERROR("agent: saved %s", filename.c_str());
}

/*********************************************************
 * Choosing action
 *********************************************************/
std::string stateToString(lilee_rl_discrete::State state)
{
    std::ostringstream ss;
    ss << "("
        << state.distance_to_goal
        << ","
        << state.theta_to_goal
        << ","
        << (bool)state.isTerminal
        << ")";
    return ss.str();
}
std::string stateActionToKey(lilee_rl_discrete::State state, int action)
{   // Convert (state,action) into a string key.

    std::ostringstream ss;
    ss  << stateToString(state)
        << action;
    return ss.str();
}

float getQValue(lilee_rl_discrete::State state, int action)
{   // Returns Q(state, action).
    
    if (state.isTerminal)
        return 0.0;
    
    std::string key = stateActionToKey(state, action);
    std::map<std::string, float>::iterator it = qValues.find(key);
    if (it != qValues.end())
    {
        return it->second;
    }
    else
    {
        /*float newVal = (rand() % 100)*-.01;
        qValues.insert(std::pair<std::string, float>(key, newVal));
        return newVal;*/
        qValues.insert(std::pair<std::string, float>(key, 0.0));
        return 0.0;
    }
        
}

float getValue(lilee_rl_discrete::State state)
{   // Returns max_action Q(state, action)
    // where the max is over all legal actions.
    
    
    float q, max = -std::numeric_limits<float>::max();
    for (int action = 0; action < actions::NUM_ACTIONS_; action++)
    {
        q=getQValue(state, action);
        if (q > max)
            max = q;
    }
    return max;
}


void update()
{   // Q-Value update

    if (srv.request.action == actions::STOP_)
    {
        state_ = srv.response.state;
        return;
    }
    
    lilee_rl_discrete::State nextState = srv.response.state;
    float nextReward = srv.response.reward;
    int action = srv.request.action;
    
    float newVal = (1-ALPHA_)*getQValue(state_, action) + ALPHA_*(nextReward + DISCOUNT_*getValue(nextState));
    std::string key = stateActionToKey(state_, action);
    std::map<std::string, float>::iterator it = qValues.find(key);
    if (it != qValues.end())
        qValues.erase(key);
    qValues.insert(std::pair<std::string, float>(key, newVal));
    
    state_ = nextState;
    cumulReward += nextReward;
    
    globalCnt++; episodicCnt++;
    
    if (globalCnt % PRINT_globalCnt_ == 0)
    {   // save Q-Value table
        saveQValueToFile();
    }
    if (globalCnt % PARAM_globalCnt_ == 0)
    {   // update parameters
        DELTA_ *= 0.99;
        EPSILON_ *= 0.99;
        //ALPHA_ *= 0.9999;

        ROS_ERROR("\nagent: Decreased DELTA_=%f,  EPSILON_=%f", DELTA_, EPSILON_);
        saveRewardHeadingToFile();

        //ROS_ERROR("\nagent: Decreased DELTA_=%f,  EPSILON_=%f", DELTA_, EPSILON_);
        //saveRewardHeadingToFile();
    }
    
    // PLOT
    if (srv.response.state.isTerminal)
    {   // Save cumulReward and episodicCnt to file after each episode
        
        saveRewardToFile(episodeNum, episodicCnt, cumulReward, (int)srv.response.state.success);
        ROS_ERROR("agent: End of episode. Saved cumulReward and episodicCnt to file.");
        
        episodicCnt = 0;
        cumulReward = 0;
        episodeNum++;
        
    }
    
}


int getRandomAction()
{   // Choose a random action.
    int action = rand() % actions::NUM_ACTIONS_;
    //ROS_ERROR("\nagent, getRandomAction(): Chose %d\n", action);
    return action;
}

int getPolicy(lilee_rl_discrete::State state)
{   // Compute the best action to take in a state.

    std::string key;
    float max = -std::numeric_limits<float>::max(), q;
    std::set<int> actions;
    
    for (int action = 0; action < actions::NUM_ACTIONS_; action++)
    {
        
        q = getQValue(state, action);
        if (q > max)
        {
            max = q;
            actions.clear();
            actions.insert(action);
        }
        else if (q == max)
        {
            actions.insert(action);
        }
        
    }
    
    if (!actions.empty())
    {
        double random = rand() % actions.size();
        std::set<int>::iterator it = actions.begin();
        for (; random != 0; random--) it++;
        
        // DEBUG
        //ROS_ERROR("\nagent, getPolicy():     Chose q(%d)=%f\n", *it, max);
      
        return *it;
    }
    else
    {
        return getRandomAction();
    }
}

lilee_rl_discrete::State getNearestState(lilee_rl_discrete::State state)
{   // Return the nearest state in userPolicy.

    int minDist = INT_MAX, tempDist, state_d, state_t;
    int temp_d, temp_t, temp_y, best_d, best_t;
    std::string nearestStateString;
    lilee_rl_discrete::State nearestState;
    
    sscanf(stateToString(state).c_str(), "(%d,%d)", &state_d, &state_t);
    
    // Compare state with all states in userPolicy
    for (std::map<std::string, std::map<int, int> >::iterator it=userPolicy.begin(); it != userPolicy.end(); ++it)
    {
        sscanf(it->first.c_str(), "(%d,%d)", &temp_d, &temp_t);
        tempDist = pow(temp_d-state_d,2)+pow(temp_t-state_t,2);
        if (tempDist < minDist)
        {
            minDist = tempDist;
            nearestStateString = it->first;
        }
    }
    
    // Store nearest state in nearestState
    sscanf(nearestStateString.c_str(), "(%d,%d)", &best_d, &best_t);
    nearestState.distance_to_goal = best_d;
    nearestState.theta_to_goal = best_t;
    
    return nearestState;
}

int getUserPolicy(lilee_rl_discrete::State state)
{   // Return the next action based on the user's policy.

    lilee_rl_discrete::State nearestState = getNearestState(state);
    
    std::map<int, int> actionCounts = userPolicy[stateToString(nearestState)];
    int totCnt = 0;
    double maxProb = 0, prob;
    std::set<int> actions;
    
    for (std::map<int, int>::iterator it = actionCounts.begin(); it != actionCounts.end(); ++it)
    {
        totCnt += it->second;
    }
    for (std::map<int, int>::iterator it = actionCounts.begin(); it != actionCounts.end(); ++it)
    {
        prob = (double)(it->second)/totCnt;
        if (prob > maxProb)
        {
            maxProb = prob;
            actions.clear();
            actions.insert(it->first);
        }
        else if (prob == maxProb)
        {
            actions.insert(it->first);
        }
        
    }
    
    if (!actions.empty())
    {
        double random = rand() % actions.size();
        std::set<int>::iterator it = actions.begin();
        for (; random != 0; random--) it++;
        
        // DEBUG
        //ROS_ERROR("\nagent, getUserPolicy(): Chose p(%d)=%f\n", *it, maxProb);
        return *it;
    }
    else
    {
        return getRandomAction();
    }
}

int getAction()
{   // Compute the action to take in the current state.
    // With probability epsilon, we take a random action;
    // with probability delta, we follow the user's policy;
    // otherwise, we take the best policy action.
    
    ros::spinOnce();
    
    if (!srv.response.state.isTerminal)
    {
        if (!userControl_)
        {
            int p = rand() % 100;
            
            if (p < EPSILON_)
                return getRandomAction();
            else if (p >= EPSILON_ && p < EPSILON_+DELTA_)
                return getUserPolicy(state_);
            else
                return getPolicy(state_);
        }
        else
        {
            // wait for next teleop command
            while (!teleopcmdFlag_) { ros::spinOnce(); }
            teleopcmdFlag_ = false;
            
            userPolicy[stateToString(state_)][userActionCmd_]++;
            return userActionCmd_;
        }
    }
    else
    {
        if (userControl_)
        {
            while (!teleopcmdFlag_) { ros::spinOnce(); }
            teleopcmdFlag_ = false;
        }
        return actions::STOP_;
    }
}


/*********************************************************
 * User Control
 *********************************************************/
void saveUserPolicyToFile()
{
    ROS_ERROR("agent: Saving UserPolicy to file");
    FILE *fp = fopen(userPolicy_filename.c_str(), "w");
    
    for (std::map<std::string, std::map<int, int> >::iterator it=userPolicy.begin(); it != userPolicy.end(); ++it)
    {
        fprintf(fp, "%s\n", it->first.c_str());
        
        for (std::map<int, int>::iterator it2=it->second.begin(); it2 != it->second.end(); ++it2)
        {
            fprintf(fp, "%d\t%d\n", it2->first, it2->second);
        }
    }
    fclose(fp);
}

int loadUserPolicyFromFile()
{
    FILE *fp = fopen(userPolicy_filename.c_str(), "r");
    
    int action, count; char line[4096]; char key[4096];
    
    if (!fp)
    {
        ROS_ERROR("Could not open file %s for reading", userPolicy_filename.c_str());
        return -1;
    }
    
    if (fp != NULL)
    {
        while (fgets(line, sizeof(line), fp) != NULL)
        {
            if (line[0] == '(')
            {
                sscanf(line, "%s", key);
                std::map<std::string, std::map<int, int> >::iterator it = userPolicy.find(key);
                if (it != userPolicy.end())
                    userPolicy.erase(key);
            }
            else
            {
                sscanf(line, "%d %d", &action, &count);
                userPolicy[key].insert(std::pair<int, int>(action, count));
            }
        }
    }
    fclose(fp);
    return 0;
}

void teleopcmdcb(const lilee_rl_discrete::TeleopCmd::ConstPtr& msg)
{
    if (!teleopcmdFlag_)
    {
        teleopcmdFlag_ = true;
        if (userControl_ != msg->user_control)
        {
            if (msg->user_control)
            {
                ROS_ERROR("agent: User control is ON");
                loadUserPolicyFromFile();
            }
            else
            {
                ROS_ERROR("agent: User control is OFF");
                saveUserPolicyToFile();
            }
        }
        userControl_ = msg->user_control;
        userActionCmd_ = msg->action;
    }
}

/*********************************************************
 * Testing
 *********************************************************/
void printSrv()
{   // Print srv request and response (for debugging)
    ROS_ERROR(
        "\nagent: prevState: (%d,%d,%d)\n          action: %d\n           state: (%d,%d,%d)\n          reward: %f\n",
        
        (int)state_.distance_to_goal,
        (int)state_.theta_to_goal,
        (int)state_.isTerminal,
        
        srv.request.action,
        
        (int)srv.response.state.distance_to_goal,
        (int)srv.response.state.theta_to_goal,
        (int)srv.response.state.isTerminal,
        
        srv.response.reward
        );
}

/*********************************************************
 * main(): Initialize
 *********************************************************/
 

int main(int argc, char **argv)
{
    ROS_ERROR("Hello from agent");
    
	ros::init(argc, argv, "agent");
	ros::NodeHandle n;
	srand (time(NULL));
	
	// PLOT
    saveRewardHeadingToFile();
	
	// Initialize
	lilee_rl_discrete::GoalLocation location;
	location.position_x = location.position_z = std::numeric_limits<float>::quiet_NaN();
	srv.response.state.goal = location;
	srv.response.state.bumper.bumper = 1; srv.response.state.bumper.state = 0;
	srv.response.reward = 0.0;
	srv.request.action = actions::STOP_;
	globalCnt = episodicCnt = 0;
	teleopcmdFlag_ = false;
	userControl_ = true;
	userActionCmd_ = actions::STOP_;
	cumulReward = 0.0;
	
	// Load knowledge base from file
	loadQValueFromFile();
	loadUserPolicyFromFile();
	
	reinforcement_client_ = n.serviceClient<lilee_rl_discrete::AgentEnvironment>("agent_environment");
    teleopcmdsub_ = n.subscribe("teleop_cmd", 1, teleopcmdcb);
	
	// Wait until environment and state are ready
	ros::Duration(3).sleep();
	
	ros::Rate r(10);
	while (ros::ok())
	{
	    
	    ros::spinOnce();
	    r.sleep();
	    
	    if (reinforcement_client_.call(srv))
	    {
	        printSrv();
	        update();
	        srv.request.action = getAction();
	    }
	    else
	    {
	        //ROS_ERROR("agent: Call to environment failed!");
	    }
	    
	    //ros::Duration(1).sleep();
	}
	
	
	return 0;
}
