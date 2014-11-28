#include "ros/ros.h"
#include <string> //std::string
#include <sstream> //std::ostringstream
#include <stdlib.h> // rand
#include <map> // std::map
#include <limits> // quiet_NaN(), std::numeric_limits
#include <climits> //INT_MAX
#include <algorithm> // std::advance
#include <queue> // std::queue
#include <math.h> // fabs, ceil
#include <fstream> // ofstream

#include "geometry_msgs/Transform.h"
#include "std_msgs/Float32.h"

#include "lilee_rl_continuous/AgentEnvironment.h"
#include "lilee_rl_continuous/GoalLocation.h"
#include "lilee_rl_continuous/actions.h"
#include "lilee_rl_continuous/TeleopCmd.h"
#include "lilee_rl_continuous/WorldControl.h"

ros::ServiceClient reinforcement_client_, worldcontrolclient_;
ros::Subscriber teleopcmdsub_, pgoutputsub_;
ros::Publisher pginputpub_, pginitpolicypub_;

// Saving trajectories
lilee_rl_continuous::AgentEnvironment srv;
lilee_rl_continuous::State state_;

std::map<std::string, float> qValues;
double EPSILON_ = 1; // prob. (in %) of choosing random action
double DELTA_ = 90; // prob. (in %) of following user's policy

float MIN_LINEAR_VEL_ = -1.0;
float MAX_LINEAR_VEL_ = 1.5;
float MIN_ANGULAR_VEL_=-1.5;
float MAX_ANGULAR_VEL_= 1.5;

std::string policy_filename = "lilee_rl_continuous_POLICY";
std::string policy1_filename = "lilee_rl_continuous_POLICY_world%d-policy%d-linear";
std::string policy2_filename = "lilee_rl_continuous_POLICY_world%d-policy%d-angular";
std::string plot_filename = "lilee_rl_continuous_PLOT_world%d-policy(%d,%d)";

int PARAM_CNT_ = 1000; // Update parameters after PARAM_CNT_ steps
int globalCnt;   // Total number of steps taken over all episodes
int episodicCnt; // Number of steps taken in an episode
int episodeNum;  // Episode number
int numFailures = 0; // Number of failures
double cumulReward; // Cumulative reward in an episode

bool userControl_, teleopcmdFlag_, pgoutputFlag_;
float userLinearVelCmd_, userAngularVelCmd_;

// Communication with MATLAB/PG
geometry_msgs::Transform curStateActionReward_;

// Policy for linear_vel and angular_vel
float k1[2], k2[2];   // thetas
float SIGMA1_, SIGMA2_; // sigma
float CONST_SIGMA_ = 0.1;

// Which policy the agent should follow
// Set 0 if agent shouldn't change policy
int DEFAULT_LEARNINGRATE1_ = 3;
int DEFAULT_LEARNINGRATE2_ = 2;

// if isTest==true, turns off learning and tests learning rates (TEST_LEARNINGRATE1_, TEST_LEARNINGRATE2_)
bool isTest = true;  
int TEST_LEARNINGRATE1_ = 3;// = 1;
int TEST_LEARNINGRATE2_ = 2;// = 1;
int NUM_LEARNINGRATES_ = 4;
int TRAIN_NUM_EPISODES = 20000;
int TEST_NUM_EPISODES_ = 25;
int TEST_MAX_NUM_FAILURES_ = ceil(TEST_NUM_EPISODES_/5.0); // Max number of failures

// Initial policy values
float INIT_K1[] = {0.3, -0.05};
float INIT_K2[] = {0.00001, 0.5};
int STATE_DIM_ = 2; // dimension of state space

// world_control
lilee_rl_continuous::WorldControl world_;
int DEFAULT_WORLD_ = 1;


int TRAIN_NUM_EPISODES_RANDOM_ = 500; // For training random policies

/*********************************************************
 * Generating random numbers
 *********************************************************/
float fRand(float fMin, float fMax)
{   // Returns a float between fMin and fMax uniformly at random.
    float f = (float)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}

float gaussian(float m, float s)
{	// Samples from a gaussian distribution with mean m and sd s.
	float x1, x2, w, y1;
	static float y2;
	static int use_last = 0;

	if (use_last)
	{
		y1 = y2;
		use_last = 0;
	}
	else
	{
		do {
			x1 = 2.0 * fRand(0,1) - 1.0;
			x2 = 2.0 * fRand(0,1) - 1.0;
			w = x1 * x1 + x2 * x2;
		} while ( w >= 1.0 );

		w = sqrt( (-2.0 * log( w ) ) / w );
		y1 = x1 * w;
		y2 = x2 * w;
		use_last = 1;
	}
	return( m + y1 * s );
}
/*********************************************************
 * toString methods
 *********************************************************/
std::string stateToString(lilee_rl_continuous::State state)
{
    std::ostringstream ss;
    ss << "("
        << state.distance_to_goal
        << ","
        << state.theta_to_goal
        << ","
        << state.agent_location.orientation_yaw
        << ","
        << (bool)state.isTerminal
        << ")";
    return ss.str();
}
std::string stateActionToString(lilee_rl_continuous::State state, int action)
{   // Convert (state,action) into a string key.

    std::ostringstream ss;
    ss  << stateToString(state)
        << action;
    return ss.str();
}
std::string getPolicyFilename(int numWorld, int numPolicy, bool isLinearVel)
{
    char buffer [100];
    if (isLinearVel)
    {
        sprintf(buffer, policy1_filename.c_str(), numWorld, numPolicy);
    }
    else
    {
        sprintf(buffer, policy2_filename.c_str(), numWorld, numPolicy);
    }
    return buffer;
}
std::string getPlotFilename(int numWorld, int numPolicy_linear, int numPolicy_angular)
{
    char buffer [100];
    sprintf(buffer, plot_filename.c_str(), numWorld, numPolicy_linear, numPolicy_angular);
    return buffer;
}
/*********************************************************
 * Print functions (for debugging)
 *********************************************************/
void printPolicy()
{
    ROS_ERROR(
        "\nagent: \nk1=[%f,%f]\nSIGMA1_=%f\nk2=[%f,%f]\nSIGMA2_=%f\n\n",
        k1[0],
        k1[1],
        SIGMA1_,
        k2[0],
        k2[1],
        SIGMA2_
    );
}

void printSrv()
{   // Print srv request and response (for debugging)
    ROS_ERROR(
        "\nagent: prevState: %s\n          action: (%f,%f)\n           state: %s\n          reward: %f\n",
        
        stateToString(state_).c_str(),
        
        srv.request.action.linear_vel,
        srv.request.action.angular_vel,
        
        stateToString(srv.response.state).c_str(),
        
        srv.response.reward
        );
}

/*********************************************************
 * Save and load saved policies
 *********************************************************/
void setPolicyToDefault()
{
    for (int i = 0; i < STATE_DIM_; i++)
	{
	    k1[i] = INIT_K1[i];
	    k2[i] = INIT_K2[i];
	}
	SIGMA1_ = CONST_SIGMA_;
	SIGMA2_ = CONST_SIGMA_;
}
void setPolicyToRandom()
{
    for (int i = 0; i < STATE_DIM_; i++)
	{
	    k1[i] = fRand(-1.5, 1.5);
	    k2[i] = fRand(-1.5, 1.5);
	}
	SIGMA1_ = CONST_SIGMA_;
	SIGMA2_ = CONST_SIGMA_;
	
	ROS_ERROR("Random policy (%f,%f,%f) and (%f,%f,%f)\n", k1[0], k1[1], SIGMA1_, k2[0], k2[1], SIGMA2_);
	
	// Save to file
	std::string filename = getPlotFilename(DEFAULT_WORLD_, DEFAULT_LEARNINGRATE1_, DEFAULT_LEARNINGRATE2_);
	std::ofstream logging;
    logging.open(filename.c_str(), std::ios_base::app);
    logging
        << "Random policy "
        << k1[0]
        << "\t"
        << k1[1]
        << "\t"
        << SIGMA1_
        << "\t"
        << k2[0]
        << "\t"
        << k2[1]
        << "\t"
        << SIGMA2_
        << '\n';
    logging.close();
}


void savePolicyToFile(int numPolicy, bool isLinearVel, float k_0, float k_1, float sigma)
{
    std::string filename = getPolicyFilename(DEFAULT_WORLD_, numPolicy, isLinearVel);
    ROS_ERROR("agent: Saving policy to file %s", filename.c_str());
    
    FILE *fp = fopen(filename.c_str(), "w");
    fprintf(fp, "%f\t%f\t%f",
        k_0,
        k_1,
        sigma);
    fflush(fp);
    fclose(fp);
}

void savePolicyToFile()
{
    ROS_ERROR("agent: Saving policy to file %s", policy_filename.c_str());
    
    FILE *fp = fopen(policy_filename.c_str(), "w");
    fprintf(fp, "%f\t%f\t%f\t%f\t%f\t%f",
        k1[0],
        k1[1],
        SIGMA1_,
        k2[0],
        k2[1],
        SIGMA2_);
    fflush(fp);
    fclose(fp);
}

void saveRewardHeadingToFile()
{
    std::string filename;
    if (isTest)
        filename = getPlotFilename(DEFAULT_WORLD_, TEST_LEARNINGRATE1_, TEST_LEARNINGRATE2_);
    else
        filename = getPlotFilename(DEFAULT_WORLD_, DEFAULT_LEARNINGRATE1_, DEFAULT_LEARNINGRATE2_);
        
    std::ofstream logging;
    logging.open(filename.c_str(), std::ios_base::app);
    logging << "episode\tsteps\treward\tsuccess\n";
    logging.close();
}
 
void saveRewardToFile(int episode, int steps, int reward, int success)
{
    std::string filename;
    if (isTest)
	    filename = getPlotFilename(DEFAULT_WORLD_, TEST_LEARNINGRATE1_, TEST_LEARNINGRATE2_);
    else
        filename = getPlotFilename(DEFAULT_WORLD_, DEFAULT_LEARNINGRATE1_, DEFAULT_LEARNINGRATE2_);
        
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

int loadPolicyFromFile(int numPolicy, bool isLinearVel)
{
    std::string filename = getPolicyFilename(DEFAULT_WORLD_, numPolicy, isLinearVel);
    FILE *fp = fopen(filename.c_str(), "r");
    char line[4096];
    
    ROS_ERROR("Loading policy %s", filename.c_str());
    if (!fp)
    {
        ROS_ERROR("Could not open file %s for reading", filename.c_str());
        return -1;
    }
    
    if (fp != NULL)
    {   
        if (isLinearVel)
        {
            if (fgets(line, sizeof(line), fp) != NULL)
            {
                sscanf(line,
                        "%f %f %f",
                        &k1[0],
                        &k1[1],
                        &SIGMA1_
                        );
                
                if (isnan(k1[0]) || isnan(k1[1]) || isnan(SIGMA1_))
                {
                    ROS_ERROR("agent_pg: Policy %d for linear is nan. Resetting policy to default.", numPolicy);
                    setPolicyToDefault();
                    return -1;
                }
            }
            else
            {
                ROS_ERROR("Failed to read file %s", filename.c_str());
                return -1;
            }
        }
        else
        {
            if (fgets(line, sizeof(line), fp) != NULL)
            {
                sscanf(line,
                        "%f %f %f",
                        &k2[0],
                        &k2[1],
                        &SIGMA2_
                        );
                if (isnan(k2[0]) || isnan(k2[1]) || isnan(SIGMA2_))
                {
                    ROS_ERROR("agent_pg: Policy %d for angular is nan. Resetting policy to default.", numPolicy);
                    setPolicyToDefault();
                    return -1;
                }
            }
            else
            {
                ROS_ERROR("Failed to read file %s", filename.c_str());
                return -1;
            }
        }
    }
    fclose(fp);

    return 0;
}


int loadPolicyFromFile()
{
    FILE *fp = fopen(policy_filename.c_str(), "r");
    char line[4096];
    
    if (!fp)
    {
        ROS_ERROR("Could not open file %s for reading", policy_filename.c_str());
        return -1;
    }
    
    if (fp != NULL)
    {   
        if (fgets(line, sizeof(line), fp) != NULL)
            sscanf(line,
                    "%f %f %f %f %f %f",
                    &k1[0],
                    &k1[1],
                    &SIGMA1_,
                    &k2[0],
                    &k2[1],
                    &SIGMA2_
                    );
        else
            ROS_ERROR("Failed to read file %s", policy_filename.c_str());
    }
    fclose(fp);

    return 0;
}

void sendInitPolicyToMATLAB()
{   // Send loaded policy to MATLAB
    
    curStateActionReward_.translation.y = DEFAULT_LEARNINGRATE2_;
    curStateActionReward_.translation.z = 1; // angular_vel
    curStateActionReward_.rotation.x = k2[0];
    curStateActionReward_.rotation.y = k2[1];
    curStateActionReward_.translation.x = SIGMA2_;
    pginitpolicypub_.publish(curStateActionReward_);
    
    curStateActionReward_.translation.y = DEFAULT_LEARNINGRATE1_;
    curStateActionReward_.translation.z = 0; // linear_vel
    curStateActionReward_.rotation.x = k1[0];
    curStateActionReward_.rotation.y = k1[1];
    curStateActionReward_.translation.x = SIGMA1_;
    pginitpolicypub_.publish(curStateActionReward_);
}

int sendInitPolicyToMATLAB(int numPolicy, bool isLinearVel)
{
    // Read policy from file
    std::string filename = getPolicyFilename(DEFAULT_WORLD_, numPolicy, isLinearVel);
    FILE *fp = fopen(filename.c_str(), "r");
    float k_0, k_1, sigma; char line[4096];
    if (!fp)
    {
        ROS_ERROR("Could not open file %s for reading", filename.c_str());
        return -1;
    }
   
    if (fp != NULL)
    {   
        if (fgets(line, sizeof(line), fp) != NULL)
                sscanf(line,
                        "%f %f %f",
                        &k_0,
                        &k_1,
                        &sigma
                        );
        else
        {
            ROS_ERROR("Failed to read file %s", filename.c_str());
            return -1;
        }
    }
    fclose(fp);

    // Send policy
    curStateActionReward_.translation.y = numPolicy;
    curStateActionReward_.rotation.x = k_0;
    curStateActionReward_.rotation.y = k_1;
    curStateActionReward_.translation.x = sigma;
    if (isLinearVel)
        curStateActionReward_.translation.z = 0; // linear_vel
    else
        curStateActionReward_.translation.z = 1; // angular_vel
        
    pginitpolicypub_.publish(curStateActionReward_);
    
    return 0;
}

/*********************************************************
 * Interaction with MATLAB/PG
 *********************************************************/
void update()
{

    if ((srv.request.action.linear_vel == 0) && (srv.request.action.angular_vel == 0))
    {   // Don't update anything if action=STOP
        state_ = srv.response.state;
        return; 
    }
    
    // Save trajectories
    lilee_rl_continuous::State nextState = srv.response.state;
    float nextReward = srv.response.reward;
    float linear_vel = srv.request.action.linear_vel;
    float angular_vel = srv.request.action.angular_vel;
    
    // Send message to MATLAB
    curStateActionReward_.translation.x = linear_vel;
    curStateActionReward_.translation.y = angular_vel;
    curStateActionReward_.translation.z = nextReward;
    curStateActionReward_.rotation.x = state_.distance_to_goal;
    curStateActionReward_.rotation.y = state_.theta_to_goal;
    curStateActionReward_.rotation.z = state_.agent_location.orientation_yaw;
    
    if (!srv.response.state.isTerminal)
        curStateActionReward_.rotation.w = state_.isTerminal;
    else
        curStateActionReward_.rotation.w = 1;
    pginputpub_.publish(curStateActionReward_);
    
    // Update variables
    globalCnt++; episodicCnt++;
    cumulReward += nextReward;
    state_ = nextState;
    
    if (globalCnt % PARAM_CNT_ == 0)
    {   // update parameters
        DELTA_ *= 0.99;
        EPSILON_ *= 0.99;
        
        ROS_ERROR("\nagent: Decreased DELTA_=%f,  EPSILON_=%f", DELTA_, EPSILON_);
    }
    
    // PLOT
    if (srv.response.state.isTerminal)
    {   // End of episode
        
        printPolicy();
        
        if (!isTest)
            savePolicyToFile();
            
        int success = (int)srv.response.state.success;
        
        
        // Save cumulReward and episodicCnt to file
        saveRewardToFile(episodeNum, episodicCnt, cumulReward, success);
        
        if (!isTest)
        {
        }
        else
        {
            if (!success)
                numFailures++;
        }
        
        // Reset counter
        episodicCnt = 0;
        cumulReward = 0;
        episodeNum++;
    }
}


/*********************************************************
 * Choosing action
 *********************************************************/
std::pair<float, float> getRandomAction()
{   // Choose a random action.

    float linear_vel = fRand(MIN_LINEAR_VEL_, MAX_LINEAR_VEL_);
    float angular_vel = fRand(MIN_ANGULAR_VEL_, MAX_ANGULAR_VEL_);

    return std::pair<float, float>(linear_vel, angular_vel);
}

std::pair<float, float> getPolicy(lilee_rl_continuous::State state, float k1_0, float k1_1, float sigma1, float k2_0, float k2_1, float sigma2)
{
    float mean1 = k1_0*state.distance_to_goal
            +  k1_1*fabs(state.theta_to_goal);
    float mean2 = k2_0*state.distance_to_goal
            +  k2_1*state.theta_to_goal;
    
    float linear_vel = gaussian(mean1,  CONST_SIGMA_);//sigma1);
    float angular_vel = gaussian(mean2, CONST_SIGMA_);//sigma2);
    
    if (linear_vel < MIN_LINEAR_VEL_)
        linear_vel = MIN_LINEAR_VEL_;
    else if (linear_vel > MAX_LINEAR_VEL_)
        linear_vel = MAX_LINEAR_VEL_;
    if (angular_vel < MIN_ANGULAR_VEL_)
        angular_vel = MIN_ANGULAR_VEL_;
    else if (angular_vel > MAX_ANGULAR_VEL_)
        angular_vel = MAX_ANGULAR_VEL_;
    
    return std::pair<float, float>(linear_vel, angular_vel);
}

std::pair<float, float> getPolicy(lilee_rl_continuous::State state)
{   // Compute the best action to take in a state.
    
    return getPolicy(state, k1[0], k1[1], CONST_SIGMA_, k2[0], k2[1], CONST_SIGMA_);
}

std::pair<float, float> getUserPolicy(lilee_rl_continuous::State state)
{   // Return the next action based on the user's policy.
    
    return getPolicy(state, INIT_K1[0], INIT_K1[1], CONST_SIGMA_, INIT_K2[0], INIT_K2[1], CONST_SIGMA_);
}

std::pair<float, float> getAction()
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
            return std::pair<float,float>(userLinearVelCmd_, userAngularVelCmd_);
        }
    }
    else
    {
        if (userControl_)
        {
            while (!teleopcmdFlag_) { ros::spinOnce(); }
            teleopcmdFlag_ = false;
        }
        return std::pair<float,float>(0,0);
    }
}



/*********************************************************
 * Callback functions
 *********************************************************/

void pgoutputcb(const geometry_msgs::Transform::ConstPtr& msg)
{
    pgoutputFlag_ = true;
    
    if (!isTest)
    {
        if (msg->translation.z == 0)
        {   // linear_vel
        
            if ((int)msg->translation.y == DEFAULT_LEARNINGRATE1_)
            {
                int numPolicy = msg->translation.y;
                float k1_0 = msg->rotation.x;
                float k1_1 = msg->rotation.y;
                float sigma1_ = msg->translation.x;
                savePolicyToFile(numPolicy, true, k1_0, k1_1, sigma1_);
                
                SIGMA1_=sigma1_;
                k1[0]=k1_0;
                k1[1]=k1_1;
            }
            else
            {
                int numPolicy = msg->translation.y;
                float k1_0 = msg->rotation.x;
                float k1_1 = msg->rotation.y;
                float sigma1_ = msg->translation.x;
                savePolicyToFile(numPolicy, true, k1_0, k1_1, sigma1_);
            }
        }
        else
        {   // angular_vel
            if ((int)msg->translation.y == DEFAULT_LEARNINGRATE2_)
            {
                int numPolicy = msg->translation.y;
                float k2_0 = msg->rotation.x;
                float k2_1 = msg->rotation.y;
                float sigma2_ = msg->translation.x;
                savePolicyToFile(numPolicy, false, k2_0, k2_1, sigma2_);
                
                SIGMA2_ = sigma2_;
                k2[0] = k2_0;
                k2[1] = k2_1;
            }
            else
            {
                int numPolicy = msg->translation.y;
                float k2_0 = msg->rotation.x;
                float k2_1 = msg->rotation.y;
                float sigma2_ = msg->translation.x;
                savePolicyToFile(numPolicy, false, k2_0, k2_1, sigma2_);
            }
        }
    }
}


void teleopcmdcb(const lilee_rl_continuous::TeleopCmd::ConstPtr& msg)
{
    if (!teleopcmdFlag_)
    {
        teleopcmdFlag_ = true;
        if (userControl_ != msg->user_control)
        {
            if (msg->user_control)
            {
                ROS_ERROR("agent: User control is ON");
            }
            else
            {
                ROS_ERROR("agent: User control is OFF");
            }
        }
        userControl_ = msg->user_control;
        userLinearVelCmd_ = msg->action.linear_vel;
        userAngularVelCmd_ = msg->action.angular_vel;
    }
}

/*********************************************************
 * Main
 *********************************************************/

 void startTraining()
 {  
    isTest=false;
    ROS_ERROR("agent_pg: Start training");
    
    // Set world
	world_.request.numWorld = DEFAULT_WORLD_;
	worldcontrolclient_.call(world_);
	
    // Load policy
    sendInitPolicyToMATLAB();
    for (int numPolicy = 1; numPolicy <= NUM_LEARNINGRATES_; numPolicy++)
    {
        sendInitPolicyToMATLAB(numPolicy, true);
        sendInitPolicyToMATLAB(numPolicy, false);
    }
    printPolicy();
    saveRewardHeadingToFile();
    
    ros::Rate r(10);
    while (ros::ok() && (episodeNum <= TRAIN_NUM_EPISODES))
    {
        ros::spinOnce();
        r.sleep();
        
        if (reinforcement_client_.call(srv))
        {
            //printSrv();
            update();
            std::pair<float,float> action = getAction();
            srv.request.action.linear_vel = action.first;
            srv.request.action.angular_vel = action.second;
        }
    }
 }
 
 void startTrainingRandomPolicies()
 {  
    isTest=false;
    ROS_ERROR("agent_pg: Start training random policies");
    episodeNum = 1;
    
	setPolicyToRandom();
    
    // Set world
	world_.request.numWorld = DEFAULT_WORLD_;
	worldcontrolclient_.call(world_);
	
    // Load policy
    sendInitPolicyToMATLAB();
    for (int numPolicy = 1; numPolicy <= NUM_LEARNINGRATES_; numPolicy++)
    {
        sendInitPolicyToMATLAB(numPolicy, true);
        sendInitPolicyToMATLAB(numPolicy, false);
    }
    printPolicy();
    saveRewardHeadingToFile();
    
    ros::Rate r(10);
    while (ros::ok() && (episodeNum <= TRAIN_NUM_EPISODES))
    {
        ros::spinOnce();
        r.sleep();
        
        if (episodeNum % TRAIN_NUM_EPISODES_RANDOM_ == 0)
        {
            setPolicyToRandom();
        }
        
        if (reinforcement_client_.call(srv))
        {
            update();
            std::pair<float,float> action = getAction();
            srv.request.action.linear_vel = action.first;
            srv.request.action.angular_vel = action.second;
        }
    }
 }
 
 void startTesting()
 {
    assert(isTest);
    ROS_ERROR("agent_pg: Start testing");
    episodeNum = 1;
    numFailures = 0;
        
    // Set world
	world_.request.numWorld = DEFAULT_WORLD_;
	worldcontrolclient_.call(world_);
	
    ros::Rate r(10);
    int numLearningRate1, numLearningRate2;
    numLearningRate1 = numLearningRate2 = 1;
    TEST_LEARNINGRATE1_ = numLearningRate1;
    TEST_LEARNINGRATE2_ = numLearningRate2;
    
    // Load policy
    loadPolicyFromFile(TEST_LEARNINGRATE1_, true);
    loadPolicyFromFile(TEST_LEARNINGRATE2_, false);
    printPolicy();
    saveRewardHeadingToFile();
    ROS_ERROR("agent_pg: Testing world%d-policy(%d,%d)\n", DEFAULT_WORLD_, TEST_LEARNINGRATE1_, TEST_LEARNINGRATE2_);

    while (ros::ok() && (numLearningRate1 <= NUM_LEARNINGRATES_) && (numLearningRate2 <= NUM_LEARNINGRATES_))
    {
        ros::spinOnce();
        r.sleep();
        
        if (reinforcement_client_.call(srv))
        {
            update();
            std::pair<float,float> action = getAction();
            srv.request.action.linear_vel = action.first;
            srv.request.action.angular_vel = action.second;
        }
        
        if ((episodeNum % (TEST_NUM_EPISODES_+1) == 0) || (numFailures > TEST_MAX_NUM_FAILURES_))
        {
            ROS_ERROR("episodeNum=%d, TEST_NUM_EPISODES=%d, numFailures=%d, TEST_MAX_NUM_FAILURES=%d", episodeNum, TEST_NUM_EPISODES_, numFailures, TEST_MAX_NUM_FAILURES_);
            episodeNum = 1;
            numFailures = 0;
            
            // Load new policy
            numLearningRate2++;
            TEST_LEARNINGRATE2_ = numLearningRate2;
            while ((loadPolicyFromFile(TEST_LEARNINGRATE2_, false) != 0) && (numLearningRate2 <= NUM_LEARNINGRATES_))
            {
                numLearningRate2++;
                TEST_LEARNINGRATE2_ = numLearningRate2;
                
            }
            if (numLearningRate2 > NUM_LEARNINGRATES_)
            {
                numLearningRate2 = 1;
                TEST_LEARNINGRATE2_ = numLearningRate2;
                
                numLearningRate1++;
                TEST_LEARNINGRATE1_ = numLearningRate1;
                
                while ((loadPolicyFromFile(TEST_LEARNINGRATE1_, true) != 0) && (numLearningRate1 <= NUM_LEARNINGRATES_))
                {
                    numLearningRate1++;
                    TEST_LEARNINGRATE1_ = numLearningRate1;
                }
            }
            if (numLearningRate1 > NUM_LEARNINGRATES_)
                return;
                
            printPolicy();
            saveRewardHeadingToFile();
            ROS_ERROR("agent_pg: Testing world%d-policy(%d,%d)\n", DEFAULT_WORLD_, TEST_LEARNINGRATE1_, TEST_LEARNINGRATE2_);
        }
    }
 }
 
  void startTesting(int numLearningRate1, int numLearningRate2)
 {
    assert(isTest);
    ROS_ERROR("agent_pg: Start testing");
    episodeNum = 1;
    numFailures = 0;
        
    // Set world
	world_.request.numWorld = DEFAULT_WORLD_;
	worldcontrolclient_.call(world_);
	
    ros::Rate r(10);
    TEST_LEARNINGRATE1_ = numLearningRate1;
    TEST_LEARNINGRATE2_ = numLearningRate2;
    
    // Load policy
    loadPolicyFromFile(TEST_LEARNINGRATE1_, true);
    loadPolicyFromFile(TEST_LEARNINGRATE2_, false);
    printPolicy();
    saveRewardHeadingToFile();
    ROS_ERROR("agent_pg: Testing world%d-policy(%d,%d)\n", DEFAULT_WORLD_, TEST_LEARNINGRATE1_, TEST_LEARNINGRATE2_);

    while (ros::ok())
    {
        ros::spinOnce();
        r.sleep();
        
        if (reinforcement_client_.call(srv))
        {
            //printSrv();
            update();
            std::pair<float,float> action = getAction();
            srv.request.action.linear_vel = action.first;
            srv.request.action.angular_vel = action.second;
        }
            
        printPolicy();
        ROS_ERROR("agent_pg: Testing world%d-policy(%d,%d)\n", DEFAULT_WORLD_, TEST_LEARNINGRATE1_, TEST_LEARNINGRATE2_);
    }
 }
 
 
int main(int argc, char **argv)
{
    ROS_ERROR("Hello from agent");
    
	ros::init(argc, argv, "agent_pg");
	ros::NodeHandle n;
	srand (time(NULL));
	
	reinforcement_client_ = n.serviceClient<lilee_rl_continuous::AgentEnvironment>("agent_environment");
	reinforcement_client_.waitForExistence();
	
    worldcontrolclient_ = n.serviceClient<lilee_rl_continuous::WorldControl>("/world_control");
	
    teleopcmdsub_ = n.subscribe("teleop_cmd", 1, teleopcmdcb);
    pgoutputsub_ = n.subscribe("/pg_output", 20, pgoutputcb);
    pginputpub_ = n.advertise<geometry_msgs::Transform>("/pg_input", 1);
    pginitpolicypub_ = n.advertise<geometry_msgs::Transform>("/pg_init_policy", 100);
    while (pginitpolicypub_.getNumSubscribers() == 0) { ros::Duration(0.1).sleep(); }
    
	// Initialize
	lilee_rl_continuous::GoalLocation location;
	location.position_x = location.position_z = std::numeric_limits<float>::quiet_NaN();
	srv.response.state.goal = location;
	srv.response.state.bumper.bumper = 1; srv.response.state.bumper.state = 0;
	srv.response.reward = 0.0;
	srv.request.action.linear_vel = srv.request.action.angular_vel = userLinearVelCmd_ = userAngularVelCmd_ = 0; 
	globalCnt = episodicCnt = 0;
	teleopcmdFlag_ = false;
	pgoutputFlag_ = false;
	userControl_ = true;
	cumulReward = 0.0;
	episodeNum = 1;
	
	// Train
	isTest = false;
	startTraining();
	
	// Test
	isTest = true;
	startTesting(3, 2);
	
	return 0;
}
