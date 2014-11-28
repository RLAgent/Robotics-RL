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
#include "std_msgs/Int16.h"

#include "lilee_rl_continuous/AgentEnvironment.h"
#include "lilee_rl_continuous/GoalLocation.h"
#include "lilee_rl_continuous/actions.h"
#include "lilee_rl_continuous/TeleopCmd.h"
#include "lilee_rl_continuous/WorldControl.h"
#include "lilee_rl_continuous/world_control.h"  // world_control::NUM_WORLDS_

ros::ServiceClient reinforcement_client_, worldcontrolclient_;
ros::Subscriber teleopcmdsub_, pgoutputsub_, pgellataskindexsub_;
ros::Publisher pginputpub_, pginitpolicypub_;

// Saving trajectories
lilee_rl_continuous::AgentEnvironment srv;
lilee_rl_continuous::State state_;

std::map<std::string, float> qValues;
double EPSILON_ = 0; // prob. (in %) of choosing random action
double DELTA_ = 70; // prob. (in %) of following user's policy

float MIN_LINEAR_VEL_ = -1.0;
float MAX_LINEAR_VEL_ = 1.5;
float MIN_ANGULAR_VEL_=-1.5;
float MAX_ANGULAR_VEL_= 1.5;

std::string policy_filename = "pg-ella_POLICY";
std::string policy1_filename = "pg-ella_POLICY_world%d-policy%d-linear";
std::string policy2_filename = "pg-ella_POLICY_world%d-policy%d-angular";
std::string plot_filename = "pg-ella_PLOT_world%d-policy(%d,%d)";

int PARAM_CNT_ = 1000; // Update parameters after PARAM_CNT_ steps
double DECAY_ = 0.95;
int globalCnt;   // Total number of steps taken over all episodes
int episodicCnt; // Number of steps taken in an episode
int episodeNum;  // Episode number
int numFailures = 0; // Number of failures
double cumulReward; // Cumulative reward in an episode

bool userControl_, teleopcmdFlag_, pgoutputFlag_;
float userLinearVelCmd_, userAngularVelCmd_;

// Communication with MATLAB/PG
geometry_msgs::Transform curStateActionReward_;

static const int NUM_TASKS_ =6; // number of tasks
static const int DIM_STATE_SPACE_ = 2; // dimension of state space
static const int DIM_ACTION_SPACE_ = 2;// dimension of action space

// Policy for linear_vel and angular_vel
float k1[2], k2[2]; // thetas
float SIGMA1_, SIGMA2_; // sigma
float theta[NUM_TASKS_][DIM_ACTION_SPACE_][DIM_STATE_SPACE_]; // theta
float sigma[NUM_TASKS_][DIM_ACTION_SPACE_]; // sigma
float CONST_SIGMA_ = 0.1;

// Which policy the agent should follow
// Set 0 if agent shouldn't change policy
int DEFAULT_LEARNINGRATE1_ = 0;
int DEFAULT_LEARNINGRATE2_ = 0;

// if isTest==true, turns off learning and tests learning rates (TEST_LEARNINGRATE1_, TEST_LEARNINGRATE2_)
bool isTest = false;  
int TEST_LEARNINGRATE1_;// = 1;
int TEST_LEARNINGRATE2_;// = 1;
int NUM_LEARNINGRATES_ = 4;
int TRAIN_NUM_EPISODES = 10000;
int TEST_NUM_EPISODES_ = 25;
int TEST_MAX_NUM_FAILURES_ = ceil(TEST_NUM_EPISODES_/5.0); // Max number of failures

// Initial policy values
float INIT_K1[] = {0.3, -0.005};
float INIT_K2[] = {0.0001, 0.5};
int STATE_DIM_ = 2; // dimension of state space

float INIT_K1_0[] = {0.6,  0.5,  0.3,   0.3,   0.3, 0.3};
float INIT_K1_1[] = {-0.005,-0.005, -0.005, -0.005, -0.005, -0.005};
float INIT_K2_0[] = {0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001};
float INIT_K2_1[] = {0.7,  0.7,  0.4,   0.5,   0.5, 0.5};
float INIT_SIGMA1_[] = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
float INIT_SIGMA2_[] = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1};


// world_control
lilee_rl_continuous::WorldControl world_;
int DEFAULT_WORLD_ = 1;

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
void setPolicyToDefault(int taskIndex)
{    
    k1[0] = INIT_K1_0[taskIndex-1];
    k1[1] = INIT_K1_1[taskIndex-1];
    k2[0] = INIT_K2_0[taskIndex-1];
    k2[1] = INIT_K2_1[taskIndex-1];
    
	SIGMA1_ = INIT_SIGMA1_[taskIndex-1];
	SIGMA2_ = INIT_SIGMA2_[taskIndex-1];
}

void setPolicy(int taskIndex)
{
    for (int s=0; s < DIM_STATE_SPACE_; s++)
    {
        k1[s] = theta[taskIndex-1][0][s];
        k2[s] = theta[taskIndex-1][1][s];
    }
    
    ROS_ERROR("%f, %f", theta[taskIndex-1][0][0], theta[taskIndex-1][0][0]);
    
    printPolicy();
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
    for (int i  = 0; i < NUM_TASKS_; i++)
    {
        for (int j = 0; j < DIM_ACTION_SPACE_; j++)
        {
            for (int k = 0; k < DIM_STATE_SPACE_; k++)
                fprintf(fp, "%f\t", theta[i][j][k]);
            fprintf(fp, "%f\t", sigma[i][j]);
        }
        fprintf(fp, "\n");
    }
    
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
                    ROS_ERROR("agent_pg-ella: Policy %d for linear is nan. Resetting policy to default.", numPolicy);
                    setPolicyToDefault(numPolicy);
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
                    ROS_ERROR("agent_pg-ella: Policy %d for angular is nan. Resetting policy to default.", numPolicy);
                    setPolicyToDefault(numPolicy);
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
    
    if ((globalCnt % PARAM_CNT_ == 0) && (DEFAULT_WORLD_ == 6))
    {   // update parameters
        DELTA_ *= DECAY_;
        EPSILON_ *= DECAY_;
        
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
 * Choosing action
 *********************************************************/
std::pair<float, float> getRandomAction()
{   // Return a random action.

    float linear_vel = fRand(MIN_LINEAR_VEL_, MAX_LINEAR_VEL_);
    float angular_vel = fRand(MIN_ANGULAR_VEL_, MAX_ANGULAR_VEL_);

    return std::pair<float, float>(linear_vel, angular_vel);
}

std::pair<float, float> getPolicy(lilee_rl_continuous::State state, float k1_0, float k1_1, float sigma1, float k2_0, float k2_1, float sigma2)
{   // Return the next action from policy.
    float mean1 = k1_0*state.distance_to_goal
            +  k1_1*fabs(state.theta_to_goal);
    float mean2 = k2_0*state.distance_to_goal
            +  k2_1*state.theta_to_goal;
    
    float linear_vel = gaussian(mean1,  sigma1);
    float angular_vel = gaussian(mean2, sigma2);
    
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
{   // Return the next action from policy.
    
    return getPolicy(state, k1[0], k1[1], SIGMA1_, k2[0], k2[1], SIGMA2_);
}

std::pair<float, float> getUserPolicy(lilee_rl_continuous::State state)
{   // Return the next action based on the user's policy.
    
    return getPolicy(state, INIT_K1[0], INIT_K1[1], SIGMA1_, INIT_K2[0], INIT_K2[1], SIGMA2_);
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
            
            // TODO: add user policy
            //userPolicy[stateToString(state_)][userActionCmd_]++;
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

 void pgellataskindexcb(const std_msgs::Int16::ConstPtr& msg)
 { 
    DEFAULT_WORLD_=(int)msg->data;
    
    // set policy
    if (DEFAULT_WORLD_ != 6)
        setPolicyToDefault(DEFAULT_WORLD_);
	
	// set world
	world_.request.numWorld = DEFAULT_WORLD_;
	while (!worldcontrolclient_.call(world_)) { ros::Duration(0.1).sleep(); }
 }
void pgoutputcb(const geometry_msgs::Transform::ConstPtr& msg)
{
    pgoutputFlag_ = true;
    
    if (!isTest)
    {
        int taskIndex = (int)msg->translation.y;
        int actionIndex = (int)msg->translation.z;
        ROS_ERROR("taskIndex=%d\n", taskIndex);
        assert(taskIndex <= NUM_TASKS_);
        assert (actionIndex < DIM_ACTION_SPACE_);
        
        theta[taskIndex-1][actionIndex][0] = msg->rotation.x;
        theta[taskIndex-1][actionIndex][1] = msg->rotation.y;
        sigma[taskIndex-1][actionIndex] = msg->translation.x;
        
        savePolicyToFile();
        
        if (taskIndex == 6)
            setPolicy(taskIndex);
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
    isTest = false;
    ROS_ERROR("agent_pg: Start training");
    
    // Set world
	world_.request.numWorld = DEFAULT_WORLD_;
	worldcontrolclient_.call(world_);
	
    // Load policy
	setPolicyToDefault(DEFAULT_WORLD_);
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
            //printSrv();
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
 
int main(int argc, char **argv)
{
    ROS_ERROR("Hello from agent");
    
	ros::init(argc, argv, "agent_pg");
	ros::NodeHandle n;
	srand (time(NULL));
	
	reinforcement_client_ = n.serviceClient<lilee_rl_continuous::AgentEnvironment>("agent_environment");
	reinforcement_client_.waitForExistence();
	
    worldcontrolclient_ = n.serviceClient<lilee_rl_continuous::WorldControl>("/world_control");
    worldcontrolclient_.waitForExistence();
	
	pgellataskindexsub_ = n.subscribe("/pg_ella_task_index", 1, pgellataskindexcb);
    teleopcmdsub_ = n.subscribe("teleop_cmd", 1, teleopcmdcb);
    pgoutputsub_ = n.subscribe("/pg_ella_output", 20, pgoutputcb);
    
    pginputpub_ = n.advertise<geometry_msgs::Transform>("/pg_ella_input", 1);
    while (pginputpub_.getNumSubscribers() == 0) {ros::Duration(0.1).sleep(); }
    
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
	
	return 0;
}
