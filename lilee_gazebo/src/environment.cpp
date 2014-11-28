#include "ros/ros.h"
#include <stdlib.h> // rand
#include <cmath> // std::abs
#include "gazebo_msgs/GetModelState.h"
#include "gazebo_msgs/ModelState.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"

std::string goal_model_name = "fire_hydrant";
std::string agent_model_name = "mobile_base";

ros::ServiceClient getmodelstate;
ros::Publisher respawner_pub, reward_pub;
gazebo_msgs::GetModelState agent_getmodelstate, goal_getmodelstate;

int counter;

// Respawn goal model randomly on the map.
void respawn()
{
    gazebo_msgs::ModelState goal_modelstate;
    
    goal_modelstate.model_name = goal_model_name;
    goal_modelstate.reference_frame = "world";
    
    geometry_msgs::Pose new_pose;
	new_pose.position.x = rand() % 20 - 10; new_pose.position.y = rand() % 20 - 10; new_pose.position.z = 0;
	new_pose. orientation.w = 1.0; new_pose.orientation.x = new_pose.orientation.y = new_pose.orientation.z = 0;
    goal_modelstate.pose = new_pose;
    
    respawner_pub.publish(goal_modelstate);
    ROS_INFO("Respawned goal to (%f, %f, %f).", new_pose.position.x, new_pose.position.y, new_pose.position.z);
}

void send_reward(float32 val)
{
    std_msgs::Float32 reward;
    reward.data = val;
    reward_pub.publish(reward);
    
    ROS_INFO("environment: Sent reward %f", val);
}

// Turtlebot has reached goal state. Publish reward and respawn goal model.
void goal_state()
{
    ROS_INFO("In goal_state()");
    respawn();
    send_reward(1000.0);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "environment");
	ros::NodeHandle n;
	srand (time(NULL));
	
	getmodelstate = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
	reward_pub = n.advertise<std_msgs::String>("rewards", 1000);
	respawner_pub = n.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1);
	
	agent_getmodelstate.request.model_name = agent_model_name; 
	goal_getmodelstate.request.model_name = goal_model_name;
	
	while (ros::ok())
	{
	    {
	        if (getmodelstate.call(agent_getmodelstate) && getmodelstate.call(goal_getmodelstate))
	        {
	            if (
	            (std::abs(agent_getmodelstate.response.pose.position.x - goal_getmodelstate.response.pose.position.x) < 0.8) &&
	            (std::abs(agent_getmodelstate.response.pose.position.y - goal_getmodelstate.response.pose.position.y) < 0.8) &&
	            (std::abs(agent_getmodelstate.response.pose.position.z - goal_getmodelstate.response.pose.position.z) < 0.1)
	            )
	            {
	                //goal_state();
	                ros::spinOnce();
                    ros::Duration(5).sleep();
	            }
	            else
	            {
	                // publish -1 for step cost
	            }
	        }
	    }
	}
	
	return 0;
}
