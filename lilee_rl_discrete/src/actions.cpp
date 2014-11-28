#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "lilee_rl_discrete/actions.h" // parameters for agent's actions

namespace actions {
    int NUM_ACTIONS_ = 3; // Number of discrete actions
    
    double ACTION_DURATION_ = 0.5; // duration of each action (sec)
    double LINEAR_X_ = 1; // linear.x velocity
    double ANGULAR_Z = 0.45;
    
    /*********************************************************
     * Agent's actions
     *********************************************************/
    geometry_msgs::TwistPtr move_forward()
    {
        geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());    
        cmd->linear.x = LINEAR_X_;
        cmd->angular.z = 0.0;
        
        return cmd;
    }
    geometry_msgs::TwistPtr move_turnleft()
    {
        geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());    
        cmd->linear.x = 0.0;
        cmd->angular.z = ANGULAR_Z;
        return cmd;
    }
    geometry_msgs::TwistPtr move_turnright()
    {
        geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());    
        cmd->linear.x = 0.0;
        cmd->angular.z = -1*ANGULAR_Z;
        return cmd;
    }
    geometry_msgs::TwistPtr stop()
    {
        geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());
        return cmd;
    }
}
