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
    
    /*
    void move_backward()
    {
        //ROS_ERROR("agent: In move_backward()");
        geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());    
        cmd->linear.x = -1*LINEAR_X_;
        cmd->angular.z = 0.0;
        cmdpub_.publish(cmd);
        ros::Duration(ACTION_DURATION_).sleep();
        
        
    }
    void move_forwardleft()
    {
        //ROS_ERROR("agent: In move_forwardleft()");
        geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());    
        cmd->linear.x = LINEAR_X_;
        cmd->angular.z = 0.5;
        cmdpub_.publish(cmd);
        ros::Duration(ACTION_DURATION_).sleep();
        
    }
    void move_forwardright()
    {
        //ROS_ERROR("agent: In move_forwardright()");
        geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());    
        cmd->linear.x = LINEAR_X_;
        cmd->angular.z = -0.5;
        cmdpub_.publish(cmd);
        ros::Duration(ACTION_DURATION_).sleep();
    }
    void move_backwardleft()
    {
        //ROS_ERROR("agent: In move_forwardleft()");
        geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());    
        cmd->linear.x = -1*LINEAR_X_;
        cmd->angular.z = 0.5;
        cmdpub_.publish(cmd);
        ros::Duration(ACTION_DURATION_).sleep();
        
    }
    void move_backwardright()
    {
        //ROS_ERROR("agent: In move_forwardright()");
        geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());    
        cmd->linear.x = -1*LINEAR_X_;
        cmd->angular.z = -0.5;
        cmdpub_.publish(cmd);
        ros::Duration(ACTION_DURATION_).sleep();
    }*/
}
