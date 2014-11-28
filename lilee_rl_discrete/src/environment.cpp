#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <turtlebot_msgs/SetFollowState.h>
#include "std_msgs/String.h"
#include <iostream>
#include <sensor_msgs/Image.h>

#include <limits> // quiet_NaN()
#include <math.h> //floor
#include <arpa/inet.h> // htonl
#include <stdlib.h> // rand
#include <cmath> // std::abs
#include <tf/tf.h>

#include "gazebo_msgs/GetModelState.h" // Agent's GPS location
#include "gazebo_msgs/ModelState.h" // respawn
#include "geometry_msgs/Pose.h" // respawn
#include <geometry_msgs/Twist.h> 
#include <kobuki_msgs/BumperEvent.h> // bumper
///#include <nav_msgs/Odometry.h>

#include "lilee_rl_discrete/AgentEnvironment.h"
#include "lilee_rl_discrete/State.h"
#include "lilee_rl_discrete/actions.h" // parameters for agent's actions
#include "lilee_rl_discrete/gazebo_parameters.h" // parameters for gazebo

namespace lilee_rl_discrete
{
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
/**
 * Subscribes to point clouds from the 3dsensor, 
 * processes them, and publishes state messages.
 */

class LileeEnvironment : public nodelet::Nodelet
{
public:
  /*!
   * @brief The constructor for the reinforcement.
   * Constructor for the reinforcement.
   */
  LileeEnvironment() : min_y_(0.1), max_y_(0.5),
                        min_x_(-0.2), max_x_(0.2),
                        max_z_(0.8), goal_z_(0.6),
                        z_scale_(1.0), x_scale_(5.0)
  {
  }

///private:
    double min_y_; /**< The minimum y position of the points in the box. */
    double max_y_; /**< The maximum y position of the points in the box. */
    double min_x_; /**< The minimum x position of the points in the box. */
    double max_x_; /**< The maximum x position of the points in the box. */
    double max_z_; /**< The maximum z position of the points in the box. */
    double goal_z_; /**< The distance away from the robot to hold the centroid */
    double z_scale_; /**< The scaling factor for translational robot speed */
    double x_scale_; /**< The scaling factor for rotational robot speed */
    bool   enabled_; /**< Enable/disable following; just prevents motor commands */
    int    ul_x, ul_y, lr_x, lr_y; /** coordinate of rectangle vertices */
    float  focal_length_; /** focal length of camera */
    bool receivedRectCoord;
    
    /** Communication with agent */
    ros::ServiceServer reinforcement_srv_;
    bool goaldepthFlag, odomFlag;
    lilee_rl_discrete::State state_;
    float DIST_TO_GOAL_;
    int APPROX_ORDER_; // order of magniatude to which we approximate float position values
    lilee_rl_discrete::State TERMINAL_STATE_;
    int episodeNum, stepCnt, MAX_STEP_CNT_;
    
    int ANGLE_DIVIDER_, DIST_DIVIDER_;
    
    /** Agent's GPS location */
    ros::ServiceClient getmodelstate_;
   
    
    
   /*********************************************************
    * @brief OnInit method from node handle.
    * OnInit method from node handle. Sets up the parameters
    * and topics.
    *********************************************************/
    virtual void onInit()
    {
        ROS_ERROR("envir: hello from onInit()");
        ros::NodeHandle& nh = getMTNodeHandle();//getNodeHandle();
        ros::NodeHandle& private_nh = getMTPrivateNodeHandle();//getPrivateNodeHandle();

        private_nh.getParam("min_y", min_y_);
        private_nh.getParam("max_y", max_y_);
        private_nh.getParam("min_x", min_x_);
        private_nh.getParam("max_x", max_x_);
        private_nh.getParam("max_z", max_z_);
        private_nh.getParam("goal_z", goal_z_);
        private_nh.getParam("z_scale", z_scale_);
        private_nh.getParam("x_scale", x_scale_);
        private_nh.getParam("enabled", enabled_);
        
        receivedRectCoord = false;
        ul_x = ul_y = lr_x = lr_y = 0;
        focal_length_ = 575.8157348632812;
        goaldepthFlag = odomFlag = false;
        
        DIST_TO_GOAL_ = 0.5;
        APPROX_ORDER_ = 4;
        TERMINAL_STATE_.isTerminal = true;
        episodeNum=0;
        stepCnt=0;
        MAX_STEP_CNT_ = 50; // Max number of steps the agent can take to find goal
        
        // Constants for discretizing state space
        ANGLE_DIVIDER_ = 12;
        DIST_DIVIDER_ = 1;
        
        
        /** Agent's GPS location */
        getmodelstate_ = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

        reinforcement_srv_ = nh.advertiseService("agent_environment", &LileeEnvironment::agentenvironmentcb, this);
        respawnerpub_ = private_nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1);
        cmdpub_ = private_nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1); //("/cmd_vel", 1);
        rectanglesub_ = nh.subscribe<std_msgs::String>("/location_data", 1, &LileeEnvironment::rectanglecb, this);
        goaldepthsub_ = nh.subscribe("/camera/depth/image_raw", 1, &LileeEnvironment::goaldepthcb, this);
        bumpersub_ = nh.subscribe("/mobile_base/events/bumper", 1, &LileeEnvironment::bumpercb, this);
        //odomsub_ = nh.subscribe("/odom", 1, &LileeEnvironment::odomcb, this);
        
        respawn_goal();
        respawn_agent();
    }
    
    
    /*********************************************************
     * Respawn functions
     *********************************************************/
    void respawn_goal()
    {   // Respawn goal model randomly on the map.
        gazebo_msgs::ModelState modelstate;
        modelstate.model_name = goal_model_name;
        modelstate.reference_frame = "world";
        
        geometry_msgs::Pose pose;
	    pose.position.x = 2;//rand() % WORLD_BOUND_;//rand() % (2*WORLD_BOUND_) - WORLD_BOUND_;
	    pose.position.y = 2;//rand() % WORLD_BOUND_;//rand() % (2*WORLD_BOUND_) - WORLD_BOUND_;
	    pose.position.z = 0;
	    pose. orientation.w = 1.0; pose.orientation.x = pose.orientation.y = pose.orientation.z = 0;
        modelstate.pose = pose;
        
        respawnerpub_.publish(modelstate);
        //ROS_ERROR("envir: Respawned goal to (%f, %f, %f).", pose.position.x, pose.position.y, pose.position.z);
    }
    void respawn_agent()
    {   // Respawn agent model to (0,0,0) on the map.
        gazebo_msgs::ModelState modelstate;
        modelstate.model_name = agent_model_name;
        modelstate.reference_frame = "world";
        
        geometry_msgs::Pose pose;
	    //pose.position.x = AGENT_INIT_POS_X; pose.position.y = AGENT_INIT_POS_Y; pose.position.z = 0; pose.orientation.w = 1; pose.orientation.z = pose.orientation.x = pose.orientation.y = 0;
	    pose.position.x=(rand() % WORLD_BOUND_); pose.position.y=(rand() % WORLD_BOUND_); pose.position.z = 0; pose.orientation.w = (rand() % 200)*(.01)-1; pose.orientation.z = (rand() % 200)*(.01)-1;pose.orientation.x = pose.orientation.y = 0;
	    
        modelstate.pose = pose;
        
        respawnerpub_.publish(modelstate);
        //ROS_ERROR("envir: Respawned agent");
        
        
        stepCnt = 0;
        episodeNum++;
        ROS_ERROR("envir: Episode %d", episodeNum);
    }
    
    // Execute action.
    void executeAction(int action)
    {
        switch(action)
        {
            case actions::FORWARD_:
                cmdpub_.publish(actions::move_forward());
                break;
            case actions::TURN_RIGHT_:
                cmdpub_.publish(actions::move_turnright());
                break;
            case actions::TURN_LEFT_:
                cmdpub_.publish(actions::move_turnleft());
                break;
            case actions::STOP_:
                cmdpub_.publish(actions::stop());
            /*case FORWARD_LEFT_:
                move_forwardleft();
                break;
            case FORWARD_RIGHT_:
                move_forwardright();
                break;
            case BACKWARD_:
                move_backward();
                break;
            case BACKWARD_LEFT_:
                move_backwardleft();
                break;
            case BACKWARD_RIGHT_:
                move_backwardright();
                break;
                */
        }
        //ros::Duration(actions::ACTION_DURATION_).sleep();
        stepCnt++;
    }

    /*********************************************************
    * Communication with agent
    *********************************************************/
    bool agentenvironmentcb(lilee_rl_discrete::AgentEnvironment::Request& req, lilee_rl_discrete::AgentEnvironment::Response& res)
    {
	    // perform action
        executeAction(req.action);
        ros::Duration(actions::ACTION_DURATION_).sleep();
        
        // read next state
        goaldepthFlag = odomFlag = false;
	    while (!(goaldepthFlag && odomFlag)) { ros::Duration(0.1).sleep(); }
	    
	    // Send next state and reward
        if (((int)state_.distance_to_goal < 1) && (state_.theta_to_goal <= 1))
        {   // Check if goal state
            ROS_ERROR("envir: GOAL!!");
            respawn_goal();
            respawn_agent();
            res.reward = 100.0;
            res.state = TERMINAL_STATE_;
            res.state.success = 1;
        }
        else if (stepCnt > MAX_STEP_CNT_)
        {   // If agent took MAX_STEP_CNT steps, restart episode.
            ROS_ERROR("envir: Agent failed to find goal after %d steps.", MAX_STEP_CNT_);
            respawn_goal();
            respawn_agent();
            res.reward = -1.0;
            res.state = state_;
            res.state.isTerminal = true;
            res.state.success = 0;
        }
        else
        {
            //return -1*state.distance_to_goal;
            res.reward = -1.0;
            res.state = state_;
        }
	    
        return true;
    }
    
    void storeGoalNaN()
    {
        state_.goal.position_x = std::numeric_limits<float>::quiet_NaN();
        state_.goal.position_z = std::numeric_limits<float>::quiet_NaN();
        
    }
    void storeGoalXYZ(float position_x, float position_z)
    {
        state_.goal.position_x = floor(position_x * APPROX_ORDER_ + 0.5);
        state_.goal.position_z = floor(position_z * APPROX_ORDER_ + 0.5);
    }
    
    int radiansToDegrees(double radians)
    {
        return (((int)floor(radians*180/M_PI + 0.5)) % 360)/ANGLE_DIVIDER_;
    }
    
    void agentPositionCb() // Agent's GPS Location
    {
        gazebo_msgs::GetModelState agent, goal;
        agent.request.model_name = agent_model_name;
        goal.request.model_name = goal_model_name;
        float abs_theta_to_goal, difference;
        
        if (!odomFlag)
        {
            if (getmodelstate_.call(agent) && getmodelstate_.call(goal))
            {
                odomFlag = true;
                float radians;
                float x1 = agent.response.pose.position.x;
                float y1 = agent.response.pose.position.y;
                float x2 = goal.response.pose.position.x;
                float y2 = goal.response.pose.position.y;
                
               
                state_.distance_to_goal = floor(sqrt(pow((x1-x2),2) + pow((y1-y2),2))+0.5);
                if (x1 < x2)
                {
                    radians=atan((y1-y2)/(x1-x2));
                }
                else if (x1 > x2)
                {
                    radians=atan((y1-y2)/(x1-x2)) + M_PI;
                    
                }
                else
                {
                    if (y2 > y1)
                        radians = M_PI/2;
                    else
                        radians = 3*M_PI/2;
                }
                
                
                if (radians > M_PI)
                    abs_theta_to_goal = radians - 2*M_PI;
                else
                    abs_theta_to_goal= radians;
                
                odomFlag = true;
                state_.agent_location.position_x = x1;
                state_.agent_location.position_y = y1;
                
                double roll, pitch, yaw;
                tf::Quaternion q(agent.response.pose.orientation.x,
                    agent.response.pose.orientation.y,
                    agent.response.pose.orientation.z,
                    agent.response.pose.orientation.w);
                tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
                
                difference = abs_theta_to_goal - yaw;
                if (difference < 0)
                    difference = difference + 2*M_PI;
                
                if (difference > M_PI)
                    state_.theta_to_goal = radiansToDegrees(difference - 2*M_PI);
                else
                    state_.theta_to_goal = radiansToDegrees(difference);
                state_.agent_location.orientation_yaw = radiansToDegrees(yaw);
                
                state_.isTerminal = false;
            }
        }
    }
    
    /*void agentPositionCb() // Agent's GPS Location
    {
        gazebo_msgs::GetModelState agentl;
        agent.request.model_name = agent_model_name;
        
        if (!odomFlag)
        {
            if (getmodelstate_.call(agent))
            {
                odomFlag = true;
                state_.agent_location.position_x = floor(agent.response.pose.position.x * APPROX_ORDER_ + 0.5);
                state_.agent_location.position_y = floor(agent.response.pose.position.y * APPROX_ORDER_ + 0.5);
                
                double roll, pitch, yaw;
                tf::Quaternion q(agent.response.pose.orientation.x,
                    agent.response.pose.orientation.y,
                    agent.response.pose.orientation.z,
                    agent.response.pose.orientation.w);
                tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
                state_.agent_location.orientation_yaw = floor(yaw + 0.5);
            }
        }
    }*/
    
    void bumpercb(const kobuki_msgs::BumperEvent::ConstPtr& msg)
    {
        state_.bumper.bumper = msg->bumper;
        state_.bumper.state = msg->state;
        
        //if (state_.bumper.state)
           // ros::Duration(0.5).sleep();
    }
   
    
    /*
    void odomcb(const nav_msgs::Odometry::ConstPtr& msg)
    {
        if (!odomFlag)
        {
            odomFlag = true;
            
            state_.agent_location.position_x = floor((msg->pose).pose.position.x+0.5);
            state_.agent_location.position_y = floor((msg->pose).pose.position.y+0.5);
            state_.agent_location.orientation_w = floor((msg->pose).pose.orientation.w+0.5);
        }
    }*/
  /*!*******************************************************
   * @brief Process image data and publish location of object.
   *********************************************************/
  // Convert [c1, c2, c3, c4] into a float
  float charsToFloat(unsigned char c1, unsigned char c2, unsigned char c3, unsigned char c4, bool is_bigendian)
  {
    float f;
    if ( htonl(47) == 47 ) { 
      // System is big endian
      if (is_bigendian) {
        *((char *)(&f))   = c4;
        *((char *)(&f)+1) = c3;
        *((char *)(&f)+2) = c2;
        *((char *)(&f)+3) = c1;
      }
      else {
        *((char *)(&f))   = c1;
        *((char *)(&f)+1) = c2;
        *((char *)(&f)+2) = c3;
        *((char *)(&f)+3) = c4;
      }
    }
    else { 
      //System is little endian
      if (!is_bigendian) {
        *((char *)(&f))   = c1;
        *((char *)(&f)+1) = c2;
        *((char *)(&f)+2) = c3;
        *((char *)(&f)+3) = c4;
      }
      else {
        *((char *)(&f))   = c4;
        *((char *)(&f)+1) = c3;
        *((char *)(&f)+2) = c2;
        *((char *)(&f)+3) = c1;
      }
    }
    return f;
  }

    // Callback for rectangle coordinates.
    void rectanglecb(const std_msgs::String::ConstPtr& msg){
    int temp_ul_x, temp_ul_y,  temp_lr_x, temp_lr_y;
    sscanf(msg->data.c_str(), "(%d, %d),(%d, %d)", &temp_ul_x, &temp_ul_y, &temp_lr_x, &temp_lr_y);

    ul_x = temp_ul_x;
    ul_y = temp_ul_y;
    lr_x = temp_lr_x;
    lr_y = temp_lr_y;

    receivedRectCoord = true;
    }
    
    //Callback for /camera/depth/image.
    void goaldepthcb(const sensor_msgs::ImageConstPtr& msg)
     {
        // TODO: DEBUG
        //ROS_ERROR("goaldepthcb! goaldepthFlag = %d", goaldepthFlag);
        
        // TODO: Put this in separate callback
        agentPositionCb(); // Agent's GPS location
      
        if (!goaldepthFlag) {
          goaldepthFlag = true;

          // TODO: Obstacle detection
          
          // Goal location
          if (!receivedRectCoord)
          {
            storeGoalNaN();
          }
          else
          {
            receivedRectCoord = false;
            float x = 0.0, y = 0.0, z = 1e6, temp_x, temp_y, temp_z;
            unsigned int n = 0;

            // for each (x, y) inside rectangle:
            for (int i = ul_x; i < lr_x; i++) for (int j = ul_y; j < lr_y; j++)
            {
              int index = j*4*(msg->width) + i*4;
              temp_z = charsToFloat(msg->data[index], msg->data[index+1], msg->data[index+2], msg->data[index+3], msg->is_bigendian);
              if (!std::isnan(temp_z) && temp_z > 0.0)
              {
                temp_x = (i-320)*temp_z/focal_length_;
                temp_y = (j-240)*temp_z/focal_length_;
                if (-temp_y > min_y_ && -temp_y < max_y_ && temp_x < max_x_ && temp_x > min_x_ && temp_z < max_z_)
                {
                  x += temp_x;
                  y += temp_y;
                  z = std::min(z, temp_z);
                  n++;
                }
              }
            }
            if (n>100)
            {
              x /= n;
              y /= n;
              if(z > max_z_)
              {
                if (enabled_)
                    storeGoalNaN();
                return;
              }
              if (enabled_)
                storeGoalXYZ(x,z);
            }
            else
            {
              if (enabled_)
                storeGoalNaN();
            }
          } // end else
          
      } // end if(receivedAction)
    } // end goaldepthcb

  ros::Subscriber rectanglesub_, goaldepthsub_, odomsub_, bumpersub_;
  ros::Publisher cmdpub_, respawnerpub_;
};

PLUGINLIB_DECLARE_CLASS(lilee_rl_discrete, LileeEnvironment, lilee_rl_discrete::LileeEnvironment, nodelet::Nodelet);

}
