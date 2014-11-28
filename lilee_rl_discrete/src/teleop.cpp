#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include "lilee_rl_discrete/actions.h"
#include "lilee_rl_discrete/TeleopCmd.h"

#define KEYCODE_R 0x43 
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71

class LileeTeleop
{
public:
  LileeTeleop();
  void keyLoop();
  void watchdog();
  
private:

  ros::NodeHandle nh_,ph_;
  double linear_, angular_;
  ros::Time first_publish_;
  ros::Time last_publish_;
  double l_scale_, a_scale_;
  ros::Publisher cmdpub_, teleopcmdpub_;
  void publish(double, double);
  void sendTeleopCmd(int);
  void setUserControl();
  boost::mutex publish_mutex_;
    
  bool userControl;
    
};

LileeTeleop::LileeTeleop():
  ph_("~"),
  linear_(0),
  angular_(0),
  l_scale_(1.0),
  a_scale_(1.0)
{
  ph_.param("scale_angular", a_scale_, a_scale_);
  ph_.param("scale_linear", l_scale_, l_scale_);

  cmdpub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
  teleopcmdpub_ = nh_.advertise<lilee_rl_discrete::TeleopCmd>("teleop_cmd",1);
  
  userControl = true;
}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop");
  LileeTeleop Lilee_teleop;
  ros::NodeHandle n;

  signal(SIGINT,quit);

  boost::thread my_thread(boost::bind(&LileeTeleop::keyLoop, &Lilee_teleop));
  
  
  //ros::Timer timer = n.createTimer(ros::Duration(0.1), boost::bind(&LileeTeleop::watchdog, &Lilee_teleop));

  ros::spin();

  my_thread.interrupt() ;
  my_thread.join() ;
  return(0);
}


void LileeTeleop::watchdog()
{
  boost::mutex::scoped_lock lock(publish_mutex_);
  if ((ros::Time::now() > last_publish_ + ros::Duration(0.15)) && 
      (ros::Time::now() > first_publish_ + ros::Duration(0.50)))
    publish(0, 0);
}

void LileeTeleop::sendTeleopCmd(int action)
{
    lilee_rl_discrete::TeleopCmd msg;
    msg.action = action;
    msg.user_control = userControl;
    
    teleopcmdpub_.publish(msg);
}

void LileeTeleop::setUserControl()
{
    if (userControl)
    {
        userControl = false;
        
    }
    else
    {
        userControl = true;
    }
    lilee_rl_discrete::TeleopCmd msg;
    msg.user_control = userControl;
    teleopcmdpub_.publish(msg);
}

void LileeTeleop::keyLoop()
{
  char c;

  // get the console in raw mode                                                              
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file                         
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move the Turtlebot.");


  while (ros::ok())
  {
    // get the next event from the keyboard  
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    linear_=angular_=0;
    ROS_DEBUG("value: 0x%02X\n", c);
  
    switch(c)
    {
        case KEYCODE_L:
            sendTeleopCmd(actions::TURN_LEFT_);
            break;
        case KEYCODE_R:
            sendTeleopCmd(actions::TURN_RIGHT_);
            break;
        case KEYCODE_U:
            sendTeleopCmd(actions::FORWARD_);
            break;
        case KEYCODE_D:
            setUserControl();
            break;
    }
    
    boost::mutex::scoped_lock lock(publish_mutex_);
    if (ros::Time::now() > last_publish_ + ros::Duration(1.0)) { 
      first_publish_ = ros::Time::now();
    }
    last_publish_ = ros::Time::now();
  }
  return;
}

void LileeTeleop::publish(double angular, double linear)  
{
    geometry_msgs::Twist vel;
    vel.angular.z = a_scale_*angular;
    vel.linear.x = l_scale_*linear;

    cmdpub_.publish(vel);    


  return;
}
