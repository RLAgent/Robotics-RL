#ifndef ACTIONS_H_
#define ACTIONS_H_
#include "geometry_msgs/Twist.h"

namespace actions
{
    extern int NUM_ACTIONS_;
    extern double ACTION_DURATION_;

    enum Move
    {
        FORWARD_ = 0,
        TURN_LEFT_ = 1,
        TURN_RIGHT_ = 2,
        STOP_ = 3
    };

    geometry_msgs::TwistPtr move_forward();
    geometry_msgs::TwistPtr move_turnleft();
    geometry_msgs::TwistPtr move_turnright();
    geometry_msgs::TwistPtr stop();
    geometry_msgs::TwistPtr executeAction(int action);
    
}

#endif
