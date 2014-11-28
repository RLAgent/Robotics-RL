#ifndef ACTIONS_H_
#define ACTIONS_H_
#include "geometry_msgs/Twist.h"

namespace actions {
    extern int NUM_ACTIONS_;
    extern double ACTION_DURATION_;

    enum Move
    {
        FORWARD_ = 0,
        //BACKWARD_ = 1,
        //FORWARD_LEFT_ = 2,
        //FORWARD_RIGHT_ = 3,
        TURN_LEFT_ = 1,
        TURN_RIGHT_ = 2,
        //BACKWARD_LEFT_ = 6,
        //BACKWARD_RIGHT_ = 7
        STOP_ = 3//NUM_ACTIONS_
    };

    geometry_msgs::TwistPtr move_forward();
    geometry_msgs::TwistPtr move_turnleft();
    geometry_msgs::TwistPtr move_turnright();
    geometry_msgs::TwistPtr stop();
    geometry_msgs::TwistPtr executeAction(int action);
    
}

#endif
