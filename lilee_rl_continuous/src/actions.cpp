#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "lilee_rl_continuous/actions.h" // parameters for agent's actions

namespace actions {
    double ACTION_DURATION_ = 0.5; // duration of each action (sec)
    double LINEAR_X_ = 0.6; // linear.x velocity
    double ANGULAR_Z = 0.45;
}
