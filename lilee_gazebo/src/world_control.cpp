#include "ros/ros.h"
#include <string> //std::string

#include "lilee_rl_continuous/world_control.h"
#include "gazebo_msgs/SpawnModel.h"
#include "gazebo_msgs/DeleteModel.h"
#include "std_srvs/Empty.h"
#include "lilee_rl_continuous/WorldControl.h"

namespace world_control
{
    int NUM_WORLDS_ = 5;
}

ros::ServiceClient pause_client, unpause_client, gazebo_spawn_client, gazebo_delete_client;
ros::ServiceServer worldcontrolsrv_;

/*********************************************************
 * Pause/Unpause world
 *********************************************************/
int pauseWorld()
{
    std_srvs::Empty empty;
    if (!pause_client.call(empty))
    {
        ROS_ERROR("unable to pause physics engine");
        return -1;
    }
    return 0;
}

int unpauseWorld()
{
    std_srvs::Empty empty;
    if (!unpause_client.call(empty))
    {
        ROS_ERROR("unable to unpause physics engine");
        return -1;
    }
    return 0;
}

/*********************************************************
 * Change friction of 'ground_plane' model to (mu, mu2)
 *********************************************************/
int changeFriction(double mu, double mu2)
{
    pauseWorld();
    
    char buffer [780];
    sprintf(buffer,
     "<sdf version='1.4'><model name='ground_plane'><static>1</static><link name='link'><collision name='collision'><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry><surface><friction><ode><mu>%f</mu><mu2>%f</mu2></ode></friction><bounce/><contact><ode/></contact></surface><max_contacts>10</max_contacts></collision><visual name='visual'><cast_shadows>0</cast_shadows><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry><material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Grey</name></script></material></visual><velocity_decay><linear>0</linear><angular>0</angular></velocity_decay><self_collide>0</self_collide><kinematic>0</kinematic><gravity>1</gravity></link></model></sdf>",
        mu, mu2
        );
    
    // Delete 'ground_plane'
    gazebo_msgs::DeleteModel deletemodel;
    deletemodel.request.model_name = "ground_plane";
    gazebo_delete_client.call(deletemodel);
    
    // respawn new 'ground_plane'
    gazebo_msgs::SpawnModel model;
    model.request.model_xml = buffer;

    
    model.request.model_name="ground_plane";
    model.request.reference_frame="world";
    if (!gazebo_spawn_client.call(model))
    {
        ROS_ERROR("world_control: Failed to respawn new 'ground_plane'");
        return -1;
    }
    else
        ROS_ERROR("world_control: Changed friction to (%f, %f)", mu, mu2);
    
    unpauseWorld();
    
    return 0;
}

int changeWorld(int numWorld)
{   // Change world to numWorld
    
    if (numWorld > world_control::NUM_WORLDS_)
    {
        ROS_ERROR("world_control: Invalid world number");
        return -1;
    }
    
    if (numWorld == 1)
        changeFriction(100, 50);
    else if (numWorld == 2)
        changeFriction(5,10);
    else if (numWorld == 3)
        changeFriction(100, 1.5);
    else if (numWorld == 4)
        changeFriction(3, 1.5);
    else if (numWorld == 5)
        changeFriction(0.6, 50);
    
    return 0;
}
/*********************************************************
 * Callback
 *********************************************************/
bool worldcontrolcb(lilee_rl_continuous::WorldControl::Request& req, lilee_rl_continuous::WorldControl::Response& res)
{   
    int iSuccess = changeWorld((int)req.numWorld);
    if (iSuccess != 0)
        return false;
    else
        return true;
}

/*********************************************************
 * Main
 *********************************************************/
int main(int argc, char **argv)
{
    ros::init(argc, argv, "world_control");
    ros::NodeHandle n;

    pause_client = n.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
    pause_client.waitForExistence();

    unpause_client = n.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");
    unpause_client.waitForExistence();
    
    gazebo_spawn_client = n.serviceClient<gazebo_msgs::SpawnModel> ("/gazebo/spawn_sdf_model");
    gazebo_spawn_client.waitForExistence();
    
    gazebo_delete_client = n.serviceClient<gazebo_msgs::DeleteModel> ("/gazebo/delete_model");
    gazebo_delete_client.waitForExistence();
    
    worldcontrolsrv_ = n.advertiseService("/world_control", worldcontrolcb);
    
    ros::spin();
    
    return 0;
}
