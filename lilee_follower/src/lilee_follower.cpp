#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <geometry_msgs/Twist.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <visualization_msgs/Marker.h>
#include <turtlebot_msgs/SetFollowState.h>

#include "std_msgs/String.h"
#include <iostream>
#include <sensor_msgs/Image.h>
#include <arpa/inet.h> // htonl
#include "lilee_reinforcement/State.h"

#include "dynamic_reconfigure/server.h"
#include "lilee_follower/FollowerConfig.h"

namespace lilee_follower
{
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
/**
 * The turtlebot follower nodelet. Subscribes to point clouds
 * from the 3dsensor, processes them, and publishes command vel
 * messages.
 */
class LileeFollower : public nodelet::Nodelet
{
public:
  /*!
   * @brief The constructor for the follower.
   * Constructor for the follower.
   */
  LileeFollower() : min_y_(0.1), max_y_(0.5),
                        min_x_(-0.2), max_x_(0.2),
                        max_z_(0.8), goal_z_(0.6),
                        z_scale_(1.0), x_scale_(5.0)
  {

  }

  ~LileeFollower()
  {
    delete config_srv_;
  }

private:
  double min_y_; /**< The minimum y position of the points in the box. */
  double max_y_; /**< The maximum y position of the points in the box. */
  double min_x_; /**< The minimum x position of the points in the box. */
  double max_x_; /**< The maximum x position of the points in the box. */
  double max_z_; /**< The maximum z position of the points in the box. */
  double goal_z_; /**< The distance away from the robot to hold the centroid */
  double z_scale_; /**< The scaling factor for translational robot speed */
  double x_scale_; /**< The scaling factor for rotational robot speed */
  bool   enabled_; /**< Enable/disable following; just prevents motor commands */
  float focal_length_;
  bool flag;

  // coordinate of rectangle vertices
  int ul_x, ul_y, ll_x, ll_y, lr_x, lr_y, ur_x, ur_y;

  // Service for start/stop following
  ros::ServiceServer switch_srv_;

  // Dynamic reconfigure server
  dynamic_reconfigure::Server<lilee_follower::FollowerConfig>* config_srv_;

  /*!
   * @brief OnInit method from node handle.
   * OnInit method from node handle. Sets up the parameters
   * and topics.
   */
  virtual void onInit()
  {
    ROS_ERROR("lilee_follower: hello from onInit()");
    ros::NodeHandle& nh = getNodeHandle();
    ros::NodeHandle& private_nh = getPrivateNodeHandle();

    flag = true;
    ul_x = ul_y = lr_x = lr_y = 0;
    focal_length_ = 575.8157348632812;

    private_nh.getParam("min_y", min_y_);
    private_nh.getParam("max_y", max_y_);
    private_nh.getParam("min_x", min_x_);
    private_nh.getParam("max_x", max_x_);
    private_nh.getParam("max_z", max_z_);
    private_nh.getParam("goal_z", goal_z_);
    private_nh.getParam("z_scale", z_scale_);
    private_nh.getParam("x_scale", x_scale_);
    private_nh.getParam("enabled", enabled_);

    cmdpub_ = private_nh.advertise<geometry_msgs::Twist> ("cmd_vel", 1);
    markerpub_ = private_nh.advertise<visualization_msgs::Marker>("marker",1);
    bboxpub_ = private_nh.advertise<visualization_msgs::Marker>("bbox",1);
	statespub_ = nh.advertise<lilee_reinforcement::State>("states",1);

    rectanglesub_ = nh.subscribe<std_msgs::String>("/location_data", 1, &LileeFollower::rectanglecb, this);
    depthrectanglesub_ = nh.subscribe("/camera/depth/image_raw", 1, &LileeFollower::depthrectanglecb, this);

    switch_srv_ = private_nh.advertiseService("change_state", &LileeFollower::changeModeSrvCb, this);

    config_srv_ = new dynamic_reconfigure::Server<lilee_follower::FollowerConfig>(private_nh);
    dynamic_reconfigure::Server<lilee_follower::FollowerConfig>::CallbackType f =
        boost::bind(&LileeFollower::reconfigure, this, _1, _2);
    config_srv_->setCallback(f);
  }

  void reconfigure(lilee_follower::FollowerConfig &config, uint32_t level)
  {
    min_y_ = config.min_y;
    max_y_ = config.max_y;
    min_x_ = config.min_x;
    max_x_ = config.max_x;
    max_z_ = config.max_z;
    goal_z_ = config.goal_z;
    z_scale_ = config.z_scale;
    x_scale_ = config.x_scale;
  }

  /* Convert [c1, c2, c3, c4] into a float */
  float charsToFloat(unsigned char c1, unsigned char c2, unsigned char c3, unsigned char c4, bool is_bigendian)
  {
    float f;
    
    if ( htonl(47) == 47 ) { 
      // System is big endian
      if (is_bigendian) {
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

  /*!
   * @brief Callback for rectangle coordinates.
  */
  void rectanglecb(const std_msgs::String::ConstPtr& msg){
    int temp_ul_x, temp_ul_y,  temp_lr_x, temp_lr_y;

    sscanf(msg->data.c_str(), "(%d, %d),(%d, %d)", &temp_ul_x, &temp_ul_y, &temp_lr_x, &temp_lr_y);

    ul_x = temp_ul_x;
    ul_y = temp_ul_y;
    lr_x = temp_lr_x;
    lr_y = temp_lr_y;

    flag = false;
  }

    /*!
   * @brief Callback for /camera/depth/image.
  */
void depthrectanglecb(const sensor_msgs::ImageConstPtr& msg)
 {
  if (flag == false) {
    flag = true;

	lilee_reinforcement::State state;
    float x = 0.0, y = 0.0, z = 1e6, temp_x, temp_y, temp_z;
    unsigned int n = 0;

    // for each (x, y) inside rectangle
    for (int i = ul_x; i < lr_x; i++) for (int j = ul_y; j < lr_y; j++)
    {
      int index = j*4*(msg->width) + i*4;

      temp_z = charsToFloat(msg->data[index], msg->data[index+1], msg->data[index+2], msg->data[index+3], msg->is_bigendian);

      //If there are points, calculate the command goal.
      //If there are no points, simply publish a stop goal.
      if (!std::isnan(temp_z) && temp_z > 0.0) {
        
        temp_x = (i-320)*temp_z/focal_length_;
        temp_y = (j-240)*temp_z/focal_length_;

        //Test to ensure the point is within the aceptable box.
        if (-temp_y > min_y_ && -temp_y < max_y_ && temp_x < max_x_ && temp_x > min_x_ && temp_z < max_z_)
        {
          //Add the point to the totals
          x += temp_x;
          y += temp_y;

          z = std::min(z, temp_z);

          n++;
        }
      }
    }


    //If there are points, find the centroid and calculate the command goal.
    //If there are no points, simply publish a stop goal.
    if (n>100)
    {
      x /= n;
      y /= n;

      ROS_INFO("Centroid at %f %f %f with %d points", x, y, z, n);

      if(z > max_z_)
      {
        ROS_ERROR("No valid points detected, stopping the robot");
        if (enabled_)
        {
          cmdpub_.publish(geometry_msgs::TwistPtr(new geometry_msgs::Twist()));
        }
        return;
      }

      publishMarker(x, y, z);

      if (enabled_)
      {
        geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());
        ROS_INFO("In depthrectanglecb: cmd = (linear.x = %f,  angular.z = %f)",  (z - goal_z_) * z_scale_,  -x * x_scale_);
        
        cmd->linear.x = (z - goal_z_) * z_scale_;
        cmd->angular.z = -x * x_scale_;
        
        cmdpub_.publish(cmd);
      }
    }
    else
    {
      ROS_DEBUG("No points detected, stopping the robot");
      publishMarker(x, y, z);

      if (enabled_)
      {
        cmdpub_.publish(geometry_msgs::TwistPtr(new geometry_msgs::Twist()));
      }
    }

    publishBbox();
  }
} 

  bool changeModeSrvCb(turtlebot_msgs::SetFollowState::Request& request,
                       turtlebot_msgs::SetFollowState::Response& response)
  {
    if ((enabled_ == true) && (request.state == request.STOPPED))
    {
      ROS_INFO("Change mode service request: following stopped");
      cmdpub_.publish(geometry_msgs::TwistPtr(new geometry_msgs::Twist()));
      enabled_ = false;
    }
    else if ((enabled_ == false) && (request.state == request.FOLLOW))
    {
      ROS_INFO("Change mode service request: following (re)started");
      enabled_ = true;
    }

    response.result = response.OK;
    return true;
  }

  void publishMarker(double x,double y,double z)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/camera_rgb_optical_frame";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    //only if using a MESH_RESOURCE marker type:
    markerpub_.publish( marker );
  }

  void publishBbox()
  {
    double x = (min_x_ + max_x_)/2;
    double y = (min_y_ + max_y_)/2;
    double z = (0 + max_z_)/2;

    double scale_x = (max_x_ - x)*2;
    double scale_y = (max_y_ - y)*2;
    double scale_z = (max_z_ - z)*2;

    visualization_msgs::Marker marker;
    marker.header.frame_id = "/camera_rgb_optical_frame";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = 1;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = -y;
    marker.pose.position.z = z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = scale_x;
    marker.scale.y = scale_y;
    marker.scale.z = scale_z;
    marker.color.a = 0.5;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    //only if using a MESH_RESOURCE marker type:
    bboxpub_.publish( marker );
  }

  ros::Subscriber sub_;
  ros::Subscriber centersub_;
  ros::Subscriber depthsub_;
  ros::Subscriber rectanglesub_;
  ros::Subscriber depthrectanglesub_;

  ros::Publisher cmdpub_;
  ros::Publisher markerpub_;
  ros::Publisher bboxpub_;
  ros::Publisher statespub_;
};

PLUGINLIB_DECLARE_CLASS(lilee_follower, LileeFollower, lilee_follower::LileeFollower, nodelet::Nodelet);

}
