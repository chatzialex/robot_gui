#define CVUI_IMPLEMENTATION
#include "robot_gui/cvui.h"
#include "robotinfo_msgs/RobotInfo10Fields.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <std_srvs/Trigger.h>
#include <string>

class RobotGui {
public:
  explicit RobotGui(::ros::NodeHandle *nh);
  void run();

private:
  static constexpr float kLinearVelocityStep{0.1};
  static constexpr float kAngularVelocityStep{0.1};
  static const std::string kWindowName;

  ::ros::Subscriber odometry_subscriber_{};
  ::ros::Subscriber robot_info_subscriber_{};
  ::ros::Publisher twist_publisher_{};
  ::ros::ServiceClient distance_client_{};

  ::nav_msgs::Odometry odometry_{};
  ::robotinfo_msgs::RobotInfo10Fields robot_info_{};
  ::geometry_msgs::Twist twist_{};
  ::std_srvs::Trigger distance_request_{};
  std::string distance_travelled_{};

  void odometryCallback(const ::nav_msgs::Odometry::ConstPtr &msg);
  void
  robotInfoCallback(const ::robotinfo_msgs::RobotInfo10Fields::ConstPtr &msg);
};