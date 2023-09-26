#include "robot_gui/robot_gui.h"
#include "robotinfo_msgs/RobotInfo10Fields.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/node_handle.h>
#include <std_srvs/Trigger.h>
#include <string>

const std::string RobotGui::kWindowName{"robot_gui"};

RobotGui::RobotGui(::ros::NodeHandle *nh)
    : odometry_subscriber_{nh->subscribe<::nav_msgs::Odometry>(
          "/odom", 2, &RobotGui::odometryCallback, this)},
      robot_info_subscriber_{nh->subscribe<::robotinfo_msgs::RobotInfo10Fields>(
          "/robot_info", 2, &RobotGui::robotInfoCallback, this)},
      twist_publisher_{nh->advertise<::geometry_msgs::Twist>("cmd_vel", 10)},
      distance_client_{nh->serviceClient<std_srvs::Trigger>("/get_distance")} {
  ROS_DEBUG("RobotGui initialized.");
}

void RobotGui::odometryCallback(const ::nav_msgs::Odometry::ConstPtr &msg) {
  odometry_ = *msg;
  ROS_DEBUG("Position x,y,z: [%0.2f, %0.2f, %0.2f]", msg->pose.pose.position.x,
            msg->pose.pose.position.y, msg->pose.pose.position.z);
}

void RobotGui::robotInfoCallback(
    const ::robotinfo_msgs::RobotInfo10Fields::ConstPtr &msg) {
  robot_info_ = *msg;
  ROS_DEBUG("robot_info.data_field_01: %s", msg->data_field_01.c_str());
}

void RobotGui::run() {
  cv::Mat frame{cv::Mat(650, 300, CV_8UC3)};

  cv::namedWindow(kWindowName);
  cvui::init(kWindowName);

  while (ros::ok()) {
    frame = cv::Scalar(49, 52, 49);

    cvui::window(frame, 30, 20, 250, 185, "Info");
    cvui::printf(frame, 35, 50, 0.4, 0xffffff,
                 robot_info_.data_field_01.c_str());
    cvui::printf(frame, 35, 70, 0.4, 0xffffff,
                 robot_info_.data_field_02.c_str());
    cvui::printf(frame, 35, 90, 0.4, 0xffffff,
                 robot_info_.data_field_03.c_str());
    cvui::printf(frame, 35, 110, 0.4, 0xffffff,
                 robot_info_.data_field_04.c_str());
    cvui::printf(frame, 35, 130, 0.4, 0xffffff,
                 robot_info_.data_field_05.c_str());
    cvui::printf(frame, 35, 150, 0.4, 0xffffff,
                 robot_info_.data_field_06.c_str());
    cvui::printf(frame, 35, 170, 0.4, 0xffffff,
                 robot_info_.data_field_07.c_str());
    cvui::printf(frame, 35, 190, 0.4, 0xffffff,
                 robot_info_.data_field_08.c_str());

    if (cvui::button(frame, 100, 220, " Forward ")) {
      twist_.linear.x += kLinearVelocityStep;
    }
    if (cvui::button(frame, 100, 250, "   Stop  ")) {
      twist_.linear.x = 0.0;
      twist_.angular.z = 0.0;
    }
    if (cvui::button(frame, 30, 250, " Left ")) {
      twist_.angular.z += kAngularVelocityStep;
    }
    if (cvui::button(frame, 195, 250, " Right ")) {
      twist_.angular.z -= kAngularVelocityStep;
    }
    if (cvui::button(frame, 100, 280, "Backward")) {
      twist_.linear.x -= kLinearVelocityStep;
    }

    cvui::window(frame, 30, 320, 120, 40, "Linear velocity:");
    cvui::printf(frame, 60, 345, 0.4, 0xff0000, "%.02f m/sec", twist_.linear.x);
    cvui::window(frame, 160, 320, 115, 40, "Angular velocity:");
    cvui::printf(frame, 180, 345, 0.4, 0xff0000, "%.02f rad/sec",
                 twist_.angular.z);

    cvui::printf(frame, 30, 380, 0.35, 0xffffff,
                 "Estimated robot position based off odometry");
    cvui::window(frame, 30, 395, 75, 75, "x");
    cvui::printf(frame, 50, 435, 0.9, 0xffffff, "%.1f",
                 odometry_.pose.pose.position.x);
    cvui::window(frame, 115, 395, 75, 75, "y");
    cvui::printf(frame, 135, 435, 0.9, 0xffffff, "%.1f",
                 odometry_.pose.pose.position.y);
    cvui::window(frame, 200, 395, 75, 75, "z");
    cvui::printf(frame, 220, 435, 0.9, 0xffffff, "%.1f",
                 odometry_.pose.pose.position.z);

    cvui::printf(frame, 30, 490, 0.4, 0xffffff, "Distance travelled");
    if (cvui::button(frame, 30, 510, 75, 75, "Call")) {
      if (distance_client_.call(distance_request_)) {
        ROS_DEBUG("Response message: %s (success)",
                  distance_request_.response.message.c_str());
        distance_travelled_ = distance_request_.response.message;
      } else {
        ROS_DEBUG("Response message: %s (failure)",
                  distance_request_.response.message.c_str());
        distance_travelled_ = "???";
      }
    }
    cvui::window(frame, 110, 505, 165, 80, "Distance in meters");
    cvui::printf(frame, 200, 545, 0.9, 0xffffff, distance_travelled_.c_str());

    twist_publisher_.publish(twist_);
    cvui::update();
    cv::imshow(kWindowName, frame);

    if (cv::waitKey(20) == 27) {
      break;
    }

    ::ros::spinOnce();
  }
}
