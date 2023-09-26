#include "robot_gui/robot_gui.h"
#include "ros/node_handle.h"
#include <ros/ros.h>
#include <thread>

int main(int argc, char **argv) {
  ros::init(argc, argv, "robot_gui_node");
  ros::NodeHandle node_handle{};

  RobotGui robot_gui{&node_handle};
  robot_gui.run();

  return 0;
}