#include <ros/ros.h>
#include "gait_switcher/gait_switcher.h"

int main(int argc, char **argv)
{

  ros::init(argc, argv, "gait_switcher_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  GaitSwitcher GaitSwitcher_(nh, nh_private);
  ROS_INFO("Hello world, the gait switcher is starting.");
  int spinner_thread;
  nh.param<int>("/gait_switcher/gait_switcher_settings/spinner_thread", spinner_thread, 1);
  ros::AsyncSpinner spinner(spinner_thread); // Use n threads
  spinner.start();
  ros::waitForShutdown();

  ROS_INFO("Hello world, the gait switcher is closing.");
  return 0;
}