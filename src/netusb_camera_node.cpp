/*
 * netusb_camera_node.cpp
 * Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>
 */

#include <ros/ros.h>
#include <netusb_camera_driver/NETUSBCamera.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "netusb_camera_node");

  NETUSBCamera cam = NETUSBCamera();

  ros::spin();

  return 0;
}
