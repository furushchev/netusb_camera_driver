/*
 * NETUSBCamera.h
 * Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>
 */

#ifndef NETUSBCAMERA_H__
#define NETUSBCAMERA_H__

#include <NETUSBCAM_API.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/fill_image.h>
#include <netusb_camera_driver/exceptions.h>

//#include <netusb_camera_driver/NETUSBCameraConfig.h>

class NETUSBCamera
{

public:
  NETUSBCamera();
  virtual ~NETUSBCamera();

//  bool setNewConfiguration(netusb_camera_driver::NETUSBCameraConfig &config, const uint32_t &level);
  void connect();
  void disconnect();
  void start();
  void stop();
  int RGBImageCallback(void *buffer, unsigned int buffersize);
private:
  int camera_index_;
  std::string camera_name_;
  std::string camera_serial_;
};

#endif // NETUSBCAMERA_H__
