/*
 * NETUSBCamera.h
 * Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>
 */

#ifndef NETUSBCAMERA_H__
#define NETUSBCAMERA_H__

#include <NETUSBCAM_API.h>

#include <string>
#include <ros/ros.h>
#include <netusb_camera_driver/exceptions.h>

class NETUSBCamera
{

public:
  NETUSBCamera();
  virtual ~NETUSBCamera();

  void connect();
  void disconnect();
  void start();
  void stop();
  bool isStopped();
  std::string getName();
  bool getImage(uint8_t *buffer, unsigned int &bufsize);
  int RGBImageCallback(void *buffer, unsigned int buffersize);
private:
  int camera_index_;
  std::string camera_name_;
  std::string camera_serial_;
};

#endif // NETUSBCAMERA_H__
