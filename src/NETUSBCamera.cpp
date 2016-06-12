/*
 * NETUSBCamera.cpp
 * Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>
 */

#include <netusb_camera_driver/NETUSBCamera.h>
#include <string>
#include <sstream>

int RGBImageCallbackDelegate(void*,unsigned int,void*);

NETUSBCamera::NETUSBCamera() :
  camera_index_(0), camera_name_(""), camera_serial_("")
{}

NETUSBCamera::~NETUSBCamera()
{}
/*
bool NETUSBCamera::setNewConfiguration(netusb_camera_driver::NETUSBCameraConfig &config, const uint32_t &level)
{
}
*/
void NETUSBCamera::connect()
{
  int result = 0;

  result = NETUSBCAM_Init();
  if (result == 0) {
    throw CameraNotFoundException();
  }
  ROS_DEBUG("found %d netusb cameras", result);

  result = NETUSBCAM_Open(camera_index_);
  if (result != 0) {
    std::stringstream ss;
    ss << "failed to open camera: " << camera_index_ << ". retcode: " << result;
    throw CameraNotRunningException(ss.str());
  }

  char c_cam_name[256];
  result = NETUSBCAM_GetName(camera_index_, c_cam_name, 256);
  if (result != 0) {
    std::stringstream ss;
    ss << "failed to get name of camera: " << camera_index_ << ". retcode: " << result;
    throw CameraNotRunningException(ss.str());
  }

  char c_cam_serial[256];
  result = NETUSBCAM_GetSerialNum(camera_index_, c_cam_serial, 256);
  if (result != 0) {
    std::stringstream ss;
    ss << "failed to get serial number  of camera: " << camera_index_ << ". retcode: " << result;
    throw CameraNotRunningException(ss.str());
  }

  camera_name_ = std::string(c_cam_name);
  camera_serial_ = std::string(c_cam_serial);
  ROS_INFO_STREAM("camera found: " << camera_name_);
// ROS_INFO("camera found: %s (%s)", camera_name_, camera_serial_);

  result = NETUSBCAM_SetCallback(camera_index_, CALLBACK_RGB, &RGBImageCallbackDelegate, this);
  if (result != 0) {
    std::stringstream ss;
    ss << "failed to set callback of camera: " << camera_index_ << ". retcode: " << result;
    throw CameraNotRunningException(ss.str());
  }
}

void NETUSBCamera::start()
{
  int result = 0;

  result = NETUSBCAM_Start(camera_index_);
  if (result != 0) {
    std::stringstream ss;
    ss << "failed to start capture of camera: " << camera_index_ << ". retcode: " << result;
    throw CameraNotRunningException(ss.str());
  }
}

void NETUSBCamera::stop()
{
  int result = 0;

  result = NETUSBCAM_Stop(camera_index_);
  if (result != 0) {
    std::stringstream ss;
    ss << "failed to stop capture of camera: " << camera_index_ << ". retcode: " << result;
    throw CameraNotRunningException(ss.str());
  }
}

void NETUSBCamera::disconnect()
{
  int result = 0;

  result = NETUSBCAM_Close(camera_index_);
  if (result != 0) {
    std::stringstream ss;
    ss << "failed to close camera: " << camera_index_ << ". retcode: " << result;
    throw CameraNotRunningException(ss.str());
  }

  result = NETUSBCAM_Destroy(camera_index_);
  if (result != 0) {
    std::stringstream ss;
    ss << "failed to destroy camera: " << camera_index_ << ". retcode: " << result;
    throw CameraNotRunningException(ss.str());
  }
}

int NETUSBCamera::RGBImageCallback(void* buffer, unsigned int bufferSize)
{
  ROS_INFO_STREAM("callback with buffersize: " << bufferSize);
}

int RGBImageCallbackDelegate(void* buffer, unsigned int bufferSize, void* ctx)
{
  NETUSBCamera* cam = (NETUSBCamera*)ctx;
  cam->RGBImageCallback(buffer, bufferSize);
  return 0;
}
