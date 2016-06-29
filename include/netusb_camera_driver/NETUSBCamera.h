/*
 * NETUSBCamera.h
 * Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>
 */

#ifndef NETUSBCAMERA_H__
#define NETUSBCAMERA_H__

#include <netusb_camera_driver/NETUSBCAM_API.h>

#include <boost/thread.hpp>
#include <string>
#include <ros/ros.h> // just for logging
#include <netusb_camera_driver/exceptions.h>

namespace netusb_camera_driver {

class NETUSBCamera
{
public:

  enum Mode {
    QVGA = 0,
    VGA = 1,
    WVGA = 2,
    SVGA = 3,
    XGA = 4,
    SXGA = 5,
    UXGA = 6,
    QXGA = 7,
    QSXGA = 8,
    WQUXGA = 9
  };

  NETUSBCamera();
  virtual ~NETUSBCamera();

  void connect();
  void disconnect();
  void start();
  void stop();
  int RGBImageCallback(void *buffer, unsigned int buffersize);
  bool getImage(std::vector<uint8_t> &buffer);
  bool setMode(const Mode &mode);
  bool setExposure(const float &value);
  float getExposure() const;
  int getWidth() const {
    return image_width_;
  }
  int getHeight() const {
    return image_height_;
  };
  inline bool isStopped() const {
    return is_stopped_;
  };
  inline bool isConnected() const {
    return is_connected_;
  }
  inline std::string getName() const {
    return camera_name_;
  };
private:
  bool is_stopped_;
  bool is_connected_;
  int camera_index_;
  int camera_num_;
  std::string camera_name_;
  std::string camera_serial_;
  std::vector<Mode> available_modes_;
  uint8_t* latest_buffer_;
  unsigned int latest_buffersize_;
  int image_width_, image_height_;
  float exposure_min_, exposure_max_, exposure_default_;
  boost::mutex mutex_;
};
};

#endif // NETUSBCAMERA_H__
