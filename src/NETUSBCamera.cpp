/*
 * NETUSBCamera.cpp
 * Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>
 */

#include <netusb_camera_driver/NETUSBCamera.h>
#include <string>
#include <sstream>

namespace netusb_camera_driver {

  int RGBImageCallbackDelegate(void*,unsigned int,void*);
  std::string ImageModeString(const NETUSBCamera::Mode &mode);

  NETUSBCamera::NETUSBCamera() :
    camera_index_(0),
    camera_num_(0),
    camera_name_(""),
    camera_serial_(""),
    is_stopped_(true),
    is_connected_(false),
    latest_buffer_(NULL),
    latest_buffersize_(0),
    image_width_(0),
    image_height_(0)
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

    char c_api_version[20];
    result = NETUSBCAM_GetApiVersion(c_api_version, 20);
    if (result != 0) {
      std::stringstream ss;
      ss << "failed to get API Version. retcode: " << result;
      throw CameraNotRunningException(ss.str());
    }
    ROS_INFO_STREAM("Using NETUSBCAM API " << std::string(c_api_version));

    result = NETUSBCAM_Init();
    if (result == 0) {
      throw CameraNotFoundException();
    }
    camera_num_ = result;
    ROS_INFO("found %d netusb cameras", result);

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
      ss << "failed to get serial number of camera: " << camera_index_ << ". retcode: " << result;
      throw CameraNotRunningException(ss.str());
    }

    unsigned int modeList_length = 0;
    unsigned int modeList[10];
    result = NETUSBCAM_GetModeList(camera_index_, &modeList_length, modeList);
    if (result != 0) {
      std::stringstream ss;
      ss << "failed to get mode list of camera: " << camera_index_ << ". retcode: " << result;
      throw CameraNotRunningException(ss.str());
    }
    ROS_INFO_STREAM("Available modes:");
    available_modes_.resize(modeList_length);
    for (int i = 0; i < modeList_length; ++i) {
      Mode m = (Mode)modeList[i];
      ROS_INFO_STREAM("     - " << ImageModeString(m));
      available_modes_[i] = m;
    }

    camera_name_ = std::string(c_cam_name);
    camera_serial_ = std::string(c_cam_serial);
    ROS_INFO_STREAM("camera found: " << camera_name_ << " (serial: " << camera_serial_ << ")");

    result = NETUSBCAM_SetCallback(camera_index_, CALLBACK_RGB, &RGBImageCallbackDelegate, this);
    if (result != 0) {
      std::stringstream ss;
      ss << "failed to set callback of camera: " << camera_index_ << ". retcode: " << result;
      throw CameraNotRunningException(ss.str());
    }

    // exposure
    PARAM_PROPERTY_f e_prop;
    result = NETUSBCAM_GetExposureRange(camera_index_, &e_prop);
    if (result != 0) {
      std::stringstream ss;
      ss << "failed to get exposure range of camera: " << camera_index_ << ". retcode: " << result;
      throw CameraNotRunningException(ss.str());
    }
    exposure_min_ = e_prop.nMin;
    exposure_default_ = e_prop.nDef;
    exposure_max_ = e_prop.nMax;
    ROS_INFO_STREAM("Available exposure range: " << exposure_min_ <<
                    " - " << exposure_max_ << " (default: " << exposure_default_ << ")");

    is_connected_ = true;
  }

  void NETUSBCamera::start()
  {
    int result = 0;

    result = NETUSBCAM_Start(camera_index_);
    if (result < 0) {
      std::stringstream ss;
      ss << "failed to start capture of camera: " << camera_index_ << ". retcode: " << result;
      throw CameraNotRunningException(ss.str());
    }
    is_stopped_ = false;
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
    result = NETUSBCAM_SetCallback(camera_index_, CALLBACK_RGB, NULL, NULL);
    if (result != 0) {
      std::stringstream ss;
      ss << "failed to unset callback of camera: " << camera_index_ << ". retcode: " << result;
      throw CameraNotRunningException(ss.str());
    }
    is_stopped_ = true;
    latest_buffer_ = NULL;
    latest_buffersize_ = 0;
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
    is_connected_ = false;
  }

  bool NETUSBCamera::getImage(std::vector<uint8_t> &buffer)
  {
    boost::mutex::scoped_lock lock(mutex_);
    if (latest_buffersize_ > 0 && latest_buffer_ != NULL) {
      buffer.resize(latest_buffersize_);
      memcpy(&buffer[0], (const void*)latest_buffer_, latest_buffersize_);
      latest_buffersize_ = 0;
      return true;
    } else {
      return false;
    }
  }

  bool NETUSBCamera::setMode(const Mode &mode)
  {
    int result = 0;

    std::vector<Mode>::iterator it = std::find(available_modes_.begin(), available_modes_.end(), mode);
    if (it == available_modes_.end()) {
      std::stringstream ss;
      ss << "this mode is unavailable for camera: " << camera_index_ << ". retcode: " << result;
      throw CameraNotRunningException(ss.str());
    }

    result = NETUSBCAM_SetMode(camera_index_, mode);
    if (result < 0) {
      std::stringstream ss;
      ss << "failed to set mode of camera: " << camera_index_ << ". retcode: " << result;
      throw CameraNotRunningException(ss.str());
    }

    ROS_INFO_STREAM("Set mode: " << ImageModeString(mode));

    result = NETUSBCAM_GetSize(camera_index_, &image_width_, &image_height_);
    if (result < 0) {
      std::stringstream ss;
      ss << "failed to get size of camera: " << camera_index_ << ". retcode: " << result;
      throw CameraNotRunningException(ss.str());
    }

    if (!isStopped()) {
      stop();
      start();
    }
    return true;
  }

  float NETUSBCamera::getExposure() const
  {
    int result = 0;
    float value;
    result = NETUSBCAM_GetExposure(camera_index_, &value);
    if (result < 0) {
      std::stringstream ss;
      ss << "failed to get exposure of camera: " << camera_index_ << ". retcode: " << result;
      throw CameraNotRunningException(ss.str());
    }
    return value;
  }

  bool NETUSBCamera::setExposure(const float &value)
  {
    int result = 0;

    result = NETUSBCAM_SetExposure(camera_index_, value);
    if (result < 0) {
      std::stringstream ss;
      ss << "failed to set exposure of camera: " << camera_index_ << ". retcode: " << result;
      throw CameraNotRunningException(ss.str());
    }
  }

  int NETUSBCamera::RGBImageCallback(void* buffer, unsigned int bufferSize)
  {
    boost::mutex::scoped_lock lock(mutex_);
    latest_buffer_ = (uint8_t*)buffer;
    latest_buffersize_ = bufferSize;
  }

  int RGBImageCallbackDelegate(void* buffer, unsigned int bufferSize, void* ctx)
  {
    NETUSBCamera* cam = (NETUSBCamera*)ctx;
    cam->RGBImageCallback(buffer, bufferSize);
    return 0;
  }

  std::string ImageModeString(const NETUSBCamera::Mode &mode)
  {
    static const char *modeNames[10] = {
      "320x240",
      "640x480",
      "752x480",
      "800x600",
      "1024x768",
      "1280x1024",
      "1600x1200",
      "2048x1536",
      "2592x1944",
      "3840x2748"
    };
    return std::string(modeNames[mode]);
  }

};
