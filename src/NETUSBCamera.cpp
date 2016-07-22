/*
 * NETUSBCamera.cpp
 * Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>
 */

#include <netusb_camera_driver/NETUSBCamera.h>
#include <string>
#include <sstream>

namespace netusb_camera_driver {

  int ImageCallbackDelegate(void*,unsigned int,void*);
  std::string ImageModeString(const NETUSBCamera::Mode &mode);
  std::string ErrorString(const NETUSBCamera::Result &result);
  std::string ParameterTypeString(const int &type);

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
    checkResult(result, "GetApiVersion");
    ROS_INFO_STREAM("Using NETUSBCAM API " << std::string(c_api_version));

    result = NETUSBCAM_Init();
    if (result == 0) {
      throw CameraNotFoundException();
    }
    camera_num_ = result;
    ROS_INFO("found %d netusb cameras", result);

    result = NETUSBCAM_Open(camera_index_);
    checkResult(result, "Open");

    char c_cam_name[256];
    result = NETUSBCAM_GetName(camera_index_, c_cam_name, 256);
    checkResult(result, "GetName");

    char c_cam_serial[256];
    result = NETUSBCAM_GetSerialNum(camera_index_, c_cam_serial, 256);
    checkResult(result, "GetSerialNum");

    unsigned int modeList_length = 0;
    unsigned int modeList[10];
    result = NETUSBCAM_GetModeList(camera_index_, &modeList_length, modeList);
    checkResult(result, "GetModeList");
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

    is_connected_ = true;
  }

  void NETUSBCamera::start()
  {
    int result = 0;
    
    result = NETUSBCAM_SetCallback(camera_index_, CALLBACK_RAW, &ImageCallbackDelegate, this);
    checkResult(result, "SetCallback");

    result = NETUSBCAM_Start(camera_index_);
    checkResult(result, "Start");
    is_stopped_ = false;
  }

  void NETUSBCamera::stop()
  {
    int result = 0;

    result = NETUSBCAM_Stop(camera_index_);
    checkResult(result, "Stop");

    result = NETUSBCAM_SetCallback(camera_index_, CALLBACK_RAW, NULL, NULL);
    checkResult(result, "SetCallback to NULL");

    is_stopped_ = true;
    latest_buffer_ = NULL;
    latest_buffersize_ = 0;
  }

  void NETUSBCamera::disconnect()
  {
    int result = 0;

    result = NETUSBCAM_Close(camera_index_);
    checkResult(result, "Close");

    result = NETUSBCAM_Destroy(camera_index_);
    checkResult(result, "Destroy");

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

  NETUSBCamera::Mode NETUSBCamera::getMode() const
  {
    int result = 0;
    unsigned int mode = -1;
    result = NETUSBCAM_GetMode(camera_index_, &mode);
    checkResult(result, "getMode");
    return (Mode)mode;
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

    bool need_restart = !isStopped();
    if (need_restart) stop();

    result = NETUSBCAM_SetMode(camera_index_, mode);
    checkResult(result, "SetMode");

    ROS_INFO_STREAM("Set mode: " << ImageModeString(mode));

    result = NETUSBCAM_GetSize(camera_index_, &image_width_, &image_height_);
    checkResult(result, "GetSize");

    if (need_restart) start();

    return true;
  }

  bool NETUSBCamera::setParameterAuto(const ParameterRangeType &type, const bool &enable)
  {
    int result = 0;
    int supported = 0;
    result = NETUSBCAM_GetParamAuto(camera_index_, (int)type, &supported);
    std::string type_str = ParameterTypeString(type);
    std::stringstream ss;
    ss << "getParamAuto type: " << type_str;
    checkResult(result, ss.str());
    if (supported == 0) return false;

    result = NETUSBCAM_SetParamAuto(camera_index_, (int)type, enable ? 1 : 0);
    if (result < 0) {
      std::stringstream ss;
      ss << "failed to set parameter type " << type_str << " auto of camera: " << camera_index_ << ". retcode: " << result;
      throw CameraNotRunningException(ss.str());
    }
    return true;
  }

  bool NETUSBCamera::setParameter(const ParameterToggleType &type, const bool &value)
  {
    int result = 0;
    unsigned long v = value == true ? 0l : 1l;
    result = NETUSBCAM_SetCamParameter(camera_index_, (int)type, v);
    std::stringstream ss;
    std::string type_str = ParameterTypeString(type);
    ss << "setCamParameter type: " << type_str;
    checkResult(result, ss.str());
    ROS_INFO_STREAM("setCamParameter type: " << type_str << ", bool: " << v);
    return true;
  }

  bool NETUSBCamera::setParameter(const ParameterRangeType &type, const unsigned long &value)
  {
    int result = 0;
    unsigned long v = value;
    PARAM_PROPERTY prop;
    std::string type_str = ParameterTypeString(type);
    result = NETUSBCAM_GetCamParameterRange(camera_index_, (int)type, &prop);
    std::stringstream ss;
    ss << "getCamParameter type: " << type_str;
    checkResult(result, ss.str());
    if (!prop.bEnabled) {
      ROS_ERROR_STREAM("parameter type " << type_str << " disabled");
      return false;
    } else if (value < prop.nMin) {
      v = prop.nMin;
    } else if (prop.nMax < value) {
      v = prop.nMax;
    }
    result = NETUSBCAM_SetCamParameter(camera_index_, (int)type, v);
    ss.clear();
    ss << "setCamParameter type: " << type_str << ", value: " << v;
    checkResult(result, ss.str());
    ROS_INFO_STREAM("setCamParameter type: " << type_str << ", value: " << v);
    return true;
  }

  bool NETUSBCamera::resetParameter(const ParameterToggleType &type)
  {
    int result = 0;
    result = NETUSBCAM_SetParamAutoDef(camera_index_, (int)type);
    std::stringstream ss;
    ss << "resetParameter of type " << ParameterTypeString(type);
    checkResult(result, ss.str());
    return true;
  }

  bool NETUSBCamera::resetParameter(const ParameterRangeType &type)
  {
    int result = 0;
    result = NETUSBCAM_SetParamAutoDef(camera_index_, (int)type);
    std::stringstream ss;
    ss << "resetParameter of type " << ParameterTypeString(type);
    checkResult(result, ss.str());
    return true;
  }

  bool NETUSBCamera::getParameter(const ParameterToggleType &type) const
  {
    int result = 0;
    unsigned long value = -1;
    result = NETUSBCAM_GetCamParameter(camera_index_, (int)type, &value);
    std::stringstream ss;
    ss << "getParameter type: " << ParameterTypeString(type);
    checkResult(result, ss.str());
    return value == 0l;
  }

  unsigned long NETUSBCamera::getParameter(const ParameterRangeType &type) const
  {
    int result = 0;
    unsigned long value = 0;
    result = NETUSBCAM_GetCamParameter(camera_index_, (int)type, &value);
    std::stringstream ss;
    ss << "getParameter type: " << ParameterTypeString(type);
    checkResult(result, ss.str());
    return value;
  }

  bool NETUSBCamera::getParameterRange(const ParameterRangeType &type,
                                       int &min, int &max, int &def) const
  {
    int result = 0;
    PARAM_PROPERTY prop;
    result = NETUSBCAM_GetCamParameterRange(camera_index_, (int)type, &prop);
    min = (int)prop.nMin;
    max = (int)prop.nMax;
    def = (int)prop.nDef;
    std::stringstream ss;
    std::string type_str = ParameterTypeString(type);
    ss << "getParameterRange type: " << type_str;
    checkResult(result, ss.str());
    ROS_INFO_STREAM("type: " << type_str << ", enable: " << prop.bEnabled << ", auto: " << prop.bAuto << ", min: " << min << ", max: " << max << ", default: " << def);
  }

  float NETUSBCamera::getExposure() const
  {
    int result = 0;
    float value;
    result = NETUSBCAM_GetExposure(camera_index_, &value);
    checkResult(result, "getExposure");
    return value;
  }

  bool NETUSBCamera::getExposureRange(double &min, double &max, double &def) const
  {
    int result = 0;
    PARAM_PROPERTY_f prop;
    result = NETUSBCAM_GetExposureRange(camera_index_, &prop);
    checkResult(result, "GetExposureRange");
    min = (double)prop.nMin;
    max = (double)prop.nMax;
    def = (double)prop.nDef;
    ROS_INFO_STREAM("exposure range min: " << prop.nMin << ", max: " << prop.nMax << ", default: " << prop.nDef);
  }

  bool NETUSBCamera::setExposure(const float &value)
  {
    int result = 0;
    result = NETUSBCAM_SetExposure(camera_index_, value);
    checkResult(result, "setExposure");
    return true;
  }

  int NETUSBCamera::ImageCallback(void* buffer, unsigned int bufferSize)
  {
    boost::mutex::scoped_lock lock(mutex_);
    latest_buffer_ = (uint8_t*)buffer;
    latest_buffersize_ = bufferSize;
  }

  int ImageCallbackDelegate(void* buffer, unsigned int bufferSize, void* ctx)
  {
    NETUSBCamera* cam = (NETUSBCamera*)ctx;
    cam->ImageCallback(buffer, bufferSize);
    return 0;
  }


  // for debuuging

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

  void NETUSBCamera::checkResult(const int &result, const std::string &message) const
  {
    Result res = (Result)result;
    if (result != SUCCESS) {
      std::stringstream ss;
      ss << "[" << ErrorString(res) << "](camera: " << camera_index_ << ") failed: " << message;
      throw CameraNotRunningException(ss.str());
    }
  }

  std::string ErrorString(const NETUSBCamera::Result &result)
  {
    switch (result) {
    case NETUSBCamera::SUCCESS: return "SUCCESS";
    case NETUSBCamera::ERROR: return "ERROR";
    case NETUSBCamera::IF_NOT_OPEN: return "IF_NOT_OPEN";
    case NETUSBCamera::WRONG_PARAM: return "WRONG_PARAM";
    case NETUSBCamera::OUT_OF_MEMORY: return "OUT_OF_MEMORY";
    case NETUSBCamera::ALREADY_DONE: return "ALREADY_DONE";
    case NETUSBCamera::WRONG_CLOCK_VAL: return "WRONG_CLOCK_VAL";
    case NETUSBCamera::COM_LIB_INIT: return "COM_LIB_INIT";
    case NETUSBCamera::NOT_IF_STARTED: return "NOT_IF_STARTED";
    default: return "UNKNOWN";
    }
  };

  std::string ParameterTypeString(const int &type)
  {
    switch (type) {
    case NETUSBCamera::BRIGHTNESS: return "BRIGHTNESS";
    case NETUSBCamera::CONTRAST: return "CONTRAST";
    case NETUSBCamera::GAMMA: return "GAMMA";
    case NETUSBCamera::WHITE_BALANCE: return "WHITE_BALANCE";
    case NETUSBCamera::EXPOSURE_TIME: return "EXPOSURE_TIME";
    case NETUSBCamera::EXPOSURE_TARGET: return "EXPOSURE_TARGET";
    case NETUSBCamera::RED: return "RED";
    case NETUSBCamera::GREEN: return "GREEN";
    case NETUSBCamera::BLUE: return "BLUE";
    case NETUSBCamera::COLOR: return "COLOR";
    case NETUSBCamera::BLACKLEVEL: return "BLACKLEVEL";
    case NETUSBCamera::GAIN: return "GAIN";
    case NETUSBCamera::PLL: return "PLL";
    case NETUSBCamera::STROBE_LENGTH: return "STROBE_LENGTH";
    case NETUSBCamera::STROBE_DELAY: return "STROBE_DELAY";
    case NETUSBCamera::TRIGGER_DELAY: return "TRIGGER_DELAY";
    case NETUSBCamera::MEASURE_FIELD_AE: return "MEASURE_FIELD_AE";
    case NETUSBCamera::SHUTTER: return "SHUTTER";
    case NETUSBCamera::ROI_ID: return "ROI_ID";
    case NETUSBCamera::ROI_CYCLE: return "ROI_CYCLE";
    case NETUSBCamera::SENSOR_TIMING: return "SENSOR_TIMING";
    case NETUSBCamera::PIXEL_DEPTH: return "PIXEL_DEPTH";
    default: return "UNKNOWN";
    }
  }
};
