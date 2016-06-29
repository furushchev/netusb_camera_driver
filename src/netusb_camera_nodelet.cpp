/*
 * netusb_camera_nodelet.cpp
 * Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>
 */

#include <netusb_camera_driver/netusb_camera_nodelet.h>

namespace netusb_camera_driver
{

  NETUSBCameraNodelet::NETUSBCameraNodelet() {}

  NETUSBCameraNodelet::~NETUSBCameraNodelet()
  {
    if (pub_thread_)
    {
      NODELET_INFO("interrupting polling thread");
      pub_thread_->interrupt();
      pub_thread_->join();
    }

    try
    {
      NODELET_INFO("stopping netusb camera");
      cam_.stop();
      NODELET_INFO("disconnecting netusb camera");
      cam_.disconnect();
    }
    catch (std::runtime_error &err)
    {
      NODELET_ERROR("%s", err.what());
    }
  }

  void NETUSBCameraNodelet::configCallback(Config &config, const uint32_t level)
  {
    boost::mutex::scoped_lock slock(cfg_mutex_);
    NODELET_INFO("configCallback called");
    if (cfg_.video_mode != config.video_mode) {
      NODELET_INFO("will change mode to %d", config.video_mode);
      cam_.setMode((NETUSBCamera::Mode)config.video_mode);
    }
    if (cfg_.exposure != config.exposure)
      cam_.setExposure((float)config.exposure);
    if (cfg_.brightness != config.brightness)
      cam_.setParameter(NETUSBCamera::BRIGHTNESS, config.brightness);
    if (cfg_.contrast != config.contrast)
      cam_.setParameter(NETUSBCamera::CONTRAST, config.contrast);
    if (cfg_.gamma != config.gamma)
      cam_.setParameter(NETUSBCamera::GAMMA, config.gamma);
    if (cfg_.white_balance != config.white_balance)
      cam_.setParameter(NETUSBCamera::WHITE_BALANCE, config.white_balance);
    if (cfg_.gain != config.gain)
      cam_.setParameter(NETUSBCamera::GAIN, config.gain);
    if (cfg_.red_gain != config.red_gain)
      cam_.setParameter(NETUSBCamera::RED, config.red_gain);
    if (cfg_.green_gain != config.green_gain)
      cam_.setParameter(NETUSBCamera::GREEN, config.green_gain);
    if (cfg_.blue_gain != config.blue_gain)
      cam_.setParameter(NETUSBCamera::BLUE, config.blue_gain);
    if (cfg_.shutter != config.shutter)
      cam_.setParameter(NETUSBCamera::SHUTTER, config.shutter);
    cfg_ = config;
  }

  void NETUSBCameraNodelet::updateConfig()
  {
    Config min, max, def;
    def.video_mode = cam_.getMode();
    cam_.getExposureRange(min.exposure, max.exposure, def.exposure);
    cam_.getParameterRange(NETUSBCamera::BRIGHTNESS,
                           min.brightness, max.brightness, def.brightness);
    cam_.getParameterRange(NETUSBCamera::CONTRAST,
                           min.contrast, max.contrast, def.contrast);
    cam_.getParameterRange(NETUSBCamera::GAMMA,
                           min.gamma, max.gamma, def.gamma);
    cam_.getParameterRange(NETUSBCamera::WHITE_BALANCE,
                           min.white_balance, max.white_balance, def.white_balance);
    cam_.getParameterRange(NETUSBCamera::GAIN,
                           min.gain, max.gain, def.gain);
    cam_.getParameterRange(NETUSBCamera::RED,
                           min.red_gain, max.red_gain, def.red_gain);
    cam_.getParameterRange(NETUSBCamera::GREEN,
                           min.green_gain, max.green_gain, def.green_gain);
    cam_.getParameterRange(NETUSBCamera::BLUE,
                           min.blue_gain, max.blue_gain, def.blue_gain);
    cam_.getParameterRange(NETUSBCamera::SHUTTER,
                           min.shutter, max.shutter, def.shutter);

    srv_->setConfigDefault(def);
    srv_->setConfigMin(min);
    srv_->setConfigMax(max);

    configCallback(def, 0);
  }

  void NETUSBCameraNodelet::connectCallback()
  {
    NODELET_INFO("called connectCb");
    boost::mutex::scoped_lock slock(conn_mutex_);
    if (pub_.getNumSubscribers() == 0)
    {
      try
      {
        NODELET_INFO("Stopping netusb camera");
        cam_.stop();
        pub_thread_->interrupt();
      }
      catch (std::runtime_error &err)
      {
        NODELET_ERROR("%s", err.what());
        ros::Duration(1.0).sleep();
      }
    } else if (cam_.isStopped()) {
      while (!cam_.isConnected() && ros::ok())
      {
        try
        {
          NODELET_INFO("Connecting to netusb camera");
          cam_.connect();
          this->updateConfig();
        }
        catch (std::runtime_error &err)
        {
          NODELET_ERROR("%s", err.what());
          ros::Duration(1.0).sleep();
        }
      }
      while (cam_.isStopped() && ros::ok())
      {
        try
        {
          NODELET_INFO("Starting netusb camera");
          image_width_ = cam_.getWidth();
          image_height_ = cam_.getHeight();
          cam_.start();
        }
        catch (std::runtime_error &err)
        {
          NODELET_ERROR("%s", err.what());
          ros::Duration(1.0).sleep();
        }
      }
      pub_thread_.reset(new boost::thread(
                          boost::bind(&NETUSBCameraNodelet::imagePoll, this)));
    } else {
      NODELET_INFO("current subscriber: %d", pub_.getNumSubscribers());
    }
  }

  void NETUSBCameraNodelet::imagePoll()
  {
    NODELET_INFO("image polling thread started");
    uint32_t img_seq = 0;
    ros::Time prev_time = ros::Time::now();
    uint32_t fps_count = 0;
    while(!boost::this_thread::interruption_requested())
    {
      if(pub_.getNumSubscribers() == 0)
        continue;
      try
      {
        sensor_msgs::ImagePtr imgmsg(new sensor_msgs::Image);
        bool newImage = cam_.getImage(imgmsg->data);
        if (!newImage) continue;

        imgmsg->header.seq = img_seq++;
        imgmsg->header.frame_id = frame_id_;
        imgmsg->header.stamp = ros::Time::now(); // FIXME: use device time
        imgmsg->encoding = image_encoding_;
        imgmsg->height = image_height_;
        imgmsg->width = image_width_;
        imgmsg->step = imgmsg->data.size() / image_height_;
        imgmsg->is_bigendian = 0;

        // camera info
        cam_info_.reset(new sensor_msgs::CameraInfo(cim_->getCameraInfo()));
        cam_info_->header.seq = imgmsg->header.seq;
        cam_info_->header.stamp = imgmsg->header.stamp;
        cam_info_->header.frame_id = imgmsg->header.frame_id;
        // FIXME: ROI

        pub_.publish(imgmsg, cam_info_);

        // fps
        ++fps_count;
        ros::Time now = ros::Time::now();
        if ((now - prev_time).toSec() > 1.0) {
          ROS_INFO_STREAM("publishing " << cam_.getName() << " camera image (" << fps_count << " fps.)");
          prev_time = now;
          fps_count = 0;
        }
      }
      catch (CameraTimeoutException &err)
      {
        NODELET_WARN("%s", err.what());
      }
      catch (std::runtime_error &err)
      {
        NODELET_ERROR("%s", err.what());
      }
    }
    NODELET_INFO("image polling thread ended");
  }

  void NETUSBCameraNodelet::onInit()
  {
    // prevent calling connectCallback before onInit
    boost::mutex::scoped_lock slock(conn_mutex_);

    nh_ = getMTNodeHandle();
    pnh_ = getMTPrivateNodeHandle();

    pnh_.param<int>("device_number", cam_index_, 0);
    pnh_.param<double>("conenction_timeout", conn_timeout_, 0.0);
    pnh_.param<std::string>("frame_id", frame_id_, "camera");

    image_encoding_ = sensor_msgs::image_encodings::BGR8;

    // setup camera info manager
    std::string cam_info_url;
    pnh_.param<std::string>("camera_info_url", cam_info_url, "");
    std::string cam_name = cam_.getName();
    cim_.reset(new camera_info_manager::CameraInfoManager(nh_, cam_name, cam_info_url));

    // setup dynamic reconfigure server
    srv_ = boost::make_shared<dynamic_reconfigure::Server<Config> > (pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind(&NETUSBCameraNodelet::configCallback, this, _1, _2);
    srv_->setCallback(f);

    // setup publishers
    it_.reset(new image_transport::ImageTransport(nh_));
    image_transport::SubscriberStatusCallback it_conn_cb =
      boost::bind(&NETUSBCameraNodelet::connectCallback, this);
    pub_ = it_->advertiseCamera("image_raw", 5, it_conn_cb, it_conn_cb);
  }


} // namespace netusb_camera_driver

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(netusb_camera_driver::NETUSBCameraNodelet, nodelet::Nodelet);
